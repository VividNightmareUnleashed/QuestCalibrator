#include "../Driver/IPCServer.h"
#include "../Driver/Logging.h"
#include "../Overlay/IPCClient.h"
#include "../common/Protocol.h"
#include "../common/Version.h"
#include "../common/VersionProbe.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

// IPCServer and IPCClient over real named pipes, on their own pipe names and a
// clock the test advances. The accept path reaps idle connections before its
// connection cap, and "a stale connection can never be what refuses the real
// client". Every connection starts with the version probe, whose fixed layout
// lets an app and a driver from different releases tell each other apart.
namespace
{
using Check = void (*)(const char *, bool, const char *);

// IPCServer::MaxConcurrentConnections.
constexpr size_t Cap = 8;

std::string TestPipeName(const char *purpose)
{
	return "\\\\.\\pipe\\QuestCalibratorTransportTest-" +
		std::to_string(GetCurrentProcessId()) + "-" + purpose;
}

IPCServer::RequestSink AcceptingSink()
{
	IPCServer::RequestSink sink;
	sink.setDeviceTransform = [](const protocol::SetDeviceTransform &) { return protocol::RejectReason::None; };
	sink.setRuntimeState = [](const protocol::SetRuntimeState &) { return protocol::RejectReason::None; };
	sink.poseHookMask = []() { return 0u; };
	return sink;
}

HANDLE Connect(const std::string &name)
{
	for (int attempt = 0; attempt < 40; ++attempt)
	{
		WaitNamedPipeA(name.c_str(), 250);
		HANDLE pipe = CreateFileA(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
			OPEN_EXISTING, 0, nullptr);
		if (pipe != INVALID_HANDLE_VALUE)
		{
			DWORD mode = PIPE_READMODE_MESSAGE;
			SetNamedPipeHandleState(pipe, &mode, nullptr, nullptr);
			return pipe;
		}
		Sleep(10);
	}
	return INVALID_HANDLE_VALUE;
}

// One handshake round trip; false if the server closed the connection.
bool Handshake(HANDLE pipe)
{
	protocol::Request request{};
	request.type = protocol::RequestHandshake;
	DWORD written = 0;
	if (!WriteFile(pipe, &request, sizeof request, &written, nullptr) || written != sizeof request)
		return false;
	protocol::Response response{};
	DWORD read = 0;
	return ReadFile(pipe, &response, sizeof response, &read, nullptr) &&
		read == sizeof response && response.type == protocol::ResponseHandshake;
}

// The server stamps a connection's activity in the completion routine that
// runs after its response is written, and reads the clock on every pass of its
// loop before it blocks. Once the clock has gone unread for a while, every
// completion has run and the server is blocked in its wait, so advancing the
// clock cannot land between a response and its stamp.
bool WaitForServerQuiet(const std::atomic<int> &clockReads)
{
	for (int attempt = 0; attempt < 100; ++attempt)
	{
		const int before = clockReads.load();
		Sleep(50);
		if (clockReads.load() == before)
			return true;
	}
	return false;
}

void IdleConnectionsDoNotRefuseTheClient(Check check)
{
	const std::string name = TestPipeName("idle");
	std::atomic<ULONGLONG> now{ 1000 };
	std::atomic<int> clockReads{ 0 };

	IPCServer server;
	server.SetPipeNameForTest(name.c_str());
	server.SetClockForTest([&now, &clockReads]() { ++clockReads; return now.load(); });
	const bool running = server.Run(AcceptingSink());

	// Fill the cap with connections that then go quiet.
	std::vector<HANDLE> stale;
	size_t served = 0;
	for (size_t i = 0; running && i < Cap; ++i)
	{
		HANDLE pipe = Connect(name);
		if (pipe == INVALID_HANDLE_VALUE)
			break;
		stale.push_back(pipe);
		served += Handshake(pipe) ? 1 : 0;
	}

	// Past the idle deadline on the server's clock, then the real client.
	const bool quiet = WaitForServerQuiet(clockReads);
	now += 30001;
	HANDLE client = running ? Connect(name) : INVALID_HANDLE_VALUE;
	const bool clientServed = client != INVALID_HANDLE_VALUE && Handshake(client);

	// The reaped connections really are gone.
	size_t reaped = 0;
	for (HANDLE pipe : stale)
		reaped += Handshake(pipe) ? 0 : 1;

	if (client != INVALID_HANDLE_VALUE)
		CloseHandle(client);
	for (HANDLE pipe : stale)
		CloseHandle(pipe);
	server.Stop();

	char detail[160];
	snprintf(detail, sizeof detail, "running %d, %zu/%zu stale served then %zu reaped, client served %d, quiet %d",
		running, served, Cap, reaped, clientServed, quiet);
	check("ipc server: idle connections do not refuse the client",
		running && quiet && served == Cap && reaped == Cap && clientServed, detail);
}

// The probe gets its fixed-layout answer, and the same connection then carries
// the protocol as usual.
void VersionProbeIsAnswered(Check check)
{
	const std::string name = TestPipeName("probe");
	IPCServer server;
	server.SetPipeNameForTest(name.c_str());
	const bool running = server.Run(AcceptingSink());
	HANDLE pipe = running ? Connect(name) : INVALID_HANDLE_VALUE;

	protocol::VersionProbeRequest probe;
	probe.protocolVersion = protocol::Version;
	protocol::VersionProbeResponse answer;
	answer.magic = 0;
	DWORD written = 0;
	DWORD read = 0;
	const bool exchanged = pipe != INVALID_HANDLE_VALUE &&
		WriteFile(pipe, &probe, sizeof probe, &written, nullptr) && written == sizeof probe &&
		ReadFile(pipe, &answer, sizeof answer, &read, nullptr) && read == sizeof answer;
	answer.release[sizeof answer.release - 1] = '\0';
	const bool fields = exchanged && answer.magic == protocol::VersionProbeMagic &&
		answer.protocolVersion == protocol::Version &&
		answer.requestSize == sizeof(protocol::Request) &&
		answer.responseSize == sizeof(protocol::Response) &&
		std::strcmp(answer.release, QUESTCAL_VERSION_STRING) == 0;
	const bool handshake = exchanged && Handshake(pipe);
	if (pipe != INVALID_HANDLE_VALUE)
		CloseHandle(pipe);
	server.Stop();

	char detail[160];
	snprintf(detail, sizeof detail, "running %d, %lu bytes: protocol %u, frames %u/%u, release %s; then handshake %d",
		running, read, answer.protocolVersion, answer.requestSize, answer.responseSize,
		exchanged ? answer.release : "-", handshake);
	check("ipc server: answers the version probe in its fixed layout",
		running && fields && handshake, detail);
}

void ClientProbesBeforeTheHandshake(Check check)
{
	const std::string name = TestPipeName("client");
	IPCServer server;
	server.SetPipeNameForTest(name.c_str());
	const bool running = server.Run(AcceptingSink());

	std::string error;
	protocol::Response response;
	uint64_t generation = 0;
	{
		IPCClient client;
		client.SetPipeNameForTest(name.c_str());
		try
		{
			client.Connect();
			response = client.SendBlocking(protocol::Request(protocol::RequestHandshake));
		}
		catch (const std::exception &e)
		{
			error = e.what();
		}
		generation = client.ConnectionGeneration();
	}
	server.Stop();

	char detail[160];
	snprintf(detail, sizeof detail, "running %d, generation %llu, response %u%s%.100s",
		running, static_cast<unsigned long long>(generation), static_cast<unsigned>(response.type),
		error.empty() ? "" : ", error: ", error.c_str());
	check("ipc client: probes the driver's version before the handshake",
		running && error.empty() && generation == 1 &&
			response.type == protocol::ResponseHandshake, detail);
}

// A stand-in driver of another release: one pipe instance that reads one
// message, then either closes, as a release from before the probe does, or
// answers with `answer`.
class StandInDriver
{
public:
	StandInDriver(const std::string &name, const protocol::VersionProbeResponse *answer)
		: answers(answer != nullptr)
	{
		if (answer)
			reply = *answer;
		pipe = CreateNamedPipeA(name.c_str(), PIPE_ACCESS_DUPLEX,
			PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT, 1,
			sizeof(protocol::Response), sizeof(protocol::Request), 1000, nullptr);
		if (pipe != INVALID_HANDLE_VALUE)
			thread = std::thread([this] { Serve(); });
	}

	~StandInDriver()
	{
		// A client that never came leaves the thread blocked in a pipe call.
		while (thread.joinable() && !done.load())
		{
			CancelSynchronousIo(thread.native_handle());
			Sleep(1);
		}
		if (thread.joinable())
			thread.join();
		if (pipe != INVALID_HANDLE_VALUE)
			CloseHandle(pipe);
	}

	StandInDriver(const StandInDriver &) = delete;
	StandInDriver &operator=(const StandInDriver &) = delete;

	bool Ready() const { return pipe != INVALID_HANDLE_VALUE; }

private:
	void Serve()
	{
		if (ConnectNamedPipe(pipe, nullptr) || GetLastError() == ERROR_PIPE_CONNECTED)
		{
			protocol::Request request;
			DWORD read = 0;
			if (ReadFile(pipe, &request, sizeof request, &read, nullptr) && answers)
			{
				DWORD written = 0;
				WriteFile(pipe, &reply, sizeof reply, &written, nullptr);
				// Disconnecting discards what the client has not read yet.
				FlushFileBuffers(pipe);
			}
			DisconnectNamedPipe(pipe);
		}
		done.store(true);
	}

	const bool answers;
	protocol::VersionProbeResponse reply;
	HANDLE pipe = INVALID_HANDLE_VALUE;
	std::atomic<bool> done{ false };
	std::thread thread;
};

// 1 for a version mismatch, 2 for any other failure, 0 for a connection.
int ConnectTo(const std::string &name, std::string &message)
{
	IPCClient client;
	client.SetPipeNameForTest(name.c_str());
	try
	{
		client.Connect();
	}
	catch (const DriverVersionMismatch &e)
	{
		message = e.what();
		return 1;
	}
	catch (const std::exception &e)
	{
		message = e.what();
		return 2;
	}
	return 0;
}

// Whether the driver predates the probe (it closes the pipe on it) or comes
// from a later release (it answers with its own version), the app reports a
// version mismatch rather than a frame of the wrong size.
void ClientReportsADriverFromAnotherRelease(Check check)
{
	std::string olderMessage;
	std::string newerMessage;
	int older = -1;
	int newer = -1;
	{
		const std::string name = TestPipeName("older");
		StandInDriver driver(name, nullptr);
		if (driver.Ready())
			older = ConnectTo(name, olderMessage);
	}
	{
		protocol::VersionProbeResponse answer;
		answer.protocolVersion = protocol::Version + 1;
		answer.requestSize = sizeof(protocol::Request) + 8;
		answer.responseSize = sizeof(protocol::Response) + 8;
		std::memcpy(answer.release, "9.9.9", sizeof "9.9.9");
		const std::string name = TestPipeName("newer");
		StandInDriver driver(name, &answer);
		if (driver.Ready())
			newer = ConnectTo(name, newerMessage);
	}
	const bool namesTheRelease = newerMessage.find("9.9.9") != std::string::npos;

	char detail[256];
	snprintf(detail, sizeof detail, "older %d (%.80s); newer %d (%.80s)",
		older, olderMessage.c_str(), newer, newerMessage.c_str());
	check("ipc client: a driver from another release reads as a version mismatch",
		older == 1 && newer == 1 && namesTheRelease, detail);
}
} // namespace

void RunIPCServerTransportScenarios(Check check)
{
	if (!LogFile)
		LogFile = stderr;   // the LOG macro writes unconditionally

	IdleConnectionsDoNotRefuseTheClient(check);
	VersionProbeIsAnswered(check);
	ClientProbesBeforeTheHandshake(check);
	ClientReportsADriverFromAnotherRelease(check);
}
