#include "../Driver/IPCServer.h"
#include "../Driver/Logging.h"
#include "../common/Protocol.h"

#include <atomic>
#include <cstdio>
#include <string>
#include <vector>

// IPCServer over a real named pipe, on its own pipe name and a clock the test
// advances. The accept path reaps idle connections before its connection cap,
// and "a stale connection can never be what refuses the real client".
namespace
{
using Check = void (*)(const char *, bool, const char *);

// IPCServer::MaxConcurrentConnections.
constexpr size_t Cap = 8;

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
	if (!LogFile)
		LogFile = stderr;   // the LOG macro writes unconditionally

	const std::string name = "\\\\.\\pipe\\QuestCalibratorTransportTest-" +
		std::to_string(GetCurrentProcessId());
	std::atomic<ULONGLONG> now{ 1000 };
	std::atomic<int> clockReads{ 0 };

	IPCServer server;
	server.SetPipeNameForTest(name.c_str());
	server.SetClockForTest([&now, &clockReads]() { ++clockReads; return now.load(); });
	IPCServer::RequestSink sink;
	sink.setDeviceTransform = [](const protocol::SetDeviceTransform &) { return true; };
	sink.setRuntimeState = [](const protocol::SetRuntimeState &) { return true; };
	sink.poseHookMask = []() { return 0u; };
	const bool running = server.Run(sink);

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
} // namespace

void RunIPCServerTransportScenarios(Check check)
{
	IdleConnectionsDoNotRefuseTheClient(check);
}
