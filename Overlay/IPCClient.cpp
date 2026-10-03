#include "stdafx.h"
#include "IPCClient.h"
#include "../common/IPCFramePolicy.h"
#include "../common/Version.h"

#include <string>

static std::string LastErrorString(DWORD lastError)
{
	LPSTR buffer = nullptr;
	size_t size = FormatMessageA(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL
	);

	std::string message;
	if (buffer && size != 0)
		message.assign(buffer, size);
	else
		message = "Windows error " + std::to_string(lastError);
	if (buffer)
		LocalFree(buffer);
	return message;
}

IPCClient::~IPCClient()
{
	Close();
}

void IPCClient::Close()
{
	if (pipe != INVALID_HANDLE_VALUE)
	{
		CloseHandle(pipe);
		pipe = INVALID_HANDLE_VALUE;
	}
	if (transactionEvent)
	{
		CloseHandle(transactionEvent);
		transactionEvent = nullptr;
	}
}

bool IPCClient::Complete(BOOL started, OVERLAPPED &ov, const char *what, DWORD &transferred,
	DWORD &error)
{
	transferred = 0;
	if (!started)
	{
		error = GetLastError();
		if (error != ERROR_IO_PENDING)
			return false;
		DWORD wait = WaitForSingleObject(ov.hEvent, TransactionTimeoutMs);
		if (wait != WAIT_OBJECT_0)
		{
			// The kernel may still write into `ov` and the caller's buffer, so
			// cancel and let the cancellation settle before either goes away.
			CancelIoEx(pipe, &ov);
			GetOverlappedResult(pipe, &ov, &transferred, TRUE);
			throw std::runtime_error(std::string("Timed out ") + what +
				". The driver is not responding.");
		}
	}
	if (!GetOverlappedResult(pipe, &ov, &transferred, FALSE))
	{
		error = GetLastError();
		return false;
	}
	error = ERROR_SUCCESS;
	return true;
}

// A timeout throws like an I/O error, so SendBlocking's reconnect-and-replay
// handles both.
DWORD IPCClient::AwaitOverlapped(BOOL started, OVERLAPPED &ov, const char *what)
{
	DWORD transferred = 0;
	DWORD error = ERROR_SUCCESS;
	if (!Complete(started, ov, what, transferred, error))
		throw std::runtime_error(std::string("Error ") + what + ". Error: " +
			LastErrorString(error));
	return transferred;
}

// A driver from before the probe reads it as a malformed request and closes
// the pipe, which is how that release is told apart from one that answers.
protocol::VersionProbeResponse IPCClient::ProbeConnected()
{
	protocol::VersionProbeRequest probe;
	probe.protocolVersion = protocol::Version;
	OVERLAPPED ov = {};
	ov.hEvent = transactionEvent;
	ResetEvent(transactionEvent);
	if (AwaitOverlapped(WriteFile(pipe, &probe, sizeof probe, nullptr, &ov), ov,
		"writing the version probe") != sizeof probe)
	{
		throw std::runtime_error("Error writing the version probe. Error: " +
			LastErrorString(ERROR_WRITE_FAULT));
	}

	protocol::VersionProbeResponse answer;
	ov = {};
	ov.hEvent = transactionEvent;
	ResetEvent(transactionEvent);
	DWORD bytesRead = 0;
	DWORD error = ERROR_SUCCESS;
	if (!Complete(ReadFile(pipe, &answer, sizeof answer, nullptr, &ov), ov,
		"reading the driver's version", bytesRead, error))
	{
		if (error == ERROR_BROKEN_PIPE || error == ERROR_PIPE_NOT_CONNECTED ||
			error == ERROR_NO_DATA)
		{
			throw DriverVersionMismatch("QuestCalibrator's SteamVR driver is older than this app: "
				"it closed the connection at the version probe. Reinstall QuestCalibrator, then "
				"restart SteamVR.");
		}
		if (error != ERROR_MORE_DATA)
			throw std::runtime_error("Error reading the driver's version. Error: " +
				LastErrorString(error));
		bytesRead = 0;   // longer than any answer to the probe
	}
	if (!questcal::ipc::VersionProbeAnswerComplete(bytesRead) ||
		answer.magic != protocol::VersionProbeMagic)
	{
		throw DriverVersionMismatch("The program serving QuestCalibrator's driver pipe answered "
			"the version probe in an unknown format. Reinstall QuestCalibrator, then restart "
			"SteamVR.");
	}
	answer.release[sizeof answer.release - 1] = '\0';
	return answer;
}

void IPCClient::Connect()
{
	Close();
	WaitNamedPipeA(pipeName, 1000);
	pipe = CreateFileA(pipeName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING,
		FILE_FLAG_OVERLAPPED, 0);

	if (pipe == INVALID_HANDLE_VALUE)
	{
		throw std::runtime_error("QuestCalibrator driver unavailable. Make sure SteamVR is running, and the QuestCalibrator addon is enabled in SteamVR settings.");
	}

	// Manual reset: GetOverlappedResult is what consumes completion here, and
	// one event is reused for every transfer on this connection.
	transactionEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
	if (!transactionEvent)
	{
		DWORD error = GetLastError();
		Close();
		throw std::runtime_error("Couldn't create pipe wait event. Error: " +
			LastErrorString(error));
	}

	DWORD mode = PIPE_READMODE_MESSAGE;
	if (!SetNamedPipeHandleState(pipe, &mode, 0, 0))
	{
		DWORD error = GetLastError();
		Close();
		throw std::runtime_error("Couldn't set pipe mode. Error: " + LastErrorString(error));
	}

	protocol::Response response;
	try
	{
		const protocol::VersionProbeResponse probe = ProbeConnected();
		if (probe.protocolVersion != protocol::Version ||
			probe.requestSize != sizeof(protocol::Request) ||
			probe.responseSize != sizeof(protocol::Response))
		{
			throw DriverVersionMismatch("QuestCalibrator's SteamVR driver is from release " +
				std::string(probe.release) + " (protocol " + std::to_string(probe.protocolVersion) +
				", frames of " + std::to_string(probe.requestSize) + "/" +
				std::to_string(probe.responseSize) + " bytes), this app from release "
				QUESTCAL_VERSION_STRING " (protocol " + std::to_string(protocol::Version) +
				", frames of " + std::to_string(sizeof(protocol::Request)) + "/" +
				std::to_string(sizeof(protocol::Response)) + " bytes). Reinstall QuestCalibrator, "
				"then restart SteamVR.");
		}
		response = SendBlockingConnected(protocol::Request(protocol::RequestHandshake));
	}
	catch (...)
	{
		Close();
		throw;
	}
	if (response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version)
	{
		Close();
		throw DriverVersionMismatch(
			"Incorrect driver version installed, try reinstalling QuestCalibrator. (Client: " +
			std::to_string(protocol::Version) +
			", Driver: " +
			std::to_string(response.protocol.version) +
			")"
		);
	}
	++connectionGeneration;
}

protocol::Response IPCClient::SendBlocking(const protocol::Request &request)
{
	if (pipe == INVALID_HANDLE_VALUE)
		Connect();

	try
	{
		return SendBlockingConnected(request);
	}
	catch (...)
	{
		// A vrserver restart invalidates the old pipe instance. Every mutation
		// is a complete-state setter, so replaying it after a fresh handshake
		// is safe.
		Close();
	}

	Connect();
	try
	{
		return SendBlockingConnected(request);
	}
	catch (...)
	{
		Close();
		throw;
	}
}

protocol::Response IPCClient::SendBlockingConnected(const protocol::Request &request)
{
	SendConnected(request);
	return ReceiveConnected();
}

void IPCClient::SendConnected(const protocol::Request &request)
{
	OVERLAPPED ov = {};
	ov.hEvent = transactionEvent;
	ResetEvent(transactionEvent);

	DWORD bytesWritten = AwaitOverlapped(WriteFile(pipe, &request, sizeof request, nullptr, &ov),
		ov, "writing IPC request");
	if (bytesWritten != sizeof request)
	{
		throw std::runtime_error("Error writing IPC request. Error: " +
			LastErrorString(ERROR_WRITE_FAULT));
	}
}

protocol::Response IPCClient::ReceiveConnected()
{
	protocol::Response response(protocol::ResponseInvalid);
	OVERLAPPED ov = {};
	ov.hEvent = transactionEvent;
	ResetEvent(transactionEvent);

	DWORD bytesRead = AwaitOverlapped(ReadFile(pipe, &response, sizeof response, nullptr, &ov),
		ov, "reading IPC response");

	if (!questcal::ipc::ResponseFrameComplete(bytesRead))
	{
		throw std::runtime_error("Invalid IPC response with size " + std::to_string(bytesRead));
	}

	return response;
}
