#include "stdafx.h"
#include "IPCClient.h"

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

// Waits out one overlapped transfer under TransactionTimeoutMs and returns the
// bytes transferred. A timeout is thrown exactly like an I/O error so
// SendDriverRequest's ConnectionGeneration bookkeeping and the
// replay-after-reconnect path in SendBlocking both behave unchanged.
DWORD IPCClient::AwaitOverlapped(OVERLAPPED &ov, const char *what)
{
	DWORD transferred = 0;
	if (GetLastError() == ERROR_IO_PENDING)
	{
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
		throw std::runtime_error(std::string("Error ") + what + ". Error: " +
			LastErrorString(GetLastError()));
	}
	return transferred;
}

void IPCClient::Connect()
{
	Close();
	LPTSTR pipeName = TEXT(QUESTCALIBRATOR_PIPE_NAME);

	WaitNamedPipe(pipeName, 1000);
	pipe = CreateFile(pipeName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING,
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
		throw std::runtime_error(
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
		// A vrserver restart invalidates the old named-pipe instance. All
		// current mutations are complete-state setters, so replaying one after
		// a fresh same-version handshake is safe and lets the overlay recover
		// without a restart of its own.
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

	SetLastError(ERROR_SUCCESS);
	WriteFile(pipe, &request, sizeof request, nullptr, &ov);
	DWORD bytesWritten = AwaitOverlapped(ov, "writing IPC request");
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

	SetLastError(ERROR_SUCCESS);
	ReadFile(pipe, &response, sizeof response, nullptr, &ov);
	DWORD bytesRead = AwaitOverlapped(ov, "reading IPC response");

	if (bytesRead != sizeof response)
	{
		throw std::runtime_error("Invalid IPC response with size " + std::to_string(bytesRead));
	}

	return response;
}
