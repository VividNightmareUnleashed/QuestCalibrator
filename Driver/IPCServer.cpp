#include "IPCServer.h"
#include "Logging.h"

#include <algorithm>
#include <exception>
#include <new>
#include <utility>

namespace
{

// ResponseInvalid is the only rejection the wire carries, so a gate refusal and
// a value rejection look the same to the overlay; the driver log is where the
// two causes are told apart.
protocol::ResponseType SetterResult(bool accepted, const char *operation)
{
	if (accepted)
		return protocol::ResponseSuccess;

	LOG("IPC %s rejected: the request cleared the protocol gate, so it was its "
		"values that failed the driver trust boundary", operation);
	return protocol::ResponseInvalid;
}

} // namespace

void IPCServer::HandleRequest(const protocol::Request &request, protocol::Response &response,
	questcal::ipc::ConnectionState &connection)
{
	if (!questcal::ipc::PrepareRequest(request, connection, response))
	{
		if (response.type == protocol::ResponseHandshake)
			response.poseHookMask = sink.poseHookMask();
		// Only a mutation refused for the connection's own state is worth a line.
		if (request.type == protocol::RequestSetDeviceTransform ||
			request.type == protocol::RequestSetRuntimeState)
			LOG("IPC mutation %d refused by the protocol gate: this connection has "
				"no same-version handshake (request version %u, driver %u)",
				request.type, request.protocol.version, protocol::Version);
		return;
	}

	// PrepareRequest passes only the two mutation types.
	if (request.type == protocol::RequestSetDeviceTransform)
		response.type = SetterResult(
			sink.setDeviceTransform(request.setDeviceTransform), "SetDeviceTransform");
	else
		response.type = SetterResult(
			sink.setRuntimeState(request.setRuntimeState), "SetRuntimeState");
}

IPCServer::~IPCServer()
{
	Stop();
}

bool IPCServer::Run(RequestSink newSink)
{
	// Only Init calls this, after Stop() has joined any previous server thread.
	stop.store(false, std::memory_order_release);
	connectOverlap = {};
	listenerPipe = INVALID_HANDLE_VALUE;
	listenerConnectPending = false;
	// Established before the first listener can accept anything.
	sink = std::move(newSink);

	// No thread is running yet, so Stop() just releases whatever was acquired.
	auto fail = [this]
	{
		Stop();
		return false;
	};

	// Created here rather than in RunThread so Stop() always has valid handles,
	// however early it runs. The stop event is separate from the connect event
	// so shutdown cannot masquerade as a connected client.
	connectEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
	stopEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
	if (!connectEvent || !stopEvent)
	{
		LOG("CreateEvent failed in Run. Error: %d", GetLastError());
		return fail();
	}
	connectOverlap.hEvent = connectEvent;

	// Establish the first listener synchronously. Init must not report success
	// for a driver instance that can never accept control requests.
	if (!CreateAndConnectInstance(pipeName, &connectOverlap, listenerPipe, listenerConnectPending))
		return fail();

	try
	{
		mainThread = std::thread(RunThread, this);
	}
	catch (const std::exception &e)
	{
		LOG("Could not start IPC server thread: %s", e.what());
		return fail();
	}
	return true;
}

void IPCServer::Stop()
{
	TRACE("IPCServer::Stop()");
	stop.store(true, std::memory_order_release);
	if (mainThread.joinable())
	{
		SetEvent(stopEvent);   // both events exist while the thread does
		mainThread.join();
	}
	CloseListenerInstance(&connectOverlap, listenerPipe, listenerConnectPending);
	if (connectEvent)
		CloseHandle(connectEvent);
	if (stopEvent)
		CloseHandle(stopEvent);
	connectEvent = nullptr;
	stopEvent = nullptr;
	TRACE("IPCServer::Stop() finished");
}

IPCServer::PipeInstance *IPCServer::CreatePipeInstance(HANDLE pipe)
{
	auto pipeInst = new (std::nothrow) PipeInstance{};
	if (!pipeInst)
	{
		LOG("Could not allocate IPC pipe state");
		return nullptr;
	}
	pipeInst->pipe = pipe;
	pipeInst->server = this;
	pipeInst->lastActivityMs = Now();
	try
	{
		pipes.insert(pipeInst);
	}
	catch (...)
	{
		LOG("Could not register IPC pipe state");
		delete pipeInst;
		return nullptr;
	}
	return pipeInst;
}

void IPCServer::ClosePipeInstance(PipeInstance *pipeInst)
{
	DisconnectNamedPipe(pipeInst->pipe);
	CloseHandle(pipeInst->pipe);
	pipes.erase(pipeInst);
	delete pipeInst;
}

ULONGLONG IPCServer::Now() const
{
#ifdef QUESTCAL_IPC_SERVER_TEST_SEAM
	if (clockForTest)
		return clockForTest();
#endif
	return GetTickCount64();
}

// A peer that connects and never writes leaves its read pending forever, so
// nothing else reclaims these. Only cancel here: the completion routine owns
// the instance and closes it once the cancelled IO completes.
void IPCServer::CloseIdleConnections()
{
	const ULONGLONG now = Now();
	for (PipeInstance *pipeInst : pipes)
	{
		if (pipeInst->closing ||
			now - pipeInst->lastActivityMs < ConnectionIdleDeadlineMs)
			continue;

		LOG("Dropping IPC connection idle for %llu ms", now - pipeInst->lastActivityMs);
		pipeInst->closing = true;
		if (!CancelIoEx(pipeInst->pipe, &pipeInst->overlap) &&
			GetLastError() != ERROR_NOT_FOUND)
		{
			LOG("CancelIoEx failed for idle IPC connection. Error: %d", GetLastError());
		}
	}
}

DWORD IPCServer::NextIdleTimeoutMs() const
{
	const ULONGLONG now = Now();
	ULONGLONG nearest = MAXDWORD;
	for (const PipeInstance *pipeInst : pipes)
	{
		if (pipeInst->closing)
			continue;
		ULONGLONG elapsed = now - pipeInst->lastActivityMs;
		if (elapsed >= ConnectionIdleDeadlineMs)
			return 0;
		nearest = std::min(nearest, ConnectionIdleDeadlineMs - elapsed);
	}
	return static_cast<DWORD>(nearest);
}

void IPCServer::RunThread(IPCServer *_this)
{
	HANDLE waitHandles[2] = { _this->stopEvent, _this->connectEvent };
	constexpr DWORD InitialListenerRetryMs = 50;
	constexpr DWORD MaximumListenerRetryMs = 1000;
	DWORD listenerRetryDelay = InitialListenerRetryMs;
	ULONGLONG nextListenerAttempt = 0;
	auto scheduleListenerRetry = [&]()
	{
		nextListenerAttempt = GetTickCount64() + listenerRetryDelay;
		listenerRetryDelay = std::min(listenerRetryDelay * 2, MaximumListenerRetryMs);
	};

	while (!_this->stop.load(std::memory_order_acquire))
	{
		if (_this->listenerPipe == INVALID_HANDLE_VALUE)
		{
			ULONGLONG now = GetTickCount64();
			if (now >= nextListenerAttempt)
			{
				if (CreateAndConnectInstance(_this->pipeName, &_this->connectOverlap,
					_this->listenerPipe, _this->listenerConnectPending))
				{
					listenerRetryDelay = InitialListenerRetryMs;
					nextListenerAttempt = 0;
				}
				else
				{
					scheduleListenerRetry();
				}
			}
		}

		DWORD handleCount = _this->listenerPipe == INVALID_HANDLE_VALUE ? 1 : 2;
		DWORD timeout = _this->NextIdleTimeoutMs();
		if (handleCount == 1)
		{
			ULONGLONG now = GetTickCount64();
			ULONGLONG remaining = nextListenerAttempt > now
				? nextListenerAttempt - now : 0;
			timeout = std::min(timeout,
				static_cast<DWORD>(std::min<ULONGLONG>(remaining, MAXDWORD)));
		}

		DWORD wait = WaitForMultipleObjectsEx(handleCount, waitHandles, FALSE, timeout, TRUE);

		if (wait == WAIT_OBJECT_0)
		{
			break;
		}
		else if (wait == WAIT_OBJECT_0 + 1)
		{
			if (_this->listenerConnectPending)
			{
				DWORD bytesConnect = 0;
				BOOL success = GetOverlappedResult(_this->listenerPipe,
					&_this->connectOverlap, &bytesConnect, FALSE);
				if (!success)
				{
					DWORD error = GetLastError();
					// ERROR_IO_INCOMPLETE means the OVERLAPPED storage is still
					// kernel-owned; preserve that fact so CloseListenerInstance
					// cancels and drains it before the next accept reuses the object.
					_this->listenerConnectPending = error == ERROR_IO_INCOMPLETE;
					if (error != ERROR_OPERATION_ABORTED ||
					    !_this->stop.load(std::memory_order_acquire))
						LOG("GetOverlappedResult failed in RunThread. Error: %d", error);
					CloseListenerInstance(&_this->connectOverlap,
						_this->listenerPipe, _this->listenerConnectPending);
					if (!_this->stop.load(std::memory_order_acquire))
						scheduleListenerRetry();
					continue;
				}
				_this->listenerConnectPending = false;
			}

			LOG("IPC client connected");

			HANDLE connectedPipe = _this->listenerPipe;
			_this->listenerPipe = INVALID_HANDLE_VALUE;
			_this->listenerConnectPending = false;

			// Reap first, so a stale connection can never be what refuses the
			// real client. A reaped instance stays in `pipes` until its cancelled
			// IO completes at the next alertable wait, so only the connections
			// not already closing count toward the cap.
			_this->CloseIdleConnections();

			PipeInstance *pipeInst = nullptr;
			const size_t open = static_cast<size_t>(std::count_if(
				_this->pipes.begin(), _this->pipes.end(),
				[](const PipeInstance *p) { return !p->closing; }));
			if (open >= MaxConcurrentConnections)
				LOG("Refusing IPC connection: %zu already open", open);
			else
				pipeInst = _this->CreatePipeInstance(connectedPipe);

			if (pipeInst)
				CompletedWriteCallback(0, sizeof(protocol::Response), (LPOVERLAPPED) pipeInst);
			else
			{
				DisconnectNamedPipe(connectedPipe);
				CloseHandle(connectedPipe);
			}
		}
		else if (wait == WAIT_TIMEOUT || wait == WAIT_IO_COMPLETION)
		{
			// Also sweep here: a peer that connects once and then goes quiet
			// produces no further connection events to reap it on. The set is
			// bounded by MaxConcurrentConnections, so this stays trivial.
			_this->CloseIdleConnections();
			continue;
		}
		else
		{
			LOG("WaitForMultipleObjectsEx failed in RunThread. Error: %d", GetLastError());
			break;
		}
	}

	_this->stop.store(true, std::memory_order_release);

	// The listener is not in `pipes`, but its OVERLAPPED storage lives on this
	// object. Cancel and drain it before closing the handle or returning.
	CloseListenerInstance(&_this->connectOverlap,
		_this->listenerPipe, _this->listenerConnectPending);

	// Every accepted pipe has exactly one APC-style read/write outstanding.
	// Cancel them first, then stay alertable until every completion callback
	// observes `stop` and closes its own instance. This preserves OVERLAPPED
	// lifetime and prevents use-after-free during fast vrserver teardown.
	for (PipeInstance *pipeInst : _this->pipes)
		CancelIoEx(pipeInst->pipe, &pipeInst->overlap);

	while (!_this->pipes.empty())
		SleepEx(INFINITE, TRUE);   // returns only after running completion routines
}

void IPCServer::CloseListenerInstance(LPOVERLAPPED overlap, HANDLE &pipe, bool &pending)
{
	if (pipe == INVALID_HANDLE_VALUE)
	{
		pending = false;
		return;
	}

	if (pending)
	{
		CancelIoEx(pipe, overlap);
		DWORD ignored = 0;
		GetOverlappedResult(pipe, overlap, &ignored, TRUE);
	}
	DisconnectNamedPipe(pipe);
	CloseHandle(pipe);
	pipe = INVALID_HANDLE_VALUE;
	pending = false;
}

bool IPCServer::CreateAndConnectInstance(const char *name, LPOVERLAPPED overlap, HANDLE &pipe,
	bool &pending)
{
	pending = false;
	ResetEvent(overlap->hEvent);
	overlap->Internal = 0;
	overlap->InternalHigh = 0;
	overlap->Offset = 0;
	overlap->OffsetHigh = 0;

	pipe = CreateNamedPipeA(
		name,
		PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
		PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
		PIPE_UNLIMITED_INSTANCES,
		sizeof(protocol::Response),
		sizeof(protocol::Request),
		1000,
		0
	);

	if (pipe == INVALID_HANDLE_VALUE)
	{
		LOG("CreateNamedPipe failed. Error: %d", GetLastError());
		return false;
	}

	if (ConnectNamedPipe(pipe, overlap))
	{
		SetEvent(overlap->hEvent);
		return true;
	}

	DWORD error = GetLastError();
	switch(error)
	{
	case ERROR_IO_PENDING:
		pending = true;
		return true;

	case ERROR_PIPE_CONNECTED:
		SetEvent(overlap->hEvent);
		return true;
	}

	LOG("ConnectNamedPipe failed. Error: %d", error);
	CloseHandle(pipe);
	pipe = INVALID_HANDLE_VALUE;
	return false;
}

IPCServer::PipeInstance *IPCServer::ActivePipeInstanceOrClose(LPOVERLAPPED overlap)
{
	PipeInstance *pipeInst = reinterpret_cast<PipeInstance *>(overlap);
	if (!pipeInst->closing &&
		!pipeInst->server->stop.load(std::memory_order_acquire))
	{
		// Both completion callbacks funnel through here, so every completed IO
		// counts as activity.
		pipeInst->lastActivityMs = pipeInst->server->Now();
		return pipeInst;
	}

	pipeInst->server->ClosePipeInstance(pipeInst);
	return nullptr;
}

void IPCServer::CompletedReadCallback(DWORD err, DWORD bytesRead, LPOVERLAPPED overlap)
{
	PipeInstance *pipeInst = ActivePipeInstanceOrClose(overlap);
	if (!pipeInst)
		return;

	// Only exactly-sized messages are dispatched: a short one would leave stale
	// bytes from the previous request in the buffer. Logged apart from I/O
	// errors, and only this connection closes.
	if (err == 0 && bytesRead != sizeof(protocol::Request))
	{
		LOG("IPC client disconnecting: malformed request rejected (bytesRead: %u, expected: %u)",
			bytesRead, static_cast<unsigned>(sizeof(protocol::Request)));
		pipeInst->server->ClosePipeInstance(pipeInst);
		return;
	}

	BOOL success = FALSE;
	if (err == 0)
	{
		pipeInst->server->HandleRequest(pipeInst->request, pipeInst->response,
			pipeInst->connection);
		success = WriteFileEx(
			pipeInst->pipe,
			&pipeInst->response,
			sizeof(protocol::Response),
			overlap,
			(LPOVERLAPPED_COMPLETION_ROUTINE) CompletedWriteCallback
		);
	}

	if (!success)
	{
		if (err == ERROR_BROKEN_PIPE)
		{
			LOG("IPC client disconnecting normally");
		}
		else
		{
			LOG("IPC client disconnecting due to error (via CompletedReadCallback), error: %d, bytesRead: %d", err, bytesRead);
		}
		pipeInst->server->ClosePipeInstance(pipeInst);
	}
}

void IPCServer::CompletedWriteCallback(DWORD err, DWORD bytesWritten, LPOVERLAPPED overlap)
{
	PipeInstance *pipeInst = ActivePipeInstanceOrClose(overlap);
	if (!pipeInst)
		return;
	BOOL success = FALSE;

	if (err == 0 && bytesWritten == sizeof(protocol::Response))
	{
		success = ReadFileEx(
			pipeInst->pipe,
			&pipeInst->request,
			sizeof(protocol::Request),
			overlap,
			(LPOVERLAPPED_COMPLETION_ROUTINE) CompletedReadCallback
		);
	}

	if (!success)
	{
		LOG("IPC client disconnecting due to error (via CompletedWriteCallback), error: %d, bytesWritten: %d", err, bytesWritten);
		pipeInst->server->ClosePipeInstance(pipeInst);
	}
}
