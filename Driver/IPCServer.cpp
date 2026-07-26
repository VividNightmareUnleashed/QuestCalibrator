#include "IPCServer.h"
#include "Logging.h"
#include "ServerTrackedDeviceProvider.h"

#include <vector>

void IPCServer::HandleRequest(const protocol::Request &request, protocol::Response &response)
{
	switch (request.type)
	{
	case protocol::RequestHandshake:
		response.type = protocol::ResponseHandshake;
		response.protocol.version = protocol::Version;
		break;

	case protocol::RequestSetDeviceTransform:
		driver->SetDeviceTransform(request.setDeviceTransform);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestSetAlignmentField:
		driver->SetAlignmentField(request.setAlignmentField);
		response.type = protocol::ResponseSuccess;
		break;

	default:
		LOG("Invalid IPC request: %d", request.type);
		response.type = protocol::ResponseInvalid;
		break;
	}
}

IPCServer::~IPCServer()
{
	Stop();
}

void IPCServer::Run()
{
	if (mainThread.joinable())
		return;

	stop.store(false, std::memory_order_release);

	// Created here rather than in RunThread so Stop() always has valid handles,
	// however early it runs. The connect event is reset before every accept;
	// the stop event is a separate signal so shutdown cannot masquerade as a
	// successfully connected client.
	connectEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
	stopEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
	if (!connectEvent || !stopEvent)
	{
		LOG("CreateEvent failed in Run. Error: %d", GetLastError());
		if (connectEvent)
			CloseHandle(connectEvent);
		if (stopEvent)
			CloseHandle(stopEvent);
		connectEvent = nullptr;
		stopEvent = nullptr;
		return;
	}

	mainThread = std::thread(RunThread, this);
}

void IPCServer::Stop()
{
	TRACE("IPCServer::Stop()");
	if (!mainThread.joinable())
		return;

	stop.store(true, std::memory_order_release);
	SetEvent(stopEvent);
	mainThread.join();
	CloseHandle(connectEvent);
	CloseHandle(stopEvent);
	connectEvent = nullptr;
	stopEvent = nullptr;
	TRACE("IPCServer::Stop() finished");
}

IPCServer::PipeInstance *IPCServer::CreatePipeInstance(HANDLE pipe)
{
	auto pipeInst = new PipeInstance{};
	pipeInst->pipe = pipe;
	pipeInst->server = this;
	pipes.insert(pipeInst);
	return pipeInst;
}

void IPCServer::ClosePipeInstance(PipeInstance *pipeInst)
{
	DisconnectNamedPipe(pipeInst->pipe);
	CloseHandle(pipeInst->pipe);
	pipes.erase(pipeInst);
	delete pipeInst;
}

void IPCServer::RunThread(IPCServer *_this)
{
	OVERLAPPED connectOverlap{};
	connectOverlap.hEvent = _this->connectEvent;

	HANDLE nextPipe = INVALID_HANDLE_VALUE;
	bool connectPending = false;
	HANDLE waitHandles[2] = { _this->stopEvent, _this->connectEvent };
	if (!CreateAndConnectInstance(&connectOverlap, nextPipe, connectPending))
		goto cleanup;

	while (!_this->stop.load(std::memory_order_acquire))
	{
		DWORD wait = WaitForMultipleObjectsEx(2, waitHandles, FALSE, INFINITE, TRUE);

		if (wait == WAIT_OBJECT_0)
		{
			break;
		}
		else if (wait == WAIT_OBJECT_0 + 1)
		{
			if (connectPending)
			{
				DWORD bytesConnect = 0;
				BOOL success = GetOverlappedResult(nextPipe, &connectOverlap, &bytesConnect, FALSE);
				if (!success)
				{
					DWORD error = GetLastError();
					if (error != ERROR_OPERATION_ABORTED ||
					    !_this->stop.load(std::memory_order_acquire))
						LOG("GetOverlappedResult failed in RunThread. Error: %d", error);
					break;
				}
			}

			LOG("IPC client connected");

			HANDLE connectedPipe = nextPipe;
			nextPipe = INVALID_HANDLE_VALUE;
			auto pipeInst = _this->CreatePipeInstance(connectedPipe);
			CompletedWriteCallback(0, sizeof(protocol::Response), (LPOVERLAPPED) pipeInst);

			if (!CreateAndConnectInstance(&connectOverlap, nextPipe, connectPending))
				break;
		}
		else if (wait != WAIT_IO_COMPLETION)
		{
			LOG("WaitForMultipleObjectsEx failed in RunThread. Error: %d", GetLastError());
			break;
		}
	}

cleanup:
	_this->stop.store(true, std::memory_order_release);

	// The listener is not in `pipes`, but its OVERLAPPED storage lives on this
	// stack. Cancel and drain it before closing the handle or returning.
	if (nextPipe != INVALID_HANDLE_VALUE)
	{
		if (connectPending)
		{
			CancelIoEx(nextPipe, &connectOverlap);
			DWORD ignored = 0;
			GetOverlappedResult(nextPipe, &connectOverlap, &ignored, TRUE);
		}
		DisconnectNamedPipe(nextPipe);
		CloseHandle(nextPipe);
	}

	// Every accepted pipe has exactly one APC-style read/write outstanding.
	// Cancel them first, then stay alertable until every completion callback
	// observes `stop` and closes its own instance. This preserves OVERLAPPED
	// lifetime and prevents use-after-free during fast vrserver teardown.
	std::vector<PipeInstance *> pending(_this->pipes.begin(), _this->pipes.end());
	for (PipeInstance *pipeInst : pending)
		CancelIoEx(pipeInst->pipe, &pipeInst->overlap);

	while (!_this->pipes.empty())
	{
		DWORD wait = SleepEx(INFINITE, TRUE);
		if (wait != WAIT_IO_COMPLETION)
		{
			LOG("Alertable IPC drain failed in RunThread. Error: %d", GetLastError());
		}
	}
}

bool IPCServer::CreateAndConnectInstance(LPOVERLAPPED overlap, HANDLE &pipe, bool &pending)
{
	pending = false;
	ResetEvent(overlap->hEvent);
	overlap->Internal = 0;
	overlap->InternalHigh = 0;
	overlap->Offset = 0;
	overlap->OffsetHigh = 0;

	pipe = CreateNamedPipe(
		TEXT(QUESTCALIBRATOR_PIPE_NAME),
		PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
		PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
		PIPE_UNLIMITED_INSTANCES,
		sizeof(protocol::Request),
		sizeof(protocol::Response),
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

IPCServer::PipeInstance *IPCServer::ActivePipeInstance(LPOVERLAPPED overlap)
{
	PipeInstance *pipeInst = reinterpret_cast<PipeInstance *>(overlap);
	if (!pipeInst->server->stop.load(std::memory_order_acquire))
		return pipeInst;

	pipeInst->server->ClosePipeInstance(pipeInst);
	return nullptr;
}

void IPCServer::CompletedReadCallback(DWORD err, DWORD bytesRead, LPOVERLAPPED overlap)
{
	PipeInstance *pipeInst = ActivePipeInstance(overlap);
	if (!pipeInst)
		return;
	BOOL success = FALSE;

	// A short message would leave stale bytes from the previous request in the
	// buffer; only dispatch exactly-sized messages.
	if (err == 0 && bytesRead == sizeof(protocol::Request))
	{
		pipeInst->server->HandleRequest(pipeInst->request, pipeInst->response);
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
	PipeInstance *pipeInst = ActivePipeInstance(overlap);
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
