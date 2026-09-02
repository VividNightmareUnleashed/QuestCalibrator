#pragma once

#include "../common/Protocol.h"

class IPCClient
{
public:
	~IPCClient();

	void Connect();
	protocol::Response SendBlocking(const protocol::Request &request);
	uint64_t ConnectionGeneration() const { return connectionGeneration; }

	// A stalled vrserver IPC thread, or another process holding the pipe name,
	// must fail in bounded time so shutdown can join the driver worker.
	static const DWORD TransactionTimeoutMs = 2000;

private:
	void Close();
	protocol::Response SendBlockingConnected(const protocol::Request &request);
	void SendConnected(const protocol::Request &request);
	protocol::Response ReceiveConnected();
	// Completes one overlapped operation, or throws on error/timeout. Cancels
	// the operation before returning so the caller can close the handle without
	// leaving the kernel writing into a dead buffer.
	DWORD AwaitOverlapped(OVERLAPPED &ov, const char *what);

	HANDLE pipe = INVALID_HANDLE_VALUE;
	HANDLE transactionEvent = nullptr;
	uint64_t connectionGeneration = 0;
};
