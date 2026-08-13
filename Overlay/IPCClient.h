#pragma once

#include "../common/Protocol.h"

class IPCClient
{
public:
	~IPCClient();

	void Connect();
	protocol::Response SendBlocking(const protocol::Request &request);
	uint64_t ConnectionGeneration() const { return connectionGeneration; }

	// Every transaction runs on the UI thread, and one driver scan is up to 65
	// of them, so an unbounded wait here is a frozen window the user cannot
	// close. A stalled vrserver IPC thread - or any local process that owns the
	// pipe name first - must surface as the same failure a disconnect already
	// produces, not as a hang.
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
