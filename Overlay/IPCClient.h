#pragma once

#include "../common/Protocol.h"
#include "../common/VersionProbe.h"

#include <stdexcept>

// The driver is from another release than this app: reinstalling fixes it,
// reconnecting does not.
class DriverVersionMismatch : public std::runtime_error
{
public:
	using std::runtime_error::runtime_error;
};

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

#ifdef QUESTCAL_IPC_CLIENT_TEST_SEAM
	// Its own pipe, so a test never meets an installed driver.
	void SetPipeNameForTest(const char *name) { pipeName = name; }
#endif

private:
	void Close();
	protocol::VersionProbeResponse ProbeConnected();
	protocol::Response SendBlockingConnected(const protocol::Request &request);
	void SendConnected(const protocol::Request &request);
	protocol::Response ReceiveConnected();
	// Completes one overlapped operation, given what its ReadFile or WriteFile
	// returned. A timeout throws, after cancelling and draining the operation so
	// the kernel never writes into a dead buffer; an I/O error returns false
	// with its code.
	bool Complete(BOOL started, OVERLAPPED &ov, const char *what, DWORD &transferred,
		DWORD &error);
	// Complete, with an I/O error thrown as well.
	DWORD AwaitOverlapped(BOOL started, OVERLAPPED &ov, const char *what);

	HANDLE pipe = INVALID_HANDLE_VALUE;
	HANDLE transactionEvent = nullptr;
	uint64_t connectionGeneration = 0;
	const char *pipeName = QUESTCALIBRATOR_PIPE_NAME;
};
