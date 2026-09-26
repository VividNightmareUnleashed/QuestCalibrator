#pragma once

#include "../common/Protocol.h"
#include "IPCProtocolGate.h"

#include <atomic>
#include <cstddef>
#include <functional>
#include <thread>
#include <set>
#include <type_traits>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

class IPCServer
{
public:
	// What the transport may call into, so it names no driver type and tests can
	// run it against a recording sink. Every member must be set.
	struct RequestSink
	{
		std::function<bool(const protocol::SetDeviceTransform &)> setDeviceTransform;
		std::function<bool(const protocol::SetRuntimeState &)> setRuntimeState;
		std::function<uint32_t()> poseHookMask;
	};

	~IPCServer();

	// True once the first listener exists and the server thread is running;
	// false leaves no thread or handles behind. The sink lives until this object
	// is destroyed, after Stop() has joined the thread.
	bool Run(RequestSink newSink);
	void Stop();

#ifdef QUESTCAL_IPC_SERVER_TEST_SEAM
	// Request dispatch without the pipe.
	void SetSinkForTest(RequestSink newSink) { sink = std::move(newSink); }
	void DispatchForTest(const protocol::Request &request, protocol::Response &response,
		questcal::ipc::ConnectionState &connection)
	{
		HandleRequest(request, response, connection);
	}
	// Transport tests: their own pipe, so they never meet an installed driver,
	// and a clock they advance past the idle deadline instead of waiting 30 s.
	// Both are set before Run().
	void SetPipeNameForTest(const char *name) { pipeName = name; }
	void SetClockForTest(std::function<ULONGLONG()> clock) { clockForTest = std::move(clock); }
#endif

private:
	void HandleRequest(const protocol::Request &request, protocol::Response &response,
		questcal::ipc::ConnectionState &connection);

	struct PipeInstance
	{
		OVERLAPPED overlap; // Used by the API
		HANDLE pipe;
		IPCServer *server;

		protocol::Request request;
		protocol::Response response;
		questcal::ipc::ConnectionState connection;
		// Cancellation is asynchronous. Once set, no callback may reuse this
		// connection; the callback that observes the cancelled operation is the
		// sole owner allowed to close and delete it.
		bool closing = false;
		// Last time this connection completed an IO. The protocol is
		// request/response, so an idle connection is owed nothing.
		ULONGLONG lastActivityMs;
	};

	// The overlay needs one connection; the headroom lets a reconnect overlap a
	// not-yet-reaped previous one. Past the cap an accept is closed at once.
	static const size_t MaxConcurrentConnections = 8;
	// A connection that never writes holds resources inside vrserver and never
	// reaches per-message validation, so it is dropped after this long.
	static const ULONGLONG ConnectionIdleDeadlineMs = 30000;

	void CloseIdleConnections();
	DWORD NextIdleTimeoutMs() const;
	// The clock the idle deadline is measured on.
	ULONGLONG Now() const;

	// Both completion callbacks cast the LPOVERLAPPED straight back to
	// PipeInstance*, which needs `overlap` first in a standard-layout struct.
	static_assert(std::is_standard_layout<PipeInstance>::value,
		"PipeInstance must stay standard-layout: the IO completion callbacks cast "
		"LPOVERLAPPED back to PipeInstance*");
	static_assert(
#if defined(__clang__)
		__builtin_offsetof(PipeInstance, overlap) == 0,
#else
		offsetof(PipeInstance, overlap) == 0,
#endif
		"PipeInstance::overlap must stay the first member: the IO completion callbacks "
		"cast LPOVERLAPPED back to PipeInstance*");

	PipeInstance *CreatePipeInstance(HANDLE pipe);
	void ClosePipeInstance(PipeInstance *pipeInst);
	// Once the instance is closing or the server is stopping, CLOSES it and
	// returns null: null means "already destroyed", never "not found".
	static PipeInstance *ActivePipeInstanceOrClose(LPOVERLAPPED overlap);

	static void RunThread(IPCServer *_this);
	static bool CreateAndConnectInstance(const char *name, LPOVERLAPPED overlap, HANDLE &pipe,
		bool &pending);
	static void CloseListenerInstance(LPOVERLAPPED overlap, HANDLE &pipe, bool &pending);
	static void WINAPI CompletedReadCallback(DWORD err, DWORD bytesRead, LPOVERLAPPED overlap);
	static void WINAPI CompletedWriteCallback(DWORD err, DWORD bytesWritten, LPOVERLAPPED overlap);

	std::thread mainThread;

	// Crossed between the caller and RunThread.
	std::atomic<bool> stop{ false };

	std::set<PipeInstance *> pipes;
	HANDLE connectEvent = nullptr;
	HANDLE stopEvent = nullptr;
	OVERLAPPED connectOverlap{};
	HANDLE listenerPipe = INVALID_HANDLE_VALUE;
	bool listenerConnectPending = false;

	RequestSink sink;
	const char *pipeName = QUESTCALIBRATOR_PIPE_NAME;
#ifdef QUESTCAL_IPC_SERVER_TEST_SEAM
	std::function<ULONGLONG()> clockForTest;
#endif
};
