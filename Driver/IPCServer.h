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
	// The two mutations this transport is allowed to perform. Naming them here
	// instead of holding a pointer to the concrete driver class makes the
	// dependency one-directional - no header cycle, and the overlapped-pipe
	// machinery (short-message rejection, per-connection state, listener retry
	// backoff, teardown drain) can be constructed against a recording sink.
	struct RequestSink
	{
		std::function<bool(const protocol::SetDeviceTransform &)> setDeviceTransform;
		std::function<bool(const protocol::SetAlignmentField &)> setAlignmentField;
	};

	~IPCServer();

	// Returns only after the first listener has been created successfully. A
	// false result means no server thread owns the IPC resources. `sink` is
	// established before the first accept and, being a member, is torn down only
	// with this object - which Stop() has already joined the server thread out
	// of, after the alertable drain.
	bool Run(RequestSink newSink);
	void Stop();

#ifdef QUESTCAL_IPC_SERVER_TEST_SEAM
	// Request dispatch is pure given a sink and a connection state, but it sat
	// behind the named-pipe transport, so nothing could reach it. The seam
	// exposes only that - the transport still needs a real pipe and stays
	// uncovered. Same pattern as the pose-channel seam.
	void SetSinkForTest(RequestSink newSink) { sink = std::move(newSink); }
	void DispatchForTest(const protocol::Request &request, protocol::Response &response,
		questcal::ipc::ConnectionState &connection)
	{
		HandleRequest(request, response, connection);
	}
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
		// Last time this connection completed an IO. The protocol is
		// request/response with no long-lived subscription, so a connection that
		// has neither read nor written for the deadline below is not waiting on
		// anything we owe it.
		ULONGLONG lastActivityMs;
	};

	// The overlay needs exactly one connection. The cap is well above that so a
	// reconnect overlapping a not-yet-reaped previous connection is never
	// refused - wedging out the real client would be a worse failure than the
	// drain this bounds. Past the cap the accept is closed immediately, before
	// any instance is allocated.
	static const size_t MaxConcurrentConnections = 8;
	// A connection that holds the pipe open without ever writing costs a kernel
	// pipe instance, a handle and an instance inside vrserver, and the
	// per-message validation the pipe's trust boundary relies on is never
	// reached because it runs only after a full message arrives.
	static const ULONGLONG ConnectionIdleDeadlineMs = 30000;

	void CloseIdleConnections();

	// Both completion callbacks reinterpret_cast the LPOVERLAPPED the API hands
	// back straight to PipeInstance*. Inserting any member above `overlap`, or
	// giving the struct a base class or a virtual, would silently corrupt every
	// callback with no diagnostic; these are the checks. Standard layout is what
	// makes an object pointer-interconvertible with its first member, and the
	// offset is what pins which member that is.
	static_assert(std::is_standard_layout<PipeInstance>::value,
		"PipeInstance must stay standard-layout: the IO completion callbacks cast "
		"LPOVERLAPPED back to PipeInstance*");
	static_assert(offsetof(PipeInstance, overlap) == 0,
		"PipeInstance::overlap must stay the first member: the IO completion callbacks "
		"cast LPOVERLAPPED back to PipeInstance*");

	PipeInstance *CreatePipeInstance(HANDLE pipe);
	void ClosePipeInstance(PipeInstance *pipeInst);
	// Named for what it does: on the stop path it CLOSES the instance the
	// overlapped belongs to and returns null. A null result means "already
	// destroyed", never "not found".
	static PipeInstance *ActivePipeInstanceOrClose(LPOVERLAPPED overlap);

	static void RunThread(IPCServer *_this);
	static bool CreateAndConnectInstance(LPOVERLAPPED overlap, HANDLE &pipe, bool &pending);
	static void CloseListenerInstance(LPOVERLAPPED overlap, HANDLE &pipe, bool &pending);
	static void WINAPI CompletedReadCallback(DWORD err, DWORD bytesRead, LPOVERLAPPED overlap);
	static void WINAPI CompletedWriteCallback(DWORD err, DWORD bytesWritten, LPOVERLAPPED overlap);

	std::thread mainThread;

	// Crossed between the caller and RunThread; the event is created by Run()
	// (not RunThread) so Stop() can never signal an indeterminate handle.
	std::atomic<bool> stop{ false };

	std::set<PipeInstance *> pipes;
	HANDLE connectEvent = nullptr;
	HANDLE stopEvent = nullptr;
	OVERLAPPED connectOverlap{};
	HANDLE listenerPipe = INVALID_HANDLE_VALUE;
	bool listenerConnectPending = false;

	RequestSink sink;
};
