#pragma once

#include "../common/Protocol.h"
#include "IPCProtocolGate.h"

#include <atomic>
#include <thread>
#include <set>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

class ServerTrackedDeviceProvider;

class IPCServer
{
public:
	IPCServer(ServerTrackedDeviceProvider *driver) : driver(driver) { }
	~IPCServer();

	// Returns only after the first listener has been created successfully. A
	// false result means no server thread owns the IPC resources.
	bool Run();
	void Stop();

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
	};

	PipeInstance *CreatePipeInstance(HANDLE pipe);
	void ClosePipeInstance(PipeInstance *pipeInst);
	static PipeInstance *ActivePipeInstance(LPOVERLAPPED overlap);

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

	ServerTrackedDeviceProvider *driver;
};
