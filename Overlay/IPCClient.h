#pragma once

#include "../common/Protocol.h"

class IPCClient
{
public:
	~IPCClient();

	void Connect();
	protocol::Response SendBlocking(const protocol::Request &request);
	uint64_t ConnectionGeneration() const { return connectionGeneration; }

private:
	void Close();
	protocol::Response SendBlockingConnected(const protocol::Request &request);
	void SendConnected(const protocol::Request &request);
	protocol::Response ReceiveConnected();

	HANDLE pipe = INVALID_HANDLE_VALUE;
	uint64_t connectionGeneration = 0;
};
