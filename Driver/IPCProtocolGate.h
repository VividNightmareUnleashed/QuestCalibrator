#pragma once

#include "../common/Protocol.h"

// Pure per-connection protocol gate. Keeping this separate from Win32 pipe
// callbacks makes the exact-version/handshake trust boundary executable in
// SolverTests instead of relying only on integration behavior.
namespace questcal
{
namespace ipc
{

struct ConnectionState
{
	bool handshakeComplete = false;
};

// Returns true only when the caller may dispatch a mutating request. Control
// and rejected requests have their complete response populated here.
inline bool PrepareRequest(const protocol::Request &request,
	ConnectionState &state, protocol::Response &response)
{
	response = protocol::Response(protocol::ResponseInvalid);
	if (request.type == protocol::RequestHandshake)
	{
		state.handshakeComplete = request.protocol.version == protocol::Version;
		response.type = protocol::ResponseHandshake;
		response.protocol.version = protocol::Version;
		return false;
	}

	bool mutation = request.type == protocol::RequestSetDeviceTransform ||
		request.type == protocol::RequestSetAlignmentField;
	return mutation && state.handshakeComplete &&
		request.protocol.version == protocol::Version;
}

} // namespace ipc
} // namespace questcal
