#pragma once
#include "Protocol.h"
#include <cstddef>
#include <cstdint>
namespace questcal { namespace ipc {
inline bool RequestFrameComplete(uint32_t error, size_t bytes)
{
	return error == 0 && bytes == sizeof(protocol::Request);
}
inline bool ResponseFrameComplete(size_t bytes)
{
	return bytes == sizeof(protocol::Response);
}
} }
