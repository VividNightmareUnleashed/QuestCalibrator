#pragma once

#include <cstddef>
#include <cstdint>

// The first exchange on every pipe connection, in a layout that never changes,
// so an app and a driver from different releases can always tell each other
// their protocol versions. The protocol's own messages are exact-size frames,
// and a frame from another release reads as a broken one rather than as an old
// version. Never change these structs; a new need gets a new message.
namespace protocol
{
	// "QCVP" in memory order.
	const uint32_t VersionProbeMagic = 0x50564351;

	struct VersionProbeRequest
	{
		uint32_t magic = VersionProbeMagic;
		uint32_t protocolVersion = 0;   // the app's
	};

	struct VersionProbeResponse
	{
		uint32_t magic = VersionProbeMagic;
		uint32_t protocolVersion = 0;   // the driver's
		// The frame sizes the driver reads and writes: equal versions built from
		// different headers could still disagree.
		uint32_t requestSize = 0;
		uint32_t responseSize = 0;
		// The driver's release (QUESTCAL_VERSION_STRING), NUL-terminated.
		char release[32] = {};
	};

	static_assert(sizeof(VersionProbeRequest) == 8, "the version probe never changes");
	static_assert(offsetof(VersionProbeRequest, magic) == 0 &&
		offsetof(VersionProbeRequest, protocolVersion) == 4, "the version probe never changes");
	static_assert(sizeof(VersionProbeResponse) == 48, "the version probe never changes");
	static_assert(offsetof(VersionProbeResponse, magic) == 0 &&
		offsetof(VersionProbeResponse, protocolVersion) == 4 &&
		offsetof(VersionProbeResponse, requestSize) == 8 &&
		offsetof(VersionProbeResponse, responseSize) == 12 &&
		offsetof(VersionProbeResponse, release) == 16, "the version probe never changes");
}
