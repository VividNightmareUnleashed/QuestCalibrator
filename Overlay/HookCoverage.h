#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

namespace questcal
{

// Target devices SteamVR tracks whose poses have not passed the driver's pose
// hook, so no calibration reaches them: another driver resolved its host
// interface first, or SteamVR uses a host version the driver does not hook.
// The driver reports the devices it has seen at each handshake, about once a
// second, so a device that started tracking just after one handshake would
// look missing until the next; only one still missing after BypassSyncs
// synchronizations in a row is reported.
struct HookCoverage
{
	// One bit per OpenVR id: vr::k_unMaxTrackedDeviceCount.
	static constexpr uint32_t Devices = 64;
	static constexpr uint8_t BypassSyncs = 3;
	std::array<uint8_t, Devices> missedSyncs{};

	// Returns the devices to report.
	uint64_t Note(uint64_t hookedDevices, uint64_t trackedDevices, uint64_t targetDevices)
	{
		uint64_t bypassing = 0;
		for (uint32_t id = 0; id < Devices; ++id)
		{
			const uint64_t bit = uint64_t{ 1 } << id;
			const bool missing = (targetDevices & bit) != 0 && (trackedDevices & bit) != 0 &&
				(hookedDevices & bit) == 0;
			missedSyncs[id] = missing
				? static_cast<uint8_t>((std::min)(missedSyncs[id] + 1, int{ BypassSyncs }))
				: uint8_t{ 0 };
			if (missedSyncs[id] >= BypassSyncs)
				bypassing |= bit;
		}
		return bypassing;
	}
};

} // namespace questcal
