#pragma once

#include "ProfileRecord.h"
#include "../common/Protocol.h"

#include <array>
#include <string>

namespace questcal
{
// Stable identity keys for local runtime state, not a security boundary.
inline uint64_t FrameIdentityKey(const std::string &identity)
{
	if (identity.empty()) return 0;
	uint64_t hash = 14695981039346656037ull;
	for (unsigned char c : identity)
	{
		hash ^= c;
		hash *= 1099511628211ull;
	}
	return hash == 0 ? 1 : hash;
}

inline uint64_t FrameProfileKey(const std::string &reference, const std::string &target,
	const std::string &hmd, double calibrationTime)
{
	if (reference.empty() || target.empty() || hmd.empty() ||
		!std::isfinite(calibrationTime) || calibrationTime <= 0.0)
		return 0;
	return FrameIdentityKey(reference + '\n' + target + '\n' + hmd + '\n' +
		std::to_string(calibrationTime));
}

using FrameSerialKeys = std::array<uint64_t, vr::k_unMaxTrackedDeviceCount>;
using RecoveredFrames = std::array<protocol::FrameCorrection, vr::k_unMaxTrackedDeviceCount>;

// No persisted matrices cross a SteamVR restart. Read the checkpoint from the
// current driver, require its handshake's session, and match each physical
// serial rather than assuming that an OpenVR index still names the device.
inline bool RecoverTrackerFrames(const protocol::Response &response, uint64_t session,
	uint64_t profile, const FrameSerialKeys &serials, RecoveredFrames &out,
	FrameSerialKeys *restoredKeys = nullptr)
{
	const auto &saved = response.runtimeState;
	if (response.type != protocol::ResponseRuntimeState || response.protocol.version != protocol::Version ||
		session == 0 || response.driverSessionId != session || profile == 0 ||
		saved.frameProfileKey != profile)
		return false;
	for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
        for (uint32_t j = i + 1; j < vr::k_unMaxTrackedDeviceCount; ++j)
            if ((serials[i] != 0 && serials[i] == serials[j]) ||
                (saved.frameSerialKeys[i] != 0 && saved.frameSerialKeys[i] == saved.frameSerialKeys[j]))
                return false;
	RecoveredFrames recovered{};
	FrameSerialKeys keys{};
	std::array<bool, vr::k_unMaxTrackedDeviceCount> used{};
	for (uint32_t id = 0; id < recovered.size(); ++id)
	{
		if (serials[id] == 0) continue;
		for (uint32_t old = 0; old < recovered.size(); ++old)
		{
			if (saved.frameSerialKeys[old] != serials[id]) continue;
			const auto &frame = saved.frames[old];
			const Eigen::Quaterniond rotation(frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z);
			if (!IsValidCalibrationTransform(rotation, Eigen::Vector3d(frame.translation.v), 1.0))
				return false;
			used[old] = true;
			keys[id] = serials[id];
			recovered[id] = frame;
			const auto unit = rotation.normalized();
			recovered[id].rotation = { unit.w(), unit.x(), unit.y(), unit.z() };
			break;
		}
	}
	// Keep sleeping devices parked in unused slots, even if another serial
	// has occupied their old index. Publication keeps parked slots disabled;
	// Bind requires the serial before moving a correction to a live device.
	for (uint32_t old = 0; old < recovered.size(); ++old)
	{
		if (used[old] || saved.frameSerialKeys[old] == 0) continue;
		uint32_t id = old;
		if (serials[id] != 0 || keys[id] != 0)
		{
			for (id = 0; id < recovered.size(); ++id)
				if (serials[id] == 0 && keys[id] == 0) break;
			if (id == recovered.size()) continue;
		}
		const auto &frame = saved.frames[old];
		const Eigen::Quaterniond rotation(frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z);
		if (!IsValidCalibrationTransform(rotation, Eigen::Vector3d(frame.translation.v), 1.0)) return false;
		recovered[id] = frame;
		const auto unit = rotation.normalized();
		recovered[id].rotation = { unit.w(), unit.x(), unit.y(), unit.z() };
		keys[id] = saved.frameSerialKeys[old];
	}
	out = recovered;
	if (restoredKeys) *restoredKeys = keys;
	return true;
}
} // namespace questcal
