#pragma once

#include "LighthouseFrameWatch.h"
#include "ProfileValidation.h"
#include "FrameRecovery.h"

#include <array>
#include <string>

namespace questcal
{

// Session-local normalization into the target world in which the profile was
// measured. It is independent of base/field generations: a shared calibration
// update must never overwrite one tracker's accumulated frame correction.
class TrackerFrameCorrections
{
public:
	using Frames = std::array<protocol::FrameCorrection, vr::k_unMaxTrackedDeviceCount>;

	void Reset() { *this = TrackerFrameCorrections{}; }
	void Restore(const Frames &saved, const FrameSerialKeys &keys)
	{
		frames = saved;
		restoredKeys = keys;
		usableAfter.fill(-1e300);
	}

	// OpenVR normally keeps indices for a server session. Still retire an old
	// correction if a known serial changes; a failed property read is not a
	// new identity and must not erase a returning tracker's correction.
	bool Bind(uint32_t id, const std::string &serial)
	{
		if (id >= frames.size() || serial.empty() || serials[id] == serial)
			return false;
		const uint64_t key = FrameIdentityKey(serial);
		const bool replaced = (!serials[id].empty() && serials[id] != serial) ||
			(restoredKeys[id] != 0 && restoredKeys[id] != key);
		bool relocated = false;
		for (uint32_t old = 0; old < frames.size(); ++old)
		{
			if (old == id || restoredKeys[old] != key) continue;
			frames[id] = frames[old];
			usableAfter[id] = usableAfter[old];
			frames[old] = {};
			usableAfter[old] = -1e300;
			serials[old].clear();
			restoredKeys[old] = 0;
			relocated = true;
			break;
		}
		serials[id] = serial;
		restoredKeys[id] = key;
		if (replaced && !relocated)
		{
			frames[id] = {};
			usableAfter[id] = -1e300;
		}
		return replaced || relocated;
	}

	bool Follow(const LighthouseFrameWatch::Move &move)
	{
		if (move.id >= frames.size() || !std::isfinite(move.time) ||
			(restoredKeys[move.id] != 0 && serials[move.id].empty()) ||
			move.time <= usableAfter[move.id] ||
			!IsValidCalibrationTransform(move.rotation, move.translation, 1.0))
			return false;
		const auto &old = frames[move.id];
		const Eigen::Quaterniond rotation(old.rotation.w, old.rotation.x, old.rotation.y, old.rotation.z);
		const Eigen::Vector3d translation(old.translation.v);
		// N' F = N. Translation here is unscaled; C applies its scale later.
		const Eigen::Quaterniond nextRotation = (rotation * move.rotation.conjugate()).normalized();
		const Eigen::Vector3d nextTranslation = translation - nextRotation * move.translation;
		if (!IsValidCalibrationTransform(nextRotation, nextTranslation, 1.0))
			return false;
		frames[move.id].rotation = { nextRotation.w(), nextRotation.x(), nextRotation.y(), nextRotation.z() };
		for (int axis = 0; axis < 3; ++axis)
			frames[move.id].translation.v[axis] = nextTranslation(axis);
		// The monitor and solver drain separately. Discard samples preceding the
		// most recent move, including a move inferred after a tracker returned,
		// rather than applying the latest correction to the old frame.
		usableAfter[move.id] = move.time;
		return true;
	}

	bool Normalize(uint32_t id, PoseSample &sample) const
	{
		if (id >= frames.size() || !std::isfinite(sample.time) || sample.time < usableAfter[id])
			return false;
		ApplyToPose(id, sample);
		return true;
	}

	// For an already checked sample on another clock (runtime-pose collection).
	void ApplyToPose(uint32_t id, PoseSample &sample) const
	{
		if (id >= frames.size())
			return;
		const auto &frame = frames[id];
		const Eigen::Quaterniond rotation(frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z);
		sample.pos = rotation * sample.pos + Eigen::Vector3d(frame.translation.v);
		sample.rot = (rotation * sample.rot).normalized();
		sample.vel = rotation * sample.vel;
		sample.angVel = rotation * sample.angVel;
	}

	const Frames &Snapshot() const { return frames; }
	// A solve C measured raw poses in one device's frame. Keep the shared
	// calibration in the existing normalized space: C_normalized = C N^-1.
	static bool ExpressCalibration(const Eigen::Quaterniond &n, const Eigen::Vector3d &t,
		Eigen::Quaterniond &rotation, Eigen::Vector3d &translation, double scale)
	{
		const Eigen::Quaterniond nextRotation = (rotation * n.conjugate()).normalized();
		const Eigen::Vector3d nextTranslation = translation - scale * (nextRotation * t);
		if (!IsValidCalibrationTransform(nextRotation, nextTranslation, scale)) return false;
		rotation = nextRotation;
		translation = nextTranslation;
		return true;
	}
	static bool ExpressCalibration(const protocol::FrameCorrection &frame,
		Eigen::Quaterniond &rotation, Eigen::Vector3d &translation, double scale)
	{
		return ExpressCalibration(Eigen::Quaterniond(frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z),
			Eigen::Vector3d(frame.translation.v), rotation, translation, scale);
	}
	const std::string &Serial(uint32_t id) const { return serials.at(id); }
	uint64_t IdentityKey(uint32_t id) const { return restoredKeys.at(id); }

private:
	Frames frames{};
	FrameSerialKeys restoredKeys{};
	std::array<std::string, vr::k_unMaxTrackedDeviceCount> serials{};
	std::array<double, vr::k_unMaxTrackedDeviceCount> usableAfter = []
	{
		std::array<double, vr::k_unMaxTrackedDeviceCount> times{};
		times.fill(-1e300);
		return times;
	}();
};

} // namespace questcal
