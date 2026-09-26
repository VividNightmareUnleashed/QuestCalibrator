#pragma once

// RingPoseMath.h first: its Protocol.h settles which OpenVR header this TU
// has, and ChaperoneMath.h adds none once one is in.
#include "RingPoseMath.h"
#include "ChaperoneMath.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

// Moves of the lighthouse side's frame, which nothing else watches: the jump
// detector follows the headset side only, and the vrserver.txt tailer reads
// per-device lines. The lighthouse driver reports a tracker's pose in a base
// station's frame, with that station's pose in the universe as the sample's
// worldFromDriver; a base station's own sample is its pose over an identity
// local pose. If SteamVR re-solves the stations, every lighthouse device's
// worldFromDriver changes together while its local pose stays on its
// trajectory, and each world pose moves by the same transform. A headset
// tracker moved that way looks exactly like the two universes moving apart
// (live 2026-09-24: 62 deg of yaw, 78.6 deg of tilt, 5 m at the head, rigid
// for an hour and gone after a SteamVR restart); this records whether it was.
//
// Each worldFromDriver change is classified from the device's own samples:
//   moved         the local pose stayed on its trajectory, or unchanged for a
//                 device at rest, so the world pose moved by the change;
//   re-expressed  the world pose stayed put: the same pose in another frame;
//   unclear       both moved, e.g. a tracker restarting at the same moment.
// Changes within groupSeconds of the first one form a report carrying the
// world delta, how far the moved devices' deltas disagree (not at all for a
// whole-universe move) and which devices moved.
class LighthouseFrameWatch
{
public:
	struct Config
	{
		double groupSeconds = 1.0;
		// A device at rest: its pose unchanged within these, however long
		// between samples (base stations publish rarely).
		double restPositionM = 0.001;
		double restRotationRad = 0.05 * 3.14159265358979323846 / 180.0;
	};

	enum class Kind { Moved, ReExpressed, Unclear };

	struct Report
	{
		double time = 0.0;
		int moved = 0, movedBaseStations = 0, reExpressed = 0, unclear = 0;
		std::vector<uint32_t> movedIds;
		// The world delta (new world = rotation * old world + translation) of
		// the first moved device, a base station's when one moved.
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
		Eigen::Vector3d translation{ 0, 0, 0 };
		double yawDeg = 0.0, tiltDeg = 0.0;
		double largestShiftM = 0.0;             // furthest a moved device's world pose jumped
		double spreadDeg = 0.0, spreadM = 0.0;  // the moved deltas' largest disagreement with it
	};

	LighthouseFrameWatch() = default;
	explicit LighthouseFrameWatch(const Config &c) : config(c) {}

	// A sample from a device off the headset side. Untrusted samples are
	// skipped; a disconnected device is forgotten, so a device later given
	// its index is not compared with it.
	void Note(const protocol::DevicePoseSample &s, double qpcToSeconds, bool baseStation)
	{
		if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
			return;
		Device &d = devices[s.deviceId];
		if (!s.deviceIsConnected)
		{
			d.valid = false;
			return;
		}
		if (!IsTrustedRingSample(s, qpcToSeconds))
			return;

		RingSampleParts p = UnpackRingSample(s);
		Device now;
		now.valid = true;
		now.time = RingSampleTime(s, qpcToSeconds);
		now.wfdRot = p.wfdRot;
		now.wfdTrans = p.wfdTrans;
		now.drvRot = p.drvRot;
		now.drvPos = p.drvPos;
		now.vel = Eigen::Vector3d(s.velocity[0], s.velocity[1], s.velocity[2]);
		now.angVel = Eigen::Vector3d(s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]);

		latestTime = (std::max)(latestTime, now.time);
		if (d.valid && questcal::WorldFromDriverChanged(d.wfdRot, d.wfdTrans, now.wfdRot, now.wfdTrans))
			Add(Classify(s.deviceId, baseStation, d, now));
		d = now;
	}

	// Reports whose group has closed: a sample arrived more than
	// groupSeconds after its first change.
	std::vector<Report> Flush()
	{
		if (!open.empty() && latestTime - open.front().time > config.groupSeconds)
			Close();
		std::vector<Report> out;
		out.swap(closed);
		return out;
	}

	void Reset()
	{
		for (auto &d : devices)
			d.valid = false;
		open.clear();
		closed.clear();
		latestTime = -1e300;
	}

	// A move large enough for the session log: a centimeter at a device or a
	// tenth of a degree. Smaller ones (a station's pose refined in place)
	// belong in the detailed log with the re-expressions.
	static bool Notable(const Report &r)
	{
		return r.moved > 0 && (r.largestShiftM >= 0.01 || r.yawDeg >= 0.1 || r.tiltDeg >= 0.1);
	}

	// One log line for a report.
	static std::string Describe(const Report &r, uint32_t headsetTrackerId)
	{
		char buf[320];
		std::string extra;
		if (r.moved > 0 && r.reExpressed > 0)
			extra += ", " + std::to_string(r.reExpressed) + " re-expressed without moving";
		if (r.unclear > 0)
			extra += ", " + std::to_string(r.unclear) + " whose own pose jumped as well";
		if (r.moved > 0)
		{
			const bool tracker = std::find(r.movedIds.begin(), r.movedIds.end(), headsetTrackerId) !=
				r.movedIds.end();
			char agree[96] = "";
			if (r.moved > 1)
				std::snprintf(agree, sizeof agree, "; their deltas agree within %.2f deg / %.1f cm",
					r.spreadDeg, r.spreadM * 100.0);
			std::snprintf(buf, sizeof buf,
				"lighthouse frame moved: %d device(s) (%d base station(s)%s), yaw %.2f deg, tilt %.2f deg, "
				"largest device shift %.1f cm%s%s",
				r.moved, r.movedBaseStations, tracker ? ", the headset tracker among them" : "",
				r.yawDeg, r.tiltDeg, r.largestShiftM * 100.0, agree, extra.c_str());
		}
		else if (r.unclear > 0)
		{
			std::snprintf(buf, sizeof buf,
				"lighthouse frame changed on %d device(s) whose own pose jumped as well, %d re-expressed without moving",
				r.unclear, r.reExpressed);
		}
		else
		{
			std::snprintf(buf, sizeof buf,
				"lighthouse: %d device(s) re-expressed their pose in another frame without moving", r.reExpressed);
		}
		return buf;
	}

private:
	struct Device
	{
		bool valid = false;
		double time = 0.0;
		Eigen::Quaterniond wfdRot{ 1, 0, 0, 0 };
		Eigen::Vector3d wfdTrans{ 0, 0, 0 };
		Eigen::Quaterniond drvRot{ 1, 0, 0, 0 };
		Eigen::Vector3d drvPos{ 0, 0, 0 };
		Eigen::Vector3d vel{ 0, 0, 0 }, angVel{ 0, 0, 0 };
	};

	struct Change
	{
		uint32_t id = 0;
		bool baseStation = false;
		Kind kind = Kind::Unclear;
		double time = 0.0;
		Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
		Eigen::Vector3d translation{ 0, 0, 0 };
		double shiftM = 0.0;
	};

	bool Unchanged(const Eigen::Quaterniond &r0, const Eigen::Vector3d &p0,
		const Eigen::Quaterniond &r1, const Eigen::Vector3d &p1) const
	{
		return (p1 - p0).norm() <= config.restPositionM &&
			r1.angularDistance(r0) <= config.restRotationRad;
	}

	Change Classify(uint32_t id, bool baseStation, const Device &was, const Device &is) const
	{
		ringpose::DriverLocalPoseSample localWas{ was.time, was.drvRot, was.drvPos, was.vel, was.angVel };
		ringpose::DriverLocalPoseSample localIs{ is.time, is.drvRot, is.drvPos, is.vel, is.angVel };
		const bool localKept = ringpose::IsDriverLocalPoseContinuous(localWas, localIs) ||
			Unchanged(was.drvRot, was.drvPos, is.drvRot, is.drvPos);

		ringpose::DriverLocalPoseSample worldWas{ was.time, (was.wfdRot * was.drvRot).normalized(),
			was.wfdRot * was.drvPos + was.wfdTrans, was.wfdRot * was.vel, was.wfdRot * was.angVel };
		ringpose::DriverLocalPoseSample worldIs{ is.time, (is.wfdRot * is.drvRot).normalized(),
			is.wfdRot * is.drvPos + is.wfdTrans, is.wfdRot * is.vel, is.wfdRot * is.angVel };
		const bool worldKept = ringpose::IsDriverLocalPoseContinuous(worldWas, worldIs) ||
			Unchanged(worldWas.rotation, worldWas.position, worldIs.rotation, worldIs.position);

		Change c;
		c.id = id;
		c.baseStation = baseStation;
		c.time = is.time;
		c.kind = worldKept ? Kind::ReExpressed : localKept ? Kind::Moved : Kind::Unclear;
		c.rotation = (is.wfdRot * was.wfdRot.conjugate()).normalized();
		c.translation = is.wfdTrans - c.rotation * was.wfdTrans;
		// The current local pose in both frames: how far the change alone
		// moved the device.
		c.shiftM = ((is.wfdRot * is.drvPos + is.wfdTrans) - (was.wfdRot * is.drvPos + was.wfdTrans)).norm();
		return c;
	}

	void Add(const Change &c)
	{
		if (!open.empty() && c.time - open.front().time > config.groupSeconds)
			Close();
		open.push_back(c);
	}

	void Close()
	{
		Report r;
		r.time = open.front().time;
		std::set<uint32_t> moved, movedStations, reExpressed, unclear;
		const Change *reference = nullptr;
		for (const auto &c : open)
		{
			if (c.kind == Kind::Moved)
			{
				moved.insert(c.id);
				if (c.baseStation)
					movedStations.insert(c.id);
				if (!reference || (c.baseStation && !reference->baseStation))
					reference = &c;
				r.largestShiftM = (std::max)(r.largestShiftM, c.shiftM);
			}
			else if (c.kind == Kind::ReExpressed)
				reExpressed.insert(c.id);
			else
				unclear.insert(c.id);
		}
		r.moved = static_cast<int>(moved.size());
		r.movedBaseStations = static_cast<int>(movedStations.size());
		r.reExpressed = static_cast<int>(reExpressed.size());
		r.unclear = static_cast<int>(unclear.size());
		r.movedIds.assign(moved.begin(), moved.end());
		if (reference)
		{
			r.rotation = reference->rotation;
			r.translation = reference->translation;
			double tilt = 0.0;
			Eigen::Quaterniond yaw = questcal::YawOnlyRotation(r.rotation, &tilt);
			if (yaw.w() < 0.0)
				yaw.coeffs() = -yaw.coeffs();
			r.yawDeg = std::abs(2.0 * std::atan2(yaw.y(), yaw.w())) * 180.0 / 3.14159265358979323846;
			r.tiltDeg = tilt * 180.0 / 3.14159265358979323846;
			for (const auto &c : open)
			{
				if (c.kind != Kind::Moved)
					continue;
				r.spreadDeg = (std::max)(r.spreadDeg,
					c.rotation.angularDistance(r.rotation) * 180.0 / 3.14159265358979323846);
				r.spreadM = (std::max)(r.spreadM, (c.translation - r.translation).norm());
			}
		}
		closed.push_back(r);
		open.clear();
	}

	Config config;
	Device devices[vr::k_unMaxTrackedDeviceCount];
	std::vector<Change> open;
	std::vector<Report> closed;
	double latestTime = -1e300;
};
