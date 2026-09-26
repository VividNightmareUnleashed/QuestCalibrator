#pragma once

#include "RingPoseMath.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

// The detailed log's account of each reference device's pose stream, one
// line per device a minute: how regularly it arrived, what the transport
// did to it and where the device was. These are the facts a tracking
// question comes down to and the session log otherwise never records:
//
//   - rate, the longest interval and the gaps over 100 ms;
//   - samples the driver marked invalid or not tracking;
//   - re-predicted frames: both velocity fields repeated bit for bit while
//     the pose moved on, Virtual Desktop predicting its last sample forward
//     (7.6 % of frames in the 2026-09-11 capture, never two in a row);
//   - the angular velocity alone repeated, which that capture never showed;
//   - positions repeated bit for bit, and how many of those under a moving
//     heading: the engine's 3DoF fallback holding the last position;
//   - speed and turn rate, median and 95th percentile, from the reported
//     velocities: whether the engine's correction smoother could move;
//   - where the device ended the minute and how far from the stream origin
//     it ranged horizontally: the distance a step's shift scales with.
//
// Driver queue drops are counted once for the whole stream.
class TrackingStreamDigest
{
public:
	void Note(const protocol::DevicePoseSample &s, double qpcToSeconds)
	{
		Device &d = devices[s.deviceId];
		++d.samples;
		questcal::PoseSample p;
		if (!TryComposeRingSample(s, qpcToSeconds, p))
		{
			++d.invalid;
			d.havePrevious = false;
			return;
		}
		if (d.havePrevious)
		{
			++d.pairs;
			const double dt = p.time - d.previousTime;
			if (dt > 0.0)
			{
				d.longest = (std::max)(d.longest, dt);
				d.gaps += dt > 0.1;
			}
			const bool sameV = Same(s.velocity, d.previous.velocity);
			const bool sameW = Same(s.angularVelocity, d.previous.angularVelocity);
			const bool samePos = Same(s.position, d.previous.position);
			const bool sameRot = s.rotation.w == d.previous.rotation.w && s.rotation.x == d.previous.rotation.x &&
				s.rotation.y == d.previous.rotation.y && s.rotation.z == d.previous.rotation.z;
			if (sameV && sameW && !(samePos && sameRot))
				++d.repredicted;
			else if (sameW && !sameV)
				++d.angularOnly;
			if (samePos)
			{
				++d.positionRepeats;
				d.heldWhileTurning += !sameRot;
			}
		}
		d.havePrevious = true;
		d.previous = s;
		d.previousTime = p.time;
		if (d.valid == 0)
			d.firstTime = p.time;
		++d.valid;
		d.lastTime = p.time;
		d.speeds.push_back(p.vel.norm());
		d.turns.push_back(p.angVel.norm());
		d.position = p.pos;
		d.yaw = std::atan2(2.0 * (p.rot.w() * p.rot.y() + p.rot.x() * p.rot.z()),
			1.0 - 2.0 * (p.rot.x() * p.rot.x() + p.rot.y() * p.rot.y()));
		const double horizontal = std::hypot(p.pos.x(), p.pos.z());
		d.nearest = (std::min)(d.nearest, horizontal);
		d.farthest = (std::max)(d.farthest, horizontal);
	}

	void NoteDrops(uint64_t poses)
	{
		++dropEvents;
		dropPoses += poses;
	}

	// The lines for the minute ending now, or none when it is not due. Every
	// device keeps its last sample so repeats are counted across the boundary.
	std::vector<std::string> Flush(double time)
	{
		std::vector<std::string> lines;
		if (lastFlush < -1e8)
			lastFlush = time;
		if (time - lastFlush < 60.0)
			return lines;
		lastFlush = time;
		for (auto &entry : devices)
		{
			Device &d = entry.second;
			if (d.samples == 0)
				continue;
			char buf[640];
			if (d.valid == 0)
				snprintf(buf, sizeof buf, "tracking stream, device %u over the last minute: %llu samples, none valid",
					entry.first, static_cast<unsigned long long>(d.samples));
			else
			{
				const double span = d.lastTime - d.firstTime;
				const double pairs = d.pairs > 0 ? static_cast<double>(d.pairs) : 1.0;
				snprintf(buf, sizeof buf,
					"tracking stream, device %u over the last minute: %llu samples at %.1f Hz (longest interval %.0f ms, %llu over 100 ms), %llu invalid; "
					"re-predicted %llu (%.1f%%), angular velocity alone repeated %llu, position repeated %llu (%llu under a moving heading); "
					"speed median %.3f / 95th %.3f m/s, turn median %.3f / 95th %.3f rad/s; "
					"last at (%.3f, %.3f, %.3f) m yaw %.1f deg, %.2f to %.2f m from the stream origin horizontally",
					entry.first, static_cast<unsigned long long>(d.samples),
					span > 0.0 ? (d.valid - 1) / span : 0.0, d.longest * 1000.0,
					static_cast<unsigned long long>(d.gaps), static_cast<unsigned long long>(d.invalid),
					static_cast<unsigned long long>(d.repredicted), 100.0 * d.repredicted / pairs,
					static_cast<unsigned long long>(d.angularOnly), static_cast<unsigned long long>(d.positionRepeats),
					static_cast<unsigned long long>(d.heldWhileTurning),
					Percentile(d.speeds, 0.5), Percentile(d.speeds, 0.95), Percentile(d.turns, 0.5), Percentile(d.turns, 0.95),
					d.position.x(), d.position.y(), d.position.z(), d.yaw * 180.0 / 3.14159265358979,
					d.nearest, d.farthest);
			}
			lines.push_back(buf);
			d.Restart();
		}
		if (dropEvents > 0)
		{
			char buf[128];
			snprintf(buf, sizeof buf, "driver queue drops over the last minute: %llu (%llu pose(s))",
				static_cast<unsigned long long>(dropEvents), static_cast<unsigned long long>(dropPoses));
			lines.push_back(buf);
		}
		dropEvents = dropPoses = 0;
		return lines;
	}

	void Reset()
	{
		devices.clear();
		dropEvents = dropPoses = 0;
		lastFlush = -1e9;
	}

private:
	struct Device
	{
		uint64_t samples = 0, valid = 0, invalid = 0, gaps = 0, pairs = 0;
		uint64_t repredicted = 0, angularOnly = 0, positionRepeats = 0, heldWhileTurning = 0;
		double firstTime = 0.0, lastTime = 0.0, longest = 0.0;
		std::vector<double> speeds, turns;
		Eigen::Vector3d position{ 0, 0, 0 };
		double yaw = 0.0, nearest = 1e9, farthest = 0.0;
		bool havePrevious = false;
		protocol::DevicePoseSample previous{};
		double previousTime = 0.0;

		void Restart()
		{
			samples = valid = invalid = gaps = pairs = 0;
			repredicted = angularOnly = positionRepeats = heldWhileTurning = 0;
			longest = 0.0;
			speeds.clear();
			turns.clear();
			nearest = 1e9;
			farthest = 0.0;
		}
	};

	static bool Same(const double *a, const double *b)
	{
		return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
	}

	// Only called for a device with valid samples, each of which added a value.
	static double Percentile(std::vector<double> &v, double q)
	{
		const size_t k = static_cast<size_t>(q * (v.size() - 1) + 0.5);
		std::nth_element(v.begin(), v.begin() + k, v.end());
		return v[k];
	}

	std::map<uint32_t, Device> devices;
	uint64_t dropEvents = 0, dropPoses = 0;
	double lastFlush = -1e9;
};
