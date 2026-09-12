#pragma once

// A simulated lighthouse-tracked device: the pose stream and the driver log
// lines QuestCalibrator sees from one tracker while its base stations come
// and go. Test-only.
//
// What is reproduced is the shape of the failure, not Valve's tracking code
// (which is closed): a device with two or more stations in view reports the
// truth plus optical noise; with one station its solution has a single
// baseline and is allowed to swim slowly along that station's line of
// sight; with none it coasts on the IMU for a short while, then reports its
// pose invalid and out of range; when a station returns, the optical
// solution snaps back to the truth. Every visibility change also produces
// the log lines the real driver writes for it, so a scenario can drive the
// drift monitor and the visibility state exactly as the overlay does.
//
// Where the numbers come from: the timing of real losses (how often a
// station drops out, how long it stays lost, how many remain) is read from
// a session's vrserver.txt and is documented with the scenarios. The pose
// behaviour while a station is missing (swim rate, coast bias, how long a
// coasting pose stays valid) is assumed and named as such in Config; the
// scenarios that depend on it only need the failure to exist, not its exact
// size.

#include "../Overlay/LighthouseLog.h"
#include "../common/Protocol.h"

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace vlighthouse
{

struct Config
{
	uint32_t deviceId = 5;
	std::string serial = "LHR-00000001";
	double frameRate = 120.0;
	double qpcToSeconds = 1e-6;
	double positionJitter = 0.0003;           // m; optical solution noise (uniform)

	// Assumed: a single-station solution swims along that station's line of
	// sight, linearly up to a cap.
	double singleStationSlideRate = 0.003;    // m/s
	double singleStationSlideMax = 0.03;      // m
	Eigen::Vector3d singleStationDirection{ 0, 0, 1 };

	// Assumed: with no station the pose coasts on the IMU with a constant
	// acceleration bias, stays valid this long, then goes out of range.
	double coastValidSeconds = 0.2;
	Eigen::Vector3d coastBias{ 0.3, 0, 0 };  // m/s^2

	double unixEpoch = 1.7e9;                 // wall clock of t = 0 on the log lines
	uint32_t stationIdBase = 0x1000;          // station id = base + channel
};

// The device's visible channels from `time` on. The first entry is the state
// at t = 0; its lines are emitted as historical, the way a log replayed at
// startup reports them.
struct VisibilityChange
{
	double time = 0.0;
	std::vector<int> visible;
};

struct TruthSample
{
	Eigen::Vector3d position{ 0, 0, 0 };
	Eigen::Vector3d velocity{ 0, 0, 0 };
};
using Truth = std::function<TruthSample(double)>;

Truth StillAt(const Eigen::Vector3d &position);
// Still at `from` until `start`, then moves in a straight line to `to` over
// `seconds`, then still there.
Truth MoveBetween(const Eigen::Vector3d &from, const Eigen::Vector3d &to,
	double start, double seconds);
// Still at `from`, then drifts at `velocity` from `start` on.
Truth DriftFrom(const Eigen::Vector3d &from, const Eigen::Vector3d &velocity, double start);

struct Frame
{
	double time = 0.0;
	int stations = 0;
	bool optical = true;         // the pose came from the stations, not the IMU
	protocol::DevicePoseSample sample;
};

struct TimedEvent
{
	double time = 0.0;
	lighthouselog::Event event;
};

struct Output
{
	std::vector<Frame> frames;
	std::vector<TimedEvent> events;   // in time order, before the frame of the same time
};

Output Run(const Config &config, const Truth &truth,
	const std::vector<VisibilityChange> &schedule, double duration, unsigned seed = 1);

} // namespace vlighthouse
