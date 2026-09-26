#pragma once

#include "../common/Protocol.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <deque>

// Passive drift-evidence collection. No shadow solve exists during play (the
// devices are not rigidly attached), so this gathers the honest observables:
//
//  - Stationary-slide: while a device is truly at rest, a coherent shift of
//    its raw position is direct evidence its universe is drifting under it.
//    "At rest" is decided on short-timescale statistics: the window is split
//    into ~1 s chunks, and every chunk must be internally quiet (RMS at
//    sensor-noise level) with consecutive chunk means barely moving. Worn or
//    held devices always carry short-timescale energy -- posture settling,
//    tremor, weight shifts -- so they cannot qualify, which is the point:
//    the first live session produced a storm of false "slides" from a
//    just-stopped user and set-down peripherals under the old variance gate.
//    Raw ring positions are converted to calibrated meters with the caller's
//    uniform scale. Rotation and translation are omitted because they preserve
//    displacement magnitude, so there is still no calibration feedback loop.
//  - Discontinuous tracking-loss recovery: lighthouse occlusion losses are
//    constant and benign; only a short loss whose recovery pose is far from
//    the pre-loss pose counts as evidence (glitch / base-station re-seat).
//
// Detection band: drift between ~1.5 and ~5 mm/s registers within one window.
// Slower drift never trips this detector -- calibration age and jump events
// carry that part of the score. Faster motion is, by construction, a human.
//
// The caller decides which devices are eligible: only devices that anchor a
// universe belong here (see the feed site in Calibration.cpp).
//
// Events feed the staleness score; nothing here auto-corrects anything.
class DriftMonitor
{
public:
	// Fixed detection policy, not a caller knob: no code outside this class
	// ever varied it, so the values live here as a named-constant block with
	// their rationale rather than behind a setter nobody called.
	struct Config
	{
		double window = 8.0;             // seconds of rest required before judging
		double chunkSeconds = 1.0;       // sub-window for short-timescale statistics
		double chunkJitter = 0.004;      // meters; max RMS deviation inside any chunk
		double chunkStep = 0.005;        // meters; max motion between consecutive chunk means
		double slideThreshold = 0.012;   // meters between first and last chunk means
		double lossGap = 0.3;            // seconds invalid before it counts as a loss
		double maxLossGap = 1.0;         // longer absences are not classifiable (walked away, powered off)
		double lossJump = 0.25;          // meters of recovery discontinuity
	};

	struct Event
	{
		enum Type { StationarySlide, DiscontinuousLoss };
		Type type = StationarySlide;
		uint32_t deviceId = 0;
		double magnitude = 0.0;          // meters
		double time = 0.0;               // ring seconds
	};

	explicit DriftMonitor(double qpcToSeconds) : qpcToSeconds(qpcToSeconds) { }

	// Feed an eligible device's ring samples; each device is evaluated
	// independently.
	// `linearScale` converts the sample's raw tracking units into calibrated
	// meters. Reference-system samples use 1; target-system samples use the
	// active calibration scale.
	void Push(const protocol::DevicePoseSample &sample, double linearScale = 1.0);

	bool PollEvent(Event &out);

	void Reset();

private:
	struct Snap
	{
		double t;
		Eigen::Vector3d pos;
	};

	// The last accepted sample, as one record. All four values are written
	// together and read together, with `time < 0` as the group's validity
	// sentinel — so clearing the group is one assignment and there is no way
	// to clear three of the four and leave a position from a previous
	// calibration session behind for a recovery pose to be compared against.
	struct LastValid
	{
		double time = -1.0;   // < 0: nothing accepted yet for this device
		double linearScale = 0.0;
		Eigen::Vector3d pos{ 0, 0, 0 };
		Eigen::Vector3d vel{ 0, 0, 0 };
	};

	struct DeviceState
	{
		std::deque<Snap> window;
		double lastEvalTime = -1.0;
		LastValid lastValid;
	};

	void EvaluateWindow(uint32_t id, DeviceState &dev);

	Config config;
	double qpcToSeconds;
	DeviceState devices[vr::k_unMaxTrackedDeviceCount];
	std::deque<Event> events;
};
