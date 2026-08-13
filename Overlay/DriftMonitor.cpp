// PCH-free on purpose: SolverTests compiles this file standalone.
#include "DriftMonitor.h"
#include "RingPoseMath.h"

#include <cmath>

void DriftMonitor::Push(const protocol::DevicePoseSample &s, double linearScale)
{
	if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
		return;
	if (!std::isfinite(linearScale) || linearScale < protocol::limits::MinScale ||
		linearScale > protocol::limits::MaxScale)
		return;

	bool valid = s.poseIsValid && s.trackingResult == static_cast<uint32_t>(vr::TrackingResult_Running_OK);
	if (!valid || !IsUsableRingSample(s, qpcToSeconds))
		return;   // absence; loss classification happens on recovery

	auto &dev = devices[s.deviceId];
	double t = RingSampleTime(s, qpcToSeconds);
	bool haveLast = dev.lastValid.time >= 0.0;
	if (haveLast && t <= dev.lastValid.time)
		return;

	// Scale is part of this device's coordinate basis. Never compare samples
	// across an intentional profile/scale edit: the same raw pose would appear
	// to move solely because its units changed. Drops the window too — the
	// retained positions are in the old units.
	if (haveLast &&
		std::abs(linearScale - dev.lastValid.linearScale) >
			1e-6 * (1.0 + std::abs(dev.lastValid.linearScale)))
	{
		dev.window.clear();
		dev.lastValid = LastValid();
		dev.lastEvalTime = -1.0;
		haveLast = false;
	}

	RingSampleParts p = UnpackRingSample(s);
	Eigen::Vector3d pos = linearScale * (p.wfdRot * p.drvPos + p.wfdTrans);
	Eigen::Vector3d velocity = linearScale * (p.wfdRot * Eigen::Vector3d(
		s.velocity[0], s.velocity[1], s.velocity[2]));

	// Discontinuous-loss recovery: only short absences count, and only when
	// the device reappears far from where it vanished.
	if (haveLast)
	{
		double gap = t - dev.lastValid.time;
		if (gap > config.lossGap)
		{
			dev.window.clear();
			// Ordinary motion through a short occlusion is not a re-localization.
			// Compare recovery against a trapezoidal velocity prediction so only
			// unexplained displacement contributes drift evidence.
			Eigen::Vector3d predicted = dev.lastValid.pos +
				0.5 * (dev.lastValid.vel + velocity) * gap;
			double residual = (pos - predicted).norm();
			if (gap <= config.maxLossGap && residual > config.lossJump)
				events.push_back({ Event::DiscontinuousLoss, s.deviceId, residual, t });
		}
	}

	dev.lastValid = LastValid{ t, linearScale, pos, velocity };   // all four, always together

	dev.window.push_back({ t, pos });
	while (!dev.window.empty() && t - dev.window.front().t > config.window)
		dev.window.pop_front();

	// Judging on every push would be wasted work at ring rate; a few times a
	// second is plenty for a signal this slow.
	if (!dev.window.empty() && t - dev.window.front().t > config.window * 0.95 &&
		t - dev.lastEvalTime > 0.25)
	{
		dev.lastEvalTime = t;
		EvaluateWindow(s.deviceId, dev);
	}
}

void DriftMonitor::EvaluateWindow(uint32_t id, DeviceState &dev)
{
	const double t0 = dev.window.front().t;
	const double span = dev.window.back().t - t0;
	if (span <= 0.0)
		return;

	constexpr int MaxChunks = 16;
	int chunks = static_cast<int>(span / config.chunkSeconds);
	if (chunks < 6)
		return;
	if (chunks > MaxChunks)
		chunks = MaxChunks;

	// The width the gates below actually describe: the span is divided evenly
	// among the chunks, so it is at least the configured chunkSeconds and
	// grows past it once the MaxChunks clamp binds. chunkJitter and chunkStep
	// are tuned against this timescale, not against config.chunkSeconds — a
	// window long enough to bind the clamp is applying them to a longer one.
	const double chunkWidth = span / chunks;

	// One record per chunk: filled in two passes (the stable two-pass variance,
	// not the catastrophically-cancelling sum-of-squares form on room-scale
	// coordinates) and read as a unit.
	struct Chunk
	{
		Eigen::Vector3d mean = Eigen::Vector3d::Zero();
		double sqDev = 0.0;
		int count = 0;
	};
	Chunk chunk[MaxChunks];

	auto chunkOf = [&](double t)
	{
		int c = static_cast<int>((t - t0) / chunkWidth);
		return c < chunks ? c : chunks - 1;
	};

	for (const auto &s : dev.window)
	{
		Chunk &k = chunk[chunkOf(s.t)];
		k.mean += s.pos;
		k.count++;
	}
	for (int c = 0; c < chunks; ++c)
	{
		if (chunk[c].count < 4)
			return;   // stream gap inside the window; refuse to judge
		chunk[c].mean /= static_cast<double>(chunk[c].count);
	}
	for (const auto &s : dev.window)
	{
		Chunk &k = chunk[chunkOf(s.t)];
		k.sqDev += (s.pos - k.mean).squaredNorm();
	}

	// Rest gates: every chunk internally quiet (sensor-noise level), and no
	// chunk-to-chunk step a human wouldn't undercut but slow drift would.
	for (int c = 0; c < chunks; ++c)
	{
		if (std::sqrt(chunk[c].sqDev / chunk[c].count) > config.chunkJitter)
			return;
		if (c > 0 && (chunk[c].mean - chunk[c - 1].mean).norm() > config.chunkStep)
			return;
	}

	double slide = (chunk[chunks - 1].mean - chunk[0].mean).norm();
	if (slide > config.slideThreshold)
	{
		events.push_back({ Event::StationarySlide, id, slide, dev.window.back().t });
		dev.window.clear();   // one event per rest episode
	}
}

bool DriftMonitor::PollEvent(Event &out)
{
	if (events.empty())
		return false;
	out = events.front();
	events.pop_front();
	return true;
}

void DriftMonitor::Reset()
{
	for (auto &dev : devices)
	{
		// Everything: a calibration change invalidates the retained window and
		// the last-sample record alike.
		dev.window.clear();
		dev.lastEvalTime = -1.0;
		dev.lastValid = LastValid();
	}
	events.clear();
}
