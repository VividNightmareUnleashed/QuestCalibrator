// PCH-free on purpose: SolverTests compiles this file standalone.
#include "DriftMonitor.h"
#include "RingPoseMath.h"

#include <cmath>

void DriftMonitor::Push(const protocol::DevicePoseSample &s, double linearScale)
{
	auto &dev = devices[s.deviceId];
	double t = RingSampleTime(s, qpcToSeconds);
	bool haveLast = dev.lastValid.time >= 0.0;

	// Scale is part of this device's coordinate basis: across a profile scale
	// edit the same raw pose would appear to move, so the window and the
	// last sample (both in the old units) are dropped.
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

	dev.lastValid = LastValid{ t, linearScale, pos, velocity };

	dev.window.push_back({ t, pos });
	while (t - dev.window.front().t > config.window)
		dev.window.pop_front();

	// A few evaluations a second is plenty for a signal this slow.
	if (t - dev.window.front().t > config.window * 0.95 &&
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

	// Push evaluates only once span exceeds 0.95 * window, and trims it to at
	// most window, so this is 7 or 8 chunks of about chunkSeconds each.
	constexpr int MaxChunks = 16;
	const int chunks = static_cast<int>(span / config.chunkSeconds);
	const double chunkWidth = span / chunks;

	// Two-pass variance: the sum-of-squares form cancels catastrophically on
	// room-scale coordinates.
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
		dev.window.clear();
		dev.lastEvalTime = -1.0;
		dev.lastValid = LastValid();
	}
	events.clear();
}
