// PCH-free on purpose: SolverTests compiles this file standalone.
#include "DriftMonitor.h"
#include "RingPoseMath.h"

#include <cmath>

static_assert(vr::k_unMaxTrackedDeviceCount <= 64, "DriftMonitor device array undersized");

void DriftMonitor::Push(const protocol::DevicePoseSample &s, double linearScale)
{
	if (s.deviceId >= 64)
		return;
	if (!std::isfinite(linearScale) || linearScale < protocol::limits::MinScale ||
		linearScale > protocol::limits::MaxScale)
		return;

	bool valid = s.poseIsValid && s.trackingResult == static_cast<uint32_t>(vr::TrackingResult_Running_OK);
	if (!valid || !IsUsableRingSample(s, qpcToSeconds))
		return;   // absence; loss classification happens on recovery

	auto &dev = devices[s.deviceId];
	double t = RingSampleTime(s, qpcToSeconds);
	if (dev.lastValidTime >= 0.0 && t <= dev.lastValidTime)
		return;

	// Scale is part of this device's coordinate basis. Never compare samples
	// across an intentional profile/scale edit: the same raw pose would appear
	// to move solely because its units changed.
	if (dev.lastValidTime >= 0.0 &&
		std::abs(linearScale - dev.lastLinearScale) >
			1e-6 * (1.0 + std::abs(dev.lastLinearScale)))
	{
		dev.window.clear();
		dev.lastValidTime = -1.0;
		dev.lastEvalTime = -1.0;
	}

	RingSampleParts p = UnpackRingSample(s);
	Eigen::Vector3d pos = linearScale * (p.wfdRot * p.drvPos + p.wfdTrans);
	Eigen::Vector3d velocity = linearScale * (p.wfdRot * Eigen::Vector3d(
		s.velocity[0], s.velocity[1], s.velocity[2]));

	// Discontinuous-loss recovery: only short absences count, and only when
	// the device reappears far from where it vanished.
	if (dev.lastValidTime >= 0.0)
	{
		double gap = t - dev.lastValidTime;
		if (gap > config.lossGap)
		{
			dev.window.clear();
			// Ordinary motion through a short occlusion is not a re-localization.
			// Compare recovery against a trapezoidal velocity prediction so only
			// unexplained displacement contributes drift evidence.
			Eigen::Vector3d predicted = dev.lastValidPos +
				0.5 * (dev.lastValidVel + velocity) * gap;
			double residual = (pos - predicted).norm();
			if (gap <= config.maxLossGap && residual > config.lossJump)
				events.push_back({ Event::DiscontinuousLoss, s.deviceId, residual, t });
		}
	}

	dev.lastValidTime = t;
	dev.lastLinearScale = linearScale;
	dev.lastValidPos = pos;
	dev.lastValidVel = velocity;

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

	Eigen::Vector3d mean[MaxChunks];
	double sqDev[MaxChunks];
	int count[MaxChunks];
	for (int c = 0; c < chunks; ++c)
	{
		mean[c] = Eigen::Vector3d::Zero();
		sqDev[c] = 0.0;
		count[c] = 0;
	}

	auto chunkOf = [&](double t)
	{
		int c = static_cast<int>((t - t0) / (span / chunks));
		return c < chunks ? c : chunks - 1;
	};

	for (const auto &s : dev.window)
	{
		int c = chunkOf(s.t);
		mean[c] += s.pos;
		count[c]++;
	}
	for (int c = 0; c < chunks; ++c)
	{
		if (count[c] < 4)
			return;   // stream gap inside the window; refuse to judge
		mean[c] /= static_cast<double>(count[c]);
	}
	for (const auto &s : dev.window)
	{
		int c = chunkOf(s.t);
		sqDev[c] += (s.pos - mean[c]).squaredNorm();
	}

	// Rest gates: every chunk internally quiet (sensor-noise level), and no
	// chunk-to-chunk step a human wouldn't undercut but slow drift would.
	for (int c = 0; c < chunks; ++c)
	{
		if (std::sqrt(sqDev[c] / count[c]) > config.chunkJitter)
			return;
		if (c > 0 && (mean[c] - mean[c - 1]).norm() > config.chunkStep)
			return;
	}

	double slide = (mean[chunks - 1] - mean[0]).norm();
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
		dev.lastValidTime = -1.0;
		dev.lastEvalTime = -1.0;
		dev.lastLinearScale = 0.0;
		dev.lastValidVel = Eigen::Vector3d::Zero();
	}
	events.clear();
}
