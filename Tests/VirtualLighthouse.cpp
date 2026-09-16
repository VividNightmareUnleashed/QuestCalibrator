#include "VirtualLighthouse.h"

#include <algorithm>
#include <random>

namespace vlighthouse
{

Truth StillAt(const Eigen::Vector3d &position)
{
	return [position](double) { return TruthSample{ position, Eigen::Vector3d::Zero() }; };
}

Truth MoveBetween(const Eigen::Vector3d &from, const Eigen::Vector3d &to,
	double start, double seconds)
{
	return [=](double t)
	{
		if (t <= start)
			return TruthSample{ from, Eigen::Vector3d::Zero() };
		if (t >= start + seconds)
			return TruthSample{ to, Eigen::Vector3d::Zero() };
		Eigen::Vector3d velocity = (to - from) / seconds;
		return TruthSample{ from + velocity * (t - start), velocity };
	};
}

Truth DriftFrom(const Eigen::Vector3d &from, const Eigen::Vector3d &velocity, double start)
{
	return [=](double t)
	{
		if (t <= start)
			return TruthSample{ from, Eigen::Vector3d::Zero() };
		return TruthSample{ from + velocity * (t - start), velocity };
	};
}

namespace
{

lighthouselog::Event Line(const Config &cfg, lighthouselog::Event::Kind kind, double t,
	int channel, const std::vector<int> &visible, bool historical)
{
	lighthouselog::Event e;
	e.kind = kind;
	e.serial = cfg.serial;
	e.channel = channel;
	e.stationId = channel >= 0 ? cfg.stationIdBase + static_cast<uint32_t>(channel) : 0;
	e.visibleKnown = kind != lighthouselog::Event::Kind::Bootstrapped &&
		kind != lighthouselog::Event::Kind::BootstrapFailed;
	if (e.visibleKnown)
	{
		e.visibleChannels = visible;
		for (int c : visible)
			e.visibleIds.push_back(cfg.stationIdBase + static_cast<uint32_t>(c));
	}
	e.timeKnown = true;
	e.unixTime = cfg.unixEpoch + t;
	e.historical = historical;
	return e;
}

// The lines the driver writes for one change of the visible set: drops one
// at a time (each listing what remains), "no base stations seen" when the
// set empties, a bootstrap when it refills from nothing, adds one at a time.
void EmitChange(const Config &cfg, double t, const std::vector<int> &from, const std::vector<int> &to,
	bool historical, std::vector<TimedEvent> &out)
{
	using K = lighthouselog::Event::Kind;
	std::vector<int> current = from;
	for (int c : from)
	{
		if (std::find(to.begin(), to.end(), c) != to.end())
			continue;
		current.erase(std::remove(current.begin(), current.end(), c), current.end());
		out.push_back({ t, Line(cfg, K::StationDropped, t, c, current, historical) });
		if (current.empty())
			out.push_back({ t, Line(cfg, K::NoneSeen, t, -1, current, historical) });
	}
	bool bootstrapping = current.empty() && !to.empty();
	for (int c : to)
	{
		if (std::find(current.begin(), current.end(), c) != current.end())
			continue;
		if (bootstrapping)
		{
			out.push_back({ t, Line(cfg, K::Bootstrapped, t, c, {}, historical) });
			bootstrapping = false;
		}
		current.push_back(c);
		out.push_back({ t, Line(cfg, K::StationAdded, t, c, current, historical) });
	}
}

} // namespace

Output Run(const Config &cfg, const Truth &truth,
	const std::vector<VisibilityChange> &schedule, double duration, unsigned seed)
{
	Output out;
	std::mt19937 rng(seed);
	std::uniform_real_distribution<double> jitter(-cfg.positionJitter, cfg.positionJitter);

	std::vector<int> visible;
	size_t next = 0;
	// The initial state is what a replayed log would report at startup.
	if (!schedule.empty() && schedule[0].time <= 0.0)
	{
		EmitChange(cfg, 0.0, {}, schedule[0].visible, true, out.events);
		visible = schedule[0].visible;
		next = 1;
	}

	const double dt = 1.0 / cfg.frameRate;
	int previousCount = static_cast<int>(visible.size());
	double singleSince = 0.0;
	double coastSince = 0.0;
	Eigen::Vector3d coastOrigin = Eigen::Vector3d::Zero();
	Eigen::Vector3d coastVelocity = Eigen::Vector3d::Zero();
	Eigen::Vector3d lastReported = truth(0.0).position;

	const long frames = static_cast<long>(duration * cfg.frameRate + 0.5);
	for (long i = 0; i <= frames; ++i)
	{
		const double t = i * dt;
		while (next < schedule.size() && schedule[next].time <= t)
		{
			EmitChange(cfg, schedule[next].time, visible, schedule[next].visible, false, out.events);
			visible = schedule[next].visible;
			++next;
		}
		const int count = static_cast<int>(visible.size());
		if (count != previousCount)
		{
			if (count == 1)
				singleSince = t;
			if (count == 0)
			{
				coastSince = t;
				coastOrigin = lastReported;
				coastVelocity = truth(t).velocity;
			}
			previousCount = count;
		}

		const TruthSample now = truth(t);
		Frame f;
		f.time = t;
		f.stations = count;
		Eigen::Vector3d position = now.position;
		Eigen::Vector3d velocity = now.velocity;
		bool valid = true;
		if (count == 1)
		{
			double swim = std::min(cfg.singleStationSlideRate * (t - singleSince),
				cfg.singleStationSlideMax);
			position += swim * cfg.singleStationDirection.normalized();
		}
		else if (count == 0)
		{
			f.optical = false;
			const double coasting = t - coastSince;
			position = coastOrigin + coastVelocity * coasting + 0.5 * cfg.coastBias * coasting * coasting;
			velocity = coastVelocity + cfg.coastBias * coasting;
			valid = coasting <= cfg.coastValidSeconds;
		}
		if (count > 0)
			for (int k = 0; k < 3; ++k)
				position(k) += jitter(rng);
		lastReported = position;

		protocol::DevicePoseSample &s = f.sample;
		s.sampleTimeQpc = static_cast<int64_t>(t / cfg.qpcToSeconds + 0.5);
		s.deviceId = cfg.deviceId;
		s.deviceIsConnected = true;
		s.poseIsValid = valid;
		s.trackingResult = static_cast<uint32_t>(valid
			? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange);
		s.worldFromDriverRotation = { 1, 0, 0, 0 };
		s.rotation = { 1, 0, 0, 0 };
		for (int k = 0; k < 3; ++k)
		{
			s.position[k] = position(k);
			s.velocity[k] = velocity(k);
		}
		out.frames.push_back(f);
	}
	return out;
}

} // namespace vlighthouse
