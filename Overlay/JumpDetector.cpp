// PCH-free on purpose: SolverTests compiles this file standalone.
#include "JumpDetector.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

// vr:: constants come from whichever OpenVR header Protocol.h resolved
// (openvr.h when it is already loaded, openvr_driver.h otherwise) — both
// define the ones used here.

static_assert(vr::k_unMaxTrackedDeviceCount <= 64, "JumpDetector device array undersized");

namespace
{

// Heading component of a rotation: the twist around +Y. Well-behaved for the
// gravity-aligned rotations a recenter produces.
double YawOf(const Eigen::Quaterniond &q)
{
	Eigen::Quaterniond twist(q.w(), 0.0, q.y(), 0.0);
	double n = twist.norm();
	if (n < 1e-9)
		return 0.0;
	twist.coeffs() /= n;
	return 2.0 * std::atan2(twist.y(), twist.w());
}

double WrapAngle(double a)
{
	while (a > EIGEN_PI) a -= 2.0 * EIGEN_PI;
	while (a < -EIGEN_PI) a += 2.0 * EIGEN_PI;
	return a;
}

Eigen::Quaterniond YawQuat(double yaw)
{
	return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()));
}

std::string Format(const char *fmt, ...)
{
	char buf[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof buf, fmt, args);
	va_end(args);
	return buf;
}

} // namespace

void JumpDetector::Push(const protocol::DevicePoseSample &s)
{
	if (s.deviceId >= 64)
		return;

	bool valid = s.poseIsValid && s.trackingResult == static_cast<uint32_t>(vr::TrackingResult_Running_OK);
	if (!valid)
		return;   // treated as absence; gaps are measured between valid samples

	auto &dev = devices[s.deviceId];
	double t = static_cast<double>(s.sampleTimeQpc) * qpcToSeconds + s.poseTimeOffset;

	// A hard gap in the reference stream (disconnect, standby): the universe
	// may have moved with no observable frame pair. Never compensate across
	// it — reset baselines and report the event for staleness scoring.
	if (dev.lastValidTime >= 0.0 && t - dev.lastValidTime > config.gapSeconds)
	{
		gaps.push_back({ s.deviceId, t - dev.lastValidTime, t });
		dev.hist.clear();
		dev.wfdValid = false;
		for (auto &c : candidates)
			if (c.deviceId == s.deviceId && !c.ready)
				c.dead = true;
	}

	RingSampleParts p = UnpackRingSample(s);

	bool rebased = false;
	if (dev.wfdValid)
	{
		double dRot = p.wfdRot.angularDistance(dev.wfdRot);
		double dTrans = (p.wfdTrans - dev.wfdTrans).norm();
		if (dRot > config.wfdRotEpsRad || dTrans > config.wfdTransEps)
		{
			DetectWfdRebase(s.deviceId, dev, t, p.wfdRot, p.wfdTrans);
			rebased = true;
		}
	}

	Hist h;
	h.t = t;
	h.pos = p.wfdRot * p.drvPos + p.wfdTrans;
	h.vel = p.wfdRot * Eigen::Vector3d(s.velocity[0], s.velocity[1], s.velocity[2]);
	Eigen::Vector3d worldAngVel = p.wfdRot * Eigen::Vector3d(s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]);
	h.yaw = YawOf(p.wfdRot * p.drvRot);
	h.yawRate = worldAngVel.y();

	if (rebased)
	{
		// The discontinuity is explained exactly; clear composed history so
		// the heuristic path cannot double-fire on the same event.
		dev.hist.clear();
	}
	else if (!dev.hist.empty())
	{
		DetectDiscontinuity(s.deviceId, dev, h);
	}

	dev.hist.push_back(h);
	while (!dev.hist.empty() && h.t - dev.hist.front().t > 2.0 * config.window)
		dev.hist.pop_front();

	dev.wfdRot = p.wfdRot;
	dev.wfdTrans = p.wfdTrans;
	dev.wfdValid = true;
	dev.lastValidTime = t;

	EvaluatePendingCandidates(t);
	TryAccept(t);
}

void JumpDetector::DetectWfdRebase(uint32_t id, DeviceState &dev, double t,
                                   const Eigen::Quaterniond &newRot, const Eigen::Vector3d &newTrans)
{
	// Exact world delta: worlds relate by D = new_wfd ∘ old_wfd⁻¹.
	Eigen::Quaterniond dRot = (newRot * dev.wfdRot.conjugate()).normalized();

	double yaw = YawOf(dRot);
	Eigen::Quaterniond yawRot = YawQuat(yaw);
	double tilt = yawRot.angularDistance(dRot);

	// Constrain to yaw + translation (a recenter preserves gravity); compute
	// the translation with the projected rotation so D stays self-consistent.
	Eigen::Vector3d dTrans = newTrans - yawRot * dev.wfdTrans;

	Candidate c;
	c.deviceId = id;
	c.t = t;
	c.exact = true;
	c.ready = true;
	c.rot = yawRot;
	c.trans = dTrans;
	c.residualTiltRad = tilt;
	candidates.push_back(c);

	notes.push_back(Format("worldFromDriver rebase on device %u: yaw %+.2f deg, shift %.3f m (tilt residual %.2f deg)",
		id, yaw * 180.0 / EIGEN_PI, dTrans.norm(), tilt * 180.0 / EIGEN_PI));
}

void JumpDetector::DetectDiscontinuity(uint32_t id, DeviceState &dev, const Hist &incoming)
{
	const Hist &prev = dev.hist.back();
	double dt = incoming.t - prev.t;
	if (dt <= 0.0 || dt > config.maxFrameGap)
		return;

	Eigen::Vector3d meanVel = 0.5 * (prev.vel + incoming.vel);
	double posErr = (incoming.pos - prev.pos - meanVel * dt).norm();
	double yawErr = std::abs(WrapAngle(incoming.yaw - prev.yaw - prev.yawRate * dt));

	if (posErr < config.discontinuityPos && yawErr < config.discontinuityYawRad)
		return;

	// One pending candidate per device per window.
	for (const auto &c : candidates)
		if (c.deviceId == id && !c.dead && std::abs(c.t - incoming.t) < config.window)
			return;

	Candidate c;
	c.deviceId = id;
	c.t = incoming.t;   // first post-jump sample time
	c.exact = false;
	c.ready = false;
	c.preWindow.assign(dev.hist.begin(), dev.hist.end());
	// Keep only the pre-jump window.
	while (!c.preWindow.empty() && c.t - c.preWindow.front().t > config.window)
		c.preWindow.erase(c.preWindow.begin());
	candidates.push_back(c);
}

void JumpDetector::EvaluatePendingCandidates(double now)
{
	for (auto &c : candidates)
	{
		if (c.ready || c.dead || c.exact)
			continue;
		if (now < c.t + config.window)
			continue;   // post window still filling

		const auto &dev = devices[c.deviceId];

		// Estimate the state at the jump instant from each side by regressing
		// the window against time (position per axis, unwrapped heading), then
		// evaluating the fit at c.t. A single frame pair would bake one frame
		// of noise into the profile permanently, and extrapolating heading via
		// angular velocity is biased when the device is pitched (the twist
		// rate is not the world-Y angular velocity component).
		struct WindowFit
		{
			int n = 0;
			double sumT = 0, sumTT = 0;
			Eigen::Vector3d sumP = Eigen::Vector3d::Zero(), sumTP = Eigen::Vector3d::Zero();
			double sumY = 0, sumTY = 0;
			double prevYaw = 0;

			void Add(const Hist &h, double t0)
			{
				double dt = h.t - t0;
				double yaw = n == 0 ? h.yaw : prevYaw + WrapAngle(h.yaw - prevYaw);   // unwrap
				prevYaw = yaw;
				sumT += dt; sumTT += dt * dt;
				sumP += h.pos; sumTP += dt * h.pos;
				sumY += yaw; sumTY += dt * yaw;
				n++;
			}

			// Evaluate the least-squares line at dt = 0 (the jump instant).
			bool At(Eigen::Vector3d &pos, double &yaw) const
			{
				if (n < 3)
					return false;
				double denom = n * sumTT - sumT * sumT;
				if (std::abs(denom) < 1e-12)
					return false;
				Eigen::Vector3d slopeP = (n * sumTP - sumT * sumP) / denom;
				double slopeY = (n * sumTY - sumT * sumY) / denom;
				pos = (sumP - slopeP * sumT) / n;
				yaw = (sumY - slopeY * sumT) / n;
				return true;
			}
		};

		WindowFit pre, post;
		for (const auto &h : c.preWindow)
			pre.Add(h, c.t);
		for (const auto &h : dev.hist)
		{
			if (h.t < c.t || h.t > c.t + config.window)
				continue;
			post.Add(h, c.t);
		}

		Eigen::Vector3d prePos, postPos;
		double preYaw, postYaw;
		if (!pre.At(prePos, preYaw) || !post.At(postPos, postYaw))
		{
			c.dead = true;
			continue;
		}

		double dYaw = WrapAngle(postYaw - preYaw);
		c.rot = YawQuat(dYaw);
		c.trans = postPos - c.rot * prePos;
		c.ready = true;

		notes.push_back(Format("pose discontinuity on device %u: yaw %+.2f deg, shift %.3f m",
			c.deviceId, dYaw * 180.0 / EIGEN_PI, c.trans.norm()));
	}
}

void JumpDetector::TryAccept(double now)
{
	// Hysteresis gates re-triggering only: candidates born inside the hold
	// window after an accepted jump are echoes of the same event.
	if (now - lastAcceptTime < config.retriggerHold)
	{
		candidates.clear();
		return;
	}

	// Exact path: a worldFromDriver rebase on the HMD is authoritative (the
	// HMD defines the reference universe). Agreement from other devices is a
	// sanity check, not a requirement.
	for (auto &c : candidates)
	{
		if (!c.exact || c.dead || c.deviceId != vr::k_unTrackedDeviceIndex_Hmd)
			continue;

		UniverseDelta d;
		d.rotation = c.rot;
		d.translation = c.trans;
		d.time = c.t;
		d.exact = true;
		d.devicesAgreeing = 1;
		d.residualTiltRad = c.residualTiltRad;

		for (const auto &o : candidates)
		{
			if (&o == &c || !o.exact || o.dead || std::abs(o.t - c.t) > config.agreeWindow)
				continue;
			double spread = (o.trans - c.trans).norm();
			d.residualSpread = std::max(d.residualSpread, spread);
			if (spread < config.agreePosTol)
				d.devicesAgreeing++;
		}

		accepted.push_back(d);
		lastAcceptTime = now;
		candidates.clear();
		return;
	}

	// Heuristic path: windowed multi-device agreement.
	for (auto &c0 : candidates)
	{
		if (!c0.ready || c0.dead || c0.exact)
			continue;

		std::vector<const Candidate *> agree;
		double spread = 0.0;
		for (const auto &o : candidates)
		{
			if (!o.ready || o.dead || o.exact || std::abs(o.t - c0.t) > config.agreeWindow)
				continue;
			double dPos = (o.trans - c0.trans).norm();
			double dYaw = std::abs(WrapAngle(YawOf(o.rot) - YawOf(c0.rot)));
			if (dPos < config.agreePosTol && dYaw < config.agreeYawTolRad)
			{
				agree.push_back(&o);
				spread = std::max(spread, dPos);
			}
		}

		bool solo = ActiveDeviceCount(now) <= 1;
		bool bigSolo = solo &&
			(c0.trans.norm() > config.soloPosThreshold ||
			 std::abs(YawOf(c0.rot)) > config.soloYawThresholdRad);

		if (agree.size() >= 2 || bigSolo)
		{
			UniverseDelta d;
			d.time = c0.t;
			d.exact = false;
			d.devicesAgreeing = static_cast<int>(agree.size());
			d.residualSpread = spread;

			Eigen::Vector3d trans = Eigen::Vector3d::Zero();
			double s = 0.0, cs = 0.0;
			for (const auto *a : agree)
			{
				trans += a->trans;
				double y = YawOf(a->rot);
				s += std::sin(y);
				cs += std::cos(y);
			}
			d.translation = trans / static_cast<double>(agree.size());
			d.rotation = YawQuat(std::atan2(s, cs));

			accepted.push_back(d);
			lastAcceptTime = now;
			candidates.clear();
			return;
		}

		if (now > c0.t + config.agreeWindow && !bigSolo && agree.size() < 2)
		{
			c0.dead = true;
			notes.push_back(Format("unconfirmed pose discontinuity on device %u ignored", c0.deviceId));
		}
	}

	// Prune stale entries.
	candidates.erase(
		std::remove_if(candidates.begin(), candidates.end(),
			[&](const Candidate &c) { return c.dead || now - c.t > 5.0; }),
		candidates.end());
}

int JumpDetector::ActiveDeviceCount(double now) const
{
	int n = 0;
	for (const auto &dev : devices)
		if (dev.lastValidTime > now - 1.0)
			n++;
	return n;
}

bool JumpDetector::PollDelta(UniverseDelta &out)
{
	if (accepted.empty())
		return false;
	out = accepted.front();
	accepted.pop_front();
	return true;
}

bool JumpDetector::PollGap(GapEvent &out)
{
	if (gaps.empty())
		return false;
	out = gaps.front();
	gaps.pop_front();
	return true;
}

bool JumpDetector::PollNote(std::string &out)
{
	if (notes.empty())
		return false;
	out = notes.front();
	notes.pop_front();
	return true;
}

void JumpDetector::Reset()
{
	for (auto &dev : devices)
	{
		dev.wfdValid = false;
		dev.lastValidTime = -1.0;
		dev.hist.clear();
	}
	candidates.clear();
	accepted.clear();
	gaps.clear();
	notes.clear();
	lastAcceptTime = -1e9;
}
