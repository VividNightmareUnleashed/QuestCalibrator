// PCH-free on purpose: SolverTests compiles this file standalone.
#include "JumpDetector.h"

// For the shared yaw projection only. Included after JumpDetector.h so the
// OpenVR header this TU already resolved (openvr_driver.h, via Protocol.h) is
// the one ChaperoneMath.h sees.
#include "ChaperoneMath.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstdarg>
#include <cstdio>

// vr:: constants come from whichever OpenVR header Protocol.h resolved
// (openvr.h when it is already loaded, openvr_driver.h otherwise) — both
// define the ones used here.

namespace
{

// Heading component of a rotation: the scalar form of the shared yaw
// projection. The heuristic path regresses an ANGLE against time (a quaternion
// cannot be least-squares fitted), so it needs the scalar; the exact path needs
// the quaternion. Both come off questcal::YawOnlyRotation so there is one
// projection convention, not two — see the comment there.
double YawOf(const Eigen::Quaterniond &q)
{
	Eigen::Quaterniond yaw = questcal::YawOnlyRotation(q);
	return 2.0 * std::atan2(yaw.y(), yaw.w());
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

// Least-squares line fit over one side of a candidate: position per axis and
// unwrapped heading against time, evaluated at the jump instant. A single
// frame pair would bake one frame of noise into the profile permanently, and
// extrapolating heading via angular velocity is biased when the device is
// pitched (the twist rate is not the world-Y angular velocity component).
// File scope rather than inside the candidate loop: it is the one piece of the
// heuristic path with self-contained, checkable behaviour.
struct WindowFit
{
	int n = 0;
	double sumT = 0, sumTT = 0;
	Eigen::Vector3d sumP = Eigen::Vector3d::Zero(), sumTP = Eigen::Vector3d::Zero();
	double sumY = 0, sumTY = 0;
	double prevYaw = 0;

	void Add(double t, const Eigen::Vector3d &pos, double heading, double t0)
	{
		double dt = t - t0;
		double yaw = n == 0 ? heading : prevYaw + WrapAngle(heading - prevYaw);   // unwrap
		prevYaw = yaw;
		sumT += dt; sumTT += dt * dt;
		sumP += pos; sumTP += dt * pos;
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
	if (s.deviceId >= vr::k_unMaxTrackedDeviceCount)
		return;

	auto &dev = devices[s.deviceId];
	auto breakObservationContinuity = [&]()
	{
		// An explicitly observed bad frame is stronger evidence than mere
		// absence. Neither the exact WFD path nor the composed-pose heuristic may
		// bridge it, even when the valid endpoints are less than gapSeconds apart.
		dev.wfdValid = false;
		dev.hist.clear();
		for (auto &candidate : candidates)
			if (candidate.deviceId == s.deviceId && candidate.life == Life::Pending)
				candidate.life = Life::Dead;
	};

	bool valid = s.poseIsValid && s.trackingResult == static_cast<uint32_t>(vr::TrackingResult_Running_OK);
	if (!valid || !IsUsableRingSample(s, qpcToSeconds))
	{
		breakObservationContinuity();
		return;   // long gaps are still measured from the last valid sample
	}

	double t = RingSampleTime(s, qpcToSeconds);
	if (dev.lastValidTime >= 0.0 && t <= dev.lastValidTime)
	{
		// A composed-time inversion is a rejected observation too. Clear both
		// continuity paths before accepting a later monotonic sample.
		breakObservationContinuity();
		return;
	}

	// A hard gap in the reference stream (disconnect, standby): the universe
	// may have moved with no observable frame pair. Never compensate across
	// it — reset baselines and report the event for staleness scoring.
	if (dev.lastValidTime >= 0.0 && t - dev.lastValidTime > config.gapSeconds)
	{
		gaps.push_back({ s.deviceId, t - dev.lastValidTime, t });
		// Same three actions as an observed bad frame, and they must stay the
		// same three: a gap must never be bridged by driver-local state the
		// bad-frame path correctly refuses to bridge. Deliberately no return —
		// this sample is still processed.
		breakObservationContinuity();
	}

	RingSampleParts p = UnpackRingSample(s);
	Eigen::Vector3d driverVelocity(s.velocity[0], s.velocity[1], s.velocity[2]);
	Eigen::Vector3d driverAngularVelocity(
		s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]);

	Hist h;
	h.t = t;
	h.pos = p.wfdRot * p.drvPos + p.wfdTrans;
	h.vel = p.wfdRot * driverVelocity;
	Eigen::Vector3d worldAngVel = p.wfdRot * driverAngularVelocity;
	h.yaw = YawOf(p.wfdRot * p.drvRot);
	h.yawRate = worldAngVel.y();

	bool rebased = false;
	if (dev.wfdValid)
	{
		double dRot = p.wfdRot.angularDistance(dev.wfdRot);
		double dTrans = (p.wfdTrans - dev.wfdTrans).norm();
		if (dRot > config.wfdRotEpsRad || dTrans > config.wfdTransEps)
		{
			// A WFD change is exact only when the driver-local pose remained on
			// its predicted trajectory. Some drivers can instead change WFD and
			// inversely re-express the local pose, leaving the composed world pose
			// continuous; compensating that bookkeeping change would create a
			// jump that never happened. Ambiguous changes fall through to the
			// composed-pose heuristic below.
			ringpose::DriverLocalPoseSample previous;
			previous.time = dev.lastValidTime;
			previous.rotation = dev.drvRot;
			previous.position = dev.drvPos;
			previous.velocity = dev.drvVel;
			previous.angularVelocity = dev.drvAngVel;
			ringpose::DriverLocalPoseSample current;
			current.time = t;
			current.rotation = p.drvRot;
			current.position = p.drvPos;
			current.velocity = driverVelocity;
			current.angularVelocity = driverAngularVelocity;
			bool localContinuous = ringpose::IsDriverLocalPoseContinuous(
				previous, current, config.maxFrameGap,
				config.localContinuityPos, config.localContinuityRotRad);

			if (localContinuous)
			{
				DetectWfdRebase(s.deviceId, dev, t, p.wfdRot, p.wfdTrans);
				rebased = true;
			}
			else
			{
				notes.push_back(Format(
					"worldFromDriver changed on device %u with discontinuous local pose; exact compensation ignored",
					s.deviceId));
			}
		}
	}

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
	dev.drvRot = p.drvRot;
	dev.drvPos = p.drvPos;
	dev.drvVel = driverVelocity;
	dev.drvAngVel = driverAngularVelocity;
	dev.wfdValid = true;
	dev.lastValidTime = t;

	EvaluatePendingCandidates();
	TryAccept();
}

void JumpDetector::DetectWfdRebase(uint32_t id, DeviceState &dev, double t,
                                   const Eigen::Quaterniond &newRot, const Eigen::Vector3d &newTrans)
{
	// Exact world delta: worlds relate by D = new_wfd ∘ old_wfd⁻¹.
	Eigen::Quaterniond dRot = (newRot * dev.wfdRot.conjugate()).normalized();

	// Constrain to yaw + translation (a recenter preserves gravity), through
	// the projection the chaperone re-anchor path shares: invariants 14/15
	// require the delta applied here and the standing-center re-anchor derived
	// in ChaperoneMath to be the same transform. The discarded swing is the
	// non-rigid residual, reported and never applied.
	double tilt = 0.0;
	Eigen::Quaterniond yawRot = questcal::YawOnlyRotation(dRot, &tilt);
	double yaw = YawOf(yawRot);

	// Compute the translation with the projected rotation so D stays
	// self-consistent with the yaw actually applied.
	Eigen::Vector3d dTrans = newTrans - yawRot * dev.wfdTrans;

	Candidate c;
	c.deviceId = id;
	c.t = t;
	c.kind = Kind::Exact;
	c.life = Life::Ready;   // the exact path has its delta immediately
	c.rot = yawRot;
	c.trans = dTrans;
	c.residualTiltRad = tilt;
	c.endpoint = ExactEndpoint{ newRot, newTrans };
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
		if (c.deviceId == id && c.life != Life::Dead &&
			std::abs(c.t - incoming.t) < config.window)
			return;

	Candidate c;
	c.deviceId = id;
	c.t = incoming.t;   // first post-jump sample time
	c.kind = Kind::Heuristic;
	c.life = Life::Pending;
	// Keep only the pre-jump window, in order: find where it starts and copy
	// the suffix once (erasing the expired head one element at a time
	// relocated the remainder on every step).
	size_t first = 0;
	while (first < dev.hist.size() && c.t - dev.hist[first].t > config.window)
		++first;
	c.preWindow.assign(dev.hist.begin() + first, dev.hist.end());
	candidates.push_back(c);
}

void JumpDetector::EvaluatePendingCandidates()
{
	for (auto &c : candidates)
	{
		if (!c.PendingHeuristic())
			continue;
		double now = devices[c.deviceId].lastValidTime;
		if (now < c.t + config.window)
			continue;   // post window still filling

		const auto &dev = devices[c.deviceId];

		// Estimate the state at the jump instant from each side (see WindowFit).
		WindowFit pre, post;
		for (const auto &h : c.preWindow)
			pre.Add(h.t, h.pos, h.yaw, c.t);
		for (const auto &h : dev.hist)
		{
			if (h.t < c.t || h.t > c.t + config.window)
				continue;
			post.Add(h.t, h.pos, h.yaw, c.t);
		}

		Eigen::Vector3d prePos, postPos;
		double preYaw, postYaw;
		if (!pre.At(prePos, preYaw) || !post.At(postPos, postYaw))
		{
			c.life = Life::Dead;
			continue;
		}

		double dYaw = WrapAngle(postYaw - preYaw);
		c.rot = YawQuat(dYaw);
		c.trans = postPos - c.rot * prePos;
		c.life = Life::Ready;

		notes.push_back(Format("pose discontinuity on device %u: yaw %+.2f deg, shift %.3f m",
			c.deviceId, dYaw * 180.0 / EIGEN_PI, c.trans.norm()));
	}
}

void JumpDetector::TryAccept()
{
	// Exact path: a worldFromDriver rebase on the HMD is authoritative (the
	// HMD defines the reference universe). Agreement from other devices is a
	// sanity check, not a requirement. Evaluate it before heuristic hysteresis:
	// two distinct WFD transitions telescope even inside retriggerHold, so
	// dropping the second would leave the calibration one rebase behind.
	for (auto &c : candidates)
	{
		if (!c.LiveExact() || c.deviceId != vr::k_unTrackedDeviceIndex_Hmd)
			continue;

		UniverseDelta d;
		d.rotation = c.rot;
		d.translation = c.trans;
		d.time = c.t;
		d.exact = true;
		d.devicesAgreeing = 1;
		d.residualTiltRad = c.residualTiltRad;
		d.worldFromDriverRotation = c.endpoint.rotation;
		d.worldFromDriverTranslation = c.endpoint.translation;
		std::bitset<vr::k_unMaxTrackedDeviceCount> counted;
		counted.set(c.deviceId);

		for (const auto &o : candidates)
		{
			if (&o == &c || !o.LiveExact() || std::abs(o.t - c.t) > config.agreeWindow)
				continue;
			double spread = (o.trans - c.trans).norm();
			d.residualSpread = std::max(d.residualSpread, spread);
			if (spread < config.agreePosTol && !counted.test(o.deviceId))
			{
				counted.set(o.deviceId);
				d.devicesAgreeing++;
			}
		}

		accepted.push_back(d);
		lastAcceptTime = c.t;
		candidates.clear();
		return;
	}

	// Hysteresis suppresses heuristic echoes only. Non-HMD exact candidates
	// inside this window describe the already-applied HMD event and can go too.
	// Heuristic path: windowed multi-device agreement.
	for (auto &c0 : candidates)
	{
		if (!c0.ReadyHeuristic())
			continue;
		double now = devices[c0.deviceId].lastValidTime;
		if (c0.t - lastAcceptTime < config.retriggerHold)
		{
			c0.life = Life::Dead;
			continue;
		}

		// Candidates from one device can coexist across adjacent fit windows.
		// Each device gets one vote and one contribution to the averaged delta.
		std::vector<const Candidate *> agree{ &c0 };
		std::bitset<vr::k_unMaxTrackedDeviceCount> counted;
		counted.set(c0.deviceId);
		double spread = 0.0;
		for (const auto &o : candidates)
		{
			if (!o.ReadyHeuristic() || counted.test(o.deviceId) ||
				std::abs(o.t - c0.t) > config.agreeWindow)
				continue;
			double dPos = (o.trans - c0.trans).norm();
			double dYaw = std::abs(WrapAngle(YawOf(o.rot) - YawOf(c0.rot)));
			if (dPos < config.agreePosTol && dYaw < config.agreeYawTolRad)
			{
				counted.set(o.deviceId);
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
			lastAcceptTime = c0.t;
			candidates.clear();
			return;
		}

		if (now > c0.t + config.window + config.agreeWindow &&
			!bigSolo && agree.size() < 2)
		{
			c0.life = Life::Dead;
			notes.push_back(Format("unconfirmed pose discontinuity on device %u ignored", c0.deviceId));
		}
	}

	// Prune stale entries.
	candidates.erase(
		std::remove_if(candidates.begin(), candidates.end(),
			[&](const Candidate &c) {
				return c.life == Life::Dead ||
					devices[c.deviceId].lastValidTime - c.t > 5.0;
			}),
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
