// Synthetic-data harness for the calibration engine.
//
// Generates two device trajectories with a known ground-truth universe
// transform, mount offset, per-system sample rates, latency, noise, and
// outliers, then asserts the solver recovers the transform within tolerance.
// Exit code is the number of failed scenarios.

#include "../Driver/AlignmentField.h"
#include "../Overlay/CalibrationEngine.h"
#include "../Overlay/ChaperoneMath.h"
#include "../Overlay/ContinuousAlignment.h"
#include "../Overlay/FieldMath.h"
#include "../Overlay/DriftMonitor.h"
#include "../Overlay/JumpDetector.h"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace questcal;

namespace
{

struct GroundTruth
{
	Eigen::Quaterniond rotation;    // maps target universe into reference universe
	Eigen::Vector3d translation;
	double scale = 1.0;
	double latency = 0.0;           // seconds the target stream lags the reference
};

struct SceneConfig
{
	double duration = 25.0;
	double refRate = 90.0;
	double targetRate = 72.0;
	double posNoise = 0.0;          // meters, 1 sigma per axis
	double rotNoiseDeg = 0.0;       // degrees, 1 sigma
	double outlierRate = 0.0;       // probability per sample of a 0.5 m glitch
	bool   yawOnlyMotion = false;
};

// Smooth, rich test motion: two-axis rotation plus a figure-eight translation,
// the sort of wiggling a user actually does while calibrating.
Eigen::Quaterniond RotationAt(double t, bool yawOnly)
{
	Eigen::AngleAxisd yaw(1.1 * std::sin(0.9 * t) + 0.25 * t, Eigen::Vector3d::UnitY());
	if (yawOnly)
		return Eigen::Quaterniond(yaw);
	Eigen::AngleAxisd pitch(0.9 * std::sin(1.35 * t + 0.7), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd roll(0.5 * std::sin(1.7 * t + 2.1), Eigen::Vector3d::UnitZ());
	return Eigen::Quaterniond(yaw) * pitch * roll;
}

Eigen::Vector3d PositionAt(double t)
{
	return Eigen::Vector3d(
		0.35 * std::sin(0.8 * t),
		1.25 + 0.15 * std::sin(1.4 * t + 0.4),
		0.30 * std::sin(1.6 * t));
}

// Pose of the reference device (device A) in the reference universe at time t.
void DevicePoseAt(double t, bool yawOnly, Eigen::Quaterniond &rot, Eigen::Vector3d &pos)
{
	rot = RotationAt(t, yawOnly);
	pos = PositionAt(t);
}

PoseSample MakeSample(double stamp, double poseTime, const SceneConfig &scene,
                      const GroundTruth &truth, bool isTarget,
                      const Eigen::Quaterniond &mountRot, const Eigen::Vector3d &mountPos,
                      std::mt19937 &rng)
{
	Eigen::Quaterniond rotA;
	Eigen::Vector3d posA;
	DevicePoseAt(poseTime, scene.yawOnlyMotion, rotA, posA);

	Eigen::Quaterniond rot;
	Eigen::Vector3d pos;
	if (!isTarget)
	{
		rot = rotA;
		pos = posA;
	}
	else
	{
		// Device B is rigidly mounted on A, then mapped into the target universe
		// by the inverse of the ground-truth calibration:
		//   worldRef = R * (s * pTarget) + t   =>   pTarget = R^-1 (worldRef - t) / s
		Eigen::Quaterniond rotB = rotA * mountRot;
		Eigen::Vector3d posB = posA + rotA * mountPos;
		rot = truth.rotation.conjugate() * rotB;
		pos = truth.rotation.conjugate() * (posB - truth.translation) / truth.scale;
	}

	// Velocities via central finite differences of the same construction.
	const double h = 1e-4;
	auto poseAt = [&](double tt, Eigen::Quaterniond &r, Eigen::Vector3d &p)
	{
		Eigen::Quaterniond ra;
		Eigen::Vector3d pa;
		DevicePoseAt(tt, scene.yawOnlyMotion, ra, pa);
		if (!isTarget) { r = ra; p = pa; return; }
		Eigen::Quaterniond rb = ra * mountRot;
		Eigen::Vector3d pb = pa + ra * mountPos;
		r = truth.rotation.conjugate() * rb;
		p = truth.rotation.conjugate() * (pb - truth.translation) / truth.scale;
	};
	Eigen::Quaterniond r0, r1;
	Eigen::Vector3d p0, p1;
	poseAt(poseTime - h, r0, p0);
	poseAt(poseTime + h, r1, p1);

	PoseSample s;
	s.time = stamp;
	s.rot = rot;
	s.pos = pos;
	s.vel = (p1 - p0) / (2.0 * h);
	Eigen::Quaterniond dq = r1 * r0.conjugate();
	dq.normalize();
	if (dq.w() < 0.0) dq.coeffs() = -dq.coeffs();
	double angle = 2.0 * std::atan2(dq.vec().norm(), dq.w());
	s.angVel = (dq.vec().norm() > 1e-12)
		? Eigen::Vector3d(dq.vec().normalized() * (angle / (2.0 * h)))
		: Eigen::Vector3d::Zero();

	// Noise and outliers.
	std::normal_distribution<double> pn(0.0, scene.posNoise);
	std::normal_distribution<double> rn(0.0, scene.rotNoiseDeg * EIGEN_PI / 180.0);
	std::uniform_real_distribution<double> u(0.0, 1.0);
	std::uniform_real_distribution<double> axisPick(-1.0, 1.0);

	if (scene.posNoise > 0.0)
		s.pos += Eigen::Vector3d(pn(rng), pn(rng), pn(rng));
	if (scene.rotNoiseDeg > 0.0)
	{
		Eigen::Vector3d axis(axisPick(rng), axisPick(rng), axisPick(rng));
		if (axis.norm() > 1e-6)
			s.rot = Eigen::Quaterniond(Eigen::AngleAxisd(rn(rng), axis.normalized())) * s.rot;
	}
	if (scene.outlierRate > 0.0 && u(rng) < scene.outlierRate)
		s.pos += Eigen::Vector3d(0.5 * axisPick(rng), 0.5 * axisPick(rng), 0.5 * axisPick(rng));

	return s;
}

void GenerateStreams(const SceneConfig &scene, const GroundTruth &truth, uint32_t seed,
                     std::vector<PoseSample> &refStream, std::vector<PoseSample> &targetStream)
{
	std::mt19937 rng(seed);
	Eigen::Quaterniond mountRot(Eigen::AngleAxisd(2.7, Eigen::Vector3d(0.2, 0.7, -0.3).normalized()));
	Eigen::Vector3d mountPos(0.05, -0.08, 0.03);   // ~10 cm mount offset

	refStream.clear();
	targetStream.clear();

	for (double t = 0.0; t < scene.duration; t += 1.0 / scene.refRate)
		refStream.push_back(MakeSample(t, t, scene, truth, false, mountRot, mountPos, rng));

	// The target stream lags: the pose stamped t is the physical state at t - latency.
	for (double t = 0.0; t < scene.duration; t += 1.0 / scene.targetRate)
		targetStream.push_back(MakeSample(t, t - truth.latency, scene, truth, true, mountRot, mountPos, rng));
}

struct Expectation
{
	bool expectValid = true;
	double maxRotErrDeg = 0.1;
	double maxTransErrM = 0.005;
	double maxOffsetErr = 0.004;       // only checked when latency != 0
	double maxScaleErr = 0.0;          // only checked when > 0
	const char *messageContains = nullptr;
};

int failures = 0;

void RunScenario(const char *name, const SceneConfig &scene, const GroundTruth &truth,
                 const EngineConfig &config, const Expectation &expect, uint32_t seed = 1234)
{
	std::vector<PoseSample> refStream, targetStream;
	GenerateStreams(scene, truth, seed, refStream, targetStream);

	EngineResult r = CalibrationEngine::Solve(refStream, targetStream, config);

	double rotErr = truth.rotation.angularDistance(r.rotation) * 180.0 / EIGEN_PI;
	double transErr = (r.translation - truth.translation).norm();
	double offsetErr = std::abs(r.timeOffset - truth.latency);
	double scaleErr = std::abs(r.scale - truth.scale);

	bool pass = true;
	std::string why;

	if (r.valid != expect.expectValid)
	{
		pass = false;
		why += " valid=" + std::string(r.valid ? "true" : "false") +
		       " (expected " + (expect.expectValid ? "true" : "false") + ")";
	}
	if (expect.expectValid && r.valid)
	{
		if (rotErr > expect.maxRotErrDeg) { pass = false; why += " rotErr"; }
		if (transErr > expect.maxTransErrM) { pass = false; why += " transErr"; }
		if (truth.latency != 0.0 && config.estimateTimeOffset && offsetErr > expect.maxOffsetErr)
		{
			pass = false; why += " offsetErr";
		}
		if (expect.maxScaleErr > 0.0 && scaleErr > expect.maxScaleErr) { pass = false; why += " scaleErr"; }
	}
	if (expect.messageContains && r.message.find(expect.messageContains) == std::string::npos)
	{
		pass = false;
		why += " message=\"" + r.message + "\"";
	}

	printf("%-28s %s  rot %.4f deg  trans %.4f m  offset %+.1f ms (true %+.1f)  scale %.4f  spread %.4f  pairs %zu%s%s\n",
		name, pass ? "PASS" : "FAIL",
		rotErr, transErr, r.timeOffset * 1000.0, truth.latency * 1000.0,
		r.scale, r.axisSpread, r.pairsUsed,
		why.empty() ? "" : "  <-", why.c_str());

	if (!pass)
	{
		printf("%-28s      message: %s\n", "", r.message.c_str());
		failures++;
	}
}

// ---------------------------------------------------------------------------
// Universe-jump detection scenarios

// Ring sample on a synthetic 10 MHz QPC clock.
const double TestQpcToSeconds = 1e-7;

protocol::DevicePoseSample RingSample(uint32_t id, double t,
                                      const Eigen::Quaterniond &wfdRot, const Eigen::Vector3d &wfdTrans,
                                      const Eigen::Quaterniond &drvRot, const Eigen::Vector3d &drvPos,
                                      const Eigen::Vector3d &drvVel, const Eigen::Vector3d &drvAngVel)
{
	protocol::DevicePoseSample s;
	s.sampleTimeQpc = static_cast<int64_t>(t / TestQpcToSeconds + 0.5);
	s.deviceId = id;
	s.trackingResult = static_cast<uint32_t>(vr::TrackingResult_Running_OK);
	s.poseIsValid = true;
	s.deviceIsConnected = true;
	s.poseTimeOffset = 0.0;
	s.worldFromDriverRotation = { wfdRot.w(), wfdRot.x(), wfdRot.y(), wfdRot.z() };
	s.rotation = { drvRot.w(), drvRot.x(), drvRot.y(), drvRot.z() };
	for (int i = 0; i < 3; ++i)
	{
		s.worldFromDriverTranslation[i] = wfdTrans(i);
		s.position[i] = drvPos(i);
		s.velocity[i] = drvVel(i);
		s.angularVelocity[i] = drvAngVel(i);
	}
	return s;
}

// World trajectory of a reference device (HMD or its controller), with
// consistent finite-difference velocities.
void RefTrajectory(double t, uint32_t id, Eigen::Quaterniond &rot, Eigen::Vector3d &pos,
                   Eigen::Vector3d &vel, Eigen::Vector3d &angVel)
{
	auto poseOf = [id](double tt, Eigen::Quaterniond &r, Eigen::Vector3d &p)
	{
		r = RotationAt(tt, false);
		p = PositionAt(tt);
		if (id != 0)
			p += r * Eigen::Vector3d(0.1, -0.3, -0.25);   // hand-ish offset from the head
	};

	poseOf(t, rot, pos);

	const double h = 1e-4;
	Eigen::Quaterniond r0, r1;
	Eigen::Vector3d p0, p1;
	poseOf(t - h, r0, p0);
	poseOf(t + h, r1, p1);
	vel = (p1 - p0) / (2.0 * h);
	Eigen::Quaterniond dq = r1 * r0.conjugate();
	dq.normalize();
	if (dq.w() < 0.0) dq.coeffs() = -dq.coeffs();
	double angle = 2.0 * std::atan2(dq.vec().norm(), dq.w());
	angVel = (dq.vec().norm() > 1e-12)
		? Eigen::Vector3d(dq.vec().normalized() * (angle / (2.0 * h)))
		: Eigen::Vector3d::Zero();
}

void Check(const char *name, bool pass, const char *detail)
{
	printf("%-28s %s%s%s\n", name, pass ? "PASS" : "FAIL", detail[0] ? "  " : "", detail);
	if (!pass)
		failures++;
}

// Re-express a world state after the universe re-bases by (R, T).
void ApplyUniverse(const Eigen::Quaterniond &R, const Eigen::Vector3d &T,
                   Eigen::Quaterniond &rot, Eigen::Vector3d &pos,
                   Eigen::Vector3d &vel, Eigen::Vector3d &angVel)
{
	rot = R * rot;
	pos = R * pos + T;
	vel = R * vel;
	angVel = R * angVel;
}

struct JumpRun
{
	int deltas = 0;
	JumpDetector::UniverseDelta last;

	double YawErrDeg(double trueYaw) const
	{
		return std::abs(2.0 * std::atan2(last.rotation.y(), last.rotation.w()) - trueYaw) * 180.0 / EIGEN_PI;
	}
	double TransErr(const Eigen::Vector3d &trueT) const
	{
		return (last.translation - trueT).norm();
	}
};

// Drive both reference devices through 3 s of trajectory, pushing whatever
// sample `make(t, id)` produces and collecting accepted deltas.
template <typename MakeFn>
JumpRun DriveJump(JumpDetector &jd, double rate, MakeFn make)
{
	JumpRun r;
	JumpDetector::UniverseDelta d;
	for (double t = 0.0; t < 3.0; t += 1.0 / rate)
	{
		for (uint32_t id = 0; id <= 1; ++id)
			jd.Push(make(t, id));
		while (jd.PollDelta(d)) { r.deltas++; r.last = d; }
	}
	return r;
}

void RunJumpScenarios()
{
	const double jumpYaw = 25.0 * EIGEN_PI / 180.0;
	const Eigen::Quaterniond D_R(Eigen::AngleAxisd(jumpYaw, Eigen::Vector3d::UnitY()));
	const Eigen::Vector3d D_T(0.4, 0.0, -0.3);
	const double tJump = 1.5;
	const double rate = 90.0;
	char detail[256];

	// A. worldFromDriver rebase: driver-local pose continuous, wfd carries the
	// jump. Exactly one delta, exact, recovered near-exactly, no double-apply
	// from the second device.
	{
		JumpDetector jd(TestQpcToSeconds);
		JumpRun r = DriveJump(jd, rate, [&](double t, uint32_t id)
		{
			Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
			RefTrajectory(t, id, rot, pos, vel, angVel);
			bool after = t >= tJump;
			// wfd rebase: driver pose IS the trajectory; wfd jumps.
			return RingSample(id, t,
				after ? D_R : Eigen::Quaterniond::Identity(),
				after ? D_T : Eigen::Vector3d::Zero(),
				rot, pos, vel, angVel);
		});

		bool pass = r.deltas == 1 && r.last.exact && r.YawErrDeg(jumpYaw) < 0.1 && r.TransErr(D_T) < 0.01;
		snprintf(detail, sizeof detail, "deltas %d  exact %d  yawErr %.3f deg  transErr %.4f m",
			r.deltas, r.last.exact ? 1 : 0, r.YawErrDeg(jumpYaw), r.TransErr(D_T));
		Check("jump: wfd rebase", pass, detail);
	}

	// B. Raw-pose jump under noise: wfd static, the driver pose stream itself
	// re-bases. Heuristic path, windowed estimation, two-device agreement.
	{
		JumpDetector jd(TestQpcToSeconds);
		std::mt19937 rng(99);
		std::normal_distribution<double> noise(0.0, 0.002);
		JumpRun r = DriveJump(jd, rate, [&](double t, uint32_t id)
		{
			Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
			RefTrajectory(t, id, rot, pos, vel, angVel);
			if (t >= tJump)
				ApplyUniverse(D_R, D_T, rot, pos, vel, angVel);
			pos += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
			return RingSample(id, t, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
				rot, pos, vel, angVel);
		});

		bool pass = r.deltas == 1 && !r.last.exact && r.last.devicesAgreeing >= 2 &&
			r.YawErrDeg(jumpYaw) < 0.5 && r.TransErr(D_T) < 0.02;
		snprintf(detail, sizeof detail, "deltas %d  agree %d  yawErr %.3f deg  transErr %.4f m",
			r.deltas, r.last.devicesAgreeing, r.YawErrDeg(jumpYaw), r.TransErr(D_T));
		Check("jump: raw-pose (windowed)", pass, detail);
	}

	// C. Fast continuous motion, no jump: no false positives.
	{
		JumpDetector jd(TestQpcToSeconds);
		JumpRun r = DriveJump(jd, rate, [&](double t, uint32_t id)
		{
			Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
			RefTrajectory(3.0 * t, id, rot, pos, vel, angVel);   // 3x speed
			return RingSample(id, t, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
				rot, pos, vel * 3.0, angVel * 3.0);
		});

		snprintf(detail, sizeof detail, "deltas %d", r.deltas);
		Check("jump: no false positive", r.deltas == 0, detail);
	}

	// D. Hard gap in the reference stream: gap event, never a compensation.
	{
		JumpDetector jd(TestQpcToSeconds);
		int deltas = 0, gapEvents = 0;
		JumpDetector::UniverseDelta d;
		JumpDetector::GapEvent g;

		auto feed = [&](double t0, double t1, bool after)
		{
			for (double t = t0; t < t1; t += 1.0 / rate)
			{
				Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
				RefTrajectory(t, 0, rot, pos, vel, angVel);
				if (after)
					ApplyUniverse(D_R, D_T, rot, pos, vel, angVel);
				jd.Push(RingSample(0, t, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
					rot, pos, vel, angVel));
				while (jd.PollDelta(d)) deltas++;
				while (jd.PollGap(g)) gapEvents++;
			}
		};
		feed(0.0, 1.0, false);
		feed(4.0, 5.0, true);   // universe moved during the 3 s hole

		snprintf(detail, sizeof detail, "deltas %d  gaps %d", deltas, gapEvents);
		Check("jump: gap => event only", deltas == 0 && gapEvents == 1, detail);
	}
}

// ---------------------------------------------------------------------------
// Drift-monitor scenarios

void RunDriftScenarios()
{
	const double rate = 90.0;
	char detail[256];

	struct DriftRun
	{
		int slides = 0;
		int losses = 0;
		double lastMag = 0.0;
	};

	auto drain = [](DriftMonitor &dm, DriftRun &r)
	{
		DriftMonitor::Event e;
		while (dm.PollEvent(e))
		{
			if (e.type == DriftMonitor::Event::StationarySlide)
				r.slides++;
			else
				r.losses++;
			r.lastMag = e.magnitude;
		}
	};

	auto stillSample = [](double t, const Eigen::Vector3d &pos)
	{
		return RingSample(5, t, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
			Eigen::Quaterniond::Identity(), pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
	};

	const Eigen::Vector3d base(0.3, 1.0, -0.2);

	// A. Resting device on a slowly drifting universe: exactly one slide event
	// per rest episode, magnitude in the plausible band.
	{
		DriftMonitor dm(TestQpcToSeconds);
		DriftRun r;
		std::mt19937 rng(7);
		std::normal_distribution<double> n(0.0, 0.0015);
		const Eigen::Vector3d driftRate(0.0033, 0.0, 0.0011);   // ~3.5 mm/s
		for (double t = 0.0; t < 10.0; t += 1.0 / rate)
		{
			Eigen::Vector3d pos = base + driftRate * t + Eigen::Vector3d(n(rng), n(rng), n(rng));
			dm.Push(stillSample(t, pos));
			drain(dm, r);
		}
		bool pass = r.slides == 1 && r.losses == 0 && r.lastMag > 0.012 && r.lastMag < 0.04;
		snprintf(detail, sizeof detail, "slides %d  losses %d  mag %.1f mm", r.slides, r.losses, r.lastMag * 1000.0);
		Check("drift: stationary slide", pass, detail);
	}

	// B. Zero-mean sway (a person standing "still"): short-timescale energy
	// fails the chunk-jitter gate; no events.
	{
		DriftMonitor dm(TestQpcToSeconds);
		DriftRun r;
		std::mt19937 rng(8);
		std::normal_distribution<double> n(0.0, 0.0015);
		for (double t = 0.0; t < 10.0; t += 1.0 / rate)
		{
			Eigen::Vector3d sway(
				0.015 * std::sin(2.0 * EIGEN_PI * t / 1.2),
				0.0,
				0.012 * std::sin(2.0 * EIGEN_PI * t / 0.9 + 0.5));
			dm.Push(stillSample(t, base + sway + Eigen::Vector3d(n(rng), n(rng), n(rng))));
			drain(dm, r);
		}
		snprintf(detail, sizeof detail, "slides %d  losses %d", r.slides, r.losses);
		Check("drift: sway rejected", r.slides == 0 && r.losses == 0, detail);
	}

	// B2. A user coming to rest (regression for the first live session): the
	// settling tail is non-zero-mean and centimeters large, then genuine
	// stillness follows. The chunk-step gate must reject the tail and the
	// residual settle must stay under the slide threshold -- no events.
	{
		DriftMonitor dm(TestQpcToSeconds);
		DriftRun r;
		std::mt19937 rng(9);
		std::normal_distribution<double> n(0.0, 0.0015);
		const Eigen::Vector3d dir = Eigen::Vector3d(0.7, 0.2, -0.4).normalized();
		for (double t = 0.0; t < 14.0; t += 1.0 / rate)
		{
			Eigen::Vector3d pos = base + dir * (0.04 * std::exp(-t / 1.5)) + Eigen::Vector3d(n(rng), n(rng), n(rng));
			dm.Push(stillSample(t, pos));
			drain(dm, r);
		}
		snprintf(detail, sizeof detail, "slides %d  losses %d", r.slides, r.losses);
		Check("drift: settling rejected", r.slides == 0 && r.losses == 0, detail);
	}

	// C. Moving device: the rest gates must refuse to conclude anything, even
	// with real drift underneath.
	{
		DriftMonitor dm(TestQpcToSeconds);
		DriftRun r;
		const Eigen::Vector3d driftRate(0.004, 0.0, 0.0);
		for (double t = 0.0; t < 10.0; t += 1.0 / rate)
			dm.Push(stillSample(t, PositionAt(t) + driftRate * t)), drain(dm, r);
		snprintf(detail, sizeof detail, "slides %d  losses %d", r.slides, r.losses);
		Check("drift: moving ignored", r.slides == 0 && r.losses == 0, detail);
	}

	// Two still segments around a tracking hole: [0,2) at `base`, then 1.5 s
	// at `recovery` starting from `resume`.
	auto lossRun = [&](double resume, const Eigen::Vector3d &recovery)
	{
		DriftMonitor dm(TestQpcToSeconds);
		DriftRun r;
		for (double t = 0.0; t < 2.0; t += 1.0 / rate)
			dm.Push(stillSample(t, base)), drain(dm, r);
		for (double t = resume; t < resume + 1.5; t += 1.0 / rate)
			dm.Push(stillSample(t, recovery)), drain(dm, r);
		return r;
	};

	// D. Short tracking loss with a discontinuous recovery: one loss event with
	// the displacement as magnitude, and no slide from the post-gap window.
	{
		DriftRun r = lossRun(2.5, base + Eigen::Vector3d(0.4, 0.0, 0.0));
		bool pass = r.losses == 1 && r.slides == 0 && std::abs(r.lastMag - 0.4) < 0.01;
		snprintf(detail, sizeof detail, "slides %d  losses %d  mag %.2f m", r.slides, r.losses, r.lastMag);
		Check("drift: loss jump", pass, detail);
	}

	// E. Short loss, benign recovery (came back where it vanished): no events.
	{
		DriftRun r = lossRun(2.5, base);
		snprintf(detail, sizeof detail, "slides %d  losses %d", r.slides, r.losses);
		Check("drift: benign loss", r.slides == 0 && r.losses == 0, detail);
	}

	// F. Long absence (walked away / powered off): unclassifiable, no events
	// even though the device reappears far away.
	{
		DriftRun r = lossRun(4.0, base + Eigen::Vector3d(0.4, 0.0, 0.0));
		snprintf(detail, sizeof detail, "slides %d  losses %d", r.slides, r.losses);
		Check("drift: long absence", r.slides == 0 && r.losses == 0, detail);
	}
}

// ---------------------------------------------------------------------------
// Spatial-correction-field scenarios

struct FieldTransform   // p -> R p + T
{
	Eigen::Quaterniond R{ 1, 0, 0, 0 };
	Eigen::Vector3d T{ 0, 0, 0 };

	Eigen::Vector3d Apply(const Eigen::Vector3d &p) const { return R * p + T; }
};

FieldTransform Compose(const FieldTransform &a, const FieldTransform &b)   // a o b
{
	return { (a.R * b.R).normalized(), a.R * b.T + a.T };
}

// The overlay's delta derivation (SendAlignmentField's spec):
// delta_i = anchor_i o base^-1.
protocol::SetAlignmentField BuildField(const FieldTransform &base,
                                       const std::vector<FieldTransform> &anchors,
                                       const std::vector<Eigen::Vector3d> &positions,
                                       uint32_t generation)
{
	protocol::SetAlignmentField f;
	f.enabled = true;
	f.generation = generation;
	f.anchorCount = static_cast<uint32_t>(anchors.size());

	Eigen::Quaterniond baseInv = base.R.conjugate();
	for (size_t i = 0; i < anchors.size(); ++i)
	{
		Eigen::Quaterniond dR = (anchors[i].R * baseInv).normalized();
		Eigen::Vector3d dT = anchors[i].T - dR * base.T;
		f.anchors[i].rotationDelta = { dR.w(), dR.x(), dR.y(), dR.z() };
		for (int k = 0; k < 3; ++k)
		{
			f.anchors[i].position[k] = positions[i](k);
			f.anchors[i].translationDelta[k] = dT(k);
		}
	}
	return f;
}

// Independent Eigen reference of the driver's blend.
void ReferenceBlend(const protocol::SetAlignmentField &f, const Eigen::Vector3d &pos,
                    Eigen::Quaterniond &rotOut, Eigen::Vector3d &transOut)
{
	double wSum = alignfield::IdentityFloorWeight;
	Eigen::Vector4d q(alignfield::IdentityFloorWeight, 0.0, 0.0, 0.0);   // w, x, y, z
	Eigen::Vector3d t = Eigen::Vector3d::Zero();

	double sigma = f.sigmaMeters > 0.01 ? f.sigmaMeters : 1.5;
	for (uint32_t i = 0; i < f.anchorCount; ++i)
	{
		const auto &a = f.anchors[i];
		double dx = pos.x() - a.position[0];
		double dz = pos.z() - a.position[2];
		double w = std::exp(-(dx * dx + dz * dz) / (2.0 * sigma * sigma));

		Eigen::Vector4d qa(a.rotationDelta.w, a.rotationDelta.x, a.rotationDelta.y, a.rotationDelta.z);
		if (qa(0) < 0.0)
			qa = -qa;

		q += w * qa;
		t += w * Eigen::Vector3d(a.translationDelta[0], a.translationDelta[1], a.translationDelta[2]);
		wSum += w;
	}

	q.normalize();
	rotOut = Eigen::Quaterniond(q(0), q(1), q(2), q(3));
	transOut = t / wSum;
}

// The driver's effective transform at a base-calibrated position: delta o base.
FieldTransform DriverEffective(const protocol::SetAlignmentField &f, const FieldTransform &base,
                               const Eigen::Vector3d &basePos)
{
	double p[3] = { basePos.x(), basePos.y(), basePos.z() };
	vr::HmdQuaternion_t r;
	double t[3];
	alignfield::BlendAt(f, p, r, t);

	FieldTransform delta{ Eigen::Quaterniond(r.w, r.x, r.y, r.z), Eigen::Vector3d(t[0], t[1], t[2]) };
	return Compose(delta, base);
}

FieldTransform SmallDelta(double yawDeg, const Eigen::Vector3d &t)
{
	return { Eigen::Quaterniond(Eigen::AngleAxisd(yawDeg * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY())), t };
}

double Dist3(const double (&a)[3], const double (&b)[3])
{
	return std::sqrt((a[0] - b[0]) * (a[0] - b[0])
		+ (a[1] - b[1]) * (a[1] - b[1])
		+ (a[2] - b[2]) * (a[2] - b[2]));
}

void RunFieldScenarios()
{
	char detail[256];

	const FieldTransform base{
		Eigen::Quaterniond(Eigen::AngleAxisd(1.9, Eigen::Vector3d::UnitY())),
		Eigen::Vector3d(1.2, 0.03, -0.7) };

	// Room-scale anchor layout, far enough apart that cross-anchor bleed is
	// negligible next to the identity-floor undershoot.
	const std::vector<Eigen::Vector3d> positions = {
		{ -3.0, 1.2, -3.0 }, { 3.0, 1.0, 3.0 }, { -3.0, 1.5, 3.0 } };
	const std::vector<FieldTransform> anchors = {
		Compose(SmallDelta(1.0, { 0.03, 0.00, -0.01 }), base),
		Compose(SmallDelta(-0.8, { -0.02, 0.01, 0.02 }), base),
		Compose(SmallDelta(0.5, { 0.00, -0.02, 0.03 }), base) };
	const protocol::SetAlignmentField field = BuildField(base, anchors, positions, 1);

	// A. At each anchor the effective transform reproduces the absolute solve
	// for that spot, up to the identity floor's ~5% undershoot.
	{
		double worstPos = 0.0, worstRot = 0.0;
		for (size_t i = 0; i < anchors.size(); ++i)
		{
			Eigen::Vector3d targetPt = base.R.conjugate() * (positions[i] - base.T);
			FieldTransform eff = DriverEffective(field, base, positions[i]);
			double rotErrDeg = eff.R.angularDistance(anchors[i].R) * 180.0 / EIGEN_PI;
			worstPos = std::max(worstPos, (eff.Apply(targetPt) - anchors[i].Apply(targetPt)).norm());
			worstRot = std::max(worstRot, rotErrDeg);
		}
		snprintf(detail, sizeof detail, "posErr %.2f mm  rotErr %.3f deg", worstPos * 1000.0, worstRot);
		Check("field: anchor recovery", worstPos < 0.008 && worstRot < 0.15, detail);
	}

	// B. Far from every anchor the correction fades to the base calibration.
	{
		Eigen::Vector3d farPos(30.0, 1.0, -25.0);
		Eigen::Vector3d targetPt = base.R.conjugate() * (farPos - base.T);
		FieldTransform eff = DriverEffective(field, base, farPos);
		double posErr = (eff.Apply(targetPt) - base.Apply(targetPt)).norm();
		double rotErr = eff.R.angularDistance(base.R) * 180.0 / EIGEN_PI;
		snprintf(detail, sizeof detail, "posErr %.4f mm  rotErr %.5f deg", posErr * 1000.0, rotErr);
		Check("field: identity fade", posErr < 0.001 && rotErr < 0.01, detail);
	}

	// C. q and -q are the same rotation: negating a stored anchor quaternion
	// must not change the blend.
	{
		protocol::SetAlignmentField negated = field;
		auto &q = negated.anchors[0].rotationDelta;
		q = { -q.w, -q.x, -q.y, -q.z };

		double p[3] = { -2.0, 1.0, -2.5 };
		vr::HmdQuaternion_t r1, r2;
		double t1[3], t2[3];
		alignfield::BlendAt(field, p, r1, t1);
		alignfield::BlendAt(negated, p, r2, t2);

		double dq = std::abs(r1.w * r2.w + r1.x * r2.x + r1.y * r2.y + r1.z * r2.z);
		double dt = Dist3(t1, t2);
		snprintf(detail, sizeof detail, "|dot| %.12f  dTrans %.2e", dq, dt);
		Check("field: hemisphere invariance", dq > 1.0 - 1e-12 && dt < 1e-12, detail);
	}

	// D. The driver's hand-rolled blend agrees with the Eigen reference over
	// random fields and query points.
	{
		std::mt19937 rng(2024);
		std::uniform_real_distribution<double> u(-1.0, 1.0);
		double worstQ = 0.0, worstT = 0.0;

		for (int trial = 0; trial < 3; ++trial)
		{
			protocol::SetAlignmentField rf;
			rf.enabled = true;
			rf.generation = trial;
			rf.anchorCount = 1 + (trial * 3) % protocol::SetAlignmentField::MaxAnchors;
			rf.sigmaMeters = 1.0 + 0.5 * (trial + 1);
			for (uint32_t i = 0; i < rf.anchorCount; ++i)
			{
				Eigen::Quaterniond dq(Eigen::AngleAxisd(0.05 * u(rng), Eigen::Vector3d(u(rng), u(rng), u(rng)).normalized()));
				if (u(rng) < 0.0)
					dq.coeffs() = -dq.coeffs();   // exercise both representations
				rf.anchors[i].rotationDelta = { dq.w(), dq.x(), dq.y(), dq.z() };
				for (int k = 0; k < 3; ++k)
				{
					rf.anchors[i].position[k] = 4.0 * u(rng);
					rf.anchors[i].translationDelta[k] = 0.05 * u(rng);
				}
			}

			for (int s = 0; s < 50; ++s)
			{
				Eigen::Vector3d pos(6.0 * u(rng), 1.0 + u(rng), 6.0 * u(rng));
				double p[3] = { pos.x(), pos.y(), pos.z() };

				vr::HmdQuaternion_t r;
				double t[3];
				alignfield::BlendAt(rf, p, r, t);

				Eigen::Quaterniond refR;
				Eigen::Vector3d refT;
				ReferenceBlend(rf, pos, refR, refT);

				worstQ = std::max(worstQ, 1.0 - std::abs(r.w * refR.w() + r.x * refR.x() + r.y * refR.y() + r.z * refR.z()));
				worstT = std::max(worstT, (Eigen::Vector3d(t[0], t[1], t[2]) - refT).norm());
			}
		}
		snprintf(detail, sizeof detail, "worst 1-|dot| %.2e  worst dTrans %.2e", worstQ, worstT);
		Check("field: matches reference", worstQ < 1e-12 && worstT < 1e-12, detail);
	}

	// E. Same generation slews (rate-limited steps toward the target, then
	// convergence); a generation bump snaps immediately.
	{
		protocol::SetAlignmentField f1 = BuildField(base, anchors, positions, 7);
		double p[3] = { positions[0].x(), positions[0].y(), positions[0].z() };

		alignfield::EvalState state;
		alignfield::Evaluate(f1, p, 0.0, state);   // first eval: snap to target
		double snapTrans[3] = { state.trans[0], state.trans[1], state.trans[2] };

		// Move the anchor delta 10 cm, same generation: one 10 ms step may
		// advance at most MaxTranslationSlewPerSec * dt.
		protocol::SetAlignmentField f2 = f1;
		f2.anchors[0].translationDelta[0] += 0.10;
		alignfield::Evaluate(f2, p, 0.010, state);
		double step = Dist3(state.trans, snapTrans);
		bool limited = step < alignfield::MaxTranslationSlewPerSec * 0.010 + 1e-9 && step > 0.0;

		// Keep evaluating: it must converge to the new target within a second.
		double t = 0.010;
		for (int i = 0; i < 120; ++i)
		{
			t += 0.010;
			alignfield::Evaluate(f2, p, t, state);
		}
		vr::HmdQuaternion_t targetR;
		double targetT[3];
		alignfield::BlendAt(f2, p, targetR, targetT);
		double convErr = Dist3(state.trans, targetT);

		// Generation bump with another 10 cm move: exact snap in one step.
		protocol::SetAlignmentField f3 = f2;
		f3.generation = 8;
		f3.anchors[0].translationDelta[2] -= 0.10;
		alignfield::Evaluate(f3, p, t + 0.010, state);
		alignfield::BlendAt(f3, p, targetR, targetT);
		double snapErr = Dist3(state.trans, targetT);

		snprintf(detail, sizeof detail, "step %.2f mm (max 2.5)  conv %.2e  snap %.2e",
			step * 1000.0, convErr, snapErr);
		Check("field: slew + generation snap", limited && convErr < 1e-9 && snapErr < 1e-12, detail);
	}

	// F. Universe-jump invariance: shifting base and anchors by D (what
	// ApplyUniverseDelta does) and re-deriving the deltas must move every
	// corrected world pose by exactly D. Exact for rotation; translation holds
	// to second order in the delta angles (linear quat blend vs matrix
	// average), far below anything a sign or composition-order bug produces.
	{
		const FieldTransform D{
			Eigen::Quaterniond(Eigen::AngleAxisd(25.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY())),
			Eigen::Vector3d(0.4, 0.0, -0.3) };

		FieldTransform baseJ = Compose(D, base);
		std::vector<FieldTransform> anchorsJ;
		std::vector<Eigen::Vector3d> positionsJ;
		for (size_t i = 0; i < anchors.size(); ++i)
		{
			anchorsJ.push_back(Compose(D, anchors[i]));
			positionsJ.push_back(D.Apply(positions[i]));
		}
		protocol::SetAlignmentField fieldJ = BuildField(baseJ, anchorsJ, positionsJ, 2);

		std::mt19937 rng(5);
		std::uniform_real_distribution<double> u(-1.0, 1.0);
		double worst = 0.0, worstRot = 0.0;
		for (int s = 0; s < 40; ++s)
		{
			Eigen::Vector3d targetPt(4.0 * u(rng), 1.0 + u(rng), 4.0 * u(rng));

			FieldTransform eff = DriverEffective(field, base, base.Apply(targetPt));
			FieldTransform effJ = DriverEffective(fieldJ, baseJ, baseJ.Apply(targetPt));

			double rotErrDeg = effJ.R.angularDistance((D.R * eff.R).normalized()) * 180.0 / EIGEN_PI;
			worst = std::max(worst, (effJ.Apply(targetPt) - D.Apply(eff.Apply(targetPt))).norm());
			worstRot = std::max(worstRot, rotErrDeg);
		}
		snprintf(detail, sizeof detail, "worst pos %.4f mm  worst rot %.5f deg", worst * 1000.0, worstRot);
		Check("field: jump invariance", worst < 0.001 && worstRot < 0.001, detail);
	}
}

// ---------------------------------------------------------------------------
// Chaperone snapshot math (universe-jump re-anchoring + restore trigger)

void RunChaperoneScenarios()
{
	char detail[256];

	const Eigen::Quaterniond D_R(Eigen::AngleAxisd(25.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
	const Eigen::Vector3d D_T(0.4, 0.02, -0.3);

	// A plausible standing-center pose: yaw, slight floor tilt, offset.
	const Eigen::Matrix3d M_R = (Eigen::AngleAxisd(1.3, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX())).toRotationMatrix();
	const Eigen::Vector3d M_T(0.7, 0.0, -1.9);

	vr::HmdMatrix34_t m;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			m.m[i][j] = static_cast<float>(M_R(i, j));
		m.m[i][3] = static_cast<float>(M_T(i));
	}

	// A. Physical-point invariance: the re-anchored pose must place every
	// standing-frame point exactly where the delta moves its old raw position.
	{
		vr::HmdMatrix34_t out = DeltaTimesPose(D_R, D_T, m);
		const Eigen::Vector3d pts[] = {
			Eigen::Vector3d::Zero(),
			Eigen::Vector3d(1.1, 0.0, -0.8),
			Eigen::Vector3d(-0.4, 1.7, 0.9),
			Eigen::Vector3d(2.0, 0.1, 2.0),
		};
		double worst = 0.0;
		for (const auto &p : pts)
		{
			Eigen::Vector3d expect = D_R * (M_R * p + M_T) + D_T;
			Eigen::Vector3d got;
			for (int i = 0; i < 3; ++i)
				got(i) = out.m[i][0] * p.x() + out.m[i][1] * p.y() + out.m[i][2] * p.z() + out.m[i][3];
			worst = std::max(worst, (got - expect).norm());
		}
		snprintf(detail, sizeof detail, "worst %.2e m", worst);
		Check("chaperone: delta re-anchor", worst < 1e-5, detail);
	}

	// B. Composing with the inverse delta returns the original pose.
	{
		Eigen::Quaterniond invR = D_R.conjugate();
		Eigen::Vector3d invT = -(invR * D_T);
		vr::HmdMatrix34_t back = DeltaTimesPose(invR, invT, DeltaTimesPose(D_R, D_T, m));
		double worst = 0.0;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 4; ++j)
				worst = std::max(worst, static_cast<double>(std::fabs(back.m[i][j] - m.m[i][j])));
		snprintf(detail, sizeof detail, "worst %.2e", worst);
		Check("chaperone: delta round-trip", worst < 1e-5, detail);
	}

	// C. Restore trigger: tolerant of sub-mm round-trip noise, trips on a real
	// wall move and on a count change.
	{
		std::vector<vr::HmdQuad_t> a(6);
		for (size_t i = 0; i < a.size(); ++i)
			for (int c = 0; c < 4; ++c)
				for (int k = 0; k < 3; ++k)
					a[i].vCorners[c].v[k] = static_cast<float>(0.37 * i + 0.11 * c + 0.05 * k - 1.0);

		std::vector<vr::HmdQuad_t> jitter = a;
		jitter[3].vCorners[2].v[0] += 0.0005f;

		std::vector<vr::HmdQuad_t> moved = a;
		moved[3].vCorners[2].v[0] += 0.05f;

		std::vector<vr::HmdQuad_t> fewer(a.begin(), a.end() - 1);

		bool pass = QuadsMatch(a, a, 0.002f) && QuadsMatch(a, jitter, 0.002f) &&
			!QuadsMatch(a, moved, 0.002f) && !QuadsMatch(a, fewer, 0.002f);
		Check("chaperone: quads match", pass, "");
	}
}

// ---------------------------------------------------------------------------
// Base-transform slew (continuous calibration, protocol v5)

Eigen::Quaterniond SlewStateRot(const alignfield::EvalState &s)
{
	return Eigen::Quaterniond(s.rot.w, s.rot.x, s.rot.y, s.rot.z);
}

void RunBaseSlewScenarios()
{
	char detail[256];
	const alignfield::SlewLimits &lim = alignfield::BaseSlewLimits;

	const vr::HmdQuaternion_t rotA{ 1.0, 0.0, 0.0, 0.0 };
	const double transA[3] = { 0.10, 0.02, -0.30 };

	// 1 deg yaw + 2 cm away from A: a worst-case continuous correction.
	const Eigen::Quaterniond qB(Eigen::AngleAxisd(1.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
	const vr::HmdQuaternion_t rotB{ qB.w(), qB.x(), qB.y(), qB.z() };
	const double transB[3] = { 0.10 + 0.012, 0.02, -0.30 - 0.016 };

	// A. First eval snaps; rate-limited steps toward a moved target never
	// exceed the per-step caps and converge within the expected time.
	{
		alignfield::EvalState state;
		alignfield::SlewToward(rotA, transA, 0.0, lim, 3, state);
		bool firstSnap = Dist3(state.trans, transA) < 1e-12 &&
			SlewStateRot(state).angularDistance(Eigen::Quaterniond::Identity()) < 1e-12;

		const double dt = 1.0 / 250.0;
		double t = 0.0, worstTransStep = 0.0, worstRotStep = 0.0, converged = -1.0;
		double prevTrans[3] = { state.trans[0], state.trans[1], state.trans[2] };
		Eigen::Quaterniond prevRot = SlewStateRot(state);
		double normErr = 0.0;

		for (int i = 0; i < 1000; ++i)   // 4 s at 250 Hz
		{
			t += dt;
			alignfield::SlewToward(rotB, transB, t, lim, 3, state);

			worstTransStep = std::max(worstTransStep, Dist3(state.trans, prevTrans));
			worstRotStep = std::max(worstRotStep, SlewStateRot(state).angularDistance(prevRot));
			normErr = std::max(normErr, std::abs(1.0 - SlewStateRot(state).norm()));

			prevTrans[0] = state.trans[0]; prevTrans[1] = state.trans[1]; prevTrans[2] = state.trans[2];
			prevRot = SlewStateRot(state);

			if (converged < 0.0 && Dist3(state.trans, transB) < 1e-9 &&
				SlewStateRot(state).angularDistance(qB) < 1e-9)
				converged = t;
		}

		bool capped = worstTransStep <= lim.maxTranslationPerSec * dt + 1e-12 &&
			worstRotStep <= lim.maxRotationPerSec * dt + 1e-9;
		// 2 cm at 0.01 m/s = 2.0 s; 1 deg at 0.5 deg/s = 2.0 s.
		bool timely = converged > 1.8 && converged < 2.3;
		snprintf(detail, sizeof detail, "steps %.3f mm / %.4f deg  conv %.2f s  norm %.1e",
			worstTransStep * 1000.0, worstRotStep * 180.0 / EIGEN_PI, converged, normErr);
		Check("baseslew: rate caps + convergence", firstSnap && capped && timely && normErr < 1e-12, detail);
	}

	// B. Generation change and evaluation gaps snap; steady re-sends of an
	// unchanged target are no-ops.
	{
		alignfield::EvalState state;
		alignfield::SlewToward(rotA, transA, 0.0, lim, 3, state);

		// Same generation, 2 cm move, but a 0.5 s evaluation gap (> maxGap): snap.
		alignfield::SlewToward(rotB, transB, 0.5, lim, 3, state);
		bool gapSnap = Dist3(state.trans, transB) < 1e-12;

		// Generation bump with another move: snap in one 4 ms step.
		const double transC[3] = { 0.0, 0.0, 0.0 };
		alignfield::SlewToward(rotA, transC, 0.504, lim, 4, state);
		bool genSnap = Dist3(state.trans, transC) < 1e-12 &&
			SlewStateRot(state).angularDistance(Eigen::Quaterniond::Identity()) < 1e-12;

		// Unchanged target re-sent at 250 Hz: state must not move at all.
		double t = 0.504, worst = 0.0;
		for (int i = 0; i < 100; ++i)
		{
			t += 1.0 / 250.0;
			alignfield::SlewToward(rotA, transC, t, lim, 4, state);
			worst = std::max(worst, Dist3(state.trans, transC));
			worst = std::max(worst, SlewStateRot(state).angularDistance(Eigen::Quaterniond::Identity()));
		}
		snprintf(detail, sizeof detail, "gap %d gen %d  hold %.1e", gapSnap, genSnap, worst);
		Check("baseslew: snap conditions + hold", gapSnap && genSnap && worst < 1e-12, detail);
	}

	// C. q and -q are the same rotation: a negated target quaternion must
	// converge to the same orientation, the short way.
	{
		alignfield::EvalState a, b;
		alignfield::SlewToward(rotA, transA, 0.0, lim, 1, a);
		alignfield::SlewToward(rotA, transA, 0.0, lim, 1, b);

		const vr::HmdQuaternion_t rotBNeg{ -rotB.w, -rotB.x, -rotB.y, -rotB.z };
		double t = 0.0, worst = 0.0;
		for (int i = 0; i < 700; ++i)   // past convergence
		{
			t += 1.0 / 250.0;
			alignfield::SlewToward(rotB, transB, t, lim, 1, a);
			alignfield::SlewToward(rotBNeg, transB, t, lim, 1, b);
			worst = std::max(worst, SlewStateRot(a).angularDistance(SlewStateRot(b)));
		}
		bool atTarget = SlewStateRot(a).angularDistance(qB) < 1e-9;
		snprintf(detail, sizeof detail, "worst path div %.1e  atTarget %d", worst, atTarget);
		Check("baseslew: hemisphere invariance", worst < 1e-9 && atTarget, detail);
	}
}

// ---------------------------------------------------------------------------
// Continuous calibration (HMD-mounted tracker)

// The mount used by GenerateStreams; MakeSample models a rigidly mounted
// device B, so this IS the ground-truth extrinsic (tracker in the HMD frame).
const Eigen::Quaterniond kMountRot(Eigen::AngleAxisd(2.7, Eigen::Vector3d(0.2, 0.7, -0.3).normalized()));
const Eigen::Vector3d kMountPos(0.05, -0.08, 0.03);

struct ContinuousSim
{
	ContinuousAlignment ca;
	Eigen::Quaterniond calRot{ 1, 0, 0, 0 };
	Eigen::Vector3d calTrans{ 0, 0, 0 };
	double calScale = 1.0;
	double solvedOffset = 0.0;

	int corrections = 0;
	int freezes = 0, resumes = 0, losses = 0, recoveries = 0;
	int unstables = 0;
	double maxCorrRotDeg = 0.0;   // largest single emitted correction
	double maxCorrPosM = 0.0;     // measured as displacement at the head
	double afterMark = 1e18;
	int correctionsAfter = 0;

	double nextRef = -1.0, nextTarget = -1.0, nextUpdate = -1.0;
};

// Deviation of the sim's applied calibration from a ground truth, in the same
// terms the engine uses: yaw angle plus effective displacement at the head.
void CalError(const ContinuousSim &sim, const GroundTruth &truth, double t,
              double &yawDegOut, double &posMOut)
{
	Eigen::Quaterniond dR = (truth.rotation * sim.calRot.conjugate()).normalized();
	if (dR.w() < 0.0)
		dR.coeffs() = -dR.coeffs();
	yawDegOut = std::abs(2.0 * std::atan2(dR.y(), dR.w())) * 180.0 / EIGEN_PI;
	Eigen::Vector3d tD = truth.translation - dR * sim.calTrans;
	Eigen::Vector3d hp = PositionAt(t);
	posMOut = (dR * hp + tD - hp).norm();
}

// One closed-loop segment: generate both streams, tick Update at 50 Hz, apply
// polled corrections back onto the sim's calibration (exactly what the
// overlay's ApplyAlignmentDelta will do), and tally events.
void RunContinuousSegment(ContinuousSim &sim, const SceneConfig &scene, double t0, double t1,
	std::mt19937 &rng,
	std::function<GroundTruth(double)> truthAt,
	std::function<void(double, Eigen::Quaterniond &, Eigen::Vector3d &)> mountAt,
	std::function<bool(double)> targetVisible,
	std::function<void(double, PoseSample &)> refPost = nullptr,
	bool negateTargetQuat = false,
	std::function<void(double)> onUpdate = nullptr)
{
	if (sim.nextRef < 0.0) sim.nextRef = t0;
	if (sim.nextTarget < 0.0) sim.nextTarget = t0;
	if (sim.nextUpdate < 0.0) sim.nextUpdate = t0;

	const double refDt = 1.0 / scene.refRate;
	const double tgtDt = 1.0 / scene.targetRate;
	const double updDt = 0.02;

	while (true)
	{
		double t = std::min(sim.nextRef, std::min(sim.nextTarget, sim.nextUpdate));
		if (t >= t1)
			break;

		GroundTruth truth = truthAt(t);
		Eigen::Quaterniond mR;
		Eigen::Vector3d mP;
		mountAt(t, mR, mP);

		if (t == sim.nextRef)
		{
			PoseSample s = MakeSample(t, t, scene, truth, false, mR, mP, rng);
			if (refPost)
				refPost(t, s);
			sim.ca.PushReference(s);
			sim.nextRef += refDt;
		}
		else if (t == sim.nextTarget)
		{
			if (targetVisible(t))
			{
				PoseSample s = MakeSample(t, t - truth.latency, scene, truth, true, mR, mP, rng);
				if (negateTargetQuat && (static_cast<long long>(t * scene.targetRate) & 1))
					s.rot.coeffs() = -s.rot.coeffs();
				sim.ca.PushTarget(s);
			}
			sim.nextTarget += tgtDt;
		}
		else
		{
			sim.ca.Update(t, sim.calRot, sim.calTrans, sim.calScale, sim.solvedOffset);

			ContinuousAlignment::Correction c;
			while (sim.ca.PollCorrection(c))
			{
				double corrDeg = c.rotation.angularDistance(Eigen::Quaterniond::Identity()) * 180.0 / EIGEN_PI;
				sim.maxCorrRotDeg = std::max(sim.maxCorrRotDeg, corrDeg);
				Eigen::Vector3d hp = PositionAt(t);
				sim.maxCorrPosM = std::max(sim.maxCorrPosM, (c.rotation * hp + c.translation - hp).norm());

				sim.calRot = (c.rotation * sim.calRot).normalized();
				sim.calTrans = c.rotation * sim.calTrans + c.translation;
				sim.corrections++;
				if (t >= sim.afterMark)
					sim.correctionsAfter++;
			}

			ContinuousAlignment::Event e;
			while (sim.ca.PollEvent(e))
			{
				switch (e.type)
				{
				case ContinuousAlignment::Event::FrozenLargeDeviation: sim.freezes++; break;
				case ContinuousAlignment::Event::Resumed: sim.resumes++; break;
				case ContinuousAlignment::Event::TrackerLost: sim.losses++; break;
				case ContinuousAlignment::Event::TrackerRecovered: sim.recoveries++; break;
				case ContinuousAlignment::Event::ObservationsUnstable: sim.unstables++; break;
				}
			}

			if (onUpdate)
				onUpdate(t);
			sim.nextUpdate += updDt;
		}
	}
}

void RunContinuousScenarios()
{
	char detail[256];

	GroundTruth baseTruth;
	baseTruth.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(1.9, Eigen::Vector3d::UnitY()));
	baseTruth.translation = Eigen::Vector3d(1.2, 0.03, -0.7);
	baseTruth.latency = 0.018;

	SceneConfig scene;
	scene.posNoise = 0.002;
	scene.rotNoiseDeg = 0.15;

	auto constTruth = [&](double) { return baseTruth; };
	auto constMount = [](double, Eigen::Quaterniond &r, Eigen::Vector3d &p)
	{
		r = kMountRot;
		p = kMountPos;
	};
	auto alwaysVisible = [](double) { return true; };

	const MountExtrinsic trueExtrinsic = []
	{
		MountExtrinsic e;
		e.valid = true;
		e.rot = kMountRot;
		e.pos = kMountPos;
		return e;
	}();

	// Sim whose calibration starts a known 0.3 deg / few-mm delta away from
	// truth, for the scenarios that must walk it back.
	auto makeOffsetSim = [&](ContinuousSim &sim)
	{
		sim.ca.SetExtrinsic(trueExtrinsic);
		sim.solvedOffset = baseTruth.latency;
		Eigen::Quaterniond dR(Eigen::AngleAxisd(0.3 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
		Eigen::Vector3d dT(0.004, 0.002, -0.003);
		sim.calRot = (dR.conjugate() * baseTruth.rotation).normalized();
		sim.calTrans = dR.conjugate() * (baseTruth.translation - dT);
	};

	// 1. Extrinsic derivation recovers the mount from a manual calibration's
	// buffers; a wobbling (non-rigid) mount fails the rigidity gate.
	{
		std::mt19937 rng(77);
		std::vector<PoseSample> ref, tgt, tgtWobble;
		for (double t = 0.0; t < 12.0; t += 1.0 / scene.refRate)
			ref.push_back(MakeSample(t, t, scene, baseTruth, false, kMountRot, kMountPos, rng));
		for (double t = 0.0; t < 12.0; t += 1.0 / scene.targetRate)
			tgt.push_back(MakeSample(t, t - baseTruth.latency, scene, baseTruth, true, kMountRot, kMountPos, rng));
		for (double t = 0.0; t < 12.0; t += 1.0 / scene.targetRate)
		{
			Eigen::Quaterniond mR = kMountRot
				* Eigen::Quaterniond(Eigen::AngleAxisd(0.035 * std::sin(1.3 * t), Eigen::Vector3d::UnitX()));
			Eigen::Vector3d mP = kMountPos + Eigen::Vector3d(0.02 * std::sin(0.9 * t), 0.0, 0.0);
			tgtWobble.push_back(MakeSample(t, t - baseTruth.latency, scene, baseTruth, true, mR, mP, rng));
		}

		EngineResult cal;
		cal.valid = true;
		cal.rotation = baseTruth.rotation;
		cal.translation = baseTruth.translation;
		cal.scale = baseTruth.scale;
		cal.timeOffset = baseTruth.latency;

		ContinuousAlignment::Config cfg;
		MountExtrinsic e, eWobble;
		bool ok = ContinuousAlignment::DeriveMountExtrinsic(ref, tgt, cal, cfg, e);
		double rotErr = e.rot.angularDistance(kMountRot) * 180.0 / EIGEN_PI;
		double posErr = (e.pos - kMountPos).norm();
		bool okWobble = ContinuousAlignment::DeriveMountExtrinsic(ref, tgtWobble, cal, cfg, eWobble);

		snprintf(detail, sizeof detail, "rotErr %.3f deg  posErr %.1f mm  pairs %zu  wobble rms %.2f deg",
			rotErr, posErr * 1000.0, e.pairs, eWobble.rotRmsDeg);
		Check("continuous: extrinsic + rigidity gate",
			ok && rotErr < 0.1 && posErr < 0.002 && !okWobble && !eWobble.valid, detail);
	}

	// 2. Frame-convention keystone: start the calibration a known small delta
	// away from truth; the closed loop must walk it back to truth (any sign or
	// composition-order bug diverges or leaves a large stable residual).
	{
		std::mt19937 rng(101);
		ContinuousSim sim;
		makeOffsetSim(sim);

		RunContinuousSegment(sim, scene, 0.0, 15.0, rng, constTruth, constMount, alwaysVisible);

		double yawErr, posErr;
		CalError(sim, baseTruth, 15.0, yawErr, posErr);
		snprintf(detail, sizeof detail, "residual yaw %.3f deg  pos %.1f mm  corrections %d",
			yawErr, posErr * 1000.0, sim.corrections);
		Check("continuous: converges to truth",
			sim.corrections >= 1 && sim.freezes == 0 && yawErr < 0.12 && posErr < 0.005, detail);
	}

	// 3. Double cover + outliers: q and -q fed alternately, plus 5% half-meter
	// glitch samples. The eigenvector mean is sign-invariant, the trim pass
	// eats the glitches, and the two-obs jump-guard confirmation keeps single
	// glitches from dropping the window.
	{
		std::mt19937 rng(202);
		SceneConfig dirty = scene;
		dirty.outlierRate = 0.05;

		ContinuousSim sim;
		makeOffsetSim(sim);

		RunContinuousSegment(sim, dirty, 0.0, 15.0, rng, constTruth, constMount, alwaysVisible,
			nullptr, /*negateTargetQuat=*/true);

		double yawErr, posErr;
		CalError(sim, baseTruth, 15.0, yawErr, posErr);
		snprintf(detail, sizeof detail, "residual yaw %.3f deg  pos %.1f mm  corrections %d",
			yawErr, posErr * 1000.0, sim.corrections);
		Check("continuous: double cover + outliers",
			sim.corrections >= 1 && sim.freezes == 0 && yawErr < 0.12 && posErr < 0.006, detail);
	}

	// 4. Slow drift maintenance: truth drifts 2 mm/s + 0.05 deg/s for 70 s;
	// the applied calibration must track it (steady-state lag = drift rate x
	// half the window, plus the deadband), with zero freezes.
	{
		std::mt19937 rng(303);
		auto driftTruth = [&](double t)
		{
			GroundTruth g = baseTruth;
			g.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(
				1.9 + (0.05 * EIGEN_PI / 180.0) * t, Eigen::Vector3d::UnitY()));
			g.translation = baseTruth.translation
				+ 0.002 * t * Eigen::Vector3d(1.0, 0.0, 0.3).normalized();
			return g;
		};

		ContinuousSim sim;
		sim.ca.SetExtrinsic(trueExtrinsic);
		sim.solvedOffset = baseTruth.latency;
		sim.calRot = baseTruth.rotation;
		sim.calTrans = baseTruth.translation;

		double maxYaw = 0.0, maxPos = 0.0;
		RunContinuousSegment(sim, scene, 0.0, 70.0, rng, driftTruth, constMount, alwaysVisible,
			nullptr, false,
			[&](double t)
			{
				if (t < 25.0)
					return;   // warmup: window fill + first corrections
				double yawErr, posErr;
				CalError(sim, driftTruth(t), t, yawErr, posErr);
				maxYaw = std::max(maxYaw, yawErr);
				maxPos = std::max(maxPos, posErr);
			});

		// Steady-state position lag = drift rate x half the window (10 mm) plus
		// the apply cadence, deadband, and noise; uncorrected 70 s of this
		// drift would be 140 mm / 3.5 deg.
		snprintf(detail, sizeof detail, "worst yaw %.3f deg  pos %.1f mm  corrections %d  freezes %d",
			maxYaw, maxPos * 1000.0, sim.corrections, sim.freezes);
		Check("continuous: tracks slow drift",
			sim.corrections > 10 && sim.freezes == 0 && maxYaw < 0.5 && maxPos < 0.028, detail);
	}

	// 5. Universe jump mid-run: the reference universe rebases 20 deg / 30 cm
	// in one frame. The guard drops the straddling window (never averaging it
	// into a correction), every emitted correction stays inside the step
	// clamp, and after the JumpDetector path wins (delta applied + Reset)
	// tracking resumes cleanly.
	{
		std::mt19937 rng(404);
		const Eigen::Quaterniond jR(Eigen::AngleAxisd(20.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
		const Eigen::Vector3d jT(0.3, 0.0, -0.1);
		auto refPost = [&](double t, PoseSample &s)
		{
			if (t < 30.0)
				return;
			s.rot = (jR * s.rot).normalized();
			s.pos = jR * s.pos + jT;
		};

		ContinuousSim sim;
		sim.ca.SetExtrinsic(trueExtrinsic);
		sim.solvedOffset = baseTruth.latency;
		sim.calRot = baseTruth.rotation;
		sim.calTrans = baseTruth.translation;

		RunContinuousSegment(sim, scene, 0.0, 31.0, rng, constTruth, constMount, alwaysVisible, refPost);

		// JumpDetector wins one second after the jump: fold its (slightly
		// imperfect, 0.4 deg off) delta estimate, reset us. The continuous
		// loop must clean up the residual afterwards.
		const Eigen::Quaterniond jErr(Eigen::AngleAxisd(0.4 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
		sim.calRot = ((jErr * jR) * sim.calRot).normalized();
		sim.calTrans = (jErr * jR) * sim.calTrans + jT;
		sim.ca.Reset();
		sim.afterMark = 31.0;

		RunContinuousSegment(sim, scene, 31.0, 50.0, rng, constTruth, constMount, alwaysVisible, refPost);

		GroundTruth jumped = baseTruth;
		jumped.rotation = (jR * baseTruth.rotation).normalized();
		jumped.translation = jR * baseTruth.translation + jT;
		double yawErr, posErr;
		CalError(sim, jumped, 50.0, yawErr, posErr);

		snprintf(detail, sizeof detail,
			"maxCorr %.2f deg / %.1f mm  post-jump corr %d  residual %.3f deg / %.1f mm",
			sim.maxCorrRotDeg, sim.maxCorrPosM * 1000.0, sim.correctionsAfter, yawErr, posErr * 1000.0);
		Check("continuous: jump guard + resume",
			sim.freezes == 0 && sim.maxCorrRotDeg <= 0.51 && sim.maxCorrPosM <= 0.02 &&
			sim.correctionsAfter >= 1 && yawErr < 0.15 && posErr < 0.008, detail);
	}

	// 6. Mount slip: the physical mount shifts 3 deg / 2 cm mid-run while the
	// stored extrinsic stays put. The deviation (whose error rotates with the
	// head) must freeze auto-apply — one event, no corrections once frozen, no
	// resume — because a real slip can only be fixed by recalibrating.
	{
		std::mt19937 rng(505);
		auto slipMount = [&](double t, Eigen::Quaterniond &r, Eigen::Vector3d &p)
		{
			if (t < 30.0)
			{
				r = kMountRot;
				p = kMountPos;
				return;
			}
			r = kMountRot * Eigen::Quaterniond(Eigen::AngleAxisd(
				3.0 * EIGEN_PI / 180.0, Eigen::Vector3d(1.0, 0.2, 0.0).normalized()));
			p = kMountPos + Eigen::Vector3d(0.02, 0.0, -0.01);
		};

		ContinuousSim sim;
		sim.ca.SetExtrinsic(trueExtrinsic);
		sim.solvedOffset = baseTruth.latency;
		sim.calRot = baseTruth.rotation;
		sim.calTrans = baseTruth.translation;
		sim.afterMark = 55.0;

		RunContinuousSegment(sim, scene, 0.0, 75.0, rng, constTruth, slipMount, alwaysVisible);

		bool frozenAtEnd = sim.ca.GetState() == ContinuousAlignment::State::Frozen;
		snprintf(detail, sizeof detail, "freezes %d  resumes %d  late corr %d  frozen %d",
			sim.freezes, sim.resumes, sim.correctionsAfter, frozenAtEnd);
		Check("continuous: mount slip freezes",
			sim.freezes >= 1 && sim.resumes == 0 && sim.correctionsAfter == 0 && frozenAtEnd, detail);
	}

	// 7. Occlusion + reset: short and long tracker dropouts coast (hold the
	// calibration, one lost/recovered event each, no freeze), a small truth
	// step after the long dropout is corrected once the window refills, and
	// Reset requires a full refill before correcting again.
	{
		std::mt19937 rng(606);
		auto visible = [](double t)
		{
			if (t >= 20.0 && t < 23.0) return false;
			if (t >= 40.0 && t < 70.0) return false;
			return true;
		};

		// Truth steps 0.3 deg at t=73 (post-recovery, inside the guard band) so
		// "resumed correcting" is observable.
		GroundTruth stepped = baseTruth;
		stepped.rotation = (Eigen::Quaterniond(Eigen::AngleAxisd(
			0.3 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY())) * baseTruth.rotation).normalized();
		auto stepTruth = [&](double t) { return t < 73.0 ? baseTruth : stepped; };

		ContinuousSim sim;
		sim.ca.SetExtrinsic(trueExtrinsic);
		sim.solvedOffset = baseTruth.latency;
		sim.calRot = baseTruth.rotation;
		sim.calTrans = baseTruth.translation;
		sim.afterMark = 73.0;

		RunContinuousSegment(sim, scene, 0.0, 82.0, rng, stepTruth, constMount, visible);
		bool coastOk = sim.losses == 2 && sim.recoveries == 2 && sim.freezes == 0 &&
			sim.correctionsAfter >= 1;

		// Reset semantics: perturb the calibration, reset — no corrections
		// until the window refills (minObs at ~10 obs/s), then it heals.
		sim.ca.Reset();
		sim.calRot = (Eigen::Quaterniond(Eigen::AngleAxisd(
			-0.3 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY())) * sim.calRot).normalized();
		int before = sim.corrections;
		RunContinuousSegment(sim, scene, 82.0, 85.5, rng, stepTruth, constMount, alwaysVisible);
		bool quietAfterReset = sim.corrections == before;
		RunContinuousSegment(sim, scene, 85.5, 97.0, rng, stepTruth, constMount, alwaysVisible);
		bool resumed = sim.corrections > before;

		snprintf(detail, sizeof detail, "losses %d  recoveries %d  freezes %d  late corr %d  reset %d/%d",
			sim.losses, sim.recoveries, sim.freezes, sim.correctionsAfter, quietAfterReset, resumed);
		Check("continuous: occlusion + reset", coastOk && quietAfterReset && resumed, detail);
	}

	// 8. Opt-in latency re-estimation: the true latency ramps 8 -> 16 ms over
	// six minutes; the EWMA-tracked offset (mirroring ContinuousTick's clamp)
	// follows within 3 ms after the ramp settles and never steps more than
	// 2 ms per update. With the option off the offset never moves, and a
	// motionless window yields no estimate at all.
	{
		std::mt19937 rng(707);
		auto rampTruth = [&](double t)
		{
			GroundTruth g = baseTruth;
			g.latency = 0.008 + 0.008 * std::min(1.0, t / 360.0);
			return g;
		};

		ContinuousAlignment::Config cfg;
		cfg.latencyReestimation = true;

		ContinuousSim sim;
		sim.ca.SetConfig(cfg);
		sim.ca.SetExtrinsic(trueExtrinsic);
		sim.solvedOffset = 0.008;
		sim.calRot = baseTruth.rotation;
		sim.calTrans = baseTruth.translation;

		double maxStep = 0.0;
		int offsetUpdates = 0;
		auto applyEwma = [&](double)
		{
			double measured;
			while (sim.ca.PollTimeOffset(measured))
			{
				double target = 0.75 * sim.solvedOffset + 0.25 * measured;
				double step = target - sim.solvedOffset;
				if (step > 0.002) step = 0.002;
				if (step < -0.002) step = -0.002;
				maxStep = std::max(maxStep, std::abs(step));
				sim.solvedOffset += step;
				offsetUpdates++;
			}
		};
		RunContinuousSegment(sim, scene, 0.0, 600.0, rng, rampTruth, constMount, alwaysVisible,
			nullptr, false, applyEwma);
		double trackErr = std::abs(sim.solvedOffset - 0.016);

		// Opt-out: identical run, offset must never move.
		ContinuousSim simOff;
		simOff.ca.SetExtrinsic(trueExtrinsic);
		simOff.solvedOffset = 0.008;
		simOff.calRot = baseTruth.rotation;
		simOff.calTrans = baseTruth.translation;
		int offUpdates = 0;
		std::mt19937 rngOff(707);
		RunContinuousSegment(simOff, scene, 0.0, 120.0, rngOff, rampTruth, constMount, alwaysVisible,
			nullptr, false,
			[&](double)
			{
				double m;
				while (simOff.ca.PollTimeOffset(m))
					offUpdates++;
			});

		// Motionless streams: the correlator must refuse.
		std::vector<PoseSample> stillRef, stillTgt;
		for (double t = 0.0; t < 30.0; t += 1.0 / 90.0)
		{
			PoseSample s;
			s.time = t;
			s.rot = Eigen::Quaterniond::Identity();
			s.pos = Eigen::Vector3d(0.0, 1.2, 0.0);
			stillRef.push_back(s);
			stillTgt.push_back(s);
		}
		EngineConfig ecfg;
		double stillOut = 0.0;
		bool stillEstimated = CalibrationEngine::EstimateTimeOffset(stillRef, stillTgt, ecfg, stillOut);

		snprintf(detail, sizeof detail,
			"tracked to %.1f ms (err %.2f ms)  %d updates  maxStep %.2f ms  off %d  still %d",
			sim.solvedOffset * 1000.0, trackErr * 1000.0, offsetUpdates, maxStep * 1000.0,
			offUpdates, stillEstimated);
		Check("continuous: latency re-estimation",
			offsetUpdates >= 5 && trackErr < 0.003 && maxStep <= 0.002 + 1e-12 &&
			offUpdates == 0 && simOff.solvedOffset == 0.008 && !stillEstimated, detail);
	}

	// The overlay-side anchor shape ContinuousTick blends over.
	struct OverlayAnchor
	{
		Eigen::Vector3d position;
		Eigen::Quaterniond rotation;
		Eigen::Vector3d translationMeters;
	};

	// 9. Overlay field expectation: BlendedFieldCalibration must reproduce the
	// driver's blend (BlendAt composed with base) exactly — the continuous
	// loop measures deviation against it, so any divergence between the two
	// implementations becomes a phantom deviation.
	{
		std::mt19937 rng(808);
		std::uniform_real_distribution<double> u(-1.0, 1.0);

		const FieldTransform base{
			Eigen::Quaterniond(Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitY())),
			Eigen::Vector3d(0.4, 0.02, -1.1) };

		double worstRot = 0.0, worstPos = 0.0;
		for (int trial = 0; trial < 24; ++trial)
		{
			size_t n = 1 + trial % 5;
			std::vector<FieldTransform> anchors;
			std::vector<Eigen::Vector3d> positions;
			std::vector<OverlayAnchor> overlay;
			for (size_t i = 0; i < n; ++i)
			{
				FieldTransform a = Compose(SmallDelta(2.5 * u(rng),
					0.06 * Eigen::Vector3d(u(rng), u(rng), u(rng))), base);
				Eigen::Vector3d pos(2.5 * u(rng), 1.2 + 0.3 * u(rng), 2.5 * u(rng));
				anchors.push_back(a);
				positions.push_back(pos);
				overlay.push_back({ pos, a.R, a.T });
			}
			protocol::SetAlignmentField f = BuildField(base, anchors, positions, 1);

			for (int k = 0; k < 8; ++k)
			{
				Eigen::Vector3d q(3.0 * u(rng), 1.2 + 0.3 * u(rng), 3.0 * u(rng));
				FieldTransform drv = DriverEffective(f, base, q);
				Eigen::Quaterniond eR;
				Eigen::Vector3d eT;
				BlendedFieldCalibration(overlay, base.R, base.T, q, eR, eT);
				worstRot = std::max(worstRot, eR.angularDistance(drv.R));
				worstPos = std::max(worstPos, (eT - drv.T).norm());
			}
		}

		bool constantsMatch =
			FieldBlendIdentityFloor == alignfield::IdentityFloorWeight &&
			FieldBlendSigmaMeters == protocol::SetAlignmentField().sigmaMeters;
		snprintf(detail, sizeof detail, "worst rot %.2e rad  pos %.2e m  constants %d",
			worstRot, worstPos, constantsMatch);
		Check("continuous: field expectation matches driver blend",
			worstRot < 1e-9 && worstPos < 1e-9 && constantsMatch, detail);
	}

	// 10. Anchor coexistence: an anchor capturing a genuine 6 cm local
	// deformation. Measured against raw base (the wiring that shipped in
	// v1.0.0) the anchor's own delta reads as a fault and freezes with the
	// mount warning; measured against the blended expectation the loop stays
	// quiet — the residual is only the identity-floor undershoot (~3 mm).
	{
		std::mt19937 rng(909);

		GroundTruth localTruth = baseTruth;
		localTruth.translation = baseTruth.translation + Eigen::Vector3d(0.06, 0.0, 0.0);
		auto localTruthAt = [&](double) { return localTruth; };

		// The anchor as StoreFieldAnchor keeps it: absolute local solve,
		// positioned at the target trajectory's centroid in reference space.
		Eigen::Vector3d anchorPos = Eigen::Vector3d::Zero();
		{
			int n = 0;
			for (double t = 0.0; t < 15.0; t += 0.1, ++n)
				anchorPos += PositionAt(t) + RotationAt(t, false) * kMountPos;
			anchorPos /= static_cast<double>(n);
		}
		std::vector<OverlayAnchor> anchors{
			{ anchorPos, localTruth.rotation, localTruth.translation } };

		ContinuousSim simBase;
		simBase.ca.SetExtrinsic(trueExtrinsic);
		simBase.solvedOffset = baseTruth.latency;
		simBase.calRot = baseTruth.rotation;
		simBase.calTrans = baseTruth.translation;
		RunContinuousSegment(simBase, scene, 0.0, 20.0, rng, localTruthAt, constMount, alwaysVisible);

		Eigen::Quaterniond expRot;
		Eigen::Vector3d expTrans;
		BlendedFieldCalibration(anchors, baseTruth.rotation, baseTruth.translation,
			anchorPos, expRot, expTrans);

		std::mt19937 rng2(910);
		ContinuousSim simField;
		simField.ca.SetExtrinsic(trueExtrinsic);
		simField.solvedOffset = baseTruth.latency;
		simField.calRot = expRot;
		simField.calTrans = expTrans;
		RunContinuousSegment(simField, scene, 0.0, 20.0, rng2, localTruthAt, constMount, alwaysVisible);

		bool baseFroze = simBase.freezes >= 1 && simBase.corrections == 0 &&
			simBase.ca.GetState() == ContinuousAlignment::State::Frozen;
		bool fieldQuiet = simField.freezes == 0 &&
			simField.ca.GetState() != ContinuousAlignment::State::Frozen &&
			simField.maxCorrPosM <= 0.011;
		snprintf(detail, sizeof detail,
			"vs base: freezes %d corr %d  vs blend: freezes %d corr %d maxCorr %.1f mm",
			simBase.freezes, simBase.corrections, simField.freezes, simField.corrections,
			simField.maxCorrPosM * 1000.0);
		Check("continuous: anchors do not read as faults", baseFroze && fieldQuiet, detail);
	}

	// 11. Degraded tracking: mid-frequency warble (grazing lighthouse geometry
	// while lying down) inflates window scatter past the gates but decorrelates
	// between consecutive observations, so the classifier calls it noise — the
	// loop must hold quietly with one informational event, never freeze with
	// the mount warning, and resume by itself once tracking settles.
	{
		std::mt19937 rng(1010);
		auto warble = [&](double t, PoseSample &s)
		{
			if (t < 20.0 || t >= 50.0)
				return;
			double a = (1.6 * EIGEN_PI / 180.0) * std::sin(2.0 * EIGEN_PI * t / 0.8);
			s.rot = (Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()))
				* s.rot).normalized();
			s.pos += Eigen::Vector3d(
				0.008 * std::sin(2.0 * EIGEN_PI * t / 0.7),
				0.008 * std::sin(2.0 * EIGEN_PI * t / 0.5 + 0.8),
				0.008 * std::sin(2.0 * EIGEN_PI * t / 0.9 + 2.0));
		};

		ContinuousSim sim;
		sim.ca.SetExtrinsic(trueExtrinsic);
		sim.solvedOffset = baseTruth.latency;
		sim.calRot = baseTruth.rotation;
		sim.calTrans = baseTruth.translation;

		bool holdingSeen = false;
		RunContinuousSegment(sim, scene, 0.0, 50.0, rng, constTruth, constMount, alwaysVisible,
			warble, false,
			[&](double t)
			{
				if (t >= 40.0 && sim.ca.GetState() == ContinuousAlignment::State::Holding)
					holdingSeen = true;
			});
		bool heldDuring = holdingSeen && sim.freezes == 0 && sim.unstables >= 1;

		// Recovery: perturb the calibration, let the warbled window age out —
		// corrections must resume with no Resumed hysteresis (that event is
		// for freezes).
		sim.calRot = (Eigen::Quaterniond(Eigen::AngleAxisd(
			0.3 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY())) * sim.calRot).normalized();
		int before = sim.corrections;
		RunContinuousSegment(sim, scene, 50.0, 70.0, rng, constTruth, constMount, alwaysVisible, warble);
		bool recovered = sim.corrections > before && sim.freezes == 0 && sim.resumes == 0 &&
			sim.ca.GetState() == ContinuousAlignment::State::Tracking;

		snprintf(detail, sizeof detail,
			"holding %d  freezes %d  unstable events %d  post-settle corr %d",
			holdingSeen, sim.freezes, sim.unstables, sim.corrections - before);
		Check("continuous: unstable tracking holds, not freezes", heldDuring && recovered, detail);
	}
}

} // namespace

int main()
{
	GroundTruth truth;
	truth.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(1.9, Eigen::Vector3d::UnitY()));   // yaw-only truth
	truth.translation = Eigen::Vector3d(1.2, 0.03, -0.7);

	EngineConfig config;

	// 1. Clean data: near-exact recovery.
	{
		SceneConfig scene;
		Expectation e;
		RunScenario("clean", scene, truth, config, e);
	}

	// 2. Realistic noise.
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.2;
		Expectation e;
		e.maxRotErrDeg = 0.5;
		e.maxTransErrM = 0.015;
		RunScenario("noise", scene, truth, config, e);
	}

	// 3. 18 ms latency on the target stream, plus noise.
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.2;
		GroundTruth t2 = truth;
		t2.latency = 0.018;
		Expectation e;
		e.maxRotErrDeg = 0.5;
		e.maxTransErrM = 0.015;
		RunScenario("latency 18ms", scene, t2, config, e);

		// Same data without compensation: document the damage (not asserted).
		EngineConfig noComp = config;
		noComp.estimateTimeOffset = false;
		Expectation e2;
		e2.expectValid = true;
		e2.maxRotErrDeg = 90.0;    // report-only run
		e2.maxTransErrM = 10.0;
		RunScenario("latency uncompensated", scene, t2, noComp, e2);
	}

	// 4. Outliers on top of noise: IRLS must hold the line.
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.2;
		scene.outlierRate = 0.02;
		Expectation e;
		e.maxRotErrDeg = 0.7;
		e.maxTransErrM = 0.02;
		RunScenario("outliers 2%", scene, truth, config, e);
	}

	// 5. Genuinely tilted target universe: rich motion must outvote the
	// gravity prior and recover the tilt.
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.2;
		GroundTruth tilted = truth;
		tilted.rotation = truth.rotation * Eigen::Quaterniond(Eigen::AngleAxisd(3.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitX()));
		Expectation e;
		e.maxRotErrDeg = 1.0;
		e.maxTransErrM = 0.02;
		RunScenario("tilted universe 3deg", scene, tilted, config, e);
	}

	// 6. Yaw-only motion: tilt/roll unconstrained; the engine must refuse
	// rather than hallucinate.
	{
		SceneConfig scene;
		scene.yawOnlyMotion = true;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.2;
		Expectation e;
		e.expectValid = false;
		e.messageContains = "one axis";
		RunScenario("yaw-only motion", scene, truth, config, e);
	}

	// 7. Playspace scale.
	{
		SceneConfig scene;
		scene.posNoise = 0.001;
		GroundTruth scaled = truth;
		scaled.scale = 1.03;
		EngineConfig sc = config;
		sc.solveScale = true;
		Expectation e;
		e.maxRotErrDeg = 0.5;
		e.maxTransErrM = 0.03;
		e.maxScaleErr = 0.005;
		RunScenario("scale 1.03", scene, scaled, sc, e);
	}

	// 8. Runtime application of the solved offset: sign and asymmetric clamp.
	// Production shape: laggy wireless reference => solved offset is negative
	// => positive shift (delay the fresh lighthouse targets). Delaying may use
	// the full range; advancing (extra prediction) is clamped tighter.
	{
		bool pass =
			std::abs(ComputeAppliedTimeOffset(-0.018) - 0.018) < 1e-12 &&
			std::abs(ComputeAppliedTimeOffset(-0.080) - 0.050) < 1e-12 &&
			std::abs(ComputeAppliedTimeOffset(+0.010) + 0.010) < 1e-12 &&
			std::abs(ComputeAppliedTimeOffset(+0.040) + 0.015) < 1e-12;
		printf("%-28s %s\n", "applied offset sign/clamp", pass ? "PASS" : "FAIL");
		if (!pass)
			failures++;
	}

	// 9. Publish-before-rewrite invariant (regression). The driver publishes
	// ring samples BEFORE shifting poseTimeOffset. If it ever published after,
	// recorded target times would absorb the applied shift, the next solve
	// would find ~zero offset, and recalibration would silently erase the
	// latency correction. Model both pipelines and assert the divergence.
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.2;
		GroundTruth t2 = truth;
		t2.latency = 0.018;

		std::vector<PoseSample> refStream, targetStream;
		GenerateStreams(scene, t2, 1234, refStream, targetStream);

		EngineResult first = CalibrationEngine::Solve(refStream, targetStream, config);
		// Symmetric clamp bounds here: the harness's laggy side is the target,
		// so the shift is negative and would hit the asymmetric advance clamp.
		double shift = ComputeAppliedTimeOffset(first.timeOffset, 0.05, 0.05);

		// Correct pipeline: ring samples are pre-rewrite, a re-solve reproduces
		// the same offset.
		EngineResult second = CalibrationEngine::Solve(refStream, targetStream, config);

		// Bugged pipeline: ring records post-rewrite poseTimeOffset, i.e. the
		// target timeline already carries the applied shift.
		std::vector<PoseSample> bugged = targetStream;
		for (auto &s : bugged)
			s.time += shift;
		EngineResult erased = CalibrationEngine::Solve(refStream, bugged, config);

		bool pass = first.valid && second.valid && erased.valid &&
			std::abs(second.timeOffset - first.timeOffset) < 0.001 &&
			std::abs(erased.timeOffset) < 0.004;
		printf("%-28s %s  first %+.1f ms  re-solve %+.1f ms  bugged-pipeline %+.1f ms\n",
			"publish-before-rewrite", pass ? "PASS" : "FAIL",
			first.timeOffset * 1000.0, second.timeOffset * 1000.0, erased.timeOffset * 1000.0);
		if (!pass)
			failures++;
	}

	// ---- Universe-jump detection ----
	RunJumpScenarios();

	// ---- Drift staleness monitoring ----
	RunDriftScenarios();

	// ---- Spatial correction field ----
	RunFieldScenarios();

	// ---- Chaperone snapshot math ----
	RunChaperoneScenarios();

	// ---- Base-transform slew (continuous calibration) ----
	RunBaseSlewScenarios();

	// ---- Continuous calibration (HMD-mounted tracker) ----
	RunContinuousScenarios();

	printf("\n%d scenario(s) failed\n", failures);
	return failures;
}
