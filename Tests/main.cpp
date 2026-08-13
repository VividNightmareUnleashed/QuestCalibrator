// Synthetic-data harness for the calibration engine.
//
// Generates two device trajectories with a known ground-truth universe
// transform, mount offset, per-system sample rates, latency, noise, and
// outliers, then asserts the solver recovers the transform within tolerance.
// Exit code is the number of failed scenarios.

#ifndef QUESTCAL_POSE_CHANNEL_TEST_SEAM
#define QUESTCAL_POSE_CHANNEL_TEST_SEAM
#endif
#ifndef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
#define QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
#endif

#include "../Driver/AlignmentField.h"
#include "../Driver/PoseScale.h"
#include "../Driver/PoseTransform.h"
#include "../Driver/ProtocolValidation.h"
#include "../Driver/IPCProtocolGate.h"
#include "../Overlay/CalibrationEngine.h"
#include "../Overlay/ChaperoneMath.h"
#include "../Overlay/ContinuousAlignment.h"
#include "../Overlay/FieldMath.h"
#include "../Overlay/DriftMonitor.h"
#include "../Overlay/JumpDetector.h"
#include "../Overlay/ProfileValidation.h"
#include "../Overlay/RingPoseMath.h"
#include "../Overlay/PoseStreamHub.h"
#include "../common/PoseChannel.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
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
	double timestampJitter = 0.0;   // seconds, 1 sigma; clamped to preserve ordering
	bool   yawOnlyMotion = false;
	double motionScale = 1.0;       // scales ALL rotation amplitudes (slow, cautious motion)
	double offAxisScale = 1.0;      // scales the pitch/roll motion amplitudes
	double offAxisRate = 1.0;       // scales the pitch/roll frequencies (small fast nods)
	// When set, the pitch/roll motion only happens inside this window (smooth
	// sin^2 envelope): "nodded briefly, then only turned" — the burst returns
	// to neutral, so long-lag pairs spanning it carry no off-axis delta.
	double offAxisBurstT0 = -1e9;
	double offAxisBurstT1 = 1e9;
};

// Smooth, rich test motion: two-axis rotation plus a figure-eight translation,
// the sort of wiggling a user actually does while calibrating.
Eigen::Quaterniond RotationAt(double t, const SceneConfig &scene)
{
	Eigen::AngleAxisd yaw(scene.motionScale * (1.1 * std::sin(0.9 * t) + 0.25 * t),
		Eigen::Vector3d::UnitY());
	if (scene.yawOnlyMotion || scene.offAxisScale <= 0.0)
		return Eigen::Quaterniond(yaw);
	double envelope = 0.0;
	if (t >= scene.offAxisBurstT0 && t <= scene.offAxisBurstT1)
	{
		double u = std::sin(EIGEN_PI * (t - scene.offAxisBurstT0)
			/ (scene.offAxisBurstT1 - scene.offAxisBurstT0));
		envelope = u * u;
	}
	double a = scene.motionScale * scene.offAxisScale * envelope;
	Eigen::AngleAxisd pitch(a * 0.9 * std::sin(scene.offAxisRate * 1.35 * t + 0.7), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd roll(a * 0.5 * std::sin(scene.offAxisRate * 1.7 * t + 2.1), Eigen::Vector3d::UnitZ());
	return Eigen::Quaterniond(yaw) * pitch * roll;
}

Eigen::Quaterniond RotationAt(double t, bool yawOnly)
{
	SceneConfig scene;
	scene.yawOnlyMotion = yawOnly;
	return RotationAt(t, scene);
}

Eigen::Vector3d PositionAt(double t)
{
	return Eigen::Vector3d(
		0.35 * std::sin(0.8 * t),
		1.25 + 0.15 * std::sin(1.4 * t + 0.4),
		0.30 * std::sin(1.6 * t));
}

// Pose of the reference device (device A) in the reference universe at time t.
void DevicePoseAt(double t, const SceneConfig &scene,
                  Eigen::Quaterniond &rot, Eigen::Vector3d &pos)
{
	rot = RotationAt(t, scene);
	pos = PositionAt(t);
}

PoseSample MakeSample(double stamp, double poseTime, const SceneConfig &scene,
                      const GroundTruth &truth, bool isTarget,
                      const Eigen::Quaterniond &mountRot, const Eigen::Vector3d &mountPos,
                      std::mt19937 &rng)
{
	Eigen::Quaterniond rotA;
	Eigen::Vector3d posA;
	DevicePoseAt(poseTime, scene, rotA, posA);

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
		DevicePoseAt(tt, scene, ra, pa);
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

void GenerateStreamsWithMount(const SceneConfig &scene, const GroundTruth &truth, uint32_t seed,
                              const Eigen::Quaterniond &mountRot,
                              const Eigen::Vector3d &mountPos,
                              std::vector<PoseSample> &refStream,
                              std::vector<PoseSample> &targetStream)
{
	std::mt19937 rng(seed);
	// normal_distribution requires sigma > 0 at construction (MSVC's debug STL
	// asserts it); substitute a dummy sigma when jitter is off — the
	// distribution is only ever sampled when timestampJitter > 0.
	std::normal_distribution<double> timestampNoise(
		0.0, scene.timestampJitter > 0.0 ? scene.timestampJitter : 1.0);

	refStream.clear();
	targetStream.clear();

	for (double t = 0.0; t < scene.duration; t += 1.0 / scene.refRate)
	{
		double stamp = scene.timestampJitter > 0.0 ? t + timestampNoise(rng) : t;
		if (!refStream.empty())
			stamp = std::max(stamp, refStream.back().time + 1e-6);
		refStream.push_back(MakeSample(stamp, stamp, scene, truth, false, mountRot, mountPos, rng));
	}

	// The target stream lags: the pose stamped t is the physical state at t - latency.
	for (double t = 0.0; t < scene.duration; t += 1.0 / scene.targetRate)
	{
		double stamp = scene.timestampJitter > 0.0 ? t + timestampNoise(rng) : t;
		if (!targetStream.empty())
			stamp = std::max(stamp, targetStream.back().time + 1e-6);
		targetStream.push_back(
			MakeSample(stamp, stamp - truth.latency, scene, truth, true, mountRot, mountPos, rng));
	}
}

void GenerateStreams(const SceneConfig &scene, const GroundTruth &truth, uint32_t seed,
                     std::vector<PoseSample> &refStream, std::vector<PoseSample> &targetStream)
{
	const Eigen::Quaterniond mountRot(
		Eigen::AngleAxisd(2.7, Eigen::Vector3d(0.2, 0.7, -0.3).normalized()));
	const Eigen::Vector3d mountPos(0.05, -0.08, 0.03);   // ~10 cm mount offset
	GenerateStreamsWithMount(
		scene, truth, seed, mountRot, mountPos, refStream, targetStream);
}

std::vector<AlignedSample> GenerateAlignedSamples(
	const SceneConfig &scene, const GroundTruth &truth, uint32_t seed,
	size_t count = 240, double step = 1.0 / 30.0)
{
	const Eigen::Quaterniond mountRot(
		Eigen::AngleAxisd(2.7, Eigen::Vector3d(0.2, 0.7, -0.3).normalized()));
	const Eigen::Vector3d mountPos(0.05, -0.08, 0.03);
	std::mt19937 rng(seed);
	std::vector<AlignedSample> aligned;
	aligned.reserve(count);
	for (size_t i = 0; i < count; ++i)
	{
		double t = static_cast<double>(i) * step;
		AlignedSample sample;
		sample.time = t;
		sample.ref = MakeSample(t, t, scene, truth, false, mountRot, mountPos, rng);
		sample.target = MakeSample(t, t, scene, truth, true, mountRot, mountPos, rng);
		aligned.push_back(sample);
	}
	return aligned;
}

// Replay the same trajectory `factor` times slower: timestamps stretch,
// velocities shrink, poses unchanged.
std::vector<PoseSample> StretchTime(const std::vector<PoseSample> &stream, double factor)
{
	std::vector<PoseSample> out = stream;
	for (auto &s : out)
	{
		s.time *= factor;
		s.vel /= factor;
		s.angVel /= factor;
	}
	return out;
}

// Zero-phase low-pass (forward+backward one-pole) over a pose stream: models
// what streamed Quest poses look like after smoothing plus the runtime's
// prediction — the prediction restores the phase (latency) but cannot restore
// the high-frequency amplitude the smoothing removed. Position and rotation
// take separate time constants (real runtimes smooth position aggressively
// while keeping orientation crisp for reprojection); velocities are filtered
// with their pose's tau so they stay consistent.
std::vector<PoseSample> SmoothStreamZeroPhase(const std::vector<PoseSample> &stream,
                                              double posTau, double rotTau)
{
	std::vector<PoseSample> out = stream;
	auto pass = [&](auto begin, auto end)
	{
		auto it = begin;
		PoseSample prev = *it;
		for (++it; it != end; ++it)
		{
			double dt = std::abs(it->time - prev.time);
			double aPos = dt / (posTau + dt);
			double aRot = dt / (rotTau + dt);
			it->pos = prev.pos + aPos * (it->pos - prev.pos);
			it->vel = prev.vel + aPos * (it->vel - prev.vel);
			it->angVel = prev.angVel + aRot * (it->angVel - prev.angVel);
			it->rot = prev.rot.slerp(aRot, it->rot);
			prev = *it;
		}
	};
	pass(out.begin(), out.end());
	pass(out.rbegin(), out.rend());
	return out;
}

struct Expectation
{
	bool expectValid = true;
	double maxRotErrDeg = 0.1;
	double maxTransErrM = 0.005;
	double maxOffsetErr = 0.004;       // checked whenever time-offset estimation is enabled
	double maxScaleErr = 0.0;          // only checked when > 0
	const char *messageContains = nullptr;
};

int failures = 0;
int checksRun = 0;

// Every reported result funnels through here. The exit code alone cannot
// distinguish "everything passed" from "nothing ran", so a deleted
// Run*Scenarios() call or an early return that skips the rest of a group
// shows up as a drop in the reported count instead of a green build.
void RecordResult(bool pass)
{
	checksRun++;
	if (!pass)
		failures++;
}

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
		if (config.estimateTimeOffset && offsetErr > expect.maxOffsetErr)
		{
			pass = false; why += " offsetErr";
		}
		if (expect.maxScaleErr > 0.0 && scaleErr > expect.maxScaleErr) { pass = false; why += " scaleErr"; }
		// solveScale is experimental and opt-in, so this is the configuration
		// essentially every user runs -- and with it off the engine must leave
		// scale at EXACTLY 1.0 (nothing on that path ever writes it). Without
		// this, a leak from the guard path, an unconditional 1-D search, or a
		// refinement writing out its nuisance scale would go unnoticed as long
		// as the composed transform stayed self-consistent, while the driver
		// multiplied every position, velocity and acceleration by it.
		if (!config.solveScale && r.scale != 1.0) { pass = false; why += " scaleLeak"; }
	}
	if (expect.messageContains && r.message.find(expect.messageContains) == std::string::npos)
	{
		pass = false;
		why += " message=\"" + r.message + "\"";
	}

	printf("%-28s %s  rot %.4f deg  trans %.4f m  offset %+.1f ms (true %+.1f)  scale %.4f  spread %.4f  cond %.4f  pairs %zu%s%s\n",
		name, pass ? "PASS" : "FAIL",
		rotErr, transErr, r.timeOffset * 1000.0, truth.latency * 1000.0,
		r.scale, r.axisSpread, r.transEigRatio, r.pairsUsed,
		why.empty() ? "" : "  <-", why.c_str());

	RecordResult(pass);
	if (!pass)
		printf("%-28s      message: %s\n", "", r.message.c_str());
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

std::string PoseRingMappingName(const char *scenario)
{
	char name[160];
	snprintf(name, sizeof name,
		"Local\\QuestCalibratorSolverTests_%s_%lu_%llu", scenario,
		static_cast<unsigned long>(GetCurrentProcessId()),
		static_cast<unsigned long long>(GetTickCount64()));
	return name;
}

struct PoseRingFixture
{
	explicit PoseRingFixture(const char *scenario) : mappingName(PoseRingMappingName(scenario)) { }

	bool Open()
	{
		return writer.Create(mappingName.c_str()) && reader.Open(mappingName.c_str());
	}

	std::string mappingName;
	protocol::PoseRingWriter writer;
	protocol::PoseRingReader reader;
};

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
	RecordResult(pass);
}

Eigen::Quaterniond RandomQuaternion(std::mt19937 &rng, double maxAngle = EIGEN_PI)
{
	std::uniform_real_distribution<double> u(-1.0, 1.0);
	std::uniform_real_distribution<double> angle(-maxAngle, maxAngle);
	Eigen::Vector3d axis;
	do
	{
		axis = Eigen::Vector3d(u(rng), u(rng), u(rng));
	} while (axis.squaredNorm() < 1e-8);
	return Eigen::Quaterniond(Eigen::AngleAxisd(angle(rng), axis.normalized()));
}

Eigen::Vector3d RandomVector(std::mt19937 &rng, double magnitude)
{
	std::uniform_real_distribution<double> u(-magnitude, magnitude);
	return Eigen::Vector3d(u(rng), u(rng), u(rng));
}

void RunDriverPoseTransformScenarios()
{
	std::mt19937 rng(0xD12E4u);
	std::uniform_real_distribution<double> scaleDist(0.85, 1.15);
	std::uniform_real_distribution<double> timeDist(-0.05, 0.05);

	double worstWorld = 0.0;
	double worstVelocity = 0.0;
	double worstRotation = 0.0;
	double worstTime = 0.0;
	double worstGeometry = 0.0;
	bool untouched = true;

	for (int trial = 0; trial < 256; ++trial)
	{
		Eigen::Quaterniond calR = RandomQuaternion(rng);
		Eigen::Vector3d calT = RandomVector(rng, 3.0);
		Eigen::Quaterniond wfdR = RandomQuaternion(rng);
		Eigen::Vector3d wfdT = RandomVector(rng, 2.0);
		Eigen::Quaterniond localR = RandomQuaternion(rng);
		Eigen::Vector3d localP = RandomVector(rng, 2.0);
		Eigen::Vector3d velocity = RandomVector(rng, 5.0);
		Eigen::Vector3d acceleration = RandomVector(rng, 8.0);
		Eigen::Vector3d angularVelocity = RandomVector(rng, 4.0);
		double scale = scaleDist(rng);
		double timeShift = timeDist(rng);
		double initialTime = timeDist(rng);

		// A real device's tracked-origin-to-head offset. It must be non-zero
		// here: leaving it at zero makes the deliberate decision below
		// unfalsifiable in either direction.
		Eigen::Vector3d driverFromHead = RandomVector(rng, 0.4);

		vr::DriverPose_t pose{};
		pose.qWorldFromDriverRotation = { wfdR.w(), wfdR.x(), wfdR.y(), wfdR.z() };
		pose.qRotation = { localR.w(), localR.x(), localR.y(), localR.z() };
		pose.poseTimeOffset = initialTime;
		for (int i = 0; i < 3; ++i)
		{
			pose.vecWorldFromDriverTranslation[i] = wfdT(i);
			pose.vecPosition[i] = localP(i);
			pose.vecVelocity[i] = velocity(i);
			pose.vecAcceleration[i] = acceleration(i);
			pose.vecAngularVelocity[i] = angularVelocity(i);
			pose.vecDriverFromHeadTranslation[i] = driverFromHead(i);
		}

		const Eigen::Vector3d expectedWorld =
			calR * (scale * (wfdR * localP + wfdT)) + calT;
		const Eigen::Quaterniond expectedWorldRotation = calR * wfdR * localR;
		const Eigen::Vector3d expectedVelocity = scale * velocity;
		const Eigen::Vector3d expectedAcceleration = scale * acceleration;
		double translation[3] = { calT.x(), calT.y(), calT.z() };
		vr::HmdQuaternion_t rotation{ calR.w(), calR.x(), calR.y(), calR.z() };

		questcal::driverpose::Apply(pose, rotation, translation, scale, timeShift);

		Eigen::Quaterniond actualWfd(
			pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x,
			pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z);
		Eigen::Quaterniond actualLocal(
			pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z);
		Eigen::Vector3d actualPosition(
			pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]);
		Eigen::Vector3d actualWfdT(
			pose.vecWorldFromDriverTranslation[0],
			pose.vecWorldFromDriverTranslation[1],
			pose.vecWorldFromDriverTranslation[2]);
		Eigen::Vector3d actualWorld = actualWfd * actualPosition + actualWfdT;
		Eigen::Quaterniond actualWorldRotation = actualWfd * actualLocal;
		Eigen::Vector3d actualVelocity(
			pose.vecVelocity[0], pose.vecVelocity[1], pose.vecVelocity[2]);
		Eigen::Vector3d actualAcceleration(
			pose.vecAcceleration[0], pose.vecAcceleration[1], pose.vecAcceleration[2]);
		Eigen::Vector3d actualAngularVelocity(
			pose.vecAngularVelocity[0], pose.vecAngularVelocity[1], pose.vecAngularVelocity[2]);

		worstWorld = std::max(worstWorld, (actualWorld - expectedWorld).norm());
		worstVelocity = std::max(worstVelocity,
			std::max((actualVelocity - expectedVelocity).norm(),
				(actualAcceleration - expectedAcceleration).norm()));
		worstRotation = std::max(worstRotation,
			actualWorldRotation.angularDistance(expectedWorldRotation));
		worstTime = std::max(worstTime,
			std::abs(pose.poseTimeOffset - (initialTime + timeShift)));
		// Deliberately NOT scaled: `scale` reconciles two tracking systems'
		// universe scales, while this offset is the device's real physical
		// geometry and stays real-size. Asserted exactly, because the decision
		// is only safe while the solver's model makes the same assumption -
		// scaling it here without changing the model is a silent bias.
		Eigen::Vector3d actualDriverFromHead(
			pose.vecDriverFromHeadTranslation[0],
			pose.vecDriverFromHeadTranslation[1],
			pose.vecDriverFromHeadTranslation[2]);
		worstGeometry = std::max(worstGeometry,
			(actualDriverFromHead - driverFromHead).norm());

		untouched = untouched &&
			actualLocal.angularDistance(localR) < 1e-12 &&
			(actualAngularVelocity - angularVelocity).norm() < 1e-12;
	}

	char detail[256];
	snprintf(detail, sizeof detail,
		"world %.2e  deriv %.2e  rot %.2e rad  time %.2e  geometry %.2e  angular/local %d",
		worstWorld, worstVelocity, worstRotation, worstTime, worstGeometry, untouched);
	Check("driver: randomized transform oracle",
		worstWorld < 1e-11 && worstVelocity < 1e-11 &&
		worstRotation < 1e-11 && worstTime < 1e-12 &&
		worstGeometry == 0.0 && untouched, detail);
}

void RunDriverProtocolValidationScenarios()
{
	using questcal::driverinput::ValidateAndSanitize;
	const double nan = std::numeric_limits<double>::quiet_NaN();
	const double inf = std::numeric_limits<double>::infinity();

	protocol::SetDeviceTransform good;
	good.openVRID = 3;
	good.enabled = true;
	good.translation = { { 1.0, -2.0, 3.0 } };
	good.rotation = { 2.0, 0.0, 0.0, 0.0 };
	good.scale = 1.25;
	good.timeOffset = -0.055;
	protocol::SetDeviceTransform sanitized;
	bool pass = ValidateAndSanitize(good, sanitized) && sanitized.openVRID == 3 &&
		std::abs(sanitized.rotation.w - 1.0) < 1e-12 && sanitized.scale == good.scale;

	auto rejectsTransform = [&](protocol::SetDeviceTransform candidate)
	{
		protocol::SetDeviceTransform ignored;
		return !ValidateAndSanitize(candidate, ignored);
	};
	protocol::SetDeviceTransform badTransform = good;
	badTransform.openVRID = vr::k_unMaxTrackedDeviceCount;
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.translation.v[0] = nan;
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.translation.v[1] = 10001.0;
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.rotation.w = inf;
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.rotation = { 0.0, 0.0, 0.0, 0.0 };
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.scale = 0.24;
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.scale = 4.01;
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.scale = nan;
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.timeOffset = 1.01;
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.timeOffset = inf;
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.enabled = 2;
	pass = pass && rejectsTransform(badTransform);
	badTransform = good; badTransform.hidden = 2;
	pass = pass && rejectsTransform(badTransform);

	protocol::SetAlignmentField goodField;
	goodField.enabled = true;
	goodField.anchorCount = 1;
	goodField.sigmaMeters = 1.5;
	goodField.anchors[0].position[0] = 2.0;
	goodField.anchors[0].rotationDelta = { 0.0, 3.0, 0.0, 0.0 };
	goodField.anchors[0].translationDelta[2] = -0.2;
	protocol::SetAlignmentField sanitizedField;
	pass = pass && ValidateAndSanitize(goodField, sanitizedField) &&
		std::abs(sanitizedField.anchors[0].rotationDelta.x - 1.0) < 1e-12;

	auto rejectsField = [&](protocol::SetAlignmentField candidate)
	{
		protocol::SetAlignmentField ignored;
		return !ValidateAndSanitize(candidate, ignored);
	};
	protocol::SetAlignmentField badField = goodField;
	badField.anchorCount = protocol::SetAlignmentField::MaxAnchors + 1;
	pass = pass && rejectsField(badField);
	badField = goodField; badField.sigmaMeters = 0.01;
	pass = pass && rejectsField(badField);
	badField = goodField; badField.sigmaMeters = 101.0;
	pass = pass && rejectsField(badField);
	badField = goodField; badField.sigmaMeters = nan;
	pass = pass && rejectsField(badField);
	badField = goodField; badField.anchors[0].position[1] = inf;
	pass = pass && rejectsField(badField);
	badField = goodField; badField.anchors[0].position[2] = 10001.0;
	pass = pass && rejectsField(badField);
	badField = goodField; badField.anchors[0].translationDelta[0] = 100.01;
	pass = pass && rejectsField(badField);
	badField = goodField; badField.anchors[0].rotationDelta = { 0.0, 0.0, 0.0, 0.0 };
	pass = pass && rejectsField(badField);
	badField = goodField; badField.enabled = 2;
	pass = pass && rejectsField(badField);

	// Unused wire anchors are deliberately scrubbed instead of trusted.
	protocol::SetAlignmentField unusedGarbage;
	unusedGarbage.anchorCount = 0;
	unusedGarbage.anchors[0].position[0] = nan;
	protocol::SetAlignmentField scrubbed;
	pass = pass && ValidateAndSanitize(unusedGarbage, scrubbed) &&
		scrubbed.anchors[0].position[0] == 0.0 && scrubbed.anchors[0].rotationDelta.w == 1.0;

	Check("driver: protocol validation", pass, "finite/range/quaternion/unused-anchor matrix");

	questcal::ipc::ConnectionState connection;
	protocol::Response response;
	protocol::Request mutation(protocol::RequestSetDeviceTransform);
	bool preHandshake = questcal::ipc::PrepareRequest(mutation, connection, response);
	bool preHandshakeRejected = !preHandshake && response.type == protocol::ResponseInvalid;

	protocol::Request wrongHandshake(protocol::RequestHandshake);
	wrongHandshake.protocol.version = protocol::Version - 1;
	bool wrongDispatched = questcal::ipc::PrepareRequest(wrongHandshake, connection, response);
	bool wrongRejected = !wrongDispatched && !connection.handshakeComplete &&
		response.type == protocol::ResponseHandshake &&
		response.protocol.version == protocol::Version;

	protocol::Request handshake(protocol::RequestHandshake);
	bool handshakeDispatched = questcal::ipc::PrepareRequest(handshake, connection, response);
	bool handshakeAccepted = !handshakeDispatched && connection.handshakeComplete &&
		response.type == protocol::ResponseHandshake;
	bool mutationAccepted = questcal::ipc::PrepareRequest(mutation, connection, response);
	mutation.protocol.version = protocol::Version - 1;
	bool staleMutation = questcal::ipc::PrepareRequest(mutation, connection, response);

	Check("driver: per-connection protocol gate",
		preHandshakeRejected &&
		wrongRejected && handshakeAccepted && mutationAccepted && !staleMutation,
		"pre-handshake/wrong-handshake/current/stale mutation matrix");
}

void RunPoseSampleScenarios()
{
	// Ring consumers reject finite-but-implausible values before Eigen
	// composition. These are numerically finite yet large enough to overflow
	// downstream squared norms/products.
	protocol::DevicePoseSample bounded = RingSample(2, 1.0,
		Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
		Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.1, 1.0, -0.2),
		Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d(0.0, 0.2, 0.0));
	protocol::DevicePoseSample extremePosition = bounded;
	extremePosition.position[0] = 1e300;
	protocol::DevicePoseSample extremeVelocity = bounded;
	extremeVelocity.velocity[1] = 1e300;
	protocol::DevicePoseSample extremeAngularVelocity = bounded;
	extremeAngularVelocity.angularVelocity[2] = 1e300;
	Check("pose ring: finite range boundary",
		IsUsableRingSample(bounded, TestQpcToSeconds) &&
		!IsUsableRingSample(extremePosition, TestQpcToSeconds) &&
		!IsUsableRingSample(extremeVelocity, TestQpcToSeconds) &&
		!IsUsableRingSample(extremeAngularVelocity, TestQpcToSeconds) &&
		!IsUsableRingSample(bounded, 1e300),
		"bounded accepted; extreme pos/vel/ang/time rejected");

	// Backlog drained after a UI stall is fresh only if its producer capture
	// QPC is fresh; the UI observation time and pose prediction offset are not
	// the capture time. A positive prediction offset must not reject fresh data.
	protocol::DevicePoseSample predictedSample{};
	predictedSample.sampleTimeQpc = 1000000000;
	predictedSample.poseTimeOffset = 0.05;
	Check("pose ring: capture timestamp freshness",
		ringpose::IsFreshCaptureTime(100.0, 100.0, 2.0) &&
		ringpose::IsFreshCaptureTime(100.0, 102.0, 2.0) &&
		!ringpose::IsFreshCaptureTime(100.0, 102.001, 2.0) &&
		!ringpose::IsFreshCaptureTime(100.001, 100.0, 2.0) &&
		!ringpose::IsFreshCaptureTime(
			std::numeric_limits<double>::quiet_NaN(), 100.0, 2.0),
		"current/boundary accepted; stale/future/non-finite rejected");
	Check("pose ring: capture freshness ignores prediction offset",
		ringpose::IsFreshCaptureTime(
			RingCaptureTime(predictedSample, TestQpcToSeconds), 100.0, 2.0) &&
		RingSampleTime(predictedSample, TestQpcToSeconds) > 100.0,
		"fresh capture accepted even when the pose validity time is predicted ahead");

	char detail[256];

	// The composition every solver input goes through: worldFromDriver o driver
	// pose, velocities rotated into the world frame, and the driver's own
	// prediction offset folded into the timestamp. Expectations are literals
	// rather than the same expression re-evaluated, so a rewritten composition
	// (dropped rotation on the velocities, inverted worldFromDriver, reversed
	// rotation product, dropped poseTimeOffset) shows up here.
	{
		const Eigen::Quaterniond wfdRot(
			Eigen::AngleAxisd(EIGEN_PI / 2.0, Eigen::Vector3d::UnitY()));
		const Eigen::Quaterniond drvRot(
			Eigen::AngleAxisd(EIGEN_PI / 2.0, Eigen::Vector3d::UnitX()));
		protocol::DevicePoseSample s = RingSample(4, 2.0,
			wfdRot, Eigen::Vector3d(1.0, 2.0, 3.0),
			drvRot, Eigen::Vector3d(1.0, 0.0, 0.0),
			Eigen::Vector3d(0.0, 0.0, 2.0), Eigen::Vector3d(0.0, 1.0, 0.0));
		s.poseTimeOffset = 0.05;

		PoseSample composed;
		bool accepted = TryComposeRingSample(s, TestQpcToSeconds, composed);
		// +90 deg about Y sends +x to -z and +z to +x; the composed rotation is
		// checked by its action on +z, which distinguishes wfd o drv from
		// drv o wfd (the reversed product leaves +z at +x).
		bool ok = accepted &&
			std::abs(composed.time - 2.05) < 1e-9 &&
			(composed.pos - Eigen::Vector3d(1.0, 2.0, 2.0)).norm() < 1e-9 &&
			(composed.vel - Eigen::Vector3d(2.0, 0.0, 0.0)).norm() < 1e-9 &&
			(composed.angVel - Eigen::Vector3d(0.0, 1.0, 0.0)).norm() < 1e-9 &&
			(composed.rot * Eigen::Vector3d(0.0, 0.0, 1.0) -
				Eigen::Vector3d(0.0, -1.0, 0.0)).norm() < 1e-9 &&
			std::abs(composed.rot.norm() - 1.0) < 1e-12;
		snprintf(detail, sizeof detail,
			"t %.4f  pos (%.3f, %.3f, %.3f)  vel (%.3f, %.3f, %.3f)",
			composed.time, composed.pos.x(), composed.pos.y(), composed.pos.z(),
			composed.vel.x(), composed.vel.y(), composed.vel.z());
		Check("pose ring: composed world sample", ok, detail);
	}

	// The trust boundary: a driver may mark a pose Running_OK while supplying
	// malformed numerics, and it may supply clean numerics while reporting the
	// device as not tracking. Both are tracking ABSENCE, so the composed output
	// must be left exactly as the caller had it -- softening a rejection into a
	// defaulted (identity, origin) pose would feed the solver a fabricated
	// sample that passes every downstream check.
	{
		const Eigen::Quaterniond wfdRot(
			Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
		protocol::DevicePoseSample good = RingSample(4, 3.0,
			wfdRot, Eigen::Vector3d(0.5, 0.0, -0.5),
			Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.2, 1.1, -0.3),
			Eigen::Vector3d(0.4, 0.0, 0.0), Eigen::Vector3d(0.0, 0.3, 0.0));

		PoseSample sentinel;
		sentinel.time = -77.0;
		sentinel.pos = Eigen::Vector3d(9.0, 9.0, 9.0);

		auto rejectsAndPreserves = [&](const protocol::DevicePoseSample &bad)
		{
			PoseSample out = sentinel;
			return !TryComposeRingSample(bad, TestQpcToSeconds, out) &&
				out.time == sentinel.time && out.pos == sentinel.pos;
		};

		protocol::DevicePoseSample notValid = good;
		notValid.poseIsValid = false;
		protocol::DevicePoseSample notTracking = good;
		notTracking.trackingResult =
			static_cast<uint32_t>(vr::TrackingResult_Running_OutOfRange);
		protocol::DevicePoseSample malformedNumeric = good;
		malformedNumeric.position[1] = 1e300;
		protocol::DevicePoseSample degenerateRotation = good;
		degenerateRotation.rotation = { 0.0, 0.0, 0.0, 0.0 };
		// Every field is individually inside the ring bounds, yet the sum is not:
		// the driver offset and the device position are each 9 km, so the
		// composed world position lands at 18 km. This is the case only the
		// second (composed) gate can catch, so it fails if that gate is dropped
		// as redundant with the per-field one.
		protocol::DevicePoseSample overflowingComposition = good;
		overflowingComposition.worldFromDriverRotation = { 1.0, 0.0, 0.0, 0.0 };
		overflowingComposition.worldFromDriverTranslation[0] = 9000.0;
		overflowingComposition.position[0] = 9000.0;

		PoseSample accepted;
		bool ok = TryComposeRingSample(good, TestQpcToSeconds, accepted) &&
			IsTrustedRingSample(good, TestQpcToSeconds) &&
			rejectsAndPreserves(notValid) &&
			rejectsAndPreserves(notTracking) &&
			rejectsAndPreserves(malformedNumeric) &&
			rejectsAndPreserves(degenerateRotation) &&
			IsTrustedRingSample(overflowingComposition, TestQpcToSeconds) &&
			rejectsAndPreserves(overflowingComposition) &&
			!IsTrustedRingSample(notTracking, TestQpcToSeconds);
		Check("pose ring: ingestion trust boundary", ok,
			"healthy accepted; invalid/not-tracking/unbounded/degenerate/overflowing rejected without writing out");
	}

	// Drift-feed eligibility. The reference side contributes the HMD only (its
	// SLAM map IS the universe); the target side contributes everything rigid to
	// it EXCEPT the HMD-mounted tracker, which rides a human head. Losing that
	// one clause is what puts a resting head into the drift feed and toasts the
	// user about a calibration that is being actively maintained.
	{
		ringpose::DriftFeedCandidate c;
		// Stale HMD observation so the proximity heuristic cannot mask the
		// identity rules being checked here.
		c.composedTime = 100.0;
		c.hmdRawTime = 90.0;
		c.mountedTrackerId = 9;

		auto eligible = [&](uint32_t id, bool referenceSide, bool targetSide)
		{
			ringpose::DriftFeedCandidate q = c;
			q.deviceId = id;
			q.referenceSide = referenceSide;
			q.targetSide = targetSide;
			return ringpose::AnchorsUniverse(q);
		};

		ringpose::DriftFeedCandidate unresolvedTracker = c;
		unresolvedTracker.deviceId = 9;
		unresolvedTracker.targetSide = true;
		unresolvedTracker.mountedTrackerId = vr::k_unTrackedDeviceIndexInvalid;

		bool ok =
			eligible(vr::k_unTrackedDeviceIndex_Hmd, true, false) &&
			!eligible(3, true, false) &&          // set-down reference controller
			eligible(5, false, true) &&           // lighthouse device, target side
			!eligible(9, false, true) &&          // the HMD-mounted tracker
			!eligible(4, false, false) &&         // neither system
			ringpose::AnchorsUniverse(unresolvedTracker);
		Check("drift feed: universe anchors", ok,
			"HMD-only on the reference side; target side minus the mounted tracker");
	}

	// The worn-device proximity heuristic, and the placement of the calibrated
	// scale inside it: a target-raw position has to be scaled BEFORE the base
	// rotation and translation, exactly as the driver applies it. At scale 1.5 a
	// tracker 3 m out sits at 4.5 m in reference space -- dropping the factor
	// puts it 1.5 m away from the headset it is actually strapped to, back over
	// the arm's-reach threshold and into the drift feed.
	{
		ringpose::DriftFeedCandidate c;
		c.deviceId = 5;
		c.targetSide = true;
		c.mountedTrackerId = vr::k_unTrackedDeviceIndexInvalid;
		c.rawPosition = Eigen::Vector3d(3.0, 0.0, 0.0);
		c.calibratedScale = 1.5;
		c.composedTime = 100.0;
		c.hmdRawTime = 99.0;

		ringpose::DriftFeedCandidate wornOnUser = c;
		wornOnUser.hmdRawPosition = Eigen::Vector3d(4.5, 0.0, 0.0);

		// Where the unscaled (wrong) spelling would place the device.
		ringpose::DriftFeedCandidate acrossTheRoom = c;
		acrossTheRoom.hmdRawPosition = Eigen::Vector3d(3.0, 0.0, 0.0);

		// Same worn device, but the HMD has not been seen for longer than the
		// proximity window, so the heuristic must not keep excluding it.
		ringpose::DriftFeedCandidate staleHmd = wornOnUser;
		staleHmd.hmdRawTime = 96.0;

		bool ok =
			!ringpose::AnchorsUniverse(wornOnUser) &&
			ringpose::AnchorsUniverse(acrossTheRoom) &&
			ringpose::AnchorsUniverse(staleHmd);
		snprintf(detail, sizeof detail,
			"scale %.2f raw %.1f m -> reference %.1f m; window %.1f s",
			c.calibratedScale, c.rawPosition.x(),
			ringpose::BaseCalibratedPosition(c.calibratedRotation,
				c.calibratedTranslationMeters, c.calibratedScale, c.rawPosition).x(),
			ringpose::DriftHmdProximityWindowSeconds);
		Check("drift feed: worn-device proximity", ok, detail);
	}
}

void RunPoseRingConcurrentScenario()
{
	// Exercise the actual named shared-memory implementation with concurrent
	// publishers. Every payload carries redundant token fields so a torn or
	// mismatched sample is distinguishable from a merely missing one.
	const int producerCount = 4;
	const int samplesPerProducer = 200;
	const int total = producerCount * samplesPerProducer;

	PoseRingFixture ring("Concurrent");
	auto &writer = ring.writer;
	auto &reader = ring.reader;
	bool opened = ring.Open();
	if (!opened)
	{
		Check("pose ring: concurrent integrity", false, "could not create/open mapping");
		return;
	}

	std::unique_ptr<std::atomic<int>[]> seen(new std::atomic<int>[total]);
	for (int i = 0; i < total; ++i)
		seen[i].store(0, std::memory_order_relaxed);
	std::atomic<int> producersDone{ 0 };
	std::atomic<int> received{ 0 };
	std::atomic<int> corrupt{ 0 };
	std::atomic<int> publishRetries{ 0 };

	std::thread consumer([&]()
	{
		auto consume = [&](const protocol::DevicePoseSample &sample)
		{
			int token = static_cast<int>(sample.sampleTimeQpc - 1);
			bool valid = token >= 0 && token < total &&
				sample.deviceId == static_cast<uint32_t>(token / samplesPerProducer) &&
				sample.position[0] == static_cast<double>(token) &&
				sample.position[1] == static_cast<double>(-token) &&
				sample.rotation.w == 1.0;
			if (!valid)
				corrupt.fetch_add(1, std::memory_order_relaxed);
			else
				seen[token].fetch_add(1, std::memory_order_relaxed);
			received.fetch_add(1, std::memory_order_relaxed);
		};
		ULONGLONG started = GetTickCount64();
		while (producersDone.load(std::memory_order_acquire) < producerCount ||
		       received.load(std::memory_order_relaxed) < total)
		{
			reader.Drain(consume);
			if (received.load(std::memory_order_relaxed) >= total)
				break;
			if (producersDone.load(std::memory_order_acquire) == producerCount &&
			    GetTickCount64() - started > 5000)
				break;
			Sleep(0);
		}
		// The timeout break can race a producer's final publishes; one more
		// drain keeps a starved run from miscounting that tail as missing.
		reader.Drain(consume);
	});

	std::vector<std::thread> producers;
	producers.reserve(producerCount);
	for (int producer = 0; producer < producerCount; ++producer)
	{
		producers.emplace_back([&, producer]()
		{
			for (int i = 0; i < samplesPerProducer; ++i)
			{
				int token = producer * samplesPerProducer + i;
				protocol::DevicePoseSample sample{};
				sample.sampleTimeQpc = static_cast<int64_t>(token + 1);
				sample.deviceId = static_cast<uint32_t>(producer);
				sample.poseIsValid = true;
				sample.deviceIsConnected = true;
				sample.rotation.w = 1.0;
				sample.position[0] = static_cast<double>(token);
				sample.position[1] = static_cast<double>(-token);
				// This scenario checks payload ownership/integrity, not the
				// separately-tested fail-fast policy. Retry an intentional claim-lock
				// contention drop so a preempted producer cannot make CI flaky.
				while (!writer.Publish(sample))
				{
					publishRetries.fetch_add(1, std::memory_order_relaxed);
					Sleep(0);
				}
			}
			producersDone.fetch_add(1, std::memory_order_release);
		});
	}

	for (auto &producer : producers)
		producer.join();
	consumer.join();

	int missing = 0;
	int duplicate = 0;
	for (int i = 0; i < total; ++i)
	{
		int count = seen[i].load(std::memory_order_relaxed);
		if (count == 0) ++missing;
		if (count > 1) duplicate += count - 1;
	}

	char detail[192];
	snprintf(detail, sizeof detail,
		"received %d/%d missing %d duplicate %d corrupt %d retries %d",
		received.load(), total, missing, duplicate, corrupt.load(), publishRetries.load());
	Check("pose ring: concurrent integrity",
		received.load() == total && missing == 0 && duplicate == 0 && corrupt.load() == 0,
		detail);
}

void RunPoseRingDrainStatusScenario()
{
	PoseRingFixture ring("DrainStatus");
	bool opened = ring.Open();
	protocol::PoseRingReader::DrainStatus resetStatus =
		protocol::PoseRingReader::DrainStatus::WriterDead;
	protocol::PoseRingReader::DrainStatus deadStatus =
		protocol::PoseRingReader::DrainStatus::Drained;
	if (opened)
	{
		ring.reader.SetResetInProgressForTest(true);
		resetStatus = ring.reader.Drain([](const protocol::DevicePoseSample &) { });
		ring.reader.SetResetInProgressForTest(false);
		ring.writer.Close();
		deadStatus = ring.reader.Drain([](const protocol::DevicePoseSample &) { });
	}
	Check("pose ring: drain status",
		opened && resetStatus == protocol::PoseRingReader::DrainStatus::ResetInProgress &&
		deadStatus == protocol::PoseRingReader::DrainStatus::WriterDead,
		"reset gate and writer death distinguished");
}

void RunPoseRingOverflowScenario()
{
	// Let the bounded queue overflow without a reader. Producers safely reclaim
	// completed oldest slots, so the newest full ring remains available without
	// any reader/writer payload race.
	PoseRingFixture ring("Overflow");
	auto &writer = ring.writer;
	auto &reader = ring.reader;
	if (!ring.Open())
	{
		Check("pose ring: overwrite recovery", false, "could not create/open mapping");
		return;
	}

	const uint64_t overflowExtra = 257;
	for (uint64_t i = 0; i < protocol::PoseRing::Capacity + overflowExtra; ++i)
	{
		protocol::DevicePoseSample sample{};
		sample.sampleTimeQpc = static_cast<int64_t>(1000000 + i);
		sample.deviceId = 7;
		sample.rotation.w = 1.0;
		sample.position[0] = static_cast<double>(i);
		writer.Publish(sample);
	}
	int overflowCount = 0;
	int overflowCorrupt = 0;
	uint64_t overflowGapDrops = 0;
	bool overflowGapBeforeFirst = false;
	int64_t firstToken = -1;
	int64_t lastToken = -1;
	reader.Drain(
		[&](const protocol::DevicePoseSample &sample)
		{
			int64_t token = sample.sampleTimeQpc - 1000000;
			if (overflowCount == 0)
				firstToken = token;
			lastToken = token;
			if (sample.deviceId != 7 || sample.rotation.w != 1.0 ||
				sample.position[0] != static_cast<double>(token))
				++overflowCorrupt;
			++overflowCount;
		},
		[&](uint64_t count)
		{
			overflowGapBeforeFirst = overflowCount == 0;
			overflowGapDrops += count;
		});
	int expectedRetained = static_cast<int>(protocol::PoseRing::Capacity);
	char detail[192];
	snprintf(detail, sizeof detail,
		"retained %d/%d first %lld last %lld corrupt %d positional %llu/%d",
		overflowCount, expectedRetained,
		static_cast<long long>(firstToken), static_cast<long long>(lastToken),
		overflowCorrupt,
		static_cast<unsigned long long>(overflowGapDrops), overflowGapBeforeFirst);
	Check("pose ring: overwrite recovery",
		overflowCount == expectedRetained &&
		firstToken == static_cast<int64_t>(overflowExtra) &&
		lastToken == static_cast<int64_t>(protocol::PoseRing::Capacity + overflowExtra - 1) &&
		overflowCorrupt == 0 &&
		overflowGapDrops == overflowExtra && overflowGapBeforeFirst, detail);
}

void RunPoseRingStalledProducerScenario()
{
	// Pause one producer after it owns the head slot, then fill the rest of the
	// queue. No later producer may lap and overwrite that owned payload; once
	// full, new poses must fail fast until the owner resumes.
	PoseRingFixture ring("Stalled");
	auto &stalledWriter = ring.writer;
	auto &stalledReader = ring.reader;
	bool stalledOpened = ring.Open();
	if (!stalledOpened)
	{
		Check("pose ring: stalled producer ownership", false, "could not create/open mapping");
		return;
	}
	char detail[192];

	std::atomic<bool> claimHeld{ false };
	std::atomic<bool> releaseClaim{ false };
	protocol::DevicePoseSample headSample{};
	headSample.sampleTimeQpc = 2000000;
	headSample.deviceId = 9;
	headSample.rotation.w = 1.0;
	headSample.position[0] = 0.0;
	std::thread stalled([&]()
	{
		stalledWriter.PublishAfterClaimForTest(headSample, [&]()
		{
			claimHeld.store(true, std::memory_order_release);
			while (!releaseClaim.load(std::memory_order_acquire))
				Sleep(0);
		});
	});

	ULONGLONG waitStart = GetTickCount64();
	while (!claimHeld.load(std::memory_order_acquire) && GetTickCount64() - waitStart < 5000)
		Sleep(0);
	bool held = claimHeld.load(std::memory_order_acquire);
	int fillSucceeded = 0;
	if (held)
	{
		for (uint64_t i = 1; i < protocol::PoseRing::Capacity; ++i)
		{
			protocol::DevicePoseSample sample{};
			sample.sampleTimeQpc = static_cast<int64_t>(2000000 + i);
			sample.deviceId = 9;
			sample.rotation.w = 1.0;
			sample.position[0] = static_cast<double>(i);
			if (stalledWriter.Publish(sample))
				++fillSucceeded;
		}
	}

	const int rejectedAttempts = 128;
	int rejected = 0;
	for (int i = 0; held && i < rejectedAttempts; ++i)
	{
		protocol::DevicePoseSample sample{};
		sample.sampleTimeQpc = 3000000 + i;
		sample.rotation.w = 1.0;
		if (!stalledWriter.Publish(sample))
			++rejected;
	}
	int consumedWhileHeld = 0;
	stalledReader.Drain([&](const protocol::DevicePoseSample &) { ++consumedWhileHeld; });
	releaseClaim.store(true, std::memory_order_release);
	stalled.join();

	int stalledConsumed = 0;
	int stalledCorrupt = 0;
	uint64_t gapAfterPrefix = 0;
	bool terminalGapPosition = false;
	stalledReader.Drain(
		[&](const protocol::DevicePoseSample &sample)
		{
			int64_t token = sample.sampleTimeQpc - 2000000;
			if (token != stalledConsumed || sample.deviceId != 9 || sample.rotation.w != 1.0 ||
				sample.position[0] != static_cast<double>(token))
				++stalledCorrupt;
			++stalledConsumed;
		},
		[&](uint64_t count)
		{
			terminalGapPosition = stalledConsumed == static_cast<int>(protocol::PoseRing::Capacity);
			gapAfterPrefix += count;
		});

	protocol::DevicePoseSample afterGap{};
	afterGap.sampleTimeQpc = 5000000;
	afterGap.deviceId = 9;
	afterGap.rotation.w = 1.0;
	afterGap.position[0] = 500.0;
	bool afterGapPublished = stalledWriter.Publish(afterGap);
	uint64_t positionalTailDrops = 0;
	bool gapBeforeTailSample = false;
	int tailSamples = 0;
	stalledReader.Drain(
		[&](const protocol::DevicePoseSample &sample)
		{
			++tailSamples;
			if (sample.sampleTimeQpc != afterGap.sampleTimeQpc)
				++stalledCorrupt;
		},
		[&](uint64_t count)
		{
			gapBeforeTailSample = tailSamples == 0;
			positionalTailDrops += count;
		});
	snprintf(detail, sizeof detail,
		"held %d fill %d/%llu rejected %d/%d pre-read %d drained %d corrupt %d terminalGap %llu/%d tail %d/%llu/%d",
		held, fillSucceeded,
		static_cast<unsigned long long>(protocol::PoseRing::Capacity - 1),
		rejected, rejectedAttempts, consumedWhileHeld, stalledConsumed, stalledCorrupt,
		static_cast<unsigned long long>(gapAfterPrefix), terminalGapPosition, tailSamples,
		static_cast<unsigned long long>(positionalTailDrops), gapBeforeTailSample);
	Check("pose ring: stalled producer ownership",
		held && fillSucceeded == static_cast<int>(protocol::PoseRing::Capacity - 1) &&
		rejected == rejectedAttempts && consumedWhileHeld == 0 &&
		stalledConsumed == static_cast<int>(protocol::PoseRing::Capacity) &&
		stalledCorrupt == 0 &&
		gapAfterPrefix == rejectedAttempts && terminalGapPosition &&
		afterGapPublished && tailSamples == 1 && positionalTailDrops == 0 &&
		!gapBeforeTailSample,
		detail);
}

void RunPoseRingOverwrittenMarkerScenario()
{
	// A failed-publish marker can be attached to a later sample and then have
	// that carrier sample overwritten before the reader reaches it. The prefix
	// loss ledger must preserve both the carrier and its marker.
	PoseRingFixture ring("MarkerOverwrite");
	auto &markerWriter = ring.writer;
	auto &markerReader = ring.reader;
	bool markerOpened = ring.Open();
	char detail[192];
	std::atomic<bool> markerClaimHeld{ false };
	std::atomic<bool> releaseMarkerClaim{ false };
	std::thread markerStalled;
	bool markerHeld = false;
	int markerFill = 0;
	const int markerFailures = 11;
	int markerRejected = 0;
	if (markerOpened)
	{
		protocol::DevicePoseSample markerHead{};
		markerHead.sampleTimeQpc = 10000000;
		markerHead.rotation.w = 1.0;
		markerStalled = std::thread([&]()
		{
			markerWriter.PublishAfterClaimForTest(markerHead, [&]()
			{
				markerClaimHeld.store(true, std::memory_order_release);
				while (!releaseMarkerClaim.load(std::memory_order_acquire))
					Sleep(0);
			});
		});
		ULONGLONG markerWaitStart = GetTickCount64();
		while (!markerClaimHeld.load(std::memory_order_acquire) &&
			GetTickCount64() - markerWaitStart < 5000)
			Sleep(0);
		markerHeld = markerClaimHeld.load(std::memory_order_acquire);
		for (uint64_t i = 1; markerHeld && i < protocol::PoseRing::Capacity; ++i)
		{
			protocol::DevicePoseSample sample{};
			sample.sampleTimeQpc = 10000000 + static_cast<int64_t>(i);
			sample.rotation.w = 1.0;
			if (markerWriter.Publish(sample))
				++markerFill;
		}
		for (int i = 0; markerHeld && i < markerFailures; ++i)
		{
			protocol::DevicePoseSample sample{};
			sample.sampleTimeQpc = 10100000 + i;
			sample.rotation.w = 1.0;
			if (!markerWriter.Publish(sample))
				++markerRejected;
		}
	}
	releaseMarkerClaim.store(true, std::memory_order_release);
	if (markerStalled.joinable())
		markerStalled.join();

	const int64_t markerTailBase = 11000000;
	int markerTailPublished = 0;
	for (uint64_t i = 0; markerHeld && i <= protocol::PoseRing::Capacity; ++i)
	{
		protocol::DevicePoseSample sample{};
		sample.sampleTimeQpc = markerTailBase + static_cast<int64_t>(i);
		sample.deviceId = 14;
		sample.rotation.w = 1.0;
		if (markerWriter.Publish(sample))
			++markerTailPublished;
	}
	int markerRetained = 0;
	bool markerOrdered = true;
	bool markerGapBeforeFirst = false;
	uint64_t markerGapTotal = 0;
	markerReader.Drain(
		[&](const protocol::DevicePoseSample &sample)
		{
			int64_t expected = markerTailBase + markerRetained + 1;
			if (sample.sampleTimeQpc != expected || sample.deviceId != 14 ||
				sample.rotation.w != 1.0)
				markerOrdered = false;
			++markerRetained;
		},
		[&](uint64_t count)
		{
			markerGapBeforeFirst = markerRetained == 0;
			markerGapTotal += count;
		});
	uint64_t markerExpectedLoss = protocol::PoseRing::Capacity + 1 + markerFailures;
	snprintf(detail, sizeof detail,
		"opened %d held %d fill %d rejected %d tail %d retained %d ordered %d gap %llu/%llu positional %d",
		markerOpened, markerHeld, markerFill, markerRejected, markerTailPublished,
		markerRetained, markerOrdered, static_cast<unsigned long long>(markerGapTotal),
		static_cast<unsigned long long>(markerExpectedLoss), markerGapBeforeFirst);
	Check("pose ring: overwritten marker accounting",
		markerOpened && markerHeld &&
		markerFill == static_cast<int>(protocol::PoseRing::Capacity - 1) &&
		markerRejected == markerFailures &&
		markerTailPublished == static_cast<int>(protocol::PoseRing::Capacity + 1) &&
		markerRetained == static_cast<int>(protocol::PoseRing::Capacity) && markerOrdered &&
		markerGapBeforeFirst && markerGapTotal == markerExpectedLoss,
		detail);
}

void RunPoseRingAbandonedWriterScenario()
{
	// Simulate vrserver dying after claiming (but before publishing) the head
	// slot while the overlay keeps the named mapping alive. A replacement writer
	// must start a clean session instead of inheriting an unreclaimable head.
	std::string reopenMappingName = PoseRingMappingName("Reopen");
	protocol::PoseRingWriter abandonedWriter;
	protocol::PoseRingReader survivingReader;
	bool reopenOpened = abandonedWriter.Create(reopenMappingName.c_str()) &&
		survivingReader.Open(reopenMappingName.c_str());
	char detail[192];
	uint64_t abandonedEpoch = survivingReader.SessionEpoch();
	bool abandoned = false;
	if (reopenOpened)
	{
		protocol::DevicePoseSample abandonedSample{};
		try
		{
			abandonedWriter.PublishAfterClaimForTest(abandonedSample, []()
			{
				throw std::runtime_error("simulated producer death");
			});
		}
		catch (const std::runtime_error &)
		{
			abandoned = true;
		}
	}
	abandonedWriter.Close();

	protocol::PoseRingWriter replacementWriter;
	bool replacementOpened = reopenOpened && replacementWriter.Create(reopenMappingName.c_str());
	uint64_t replacementEpoch = survivingReader.SessionEpoch();
	protocol::DevicePoseSample replacementSample{};
	replacementSample.sampleTimeQpc = 4000000;
	replacementSample.deviceId = 11;
	replacementSample.rotation.w = 1.0;
	replacementSample.position[0] = 42.0;
	bool replacementPublished = replacementOpened && replacementWriter.Publish(replacementSample);
	int replacementReceived = 0;
	bool replacementValid = false;
	survivingReader.Drain([&](const protocol::DevicePoseSample &sample)
	{
		++replacementReceived;
		replacementValid = sample.sampleTimeQpc == replacementSample.sampleTimeQpc &&
			sample.deviceId == replacementSample.deviceId &&
			sample.position[0] == replacementSample.position[0];
	});
	snprintf(detail, sizeof detail,
		"opened %d abandoned %d replacement %d epoch %llu->%llu published %d received %d valid %d",
		reopenOpened, abandoned, replacementOpened,
		static_cast<unsigned long long>(abandonedEpoch),
		static_cast<unsigned long long>(replacementEpoch), replacementPublished,
		replacementReceived, replacementValid);
	Check("pose ring: abandoned writer restart",
		reopenOpened && abandoned && replacementOpened && replacementPublished &&
		replacementEpoch == abandonedEpoch + 1 && replacementReceived == 1 && replacementValid,
		detail);
}

void RunPoseRingAbandonedResetOwnerScenario()
{
	// A retained mapping can also outlive a writer that dies midway through the
	// reset itself. The named writer/reset mutex must serialize a live owner,
	// become abandoned with its worker thread, and let exactly one replacement
	// clear the inherited resetting=1 gate. The replacement's live-writer
	// metadata must then reject another contender.
	std::string resetCrashMappingName = PoseRingMappingName("ResetCrash");
	protocol::PoseRingWriter resetCrashSeedWriter;
	protocol::PoseRingReader resetCrashReader;
	bool resetCrashCreated = resetCrashSeedWriter.Create(resetCrashMappingName.c_str()) &&
		resetCrashReader.Open(resetCrashMappingName.c_str());
	char detail[192];
	uint64_t resetCrashOldEpoch = resetCrashReader.SessionEpoch();
	resetCrashSeedWriter.Close();

	std::atomic<HANDLE> abandonedResetMutex{ nullptr };
	std::atomic<bool> resetOwnerReady{ false };
	std::atomic<bool> releaseResetOwner{ false };
	std::thread resetOwner;
	if (resetCrashCreated)
	{
		resetOwner = std::thread([&]()
		{
			HANDLE mutex = protocol::PoseRingWriter::AcquireResetOwnershipForTest(
				resetCrashMappingName.c_str());
			abandonedResetMutex.store(mutex, std::memory_order_release);
			resetOwnerReady.store(true, std::memory_order_release);
			while (mutex != nullptr &&
				!releaseResetOwner.load(std::memory_order_acquire))
				Sleep(0);
			if (mutex != nullptr)
			{
				// Intentionally do not call ReleaseMutex. Another handle is already
				// waiting below, so thread exit marks the persistent object abandoned.
				CloseHandle(mutex);
			}
		});
	}
	ULONGLONG resetOwnerDeadline = GetTickCount64() + 5000;
	while (resetCrashCreated && !resetOwnerReady.load(std::memory_order_acquire) &&
		GetTickCount64() < resetOwnerDeadline)
		Sleep(0);
	HANDLE liveResetMutex = abandonedResetMutex.load(std::memory_order_acquire);
	bool liveResetOwnerObserved = liveResetMutex != nullptr &&
		WaitForSingleObject(liveResetMutex, 0) == WAIT_TIMEOUT;

	protocol::PoseRingWriter resetCrashReplacement;
	std::atomic<bool> resetContenderStarted{ false };
	std::atomic<bool> resetContenderDone{ false };
	bool resetCrashReplacementOpened = false;
	std::thread resetContender;
	if (liveResetOwnerObserved)
	{
		resetContender = std::thread([&]()
		{
			resetContenderStarted.store(true, std::memory_order_release);
			resetCrashReplacementOpened = resetCrashReplacement.Create(
				resetCrashMappingName.c_str());
			resetContenderDone.store(true, std::memory_order_release);
		});
	}
	resetOwnerDeadline = GetTickCount64() + 5000;
	while (liveResetOwnerObserved &&
		!resetContenderStarted.load(std::memory_order_acquire) &&
		GetTickCount64() < resetOwnerDeadline)
		Sleep(0);
	Sleep(20);
	bool resetContenderSerialized = liveResetOwnerObserved &&
		resetContenderStarted.load(std::memory_order_acquire) &&
		!resetContenderDone.load(std::memory_order_acquire);
	releaseResetOwner.store(true, std::memory_order_release);
	if (resetOwner.joinable())
		resetOwner.join();
	if (resetContender.joinable())
		resetContender.join();

	uint64_t resetCrashNewEpoch = resetCrashReader.SessionEpoch();
	protocol::PoseRingWriter resetCrashLiveContender;
	bool resetCrashLiveContenderRejected = resetCrashReplacementOpened &&
		!resetCrashLiveContender.Create(resetCrashMappingName.c_str());
	protocol::DevicePoseSample resetCrashSample{};
	resetCrashSample.sampleTimeQpc = 4100000;
	resetCrashSample.deviceId = 12;
	resetCrashSample.rotation.w = 1.0;
	resetCrashSample.position[0] = 43.0;
	bool resetCrashPublished = resetCrashReplacementOpened &&
		resetCrashReplacement.Publish(resetCrashSample);
	int resetCrashReceived = 0;
	resetCrashReader.Drain([&](const protocol::DevicePoseSample &sample)
	{
		if (sample.sampleTimeQpc == resetCrashSample.sampleTimeQpc &&
			sample.deviceId == resetCrashSample.deviceId &&
			sample.position[0] == resetCrashSample.position[0])
			++resetCrashReceived;
	});
	snprintf(detail, sizeof detail,
		"created %d owner %d serialized %d replacement %d epoch %llu->%llu liveRejected %d published %d received %d",
		resetCrashCreated, liveResetOwnerObserved, resetContenderSerialized,
		resetCrashReplacementOpened,
		static_cast<unsigned long long>(resetCrashOldEpoch),
		static_cast<unsigned long long>(resetCrashNewEpoch),
		resetCrashLiveContenderRejected, resetCrashPublished, resetCrashReceived);
	Check("pose ring: abandoned reset-owner recovery",
		resetCrashCreated && liveResetOwnerObserved && resetContenderSerialized &&
		resetCrashReplacementOpened &&
		resetCrashNewEpoch == resetCrashOldEpoch + 1 &&
		resetCrashLiveContenderRejected && resetCrashPublished &&
		resetCrashReceived == 1,
		detail);
}

void RunPoseRingOpenResetRaceScenario()
{
	// Reset exactly across Reader::Open's read-gate release. Open must retain
	// the epoch paired with its old discarded-loss snapshot; loading the epoch
	// after releasing activeReaders can pair old loss=5 with the reset epoch and
	// suppress the first five real losses in the replacement session.
	std::string openRaceMappingName = PoseRingMappingName("OpenReset");
	protocol::PoseRingWriter openRaceOldWriter;
	char detail[192];
	bool openRaceCreated = openRaceOldWriter.Create(openRaceMappingName.c_str());
	int oldRacePublished = 0;
	for (uint64_t i = 0; openRaceCreated &&
		i < protocol::PoseRing::Capacity + 5; ++i)
	{
		protocol::DevicePoseSample sample{};
		sample.sampleTimeQpc = 14000000 + static_cast<int64_t>(i);
		sample.rotation.w = 1.0;
		if (openRaceOldWriter.Publish(sample))
			++oldRacePublished;
	}

	protocol::PoseRingReader openRaceReader;
	protocol::PoseRingWriter openRaceReplacement;
	protocol::PoseRingWriter competingReplacement;
	std::atomic<bool> openInsideGate{ false };
	std::atomic<bool> releaseOpenGate{ false };
	std::atomic<bool> replacementDone{ false };
	std::atomic<bool> competingAttemptStarted{ false };
	bool openRaceOpened = false;
	bool openRaceReplacementOpened = false;
	bool competingReplacementOpened = false;
	std::thread openingReader;
	std::thread resettingWriter;
	std::thread competingWriter;
	if (openRaceCreated)
	{
		openingReader = std::thread([&]()
		{
			openRaceOpened = openRaceReader.OpenWithGateHooksForTest(
				openRaceMappingName.c_str(),
				[&]()
				{
					openInsideGate.store(true, std::memory_order_release);
					while (!releaseOpenGate.load(std::memory_order_acquire))
						Sleep(0);
				},
				[&]()
				{
					ULONGLONG deadline = GetTickCount64() + 5000;
					while (!replacementDone.load(std::memory_order_acquire) &&
						GetTickCount64() < deadline)
						Sleep(0);
				});
		});
	}
	ULONGLONG openRaceDeadline = GetTickCount64() + 5000;
	while (openRaceCreated && !openInsideGate.load(std::memory_order_acquire) &&
		GetTickCount64() < openRaceDeadline)
		Sleep(0);
	bool openGateReached = openInsideGate.load(std::memory_order_acquire);
	if (openGateReached)
	{
		openRaceOldWriter.Close();
		resettingWriter = std::thread([&]()
		{
			openRaceReplacementOpened = openRaceReplacement.Create(openRaceMappingName.c_str());
			replacementDone.store(true, std::memory_order_release);
		});
		openRaceDeadline = GetTickCount64() + 5000;
		while (!openRaceReader.ResetInProgressForTest() &&
			GetTickCount64() < openRaceDeadline)
			Sleep(0);
	}
	bool resetWaitObserved = openGateReached && openRaceReader.ResetInProgressForTest();
	if (resetWaitObserved)
	{
		// Keep the active reader gate held while a second replacement races the
		// first one. Exactly one writer may own and reset a retained mapping.
		competingWriter = std::thread([&]()
		{
			competingAttemptStarted.store(true, std::memory_order_release);
			competingReplacementOpened = competingReplacement.Create(openRaceMappingName.c_str());
		});
		openRaceDeadline = GetTickCount64() + 5000;
		while (!competingAttemptStarted.load(std::memory_order_acquire) &&
			GetTickCount64() < openRaceDeadline)
			Sleep(0);
		// Let the contender reach the already-held reset gate before releasing
		// the reader. The failed claim is immediate in the corrected protocol.
		Sleep(20);
	}
	releaseOpenGate.store(true, std::memory_order_release);
	if (openingReader.joinable())
		openingReader.join();
	if (resettingWriter.joinable())
		resettingWriter.join();
	if (competingWriter.joinable())
		competingWriter.join();

	protocol::DevicePoseSample openRaceSeed{};
	openRaceSeed.sampleTimeQpc = 15000000;
	openRaceSeed.deviceId = 17;
	openRaceSeed.rotation.w = 1.0;
	bool openRaceSeedPublished = openRaceReplacementOpened &&
		openRaceReplacement.Publish(openRaceSeed);
	int openRaceSeedReceived = 0;
	uint64_t openRaceSeedDrops = 0;
	openRaceReader.Drain(
		[&](const protocol::DevicePoseSample &sample)
		{
			if (sample.sampleTimeQpc == openRaceSeed.sampleTimeQpc)
				++openRaceSeedReceived;
		},
		[&](uint64_t count) { openRaceSeedDrops += count; });
	int openRaceTailPublished = 0;
	for (uint64_t i = 0; openRaceSeedReceived == 1 &&
		i < protocol::PoseRing::Capacity + 1; ++i)
	{
		protocol::DevicePoseSample sample{};
		sample.sampleTimeQpc = 15100000 + static_cast<int64_t>(i);
		sample.deviceId = 17;
		sample.rotation.w = 1.0;
		if (openRaceReplacement.Publish(sample))
			++openRaceTailPublished;
	}
	int openRaceTailReceived = 0;
	uint64_t openRaceTailDrops = 0;
	openRaceReader.Drain(
		[&](const protocol::DevicePoseSample &) { ++openRaceTailReceived; },
		[&](uint64_t count) { openRaceTailDrops += count; });
	snprintf(detail, sizeof detail,
		"created %d old %d gate %d reset %d open %d replacement %d contender %d seed %d/%llu tail %d/%d gaps %llu",
		openRaceCreated, oldRacePublished, openGateReached, resetWaitObserved,
		openRaceOpened, openRaceReplacementOpened, competingReplacementOpened,
		openRaceSeedReceived,
		static_cast<unsigned long long>(openRaceSeedDrops), openRaceTailPublished,
		openRaceTailReceived, static_cast<unsigned long long>(openRaceTailDrops));
	Check("pose ring: reset during reader open",
		openRaceCreated &&
		oldRacePublished == static_cast<int>(protocol::PoseRing::Capacity + 5) &&
		openGateReached && resetWaitObserved && openRaceOpened && openRaceReplacementOpened &&
		!competingReplacementOpened &&
		openRaceSeedPublished && openRaceSeedReceived == 1 && openRaceSeedDrops == 0 &&
		openRaceTailPublished == static_cast<int>(protocol::PoseRing::Capacity + 1) &&
		openRaceTailReceived == static_cast<int>(protocol::PoseRing::Capacity) &&
		openRaceTailDrops == 1,
		detail);
}

void RunPoseHubTerminalGapScenario()
{
	// Exercise the complete producer -> shared-memory reader -> hub drain thread
	// -> consumer path. A full queue blocked behind an in-flight head records
	// failed tail publishes after the older prefix. The hub must retain the
	// standalone terminal marker even though no later sample exists yet.
	std::string hubMappingName = PoseRingMappingName("Hub");
	protocol::PoseRingWriter hubWriter;
	char detail[192];
	bool hubOpened = hubWriter.Create(hubMappingName.c_str());
	std::atomic<bool> hubClaimHeld{ false };
	std::atomic<bool> releaseHubClaim{ false };
	std::thread hubStalled;
	const int64_t hubPrefixBase = 6000000;
	int hubFillSucceeded = 0;
	int hubRejected = 0;
	const int hubRejectedAttempts = 7;
	bool hubHeld = false;
	if (hubOpened)
	{
		protocol::DevicePoseSample hubHead{};
		hubHead.sampleTimeQpc = hubPrefixBase;
		hubHead.deviceId = 12;
		hubHead.rotation.w = 1.0;
		hubStalled = std::thread([&]()
		{
			hubWriter.PublishAfterClaimForTest(hubHead, [&]()
			{
				hubClaimHeld.store(true, std::memory_order_release);
				while (!releaseHubClaim.load(std::memory_order_acquire))
					Sleep(0);
			});
		});

		ULONGLONG hubWaitStart = GetTickCount64();
		while (!hubClaimHeld.load(std::memory_order_acquire) &&
			GetTickCount64() - hubWaitStart < 5000)
			Sleep(0);
		hubHeld = hubClaimHeld.load(std::memory_order_acquire);
		if (hubHeld)
		{
			for (uint64_t i = 1; i < protocol::PoseRing::Capacity; ++i)
			{
				protocol::DevicePoseSample sample{};
				sample.sampleTimeQpc = hubPrefixBase + static_cast<int64_t>(i);
				sample.deviceId = 12;
				sample.rotation.w = 1.0;
				if (hubWriter.Publish(sample))
					++hubFillSucceeded;
			}
			for (int i = 0; i < hubRejectedAttempts; ++i)
			{
				protocol::DevicePoseSample sample{};
				sample.sampleTimeQpc = 7000000 + i;
				sample.deviceId = 12;
				sample.rotation.w = 1.0;
				if (!hubWriter.Publish(sample))
					++hubRejected;
			}
		}
		releaseHubClaim.store(true, std::memory_order_release);
		hubStalled.join();
	}

	PoseStreamHub hub;
	int hubConsumer = hub.CreateConsumer();
	if (hubOpened && hubHeld)
		hub.Start(hubMappingName.c_str());
	ULONGLONG hubDeadline = GetTickCount64() + 5000;
	while (hubOpened && hubHeld && !hub.RingOpen() && GetTickCount64() < hubDeadline)
		Sleep(1);

	std::vector<protocol::DevicePoseSample> hubOut;
	int hubPrefixCount = 0;
	bool hubPrefixValid = true;
	bool hubTerminal = false;
	uint64_t hubTerminalDrops = 0;
	while (hubOpened && hubHeld && GetTickCount64() < hubDeadline && !hubTerminal)
	{
		uint64_t dropped = hub.Drain(hubConsumer, hubOut);
		for (const auto &sample : hubOut)
		{
			int64_t token = sample.sampleTimeQpc - hubPrefixBase;
			if (token != hubPrefixCount || sample.deviceId != 12 || sample.rotation.w != 1.0)
				hubPrefixValid = false;
			++hubPrefixCount;
		}
		if (dropped != 0)
		{
			hubTerminalDrops += dropped;
			hubTerminal = hubOut.empty() &&
				hubPrefixCount == static_cast<int>(protocol::PoseRing::Capacity);
		}
		if (!hubTerminal)
			Sleep(1);
	}

	protocol::DevicePoseSample hubAfterGap{};
	hubAfterGap.sampleTimeQpc = 8000000;
	hubAfterGap.deviceId = 12;
	hubAfterGap.rotation.w = 1.0;
	bool hubAfterPublished = hubTerminal && hubWriter.Publish(hubAfterGap);
	bool hubResumed = false;
	uint64_t hubAfterDrops = 0;
	hubDeadline = GetTickCount64() + 5000;
	while (hubAfterPublished && GetTickCount64() < hubDeadline && !hubResumed)
	{
		hubAfterDrops += hub.Drain(hubConsumer, hubOut);
		hubResumed = hubOut.size() == 1 &&
			hubOut[0].sampleTimeQpc == hubAfterGap.sampleTimeQpc;
		if (!hubResumed)
			Sleep(1);
	}
	bool hubWasOpen = hub.RingOpen();
	hub.Stop();

	snprintf(detail, sizeof detail,
		"opened %d held %d fill %d/%llu rejected %d/%d prefix %d valid %d terminal %llu/%d resumed %d postDrops %llu",
		hubOpened, hubHeld, hubFillSucceeded,
		static_cast<unsigned long long>(protocol::PoseRing::Capacity - 1),
		hubRejected, hubRejectedAttempts, hubPrefixCount, hubPrefixValid,
		static_cast<unsigned long long>(hubTerminalDrops), hubTerminal,
		hubResumed, static_cast<unsigned long long>(hubAfterDrops));
	Check("pose hub: positional terminal gap",
		hubOpened && hubHeld && hubWasOpen &&
		hubFillSucceeded == static_cast<int>(protocol::PoseRing::Capacity - 1) &&
		hubRejected == hubRejectedAttempts && hubPrefixValid &&
		hubPrefixCount == static_cast<int>(protocol::PoseRing::Capacity) &&
		hubTerminal && hubTerminalDrops == hubRejectedAttempts &&
		hubAfterPublished && hubResumed && hubAfterDrops == 0,
		detail);
}

void RunPoseHubMarkerOverflowScenario()
{
	// Gap markers occupy history slots but are not themselves pose samples.
	// When a lagging consumer loses two samples plus an intervening marker, the
	// result is exactly two history losses plus the marker's source-drop count.
	PoseStreamHub overflowHub;
	char detail[192];
	std::vector<protocol::DevicePoseSample> hubOut;
	int overflowConsumer = overflowHub.CreateConsumer();
	protocol::DevicePoseSample overflowSample{};
	overflowSample.deviceId = 13;
	overflowSample.rotation.w = 1.0;
	overflowSample.sampleTimeQpc = 9000000;
	overflowHub.AppendSampleForTest(overflowSample);
	overflowSample.sampleTimeQpc++;
	overflowHub.AppendSampleForTest(overflowSample);
	overflowHub.AppendGapForTest(5);
	const int64_t retainedBase = 9100000;
	for (uint64_t i = 0; i < PoseStreamHub::HistoryCapacity; ++i)
	{
		overflowSample.sampleTimeQpc = retainedBase + static_cast<int64_t>(i);
		overflowHub.AppendSampleForTest(overflowSample);
	}
	uint64_t overflowHubDrops = overflowHub.Drain(overflowConsumer, hubOut);
	bool overflowHubSamples = hubOut.size() == PoseStreamHub::HistoryCapacity &&
		hubOut.front().sampleTimeQpc == retainedBase &&
		hubOut.back().sampleTimeQpc ==
			retainedBase + static_cast<int64_t>(PoseStreamHub::HistoryCapacity - 1);
	snprintf(detail, sizeof detail, "drops %llu/7 retained %zu/%llu ordered %d",
		static_cast<unsigned long long>(overflowHubDrops), hubOut.size(),
		static_cast<unsigned long long>(PoseStreamHub::HistoryCapacity), overflowHubSamples);
	Check("pose hub: marker overflow accounting",
		overflowHubDrops == 7 && overflowHubSamples, detail);
}

void RunPoseHubConsumerIndependenceScenario()
{
	PoseStreamHub hub;
	int firstConsumer = hub.CreateConsumer();
	int secondConsumer = hub.CreateConsumer();
	protocol::DevicePoseSample sample{};
	sample.deviceId = 18;
	sample.rotation.w = 1.0;
	auto append = [&](int64_t token)
	{
		sample.sampleTimeQpc = token;
		hub.AppendSampleForTest(sample);
	};

	append(1);
	append(2);
	std::vector<protocol::DevicePoseSample> out;
	uint64_t firstInitialDrops = hub.Drain(firstConsumer, out);
	bool firstInitial = firstInitialDrops == 0 && out.size() == 2 &&
		out[0].sampleTimeQpc == 1 && out[1].sampleTimeQpc == 2;

	// The second consumer skips only its own backlog. Both consumers must still
	// observe the next source gap immediately before the next sample.
	hub.DiscardBacklog(secondConsumer);
	hub.AppendGapForTest(3);
	append(3);
	uint64_t firstGapDrops = hub.Drain(firstConsumer, out);
	bool firstAfterDiscard = firstGapDrops == 3 && out.size() == 1 &&
		out[0].sampleTimeQpc == 3;
	uint64_t secondGapDrops = hub.Drain(secondConsumer, out);
	bool secondAfterDiscard = secondGapDrops == 3 && out.size() == 1 &&
		out[0].sampleTimeQpc == 3;

	append(4);
	append(5);
	uint64_t firstPrefixDrops = hub.Drain(firstConsumer, out);
	bool firstPrefix = firstPrefixDrops == 0 && out.size() == 2 &&
		out[0].sampleTimeQpc == 4 && out[1].sampleTimeQpc == 5;
	hub.AppendGapForTest(2);
	append(6);
	uint64_t secondPrefixDrops = hub.Drain(secondConsumer, out);
	bool secondPrefix = secondPrefixDrops == 0 && out.size() == 2 &&
		out[0].sampleTimeQpc == 4 && out[1].sampleTimeQpc == 5;
	uint64_t firstTailDrops = hub.Drain(firstConsumer, out);
	bool firstTail = firstTailDrops == 2 && out.size() == 1 &&
		out[0].sampleTimeQpc == 6;
	uint64_t secondTailDrops = hub.Drain(secondConsumer, out);
	bool secondTail = secondTailDrops == 2 && out.size() == 1 &&
		out[0].sampleTimeQpc == 6;

	char detail[192];
	snprintf(detail, sizeof detail,
		"initial %d first/second gap %d/%d prefix %d/%d tail %d/%d",
		firstInitial, firstAfterDiscard, secondAfterDiscard, firstPrefix,
		secondPrefix, firstTail, secondTail);
	Check("pose hub: consumers + discard",
		firstInitial && firstAfterDiscard && secondAfterDiscard && firstPrefix &&
		secondPrefix && firstTail && secondTail,
		detail);
}

void RunPoseHubMidDrainOverflowScenario()
{
	// Drain copies in bounded chunks so the producer thread can keep appending.
	// If those appends overwrite the consumer between chunks, return the already
	// copied prefix alone and leave the hole unacknowledged. The next call must
	// then report that hole immediately before the surviving suffix.
	PoseStreamHub midDrainHub;
	char detail[192];
	std::vector<protocol::DevicePoseSample> hubOut;
	int midDrainConsumer = midDrainHub.CreateConsumer();
	protocol::DevicePoseSample midDrainSample{};
	midDrainSample.deviceId = 15;
	midDrainSample.rotation.w = 1.0;
	const int64_t midDrainBase = 12000000;
	for (uint64_t i = 0; i < PoseStreamHub::HistoryCapacity; ++i)
	{
		midDrainSample.sampleTimeQpc = midDrainBase + static_cast<int64_t>(i);
		midDrainHub.AppendSampleForTest(midDrainSample);
	}
	bool overflowInjected = false;
	const uint64_t injectedSamples = 1024;
	midDrainHub.SetDrainChunkHookForTest([&]()
	{
		if (overflowInjected)
			return;
		overflowInjected = true;
		for (uint64_t i = 0; i < injectedSamples; ++i)
		{
			midDrainSample.sampleTimeQpc = midDrainBase +
				static_cast<int64_t>(PoseStreamHub::HistoryCapacity + i);
			midDrainHub.AppendSampleForTest(midDrainSample);
		}
	});
	uint64_t prefixDrops = midDrainHub.Drain(midDrainConsumer, hubOut);
	size_t prefixCount = hubOut.size();
	bool prefixExact = prefixCount == 512 &&
		hubOut.front().sampleTimeQpc == midDrainBase &&
		hubOut.back().sampleTimeQpc == midDrainBase + 511;
	midDrainHub.SetDrainChunkHookForTest({});
	uint64_t suffixDrops = midDrainHub.Drain(midDrainConsumer, hubOut);
	bool suffixExact = hubOut.size() == PoseStreamHub::HistoryCapacity &&
		hubOut.front().sampleTimeQpc == midDrainBase + static_cast<int64_t>(injectedSamples) &&
		hubOut.back().sampleTimeQpc == midDrainBase +
			static_cast<int64_t>(PoseStreamHub::HistoryCapacity + injectedSamples - 1);
	snprintf(detail, sizeof detail,
		"injected %d prefix %d/%zu drops %llu suffix %d/%zu drops %llu",
		overflowInjected, prefixExact, prefixCount,
		static_cast<unsigned long long>(prefixDrops), suffixExact, hubOut.size(),
		static_cast<unsigned long long>(suffixDrops));
	Check("pose hub: mid-drain overflow position",
		overflowInjected && prefixExact && prefixDrops == 0 && suffixExact &&
		suffixDrops == injectedSamples - 512,
		detail);
}

void RunPoseHubWriterLivenessScenario()
{
	// A quiet writer is alive even during SteamVR standby, but a terminated
	// writer must make RingOpen fall promptly. Simulate the hardest stale-owner
	// case: the PID was reused by a live process but its creation time differs.
	// The hub must disconnect, reopen a replacement writer, and resume after one
	// session boundary.
	std::string livenessMappingName = PoseRingMappingName("Liveness");
	DWORD staleProcessId = GetCurrentProcessId();
	uint64_t staleCreationTime = std::numeric_limits<uint64_t>::max();
	protocol::PoseRingWriter livenessWriter;
	char detail[192];
	std::vector<protocol::DevicePoseSample> hubOut;
	bool livenessCreated = livenessWriter.Create(livenessMappingName.c_str());
	PoseStreamHub livenessHub;
	int livenessConsumer = livenessHub.CreateConsumer();
	if (livenessCreated)
		livenessHub.Start(livenessMappingName.c_str());
	ULONGLONG livenessDeadline = GetTickCount64() + 5000;
	while (livenessCreated && !livenessHub.RingOpen() &&
		GetTickCount64() < livenessDeadline)
		Sleep(1);
	bool initiallyAlive = livenessHub.RingOpen();
	if (initiallyAlive)
		livenessWriter.AbandonForTest(staleProcessId, staleCreationTime);
	livenessDeadline = GetTickCount64() + 5000;
	while (initiallyAlive && livenessHub.RingOpen() &&
		GetTickCount64() < livenessDeadline)
		Sleep(1);
	bool disappearanceDetected = initiallyAlive && !livenessHub.RingOpen();

	protocol::PoseRingWriter replacementLivenessWriter;
	bool livenessRecreated = disappearanceDetected &&
		replacementLivenessWriter.Create(livenessMappingName.c_str());
	livenessDeadline = GetTickCount64() + 5000;
	while (livenessRecreated && !livenessHub.RingOpen() &&
		GetTickCount64() < livenessDeadline)
		Sleep(1);
	bool reopenedAlive = livenessHub.RingOpen();
	protocol::DevicePoseSample livenessSample{};
	livenessSample.sampleTimeQpc = 13000000;
	livenessSample.deviceId = 16;
	livenessSample.rotation.w = 1.0;
	bool livenessPublished = reopenedAlive &&
		replacementLivenessWriter.Publish(livenessSample);
	uint64_t livenessDrops = 0;
	bool livenessReceived = false;
	livenessDeadline = GetTickCount64() + 5000;
	while (livenessPublished && !livenessReceived && GetTickCount64() < livenessDeadline)
	{
		livenessDrops += livenessHub.Drain(livenessConsumer, hubOut);
		for (const auto &sample : hubOut)
		{
			if (sample.sampleTimeQpc == livenessSample.sampleTimeQpc &&
				sample.deviceId == livenessSample.deviceId)
				livenessReceived = true;
		}
		if (!livenessReceived)
			Sleep(1);
	}
	livenessHub.Stop();
	snprintf(detail, sizeof detail,
		"reusedPid %lu created %d initial %d disappeared %d recreated %d reopened %d published %d received %d gaps %llu",
		static_cast<unsigned long>(staleProcessId), livenessCreated, initiallyAlive,
		disappearanceDetected, livenessRecreated, reopenedAlive, livenessPublished,
		livenessReceived, static_cast<unsigned long long>(livenessDrops));
	Check("pose hub: writer liveness restart",
		livenessCreated && initiallyAlive && disappearanceDetected &&
		livenessRecreated && reopenedAlive && livenessPublished && livenessReceived &&
		livenessDrops == 1,
		detail);
}

void RunPoseHubLiveResetGateScenario()
{
	std::string mappingName = PoseRingMappingName("HubLiveReset");
	protocol::PoseRingWriter writer;
	protocol::PoseRingReader resetController;
	bool created = writer.Create(mappingName.c_str());
	bool controllerOpened = created && resetController.Open(mappingName.c_str());
	PoseStreamHub hub;
	int consumer = hub.CreateConsumer();
	if (controllerOpened)
		hub.Start(mappingName.c_str());
	ULONGLONG deadline = GetTickCount64() + 5000;
	while (controllerOpened && !hub.RingOpen() && GetTickCount64() < deadline)
		Sleep(1);
	bool initiallyOpen = hub.RingOpen();
	if (initiallyOpen)
		resetController.SetResetInProgressForTest(true);
	deadline = GetTickCount64() + 5000;
	while (initiallyOpen && hub.ResetDeferralsForTest() == 0 &&
		GetTickCount64() < deadline)
		Sleep(1);
	bool resetObserved = hub.ResetDeferralsForTest() != 0;
	bool mappingRetained = resetObserved && hub.RingOpen();
	if (initiallyOpen)
		resetController.SetResetInProgressForTest(false);

	protocol::DevicePoseSample sample{};
	sample.sampleTimeQpc = 16000000;
	sample.deviceId = 19;
	sample.rotation.w = 1.0;
	bool published = mappingRetained && writer.Publish(sample);
	std::vector<protocol::DevicePoseSample> out;
	uint64_t drops = 0;
	bool received = false;
	deadline = GetTickCount64() + 5000;
	while (published && !received && GetTickCount64() < deadline)
	{
		drops += hub.Drain(consumer, out);
		for (const auto &candidate : out)
		{
			if (candidate.sampleTimeQpc == sample.sampleTimeQpc &&
				candidate.deviceId == sample.deviceId)
				received = true;
		}
		if (!received)
			Sleep(1);
	}
	hub.Stop();

	char detail[192];
	snprintf(detail, sizeof detail,
		"created/controller/open %d/%d/%d reset %d retained %d published %d received %d gaps %llu",
		created, controllerOpened, initiallyOpen, resetObserved, mappingRetained,
		published, received, static_cast<unsigned long long>(drops));
	Check("pose hub: live reset gate",
		created && controllerOpened && initiallyOpen && resetObserved &&
		mappingRetained && published && received && drops == 0,
		detail);
}

void RunPoseChannelScenarios()
{
	RunPoseSampleScenarios();
	RunPoseRingConcurrentScenario();
	RunPoseRingDrainStatusScenario();
	RunPoseRingOverflowScenario();
	RunPoseRingStalledProducerScenario();
	RunPoseRingOverwrittenMarkerScenario();
	RunPoseRingAbandonedWriterScenario();
	RunPoseRingAbandonedResetOwnerScenario();
	RunPoseRingOpenResetRaceScenario();
	RunPoseHubTerminalGapScenario();
	RunPoseHubMarkerOverflowScenario();
	RunPoseHubConsumerIndependenceScenario();
	RunPoseHubMidDrainOverflowScenario();
	RunPoseHubWriterLivenessScenario();
	RunPoseHubLiveResetGateScenario();
}

void RunSolverPrimitiveScenarios()
{
	char detail[256];

	// Direct interpolation contract: endpoints, linear fields, quaternion
	// shortest-arc interpolation, out-of-range rejection, and gap rejection.
	{
		PoseSample a, b, out;
		a.time = 1.0;
		b.time = 2.0;
		a.rot = Eigen::Quaterniond::Identity();
		b.rot = Eigen::Quaterniond(Eigen::AngleAxisd(EIGEN_PI / 2.0, Eigen::Vector3d::UnitY()));
		a.pos = Eigen::Vector3d(1.0, 2.0, 3.0);
		b.pos = Eigen::Vector3d(5.0, 6.0, 7.0);
		a.vel = Eigen::Vector3d(-1.0, 0.0, 1.0);
		b.vel = Eigen::Vector3d(3.0, 4.0, 5.0);
		a.angVel = Eigen::Vector3d(0.0, 1.0, 0.0);
		b.angVel = Eigen::Vector3d(0.0, 3.0, 0.0);
		std::vector<PoseSample> stream{ a, b };

		bool ok = CalibrationEngine::InterpolateAt(stream, 1.25, 1.1, out);
		Eigen::Quaterniond expected(
			Eigen::AngleAxisd(EIGEN_PI / 8.0, Eigen::Vector3d::UnitY()));
		bool pass = ok &&
			(out.pos - Eigen::Vector3d(2.0, 3.0, 4.0)).norm() < 1e-12 &&
			(out.vel - Eigen::Vector3d(0.0, 1.0, 2.0)).norm() < 1e-12 &&
			(out.angVel - Eigen::Vector3d(0.0, 1.5, 0.0)).norm() < 1e-12 &&
			out.rot.angularDistance(expected) < 1e-12 &&
			!CalibrationEngine::InterpolateAt(stream, 0.9, 1.1, out) &&
			!CalibrationEngine::InterpolateAt(stream, 1.5, 0.5, out);
		Check("solver: interpolation contract", pass, "");
	}

	// Both offset signs, exact zero, and the edges of the configured search
	// range. Reported angular velocity is cleared to exercise finite-difference
	// correlation rather than the driver's fast path.
	{
		GroundTruth truth;
		truth.rotation = Eigen::Quaterniond(
			Eigen::AngleAxisd(1.2, Eigen::Vector3d(0.2, 0.9, -0.1).normalized()));
		truth.translation = Eigen::Vector3d(0.7, -0.2, 1.1);
		SceneConfig scene;
		scene.duration = 18.0;
		scene.refRate = 83.0;
		scene.targetRate = 71.0;

		const double offsets[] = { -0.055, -0.018, 0.0, 0.018, 0.055 };
		double worst = 0.0;
		bool pass = true;
		for (size_t i = 0; i < sizeof offsets / sizeof offsets[0]; ++i)
		{
			truth.latency = offsets[i];
			std::vector<PoseSample> ref, target;
			GenerateStreams(scene, truth, static_cast<uint32_t>(3100 + i), ref, target);
			for (auto &s : ref) s.angVel.setZero();
			for (auto &s : target) s.angVel.setZero();

			EngineConfig cfg;
			double solved = 0.0;
			bool estimated = CalibrationEngine::EstimateTimeOffset(ref, target, cfg, solved);
			worst = std::max(worst, std::abs(solved - truth.latency));
			pass = pass && estimated && std::abs(solved - truth.latency) < 0.004;
		}
		snprintf(detail, sizeof detail, "worst %.2f ms", worst * 1000.0);
		Check("solver: offset signs + bounds", pass, detail);
	}

	// Explicitly exercise both velocity gates and even thinning. The corrupted
	// velocity metadata must be dropped without perturbing the recovered pose.
	{
		GroundTruth truth{
			Eigen::Quaterniond(Eigen::AngleAxisd(1.4, Eigen::Vector3d::UnitY())),
			Eigen::Vector3d(0.5, 0.1, -0.8) };
		SceneConfig scene;
		std::vector<PoseSample> ref, target;
		GenerateStreams(scene, truth, 3200, ref, target);
		for (size_t i = 0; i < target.size(); ++i)
		{
			if (i % 5 == 0) target[i].vel.x() = 5.0;
			if (i % 7 == 0) target[i].angVel.y() = 20.0;
		}

		EngineConfig cfg;
		cfg.estimateTimeOffset = false;
		EngineResult r = CalibrationEngine::Solve(ref, target, cfg);
		double rotErr = truth.rotation.angularDistance(r.rotation) * 180.0 / EIGEN_PI;
		double transErr = (truth.translation - r.translation).norm();
		bool pass = r.valid && r.samplesGated > 300 &&
			r.samplesUsed == cfg.maxAlignedSamples &&
			rotErr < 0.1 && transErr < 0.005;
		snprintf(detail, sizeof detail,
			"gated %zu used %zu rot %.3f deg trans %.2f mm",
			r.samplesGated, r.samplesUsed, rotErr, transErr * 1000.0);
		Check("solver: gating + thinning", pass, detail);
	}

	// A reference dropout must not be interpolated across. Enough data remains
	// on both sides for an accurate solve.
	{
		GroundTruth truth{
			Eigen::Quaterniond(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitY())),
			Eigen::Vector3d(-0.4, 0.2, 0.9) };
		SceneConfig scene;
		std::vector<PoseSample> ref, target;
		GenerateStreams(scene, truth, 3300, ref, target);
		ref.erase(std::remove_if(ref.begin(), ref.end(), [](const PoseSample &s)
		{
			return s.time > 8.0 && s.time < 9.0;
		}), ref.end());

		EngineConfig cfg;
		cfg.estimateTimeOffset = false;
		cfg.maxAlignedSamples = 10000;
		EngineResult r = CalibrationEngine::Solve(ref, target, cfg);
		double rotErr = truth.rotation.angularDistance(r.rotation) * 180.0 / EIGEN_PI;
		double transErr = (truth.translation - r.translation).norm();
		bool pass = r.valid && r.samplesUsed < target.size() - 50 &&
			rotErr < 0.1 && transErr < 0.005;
		snprintf(detail, sizeof detail, "used %zu/%zu rot %.3f trans %.2f mm",
			r.samplesUsed, target.size(), rotErr, transErr * 1000.0);
		Check("solver: dropout gap", pass, detail);
	}

	// Pair-budget truncation is a configured complexity bound. The earliest
	// accepted multi-lag pairs must still form a usable, exactly bounded solve.
	{
		GroundTruth truth{
			Eigen::Quaterniond(Eigen::AngleAxisd(
				1.1, Eigen::Vector3d(0.2, 0.9, 0.1).normalized())),
			Eigen::Vector3d(0.3, -0.1, 0.8) };
		SceneConfig scene;
		std::vector<PoseSample> ref, target;
		GenerateStreams(scene, truth, 3400, ref, target);
		EngineConfig cfg;
		cfg.estimateTimeOffset = false;
		cfg.maxPairs = 80;
		cfg.minPairs = 30;
		EngineResult r = CalibrationEngine::Solve(ref, target, cfg);
		double rotErr = truth.rotation.angularDistance(r.rotation) * 180.0 / EIGEN_PI;
		double transErr = (truth.translation - r.translation).norm();
		snprintf(detail, sizeof detail, "pairs %zu rot %.3f trans %.2f mm",
			r.pairsUsed, rotErr, transErr * 1000.0);
		Check("solver: pair budget",
			r.valid && r.pairsUsed == cfg.maxPairs &&
			rotErr < 0.5 && transErr < 0.015, detail);
	}
}

void RunSolverRobustnessScenarios()
{
	char detail[320];
	const GroundTruth truth{
		Eigen::Quaterniond(
			Eigen::AngleAxisd(1.7, Eigen::Vector3d(0.1, 0.95, -0.2).normalized())),
		Eigen::Vector3d(1.0, -0.15, -0.6) };

	// Alternating q/-q representations must be invisible to interpolation,
	// delta extraction, and the refinement's quaternion mean.
	{
		SceneConfig scene;
		scene.posNoise = 0.001;
		scene.rotNoiseDeg = 0.1;
		std::vector<PoseSample> ref, target;
		GenerateStreams(scene, truth, 4100, ref, target);
		for (size_t i = 1; i < target.size(); i += 2)
			target[i].rot.coeffs() = -target[i].rot.coeffs();
		EngineResult r = CalibrationEngine::Solve(ref, target, EngineConfig());
		double rotErr = truth.rotation.angularDistance(r.rotation) * 180.0 / EIGEN_PI;
		double transErr = (truth.translation - r.translation).norm();
		snprintf(detail, sizeof detail, "rot %.3f deg trans %.2f mm",
			rotErr, transErr * 1000.0);
		Check("solver: quaternion double cover",
			r.valid && rotErr < 0.5 && transErr < 0.015, detail);
	}

	// Catastrophic orientation samples, not merely position spikes. Each bad
	// sample contaminates many delta pairs, exercising angle rejection and
	// rotation IRLS together.
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.2;
		std::vector<PoseSample> ref, target;
		GenerateStreams(scene, truth, 4200, ref, target);
		std::mt19937 rng(4201);
		size_t corrupt = 0;
		for (size_t i = 23; i < target.size(); i += 47)
		{
			target[i].rot = RandomQuaternion(rng, 1.4) * target[i].rot;
			++corrupt;
		}
		EngineResult r = CalibrationEngine::Solve(ref, target, EngineConfig());
		double rotErr = truth.rotation.angularDistance(r.rotation) * 180.0 / EIGEN_PI;
		double transErr = (truth.translation - r.translation).norm();
		snprintf(detail, sizeof detail,
			"%zu corrupt rot %.3f deg trans %.2f mm rejected %zu",
			corrupt, rotErr, transErr * 1000.0, r.pairsRejected);
		Check("solver: rotation outliers",
			r.valid && rotErr < 0.8 && transErr < 0.02 && r.pairsRejected > corrupt,
			detail);
	}

	// A contiguous tracking-fault burst combines orientation and position
	// corruption; robust estimators must recover from the healthy majority.
	{
		SceneConfig scene;
		scene.posNoise = 0.001;
		scene.rotNoiseDeg = 0.1;
		std::vector<PoseSample> ref, target;
		GenerateStreams(scene, truth, 4300, ref, target);
		for (auto &s : target)
		{
			if (s.time >= 10.0 && s.time <= 10.35)
			{
				s.pos += Eigen::Vector3d(0.35, -0.25, 0.20);
				s.rot = Eigen::Quaterniond(Eigen::AngleAxisd(
					0.8, Eigen::Vector3d(0.3, 0.4, 0.5).normalized())) * s.rot;
			}
		}
		EngineResult r = CalibrationEngine::Solve(ref, target, EngineConfig());
		double rotErr = truth.rotation.angularDistance(r.rotation) * 180.0 / EIGEN_PI;
		double transErr = (truth.translation - r.translation).norm();
		snprintf(detail, sizeof detail, "rot %.3f deg trans %.2f mm rejected %zu",
			rotErr, transErr * 1000.0, r.pairsRejected);
		Check("solver: burst corruption",
			r.valid && rotErr < 0.8 && transErr < 0.02, detail);
	}

	// Every golden-section scale candidate must start a fresh IRLS solve. A
	// deterministic heavy outlier set used to make the result depend on which
	// neighboring candidate had last mutated the shared row weights.
	{
		SceneConfig scene;
		GroundTruth scaledTruth{
			Eigen::Quaterniond(Eigen::AngleAxisd(1.35, Eigen::Vector3d::UnitY())),
			Eigen::Vector3d(0.8, -0.1, -0.55) };
		scaledTruth.scale = 1.08;
		auto aligned = GenerateAlignedSamples(scene, scaledTruth, 4320);
		for (size_t i = 4; i < aligned.size(); i += 23)
		{
			double sign = ((i / 23) & 1) ? -1.0 : 1.0;
			aligned[i].target.pos += sign * Eigen::Vector3d(0.50, -0.36, 0.44);
		}
		EngineConfig cfg;
		cfg.solveScale = true;
		cfg.refineIterations = 0;
		cfg.maxTranslationRms = 0.08;
		EngineResult r = CalibrationEngine::SolveAligned(aligned, cfg);
		double scaleErr = std::abs(r.scale - scaledTruth.scale);
		double rotErr = scaledTruth.rotation.angularDistance(r.rotation) *
			180.0 / EIGEN_PI;
		double transErr = (scaledTruth.translation - r.translation).norm();
		snprintf(detail, sizeof detail,
			"valid %d scale %.5f err %.5f rot %.3f deg trans %.1f mm rms %.1f mm",
			r.valid, r.scale, scaleErr, rotErr, transErr * 1000.0,
			r.translationRmsMeters * 1000.0);
		Check("solver: robust scale outliers",
			r.valid && scaleErr < 0.015 && rotErr < 0.5 && transErr < 0.03,
			detail);
	}

	// Force JointRefine's meter/radian conversion outside finite range while
	// leaving the sequential objective well-conditioned and finite. Refinement
	// must fail closed and preserve that usable seed byte-for-byte, never accept
	// a NaN candidate because its cost comparisons happened to be false.
	{
		SceneConfig scene;
		auto aligned = GenerateAlignedSamples(scene, truth, 4330);
		EngineConfig sequentialConfig;
		sequentialConfig.huberTranslation = 1e300;
		sequentialConfig.huberRotation = 1e-10;
		sequentialConfig.maxRotationRms = 180.0;
		sequentialConfig.maxTranslationRms = 1e300;
		sequentialConfig.refineIterations = 0;
		EngineResult sequential = CalibrationEngine::SolveAligned(
			aligned, sequentialConfig);
		EngineConfig refineConfig = sequentialConfig;
		refineConfig.refineIterations = 3;
		EngineResult fallback = CalibrationEngine::SolveAligned(aligned, refineConfig);
		bool finiteFallback = fallback.rotation.coeffs().allFinite() &&
			fallback.translation.allFinite() && std::isfinite(fallback.scale) &&
			std::isfinite(fallback.rotationRmsDeg) &&
			std::isfinite(fallback.translationRmsMeters);
		bool retained = sequential.valid && fallback.valid &&
			!fallback.refinementApplied && finiteFallback &&
			fallback.rotation.angularDistance(sequential.rotation) < 1e-12 &&
			(fallback.translation - sequential.translation).norm() < 1e-12 &&
			std::abs(fallback.scale - sequential.scale) < 1e-12;
		snprintf(detail, sizeof detail,
			"sequential %d fallback %d refined %d finite %d rotDiff %.2e transDiff %.2e",
			sequential.valid, fallback.valid, fallback.refinementApplied, finiteFallback,
			fallback.rotation.angularDistance(sequential.rotation),
			(fallback.translation - sequential.translation).norm());
		Check("solver: non-finite refinement fallback", retained, detail);
	}

	// Each quality gate gets a deliberately non-rigid data set while the other
	// residual gate is relaxed, pinning both the decision and its explanation.
	{
		SceneConfig scene;
		std::vector<PoseSample> ref, target;
		GenerateStreams(scene, truth, 4350, ref, target);
		for (auto &s : target)
		{
			Eigen::Vector3d axis(
				std::sin(0.7 * s.time), 0.3, std::cos(0.9 * s.time));
			s.rot = Eigen::Quaterniond(Eigen::AngleAxisd(
				0.18 * std::sin(2.3 * s.time), axis.normalized())) * s.rot;
		}
		EngineConfig rotCfg;
		rotCfg.maxRotationRms = 0.5;
		rotCfg.maxTranslationRms = 10.0;
		EngineResult badRotation = CalibrationEngine::Solve(ref, target, rotCfg);

		GenerateStreams(scene, truth, 4351, ref, target);
		for (auto &s : target)
		{
			s.pos += Eigen::Vector3d(
				0.10 * std::sin(1.7 * s.time),
				0.08 * std::sin(2.1 * s.time + 0.4),
				0.09 * std::cos(1.3 * s.time));
		}
		EngineConfig posCfg;
		posCfg.maxRotationRms = 180.0;
		posCfg.maxTranslationRms = 0.01;
		EngineResult badPosition = CalibrationEngine::Solve(ref, target, posCfg);

		bool pass = !badRotation.valid && !badPosition.valid &&
			badRotation.message.find("Rotation residual") != std::string::npos &&
			badPosition.message.find("Position residual") != std::string::npos;
		snprintf(detail, sizeof detail, "rotation %.2f deg (%d) position %.1f mm (%d)",
			badRotation.rotationRmsDeg,
			badRotation.message.find("Rotation residual") != std::string::npos,
			badPosition.translationRmsMeters * 1000.0,
			badPosition.message.find("Position residual") != std::string::npos);
		Check("solver: residual rejection gates", pass, detail);
	}

	// Validation and input-integrity matrix. These inputs must fail closed with
	// useful reasons rather than producing a plausible-looking transform.
	{
		SceneConfig scene;
		std::vector<PoseSample> ref, target;
		GenerateStreams(scene, truth, 4400, ref, target);

		std::vector<PoseSample> shortRef(ref.begin(), ref.begin() + 7);
		std::vector<PoseSample> shortTarget(target.begin(), target.begin() + 7);
		EngineResult tooShort = CalibrationEngine::Solve(
			shortRef, shortTarget, EngineConfig());

		SceneConfig stillScene;
		stillScene.motionScale = 0.02;
		std::vector<PoseSample> stillRef, stillTarget;
		GenerateStreams(stillScene, truth, 4401, stillRef, stillTarget);
		EngineResult tooStill = CalibrationEngine::Solve(
			stillRef, stillTarget, EngineConfig());

		auto nanRef = ref;
		nanRef[20].pos.x() = std::numeric_limits<double>::quiet_NaN();
		EngineResult nanResult = CalibrationEngine::Solve(
			nanRef, target, EngineConfig());

		auto zeroQuat = target;
		zeroQuat[30].rot = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
		EngineResult zeroResult = CalibrationEngine::Solve(
			ref, zeroQuat, EngineConfig());

		auto enormousQuat = target;
		double maxDouble = std::numeric_limits<double>::max();
		enormousQuat[30].rot = Eigen::Quaterniond(
			maxDouble, maxDouble, maxDouble, maxDouble);
		EngineResult enormousResult = CalibrationEngine::Solve(
			ref, enormousQuat, EngineConfig());

		auto hugeFiniteQuat = target;
		hugeFiniteQuat[30].rot = Eigen::Quaterniond(1e100, 1e100, 1e100, 1e100);
		EngineResult hugeFiniteResult = CalibrationEngine::Solve(
			ref, hugeFiniteQuat, EngineConfig());

		auto duplicateTime = ref;
		duplicateTime[40].time = duplicateTime[39].time;
		EngineResult duplicateResult = CalibrationEngine::Solve(
			duplicateTime, target, EngineConfig());

		auto extremePosition = ref;
		extremePosition[20].pos.x() = 1e300;
		EngineResult extremePositionResult = CalibrationEngine::Solve(
			extremePosition, target, EngineConfig());
		auto extremeVelocity = target;
		extremeVelocity[20].vel.y() = 1e300;
		EngineResult extremeVelocityResult = CalibrationEngine::Solve(
			ref, extremeVelocity, EngineConfig());
		auto extremeTime = ref;
		extremeTime[20].time = 1e300;
		EngineResult extremeTimeResult = CalibrationEngine::Solve(
			extremeTime, target, EngineConfig());

		auto extremeAligned = GenerateAlignedSamples(scene, truth, 4402, 80);
		extremeAligned[20].time = 1e300;
		extremeAligned[20].ref.pos.z() = 1e300;
		extremeAligned[20].target.vel.x() = 1e300;
		EngineResult extremeAlignedResult = CalibrationEngine::SolveAligned(
			extremeAligned, EngineConfig());

		bool pass = !tooShort.valid && !tooStill.valid &&
			!nanResult.valid && !zeroResult.valid && !enormousResult.valid &&
			!hugeFiniteResult.valid &&
			!duplicateResult.valid && !extremePositionResult.valid &&
			!extremeVelocityResult.valid && !extremeTimeResult.valid &&
			!extremeAlignedResult.valid &&
			tooShort.message.find("Not enough samples") != std::string::npos &&
			tooStill.message.find("Not enough rotation") != std::string::npos &&
			nanResult.message.find("invalid or out-of-range") != std::string::npos &&
			zeroResult.message.find("invalid or out-of-range") != std::string::npos &&
			enormousResult.message.find("invalid or out-of-range") != std::string::npos &&
			hugeFiniteResult.message.find("invalid or out-of-range") != std::string::npos &&
			duplicateResult.message.find("non-increasing") != std::string::npos &&
			extremePositionResult.message.find("out-of-range") != std::string::npos &&
			extremeVelocityResult.message.find("out-of-range") != std::string::npos &&
			extremeTimeResult.message.find("out-of-range") != std::string::npos &&
			extremeAlignedResult.message.find("out-of-range") != std::string::npos;
		snprintf(detail, sizeof detail,
			"short %d still %d nan %d zero/max/huge-q %d%d%d duplicate %d extreme p/v/t/a %d%d%d%d",
			!tooShort.valid, !tooStill.valid, !nanResult.valid,
			!zeroResult.valid, !enormousResult.valid, !hugeFiniteResult.valid,
			!duplicateResult.valid, !extremePositionResult.valid,
			!extremeVelocityResult.valid, !extremeTimeResult.valid,
			!extremeAlignedResult.valid);
		Check("solver: fail-closed inputs", pass, detail);
	}

	// Invalid work controls must fail before divisions, allocations, or loops;
	// an otherwise-valid but enormous timestamp span must also trip the explicit
	// correlation-resample cap rather than allocating proportional memory.
	{
		SceneConfig scene;
		std::vector<PoseSample> ref, target;
		GenerateStreams(scene, truth, 4410, ref, target);
		EngineConfig zeroStep;
		zeroStep.timeOffsetStep = 0.0;
		EngineResult zeroStepResult = CalibrationEngine::Solve(ref, target, zeroStep);
		EngineConfig nanStep;
		nanStep.timeOffsetStep = std::numeric_limits<double>::quiet_NaN();
		EngineResult nanStepResult = CalibrationEngine::Solve(ref, target, nanStep);
		EngineConfig zeroBudget;
		zeroBudget.maxAlignedSamples = 0;
		EngineResult zeroBudgetResult = CalibrationEngine::Solve(ref, target, zeroBudget);

		ref.resize(8);
		target.resize(8);
		for (size_t i = 0; i < 8; ++i)
		{
			ref[i].time = static_cast<double>(i) * 200000.0;
			target[i].time = ref[i].time;
		}
		double offset = 123.0;
		bool oversizedEstimated = CalibrationEngine::EstimateTimeOffset(
			ref, target, EngineConfig(), offset);
		bool pass = !zeroStepResult.valid && !nanStepResult.valid &&
			!zeroBudgetResult.valid && !oversizedEstimated && offset == 0.0 &&
			zeroStepResult.message.find("Invalid calibration engine configuration") !=
				std::string::npos &&
			nanStepResult.message.find("Invalid calibration engine configuration") !=
				std::string::npos &&
			zeroBudgetResult.message.find("Invalid calibration engine configuration") !=
				std::string::npos;
		snprintf(detail, sizeof detail,
			"zero/nan step %d/%d zero budget %d oversized estimate %d offset %.1f",
			!zeroStepResult.valid, !nanStepResult.valid, !zeroBudgetResult.valid,
			oversizedEstimated, offset);
		Check("solver: config and work caps", pass, detail);
	}
}

void RunSolverPropertyScenarios(int trials, uint32_t propertySeed)
{
	std::mt19937 rng(propertySeed);
	std::uniform_real_distribution<double> rateDist(60.0, 110.0);
	std::uniform_real_distribution<double> latencyDist(-0.035, 0.035);
	std::uniform_real_distribution<double> scaleDist(0.87, 1.13);
	std::uniform_real_distribution<double> noiseDist(0.0, 0.002);

	double worstRot = 0.0;
	double worstTrans = 0.0;
	double worstOffset = 0.0;
	double worstScale = 0.0;
	int firstFailure = -1;
	std::string firstMessage;

	for (int trial = 0; trial < trials; ++trial)
	{
		GroundTruth truth;
		truth.rotation = RandomQuaternion(rng);
		truth.translation = RandomVector(rng, 2.5);
		truth.latency = latencyDist(rng);

		EngineConfig cfg;
		cfg.gravityPriorRatio = 0.0;   // validate general SO(3), independent of the physical up prior
		cfg.solveScale = trial % 3 == 0;
		cfg.pinScaleOnSmoothing = false;
		truth.scale = cfg.solveScale ? scaleDist(rng) : 1.0;

		SceneConfig scene;
		scene.duration = 14.0;
		scene.refRate = rateDist(rng);
		scene.targetRate = rateDist(rng);
		scene.posNoise = noiseDist(rng);
		scene.rotNoiseDeg = 0.05 + 80.0 * scene.posNoise;
		scene.timestampJitter = 0.00035;
		if (trial % 7 == 0)
			scene.outlierRate = 0.003;

		Eigen::Quaterniond mountRot = RandomQuaternion(rng);
		Eigen::Vector3d mountPos = RandomVector(rng, 0.18);
		std::vector<PoseSample> ref, target;
		GenerateStreamsWithMount(scene, truth, static_cast<uint32_t>(rng()),
			mountRot, mountPos, ref, target);
		if (trial % 4 == 0)
			for (size_t i = 1; i < target.size(); i += 2)
				target[i].rot.coeffs() = -target[i].rot.coeffs();

		EngineResult r = CalibrationEngine::Solve(ref, target, cfg);
		double rotErr = truth.rotation.angularDistance(r.rotation) * 180.0 / EIGEN_PI;
		double transErr = (truth.translation - r.translation).norm();
		double offsetErr = std::abs(truth.latency - r.timeOffset);
		double scaleErr = std::abs(truth.scale - r.scale);
		worstRot = std::max(worstRot, rotErr);
		worstTrans = std::max(worstTrans, transErr);
		worstOffset = std::max(worstOffset, offsetErr);
		worstScale = std::max(worstScale, scaleErr);

		// With solveScale off the field must be untouched, not merely close:
		// roughly two thirds of these trials run that way and never looked.
		bool pass = r.valid && rotErr < 0.8 && transErr < 0.025 &&
			offsetErr < 0.005 && (cfg.solveScale ? scaleErr < 0.012 : r.scale == 1.0);
		if (!pass && firstFailure < 0)
		{
			firstFailure = trial;
			firstMessage = r.message;
		}
	}

	char detail[384];
	if (firstFailure >= 0)
	{
		snprintf(detail, sizeof detail,
			"%d trials seed %u worst %.3f deg / %.1f mm / %.2f ms / scale %.4f  first failure %d: %s",
			trials, propertySeed,
			worstRot, worstTrans * 1000.0, worstOffset * 1000.0, worstScale,
			firstFailure, firstMessage.c_str());
	}
	else
	{
		snprintf(detail, sizeof detail,
			"%d trials seed %u worst %.3f deg / %.1f mm / %.2f ms / scale %.4f",
			trials, propertySeed,
			worstRot, worstTrans * 1000.0, worstOffset * 1000.0, worstScale);
	}
	Check("solver: randomized properties", firstFailure < 0, detail);

	// The production gravity prior must remain a prior: rich motion should
	// recover a realistic range of tilted universes rather than flatten them.
	{
		const double tilts[] = { -12.0, -6.0, 0.0, 6.0, 12.0 };
		double worst = 0.0;
		bool pass = true;
		for (size_t i = 0; i < sizeof tilts / sizeof tilts[0]; ++i)
		{
			double t = tilts[i] * EIGEN_PI / 180.0;
			GroundTruth truth;
			truth.rotation =
				Eigen::Quaterniond(Eigen::AngleAxisd(1.3, Eigen::Vector3d::UnitY())) *
				Eigen::Quaterniond(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitX())) *
				Eigen::Quaterniond(Eigen::AngleAxisd(-0.5 * t, Eigen::Vector3d::UnitZ()));
			truth.translation = Eigen::Vector3d(0.8, 0.12, -1.0);
			SceneConfig scene;
			scene.posNoise = 0.001;
			scene.rotNoiseDeg = 0.1;
			std::vector<PoseSample> ref, target;
			GenerateStreams(scene, truth, static_cast<uint32_t>(5100 + i), ref, target);
			EngineResult r = CalibrationEngine::Solve(ref, target, EngineConfig());
			double err = truth.rotation.angularDistance(r.rotation) * 180.0 / EIGEN_PI;
			worst = std::max(worst, err);
			pass = pass && r.valid && err < 1.0 &&
				(truth.translation - r.translation).norm() < 0.02;
		}
		snprintf(detail, sizeof detail, "worst %.3f deg", worst);
		Check("solver: gravity-prior tilt sweep", pass, detail);
	}
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

	// Some runtimes change WFD and inversely re-express the driver-local pose,
	// leaving the composed world pose and velocity continuous. That is a basis
	// bookkeeping change, not a universe jump: the exact path must reject the
	// discontinuous local pose and the heuristic path must see zero residual.
	{
		JumpDetector jd(TestQpcToSeconds);
		JumpRun r = DriveJump(jd, rate, [&](double t, uint32_t id)
		{
			Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
			RefTrajectory(t, id, rot, pos, vel, angVel);
			if (t < tJump)
			{
				return RingSample(id, t, Eigen::Quaterniond::Identity(),
					Eigen::Vector3d::Zero(), rot, pos, vel, angVel);
			}
			Eigen::Quaterniond inverse = D_R.conjugate();
			return RingSample(id, t, D_R, D_T,
				inverse * rot, inverse * (pos - D_T),
				inverse * vel, inverse * angVel);
		});
		snprintf(detail, sizeof detail, "deltas %d", r.deltas);
		Check("jump: inverse local re-expression", r.deltas == 0, detail);
	}

	// A rejected HMD observation breaks adjacency even when the surrounding
	// valid frames are close enough that their local trajectory would otherwise
	// pass the exact WFD test. Cover the three rejection classes independently:
	// explicit invalidity, malformed numerics, and composed-time inversion.
	{
		auto transitionAcceptedAcross = [&](int rejectionKind)
		{
			JumpDetector jd(TestQpcToSeconds);
			auto sampleAt = [&](double t, const Eigen::Quaterniond &wfdRotation,
				const Eigen::Vector3d &wfdTranslation)
			{
				Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
				RefTrajectory(t, vr::k_unTrackedDeviceIndex_Hmd,
					rot, pos, vel, angVel);
				return RingSample(vr::k_unTrackedDeviceIndex_Hmd, t,
					wfdRotation, wfdTranslation, rot, pos, vel, angVel);
			};

			jd.Push(sampleAt(1.0, Eigen::Quaterniond::Identity(),
				Eigen::Vector3d::Zero()));
			protocol::DevicePoseSample rejected = sampleAt(
				rejectionKind == 2 ? 0.99 : 1.01,
				Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
			if (rejectionKind == 0)
				rejected.poseIsValid = false;
			else if (rejectionKind == 1)
				rejected.position[0] = std::numeric_limits<double>::quiet_NaN();
			jd.Push(rejected);
			jd.Push(sampleAt(1.02, D_R, D_T));

			JumpDetector::UniverseDelta delta;
			return jd.PollDelta(delta);
		};
		bool invalidAccepted = transitionAcceptedAcross(0);
		bool malformedAccepted = transitionAcceptedAcross(1);
		bool invertedAccepted = transitionAcceptedAcross(2);
		snprintf(detail, sizeof detail, "invalid/malformed/inverted accepted %d/%d/%d",
			invalidAccepted, malformedAccepted, invertedAccepted);
		Check("jump: rejected frame breaks WFD adjacency",
			!invalidAccepted && !malformedAccepted && !invertedAccepted, detail);
	}

	// Accepted deltas carry the absolute WFD endpoint from their own HMD sample.
	// Queue two distinct exact rebases before polling so a later global cache
	// cannot accidentally overwrite the first event's re-anchor endpoint.
	{
		JumpDetector jd(TestQpcToSeconds);
		const Eigen::Quaterniond wfd1(
			Eigen::AngleAxisd(12.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
		const Eigen::Vector3d trans1(0.2, 0.01, -0.1);
		const Eigen::Quaterniond wfd2(
			Eigen::AngleAxisd(-8.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
		const Eigen::Vector3d trans2(-0.15, 0.02, 0.25);
		for (double t = 0.0; t < 3.0; t += 1.0 / rate)
		{
			Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
			RefTrajectory(t, vr::k_unTrackedDeviceIndex_Hmd, rot, pos, vel, angVel);
			Eigen::Quaterniond wfdRotation = Eigen::Quaterniond::Identity();
			Eigen::Vector3d wfdTranslation = Eigen::Vector3d::Zero();
			if (t >= 2.0)
			{
				wfdRotation = wfd2;
				wfdTranslation = trans2;
			}
			else if (t >= 0.75)
			{
				wfdRotation = wfd1;
				wfdTranslation = trans1;
			}
			jd.Push(RingSample(vr::k_unTrackedDeviceIndex_Hmd, t,
				wfdRotation, wfdTranslation, rot, pos, vel, angVel));
		}
		std::vector<JumpDetector::UniverseDelta> deltas;
		JumpDetector::UniverseDelta delta;
		while (jd.PollDelta(delta))
			deltas.push_back(delta);
		bool endpoints = deltas.size() == 2 && deltas[0].exact && deltas[1].exact &&
			deltas[0].worldFromDriverRotation.angularDistance(wfd1) < 1e-12 &&
			(deltas[0].worldFromDriverTranslation - trans1).norm() < 1e-12 &&
			deltas[1].worldFromDriverRotation.angularDistance(wfd2) < 1e-12 &&
			(deltas[1].worldFromDriverTranslation - trans2).norm() < 1e-12;
		snprintf(detail, sizeof detail, "deltas %zu endpoints %d", deltas.size(), endpoints);
		Check("jump: queued exact WFD endpoints", endpoints, detail);
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

	// Malformed Running_OK samples and composed-time inversions must be treated
	// as absence, without poisoning the detector's history. A valid exact jump
	// after the bad burst still has to be recovered.
	{
		JumpDetector jd(TestQpcToSeconds);
		auto sampleAt = [&](double t)
		{
			Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
			RefTrajectory(t, 0, rot, pos, vel, angVel);
			return RingSample(0, t, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
				rot, pos, vel, angVel);
		};
		protocol::DevicePoseSample bad = sampleAt(0.1);
		bad.position[0] = std::numeric_limits<double>::quiet_NaN();
		jd.Push(bad);
		bad = sampleAt(0.2);
		bad.rotation = { 0.0, 0.0, 0.0, 0.0 };
		jd.Push(bad);
		bad = sampleAt(0.3);
		bad.poseTimeOffset = std::numeric_limits<double>::infinity();
		jd.Push(bad);
		bad = sampleAt(0.31);
		bad.poseTimeOffset = 1e300;
		jd.Push(bad);
		bad = sampleAt(0.32);
		bad.position[0] = 1e300;
		jd.Push(bad);
		bad = sampleAt(0.33);
		bad.angularVelocity[1] = 1e300;
		jd.Push(bad);
		jd.Push(sampleAt(0.25));
		bad = sampleAt(0.20);
		bad.position[0] += 10.0;   // valid numerics, but timestamp inversion
		jd.Push(bad);

		JumpRun r = DriveJump(jd, rate, [&](double t, uint32_t id)
		{
			Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
			RefTrajectory(t, id, rot, pos, vel, angVel);
			bool after = t >= tJump;
			return RingSample(id, t,
				after ? D_R : Eigen::Quaterniond::Identity(),
				after ? D_T : Eigen::Vector3d::Zero(),
				rot, pos, vel, angVel);
		});
		bool pass = r.deltas == 1 && r.last.exact && r.YawErrDeg(jumpYaw) < 0.1 &&
			r.TransErr(D_T) < 0.01;
		snprintf(detail, sizeof detail, "deltas %d exact %d yawErr %.3f transErr %.4f",
			r.deltas, r.last.exact, r.YawErrDeg(jumpYaw), r.TransErr(D_T));
		Check("jump: malformed inputs recover", pass, detail);
	}

	// E. Gravity constraint (invariant 15) under a re-localization that is NOT
	// gravity-preserving: the worldFromDriver delta carries genuine tilt. Every
	// other jump scenario builds a pure-UnitY jump, so a detector that folded
	// the raw delta straight into the calibration would pass all of them. Here
	// the accepted delta must be the yaw part alone -- applying the tilt would
	// slope the floor and roll the horizon in one step -- and the discarded
	// tilt must be reported as the non-rigid residual rather than dropped.
	{
		const double tiltRad = 6.0 * EIGEN_PI / 180.0;
		auto runWithTilt = [&](double tilt)
		{
			JumpDetector jd(TestQpcToSeconds);
			Eigen::Quaterniond jumpRot =
				(Eigen::Quaterniond(Eigen::AngleAxisd(jumpYaw, Eigen::Vector3d::UnitY())) *
				 Eigen::Quaterniond(Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitX()))).normalized();
			return DriveJump(jd, rate, [&](double t, uint32_t id)
			{
				Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
				RefTrajectory(t, id, rot, pos, vel, angVel);
				bool after = t >= tJump;
				return RingSample(id, t,
					after ? jumpRot : Eigen::Quaterniond::Identity(),
					after ? D_T : Eigen::Vector3d::Zero(),
					rot, pos, vel, angVel);
			});
		};

		JumpRun tilted = runWithTilt(tiltRad);
		JumpRun upright = runWithTilt(0.0);

		// The accepted rotation is built from an AngleAxis about UnitY, so its
		// x/z components are exactly zero -- assert the AXIS, not the magnitude.
		double offAxis = std::max(std::abs(tilted.last.rotation.x()),
			std::abs(tilted.last.rotation.z()));
		double reportedTiltDeg = tilted.last.residualTiltRad * 180.0 / EIGEN_PI;
		double uprightTiltDeg = upright.last.residualTiltRad * 180.0 / EIGEN_PI;

		bool pass = tilted.deltas == 1 && tilted.last.exact &&
			offAxis < 1e-12 &&
			tilted.YawErrDeg(jumpYaw) < 0.05 && tilted.TransErr(D_T) < 0.01 &&
			std::abs(reportedTiltDeg - 6.0) < 0.05 &&
			upright.deltas == 1 && uprightTiltDeg < 0.01;
		snprintf(detail, sizeof detail,
			"offAxis %.2e  yawErr %.4f deg  residual tilt %.3f deg (built 6.0)  upright residual %.4f deg",
			offAxis, tilted.YawErrDeg(jumpYaw), reportedTiltDeg, uprightTiltDeg);
		Check("jump: yaw-only under tilt", pass, detail);
	}

	// F. The bookkeeping an accepted exact delta carries for the drift/staleness
	// scoring, plus the two drain APIs. Push the non-HMD device FIRST at the
	// rebase frame: TryAccept clears the candidate list the instant the HMD's
	// own candidate lands, so per-device disagreement is only observable when
	// another device's candidate is already pending.
	{
		auto runWithSecondDevice = [&](const Eigen::Vector3d &shift)
		{
			JumpDetector jd(TestQpcToSeconds);
			JumpRun r;
			JumpDetector::UniverseDelta d;
			Eigen::Vector3d shifted = D_T + shift;
			for (double t = 0.0; t < 3.0; t += 1.0 / rate)
			{
				bool after = t >= tJump;
				for (int order = 1; order >= 0; --order)
				{
					uint32_t id = static_cast<uint32_t>(order);
					Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
					RefTrajectory(t, id, rot, pos, vel, angVel);
					Eigen::Quaterniond wfdRot = Eigen::Quaterniond::Identity();
					Eigen::Vector3d wfdTrans = Eigen::Vector3d::Zero();
					if (after)
					{
						wfdRot = D_R;
						wfdTrans = id == 0 ? D_T : shifted;
					}
					jd.Push(RingSample(id, t, wfdRot, wfdTrans, rot, pos, vel, angVel));
				}
				while (jd.PollDelta(d)) { r.deltas++; r.last = d; }
			}
			return r;
		};

		// The HMD is authoritative either way; the second device's disagreement
		// only decides whether it counts as corroboration.
		JumpRun agreeing = runWithSecondDevice(Eigen::Vector3d(0.03, 0.0, 0.0));
		JumpRun disagreeing = runWithSecondDevice(Eigen::Vector3d(0.25, 0.0, 0.0));
		bool spreadOk =
			agreeing.deltas == 1 && agreeing.last.devicesAgreeing == 2 &&
			std::abs(agreeing.last.residualSpread - 0.03) < 1e-9 &&
			disagreeing.deltas == 1 && disagreeing.last.devicesAgreeing == 1 &&
			std::abs(disagreeing.last.residualSpread - 0.25) < 1e-9 &&
			agreeing.TransErr(D_T) < 0.01 && disagreeing.TransErr(D_T) < 0.01;

		// Notes are the calibration log's only record of what the detector saw;
		// Reset is what calibration start and monitor-disable call.
		JumpDetector jd(TestQpcToSeconds);
		auto pushHmd = [&](double t, const Eigen::Quaterniond &wfdRot, const Eigen::Vector3d &wfdTrans)
		{
			Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
			RefTrajectory(t, vr::k_unTrackedDeviceIndex_Hmd, rot, pos, vel, angVel);
			jd.Push(RingSample(vr::k_unTrackedDeviceIndex_Hmd, t, wfdRot, wfdTrans,
				rot, pos, vel, angVel));
		};
		const Eigen::Quaterniond D_R2(
			Eigen::AngleAxisd(-10.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
		const Eigen::Vector3d D_T2(-0.2, 0.0, 0.15);

		// Every timestamp comes from the same step index. Accumulating `t +=
		// 1.0 / rate` up to a hand-written boundary constant does NOT reach it
		// cleanly: 135 additions of 1/90 land on 1.4999999999999967, which is
		// still < 1.5, so the loop emits one extra sample -- and RingSample's
		// `t / TestQpcToSeconds + 0.5` rounds a 3e-15 s difference into the SAME
		// 10 MHz tick as 1.5. Push then reads the following rebase sample as a
		// composed-time inversion, drops it, and clears wfdValid, so the rebase
		// is never looked for at all (no note, no candidate, no delta -- which
		// also made the Reset check below vacuous). Integer steps keep adjacent
		// samples a full frame apart, which is the property Push depends on.
		const int jumpStep = static_cast<int>(tJump * rate);   // 135 at 90 Hz
		std::string firstNote;
		bool noteDescribesRebase = false;
		for (int step = 0; step <= jumpStep + 1; ++step)
		{
			Eigen::Quaterniond wfdRot = Eigen::Quaterniond::Identity();
			Eigen::Vector3d wfdTrans = Eigen::Vector3d::Zero();
			if (step == jumpStep)
			{
				wfdRot = D_R;
				wfdTrans = D_T;
			}
			else if (step > jumpStep)
			{
				wfdRot = D_R2;
				wfdTrans = D_T2;
			}
			pushHmd(static_cast<double>(step) / rate, wfdRot, wfdTrans);

			// Poll where the rebase actually happens, and only the first note:
			// the second rebase then leaves its own note AND its delta pending,
			// so Reset below has something real to drop rather than passing on
			// an empty queue.
			if (step == jumpStep)
				noteDescribesRebase = jd.PollNote(firstNote) &&
					firstNote.find("rebase") != std::string::npos;
		}

		jd.Reset();
		JumpDetector::UniverseDelta stale;
		std::string drain;
		bool resetDrained = !jd.PollDelta(stale) && !jd.PollNote(drain);

		// Reset drops worldFromDriver validity, so the first sample after it
		// re-seeds silently; a genuine rebase after THAT is still detected.
		int reseedDeltas = 0, postResetDeltas = 0;
		for (double t = 2.0; t < 2.6; t += 1.0 / rate)
		{
			bool second = t >= 2.3;
			pushHmd(t, second ? D_R2 : D_R, second ? D_T2 : D_T);
			while (jd.PollDelta(stale))
			{
				if (second)
					postResetDeltas++;
				else
					reseedDeltas++;
			}
		}

		bool resetOk = resetDrained && reseedDeltas == 0 && postResetDeltas == 1;
		snprintf(detail, sizeof detail,
			"spread %.3f/%.3f m  agreeing %d/%d  note %d [%.60s]  reset drained %d reseed %d re-detect %d",
			agreeing.last.residualSpread, disagreeing.last.residualSpread,
			agreeing.last.devicesAgreeing, disagreeing.last.devicesAgreeing,
			noteDescribesRebase, firstNote.c_str(),
			resetDrained, reseedDeltas, postResetDeltas);
		Check("jump: spread, notes, reset", spreadOk && noteDescribesRebase && resetOk, detail);
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

	// A 0.3--1.0 s occlusion during ordinary ~1 m/s motion is not a tracking
	// re-localization. Trapezoidal velocity prediction removes that explainable
	// displacement, while a real 40 cm residual at the same gaps still fires.
	{
		const double gaps[] = { 0.31, 0.65, 0.99 };
		const Eigen::Vector3d velocity(1.0, 0.1, -0.2);
		int ordinaryLosses = 0;
		int relocalizationLosses = 0;
		double worstMagnitudeError = 0.0;
		for (double gap : gaps)
		{
			auto movingSample = [&](double t, const Eigen::Vector3d &position)
			{
				return RingSample(6, t, Eigen::Quaterniond::Identity(),
					Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
					position, velocity, Eigen::Vector3d::Zero());
			};

			DriftMonitor ordinary(TestQpcToSeconds);
			DriftRun ordinaryRun;
			ordinary.Push(movingSample(0.0, base));
			ordinary.Push(movingSample(gap, base + velocity * gap));
			drain(ordinary, ordinaryRun);
			ordinaryLosses += ordinaryRun.losses;

			DriftMonitor relocalized(TestQpcToSeconds);
			DriftRun relocalizedRun;
			relocalized.Push(movingSample(0.0, base));
			relocalized.Push(movingSample(gap,
				base + velocity * gap + Eigen::Vector3d(0.4, 0.0, 0.0)));
			drain(relocalized, relocalizedRun);
			relocalizationLosses += relocalizedRun.losses;
			worstMagnitudeError = std::max(worstMagnitudeError,
				std::abs(relocalizedRun.lastMag - 0.4));
		}
		snprintf(detail, sizeof detail,
			"ordinary %d relocalized %d/%zu magnitudeErr %.2e",
			ordinaryLosses, relocalizationLosses, sizeof gaps / sizeof gaps[0],
			worstMagnitudeError);
		Check("drift: moving short-gap prediction",
			ordinaryLosses == 0 &&
			relocalizationLosses == static_cast<int>(sizeof gaps / sizeof gaps[0]) &&
			worstMagnitudeError < 1e-9,
			detail);
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

	// Raw target-universe distances become calibrated meters through Push's
	// scale argument. The same raw drift/recovery stays below thresholds at 1x
	// and crosses them at 2x; reported magnitudes are calibrated too.
	{
		auto runScaledSlide = [&](double scale)
		{
			DriftMonitor dm(TestQpcToSeconds);
			DriftRun r;
			const Eigen::Vector3d rawDrift(0.0012, 0.0, 0.0);
			for (double t = 0.0; t < 10.0; t += 1.0 / rate)
			{
				dm.Push(stillSample(t, base + rawDrift * t), scale);
				drain(dm, r);
			}
			return r;
		};
		DriftRun unitSlide = runScaledSlide(1.0);
		DriftRun doubleSlide = runScaledSlide(2.0);

		auto runScaledLoss = [&](double scale)
		{
			DriftMonitor dm(TestQpcToSeconds);
			DriftRun r;
			dm.Push(stillSample(0.0, base), scale);
			dm.Push(stillSample(0.5,
				base + Eigen::Vector3d(0.15, 0.0, 0.0)), scale);
			drain(dm, r);
			return r;
		};
		DriftRun unitLoss = runScaledLoss(1.0);
		DriftRun doubleLoss = runScaledLoss(2.0);

		snprintf(detail, sizeof detail,
			"slides 1x/2x %d/%d (%.1f mm) losses 1x/2x %d/%d (%.3f m)",
			unitSlide.slides, doubleSlide.slides, doubleSlide.lastMag * 1000.0,
			unitLoss.losses, doubleLoss.losses, doubleLoss.lastMag);
		Check("drift: calibrated scale thresholds",
			unitSlide.slides == 0 && doubleSlide.slides == 1 &&
			doubleSlide.lastMag > 0.012 &&
			unitLoss.losses == 0 && doubleLoss.losses == 1 &&
			std::abs(doubleLoss.lastMag - 0.30) < 1e-9,
			detail);
	}

	// A profile scale edit changes the coordinate basis, not the physical pose.
	// Reset that device's baselines at the edit, then prove fresh same-scale
	// relocalization evidence is still detected afterward.
	{
		DriftMonitor dm(TestQpcToSeconds);
		DriftRun beforeRealLoss;
		dm.Push(stillSample(0.0, base), 1.0);
		dm.Push(stillSample(0.5, base), 2.0);
		drain(dm, beforeRealLoss);
		dm.Push(stillSample(1.0,
			base + Eigen::Vector3d(0.20, 0.0, 0.0)), 2.0);
		DriftRun afterRealLoss;
		drain(dm, afterRealLoss);
		snprintf(detail, sizeof detail,
			"after scale edit slides/losses %d/%d fresh losses %d mag %.3f",
			beforeRealLoss.slides, beforeRealLoss.losses,
			afterRealLoss.losses, afterRealLoss.lastMag);
		Check("drift: scale change resets baseline",
			beforeRealLoss.slides == 0 && beforeRealLoss.losses == 0 &&
			afterRealLoss.slides == 0 && afterRealLoss.losses == 1 &&
			std::abs(afterRealLoss.lastMag - 0.40) < 1e-9,
			detail);
	}

	// Bad numeric fields and an out-of-order composed timestamp do not enter the
	// rolling window; valid evidence immediately afterward remains usable.
	{
		DriftMonitor dm(TestQpcToSeconds);
		DriftRun r;
		protocol::DevicePoseSample bad = stillSample(0.1, base);
		bad.worldFromDriverTranslation[2] = std::numeric_limits<double>::quiet_NaN();
		dm.Push(bad);
		bad = stillSample(0.2, base);
		bad.worldFromDriverRotation = { 0.0, 0.0, 0.0, 0.0 };
		dm.Push(bad);
		bad = stillSample(0.3, base);
		bad.velocity[1] = std::numeric_limits<double>::infinity();
		dm.Push(bad);
		bad = stillSample(0.31, base);
		bad.poseTimeOffset = 1e300;
		dm.Push(bad);
		bad = stillSample(0.32, base);
		bad.worldFromDriverTranslation[0] = 1e300;
		dm.Push(bad);
		bad = stillSample(0.33, base);
		bad.velocity[2] = 1e300;
		dm.Push(bad);

		const Eigen::Vector3d driftRate(0.0033, 0.0, 0.0011);
		dm.Push(stillSample(0.25, base + driftRate * 0.25));
		bad = stillSample(0.20, base + Eigen::Vector3d(10.0, 0.0, 0.0));
		dm.Push(bad);
		std::mt19937 rng(71);
		std::normal_distribution<double> n(0.0, 0.0015);
		for (double t = 0.26; t < 10.0; t += 1.0 / rate)
		{
			Eigen::Vector3d pos = base + driftRate * t + Eigen::Vector3d(n(rng), n(rng), n(rng));
			dm.Push(stillSample(t, pos));
			drain(dm, r);
		}
		bool pass = r.slides == 1 && r.losses == 0 && r.lastMag > 0.012 && r.lastMag < 0.04;
		snprintf(detail, sizeof detail, "slides %d losses %d mag %.1f mm",
			r.slides, r.losses, r.lastMag * 1000.0);
		Check("drift: malformed inputs recover", pass, detail);
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
		Eigen::Quaterniond dR;
		Eigen::Vector3d dT;
		AnchorDelta(anchors[i].R, anchors[i].T, baseInv, base.T, dR, dT);
		f.anchors[i].rotationDelta = { dR.w(), dR.x(), dR.y(), dR.z() };
		for (int k = 0; k < 3; ++k)
		{
			f.anchors[i].position[k] = positions[i](k);
			f.anchors[i].translationDelta[k] = dT(k);
		}
	}
	return f;
}

// Independent oracle for the driver's blend, derived from the field's
// DEFINITION (see AlignmentField.h) rather than transcribed from BlendAt: a
// Gaussian radial basis over horizontal separation plus one constant identity
// contributor, combined as a weighted mean.
//
//   w_i   = exp(-(r_i / sigma)^2 / 2),  r_i = horizontal |query - anchor_i|
//   w_0   = the identity floor; its delta is the identity transform
//   rot   = normalize(sum_j w_j q_j)    (each q_j in identity's hemisphere)
//   trans = sum_j w_j t_j / sum_j w_j
//
// Every constant is spelled out as its own literal on purpose. Reading
// alignfield::IdentityFloorWeight (or the protocol's sigma default) would move
// oracle and implementation together, which is precisely the failure this
// oracle exists to catch: the floor decides how much of a measured anchor
// delta the runtime actually applies -- 1/(1+w0), ~95% at 0.05 -- so a silent
// change to it MUST break the comparison scenario below.
//
// `anchorCount` is supplied by the caller rather than read off the field, so
// the oracle carries no opinion about the implementation's MaxAnchors clamp;
// that clamp gets its own differential scenario.
void ReferenceBlend(const protocol::SetAlignmentField &f, uint32_t anchorCount,
                    const Eigen::Vector3d &pos,
                    Eigen::Quaterniond &rotOut, Eigen::Vector3d &transOut)
{
	const double identityFloorWeight = 0.05;   // must equal alignfield::IdentityFloorWeight
	const double fallbackSigmaMeters = 1.5;    // must equal protocol::SetAlignmentField's default
	const double minUsableSigma = 0.01;        // at or below this the stored sigma is unusable

	double sigma = f.sigmaMeters > minUsableSigma ? f.sigmaMeters : fallbackSigmaMeters;

	struct Contribution
	{
		double weight;
		Eigen::Quaterniond rotation;
		Eigen::Vector3d translation;
	};
	std::vector<Contribution> mix;
	mix.push_back({ identityFloorWeight,
		Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero() });

	for (uint32_t i = 0; i < anchorCount; ++i)
	{
		const auto &a = f.anchors[i];
		Eigen::Vector2d horizontal(pos.x() - a.position[0], pos.z() - a.position[2]);
		double r = horizontal.norm() / sigma;
		mix.push_back({ std::exp(-0.5 * r * r),
			Eigen::Quaterniond(a.rotationDelta.w, a.rotationDelta.x,
				a.rotationDelta.y, a.rotationDelta.z),
			Eigen::Vector3d(a.translationDelta[0], a.translationDelta[1],
				a.translationDelta[2]) });
	}

	Eigen::Vector4d qSum = Eigen::Vector4d::Zero();   // w, x, y, z
	Eigen::Vector3d tSum = Eigen::Vector3d::Zero();
	double weightSum = 0.0;
	for (const auto &c : mix)
	{
		// q and -q name the same rotation: take the representative on identity's
		// side of the hemisphere so the linear mean is the short-arc one.
		double sign = c.rotation.w() < 0.0 ? -1.0 : 1.0;
		qSum += (c.weight * sign) * Eigen::Vector4d(c.rotation.w(), c.rotation.x(),
			c.rotation.y(), c.rotation.z());
		tSum += c.weight * c.translation;
		weightSum += c.weight;
	}

	qSum.normalize();
	rotOut = Eigen::Quaterniond(qSum(0), qSum(1), qSum(2), qSum(3));
	transOut = tSum / weightSum;
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

		// Include a full field: the previous 1/4/7 progression never reached
		// MaxAnchors, so the last slot was blended by neither side.
		const uint32_t counts[] = { 1, 4, 7, protocol::SetAlignmentField::MaxAnchors };
		for (int trial = 0; trial < static_cast<int>(sizeof counts / sizeof counts[0]); ++trial)
		{
			protocol::SetAlignmentField rf;
			rf.enabled = true;
			rf.generation = trial;
			rf.anchorCount = counts[trial];
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
				ReferenceBlend(rf, rf.anchorCount, pos, refR, refT);

				worstQ = std::max(worstQ, 1.0 - std::abs(r.w * refR.w() + r.x * refR.x() + r.y * refR.y() + r.z * refR.z()));
				worstT = std::max(worstT, (Eigen::Vector3d(t[0], t[1], t[2]) - refT).norm());
			}
		}
		snprintf(detail, sizeof detail, "worst 1-|dot| %.2e  worst dTrans %.2e", worstQ, worstT);
		Check("field: matches reference", worstQ < 1e-12 && worstT < 1e-12, detail);
	}

	// D2. Bounds clamp. A stored snapshot claiming more anchors than the fixed
	// array holds must blend exactly the first MaxAnchors and read nothing past
	// them: ValidateAndSanitize rejects an over-large count on the IPC path, but
	// the clamp exists because the blend runs on vrserver's pose thread against
	// whatever the snapshot happens to hold. Differential against the same field
	// with an in-range count -- no oracle involved, so this pins the clamp alone.
	{
		// `anchors` is the last member of SetAlignmentField, so an unclamped
		// read walks straight into whatever follows the struct. Give it
		// something loud: anchors sitting on the query points with meter-scale
		// deltas, so even a partial overrun moves the blend far past epsilon.
		struct SpilledField
		{
			protocol::SetAlignmentField field;
			protocol::FieldAnchor spill[4];
		};

		SpilledField s{};
		s.field.enabled = true;
		s.field.generation = 3;
		s.field.sigmaMeters = 2.0;
		s.field.anchorCount = protocol::SetAlignmentField::MaxAnchors;

		std::mt19937 rng(4242);
		std::uniform_real_distribution<double> u(-1.0, 1.0);
		for (uint32_t i = 0; i < protocol::SetAlignmentField::MaxAnchors; ++i)
		{
			Eigen::Quaterniond dq(Eigen::AngleAxisd(0.04 * u(rng),
				Eigen::Vector3d(u(rng), u(rng), u(rng)).normalized()));
			s.field.anchors[i].rotationDelta = { dq.w(), dq.x(), dq.y(), dq.z() };
			for (int k = 0; k < 3; ++k)
			{
				s.field.anchors[i].position[k] = 3.0 * u(rng);
				s.field.anchors[i].translationDelta[k] = 0.04 * u(rng);
			}
		}
		const Eigen::Quaterniond poisonRot(Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()));
		for (auto &poison : s.spill)
		{
			poison.rotationDelta = { poisonRot.w(), poisonRot.x(), poisonRot.y(), poisonRot.z() };
			for (int k = 0; k < 3; ++k)
			{
				poison.position[k] = 0.0;             // right on top of the query points
				poison.translationDelta[k] = 5.0;     // meters
			}
		}

		// Reference behaviour: the same anchors under a count the array can hold.
		protocol::SetAlignmentField inRange = s.field;
		s.field.anchorCount = protocol::SetAlignmentField::MaxAnchors + 4;

		double worstRot = 0.0, worstTrans = 0.0;
		for (int k = 0; k < 12; ++k)
		{
			Eigen::Vector3d q(2.0 * u(rng), 1.0 + u(rng), 2.0 * u(rng));
			double p[3] = { q.x(), q.y(), q.z() };

			vr::HmdQuaternion_t rClamped, rOverlarge;
			double tClamped[3], tOverlarge[3];
			alignfield::BlendAt(inRange, p, rClamped, tClamped);
			alignfield::BlendAt(s.field, p, rOverlarge, tOverlarge);

			worstRot = std::max(worstRot, 1.0 - std::abs(
				rClamped.w * rOverlarge.w + rClamped.x * rOverlarge.x +
				rClamped.y * rOverlarge.y + rClamped.z * rOverlarge.z));
			worstTrans = std::max(worstTrans, Dist3(tClamped, tOverlarge));
		}
		snprintf(detail, sizeof detail, "count %u vs %u  worst 1-|dot| %.2e  dTrans %.2e",
			inRange.anchorCount, s.field.anchorCount, worstRot, worstTrans);
		Check("field: anchor count clamp", worstRot < 1e-12 && worstTrans < 1e-12, detail);
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
	const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();

	// The live world-from-driver watcher uses these helpers to distinguish a
	// no-op/rounding fluctuation from a real raw-universe rebase.
	{
		const Eigen::Vector3d translation(0.7, -0.02, 1.3);
		Eigen::Quaterniond deltaRotation;
		Eigen::Vector3d deltaTranslation;
		bool derived = WorldFromDriverDelta(identity, translation,
			identity, translation, deltaRotation, deltaTranslation);
		Check("chaperone: identity world delta",
			derived && deltaRotation.angularDistance(identity) < 1e-12 &&
			deltaTranslation.norm() < 1e-12 &&
			!WorldFromDriverChanged(identity, translation, identity, translation), "");
	}

	{
		const Eigen::Quaterniond oldRotation(
			Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitY()));
		const Eigen::Vector3d oldTranslation(0.8, 0.04, -1.1);
		const Eigen::Quaterniond expectedRotation(
			Eigen::AngleAxisd(-0.62, Eigen::Vector3d::UnitY()));
		const Eigen::Vector3d expectedTranslation(-0.35, 0.02, 0.47);
		const Eigen::Quaterniond newRotation =
			(expectedRotation * oldRotation).normalized();
		const Eigen::Vector3d newTranslation =
			expectedRotation * oldTranslation + expectedTranslation;
		Eigen::Quaterniond recoveredRotation;
		Eigen::Vector3d recoveredTranslation;
		bool recovered = WorldFromDriverDelta(oldRotation, oldTranslation,
			newRotation, newTranslation, recoveredRotation, recoveredTranslation);
		snprintf(detail, sizeof detail, "rot %.3e trans %.3e changed %d",
			recoveredRotation.angularDistance(expectedRotation),
			(recoveredTranslation - expectedTranslation).norm(),
			WorldFromDriverChanged(oldRotation, oldTranslation,
				newRotation, newTranslation));
		Check("chaperone: recover world delta",
			recovered && recoveredRotation.angularDistance(expectedRotation) < 1e-12 &&
			(recoveredTranslation - expectedTranslation).norm() < 1e-12 &&
			WorldFromDriverChanged(oldRotation, oldTranslation,
				newRotation, newTranslation), detail);
	}

	{
		const Eigen::Vector3d translation(0.2, 0.0, -0.4);
		Eigen::Quaterniond tinyRotation(
			Eigen::AngleAxisd(0.4e-5, Eigen::Vector3d::UnitY()));
		Eigen::Vector3d tinyTranslation = translation + Eigen::Vector3d(0.4e-4, 0.0, 0.0);
		Check("chaperone: ignore world delta epsilon",
			!WorldFromDriverChanged(identity, translation,
				tinyRotation, tinyTranslation), "");
	}

	{
		Eigen::Quaterniond zero(0.0, 0.0, 0.0, 0.0);
		double maxDouble = std::numeric_limits<double>::max();
		Eigen::Quaterniond enormous(maxDouble, maxDouble, maxDouble, maxDouble);
		Eigen::Vector3d finite = Eigen::Vector3d::Zero();
		Eigen::Vector3d nonfinite = finite;
		nonfinite.x() = std::numeric_limits<double>::infinity();
		Eigen::Quaterniond deltaRotation;
		Eigen::Vector3d deltaTranslation;
		bool pass = !WorldFromDriverDelta(zero, finite, identity, finite,
				deltaRotation, deltaTranslation) &&
			!WorldFromDriverDelta(identity, finite, enormous, finite,
				deltaRotation, deltaTranslation) &&
			!WorldFromDriverDelta(identity, nonfinite, identity, finite,
				deltaRotation, deltaTranslation) &&
			WorldFromDriverChanged(zero, finite, identity, finite);
		Check("chaperone: reject invalid world delta", pass, "");
	}

	// Chaperone re-anchoring trusts a WFD delta only when adjacent HMD-local
	// poses prove that it was a real universe rebase. The same helper is shared
	// with JumpDetector so the two state machines cannot classify one transition
	// differently.
	{
		ringpose::DriverLocalPoseSample previous;
		previous.time = 1.0;
		previous.rotation = Eigen::Quaterniond(
			Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));
		previous.position = Eigen::Vector3d(0.3, 1.1, -0.4);
		previous.velocity = Eigen::Vector3d(0.6, 0.0, -0.2);
		previous.angularVelocity = Eigen::Vector3d(0.0, 0.3, 0.0);
		ringpose::DriverLocalPoseSample continuous = previous;
		continuous.time += 0.05;
		continuous.position += previous.velocity * 0.05;
		continuous.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(
			previous.angularVelocity.norm() * 0.05,
			previous.angularVelocity.normalized())) * previous.rotation;

		Eigen::Quaterniond acceptedRotation;
		Eigen::Vector3d acceptedTranslation;
		bool wfdDeltaValid = WorldFromDriverDelta(identity, Eigen::Vector3d::Zero(),
			D_R, D_T, acceptedRotation, acceptedTranslation);
		Check("chaperone: continuous WFD rebase",
			wfdDeltaValid && ringpose::IsDriverLocalPoseContinuous(previous, continuous), "");

		ringpose::DriverLocalPoseSample inverseLocal = continuous;
		inverseLocal.rotation = D_R.conjugate() * continuous.rotation;
		inverseLocal.position = D_R.conjugate() * (continuous.position - D_T);
		inverseLocal.velocity = D_R.conjugate() * continuous.velocity;
		inverseLocal.angularVelocity = D_R.conjugate() * continuous.angularVelocity;
		Check("chaperone: inverse local WFD rejected",
			!ringpose::IsDriverLocalPoseContinuous(previous, inverseLocal), "");

		ringpose::DriverLocalPoseSample afterGap = continuous;
		afterGap.time = previous.time + ringpose::MaxAdjacentFrameSeconds + 0.01;
		Check("chaperone: unclassified WFD gap rejected",
			!ringpose::IsDriverLocalPoseContinuous(previous, afterGap), "");
	}

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

	// D. Persisted/live geometry is a trust boundary. Accept a normal rigid
	// room, but reject non-finite coordinates, invalid play sizes, scaled or
	// reflected standing bases, and implausibly distant raw-space centers.
	{
		vr::HmdVector2_t size{ { 2.0f, 3.0f } };
		std::vector<vr::HmdQuad_t> walls(1);
		for (int c = 0; c < 4; ++c)
		{
			walls[0].vCorners[c].v[0] = (c & 1) ? 1.0f : -1.0f;
			walls[0].vCorners[c].v[1] = c >= 2 ? 2.0f : 0.0f;
			walls[0].vCorners[c].v[2] = -1.5f;
		}
		bool valid = IsPlausibleChaperone(walls, m, size);

		auto nonFiniteWalls = walls;
		nonFiniteWalls[0].vCorners[0].v[0] =
			std::numeric_limits<float>::quiet_NaN();
		bool rejectsNonFinite = !IsPlausibleChaperone(nonFiniteWalls, m, size);

		vr::HmdVector2_t zeroSize{ { 0.0f, 3.0f } };
		bool rejectsSize = !IsPlausibleChaperone(walls, m, zeroSize);

		auto scaled = m;
		scaled.m[0][0] *= 1.2f;
		bool rejectsScale = !IsPlausibleChaperone(walls, scaled, size);

		auto reflected = m;
		for (int row = 0; row < 3; ++row)
			reflected.m[row][0] = -reflected.m[row][0];
		bool rejectsReflection = !IsPlausibleChaperone(walls, reflected, size);

		auto distant = m;
		distant.m[0][3] = protocol::limits::MaxAbsChaperoneCoordinateMeters + 1.0f;
		bool rejectsDistance = !IsPlausibleChaperone(walls, distant, size);

		Check("chaperone: plausibility validation",
			valid && rejectsNonFinite && rejectsSize && rejectsScale &&
			rejectsReflection && rejectsDistance, "");
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

void ApplyTrackingWarble(double t, double stopTime, PoseSample &sample)
{
	if (t < 20.0 || t >= stopTime)
		return;
	double angle = (1.6 * EIGEN_PI / 180.0) * std::sin(2.0 * EIGEN_PI * t / 0.8);
	sample.rot = (Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()))
		* sample.rot).normalized();
	sample.pos += Eigen::Vector3d(
		0.008 * std::sin(2.0 * EIGEN_PI * t / 0.7),
		0.008 * std::sin(2.0 * EIGEN_PI * t / 0.5 + 0.8),
		0.008 * std::sin(2.0 * EIGEN_PI * t / 0.9 + 2.0));
}

void SetMountAfterSlip(double t, double slipTime,
	Eigen::Quaterniond &rotation, Eigen::Vector3d &position)
{
	if (t < slipTime)
	{
		rotation = kMountRot;
		position = kMountPos;
		return;
	}
	rotation = kMountRot * Eigen::Quaterniond(Eigen::AngleAxisd(
		3.0 * EIGEN_PI / 180.0, Eigen::Vector3d(1.0, 0.2, 0.0).normalized()));
	position = kMountPos + Eigen::Vector3d(0.02, 0.0, -0.01);
}

struct ContinuousSim
{
	ContinuousAlignment ca;
	ContinuousAlignment::ExpectedCalibrationAt expectedAt;
	Eigen::Quaterniond calRot{ 1, 0, 0, 0 };
	Eigen::Vector3d calTrans{ 0, 0, 0 };
	double calScale = 1.0;
	double solvedOffset = 0.0;

	int corrections = 0;
	int freezes = 0, resumes = 0, losses = 0, recoveries = 0;
	int scatterFreezes = 0;   // subset of freezes that came via the scatter path
	int unstables = 0;
	double maxCorrRotDeg = 0.0;   // largest single emitted correction
	double maxCorrPosM = 0.0;     // measured as displacement at the head
	// Largest off-UnitY component any emitted correction quaternion carried.
	// Magnitude alone cannot tell a yaw correction from a full-delta one.
	double maxCorrOffAxis = 0.0;
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

// Tilt (non-yaw) part of the sim's calibration error. A yaw-only correction
// policy leaves it EXACTLY invariant: both left- and right-multiplying by a
// rotation about UnitY preserve |(w, y)| of the delta quaternion, and the tilt
// is 2*acos of that. So this doubles as a detector for a correction that
// carried any tilt at all.
double CalTiltDeg(const ContinuousSim &sim, const GroundTruth &truth)
{
	Eigen::Quaterniond dR = (truth.rotation * sim.calRot.conjugate()).normalized();
	if (dR.w() < 0.0)
		dR.coeffs() = -dR.coeffs();
	Eigen::Quaterniond yaw(Eigen::AngleAxisd(
		2.0 * std::atan2(dR.y(), dR.w()), Eigen::Vector3d::UnitY()));
	return yaw.angularDistance(dR) * 180.0 / EIGEN_PI;
}

// One closed-loop segment: generate both streams, tick Update at 50 Hz, apply
// polled corrections back onto the sim's calibration (exactly what the
// overlay's ApplyAlignmentDelta will do), and tally events.
void RunContinuousSegment(ContinuousSim &sim, const SceneConfig &scene, double t0, double t1,
	std::mt19937 &rng,
	const std::function<GroundTruth(double)> &truthAt,
	const std::function<void(double, Eigen::Quaterniond &, Eigen::Vector3d &)> &mountAt,
	const std::function<bool(double)> &targetVisible,
	const std::function<void(double, PoseSample &)> &refPost = nullptr,
	bool negateTargetQuat = false,
	const std::function<void(double)> &onUpdate = nullptr)
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
			sim.ca.Update(t, sim.calRot, sim.calTrans, sim.calScale,
				sim.solvedOffset, sim.expectedAt);

			ContinuousAlignment::Correction c;
			while (sim.ca.PollCorrection(c))
			{
				double corrDeg = c.rotation.angularDistance(Eigen::Quaterniond::Identity()) * 180.0 / EIGEN_PI;
				sim.maxCorrRotDeg = std::max(sim.maxCorrRotDeg, corrDeg);
				Eigen::Vector3d hp = PositionAt(t);
				sim.maxCorrPosM = std::max(sim.maxCorrPosM, (c.rotation * hp + c.translation - hp).norm());
				sim.maxCorrOffAxis = std::max(sim.maxCorrOffAxis,
					std::max(std::abs(c.rotation.x()), std::abs(c.rotation.z())));

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
				// Both are freezes. Counting only the first would silently miss
				// every scatter-path freeze, and /W3 does not warn on the gap.
				case ContinuousAlignment::Event::FrozenMountScatter:
					sim.freezes++;
					sim.scatterFreezes++;
					break;
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

		MountExtrinsic e, eWobble;
		bool ok = ContinuousAlignment::DeriveMountExtrinsic(ref, tgt, cal, e);
		double rotErr = e.rot.angularDistance(kMountRot) * 180.0 / EIGEN_PI;
		double posErr = (e.pos - kMountPos).norm();
		// eWobble is left untouched on failure now, so its rms reads 0.
		bool okWobble = ContinuousAlignment::DeriveMountExtrinsic(ref, tgtWobble, cal, eWobble);

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

	// Streaming offset signs: RunContinuousSegment feeds each reference,
	// target, and Update event in timestamp order, just like the overlay. For a
	// negative solved offset, a target needs a reference pose that arrives in a
	// later tick; it must remain pending rather than being consumed on the first
	// failed interpolation. Exercise the search bounds and representative inner
	// values in both directions, plus exact zero.
	{
		const double offsets[] = { -0.055, -0.018, 0.0, 0.018, 0.055 };
		bool pass = true;
		size_t minObs = std::numeric_limits<size_t>::max();
		int minCorrections = std::numeric_limits<int>::max();
		double worstYaw = 0.0, worstPos = 0.0;

		for (size_t i = 0; i < sizeof offsets / sizeof offsets[0]; ++i)
		{
			GroundTruth truth = baseTruth;
			truth.latency = offsets[i];
			auto truthAt = [&](double) { return truth; };

			ContinuousSim sim;
			sim.ca.SetExtrinsic(trueExtrinsic);
			sim.solvedOffset = offsets[i];
			Eigen::Quaterniond dR(Eigen::AngleAxisd(
				0.3 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
			Eigen::Vector3d dT(0.004, 0.002, -0.003);
			sim.calRot = (dR.conjugate() * truth.rotation).normalized();
			sim.calTrans = dR.conjugate() * (truth.translation - dT);

			std::mt19937 rng(1200 + static_cast<uint32_t>(i));
			RunContinuousSegment(sim, scene, 0.0, 15.0, rng,
				truthAt, constMount, alwaysVisible);

			double yawErr, posErr;
			CalError(sim, truth, 15.0, yawErr, posErr);
			minObs = std::min(minObs, sim.ca.ObservationCount());
			minCorrections = std::min(minCorrections, sim.corrections);
			worstYaw = std::max(worstYaw, yawErr);
			worstPos = std::max(worstPos, posErr);
			pass = pass && sim.ca.GetState() == ContinuousAlignment::State::Tracking &&
				sim.ca.ObservationCount() >= 60 && sim.corrections >= 1 &&
				sim.freezes == 0 && yawErr < 0.12 && posErr < 0.005;
		}

		snprintf(detail, sizeof detail,
			"minObs %zu  minCorr %d  worst residual %.3f deg / %.1f mm",
			minObs, minCorrections, worstYaw, worstPos * 1000.0);
		Check("continuous: streaming offset signs", pass, detail);
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
			SetMountAfterSlip(t, 30.0, r, p);
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
	// six minutes and the EWMA-tracked offset follows within 3 ms after the ramp
	// settles. With the option off the offset never moves, and a motionless
	// window yields no estimate at all.
	//
	// UNVERIFIED, deliberately: the EWMA itself and its limits are NOT pinned
	// here. The shipped filter lives in ContinuousTick (Overlay/Calibration.cpp
	// :1741-1758) -- the 0.75/0.25 blend, the +/-0.002 s per-update step clamp,
	// the +/-0.060 s absolute clamp on the accumulated offset, and the 0.5 ms
	// threshold that decides whether the driver is re-synchronized -- and
	// Calibration.cpp is not in SolverTests. `applyEwma` below is a DRIVER for
	// the scenario, not an oracle: it exists so the offset moves at all, and it
	// carries neither the absolute clamp nor the resync threshold. Asserting its
	// own step bound (as this scenario used to) was a tautology over the two
	// lines above the assertion and could not fail for any behaviour of the
	// shipped code. What IS pinned below reaches ContinuousAlignment:
	// PollTimeOffset's cadence, its measurements tracking a moving true latency,
	// its silence when the opt-in is off, and the correlator's refusal on
	// motionless streams. Pinning the filter needs it lifted out of
	// ContinuousTick into a compiled unit (a pure `double NextTimeOffset(double
	// current, double measured)` would do it).
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

		double maxStep = 0.0;   // reported only; see the note above
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
			"tracked to %.1f ms (err %.2f ms)  %d updates  harness maxStep %.2f ms (unasserted)  off %d  still %d",
			sim.solvedOffset * 1000.0, trackErr * 1000.0, offsetUpdates, maxStep * 1000.0,
			offUpdates, stillEstimated);
		Check("continuous: latency re-estimation",
			offsetUpdates >= 5 && trackErr < 0.003 &&
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
			// Vary the width that goes on the wire. The driver shapes its blend
			// from whatever arrives, so agreeing at the struct default proves
			// only that nobody has set it yet - the mirror has to agree at the
			// value actually shipped.
			f.sigmaMeters = 0.8 + 0.35 * (trial % 5);

			for (int k = 0; k < 8; ++k)
			{
				Eigen::Vector3d q(3.0 * u(rng), 1.2 + 0.3 * u(rng), 3.0 * u(rng));
				FieldTransform drv = DriverEffective(f, base, q);
				Eigen::Quaterniond eR;
				Eigen::Vector3d eT;
				BlendedFieldCalibration(overlay, base.R, base.T, q, eR, eT, f.sigmaMeters);
				worstRot = std::max(worstRot, eR.angularDistance(drv.R));
				worstPos = std::max(worstPos, (eT - drv.T).norm());
			}
		}

		// The identity floor is still a shared constant on both sides. The blend
		// width no longer is - it travels on the wire and is exercised above at
		// several non-default values.
		bool constantsMatch =
			FieldBlendIdentityFloor == alignfield::IdentityFloorWeight;
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

	// 11. A moving field history must be compared position by position. The
	// mounted tracker crosses from outside a 6 cm anchor into its center while
	// the universes remain perfectly stable. Comparing the whole 10 s window to
	// only the latest local transform produces a centimeter-scale false
	// correction; rebasing every observation through its own field value stays
	// at the unchanged base calibration.
	{
		const FieldTransform base{ Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero() };
		std::vector<OverlayAnchor> anchors{
			{ Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
				Eigen::Vector3d(0.06, 0.0, 0.0) } };

		// Same shape as the closure ContinuousTick builds (Overlay/Calibration.cpp
		// :1663-1670): the field is looked up at the tracker's BASE-CALIBRATED
		// world position, so the calibrated scale multiplies the raw target
		// position before the base transform. Unity here; scenario 17 drives it
		// with a non-unity scale, which is the only way that factor is visible.
		const double calScale = 1.0;
		auto expectedAt = [&](const Eigen::Vector3d &targetRawPos,
			Eigen::Quaterniond &rotationOut, Eigen::Vector3d &translationOut)
		{
			Eigen::Vector3d basePos = base.R * (calScale * targetRawPos) + base.T;
			BlendedFieldCalibration(anchors, base.R, base.T, basePos,
				rotationOut, translationOut);
		};

		MountExtrinsic identityMount;
		identityMount.valid = true;
		ContinuousAlignment perObservation;
		ContinuousAlignment latestOnly;
		perObservation.SetExtrinsic(identityMount);
		latestOnly.SetExtrinsic(identityMount);

		std::mt19937 rng(911);
		std::normal_distribution<double> refNoise(0.0, 0.0001);
		int correctCorrections = 0;
		int latestCorrections = 0;
		double latestMaxCorrection = 0.0;
		for (double t = 0.0; t < 16.0; t += 1.0 / 90.0)
		{
			double x = t < 10.0 ? -5.0 + 0.5 * t : 0.0;
			double vx = t < 10.0 ? 0.5 : 0.0;

			PoseSample target;
			target.time = t;
			target.rot = Eigen::Quaterniond::Identity();
			target.pos = Eigen::Vector3d(x, 0.0, 0.0);
			target.vel = Eigen::Vector3d(vx, 0.0, 0.0);

			Eigen::Quaterniond localRot;
			Eigen::Vector3d localTrans;
			expectedAt(target.pos, localRot, localTrans);

			PoseSample reference;
			reference.time = t;
			reference.rot = localRot;
			reference.pos = localRot * target.pos + localTrans +
				Eigen::Vector3d(refNoise(rng), refNoise(rng), refNoise(rng));
			reference.vel = Eigen::Vector3d(vx, 0.0, 0.0);

			perObservation.PushReference(reference);
			perObservation.PushTarget(target);
			latestOnly.PushReference(reference);
			latestOnly.PushTarget(target);

			perObservation.Update(t, base.R, base.T, calScale, 0.0, expectedAt);
			latestOnly.Update(t, localRot, localTrans, calScale, 0.0);

			ContinuousAlignment::Correction correction;
			while (perObservation.PollCorrection(correction))
				correctCorrections++;
			while (latestOnly.PollCorrection(correction))
			{
				latestCorrections++;
				latestMaxCorrection = std::max(latestMaxCorrection,
					correction.translation.norm());
			}
		}

		snprintf(detail, sizeof detail,
			"per-observation %d corrections / state %d; latest-only %d, max %.1f mm",
			correctCorrections, static_cast<int>(perObservation.GetState()),
			latestCorrections, latestMaxCorrection * 1000.0);
		Check("continuous: moving field baseline",
			correctCorrections == 0 &&
			perObservation.GetState() == ContinuousAlignment::State::Tracking &&
			latestCorrections > 0 && latestMaxCorrection > 0.005,
			detail);
	}

	// 12. Degraded tracking: mid-frequency warble (grazing lighthouse geometry
	// while lying down) inflates window scatter past the gates but decorrelates
	// between consecutive observations, so the classifier calls it noise — the
	// loop must hold quietly with one informational event, never freeze with
	// the mount warning, and resume by itself once tracking settles.
	{
		std::mt19937 rng(1010);
		auto warble = [&](double t, PoseSample &s)
		{
			ApplyTrackingWarble(t, 50.0, s);
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

	// 13. Alternating discontinuity candidates are valid pose pairs but not
	// usable observations. They must not keep a stale Tracking state fresh after
	// the last healthy observation ages out.
	{
		MountExtrinsic identityMount;
		identityMount.valid = true;
		ContinuousAlignment alignment;
		alignment.SetExtrinsic(identityMount);

		auto pushPair = [&](double time, double referenceShift)
		{
			PoseSample reference;
			reference.pos.x() = referenceShift;
			reference.time = time - 0.02;
			alignment.PushReference(reference);
			reference.time = time;
			alignment.PushReference(reference);
			PoseSample target;
			target.time = time;
			alignment.PushTarget(target);
			alignment.Update(time, Eigen::Quaterniond::Identity(),
				Eigen::Vector3d::Zero(), 1.0, 0.0);
		};

		double time = 0.0;
		for (; time < 7.0; time += 0.11)
			pushPair(time, 0.0);
		bool startedTracking = alignment.GetState() == ContinuousAlignment::State::Tracking;

		int corrections = 0;
		int losses = 0;
		long long mode = 0;
		for (; time < 20.0; time += 0.20, ++mode)
		{
			pushPair(time, (mode & 1) ? -0.12 : 0.12);

			ContinuousAlignment::Correction correction;
			while (alignment.PollCorrection(correction))
				corrections++;
			ContinuousAlignment::Event event;
			while (alignment.PollEvent(event))
				if (event.type == ContinuousAlignment::Event::TrackerLost)
					losses++;
		}

		bool leftTracking = alignment.GetState() != ContinuousAlignment::State::Tracking;
		snprintf(detail, sizeof detail,
			"started %d  final state %d  losses %d  corrections %d  observations %zu",
			startedTracking, static_cast<int>(alignment.GetState()), losses, corrections,
			alignment.ObservationCount());
		Check("continuous: rejected jumps go stale",
			startedTracking && leftTracking && alignment.ObservationCount() < 40 &&
			corrections == 0,
			detail);
	}

	// 14. Mount slip during degraded tracking: a long run of unstructured
	// warble votes must not indefinitely delay the freeze once the mount then
	// genuinely slips — the sliding vote window bounds the delay to
	// ~scatterVoteWindow evaluations instead of the episode's whole history.
	{
		std::mt19937 rng(1111);
		auto warble = [&](double t, PoseSample &s)
		{
			ApplyTrackingWarble(t, 90.0, s);
		};
		auto slipMount = [&](double t, Eigen::Quaterniond &r, Eigen::Vector3d &p)
		{
			SetMountAfterSlip(t, 45.0, r, p);
		};

		ContinuousSim sim;
		sim.ca.SetExtrinsic(trueExtrinsic);
		sim.solvedOffset = baseTruth.latency;
		sim.calRot = baseTruth.rotation;
		sim.calTrans = baseTruth.translation;

		RunContinuousSegment(sim, scene, 0.0, 45.0, rng, constTruth, slipMount, alwaysVisible, warble);
		bool quietBefore = sim.freezes == 0;
		RunContinuousSegment(sim, scene, 45.0, 90.0, rng, constTruth, slipMount, alwaysVisible, warble);
		bool frozeAfter = sim.freezes >= 1 &&
			sim.ca.GetState() == ContinuousAlignment::State::Frozen;

		snprintf(detail, sizeof detail, "quiet-before %d  freezes %d  frozen-at-end %d",
			quietBefore, sim.freezes, frozeAfter);
		Check("continuous: slip during warble freezes", quietBefore && frozeAfter, detail);
	}

	// 15. Corrections are yaw-only (invariant 15). Every other scenario starts
	// from a pure-yaw offset and only measures correction MAGNITUDE, so a Decide
	// that reconstructed the correction from the full delta rD -- which is
	// sitting right there, already computed -- would converge just as well and
	// pass all of them, while quietly folding mount creep and lighthouse noise
	// into the playspace as pitch and roll. Start 0.3 deg of yaw AND 0.6 deg of
	// pitch from truth, both inside the freeze band: the yaw must be walked out,
	// the tilt must survive bit-for-bit, and no correction may carry an off-axis
	// component.
	{
		std::mt19937 rng(1212);
		ContinuousSim sim;
		sim.ca.SetExtrinsic(trueExtrinsic);
		sim.solvedOffset = baseTruth.latency;
		Eigen::Quaterniond dR =
			Eigen::Quaterniond(Eigen::AngleAxisd(0.3 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY())) *
			Eigen::Quaterniond(Eigen::AngleAxisd(0.6 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitX()));
		dR.normalize();
		sim.calRot = (dR.conjugate() * baseTruth.rotation).normalized();
		sim.calTrans = baseTruth.translation;

		double startTilt = CalTiltDeg(sim, baseTruth);
		RunContinuousSegment(sim, scene, 0.0, 25.0, rng, constTruth, constMount, alwaysVisible);
		double endTilt = CalTiltDeg(sim, baseTruth);
		double yawErr, posErr;
		CalError(sim, baseTruth, 25.0, yawErr, posErr);

		// posErr is NOT asserted: the surviving tilt shows up there as ~17 mm of
		// effective displacement at the head, and removing it is exactly what
		// this scenario forbids.
		snprintf(detail, sizeof detail,
			"corr %d  offAxis %.2e  tilt %.4f -> %.4f deg (built 0.6)  yaw %.3f deg  pos %.1f mm",
			sim.corrections, sim.maxCorrOffAxis, startTilt, endTilt, yawErr, posErr * 1000.0);
		Check("continuous: corrections yaw-only",
			sim.corrections >= 1 && sim.freezes == 0 &&
			sim.maxCorrOffAxis < 1e-12 &&
			std::abs(startTilt - 0.6) < 1e-9 && std::abs(endTilt - startTilt) < 1e-6 &&
			yawErr < 0.12, detail);
	}

	// 16. Frozen -> Tracking. The resume hysteresis (deviation below the freeze
	// band by resumeFactor, latched, then confirmed for resumeConfirmSeconds) is
	// the only exit from Frozen short of Reset, and no scenario ever took it --
	// every mention of Resumed in this harness asserted it stayed at ZERO. So an
	// inverted comparison or a latch that never sets would leave continuous
	// calibration dead for the rest of the session after any transient fault
	// cleared, with the UI still blaming a mount that is fine.
	//
	// Perturbing the CALIBRATION rather than the truth is what a folded jump
	// delta or a recalibration does, and it leaves the observation stream
	// continuous, so the jump guard never sees it. Warble injected while frozen
	// also pins the `state == Frozen` early return on the scatter path: it must
	// raise neither an ObservationsUnstable event nor a second freeze.
	{
		std::mt19937 rng(1313);
		auto frozenWarble = [](double t, PoseSample &s)
		{
			if (t < 26.0 || t >= 31.0)
				return;
			double angle = (1.6 * EIGEN_PI / 180.0) * std::sin(2.0 * EIGEN_PI * t / 0.8);
			s.rot = (Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()))
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

		RunContinuousSegment(sim, scene, 0.0, 15.0, rng, constTruth, constMount, alwaysVisible);
		bool trackingFirst = sim.ca.GetState() == ContinuousAlignment::State::Tracking;

		// 3 deg of yaw error: past freezeYawDeg, sustained -> Frozen (~t=22).
		const Eigen::Quaterniond kick(
			Eigen::AngleAxisd(3.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
		int correctionsAtKick = sim.corrections;
		sim.calRot = (kick * sim.calRot).normalized();
		RunContinuousSegment(sim, scene, 15.0, 34.0, rng, constTruth, constMount, alwaysVisible,
			frozenWarble);
		int correctionsWhileFrozen = sim.corrections - correctionsAtKick;
		bool froze = sim.freezes == 1 && sim.resumes == 0 && sim.unstables == 0 &&
			correctionsWhileFrozen == 0 &&
			sim.ca.GetState() == ContinuousAlignment::State::Frozen;

		// Undo the kick, leaving a 0.3 deg residual: the deviation lands well
		// inside the resume band (freeze * resumeFactor) and stays there, and is
		// still outside the deadband so corrections are observable afterwards.
		const Eigen::Quaterniond residual(
			Eigen::AngleAxisd(0.3 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
		sim.calRot = (residual * kick.conjugate() * sim.calRot).normalized();
		RunContinuousSegment(sim, scene, 34.0, 58.0, rng, constTruth, constMount, alwaysVisible);
		bool resumed = sim.resumes == 1 && sim.freezes == 1 &&
			sim.ca.GetState() == ContinuousAlignment::State::Tracking &&
			sim.corrections > correctionsAtKick;

		snprintf(detail, sizeof detail,
			"freezes %d  resumes %d  unstable %d  corr while frozen %d  post-resume corr %d  state %d",
			sim.freezes, sim.resumes, sim.unstables, correctionsWhileFrozen,
			sim.corrections - correctionsAtKick - correctionsWhileFrozen,
			static_cast<int>(sim.ca.GetState()));
		Check("continuous: freeze then resume", trackingFirst && froze && resumed, detail);
	}

	// 17. The field lookup is a function of the tracker's BASE-CALIBRATED world
	// position, so the closure must scale the raw target position before the
	// base transform -- exactly what ContinuousTick's expectedAt does
	// (Overlay/Calibration.cpp:1663-1670). At scale 1.0 that factor is
	// invisible, which is why scenarios 9-11 never exercised it. Run the same
	// anchor crossing at scale 1.25, once through a scale-carrying closure and
	// once through one that drops the factor (the shape of the bug): the first
	// must stay silent, the second must invent corrections because it reads the
	// field 20% of the way back toward the origin.
	{
		const double calScale = 1.25;
		const FieldTransform base{ Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero() };
		std::vector<OverlayAnchor> anchors{
			{ Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
				Eigen::Vector3d(0.06, 0.0, 0.0) } };

		auto lookupAt = [&](const Eigen::Vector3d &basePos,
			Eigen::Quaterniond &rotationOut, Eigen::Vector3d &translationOut)
		{
			BlendedFieldCalibration(anchors, base.R, base.T, basePos,
				rotationOut, translationOut);
		};
		auto withScale = [&](const Eigen::Vector3d &targetRawPos,
			Eigen::Quaterniond &rotationOut, Eigen::Vector3d &translationOut)
		{
			lookupAt(base.R * (calScale * targetRawPos) + base.T, rotationOut, translationOut);
		};
		auto withoutScale = [&](const Eigen::Vector3d &targetRawPos,
			Eigen::Quaterniond &rotationOut, Eigen::Vector3d &translationOut)
		{
			lookupAt(base.R * targetRawPos + base.T, rotationOut, translationOut);
		};

		MountExtrinsic identityMount;
		identityMount.valid = true;
		ContinuousAlignment scaled, unscaled;
		scaled.SetExtrinsic(identityMount);
		unscaled.SetExtrinsic(identityMount);

		std::mt19937 rng(912);
		std::normal_distribution<double> refNoise(0.0, 0.0001);
		int scaledCorrections = 0, unscaledCorrections = 0;
		double unscaledMaxCorrection = 0.0;
		for (double t = 0.0; t < 16.0; t += 1.0 / 90.0)
		{
			// World sweep from 5 m out into the anchor's centre. The tracker
			// reports RAW positions; the driver scales them by calScale, so the
			// raw track is the world track divided by it.
			double worldX = t < 10.0 ? -5.0 + 0.5 * t : 0.0;
			double worldVx = t < 10.0 ? 0.5 : 0.0;

			PoseSample target;
			target.time = t;
			target.rot = Eigen::Quaterniond::Identity();
			target.pos = Eigen::Vector3d(worldX / calScale, 0.0, 0.0);
			target.vel = Eigen::Vector3d(worldVx / calScale, 0.0, 0.0);

			Eigen::Quaterniond localRot;
			Eigen::Vector3d localTrans;
			withScale(target.pos, localRot, localTrans);

			PoseSample reference;
			reference.time = t;
			reference.rot = localRot;
			reference.pos = localRot * (calScale * target.pos) + localTrans +
				Eigen::Vector3d(refNoise(rng), refNoise(rng), refNoise(rng));
			reference.vel = Eigen::Vector3d(worldVx, 0.0, 0.0);

			scaled.PushReference(reference);
			scaled.PushTarget(target);
			unscaled.PushReference(reference);
			unscaled.PushTarget(target);

			scaled.Update(t, base.R, base.T, calScale, 0.0, withScale);
			unscaled.Update(t, base.R, base.T, calScale, 0.0, withoutScale);

			ContinuousAlignment::Correction correction;
			while (scaled.PollCorrection(correction))
				scaledCorrections++;
			while (unscaled.PollCorrection(correction))
			{
				unscaledCorrections++;
				unscaledMaxCorrection = std::max(unscaledMaxCorrection,
					correction.translation.norm());
			}
		}

		snprintf(detail, sizeof detail,
			"scale %.2f: with-scale %d corrections / state %d; scale-dropped %d, max %.1f mm",
			calScale, scaledCorrections, static_cast<int>(scaled.GetState()),
			unscaledCorrections, unscaledMaxCorrection * 1000.0);
		Check("continuous: field lookup scale",
			scaledCorrections == 0 &&
			scaled.GetState() == ContinuousAlignment::State::Tracking &&
			unscaledCorrections > 0 && unscaledMaxCorrection > 0.005,
			detail);
	}
}

} // namespace

int main(int argc, char **argv)
{
	int propertyTrials = 64;
	uint32_t propertySeed = 0x5EED1234u;
	for (int i = 1; i < argc; ++i)
	{
		std::string arg = argv[i];
		if ((arg == "--property-trials" || arg == "--property-seed") && i + 1 < argc)
		{
			try
			{
				size_t used = 0;
				unsigned long value = std::stoul(argv[++i], &used, 0);
				if (used != std::string(argv[i]).size())
					throw std::invalid_argument("trailing characters");
				if (arg == "--property-trials")
				{
					if (value < 1 || value > 100000)
						throw std::out_of_range("property trial count");
					propertyTrials = static_cast<int>(value);
				}
				else
				{
					propertySeed = static_cast<uint32_t>(value);
				}
			}
			catch (const std::exception &)
			{
				fprintf(stderr, "Invalid value for %s\n", arg.c_str());
				return 2;
			}
		}
		else
		{
			fprintf(stderr,
				"Usage: SolverTests.exe [--property-trials N] [--property-seed N]\n");
			return 2;
		}
	}

	GroundTruth truth;
	truth.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(1.9, Eigen::Vector3d::UnitY()));   // yaw-only truth
	truth.translation = Eigen::Vector3d(1.2, 0.03, -0.7);

	EngineConfig config;

	// Production-path helpers used by profile loading and the driver. These
	// checks pin the non-solver correctness fixes alongside the math suite.
	{
		Eigen::Quaterniond identity(1.0, 0.0, 0.0, 0.0);
		Eigen::Quaterniond zero(0.0, 0.0, 0.0, 0.0);
		double maxDouble = std::numeric_limits<double>::max();
		Eigen::Quaterniond enormous(maxDouble, maxDouble, maxDouble, maxDouble);
		Eigen::Vector3d finite(1.0, -2.0, 3.0);
		Eigen::Vector3d nonfinite = finite;
		nonfinite.x() = std::numeric_limits<double>::infinity();
		Eigen::Vector3d outOfRange(protocol::limits::MaxAbsTranslationMeters + 1.0, 0.0, 0.0);
		Eigen::Vector3d anchorPosition(1.0, 0.0, 2.0);
		Eigen::Vector3d anchorTranslation(0.02, 0.0, -0.03);
		Eigen::Vector3d oversizedDelta(protocol::limits::MaxAbsAnchorDeltaMeters + 1.0, 0.0, 0.0);
		bool pass =
			IsValidTrackingSystemPair("oculus", "lighthouse") &&
			!IsValidTrackingSystemPair("oculus", "oculus") &&
			!IsValidTrackingSystemPair("", "lighthouse") &&
			!IsValidTrackingSystemPair("oculus", "") &&
			IsValidCalibrationTransform(identity, finite, 1.0) &&
			!IsValidCalibrationTransform(zero, finite, 1.0) &&
			!IsValidCalibrationTransform(enormous, finite, 1.0) &&
			!IsValidCalibrationTransform(identity, nonfinite, 1.0) &&
			!IsValidCalibrationTransform(identity, outOfRange, 1.0) &&
			IsValidFieldAnchor(anchorPosition, identity, anchorTranslation,
				identity, Eigen::Vector3d::Zero()) &&
			!IsValidFieldAnchor(anchorPosition, identity, oversizedDelta,
				identity, Eigen::Vector3d::Zero()) &&
			!IsValidFieldAnchor(anchorPosition, enormous, anchorTranslation,
				identity, Eigen::Vector3d::Zero()) &&
			!IsValidScale(0.0) && !IsValidScale(-1.0) &&
			!IsValidScale(std::numeric_limits<double>::infinity()) &&
			!IsValidScale(10.0);
		printf("%-28s %s\n", "profile semantics", pass ? "PASS" : "FAIL");
		RecordResult(pass);
	}
	bool corruptMissingUse = CanUseRecoveredSettings(
		RecordLoadState::Missing, RecordLoadState::Unreadable);
	bool corruptValidUse = CanUseRecoveredSettings(
		RecordLoadState::Loaded, RecordLoadState::Unreadable);
	bool corruptMayWrite = CanMaterializeSettings(RecordLoadState::Unreadable);
	bool missingMayWrite = CanMaterializeSettings(RecordLoadState::Missing);
	bool parsedMayWrite = CanMaterializeSettings(RecordLoadState::Loaded);
	bool corruptMissingConfigWrite = CanPersistConfig(RecordLoadState::Unreadable);
	bool corruptMissingSettingsWrite = CanPersistSettings(
		RecordLoadState::Unreadable, RecordLoadState::Missing);
	bool corruptValidSettingsWrite = CanPersistSettings(
		RecordLoadState::Unreadable, RecordLoadState::Loaded);
	Check("persistence: Config recovery gate",
		!corruptMissingUse && corruptValidUse && !corruptMayWrite &&
		missingMayWrite && parsedMayWrite && !corruptMissingConfigWrite &&
		!corruptMissingSettingsWrite && corruptValidSettingsWrite,
		"corrupt Config never writable; valid separate Settings remains usable/writable");

	{
		double position[3] = { 2.0, -4.0, 6.0 };
		double velocity[3] = { -8.0, 10.0, 12.0 };
		double acceleration[3] = { 14.0, -16.0, 18.0 };
		ScaleLinearPose(0.5, position, velocity, acceleration);
		bool pass =
			position[0] == 1.0 && position[1] == -2.0 && position[2] == 3.0 &&
			velocity[0] == -4.0 && velocity[1] == 5.0 && velocity[2] == 6.0 &&
			acceleration[0] == 7.0 && acceleration[1] == -8.0 && acceleration[2] == 9.0;
		printf("%-28s %s\n", "driver linear scale", pass ? "PASS" : "FAIL");
		RecordResult(pass);
	}

	// Production-shared driver algebra and broad solver edge/property passes.
	RunDriverPoseTransformScenarios();
	RunDriverProtocolValidationScenarios();
	RunPoseChannelScenarios();
	RunSolverPrimitiveScenarios();
	RunSolverRobustnessScenarios();
	RunSolverPropertyScenarios(propertyTrials, propertySeed);

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
		// Deliberately below the uncompensated damage the counterfactual below
		// measures (~0.185 deg / 5.4 mm on this exact data). At the old
		// 0.5 deg / 15 mm the two scenarios overlapped: an inert or inverted
		// application at CalibrationEngine.cpp:1020 landed inside the pass band
		// while r.timeOffset -- the ESTIMATE, produced before and independently
		// of the application -- still read +18.0 ms.
		e.maxRotErrDeg = 0.12;
		e.maxTransErrM = 0.004;
		RunScenario("latency 18ms", scene, t2, config, e);

		// Same data without compensation: document the damage (not asserted).
		EngineConfig noComp = config;
		noComp.estimateTimeOffset = false;
		Expectation e2;
		e2.expectValid = true;
		e2.maxRotErrDeg = 90.0;    // report-only run
		e2.maxTransErrM = 10.0;
		RunScenario("latency uncompensated", scene, t2, noComp, e2);

		// The absolute tolerances above still only bound the compensated
		// residual. What pins the APPLICATION is the ratio: compensating must
		// remove most of the damage its own counterfactual measures. A sign flip
		// at the application site roughly doubles the residual instead of
		// shrinking it, so this fails hard where an absolute band can be tuned
		// around. Same streams for both, so the comparison is exact.
		std::vector<PoseSample> refStream, targetStream;
		GenerateStreams(scene, t2, 1234, refStream, targetStream);
		EngineResult on = CalibrationEngine::Solve(refStream, targetStream, config);
		EngineResult off = CalibrationEngine::Solve(refStream, targetStream, noComp);

		double onRot = t2.rotation.angularDistance(on.rotation) * 180.0 / EIGEN_PI;
		double offRot = t2.rotation.angularDistance(off.rotation) * 180.0 / EIGEN_PI;
		double onTrans = (on.translation - t2.translation).norm();
		double offTrans = (off.translation - t2.translation).norm();

		char detail[256];
		snprintf(detail, sizeof detail,
			"rot %.4f vs %.4f deg (%.0f%%)  trans %.1f vs %.1f mm (%.0f%%)",
			onRot, offRot, 100.0 * onRot / std::max(offRot, 1e-12),
			onTrans * 1000.0, offTrans * 1000.0, 100.0 * onTrans / std::max(offTrans, 1e-12));
		Check("latency compensation separates",
			on.valid && off.valid &&
			onRot < 0.5 * offRot && onTrans < 0.5 * offTrans, detail);
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

	// 6b. Yaw sweeps with one brief nod burst that returns to neutral: the
	// burst's tilted deltas push the axis cloud past the spread gate, but
	// every LARGE delta is still pure yaw (long-lag pairs span the whole
	// burst and see no net nod), so the eq. 8 translation system stays
	// near-singular along the vertical — the conditioning gate must refuse
	// rather than ship a noise-driven floor height.
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.2;
		scene.offAxisScale = 0.6;
		scene.offAxisRate = 2.0;
		scene.offAxisBurstT0 = 5.0;
		scene.offAxisBurstT1 = 6.5;
		Expectation e;
		e.expectValid = false;
		e.messageContains = "pin the position";
		RunScenario("yaw + one nod burst", scene, truth, config, e);
	}

	// 6b'. The accept side of the same boundary: SUSTAINED slight nodding at
	// the same axis-spread scale keeps large deltas tilted too, so the
	// translation system is conditioned and the solve must go through.
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.2;
		scene.offAxisScale = 0.16;
		scene.offAxisRate = 2.0;
		Expectation e;
		e.maxRotErrDeg = 0.6;
		e.maxTransErrM = 0.02;
		RunScenario("sustained slight nods", scene, truth, config, e);
	}

	// 6b''. Slow, cautious motion: everything scaled down so most admitted
	// pairs sit just above the minimum pair angle, where axis-direction noise
	// is amplified by 1/theta — the inverse-variance angle weighting keeps
	// those pairs from dominating the rotation solve.
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.4;
		scene.motionScale = 0.35;
		Expectation e;
		e.maxRotErrDeg = 0.35;
		e.maxTransErrM = 0.02;
		RunScenario("slow cautious motion", scene, truth, config, e);
	}

	// 6c. Long continuous sweeps: large-lag deltas pass through 180 deg, where
	// the two streams' independent shortest-arc hemisphere choices decorrelate
	// under noise. The near-pi pair gate must keep that band out of the
	// initial Kabsch; recovery stays at normal-noise accuracy.
	{
		SceneConfig scene;
		scene.duration = 40.0;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.4;
		Expectation e;
		e.maxRotErrDeg = 0.6;
		e.maxTransErrM = 0.02;
		RunScenario("near-180 sweeps", scene, truth, config, e, 4321);
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

	// 7b. Joint refinement: the closing Gauss-Newton polish over (R, t, mount,
	// s) must never do worse than the sequential pipeline it starts from — it
	// exists to stop residual Kabsch rotation error from leaking into the
	// translation with a play-space lever arm.
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.3;

		std::vector<PoseSample> refStream, targetStream;
		GenerateStreams(scene, truth, 555, refStream, targetStream);

		EngineConfig sequential = config;
		sequential.refineIterations = 0;
		EngineResult seq = CalibrationEngine::Solve(refStream, targetStream, sequential);
		EngineResult joint = CalibrationEngine::Solve(refStream, targetStream, config);

		double seqErr = (seq.translation - truth.translation).norm();
		double jointErr = (joint.translation - truth.translation).norm();
		double seqRot = truth.rotation.angularDistance(seq.rotation) * 180.0 / EIGEN_PI;
		double jointRot = truth.rotation.angularDistance(joint.rotation) * 180.0 / EIGEN_PI;

		// "Never worse" is the contract (the production guard accepts ties and
		// up to 2% axis-RMS regression); don't demand strict improvement.
		bool pass = seq.valid && joint.valid &&
			!seq.refinementApplied && joint.refinementApplied &&
			jointErr <= seqErr + 0.001 &&
			jointRot <= seqRot + 0.05;
		printf("%-28s %s  applied %d  trans %.4f -> %.4f m  rot %.4f -> %.4f deg\n",
			"joint refinement", pass ? "PASS" : "FAIL",
			joint.refinementApplied, seqErr, jointErr, seqRot, jointRot);
		RecordResult(pass);
	}

	// 7c. Scale-artifact discriminator: validates the live-diagnosis advice
	// ("solved scale that sinks with faster motion = reference-stream
	// smoothing artifact; solved scale invariant to motion speed = genuine
	// metric difference") against simulated ground truth for each hypothesis.
	{
		EngineConfig sc = config;
		sc.solveScale = true;

		SceneConfig scene;
		scene.posNoise = 0.001;
		scene.rotNoiseDeg = 0.1;

		// Hypothesis A: true scale 1.0, reference stream amplitude-attenuated
		// by zero-phase smoothing. tau puts the harness's ~1.2 rad/s motion in
		// the same omega*tau regime as real 1-2 Hz calibration wiggling under
		// ~60-80 ms streaming smoothing; the 3x time stretch is the "calibrate
		// slowly" advice, which drops omega*tau and lets the amplitude through.
		std::vector<PoseSample> ref, tgt;
		GenerateStreams(scene, truth, 777, ref, tgt);
		std::vector<PoseSample> refSm = SmoothStreamZeroPhase(ref, 0.3, 0.05);

		EngineConfig rawCfg = sc;
		rawCfg.pinScaleOnSmoothing = false;
		EngineResult aRaw = CalibrationEngine::Solve(refSm, tgt, rawCfg);
		EngineResult aFast = CalibrationEngine::Solve(refSm, tgt, sc);
		EngineResult aSlow = CalibrationEngine::Solve(
			SmoothStreamZeroPhase(StretchTime(ref, 3.0), 0.3, 0.05), StretchTime(tgt, 3.0), sc);

		// Hypothesis B: genuine metric difference, no smoothing.
		GroundTruth shrunk = truth;
		shrunk.scale = 0.93;
		std::vector<PoseSample> refB, tgtB;
		GenerateStreams(scene, shrunk, 778, refB, tgtB);
		EngineResult bFast = CalibrationEngine::Solve(refB, tgtB, sc);
		EngineResult bSlow = CalibrationEngine::Solve(StretchTime(refB, 3.0), StretchTime(tgtB, 3.0), sc);

		// The harness trajectory has no truly slow component (shortest omega
		// ~0.8 rad/s), so at the simulated tau even the gross band is
		// attenuated. That must select neutral 1.0 rather than bless a dirty
		// "gross" estimate; the stretched trajectory supplies the clean-gross
		// path. A genuine metric difference stays frequency-flat and passes
		// through untouched.
		bool artifact = aRaw.valid && aFast.valid && aSlow.valid &&
			aRaw.scale < 0.97 &&                              // raw solve collapses at speed
			aFast.motionGainValid && aFast.motionSmoothingDetected &&
			aFast.motionGainHigh < aFast.motionGainLow - 0.02 &&  // diagnostic sees it
			aFast.scaleGuard == ScaleGuard::NeutralizedForSmoothing &&
			std::abs(aFast.scale - 1.0) < 0.001 &&             // dirty gross => neutral
			aSlow.scale > 0.97;                               // slow motion nears truth
		bool genuine = bFast.valid && bSlow.valid &&
			!bFast.motionSmoothingDetected && !bSlow.motionSmoothingDetected &&
			bFast.scaleGuard == ScaleGuard::NotApplied &&
			bSlow.scaleGuard == ScaleGuard::NotApplied &&
			std::abs(bFast.scale - 0.93) < 0.01 &&            // speed-invariant either way
			std::abs(bSlow.scale - 0.93) < 0.01;

		bool pass = artifact && genuine;
		printf("%-28s %s  smoothed raw %.3f guarded %.3f slow %.3f (gain %.3f/%.3f guard %d)  genuine fast %.3f slow %.3f (gain %.3f/%.3f)  valid %d%d%d%d%d\n",
			"scale discriminator", pass ? "PASS" : "FAIL",
			aRaw.scale, aFast.scale, aSlow.scale,
			aFast.motionGainLow, aFast.motionGainHigh,
			static_cast<int>(aFast.scaleGuard),
			bFast.scale, bSlow.scale,
			bFast.motionGainLow, bFast.motionGainHigh,
			aRaw.valid, aFast.valid, aSlow.valid, bFast.valid, bSlow.valid);
		RecordResult(pass);
		if (!pass)
		{
			for (const EngineResult *r : { &aRaw, &aFast, &aSlow, &bFast, &bSlow })
				if (!r->valid)
					printf("%-28s      rotRms %.2f transRms %.4f spread %.4f cond %.4f: %s\n",
						"", r->rotationRmsDeg, r->translationRmsMeters,
						r->axisSpread, r->transEigRatio, r->message.c_str());
		}
	}

	// 7d. Scale diagnostic branch coverage: a short otherwise-valid session
	// cannot identify frequency bands and must leave gain diagnostics disabled;
	// a moderate low-pass with clean gross motion must take the guarded scale
	// from that band rather than neutralizing it.
	{
		EngineConfig sc = config;
		sc.solveScale = true;

		SceneConfig shortScene;
		shortScene.duration = 4.0;
		shortScene.posNoise = 0.001;
		GroundTruth shortTruth = truth;
		shortTruth.scale = 1.02;
		std::vector<PoseSample> shortRef, shortTgt;
		GenerateStreams(shortScene, shortTruth, 779, shortRef, shortTgt);
		EngineResult shortResult = CalibrationEngine::Solve(shortRef, shortTgt, sc);

		SceneConfig scene;
		scene.posNoise = 0.001;
		scene.rotNoiseDeg = 0.1;
		std::vector<PoseSample> ref, tgt;
		GenerateStreams(scene, truth, 780, ref, tgt);

		// The default threshold's neutral branch is pinned by "scale
		// discriminator" above. Widen only the definition of clean gross here
		// so the alternate replacement branch is deterministically reachable
		// with the same known attenuation fingerprint.
		EngineConfig grossCfg = sc;
		grossCfg.maxCleanGrossDeviation = 0.10;
		EngineResult cleanGross = CalibrationEngine::Solve(
			SmoothStreamZeroPhase(ref, 0.30, 0.05), tgt, grossCfg);
		bool cleanGrossSeen = cleanGross.valid &&
			cleanGross.scaleGuard == ScaleGuard::FromGrossMotion;

		// The gain diagnostic abstains on this short stream. That means the
		// scale is not identifiable from this motion, NOT that it is clean, so
		// the guard takes the neutral path instead of committing the
		// free-scale fit. (That fit lands near this scene's 1.02 ground truth,
		// which is exactly what made the old silent commit look harmless.)
		bool pass = shortResult.valid && !shortResult.motionGainValid &&
			shortResult.scale == 1.0 &&
			shortResult.scaleGuard == ScaleGuard::NeutralizedForSmoothing &&
			cleanGrossSeen && cleanGross.motionSmoothingDetected &&
			std::abs(cleanGross.scale - cleanGross.motionGainLow) < 1e-6 &&
			std::abs(cleanGross.scale - 1.0) <= grossCfg.maxCleanGrossDeviation + 1e-6;
		printf("%-28s %s  short valid/gain/scale/guard %d/%d/%.3f/%d  clean gross %d scale %.3f gain %.3f/%.3f\n",
			"scale diagnostic branches", pass ? "PASS" : "FAIL",
			shortResult.valid, shortResult.motionGainValid, shortResult.scale,
			static_cast<int>(shortResult.scaleGuard),
			cleanGrossSeen, cleanGross.scale, cleanGross.motionGainLow, cleanGross.motionGainHigh);
		RecordResult(pass);
	}

	// 7e. Scale guard, fail-closed branch. When streamed-pose smoothing is
	// detected and the guarded fixed-scale re-solve fails its OWN gates, the
	// result must be marked invalid rather than falling back to the contaminated
	// free-scale fit: that fit collapses to ~0.88 for a true 1.0, and the driver
	// multiplies every position, velocity and acceleration by it, shrinking the
	// user's whole target universe ~12% behind a green verdict. 7c and 7d pin
	// both success branches; this else has never executed.
	//
	// Reuse 7c's hypothesis-A data, whose fingerprint (smoothing detected, gross
	// band dirty, guarded scale neutralised to 1.0) is already asserted there.
	// The residual gate is then placed BETWEEN the two fits' own measured
	// residuals rather than at a guessed constant: the free-scale fit absorbs
	// the attenuation into scale, the scale-pinned fit cannot, so one threshold
	// admits the first and rejects the second — exactly the case the branch
	// exists for, with no magic numbers to go stale.
	{
		EngineConfig sc = config;
		sc.solveScale = true;

		SceneConfig scene;
		scene.posNoise = 0.001;
		scene.rotNoiseDeg = 0.1;
		std::vector<PoseSample> ref, tgt;
		GenerateStreams(scene, truth, 777, ref, tgt);
		std::vector<PoseSample> refSm = SmoothStreamZeroPhase(ref, 0.3, 0.05);

		EngineConfig freeCfg = sc;
		freeCfg.pinScaleOnSmoothing = false;
		EngineResult freeFit = CalibrationEngine::Solve(refSm, tgt, freeCfg);

		// Exactly the fit the guard's re-solve performs on this data: scale
		// pinned, targets pre-scaled by the guarded 1.0 (i.e. unchanged).
		EngineConfig pinnedCfg = config;
		pinnedCfg.solveScale = false;
		EngineResult pinnedFit = CalibrationEngine::Solve(refSm, tgt, pinnedCfg);

		double gate = 0.5 * (freeFit.translationRmsMeters + pinnedFit.translationRmsMeters);
		EngineConfig guardCfg = sc;
		guardCfg.maxTranslationRms = gate;
		EngineResult guarded = CalibrationEngine::Solve(refSm, tgt, guardCfg);

		// A collapsed separation would let this pass vacuously; fail loudly.
		// Measured separation on this data is ~1.9x, so the bar sits below that
		// and well above the ~1.0 a collapse would produce.
		bool separated = pinnedFit.translationRmsMeters > 1.5 * freeFit.translationRmsMeters;
		bool pass = freeFit.valid && separated && !guarded.valid &&
			guarded.message.find("guarded fixed-scale re-solve failed") != std::string::npos;
		printf("%-28s %s  rms free %.4f m (scale %.3f) vs pinned %.4f m  gate %.4f  guarded valid %d: %s\n",
			"scale guard fail-closed", pass ? "PASS" : "FAIL",
			freeFit.translationRmsMeters, freeFit.scale, pinnedFit.translationRmsMeters, gate,
			guarded.valid, guarded.message.c_str());
		RecordResult(pass);
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
		RecordResult(pass);
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
		RecordResult(pass);
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

	printf("\n%d scenario(s) ran, %d failed\n", checksRun, failures);
	return failures;
}
