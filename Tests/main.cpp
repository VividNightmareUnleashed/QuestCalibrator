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
#include "../Driver/IPCServer.h"
#include "../Driver/Logging.h"
#include "../Overlay/CalibrationEngine.h"
#include "../Overlay/CalibrationRun.h"
#include "../Overlay/ChaperoneMath.h"
#include "../Overlay/ContinuousAlignment.h"
#include "../Overlay/ContinuousCorrectionGate.h"
#include "../Overlay/FieldMath.h"
#include "../Overlay/DriftMonitor.h"
#include "../Overlay/DriverSession.h"
#include "../Overlay/DriverSyncPolicy.h"
#include "../Overlay/DriverSyncTracker.h"
#include "../Overlay/CalibrationGuide.h"
#include "../Overlay/LegacyContinuous.h"
#include "../Overlay/UpdatePolicy.h"
#include "../Overlay/DriverWorker.h"
#include "../Overlay/JumpDetector.h"
#include "../Overlay/PersistenceState.h"
#include "../Overlay/ProfileValidation.h"
#include "../Overlay/ProfileRecordJson.h"
#include "../Overlay/RingPoseMath.h"
#include "../Overlay/PoseStreamHub.h"
#include "../common/PoseChannel.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

bool CalibrationContextResetScenario();
bool ControllerTriggerAxisScenario();
bool CalibrationContextCadenceScenario();
bool CalibrationContextCorrectionBasisScenario();
bool ContinuousInputDiagnosticsScenario();
bool PoseStreamDiagnosticsScenario();
bool DiagnosticsExportScenario();
bool ContinuousWindowDiagnosticsScenario();
bool ContinuousPairingDiagnosticsScenario();
void RunReviewRegressionScenarios(void (*check)(const char *, bool, const char *));

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

	// Noise and outliers. Same constraint as the timestamp jitter in
	// GenerateStreamsWithMount: normal_distribution requires sigma > 0 at
	// CONSTRUCTION, and a clean scene sets both of these to zero. The draws
	// below are already gated on the same values, so the substitute sigma is
	// never sampled - but constructing with zero aborts the Debug STL, which
	// is why no Debug build of this harness has ever run to completion.
	std::normal_distribution<double> pn(0.0, scene.posNoise > 0.0 ? scene.posNoise : 1.0);
	std::normal_distribution<double> rn(0.0,
		scene.rotNoiseDeg > 0.0 ? scene.rotNoiseDeg * EIGEN_PI / 180.0 : 1.0);
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

	// Every rejection above poisons anchor 0, and the accepted field carries one
	// anchor - so a loop bound of `i < 1`, or hoisting the per-anchor checks out
	// of the loop, passes all of them. Build a full field and poison its LAST
	// anchor. The output is value-initialised first, so an unvalidated tail does
	// not fault: the user's per-spot corrections are silently dropped to identity
	// behind a success response from the driver.
	protocol::SetAlignmentField tailField = goodField;
	tailField.anchorCount = protocol::SetAlignmentField::MaxAnchors;
	for (uint32_t i = 0; i < tailField.anchorCount; ++i)
	{
		tailField.anchors[i].position[0] = 1.0 + static_cast<double>(i);
		tailField.anchors[i].rotationDelta = { 1.0, 0.0, 0.0, 0.0 };
		tailField.anchors[i].translationDelta[2] = -0.05;
	}
	const uint32_t lastAnchor = protocol::SetAlignmentField::MaxAnchors - 1;
	protocol::SetAlignmentField sanitizedTail;
	// Non-vacuity: the tail must survive a clean field, or the rejections below
	// would prove nothing about where the loop stops.
	pass = pass && ValidateAndSanitize(tailField, sanitizedTail) &&
		sanitizedTail.anchors[lastAnchor].position[0] ==
			1.0 + static_cast<double>(lastAnchor);

	badField = tailField; badField.anchors[lastAnchor].position[1] = inf;
	pass = pass && rejectsField(badField);
	badField = tailField; badField.anchors[lastAnchor].position[2] = 10001.0;
	pass = pass && rejectsField(badField);
	badField = tailField; badField.anchors[lastAnchor].translationDelta[0] = 100.01;
	pass = pass && rejectsField(badField);
	badField = tailField; badField.anchors[lastAnchor].rotationDelta = { 0.0, 0.0, 0.0, 0.0 };
	pass = pass && rejectsField(badField);

	// Unused wire anchors are deliberately scrubbed instead of trusted.
	protocol::SetAlignmentField unusedGarbage;
	unusedGarbage.anchorCount = 0;
	unusedGarbage.anchors[0].position[0] = nan;
	protocol::SetAlignmentField scrubbed;
	pass = pass && ValidateAndSanitize(unusedGarbage, scrubbed) &&
		scrubbed.anchors[0].position[0] == 0.0 && scrubbed.anchors[0].rotationDelta.w == 1.0;

	protocol::SetRuntimeState runtime;
	runtime.enabledMask = (uint64_t{ 1 } << 3) | (uint64_t{ 1 } << 7);
	runtime.hiddenMask = uint64_t{ 1 } << 7;
	runtime.transform = good;
	runtime.transform.openVRID = 0;
	runtime.transform.enabled = 1;
	runtime.transform.hidden = 0;
	runtime.field = goodField;
	protocol::SetRuntimeState cleanRuntime;
	pass = pass && ValidateAndSanitize(runtime, cleanRuntime) &&
		cleanRuntime.enabledMask == runtime.enabledMask &&
		cleanRuntime.hiddenMask == runtime.hiddenMask;
	auto rejectsRuntime = [&](protocol::SetRuntimeState candidate)
	{
		protocol::SetRuntimeState ignored;
		return !ValidateAndSanitize(candidate, ignored);
	};
	auto badRuntime = runtime;
	badRuntime.hiddenMask |= uint64_t{ 1 } << 9;
	pass = pass && rejectsRuntime(badRuntime);
	badRuntime = runtime; badRuntime.transform.openVRID = 3;
	pass = pass && rejectsRuntime(badRuntime);
	badRuntime = runtime; badRuntime.transform.enabled = 0;
	pass = pass && rejectsRuntime(badRuntime);
	badRuntime = runtime; badRuntime.transform.hidden = 1;
	pass = pass && rejectsRuntime(badRuntime);
	badRuntime = runtime; badRuntime.enabledMask = 0; badRuntime.hiddenMask = 0;
	pass = pass && rejectsRuntime(badRuntime);

	Check("driver: protocol validation", pass,
		"finite/range/quaternion/unused-anchor/atomic-state matrix");

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

	// A late wrong-version handshake REVOKES the connection: handshakeComplete is
	// assigned from the version comparison, never or-ed into. Without this, a
	// "once handshaken, always handshaken" relaxation would let a peer that
	// downgrades mid-connection (or a second local process reusing the pipe after
	// a good handshake) keep mutating at a version this driver no longer speaks,
	// and no existing assertion would notice.
	bool revokeDispatched = questcal::ipc::PrepareRequest(wrongHandshake, connection, response);
	protocol::Request currentMutation(protocol::RequestSetDeviceTransform);
	bool afterRevoke = questcal::ipc::PrepareRequest(currentMutation, connection, response);
	Check("driver: late downgrade revokes handshake",
		!revokeDispatched && !connection.handshakeComplete && !afterRevoke &&
			response.type == protocol::ResponseInvalid,
		"a wrong-version handshake after a good one un-authorizes the connection");

	// The gate's `mutation` clause names BOTH mutating request types, and only
	// SetDeviceTransform was ever driven through it. Dropping the
	// RequestSetRuntimeState term would make every complete-state send fall
	// through to ResponseInvalid -- indistinguishable, on the wire, from a broken
	// handshake -- while the driver's own field validation kept passing.
	questcal::ipc::ConnectionState fieldConnection;
	protocol::Request fieldMutation(protocol::RequestSetRuntimeState);
	bool fieldPreHandshake =
		questcal::ipc::PrepareRequest(fieldMutation, fieldConnection, response);
	protocol::Request fieldHandshake(protocol::RequestHandshake);
	questcal::ipc::PrepareRequest(fieldHandshake, fieldConnection, response);
	bool fieldAccepted =
		questcal::ipc::PrepareRequest(fieldMutation, fieldConnection, response);
	Check("driver: alignment-field dispatch gate",
		!fieldPreHandshake && fieldConnection.handshakeComplete && fieldAccepted,
		"SetRuntimeState refused before the handshake, dispatched after it");

	// Everything above exercises the gate in isolation. This is the first
	// coverage of anything in IPCServer.cpp itself: which sink a request
	// reaches, and what a setter's refusal becomes on the wire. The transport
	// half - overlapped pipe, per-connection state, listener backoff, teardown
	// drain - still needs a real named pipe and stays uncovered.
	if (!LogFile)
		LogFile = stderr;   // the LOG macro writes unconditionally

	IPCServer server;
	int transformCalls = 0, fieldCalls = 0;
	bool setterAccepts = true;
	IPCServer::RequestSink sink;
	sink.setDeviceTransform = [&](const protocol::SetDeviceTransform &)
	{
		++transformCalls;
		return setterAccepts;
	};
	sink.setRuntimeState = [&](const protocol::SetRuntimeState &)
	{
		++fieldCalls;
		return setterAccepts;
	};
	sink.poseHookMask = [] { return protocol::PoseHook006; };
	server.SetSinkForTest(sink);

	questcal::ipc::ConnectionState dispatchConn;
	protocol::Response dispatched(protocol::ResponseInvalid);
	protocol::Request transformReq(protocol::RequestSetDeviceTransform);
	protocol::Request fieldReq(protocol::RequestSetRuntimeState);

	// A gate refusal must not reach a setter at all: the driver never sees
	// values from a connection that has not proven its version.
	server.DispatchForTest(transformReq, dispatched, dispatchConn);
	bool noSetterBeforeHandshake =
		transformCalls == 0 && dispatched.type == protocol::ResponseInvalid;

	protocol::Request dispatchHandshake(protocol::RequestHandshake);
	server.DispatchForTest(dispatchHandshake, dispatched, dispatchConn);
	bool dispatchHandshakeOk = dispatched.type == protocol::ResponseHandshake &&
		dispatched.poseHookMask == protocol::PoseHook006;

	// Each mutation reaches its own setter and only its own.
	server.DispatchForTest(transformReq, dispatched, dispatchConn);
	bool transformRouted = transformCalls == 1 && fieldCalls == 0 &&
		dispatched.type == protocol::ResponseSuccess;
	server.DispatchForTest(fieldReq, dispatched, dispatchConn);
	bool fieldRouted = transformCalls == 1 && fieldCalls == 1 &&
		dispatched.type == protocol::ResponseSuccess;

	// A setter that refuses its values reports failure rather than reporting
	// success and dropping them.
	setterAccepts = false;
	server.DispatchForTest(transformReq, dispatched, dispatchConn);
	bool refusalReported =
		transformCalls == 2 && dispatched.type == protocol::ResponseInvalid;

	char dispatchDetail[128];
	snprintf(dispatchDetail, sizeof dispatchDetail,
		"transform %d field %d; pre-handshake/handshake/route/refusal %d%d%d%d",
		transformCalls, fieldCalls, noSetterBeforeHandshake, dispatchHandshakeOk,
		transformRouted && fieldRouted, refusalReported);
	Check("driver: request dispatch routing",
		noSetterBeforeHandshake && dispatchHandshakeOk && transformRouted &&
			fieldRouted && refusalReported,
		dispatchDetail);
}

// ---------------------------------------------------------------------------
// Overlay -> driver slot reconciliation (Overlay/DriverSyncPolicy.h)
//
// Pure policy for which slots receive the calibration and how their complete
// masks are derived.

// A device table indexed by OpenVR id. A slot nobody filled in enumerates as
// TrackedDeviceClass_Invalid, which is exactly what OpenVR reports for an id it
// no longer exposes -- and the driver slot outlives that disappearance.
struct SyncDeviceTable
{
	questcal::SyncDevice devices[vr::k_unMaxTrackedDeviceCount];

	SyncDeviceTable()
	{
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			devices[id].id = id;
	}

	// A null trackingSystem models a failed property read, which is not the same
	// as a device on an unnamed system.
	void Place(uint32_t id, questcal::SyncDeviceClass deviceClass,
		const char *trackingSystem, const char *serial = nullptr)
	{
		questcal::SyncDevice &device = devices[id];
		device.id = id;
		device.deviceClass = deviceClass;
		device.trackingSystemKnown = trackingSystem != nullptr;
		device.trackingSystem = trackingSystem ? trackingSystem : "";
		device.serialKnown = serial != nullptr;
		device.serial = serial ? serial : "";
	}

	void Remove(uint32_t id)
	{
		devices[id] = questcal::SyncDevice();
		devices[id].id = id;
	}
};

questcal::DriverSyncDesired MakeDriverSyncDesired()
{
	questcal::DriverSyncDesired desired;
	desired.referenceTrackingSystem = "lighthouse";
	desired.targetTrackingSystem = "oculus";
	desired.rotation =
		Eigen::Quaterniond(Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitY()));
	desired.translationMeters = Eigen::Vector3d(0.4, -0.1, 1.2);
	desired.scale = 1.02;
	desired.timeShift = -0.012;
	desired.baseGeneration = 9;
	return desired;
}

void RunDriverSyncScenarios()
{
	questcal::DriverSyncDesired desired = MakeDriverSyncDesired();

	questcal::SyncDevice target;
	target.id = 4;
	target.deviceClass = questcal::SyncDeviceClass::Other;
	target.trackingSystemKnown = true;
	target.trackingSystem = "oculus";
	auto applied = questcal::DecideSlot(desired, target);
	bool payload = applied.action == questcal::SlotAction::ApplyTransform &&
		applied.transform.openVRID == 4 && applied.transform.enabled == 1 &&
		applied.transform.generation == desired.baseGeneration &&
		std::abs(applied.transform.scale - desired.scale) < 1e-12 &&
		std::abs(applied.transform.timeOffset - desired.timeShift) < 1e-12 &&
		std::abs(applied.transform.translation.v[0] - desired.translationMeters(0)) < 1e-12 &&
		std::abs(applied.transform.rotation.y - desired.rotation.y()) < 1e-12;
	Check("driver sync: target payload", payload,
		"one target decision carries the complete canonical transform");

	questcal::SyncDevice hmd;
	hmd.id = vr::k_unTrackedDeviceIndex_Hmd;
	hmd.deviceClass = questcal::SyncDeviceClass::Hmd;
	hmd.trackingSystemKnown = true;
	hmd.trackingSystem = "lighthouse";
	auto reference = questcal::DecideSlot(desired, hmd);
	hmd.trackingSystem = "oculus";
	auto foreign = questcal::DecideSlot(desired, hmd);
	questcal::SyncDevice unknown = target;
	unknown.trackingSystemKnown = false;
	auto unreadable = questcal::DecideSlot(desired, unknown);
	Check("driver sync: identity gates",
		reference.referenceDevice && reference.action == questcal::SlotAction::None &&
		!reference.disableProfile && foreign.disableProfile &&
		unreadable.action == questcal::SlotAction::None &&
		!unreadable.referenceDevice && !unreadable.targetDevice,
		"reference HMD is never transformed; foreign HMD disables; unknown stays neutral");

	desired.continuousTrackerSerial = "T-MOUNT";
	desired.continuousArmed = true;
	desired.hideMountedTracker = true;
	target.serialKnown = true;
	target.serial = "T-MOUNT";
	auto mounted = questcal::DecideSlot(desired, target);
	target.serial = "T-FOOT";
	auto ordinary = questcal::DecideSlot(desired, target);
	desired.continuousArmed = false;
	target.serial = "T-MOUNT";
	auto unarmed = questcal::DecideSlot(desired, target);
	Check("driver sync: mounted tracker",
		mounted.continuousTracker && mounted.transform.hidden == 1 &&
		!ordinary.continuousTracker && ordinary.transform.hidden == 0 &&
		unarmed.continuousTracker && unarmed.transform.hidden == 0,
		"only the armed tracker serial is hidden");

	questcal::SyncDevice pastEnd = target;
	pastEnd.id = vr::k_unMaxTrackedDeviceCount;
	questcal::SyncDevice last = target;
	last.id = vr::k_unMaxTrackedDeviceCount - 1;
	Check("driver sync: slot bounds",
		questcal::DecideSlot(desired, pastEnd).action == questcal::SlotAction::None &&
		questcal::DecideSlot(desired, last).action == questcal::SlotAction::ApplyTransform,
		"out-of-range ids decide nothing; slot 63 remains usable");
}

// ---------------------------------------------------------------------------
// Overlay -> driver session sequencing (Overlay/DriverSession.h)
//
// Session sequencing determines request order and what the overlay may believe
// afterward. Fault injection drives refusals, disconnects, and reconnects
// through the production transport seams.

enum class LinkFault
{
	None,
	// The driver answered, and said no.
	Refuse,
	// The send failed and IPCClient's one reconnect-and-replay failed too.
	Throw,
	// SendBlocking's replay-after-reconnect: the request WAS accepted, but by a
	// new pipe. Nothing in the response says so -- the connection generation is
	// the only evidence, which is the whole reason a batch stamps one.
	Reconnect,
};

// A scripted driver connection. Records every request in order and answers each
// from a small script keyed on the request itself, so a scenario names the
// failure the way the invariant does ("the enable for slot 3 is refused")
// rather than by counting round-trips.
struct FakeDriverLink
{
	std::vector<protocol::Request> sent;
	// Real connections start at 1: zero is the session's "this batch has not
	// been stamped yet" sentinel, so a scripted zero would drive a state
	// production cannot reach.
	uint64_t generation = 1;
	std::function<LinkFault(size_t index, const protocol::Request &)> script;

	questcal::DriverTransport Transport()
	{
		return [this](const protocol::Request &request)
		{
			size_t index = sent.size();
			sent.push_back(request);
			LinkFault fault = script ? script(index, request) : LinkFault::None;

			questcal::DriverTransportResult result;
			if (fault == LinkFault::Reconnect)
				++generation;
			// Reported after the attempt whether or not it succeeded, exactly as
			// IPCClient::ConnectionGeneration() is read in production.
			result.connectionGeneration = generation;
			if (fault == LinkFault::Throw)
			{
				result.error = "scripted pipe failure";
				return result;
			}
			result.completed = true;
			result.response = protocol::Response(
				fault == LinkFault::Refuse ? protocol::ResponseInvalid :
				request.type == protocol::RequestHandshake ? protocol::ResponseHandshake :
				protocol::ResponseSuccess);
			return result;
		};
	}
};

// One span of recorded traffic, summarized at the level the sequencing rules
// are written in. Deliberately not an exact wire transcript: the neutralization
// pass retries up to three times, and a transcript assertion would make every
// scenario brittle against a retry that is allowed to happen.
struct LinkSpan
{
	size_t requests = 0;
	int handshakes = 0;
	int fieldEnables = 0;
	int fieldDisables = 0;
	std::vector<uint32_t> enables;
	std::vector<uint32_t> disables;
};

LinkSpan SpanOf(const FakeDriverLink &link, size_t from)
{
	LinkSpan span;
	for (size_t i = from; i < link.sent.size(); ++i)
	{
		const protocol::Request &request = link.sent[i];
		++span.requests;
		if (request.type == protocol::RequestHandshake)
		{
			++span.handshakes;
		}
		else if (request.type == protocol::RequestSetRuntimeState)
		{
			if (request.setRuntimeState.field.enabled)
				++span.fieldEnables;
			else
				++span.fieldDisables;
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				if ((request.setRuntimeState.enabledMask >> id) & 1)
					span.enables.push_back(id);
				else
					span.disables.push_back(id);
			}
		}
		else if (request.type == protocol::RequestSetDeviceTransform)
		{
			if (request.setDeviceTransform.enabled)
				span.enables.push_back(request.setDeviceTransform.openVRID);
			else
				span.disables.push_back(request.setDeviceTransform.openVRID);
		}
	}
	return span;
}

// A session wired to a scripted pipe and a device table, with the error banner
// reduced to counters. Non-copyable: the transport closes over `link`'s address.
struct SessionFixture
{
	FakeDriverLink link;
	SyncDeviceTable table;
	questcal::DriverSession session;
	int errors = 0;
	int clears = 0;

	SessionFixture()
	{
		session.SetTransport(link.Transport());
		session.SetDeviceEnumerator(
			[this](uint32_t id, const questcal::DriverSyncDesired &)
			{
				return id < vr::k_unMaxTrackedDeviceCount
					? table.devices[id] : questcal::SyncDevice();
			});
		session.SetErrorSink([this](const std::string &) { ++errors; },
			[this]() { ++clears; });
	}

	SessionFixture(const SessionFixture &) = delete;
	SessionFixture &operator=(const SessionFixture &) = delete;
};

// One reconciliation scan in production's order: Begin -- the handshake that
// stamps the batch -- and then Apply.
questcal::DriverApplyResult RunSessionScan(questcal::DriverSession &session,
	const questcal::DriverApplyRequest &request, double now)
{
	return session.Apply(request, now);
}

questcal::DriverApplyRequest MakeSessionRequest(bool enabled, bool fieldWanted)
{
	questcal::DriverApplyRequest request;
	request.enabled = enabled;
	request.desired = MakeDriverSyncDesired();

	protocol::SetAlignmentField &field = request.field;
	// The caller's half of the field predicate only: the profile is live and
	// there is something to blend. Non-default generation and anchor values so a
	// message that lost the caller's payload is visible on the wire.
	field.enabled = fieldWanted ? 1 : 0;
	field.generation = 7;
	field.sigmaMeters = questcal::FieldBlendSigmaMeters;
	if (fieldWanted)
	{
		field.anchorCount = 2;
		field.anchors[0].position[0] = 1.25;
		field.anchors[1].translationDelta[2] = -0.03;
	}
	return request;
}

void RunDriverSessionScenarios()
{
	// 1. A batch that fails partway is never left half-applied. The whole
	// connection is neutralized before the scan returns, so the next one starts
	// from a driver that is known neutral instead of an unknown mixture.
	// Deleting that recovery -- or narrowing it to the one slot that failed --
	// leaves every transform that DID land live on a driver the overlay is
	// simultaneously reporting as off.
	{
		SessionFixture fx;
		fx.table.Place(0, questcal::SyncDeviceClass::Hmd, "lighthouse");
		for (uint32_t id = 1; id <= 4; ++id)
			fx.table.Place(id, questcal::SyncDeviceClass::Other, "oculus");

		questcal::DriverApplyRequest request = MakeSessionRequest(true, true);
		RunSessionScan(fx.session, request, 0.0);

		size_t from = fx.link.sent.size();
		fx.link.script = [](size_t, const protocol::Request &request)
		{
			return request.type == protocol::RequestSetRuntimeState
				? LinkFault::Refuse : LinkFault::None;
		};
		questcal::DriverApplyResult failed = RunSessionScan(fx.session, request, 1.0);
		LinkSpan span = SpanOf(fx.link, from);

		bool stopped = span.requests == 2 && span.handshakes == 1 &&
			span.enables == std::vector<uint32_t>({ 1, 2, 3, 4 }) &&
			!failed.synchronized && !failed.enabled;

		// The next scan sends one canonical disabled state after its handshake.
		fx.link.script = nullptr;
		size_t settledFrom = fx.link.sent.size();
		RunSessionScan(fx.session, MakeSessionRequest(false, true), 2.0);
		LinkSpan settled = SpanOf(fx.link, settledFrom);
		bool quiet = settled.requests == 2 && settled.handshakes == 1 &&
			settled.fieldDisables == 1;

		char detail[96];
		snprintf(detail, sizeof detail, "%d slots reset, next scan %d requests",
			static_cast<int>(span.disables.size()),
			static_cast<int>(settled.requests));
		Check("driver session: complete-state failure", stopped && quiet,
			detail);
	}

	// 2. The spatial field is enabled only on top of a COMPLETE base-transform
	// batch on this same connection, and only while the profile is still live.
	// Hoisting the send out of that guard is the obvious "always re-assert the
	// field" simplification, and it leaves a driver holding a stale or partial
	// base set blending anchor deltas on top of it.
	{
		SessionFixture fx;
		fx.table.Place(0, questcal::SyncDeviceClass::Hmd, "lighthouse");
		for (uint32_t id = 1; id <= 3; ++id)
			fx.table.Place(id, questcal::SyncDeviceClass::Other, "oculus");
		questcal::DriverApplyRequest request = MakeSessionRequest(true, true);

		size_t from = fx.link.sent.size();
		RunSessionScan(fx.session, request, 0.0);
		LinkSpan clean = SpanOf(fx.link, from);
		protocol::Request last = fx.link.sent.back();
		// Shipped last, after every base transform, with the caller's payload
		// intact -- a session that rebuilt the message would lose these.
		bool shipped = clean.fieldEnables == 1 &&
			last.type == protocol::RequestSetRuntimeState &&
			last.setRuntimeState.field.enabled == 1 &&
			last.setRuntimeState.field.anchorCount == 2 &&
			last.setRuntimeState.field.generation == 7 &&
			std::abs(last.setRuntimeState.field.anchors[0].position[0] - 1.25) < 1e-12;

		// The transform mask and field are one request. A refusal applies neither.
		from = fx.link.sent.size();
		fx.link.script = [](size_t, const protocol::Request &request)
		{
			return request.type == protocol::RequestSetRuntimeState
				? LinkFault::Refuse : LinkFault::None;
		};
		questcal::DriverApplyResult refused = RunSessionScan(fx.session, request, 1.0);
		LinkSpan broken = SpanOf(fx.link, from);
		bool withheld = broken.requests == 2 && !refused.synchronized;

		// A perfectly healthy connection with the profile off never enables it
		// either, however much the caller has to blend.
		fx.link.script = nullptr;
		from = fx.link.sent.size();
		RunSessionScan(fx.session, MakeSessionRequest(false, true), 2.0);
		LinkSpan off = SpanOf(fx.link, from);
		bool clearedWithProfile = off.fieldEnables == 0 && off.fieldDisables == 1;

		Check("driver session: field after base batch",
			shipped && withheld && clearedWithProfile,
			"enabled only after a complete base batch on a live profile");
	}

	// 3. What the caller may believe afterwards. A partial batch clears BOTH
	// device masks, the resolved tracker id and `enabled` -- including for the
	// slots that did land -- because the jump, drift and continuous monitors all
	// steer against those identities and the driver may no longer be applying
	// them. The two causes stay distinct as well: the UI sends the user to check
	// SteamVR for one and their headset for the other.
	{
		SessionFixture fx;
		fx.table.Place(0, questcal::SyncDeviceClass::Hmd, "lighthouse");
		fx.table.Place(1, questcal::SyncDeviceClass::Other, "lighthouse");
		fx.table.Place(2, questcal::SyncDeviceClass::Other, "oculus", "T-MOUNT");
		fx.table.Place(3, questcal::SyncDeviceClass::Other, "oculus", "T-FOOT");

		questcal::DriverApplyRequest request = MakeSessionRequest(true, true);
		request.desired.continuousTrackerSerial = "T-MOUNT";

		questcal::DriverApplyResult live = RunSessionScan(fx.session, request, 0.0);
		bool derived = live.synchronized && live.enabled &&
			live.cause == questcal::DriverDisableCause::None &&
			live.referenceDeviceMask[0] && live.referenceDeviceMask[1] &&
			live.targetDeviceMask[2] && live.targetDeviceMask[3] &&
			live.continuousTrackerId == 2;

		// Slot 3's enable is refused, after slot 2's landed and resolved the
		// tracker -- so there is real derived state to throw away.
		fx.link.script = [](size_t, const protocol::Request &request)
		{
			return request.type == protocol::RequestSetRuntimeState
				? LinkFault::Refuse : LinkFault::None;
		};
		questcal::DriverApplyResult partial = RunSessionScan(fx.session, request, 1.0);
		bool closed = !partial.synchronized && !partial.enabled &&
			partial.cause == questcal::DriverDisableCause::DriverUnreachable &&
			partial.continuousTrackerId == vr::k_unTrackedDeviceIndexInvalid;
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount && closed; ++id)
			closed = !partial.referenceDeviceMask[id] && !partial.targetDeviceMask[id];

		// A foreign headset on a healthy pipe is a different verdict entirely:
		// the batch completes, and the cause names the headset.
		fx.link.script = nullptr;
		fx.table.Place(0, questcal::SyncDeviceClass::Hmd, "oculus");
		questcal::DriverApplyResult foreign = RunSessionScan(fx.session, request, 2.0);
		bool distinct = foreign.synchronized && !foreign.enabled &&
			foreign.cause == questcal::DriverDisableCause::HmdMismatch;

		Check("driver session: fail-closed result", derived && closed && distinct,
			"a partial batch clears masks/tracker; a foreign HMD reports its own cause");
	}

	// 4. A connection the overlay has not converged yet is neutralized in full
	// before any desired state goes near it: a restarted vrserver may have fresh
	// slots, a new pipe to the same provider may have retained them, and nothing
	// on the wire tells the two apart. Equally, it happens ONCE -- the pass is 66
	// blocking round-trips on the UI thread, and paying it on every 1 Hz scan is
	// a visibly stuttering overlay.
	{
		SessionFixture fx;
		fx.table.Place(0, questcal::SyncDeviceClass::Hmd, "lighthouse");
		fx.table.Place(1, questcal::SyncDeviceClass::Other, "oculus");
		questcal::DriverApplyRequest request = MakeSessionRequest(true, false);

		size_t from = fx.link.sent.size();
		RunSessionScan(fx.session, request, 0.0);
		LinkSpan first = SpanOf(fx.link, from);
		bool neutralizedFirst = first.handshakes == 1 && first.requests == 2 &&
			first.enables.size() == 1 && first.enables[0] == 1;

		from = fx.link.sent.size();
		RunSessionScan(fx.session, request, 1.0);
		LinkSpan second = SpanOf(fx.link, from);
		bool steadyState = second.requests == 2 && second.handshakes == 1 &&
			second.enables.size() == 1;

		// A vrserver restart while idle is a new generation, and that pays for
		// the reset pass again.
		++fx.link.generation;
		from = fx.link.sent.size();
		RunSessionScan(fx.session, request, 2.0);
		LinkSpan restarted = SpanOf(fx.link, from);
		bool reNeutralized = restarted.handshakes == 1 &&
			restarted.requests == 2 && restarted.enables.size() == 1;

		char detail[96];
		snprintf(detail, sizeof detail, "first %d, steady %d, after restart %d requests",
			static_cast<int>(first.requests), static_cast<int>(second.requests),
			static_cast<int>(restarted.requests));
		Check("driver session: connection neutralize",
			neutralizedFirst && steadyState && reNeutralized, detail);
	}

	// 5. A reconnect mid-batch. IPCClient replays the request on the fresh pipe
	// and reports success, so the only evidence is the connection generation --
	// and the enable it just replayed is now live on a connection this batch
	// never neutralized. That one slot is retired immediately, on the new pipe,
	// before the generation-wide pass reaches the rest. Drop the recovery and
	// the slot stays enabled while the overlay reports the profile off.
	{
		SessionFixture fx;
		fx.table.Place(0, questcal::SyncDeviceClass::Hmd, "lighthouse");
		for (uint32_t id = 1; id <= 3; ++id)
			fx.table.Place(id, questcal::SyncDeviceClass::Other, "oculus");
		questcal::DriverApplyRequest request = MakeSessionRequest(true, true);
		RunSessionScan(fx.session, request, 0.0);

		size_t from = fx.link.sent.size();
		fx.link.script = [](size_t, const protocol::Request &request)
		{
			return request.type == protocol::RequestSetRuntimeState
				? LinkFault::Reconnect : LinkFault::None;
		};
		questcal::DriverApplyResult result = RunSessionScan(fx.session, request, 1.0);

		LinkSpan span = SpanOf(fx.link, from);
		// The replay may have applied state, but it applied the complete state;
		// only the overlay's confidence is withdrawn until the next scan.
		bool stopped = span.requests == 2 && span.enables.size() == 3 &&
			!result.synchronized && !result.enabled;

		// The new generation was neutralized and recorded, so the next scan is a
		// steady-state one rather than another 66-round-trip reset.
		fx.link.script = nullptr;
		size_t settledFrom = fx.link.sent.size();
		RunSessionScan(fx.session, request, 2.0);
		LinkSpan settled = SpanOf(fx.link, settledFrom);
		bool recorded = settled.handshakes == 1 && settled.requests == 2 &&
			settled.enables.size() == 3;

		Check("driver session: mid-batch reconnect",
			stopped && recorded,
			"a reconnect can only replay one complete state and the next scan reconfirms it");
	}

	// 6. One driver failure is one banner. A dead pipe fails every request in a
	// scan and every scan after it, so without the 30 s debounce the user's error
	// line is rewritten dozens of times a second; without the re-arm on a
	// recovered batch, the next genuine failure is swallowed for up to 30 s.
	{
		SessionFixture fx;
		fx.table.Place(0, questcal::SyncDeviceClass::Hmd, "lighthouse");
		fx.table.Place(1, questcal::SyncDeviceClass::Other, "oculus");
		questcal::DriverApplyRequest request = MakeSessionRequest(true, true);

		fx.link.script = [](size_t, const protocol::Request &) { return LinkFault::Throw; };
		questcal::DriverApplyResult dead = RunSessionScan(fx.session, request, 100.0);
		bool reportedOnce = fx.errors == 1 && !dead.synchronized && !dead.enabled &&
			dead.cause == questcal::DriverDisableCause::DriverUnreachable;
		// A handshake that never answered leaves nothing to roll back, so the scan
		// does not spend a neutralization pass on a pipe that is not there.
		bool cheapWhenDead = fx.link.sent.size() == 1;

		RunSessionScan(fx.session, request, 110.0);
		bool debounced = fx.errors == 1;
		RunSessionScan(fx.session, request, 131.0);
		bool reportedAgain = fx.errors == 2;

		// A recovered batch withdraws the banner and re-arms the clock...
		fx.link.script = nullptr;
		RunSessionScan(fx.session, request, 132.0);
		bool withdrawn = fx.errors == 2 && fx.clears == 1;

		// ...so the next failure is reported immediately, not 30 s later.
		fx.link.script = [](size_t, const protocol::Request &) { return LinkFault::Throw; };
		RunSessionScan(fx.session, request, 133.0);
		bool reArmed = fx.errors == 3;

		char detail[80];
		snprintf(detail, sizeof detail, "%d error(s), %d withdrawal(s)",
			fx.errors, fx.clears);
		Check("driver session: error debounce",
			reportedOnce && cheapWhenDead && debounced && reportedAgain &&
				withdrawn && reArmed, detail);
	}
}

void RunDriverWorkerScenario()
{
	std::atomic<int> requests{ 0 };
	std::atomic<int> stateRequests{ 0 };
	std::atomic<int> disableRequests{ 0 };
	questcal::DriverWorker worker;
	worker.Start([&](const protocol::Request &request)
	{
		++requests;
		if (request.type == protocol::RequestSetRuntimeState)
			++stateRequests;
		else if (request.type == protocol::RequestSetDeviceTransform)
			++disableRequests;
		questcal::DriverTransportResult result;
		result.completed = true;
		result.connectionGeneration = 1;
		result.response = protocol::Response(request.type == protocol::RequestHandshake
			? protocol::ResponseHandshake : protocol::ResponseSuccess);
		result.response.poseHookMask = protocol::PoseHook006;
		return result;
	});

	questcal::DriverStateJob job;
	job.request.enabled = false;
	auto first = worker.Submit(job);
	auto duplicate = worker.Submit(job);
	job.request.desired.scale = 1.01;
	auto changed = worker.Submit(job);
	job.request.field.generation = 1;
	auto fieldChanged = worker.Submit(job);
	job.devices[4].id = 4;
	job.devices[4].deviceClass = questcal::SyncDeviceClass::Other;
	auto deviceChanged = worker.Submit(job);

	questcal::DriverCompletion completion;
	bool gotLatest = false;
	for (int attempt = 0; attempt < 1000 && !gotLatest; ++attempt)
	{
		if (worker.Poll(completion) && completion.sequence == deviceChanged.sequence)
			gotLatest = true;
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	const uint64_t neutralization = worker.Neutralize({ 7, 9 }, 1.0);
	job.request.desired.scale = 1.02;
	auto heldState = worker.Submit(job);
	bool gotNeutralization = false;
	for (int attempt = 0; attempt < 1000 && !gotNeutralization; ++attempt)
	{
		if (worker.Poll(completion) &&
			completion.kind == questcal::DriverWorkKind::Neutralize &&
			completion.sequence == neutralization)
		{
			gotNeutralization = completion.succeeded;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
	const int stateCountWhileHeld = stateRequests.load();
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	const bool stateStayedHeld = stateRequests.load() == stateCountWhileHeld;

	worker.ReleaseNeutralization();
	bool gotReleasedState = false;
	for (int attempt = 0; attempt < 1000 && !gotReleasedState; ++attempt)
	{
		if (worker.Poll(completion) &&
			completion.kind == questcal::DriverWorkKind::Synchronize &&
			completion.sequence == heldState.sequence)
		{
			gotReleasedState = completion.result.synchronized;
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
	worker.Stop();

	Check("driver worker: coalesced and serialized background work",
		first.stateChanged && !duplicate.stateChanged && changed.stateChanged &&
		fieldChanged.stateChanged && deviceChanged.stateChanged &&
		gotLatest && completion.result.synchronized &&
		completion.result.poseHookMask == protocol::PoseHook006 && requests >= 2 &&
		gotNeutralization && disableRequests == 2 && stateStayedHeld &&
		heldState.stateChanged && gotReleasedState,
		"same states coalesce; pair neutralization holds ordinary state until release");
}

// ---------------------------------------------------------------------------
// The synchronous half of the asynchronous driver sync: the slot identities the
// submitting thread derives (Overlay/DriverSession.h DeriveDriverSlotState) and
// the refusal bookkeeping it keeps across round trips
// (Overlay/DriverSyncTracker.h).

void RunDriverSyncStateScenarios()
{
	// The identities derived at submission are the ones the session ships on
	// the wire and reports back, so nothing about a sync in flight is unknown
	// to the monitors: a live profile with an armed, hidden mounted tracker,
	// then a foreign headset that withdraws the profile.
	{
		SessionFixture fx;
		fx.table.Place(0, questcal::SyncDeviceClass::Hmd, "lighthouse");
		fx.table.Place(1, questcal::SyncDeviceClass::Other, "lighthouse");
		fx.table.Place(2, questcal::SyncDeviceClass::Other, "oculus", "MOUNT-1");
		fx.table.Place(3, questcal::SyncDeviceClass::Other, "oculus", "FOOT-1");
		questcal::DriverApplyRequest request = MakeSessionRequest(true, false);
		request.desired.continuousTrackerSerial = "MOUNT-1";
		request.desired.continuousArmed = true;
		request.desired.hideMountedTracker = true;

		auto enumerate = [&fx](uint32_t id, const questcal::DriverSyncDesired &)
		{
			return fx.table.devices[id];
		};
		auto sameIdentities = [](const questcal::DriverSlotState &derived,
			const questcal::DriverApplyResult &applied)
		{
			if (derived.continuousTrackerId != applied.continuousTrackerId)
				return false;
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				if (derived.referenceDeviceMask[id] != applied.referenceDeviceMask[id] ||
					derived.targetDeviceMask[id] != applied.targetDeviceMask[id])
					return false;
			}
			return true;
		};
		auto lastWire = [&fx]() -> const protocol::SetRuntimeState &
		{
			return fx.link.sent.back().setRuntimeState;
		};

		questcal::DriverSlotState derived =
			questcal::DeriveDriverSlotState(request.desired, enumerate);
		questcal::DriverApplyResult applied = RunSessionScan(fx.session, request, 0.0);
		const bool liveShipped = fx.link.sent.back().type == protocol::RequestSetRuntimeState &&
			applied.enabled && applied.synchronized && !derived.hmdMismatch;
		const bool liveParity = sameIdentities(derived, applied) &&
			derived.enabledMask == lastWire().enabledMask &&
			derived.hiddenMask == lastWire().hiddenMask;
		const bool liveSlots =
			derived.enabledMask == ((uint64_t{ 1 } << 2) | (uint64_t{ 1 } << 3)) &&
			derived.hiddenMask == (uint64_t{ 1 } << 2) &&
			derived.continuousTrackerId == 2 &&
			derived.referenceDeviceMask[1] && derived.targetDeviceMask[3];

		fx.table.Place(0, questcal::SyncDeviceClass::Hmd, "oculus");
		derived = questcal::DeriveDriverSlotState(request.desired, enumerate);
		applied = RunSessionScan(fx.session, request, 1.0);
		const bool foreign = derived.hmdMismatch && !applied.enabled &&
			applied.cause == questcal::DriverDisableCause::HmdMismatch &&
			applied.synchronized &&
			fx.link.sent.back().type == protocol::RequestSetRuntimeState &&
			lastWire().enabledMask == 0 && lastWire().hiddenMask == 0;

		const std::string detail = "liveShipped=" + std::to_string(liveShipped) +
			" liveParity=" + std::to_string(liveParity) +
			" liveSlots=" + std::to_string(liveSlots) +
			" foreign=" + std::to_string(foreign) +
			" enabledMask=" + std::to_string(derived.enabledMask) +
			" trackerId=" + std::to_string(derived.continuousTrackerId);
		Check("driver sync state: submit-time identities match the session",
			liveShipped && liveParity && liveSlots && foreign, detail.c_str());
	}

	// A first submission is applied optimistically. The refusal that answers it
	// holds an identical resubmission, survives a stale verdict for an older
	// sequence, lifts for a changed state (a question the driver has not
	// answered), and only returns once that changed state is itself refused.
	{
		questcal::DriverSyncTracker tracker;
		const bool fresh = !tracker.NoteSubmission(1, true) && tracker.IsLatest(1) &&
			!tracker.HoldsRefusal();
		const bool refused = tracker.NoteVerdict(1, false) && tracker.HoldsRefusal();
		const bool heldSame = tracker.NoteSubmission(2, false) && tracker.IsLatest(2);
		const bool staleIgnored = !tracker.NoteVerdict(1, true) && tracker.HoldsRefusal();
		const bool liftedByChange = !tracker.NoteSubmission(3, true);
		const bool pendingSame = !tracker.NoteSubmission(4, false);
		const bool confirmed = tracker.NoteVerdict(4, true) && !tracker.HoldsRefusal() &&
			!tracker.NoteSubmission(5, false);
		const bool refusedAgain = tracker.NoteVerdict(5, false) &&
			tracker.NoteSubmission(6, false);
		const std::string detail = "fresh=" + std::to_string(fresh) +
			" refused=" + std::to_string(refused) +
			" heldSame=" + std::to_string(heldSame) +
			" staleIgnored=" + std::to_string(staleIgnored) +
			" liftedByChange=" + std::to_string(liftedByChange) +
			" pendingSame=" + std::to_string(pendingSame) +
			" confirmed=" + std::to_string(confirmed) +
			" refusedAgain=" + std::to_string(refusedAgain);
		Check("driver sync state: a refusal holds across identical resubmissions",
			fresh && refused && heldSame && staleIgnored && liftedByChange &&
				pendingSame && confirmed && refusedAgain, detail.c_str());
	}

	// The periodic one-second scan may submit an identical retry while a pipe
	// request is still inside its two-second timeout. The older completion still
	// answers the current desired state and must be accepted; otherwise every
	// slow completion can be starved forever by the next retry.
	{
		questcal::DriverSyncTracker tracker;
		const bool firstOpen = !tracker.NoteSubmission(1, true);
		const bool retryOpen = !tracker.NoteSubmission(2, false);
		const bool slowRefusalAccepted = tracker.NoteVerdict(1, false) &&
			tracker.HoldsRefusal();
		const bool heldRetry = tracker.NoteSubmission(3, false);
		const bool recoveryAccepted = tracker.NoteVerdict(2, true) &&
			!tracker.HoldsRefusal();
		const std::string detail = "firstOpen=" + std::to_string(firstOpen) +
			" retryOpen=" + std::to_string(retryOpen) +
			" slowRefusalAccepted=" + std::to_string(slowRefusalAccepted) +
			" heldRetry=" + std::to_string(heldRetry) +
			" recoveryAccepted=" + std::to_string(recoveryAccepted);
		Check("driver sync state: equivalent slow completions remain applicable",
			firstOpen && retryOpen && slowRefusalAccepted && heldRetry &&
				recoveryAccepted, detail.c_str());
	}

	// Liveness of that hold: an unchanged resubmission is still dispatched to
	// the driver and completes under its own sequence, so a refused profile is
	// re-asked every scan and comes back the moment the driver accepts it.
	{
		std::atomic<bool> refuse{ true };
		questcal::DriverWorker worker;
		worker.Start([&refuse](const protocol::Request &request)
		{
			questcal::DriverTransportResult result;
			result.completed = true;
			result.connectionGeneration = 1;
			result.response = protocol::Response(
				request.type == protocol::RequestHandshake ? protocol::ResponseHandshake :
				refuse ? protocol::ResponseInvalid : protocol::ResponseSuccess);
			return result;
		});
		auto await = [&worker](uint64_t sequence, questcal::DriverCompletion &out)
		{
			for (int i = 0; i < 400; ++i)
			{
				if (worker.Poll(out) && out.sequence == sequence)
					return true;
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
			return false;
		};

		questcal::DriverSyncTracker tracker;
		questcal::DriverStateJob job;
		job.request.enabled = true;
		job.request.desired = MakeDriverSyncDesired();
		questcal::DriverCompletion completion;

		const auto first = worker.Submit(job);
		const bool firstOpen = !tracker.NoteSubmission(first.sequence, first.stateChanged);
		const bool firstRefused = await(first.sequence, completion) &&
			!completion.result.synchronized &&
			tracker.NoteVerdict(completion.sequence, completion.result.synchronized) &&
			tracker.HoldsRefusal();

		refuse = false;
		const auto again = worker.Submit(job);
		const bool held = !again.stateChanged &&
			tracker.NoteSubmission(again.sequence, again.stateChanged);
		const bool lifted = await(again.sequence, completion) &&
			completion.kind == questcal::DriverWorkKind::Synchronize &&
			completion.result.synchronized &&
			tracker.NoteVerdict(completion.sequence, completion.result.synchronized) &&
			!tracker.HoldsRefusal();
		worker.Stop();

		const std::string detail = "firstOpen=" + std::to_string(firstOpen) +
			" firstRefused=" + std::to_string(firstRefused) +
			" held=" + std::to_string(held) + " lifted=" + std::to_string(lifted);
		Check("driver sync state: an unchanged resubmission lifts the hold when accepted",
			firstOpen && firstRefused && held && lifted, detail.c_str());
	}
}

void RunPoseSampleScenarios()
{
	Check("diagnostics: complete exported file", DiagnosticsExportScenario(),
		"build hash, driver status, scale confidence, mount and raw poses reach the on-disk report");
	Check("pose stream diagnostics: passive snapshot", PoseStreamDiagnosticsScenario(),
		"all devices and loss markers are retained without consuming another reader's samples");
	Check("continuous diagnostics: input and export", ContinuousInputDiagnosticsScenario(),
		"tracking/numeric failures and stale captures remain distinguishable after profile reset");
	Check("continuous diagnostics: reset starvation", ContinuousWindowDiagnosticsScenario(),
		"gap evidence survives window resets; clean input still recovers normally");
	Check("continuous diagnostics: pairing gates", ContinuousPairingDiagnosticsScenario(),
		"waiting retains targets; out-of-order and speed rejection have separate counters");
	Check("calibration context: profile reset boundary",
		CalibrationContextResetScenario(),
		"profile-derived state resets as one value; preferences and chaperone survive");
	Check("controller input: trigger axis metadata",
		ControllerTriggerAxisScenario(),
		"trigger slots and threshold are respected; joystick motion and unknown types cannot confirm");
	Check("calibration context: background continuous cadence",
		CalibrationContextCadenceScenario(),
		"legacy sampling and pending confirmation wake at 20 Hz; inactive loops keep the idle interval");
	Check("calibration context: queued correction basis",
		CalibrationContextCorrectionBasisScenario(),
		"replacing the transform discards pending deltas without changing snap/slew generation semantics");
	{
		CalibrationRun run;
		run.referenceId = 1;
		run.targetId = 2;
		const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
		bool accepted = run.AcceptUniverse(1, identity, Eigen::Vector3d::Zero()) &&
			run.AcceptUniverse(2, identity, Eigen::Vector3d(1.0, 0.0, 0.0)) &&
			run.AcceptUniverse(1, identity, Eigen::Vector3d::Zero()) &&
			!run.AcceptUniverse(2, identity, Eigen::Vector3d(1.01, 0.0, 0.0));
		run.referenceSamples.reserve(8);
		run.referenceSamples.push_back(PoseSample());
		run.neutralizationSequence = 7;
		run.Reset();
		Check("calibration run: universe continuity and complete reset",
			accepted && run.referenceId == UINT32_MAX &&
			!run.referenceUniverse.valid && !run.targetUniverse.valid &&
			run.neutralizationSequence == 0 && run.referenceSamples.empty(),
			"stable epochs accepted, changed epoch rejected, reset restores run defaults");
	}
	Check("profile identity: physical HMD ownership",
		ProfileHmdIdentityMatches("quest-pro-A", "quest-pro-A") &&
		!ProfileHmdIdentityMatches("quest-pro-A", "quest-pro-B") &&
		!ProfileHmdIdentityMatches("", "quest-pro-A"),
		"only the persisted non-empty physical serial matches");

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
	char detail[256];
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
	// Nothing below names the hub's chunk size. The overwrite is sized from what
	// the first chunk actually copied (twice it, so the consumer's cursor is
	// certain to fall behind the retained window), and every position is asserted
	// relative to that observed prefix -- so this scenario tracks any chunk size
	// instead of failing when a private tuning constant moves. `overflowInjected`
	// stays asserted: if the chunk size ever exceeded the whole backlog the hook
	// would never fire, and that must fail loudly rather than pass vacuously.
	//
	// The overwrite also runs on a SEPARATE thread, parked before the drain and
	// released from inside the chunk hook. Driving it from the draining thread
	// demonstrated nothing about concurrency and could not observe the property
	// the chunking exists for: that Drain RELEASES the producer mutex between
	// chunks so a producer can publish. Held across the chunk boundary, the
	// injector blocks, the bounded wait expires, and the second Check fails --
	// rather than hanging the suite.
	std::mutex injectMutex;
	std::condition_variable injectSignal;
	uint64_t injectedSamples = 0;
	bool injectRequested = false;
	bool injectFinished = false;
	std::thread injector([&]()
	{
		std::unique_lock<std::mutex> lock(injectMutex);
		injectSignal.wait(lock, [&] { return injectRequested; });
		uint64_t count = injectedSamples;
		lock.unlock();

		protocol::DevicePoseSample overwrite = midDrainSample;
		for (uint64_t i = 0; i < count; ++i)
		{
			overwrite.sampleTimeQpc = midDrainBase +
				static_cast<int64_t>(PoseStreamHub::HistoryCapacity + i);
			midDrainHub.AppendSampleForTest(overwrite);
		}

		lock.lock();
		injectFinished = true;
		lock.unlock();
		injectSignal.notify_all();
	});

	bool overflowInjected = false;
	bool producerRanBetweenChunks = false;
	midDrainHub.SetDrainChunkHookForTest([&]()
	{
		if (overflowInjected)
			return;
		overflowInjected = true;
		std::unique_lock<std::mutex> lock(injectMutex);
		injectedSamples = static_cast<uint64_t>(hubOut.size()) * 2;
		injectRequested = true;
		injectSignal.notify_all();
		producerRanBetweenChunks = injectSignal.wait_for(lock,
			std::chrono::seconds(5), [&] { return injectFinished; });
	});
	uint64_t prefixDrops = midDrainHub.Drain(midDrainConsumer, hubOut);
	{
		// If the hook never fired the injector is still parked. Release it with
		// nothing to inject, so this fails on `overflowInjected` below instead of
		// hanging the suite on the join. Redundant when the hook did fire.
		std::lock_guard<std::mutex> lock(injectMutex);
		injectRequested = true;
	}
	injectSignal.notify_all();
	injector.join();
	midDrainHub.SetDrainChunkHookForTest({});

	size_t prefixCount = hubOut.size();
	bool prefixExact = prefixCount > 0 &&
		hubOut.front().sampleTimeQpc == midDrainBase &&
		hubOut.back().sampleTimeQpc ==
			midDrainBase + static_cast<int64_t>(prefixCount) - 1;
	uint64_t suffixDrops = midDrainHub.Drain(midDrainConsumer, hubOut);
	bool suffixExact = hubOut.size() == PoseStreamHub::HistoryCapacity &&
		hubOut.front().sampleTimeQpc == midDrainBase + static_cast<int64_t>(injectedSamples) &&
		hubOut.back().sampleTimeQpc == midDrainBase +
			static_cast<int64_t>(PoseStreamHub::HistoryCapacity + injectedSamples - 1);
	snprintf(detail, sizeof detail,
		"injected %d/%llu prefix %d/%zu drops %llu suffix %d/%zu drops %llu",
		overflowInjected, static_cast<unsigned long long>(injectedSamples),
		prefixExact, prefixCount,
		static_cast<unsigned long long>(prefixDrops), suffixExact, hubOut.size(),
		static_cast<unsigned long long>(suffixDrops));
	Check("pose hub: mid-drain overflow position",
		overflowInjected && prefixExact && prefixDrops == 0 && suffixExact &&
		suffixDrops == injectedSamples - prefixCount,
		detail);
	Check("pose hub: producer runs between chunks",
		producerRanBetweenChunks,
		producerRanBetweenChunks
			? "another thread appended while a drain was mid-backlog"
			: "the producer mutex was not released between copy chunks");
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
			!CalibrationEngine::InterpolateAt(stream, 1.5, 0.5, out) &&
			CalibrationEngine::InterpolateAt(stream, 1.0, 0.5, out) &&
			CalibrationEngine::InterpolateAt(stream, 2.0, 0.5, out);
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

	// The same recovery, pinned to a TENTH of the correlation step. On the
	// reported-velocity path the profile is instantaneous and the
	// correlation is an autocorrelation peaking on the true latency, which is
	// what makes a sub-step band meaningful: the correlator resamples the
	// reference profile once and slices that shared grid per lag, so a
	// whole-slot indexing error, a grid spacing that is not an integer divisor
	// of the step, or a tail artifact that moves with the lag all shift the peak
	// by far more than 0.2 ms.
	{
		GroundTruth subStep;
		subStep.rotation = Eigen::Quaterniond(
			Eigen::AngleAxisd(0.8, Eigen::Vector3d(-0.3, 0.8, 0.5).normalized()));
		subStep.translation = Eigen::Vector3d(-0.4, 0.9, 0.2);
		SceneConfig scene;
		scene.duration = 20.0;

		EngineConfig cfg;
		// Two of these are not multiples of the search step, so the parabolic
		// refinement has to supply the sub-step part rather than landing on a
		// grid lag by construction.
		const double latencies[] = { -0.037, -0.0007, 0.0, 0.0007, 0.037 };
		double worstSubStep = 0.0;
		bool subStepPass = true;
		for (size_t i = 0; i < sizeof latencies / sizeof latencies[0]; ++i)
		{
			subStep.latency = latencies[i];
			std::vector<PoseSample> refS, tgtS;
			GenerateStreams(scene, subStep, static_cast<uint32_t>(3150 + i), refS, tgtS);
			double solved = 0.0;
			bool ok = CalibrationEngine::EstimateTimeOffset(refS, tgtS, cfg, solved);
			double err = std::abs(solved - subStep.latency);
			worstSubStep = std::max(worstSubStep, err);
			subStepPass = subStepPass && ok && err < 0.1 * cfg.timeOffsetStep;
		}
		snprintf(detail, sizeof detail, "worst %.3f ms of %.2f ms allowed",
			worstSubStep * 1000.0, 100.0 * cfg.timeOffsetStep);
		Check("solver: sub-step offset recovery", subStepPass, detail);
	}

	// Dropouts are holes, not long interpolation ramps. Correlating across them
	// used to turn a true +18 ms lag into a high-scoring negative lag. The
	// estimator may recover the truth from the surviving support or abstain, but
	// it must never bless a distant answer.
	{
		GroundTruth truth;
		truth.latency = 0.018;
		SceneConfig scene;
		scene.duration = 20.0;
		scene.refRate = 90.0;
		scene.targetRate = 72.0;
		std::vector<PoseSample> ref, target;
		GenerateStreams(scene, truth, 3190, ref, target);
		ref.erase(std::remove_if(ref.begin(), ref.end(), [](const PoseSample &s)
		{
			return (s.time > 5.0 && s.time < 6.5) ||
				(s.time > 12.0 && s.time < 13.5);
		}), ref.end());

		EngineConfig cfg;
		double solved = 0.0, score = 0.0, margin = 0.0;
		bool estimated = CalibrationEngine::EstimateTimeOffset(ref, target, cfg,
			solved, &score, &margin);
		bool pass = !estimated || std::abs(solved - truth.latency) < 0.004;
		snprintf(detail, sizeof detail,
			"estimated %d lag %+.2f ms score %.3f margin %.4f",
			estimated, solved * 1000.0, score, margin);
		Check("solver: offset dropout confidence", pass, detail);
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
		// The one band widened rather than tightened by the measurement pass.
		// Measured scaleErr is 0.01315 against a 0.015 bound - a 12% margin on
		// a quantity that depends on the outlier draws. The property scenario,
		// the only place draws actually vary here, moves its worst case ~1.3x
		// above the mean across 12 seeds; 12% would not survive that, so a
		// different STL's normal_distribution could fail this on correct code.
		// 0.025 is ~1.9x the measured error, in line with the other bands.
		// rotErr and transErr sit at 0.000 deg / 20.6 mm, so those stay.
		Check("solver: robust scale outliers",
			r.valid && scaleErr < 0.025 && rotErr < 0.5 && transErr < 0.03,
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

	// One device can fill its post-window before another reports the same rebase.
	// Advancing the second device must not age the first candidate on the second
	// device's clock, and the agreement deadline includes the 200 ms fit window.
	{
		JumpDetector jd(TestQpcToSeconds);
		auto feedDevice = [&](uint32_t id, double start, double jumpAt, double end)
		{
			const int firstStep = static_cast<int>(std::ceil(start * 100.0));
			const int steps = static_cast<int>(end * 100.0);
			for (int step = firstStep; step <= steps; ++step)
			{
				double t = step / 100.0;
				Eigen::Quaterniond rot; Eigen::Vector3d pos, vel, angVel;
				RefTrajectory(t, id, rot, pos, vel, angVel);
				if (t >= jumpAt)
					ApplyUniverse(D_R, D_T, rot, pos, vel, angVel);
				jd.Push(RingSample(id, t, Eigen::Quaterniond::Identity(),
					Eigen::Vector3d::Zero(), rot, pos, vel, angVel));
			}
		};
		feedDevice(0, 0.0, 10.0, 1.49);
		feedDevice(1, 0.0, 10.0, 1.49); // keep the second device active: no solo acceptance
		feedDevice(0, 1.50, 1.50, 1.72); // ready, waiting for agreement
		feedDevice(1, 1.50, 1.70, 1.92); // agrees 200 ms later
		JumpDetector::UniverseDelta delta;
		bool accepted = jd.PollDelta(delta);
		Check("jump: device-local candidate clocks", accepted && !delta.exact &&
			delta.devicesAgreeing == 2 &&
			std::abs(delta.time - 1.5) < 0.02,
			"a delayed agreeing device survives the full fit + agreement window");
	}

	// Two discontinuities from one device can fit inside the agreement window
	// even though their post-fit windows do not overlap. They are not two votes.
	for (bool peerActive : { false, true })
	{
		JumpDetector jd(TestQpcToSeconds);
		JumpRun result;
		for (int step = 0; step <= 300; ++step)
		{
			const double t = step / 100.0;
			const int jumps = (step >= 150 ? 1 : 0) + (step >= 172 ? 1 : 0);
			for (uint32_t id = 0; id < (peerActive ? 2u : 1u); ++id)
			{
				const double yaw = id == 0 ? jumps * 6.0 * EIGEN_PI / 180.0 : 0.0;
				jd.Push(RingSample(id, t, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
					Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY())),
					Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
			}
			JumpDetector::UniverseDelta delta;
			while (jd.PollDelta(delta)) { ++result.deltas; result.last = delta; }
		}
		snprintf(detail, sizeof detail, "deltas %d, reported devices %d",
			result.deltas, result.last.devicesAgreeing);
		Check(peerActive ? "jump: one device cannot corroborate itself"
			: "jump: repeated small solo events stay unconfirmed", result.deltas == 0, detail);
	}

	// Repeated exact observations from a non-HMD device are one corroborating
	// device; the HMD's authoritative endpoint still determines the correction.
	{
		JumpDetector jd(TestQpcToSeconds);
		JumpRun result;
		for (int step = 0; step <= 250; ++step)
		{
			for (uint32_t id : { 1u, 0u })
			{
				const double shift = id == 1
					? (step >= 150 ? 0.01 : 0.0) + (step >= 155 ? 0.01 : 0.0)
					: (step >= 160 ? 0.02 : 0.0);
				jd.Push(RingSample(id, step / 100.0, Eigen::Quaterniond::Identity(),
					Eigen::Vector3d(shift, 0.0, 0.0), Eigen::Quaterniond::Identity(),
					Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
			}
			JumpDetector::UniverseDelta delta;
			while (jd.PollDelta(delta)) { ++result.deltas; result.last = delta; }
		}
		snprintf(detail, sizeof detail, "deltas %d, reported devices %d, shift %.3f m",
			result.deltas, result.last.devicesAgreeing, result.last.translation.x());
		Check("jump: exact corroboration counts unique devices", result.deltas == 1 &&
			result.last.exact && result.last.devicesAgreeing == 2 &&
			result.TransErr(Eigen::Vector3d(0.02, 0.0, 0.0)) < 1e-9, detail);
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
//   trans = sum_j w_j (R_j pos + t_j) / sum_j w_j - rot * pos
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
		tSum += c.weight * (c.rotation.toRotationMatrix() * pos + c.translation);
		weightSum += c.weight;
	}

	qSum.normalize();
	rotOut = Eigen::Quaterniond(qSum(0), qSum(1), qSum(2), qSum(3));
	transOut = tSum / weightSum - rotOut.toRotationMatrix() * pos;
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
	// corrected world pose by exactly D, for both rotation and translation.
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
		Check("field: jump invariance", worst < 1e-10 && worstRot < 1e-10, detail);
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
		std::vector<vr::HmdQuad_t> nonFinite = a;
		nonFinite[1].vCorners[0].v[2] = std::numeric_limits<float>::quiet_NaN();

		bool pass = QuadsMatch(a, a, 0.002f) && QuadsMatch(a, jitter, 0.002f) &&
			!QuadsMatch(a, moved, 0.002f) && !QuadsMatch(a, fewer, 0.002f) &&
			!QuadsMatch(a, nonFinite, 0.002f) && !QuadsMatch(a, a, -0.1f);
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

	// 18. Confirmation retains the latest correction and requires a trigger
	// release after the correction becomes pending. A trigger already held by
	// gameplay must not silently approve it.
	{
		ContinuousCorrectionGate gate;
		ContinuousAlignment::Correction first;
		first.rotation = Eigen::Quaterniond(
			Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY()));
		first.translation = Eigen::Vector3d(0.001, 0.0, 0.0);
		ContinuousAlignment::Correction latest;
		latest.rotation = Eigen::Quaterniond(
			Eigen::AngleAxisd(-0.02, Eigen::Vector3d::UnitY()));
		latest.translation = Eigen::Vector3d(0.0, 0.002, 0.0);

		ContinuousAlignment::Correction taken;
		gate.Offer(first, true);
		bool heldRejected = !gate.Take(true, true, taken) && gate.HasPending();
		bool releaseObserved = !gate.Take(true, false, taken) && gate.HasPending();
		gate.Offer(latest, false);
		bool confirmedLatest = gate.Take(true, true, taken) && !gate.HasPending() &&
			taken.rotation.angularDistance(latest.rotation) < 1e-12 &&
			(taken.translation - latest.translation).norm() < 1e-12;

		gate.Offer(first, true);
		bool bypassedWhenDisabled = gate.Take(false, true, taken) && !gate.HasPending();
		gate.Offer(first, false);
		gate.Clear();
		Check("continuous: trigger confirmation gate",
			heldRejected && releaseObserved && confirmedLatest &&
			bypassedWhenDisabled && !gate.HasPending(),
			"held trigger rejected, latest retained, opt-out bypasses, reset clears");
	}
}

// ---------------------------------------------------------------------------
// Persistence: the Config record codec, the write gates, and the load-plan
// state machine (Overlay/ProfileRecordJson.h + Overlay/ProfileValidation.h).
//
// Configuration.cpp is compiled by no test. These two headers carry every
// persistence decision that used to live inside it, so this is the only place a
// writer/parser divergence -- the mount extrinsic or the field anchors silently
// reverting on every launch -- can be caught before a user finds it.

const size_t PersistMaxAnchors = protocol::SetAlignmentField::MaxAnchors;

// Re-normalizing an already-normalized quaternion is not required to be
// bit-exact, so quaternion components get a few ulps of slack. Everything the
// codec stores verbatim is compared exactly: picojson emits %.17g, which
// round-trips an IEEE double. Components are compared one at a time -- an
// angular distance would not see a w/x transposition between writer and reader.
bool PersistQuatEq(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b)
{
	const double tol = 1e-15;
	return std::abs(a.w() - b.w()) <= tol && std::abs(a.x() - b.x()) <= tol &&
		std::abs(a.y() - b.y()) <= tol && std::abs(a.z() - b.z()) <= tol;
}

bool PersistVecEq(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
	return a(0) == b(0) && a(1) == b(1) && a(2) == b(2);
}

// Fully populated: every optional block present, every boolean flipped away
// from its default, so a field the writer emits and the parser stopped reading
// (or the reverse) shows up as a named difference rather than as a value that
// happens to match the default.
ProfileRecord PersistGoodRecord()
{
	ProfileRecord r;
	r.valid = true;
	r.referenceTrackingSystem = "lighthouse";
	r.targetTrackingSystem = "oculus";
	r.rotation = Eigen::Quaterniond(0.7, 0.2, -0.5, 0.3).normalized();
	r.translationMeters = Eigen::Vector3d(1.25, -0.375, 0.5);
	r.scale = 1.02;
	r.timeOffset = 0.012;
	r.calibrationUnixTime = 1.7e9;
	r.universeUnsafe = true;
	r.universeValid = true;
	r.universeHmdSerial = "LHR-HMD01";
	r.universeRotation = Eigen::Quaterniond(0.6, -0.3, 0.4, 0.62).normalized();
	r.universeTranslation = Eigen::Vector3d(-0.5, 1.75, 0.125);
	r.fieldEnabled = false;
	r.continuousEnabled = true;
	r.continuousTrackerSerial = "LHR-ABC";
	r.continuousLatencyReestimation = true;
	r.continuousRequireTrigger = true;
	r.hideMountedTracker = false;
	r.mountExtrinsic.valid = true;
	r.mountExtrinsic.rotation = Eigen::Quaterniond(0.9, 0.1, -0.25, 0.35).normalized();
	r.mountExtrinsic.translationMeters = Eigen::Vector3d(0.05, -0.125, 0.25);
	r.mountExtrinsic.rotationRmsDeg = 0.375;
	r.mountExtrinsic.translationRmsM = 0.0125;

	PersistedFieldAnchor a0;
	a0.position = Eigen::Vector3d(1.5, 0.0, -2.0);
	a0.rotation = Eigen::Quaterniond(0.95, 0.05, -0.15, 0.25).normalized();
	a0.translationMeters = Eigen::Vector3d(1.3, -0.4, 0.55);
	PersistedFieldAnchor a1;
	a1.position = Eigen::Vector3d(-0.75, 1.0, 0.5);
	a1.rotation = Eigen::Quaterniond(0.8, -0.2, 0.4, 0.4).normalized();
	a1.translationMeters = Eigen::Vector3d(0.9, 0.125, -0.625);
	r.fieldAnchors.push_back(a0);
	r.fieldAnchors.push_back(a1);
	return r;
}

std::string PersistWrite(const ProfileRecord &record, uint32_t revision)
{
	std::ostringstream out;
	WriteProfile(record, revision, out);
	return out.str();
}

// "" on success; otherwise the reason the envelope or the object was refused.
std::string PersistReadBack(const std::string &text, ProfileRecord &back,
	LegacyProfileSettings &legacy, ProfileParseResult &result)
{
	try
	{
		std::istringstream in(text);
		picojson::value v = ParseProfileEnvelope(in);
		result = ParseProfileObject(
			back, legacy, v.get<picojson::object>(), PersistMaxAnchors);
		return std::string();
	}
	catch (const std::exception &e)
	{
		return std::string("threw \"") + e.what() + "\"";
	}
}

// Names every field that differs, so a single Check can still say which one.
std::string PersistProfileDiff(const ProfileRecord &a, const ProfileRecord &b)
{
	std::string d;
	auto note = [&d](const char *field) { d += d.empty() ? field : (std::string("/") + field); };
	if (a.valid != b.valid) note("valid");
	if (a.referenceTrackingSystem != b.referenceTrackingSystem) note("reference");
	if (a.targetTrackingSystem != b.targetTrackingSystem) note("target");
	if (!PersistQuatEq(a.rotation, b.rotation)) note("rotation");
	if (!PersistVecEq(a.translationMeters, b.translationMeters)) note("translation");
	if (a.scale != b.scale) note("scale");
	if (a.timeOffset != b.timeOffset) note("timeOffset");
	if (a.calibrationUnixTime != b.calibrationUnixTime) note("calibrationTime");
	if (a.universeUnsafe != b.universeUnsafe) note("universeUnsafe");
	if (a.universeValid != b.universeValid) note("universeValid");
	if (a.universeHmdSerial != b.universeHmdSerial) note("universeSerial");
	if (a.universeValid && b.universeValid)
	{
		if (!PersistQuatEq(a.universeRotation, b.universeRotation)) note("universeRotation");
		if (!PersistVecEq(a.universeTranslation, b.universeTranslation)) note("universeTranslation");
	}
	if (a.fieldEnabled != b.fieldEnabled) note("fieldEnabled");
	if (a.continuousEnabled != b.continuousEnabled) note("continuousEnabled");
	if (a.continuousTrackerSerial != b.continuousTrackerSerial) note("continuousSerial");
	if (a.continuousLatencyReestimation != b.continuousLatencyReestimation) note("continuousLatency");
	if (a.continuousRequireTrigger != b.continuousRequireTrigger) note("continuousTrigger");
	if (a.hideMountedTracker != b.hideMountedTracker) note("hideMountedTracker");
	if (a.mountExtrinsic.valid != b.mountExtrinsic.valid) note("mount.valid");
	else if (a.mountExtrinsic.valid)
	{
		if (!PersistQuatEq(a.mountExtrinsic.rotation, b.mountExtrinsic.rotation)) note("mount.rotation");
		if (!PersistVecEq(a.mountExtrinsic.translationMeters,
			b.mountExtrinsic.translationMeters)) note("mount.translation");
		if (a.mountExtrinsic.rotationRmsDeg != b.mountExtrinsic.rotationRmsDeg) note("mount.rotRms");
		if (a.mountExtrinsic.translationRmsM != b.mountExtrinsic.translationRmsM) note("mount.posRms");
	}
	if (a.fieldAnchors.size() != b.fieldAnchors.size()) note("anchorCount");
	else
	{
		for (size_t i = 0; i < a.fieldAnchors.size(); ++i)
		{
			if (!PersistVecEq(a.fieldAnchors[i].position, b.fieldAnchors[i].position))
				note("anchor.position");
			if (!PersistQuatEq(a.fieldAnchors[i].rotation, b.fieldAnchors[i].rotation))
				note("anchor.rotation");
			if (!PersistVecEq(a.fieldAnchors[i].translationMeters,
				b.fieldAnchors[i].translationMeters)) note("anchor.translation");
		}
	}
	return d;
}

picojson::value PersistNumbers(std::initializer_list<double> values)
{
	picojson::array arr;
	arr.reserve(values.size());
	for (double v : values)
		arr.push_back(picojson::value(v));
	return picojson::value(arr);
}

// Exactly the keys ParseProfileObject requires, with an identity transform and
// settings_version 2 so nothing below trips the v1 scale migration by accident.
picojson::object PersistMinimalProfileObject()
{
	picojson::object obj;
	obj["reference_tracking_system"] = picojson::value(std::string("lighthouse"));
	obj["target_tracking_system"] = picojson::value(std::string("oculus"));
	obj["rotation_quat"] = PersistNumbers({ 1.0, 0.0, 0.0, 0.0 });
	obj["translation_meters"] = PersistNumbers({ 0.0, 0.0, 0.0 });
	obj["settings_version"] = picojson::value(2.0);
	return obj;
}

picojson::value PersistAnchorObject(double translationX)
{
	picojson::object anchor;
	anchor["position"] = PersistNumbers({ 0.0, 0.0, 0.0 });
	anchor["rotation_quat"] = PersistNumbers({ 1.0, 0.0, 0.0, 0.0 });
	anchor["translation_meters"] = PersistNumbers({ translationX, 0.0, 0.0 });
	return picojson::value(anchor);
}

// 0 = accepted, 1 = std::runtime_error (the contract), 2 = some other exception.
int PersistParseOutcome(const picojson::object &obj, std::string &message)
{
	ProfileRecord record;
	LegacyProfileSettings legacy;
	try
	{
		ParseProfileObject(record, legacy, obj, PersistMaxAnchors);
		return 0;
	}
	catch (const std::runtime_error &e) { message = e.what(); return 1; }
	catch (const std::exception &e) { message = e.what(); return 2; }
}

const char *PersistWriteGateName(PersistenceWriteGate gate)
{
	switch (gate)
	{
	case PersistenceWriteGate::Allowed: return "Allowed";
	case PersistenceWriteGate::SkippedPreview: return "SkippedPreview";
	case PersistenceWriteGate::RefusedConfigUnreadable: return "RefusedConfigUnreadable";
	case PersistenceWriteGate::RefusedSettingsUnreadable: return "RefusedSettingsUnreadable";
	}
	return "?";
}

const char *PersistLoadGateName(ChaperoneLoadGate gate)
{
	switch (gate)
	{
	case ChaperoneLoadGate::Armed: return "Armed";
	case ChaperoneLoadGate::ProfileUnreadable: return "ProfileUnreadable";
	case ChaperoneLoadGate::SettingsUnreadable: return "SettingsUnreadable";
	case ChaperoneLoadGate::IncompleteOwner: return "IncompleteOwner";
	case ChaperoneLoadGate::ForeignTrackingSystem: return "ForeignTrackingSystem";
	}
	return "?";
}

// --- A. WriteProfile -> ParseProfileObject identity --------------------------
void RunPersistenceRoundTripScenario()
{
	std::string why;
	auto fail = [&why](const char *cell, const std::string &what)
	{
		why += std::string(" ") + cell + "(" + what + ")";
	};

	// A1: fully populated, both optional blocks present, two anchors in range.
	const ProfileRecord good = PersistGoodRecord();
	const std::string a1Text = PersistWrite(good, 7);
	ProfileRecord back;
	LegacyProfileSettings legacy;
	ProfileParseResult result;
	std::string err = PersistReadBack(a1Text, back, legacy, result);
	size_t a1Anchors = 0;
	if (!err.empty())
	{
		fail("A1", err);
	}
	else
	{
		a1Anchors = back.fieldAnchors.size();
		std::string diff = PersistProfileDiff(good, back);
		if (!diff.empty()) fail("A1", diff);
		if (!back.valid) fail("A1", "valid=false");
		if (!result.revision.present || result.revision.value != 7)
			fail("A1", "revision");
		// The writer stamps settings_version 2, so a round-tripped record must
		// not re-run the one-time v1 scale migration on every launch.
		if (result.migratedScaleSetting) fail("A1", "migration re-ran");
	}

	// A2: an empty continuous serial is written by omission and must come back
	// empty rather than as a stray key or a parse failure.
	{
		ProfileRecord record = good;
		record.continuousTrackerSerial.clear();
		ProfileRecord r2;
		LegacyProfileSettings l2;
		ProfileParseResult p2;
		std::string e2 = PersistReadBack(PersistWrite(record, 7), r2, l2, p2);
		if (!e2.empty()) fail("A2", e2);
		else if (!r2.continuousTrackerSerial.empty()) fail("A2", "serial survived");
	}

	// A3: universeValid=false suppresses the world-from-driver baseline, while
	// physical-headset ownership remains independently persisted.
	{
		ProfileRecord record = good;
		record.universeValid = false;
		ProfileRecord r3;
		LegacyProfileSettings l3;
		ProfileParseResult p3;
		std::string e3 = PersistReadBack(PersistWrite(record, 7), r3, l3, p3);
		if (!e3.empty()) fail("A3", e3);
		else if (r3.universeValid ||
			r3.universeHmdSerial != record.universeHmdSerial)
			fail("A3", "baseline/identity coupling");
	}

	// A4: an invalid mount extrinsic must not be written at all, or continuous
	// calibration arms on garbage after the next restart.
	{
		ProfileRecord record = good;
		record.mountExtrinsic.valid = false;
		ProfileRecord r4;
		LegacyProfileSettings l4;
		ProfileParseResult p4;
		std::string e4 = PersistReadBack(PersistWrite(record, 7), r4, l4, p4);
		MountExtrinsicRecord defaults;
		if (!e4.empty()) fail("A4", e4);
		else if (r4.mountExtrinsic.valid ||
			!PersistQuatEq(r4.mountExtrinsic.rotation, defaults.rotation) ||
			!PersistVecEq(r4.mountExtrinsic.translationMeters, defaults.translationMeters) ||
			r4.mountExtrinsic.rotationRmsDeg != 0.0 ||
			r4.mountExtrinsic.translationRmsM != 0.0)
			fail("A4", "extrinsic survived");
	}

	// A5: zero anchors.
	{
		ProfileRecord record = good;
		record.fieldAnchors.clear();
		ProfileRecord r5;
		LegacyProfileSettings l5;
		ProfileParseResult p5;
		std::string e5 = PersistReadBack(PersistWrite(record, 7), r5, l5, p5);
		if (!e5.empty()) fail("A5", e5);
		else if (!r5.fieldAnchors.empty()) fail("A5", "anchors appeared");
	}

	// A6: an invalid record writes nothing, which is how ClearSavedProfile
	// blanks the value instead of storing a bogus one.
	const size_t a6Bytes = PersistWrite(ProfileRecord(), 7).size();
	if (a6Bytes != 0) fail("A6", "wrote " + std::to_string(a6Bytes) + " bytes");

	// A7: the revision is a uint32 on both sides, not a float or an int32.
	const uint32_t maxRevision = std::numeric_limits<uint32_t>::max();
	uint32_t a7Revision = 0;
	{
		ProfileRecord r7;
		LegacyProfileSettings l7;
		ProfileParseResult p7;
		std::string e7 = PersistReadBack(
			PersistWrite(good, maxRevision), r7, l7, p7);
		if (!e7.empty()) fail("A7", e7);
		else
		{
			a7Revision = p7.revision.value;
			if (!p7.revision.present || p7.revision.value != maxRevision)
				fail("A7", "revision " + std::to_string(p7.revision.value));
		}
	}

	char detail[512];
	snprintf(detail, sizeof detail,
		"A1 %zu bytes / %zu anchors, A6 %zu bytes, A7 rev %u%s%s",
		a1Text.size(), a1Anchors, a6Bytes, a7Revision,
		why.empty() ? "" : "  <-", why.c_str());
	Check("persistence A: round-trip", why.empty(), detail);
}

// --- B. ReadPersistenceRevision ---------------------------------------------
void RunPersistenceRevisionScenario()
{
	std::string why;
	auto read = [](const picojson::object &obj, PersistedRevision &out,
		std::string &message) -> int
	{
		try { out = ReadPersistenceRevision(obj); return 0; }
		catch (const std::runtime_error &e) { message = e.what(); return 1; }
		catch (const std::exception &e) { message = e.what(); return 2; }
	};

	PersistedRevision absent;
	std::string message;
	picojson::object empty;
	int outcome = read(empty, absent, message);
	if (outcome != 0) why += " absent(threw)";
	else if (absent.present || absent.value != 0)
		why += " absent(present=" + std::to_string(absent.present ? 1 : 0) + ")";

	PersistedRevision three;
	picojson::object valid;
	valid["persistence_revision"] = picojson::value(3.0);
	outcome = read(valid, three, message);
	if (outcome != 0) why += " three(threw)";
	else if (!three.present || three.value != 3)
		why += " three(" + std::to_string(three.value) + ")";

	// 0 collides with the absent sentinel, so accepting it would make a
	// revisionless record look coupled to whatever Settings carries.
	auto rejects = [&](const char *cell, const picojson::value &v)
	{
		picojson::object obj;
		obj["persistence_revision"] = v;
		PersistedRevision ignored;
		std::string msg;
		int o = read(obj, ignored, msg);
		if (o == 1) return;
		why += std::string(" ") + cell +
			(o == 0 ? "(accepted " + std::to_string(ignored.value) + ")"
				: "(wrong exception)");
	};
	rejects("zero", picojson::value(0.0));
	rejects("negative", picojson::value(-1.0));
	rejects("fractional", picojson::value(1.5));
	rejects("overflow", picojson::value(4294967296.0));
	rejects("boolean", picojson::value(true));

	char detail[256];
	snprintf(detail, sizeof detail,
		"absent {%d,%u}, 3 -> {%d,%u}, 0/-1/1.5/2^32/true rejected%s%s",
		absent.present ? 1 : 0, absent.value, three.present ? 1 : 0, three.value,
		why.empty() ? "" : "  <-", why.c_str());
	Check("persistence B: revision", why.empty(), detail);
}

// --- C. RejectExcessiveJsonNesting ------------------------------------------
void RunPersistenceNestingScenario()
{
	std::string why;
	auto throwsOn = [](const std::string &text)
	{
		try { RejectExcessiveJsonNesting(text); return false; }
		catch (const std::runtime_error &) { return true; }
	};

	const std::string realProfile = PersistWrite(PersistGoodRecord(), 7);
	if (throwsOn("[]")) why += " empty-array";
	if (throwsOn("{}")) why += " empty-object";
	if (throwsOn(realProfile)) why += " real-profile";

	// Test just over the cap: nesting deep enough to actually overflow the stack
	// would prove nothing, since the whole point is that the guard fires before
	// picojson's recursive descent runs at all.
	if (throwsOn(std::string(16, '['))) why += " depth16-rejected";
	if (!throwsOn(std::string(17, '['))) why += " depth17-accepted";

	// Brackets inside a string value are not nesting -- a tracker serial
	// containing them must not make the whole profile Unreadable.
	if (throwsOn("{\"serial\":\"" + std::string(100, '[') + "\"}"))
		why += " in-string";
	// ... and an escaped quote does not end that string, so the brackets after
	// it are still inside it.
	if (throwsOn("[\"a\\\"" + std::string(100, '[') + "\"]"))
		why += " escaped-quote";

	char detail[256];
	snprintf(detail, sizeof detail,
		"depth 16 ok / 17 throws; %zu-byte real profile ok; 100 in-string brackets ok%s%s",
		realProfile.size(), why.empty() ? "" : "  <-", why.c_str());
	Check("persistence C: json depth", why.empty(), detail);
}

// --- D. ParseProfileObject rejection matrix ---------------------------------
void RunPersistenceRejectionScenario()
{
	std::string why;
	int rejected = 0;
	auto rejects = [&](const char *cell, const picojson::object &obj,
		const char *mustMention)
	{
		std::string message;
		int outcome = PersistParseOutcome(obj, message);
		if (outcome == 1)
		{
			rejected++;
			if (mustMention && message.find(mustMention) == std::string::npos)
				why += std::string(" ") + cell + "(message=\"" + message + "\")";
			return;
		}
		why += std::string(" ") + cell +
			(outcome == 0 ? "(accepted)" : "(wrong exception: " + message + ")");
	};

	const picojson::object base = PersistMinimalProfileObject();
	// Without this the whole matrix below could be passing vacuously.
	{
		std::string message;
		if (PersistParseOutcome(base, message) != 0)
			why += " fixture(rejected: " + message + ")";
	}

	picojson::object m = base;
	m.erase("reference_tracking_system");
	rejects("missing-reference", m, nullptr);

	m = base;
	m["target_tracking_system"] = picojson::value(std::string("lighthouse"));
	rejects("same-systems", m, nullptr);

	m = base;
	m["rotation_quat"] = PersistNumbers({ 1.0, 0.0, 0.0 });
	rejects("quat-3", m, nullptr);

	// Catches "normalize first, validate later", which would produce NaNs.
	m = base;
	m["rotation_quat"] = PersistNumbers({ 0.0, 0.0, 0.0, 0.0 });
	rejects("quat-zero", m, nullptr);

	m = base; m["scale"] = picojson::value(0.1);
	rejects("scale-low", m, nullptr);
	m = base; m["scale"] = picojson::value(5.0);
	rejects("scale-high", m, nullptr);

	m = base; m["time_offset"] = picojson::value(2.0);
	rejects("time-offset", m, nullptr);

	m = base; m["calibration_time"] = picojson::value(-1.0);
	rejects("time-negative", m, nullptr);
	m = base; m["calibration_time"] = picojson::value(4e10);
	rejects("time-implausible", m, nullptr);

	m = base;
	m["universe_hmd_serial"] = picojson::value(std::string("LHR-1"));
	rejects("universe-partial", m, nullptr);

	m = base;
	m["universe_hmd_serial"] = picojson::value(std::string(""));
	m["universe_world_from_driver_rotation_quat"] = PersistNumbers({ 1.0, 0.0, 0.0, 0.0 });
	m["universe_world_from_driver_translation_meters"] = PersistNumbers({ 0.0, 0.0, 0.0 });
	rejects("universe-empty-serial", m, nullptr);

	m = base; m["settings_version"] = picojson::value(0.0);
	rejects("settings-version-0", m, "settings_version");
	m = base; m["settings_version"] = picojson::value(101.0);
	rejects("settings-version-101", m, nullptr);
	m = base; m["settings_version"] = picojson::value(1.5);
	rejects("settings-version-frac", m, nullptr);

	m = base; m["calibration_speed"] = picojson::value(3.0);
	rejects("speed-3", m, nullptr);
	m = base; m["calibration_speed"] = picojson::value(1.5);
	rejects("speed-frac", m, nullptr);

	auto mountObject = [](std::initializer_list<double> quat, double rotRms)
	{
		picojson::object extrinsic;
		picojson::array q;
		for (double v : quat)
			q.push_back(picojson::value(v));
		extrinsic["rotation_quat"] = picojson::value(q);
		extrinsic["translation_meters"] = PersistNumbers({ 0.01, 0.0, -0.02 });
		extrinsic["rot_rms_deg"] = picojson::value(rotRms);
		extrinsic["pos_rms_m"] = picojson::value(0.001);
		return picojson::value(extrinsic);
	};
	m = base; m["mount_extrinsic"] = mountObject({ 1.0, 0.0, 0.0 }, 0.5);
	rejects("mount-quat-3", m, nullptr);
	m = base; m["mount_extrinsic"] = mountObject({ 0.0, 0.0, 0.0, 0.0 }, 0.5);
	rejects("mount-quat-zero", m, nullptr);
	m = base; m["mount_extrinsic"] = mountObject({ 1.0, 0.0, 0.0, 0.0 }, -1.0);
	rejects("mount-rms-negative", m, nullptr);

	m = base;
	{
		picojson::array anchors;
		for (size_t i = 0; i <= PersistMaxAnchors; ++i)
			anchors.push_back(PersistAnchorObject(0.0));
		m["field_anchors"] = picojson::value(anchors);
	}
	rejects("anchors-9", m, nullptr);

	m = base;
	{
		picojson::array anchors;
		anchors.push_back(PersistAnchorObject(200.0));
		m["field_anchors"] = picojson::value(anchors);
	}
	rejects("anchor-200m", m, nullptr);

	// picojson's get<T>() is guarded only by assert(), compiled out in Release,
	// so without the type check this reads the wrong union member.
	m = base; m["scale"] = picojson::value(std::string("1.0"));
	rejects("scale-string", m, "scale");

	m = base;
	{
		picojson::array quat;
		quat.push_back(picojson::value(1.0));
		quat.push_back(picojson::value(std::string("0")));
		quat.push_back(picojson::value(0.0));
		quat.push_back(picojson::value(0.0));
		m["rotation_quat"] = picojson::value(quat);
	}
	rejects("quat-string-element", m, nullptr);

	char detail[256];
	snprintf(detail, sizeof detail,
		"%d/23 mutations rejected as runtime_error%s%s", rejected,
		why.empty() ? "" : "  <-", why.c_str());
	Check("persistence D: rejects", why.empty() && rejected == 23, detail);
}

// --- E. Legacy-settings extraction ------------------------------------------
void RunPersistenceLegacyScenario()
{
	std::string why;
	auto parse = [&](const char *cell, const picojson::object &obj,
		ProfileRecord &record, LegacyProfileSettings &legacy,
		ProfileParseResult &result) -> bool
	{
		try
		{
			result = ParseProfileObject(record, legacy, obj, PersistMaxAnchors);
			return true;
		}
		catch (const std::exception &e)
		{
			why += std::string(" ") + cell + "(threw \"" + e.what() + "\")";
			return false;
		}
	};

	// E1: a v1 record's solve_scale is never honoured -- doing so would
	// re-enable scale solving for every legacy user on every launch -- but the
	// already-applied scale is kept, because clearing it misaligns the space.
	{
		picojson::object obj = PersistMinimalProfileObject();
		obj["settings_version"] = picojson::value(1.0);
		obj["solve_scale"] = picojson::value(true);
		obj["scale"] = picojson::value(1.01);
		ProfileRecord record;
		LegacyProfileSettings legacy;
		ProfileParseResult result;
		if (parse("E1", obj, record, legacy, result))
		{
			if (!result.migratedScaleSetting) why += " E1(not migrated)";
			if (!legacy.hasSolveScale || legacy.solveScale) why += " E1(solveScale honoured)";
			if (record.scale != 1.01) why += " E1(scale clobbered)";
		}
	}

	// E2: the suspicious-scale boundary, inclusive on both ends.
	{
		const double scales[] = { 1.05, 1.0, 0.98, 1.02, 0.979, 1.021 };
		const bool expected[] = { true, false, false, false, true, true };
		for (int i = 0; i < 6; ++i)
		{
			picojson::object obj = PersistMinimalProfileObject();
			obj["settings_version"] = picojson::value(1.0);
			obj["scale"] = picojson::value(scales[i]);
			ProfileRecord record;
			LegacyProfileSettings legacy;
			ProfileParseResult result;
			if (!parse("E2", obj, record, legacy, result))
				continue;
			if (result.suspiciousLegacyScale != expected[i])
			{
				char cell[64];
				snprintf(cell, sizeof cell, " E2(scale %.3f -> %d)", scales[i],
					result.suspiciousLegacyScale ? 1 : 0);
				why += cell;
			}
		}
	}

	// E3: at v2 the migration must not re-run, and solve_scale is honoured.
	{
		picojson::object obj = PersistMinimalProfileObject();
		obj["solve_scale"] = picojson::value(true);
		ProfileRecord record;
		LegacyProfileSettings legacy;
		ProfileParseResult result;
		if (parse("E3", obj, record, legacy, result))
		{
			if (!legacy.hasSolveScale || !legacy.solveScale) why += " E3(solveScale dropped)";
			if (result.migratedScaleSetting) why += " E3(migration re-ran)";
		}
	}

	// E4: absent is not a default. Configuration.cpp copies only when hasX, so
	// this is what stops a profile resetting preferences it never carried.
	{
		picojson::object obj = PersistMinimalProfileObject();
		ProfileRecord record;
		LegacyProfileSettings legacy;
		ProfileParseResult result;
		if (parse("E4", obj, record, legacy, result))
		{
			if (legacy.hasApplyTimeOffset || legacy.hasSolveScale ||
				legacy.hasUiAdvanced || legacy.hasChaperoneWarningAck ||
				legacy.hasCalibrationSpeed)
				why += " E4(absent became present)";
		}
	}

	// E5: every key the walk still has to visit.
	{
		picojson::object obj = PersistMinimalProfileObject();
		obj["apply_time_offset"] = picojson::value(false);
		obj["ui_advanced"] = picojson::value(true);
		obj["chaperone_warning_ack"] = picojson::value(true);
		obj["calibration_speed"] = picojson::value(2.0);
		ProfileRecord record;
		LegacyProfileSettings legacy;
		ProfileParseResult result;
		if (parse("E5", obj, record, legacy, result))
		{
			if (!legacy.hasApplyTimeOffset || legacy.applyTimeOffset)
				why += " E5(apply_time_offset)";
			if (!legacy.hasUiAdvanced || !legacy.uiAdvanced) why += " E5(ui_advanced)";
			if (!legacy.hasChaperoneWarningAck || !legacy.chaperoneWarningAck)
				why += " E5(chaperone_warning_ack)";
			if (!legacy.hasCalibrationSpeed || legacy.calibrationSpeed !=
				static_cast<int>(PersistedCalibrationSpeed::VerySlow))
				why += " E5(calibration_speed)";
		}
	}

	char detail[384];
	snprintf(detail, sizeof detail,
		"v1 migration one-shot, suspicious scale outside [0.98,1.02], 5 legacy keys%s%s",
		why.empty() ? "" : "  <-", why.c_str());
	Check("persistence E: legacy keys", why.empty(), detail);
}

// --- F. ValidateProfileRecord, the parser/writer contract -------------------
void RunPersistenceContractScenario()
{
	std::string why;
	auto accepts = [](const ProfileRecord &record, std::string &reason)
	{
		reason.clear();
		return ValidateProfileRecord(record, PersistMaxAnchors, reason);
	};
	auto expect = [&](const char *cell, const ProfileRecord &record, bool want,
		const char *wantWhy)
	{
		std::string reason;
		bool got = accepts(record, reason);
		if (got != want)
		{
			why += std::string(" ") + cell + (got ? "(accepted)" : "(rejected: " + reason + ")");
			return;
		}
		if (!want && wantWhy && reason != wantWhy)
			why += std::string(" ") + cell + "(why=\"" + reason + "\")";
	};

	const ProfileRecord good = PersistGoodRecord();
	expect("good", good, true, nullptr);

	ProfileRecord m = good;
	m.targetTrackingSystem = m.referenceTrackingSystem;
	expect("same-systems", m, false, "the tracking systems must be non-empty and different");
	m = good; m.referenceTrackingSystem.clear();
	expect("empty-reference", m, false, "the tracking systems must be non-empty and different");

	m = good; m.rotation = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
	expect("zero-quat", m, false, "the calibration transform is invalid");

	m = good; m.scale = 0.24; expect("scale-0.24", m, false, nullptr);
	m = good; m.scale = 4.01; expect("scale-4.01", m, false, nullptr);
	m = good; m.scale = 0.25; expect("scale-0.25", m, true, nullptr);
	m = good; m.scale = 4.0;  expect("scale-4.0", m, true, nullptr);

	m = good; m.timeOffset = 1.0;       expect("offset-1.0", m, true, nullptr);
	m = good; m.timeOffset = 1.0000001; expect("offset-over", m, false, nullptr);

	// 0 means "unknown", which older records legitimately carry.
	m = good; m.calibrationUnixTime = 0.0;    expect("time-0", m, true, nullptr);
	m = good; m.calibrationUnixTime = -0.001; expect("time-negative", m, false, nullptr);

	m = good; m.universeHmdSerial.clear();
	expect("universe-no-serial", m, false, nullptr);
	m.universeValid = false;
	expect("universe-disarmed", m, true, nullptr);

	m = good; m.mountExtrinsic.rotationRmsDeg = -1.0;
	expect("mount-bad-rms", m, false, nullptr);
	m.mountExtrinsic.valid = false;
	expect("mount-disarmed", m, true, nullptr);

	m = good;
	m.fieldAnchors.assign(PersistMaxAnchors + 1, good.fieldAnchors[0]);
	expect("anchors-9", m, false, nullptr);
	m.fieldAnchors.assign(PersistMaxAnchors, good.fieldAnchors[0]);
	expect("anchors-8", m, true, nullptr);

	m = good;
	m.fieldAnchors[1].translationMeters =
		m.translationMeters + Eigen::Vector3d(200.0, 0.0, 0.0);
	expect("anchor-200m", m, false, nullptr);

	// The asymmetry pin: the anchor loop sits OUTSIDE the `valid` gate, exactly
	// as in the writer this replaces. A tidy-up moving it inside would turn the
	// second case below into a silent pass.
	ProfileRecord invalid;
	invalid.valid = false;
	invalid.rotation = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
	invalid.translationMeters = Eigen::Vector3d(1e9, -1e9, 1e9);
	invalid.scale = 99.0;
	invalid.timeOffset = 500.0;
	invalid.calibrationUnixTime = -12.0;
	expect("invalid-garbage", invalid, true, nullptr);

	ProfileRecord invalidAnchor;
	invalidAnchor.valid = false;
	invalidAnchor.fieldAnchors.push_back(PersistedFieldAnchor());
	invalidAnchor.fieldAnchors[0].translationMeters = Eigen::Vector3d(200.0, 0.0, 0.0);
	expect("invalid-anchor", invalidAnchor, false, "a field anchor is invalid");

	char detail[384];
	snprintf(detail, sizeof detail,
		"scale [%.2f,%.2f], |offset| <= %.1f, <= %zu anchors; anchor loop outside the valid gate%s%s",
		protocol::limits::MinScale, protocol::limits::MaxScale,
		protocol::limits::MaxAbsTimeOffsetSeconds, PersistMaxAnchors,
		why.empty() ? "" : "  <-", why.c_str());
	Check("persistence F: contract", why.empty(), detail);
}

// --- G. Write gates, including the preview guard ----------------------------
void RunPersistenceWriteGateScenario()
{
	std::string why;
	const RecordLoadState states[] = { RecordLoadState::Missing,
		RecordLoadState::Loaded, RecordLoadState::Unreadable };
	auto expectProfile = [&](const char *cell, bool preview, RecordLoadState config,
		RecordLoadState settings, bool pending, PersistenceWriteGate want)
	{
		PersistenceWriteGate got = GateProfileWrite(preview, config, settings, pending);
		if (got != want)
			why += std::string(" ") + cell + "(" + PersistWriteGateName(got) + ")";
	};
	auto expectSettings = [&](const char *cell, bool preview, RecordLoadState config,
		RecordLoadState settings, PersistenceWriteGate want)
	{
		PersistenceWriteGate got = GateSettingsWrite(preview, config, settings);
		if (got != want)
			why += std::string(" ") + cell + "(" + PersistWriteGateName(got) + ")";
	};

	expectProfile("G1", false, RecordLoadState::Loaded, RecordLoadState::Loaded,
		false, PersistenceWriteGate::Allowed);
	for (RecordLoadState settings : states)
		for (int pending = 0; pending < 2; ++pending)
			expectProfile("G2", false, RecordLoadState::Unreadable, settings,
				pending != 0, PersistenceWriteGate::RefusedConfigUnreadable);
	// Dropping legacySettingsMigrationPending here lets one profile save destroy
	// a legacy user's only copy of their global settings and protected room.
	expectProfile("G3", false, RecordLoadState::Loaded, RecordLoadState::Unreadable,
		true, PersistenceWriteGate::RefusedSettingsUnreadable);
	expectProfile("G4", false, RecordLoadState::Loaded, RecordLoadState::Unreadable,
		false, PersistenceWriteGate::Allowed);
	expectProfile("G5", true, RecordLoadState::Unreadable, RecordLoadState::Unreadable,
		true, PersistenceWriteGate::SkippedPreview);

	expectSettings("G6", false, RecordLoadState::Unreadable, RecordLoadState::Loaded,
		PersistenceWriteGate::Allowed);
	expectSettings("G7a", false, RecordLoadState::Unreadable, RecordLoadState::Missing,
		PersistenceWriteGate::RefusedConfigUnreadable);
	expectSettings("G7b", false, RecordLoadState::Unreadable, RecordLoadState::Unreadable,
		PersistenceWriteGate::RefusedConfigUnreadable);
	expectSettings("G8", false, RecordLoadState::Loaded, RecordLoadState::Unreadable,
		PersistenceWriteGate::RefusedSettingsUnreadable);
	expectSettings("G9", true, RecordLoadState::Loaded, RecordLoadState::Loaded,
		PersistenceWriteGate::SkippedPreview);

	// G10.1: preview is reachable only via the flag, and always via the flag. A
	// mis-set flag on a normal launch is total silent persistence loss.
	int profileCells = 0;
	int settingsCells = 0;
	for (RecordLoadState config : states)
	{
		for (RecordLoadState settings : states)
		{
			settingsCells++;
			for (int pending = 0; pending < 2; ++pending)
			{
				profileCells++;
				if (GateProfileWrite(false, config, settings, pending != 0) ==
					PersistenceWriteGate::SkippedPreview)
					why += " G10.1(profile preview without the flag)";
				if (GateProfileWrite(true, config, settings, pending != 0) !=
					PersistenceWriteGate::SkippedPreview)
					why += " G10.1(profile flag not honoured)";
			}
			if (GateSettingsWrite(false, config, settings) ==
				PersistenceWriteGate::SkippedPreview)
				why += " G10.1(settings preview without the flag)";
			if (GateSettingsWrite(true, config, settings) !=
				PersistenceWriteGate::SkippedPreview)
				why += " G10.1(settings flag not honoured)";
		}
	}

	// G10.2: the divergence asserted, not implied by a bare `return true`.
	if (GateWritesRecord(PersistenceWriteGate::SkippedPreview) ||
		!GateReportsSuccess(PersistenceWriteGate::SkippedPreview))
		why += " G10.2";

	// G10.3: no OTHER outcome may report success without writing. This fails the
	// moment anyone adds a second such outcome -- the silent-loss shape itself,
	// rather than one instance of it.
	const PersistenceWriteGate allGates[] = { PersistenceWriteGate::Allowed,
		PersistenceWriteGate::SkippedPreview,
		PersistenceWriteGate::RefusedConfigUnreadable,
		PersistenceWriteGate::RefusedSettingsUnreadable };
	for (PersistenceWriteGate gate : allGates)
	{
		if (gate == PersistenceWriteGate::SkippedPreview)
			continue;
		if (GateWritesRecord(gate) != GateReportsSuccess(gate))
			why += std::string(" G10.3(") + PersistWriteGateName(gate) + ")";
	}

	char detail[320];
	snprintf(detail, sizeof detail,
		"G1-G9 named cells; %d profile + %d settings cells preview-only-by-flag; %zu outcomes%s%s",
		profileCells, settingsCells, sizeof allGates / sizeof allGates[0],
		why.empty() ? "" : "  <-", why.c_str());
	Check("persistence G: write gates", why.empty(), detail);
}

// --- H. PlanPersistenceLoad, the load matrix --------------------------------
void RunPersistenceLoadPlanScenario()
{
	struct LoadCase
	{
		const char *cell;
		PersistenceLoadFacts facts;
		uint32_t revision;
		bool mismatch;
		bool report;
		bool disarm;
		ChaperoneLoadGate gate;
		bool rewrite;
		bool latch;
		bool latchIfRewriteFails;
	};
	const RecordLoadState Mi = RecordLoadState::Missing;
	const RecordLoadState Lo = RecordLoadState::Loaded;
	const RecordLoadState Un = RecordLoadState::Unreadable;
	const ChaperoneLoadGate GArmed = ChaperoneLoadGate::Armed;

	// facts: profile, settings, profileRevision, settingsRevision, armed,
	//        ownerComplete, ownerMatchesReference, profileValid
	const LoadCase cases[] = {
		{ "H1",  { Mi, Mi, { false, 0 }, { false, 0 }, false, false, true,  false },
			1, false, false, false, GArmed, true,  false, false },
		// !=  ->  == in the revision compare would disarm every healthy launch.
		{ "H2",  { Lo, Lo, { true,  7 }, { true,  7 }, true,  true,  true,  true  },
			7, false, false, false, GArmed, false, false, false },
		// The cell that stops a pre-rebase snapshot landing on a rebased playspace.
		{ "H3",  { Lo, Lo, { true,  7 }, { true,  6 }, true,  true,  true,  true  },
			7, true,  true,  true,  GArmed, true,  false, false },
		// No snapshot to lose: the mismatch is not worth a banner.
		{ "H4",  { Lo, Lo, { true,  7 }, { true,  6 }, false, true,  true,  true  },
			7, true,  false, false, GArmed, true,  false, false },
		// The migration latch. Dropping it lets the next profile save strip the
		// embedded settings and room out of Config with no other copy anywhere.
		{ "H5",  { Lo, Mi, { false, 0 }, { false, 0 }, false, false, true,  true  },
			1, false, false, false, GArmed, true,  true,  true  },
		{ "H6",  { Lo, Lo, { false, 0 }, { true,  5 }, false, false, true,  true  },
			5, false, false, false, GArmed, false, false, true  },
		{ "H7",  { Lo, Lo, { false, 0 }, { false, 0 }, false, false, true,  true  },
			1, false, false, false, GArmed, true,  true,  true  },
		// Rewriting here would overwrite Settings while Config -- possibly the
		// only copy of the legacy room -- is unreadable.
		{ "H8",  { Un, Mi, { false, 0 }, { false, 0 }, true,  true,  true,  false },
			1, false, false, true,  ChaperoneLoadGate::ProfileUnreadable, false, false, false },
		// CanUseRecoveredSettings is not CanMaterializeSettings.
		{ "H9",  { Un, Lo, { false, 0 }, { true,  7 }, true,  true,  true,  false },
			7, false, false, false, GArmed, false, false, false },
		{ "H10", { Lo, Un, { false, 0 }, { false, 0 }, true,  true,  true,  true  },
			1, false, false, true,  ChaperoneLoadGate::SettingsUnreadable, false, true, true },
		// Same inputs but with a profile revision: the revision branch disarms
		// first, so the settings-unreadable gate no longer fires. Same ordering
		// H14 pins; recorded here because the spec left the revision unstated.
		{ "H10b",{ Lo, Un, { true,  7 }, { false, 0 }, true,  true,  true,  true  },
			7, true,  true,  true,  GArmed, false, false, false },
		// Not persisting the disarm re-arms it next launch.
		{ "H11", { Lo, Lo, { true,  7 }, { true,  7 }, true,  false, true,  true  },
			7, false, false, true,  ChaperoneLoadGate::IncompleteOwner, true, false, false },
		{ "H12", { Lo, Lo, { true,  7 }, { true,  7 }, true,  true,  false, true  },
			7, false, false, true,  ChaperoneLoadGate::ForeignTrackingSystem, true, false, false },
		// With no profile there is no reference system to disagree with.
		{ "H13", { Lo, Lo, { true,  7 }, { true,  7 }, true,  true,  false, false },
			7, false, false, false, GArmed, false, false, false },
		// Reordering the chain, or judging the later gates against the PRE-disarm
		// state, would show the user two banners for one fault.
		{ "H14", { Lo, Lo, { true,  7 }, { true,  6 }, true,  false, true,  true  },
			7, true,  true,  true,  GArmed, true,  false, false },
		// `=` rather than `|=` in the gate branches: a conservative disarm must
		// not be persisted over data that is still recoverable.
		{ "H15", { Un, Lo, { false, 0 }, { true,  7 }, true,  false, true,  false },
			7, false, false, true,  ChaperoneLoadGate::IncompleteOwner, false, false, false },
	};

	std::string why;
	for (const LoadCase &c : cases)
	{
		PersistenceLoadPlan plan = PlanPersistenceLoad(c.facts);
		std::string bad;
		auto note = [&bad](const char *field, bool got, bool want)
		{
			if (got == want)
				return;
			bad += bad.empty() ? "" : ",";
			bad += field;
			bad += got ? "=1" : "=0";
		};
		if (plan.persistenceRevision != c.revision)
		{
			bad += "rev=" + std::to_string(plan.persistenceRevision);
		}
		note("mismatch", plan.revisionMismatch, c.mismatch);
		note("report", plan.reportRevisionMismatch, c.report);
		note("disarm", plan.disarmChaperone, c.disarm);
		note("rewrite", plan.settingsRewriteNeeded, c.rewrite);
		note("latch", plan.legacySettingsMigrationPending, c.latch);
		note("latchIfFails", plan.legacySettingsMigrationPendingIfRewriteFails,
			c.latchIfRewriteFails);
		if (plan.gate != c.gate)
		{
			bad += bad.empty() ? "" : ",";
			bad += std::string("gate=") + PersistLoadGateName(plan.gate);
		}
		if (!bad.empty())
			why += std::string(" ") + c.cell + "(" + bad + ")";
	}

	char detail[320];
	snprintf(detail, sizeof detail, "%zu rows over {profile} x {settings} x {revision}%s%s",
		sizeof cases / sizeof cases[0], why.empty() ? "" : "  <-", why.c_str());
	Check("persistence H: load plan", why.empty(), detail);
}

// The save-scheduling machine that used to be eight loose fields on
// CalibrationContext, in a header the harness could not compile. Each check is
// one line from the finding's "Preserve" list: two independent dirty bits over
// one shared clock, a partial write that retries only the record that failed,
// a correction stream that cannot defer the flush forever, and Clear()'s
// survivor rule.
void RunPersistenceScheduleScenario()
{
	using questcal::PersistenceState;
	std::string why;
	auto want = [&why](const char *cell, bool ok)
	{
		if (!ok)
			why += std::string(" ") + cell;
	};

	// A: one shared clock, two independent bits. The quiet-period clock follows
	// the latest mark; the max-age clock stays on the clean -> dirty edge.
	{
		PersistenceState p;
		want("A1-clean", !p.HasDirty() && !p.Due(0.0));
		p.MarkProfile(0.0);
		p.MarkSettings(3.0);
		want("A2-bits", p.profileDirty && p.settingsDirty);
		want("A3-quiet-clock", p.dirtyTime == 3.0);
		want("A4-age-clock", p.firstDirtyTime == 0.0);
		want("A5-not-due", !p.Due(7.0));    // 4 s quiet, 7 s old
		want("A6-due", p.Due(9.0));         // 6 s quiet -> past the 5 s period
	}

	// B: the anti-starvation ceiling. A correction every second never lets the
	// stream go quiet, so only the max-age clock can force the write.
	{
		PersistenceState p;
		p.MarkProfile(0.0);
		bool dueEarly = false;
		for (int t = 1; t <= 60; ++t)
		{
			p.MarkProfile(static_cast<double>(t));
			if (p.Due(static_cast<double>(t)))
				dueEarly = true;
		}
		want("B1-never-quiet", !dueEarly);
		want("B2-age-clock-held", p.firstDirtyTime == 0.0);
		want("B3-ceiling-fires", p.Due(61.0));
	}

	// C: a partial write. The profile half landed, Settings did not; only the
	// failed record stays dirty, and Retry puts it on the quiet-period cadence
	// rather than re-attempting once per tick.
	{
		PersistenceState p;
		p.MarkProfileAndSettings(0.0);
		p.profileDirty = false;          // the half that succeeded
		want("C1-settings-alone", !p.profileDirty && p.settingsDirty && p.HasDirty());
		want("C2-due-at-failure", p.Due(61.0));
		p.Retry(61.0);
		want("C3-both-clocks-reset", p.dirtyTime == 61.0 && p.firstDirtyTime == 61.0);
		want("C4-not-every-tick", !p.Due(61.1) && !p.Due(65.0));
		want("C5-retry-cadence", p.Due(67.0));
		// Retry on a clean machine must not arm the clocks.
		PersistenceState clean;
		clean.Retry(5.0);
		want("C6-retry-clean-noop", clean.dirtyTime == 0.0 && !clean.Due(1e6));
	}

	// D: Clear()'s survivor list. Discarding a calibration drops the profile's
	// pending write and nothing else -- the Settings record is not part of the
	// calibration, and revision/migration state outlives any one of them.
	{
		PersistenceState p;
		p.MarkProfileAndSettings(2.0);
		p.SetRevision(9);
		p.coupled = true;
		p.legacySettingsMigrationPending = true;
		p.OnProfileDiscarded();
		want("D1-profile-dropped", !p.profileDirty);
		want("D2-settings-survives", p.settingsDirty);
		want("D3-revision-survives", p.revision == 9);
		want("D4-coupled-survives", p.coupled);
		want("D5-migration-survives", p.legacySettingsMigrationPending);
		want("D6-clocks-survive", p.dirtyTime == 2.0 && p.firstDirtyTime == 2.0);
	}

	// E: a persisted revision is never zero, on every path that sets one.
	{
		PersistenceState p;
		p.SetRevision(0);
		want("E1-zero-becomes-one", p.revision == 1);
		p.SetRevision(7);
		want("E2-passthrough", p.revision == 7);
		PersistenceState fresh;
		fresh.AdvanceRevision();
		want("E3-advance", fresh.revision == 1 && fresh.coupled);
		PersistenceState wrap;
		wrap.SetRevision(0xFFFFFFFFu);
		wrap.AdvanceRevision();
		want("E4-wrap-skips-zero", wrap.revision == 1);
	}

	char detail[320];
	snprintf(detail, sizeof detail,
		"quiet %.0f s / ceiling %.0f s; bits independent, retry restarts both clocks%s%s",
		PersistenceState::QuietPeriodSeconds, PersistenceState::MaxDirtyAgeSeconds,
		why.empty() ? "" : "  <-", why.c_str());
	Check("persistence I: save schedule", why.empty(), detail);
}

void RunPersistenceScenarios()
{
	RunPersistenceRoundTripScenario();
	RunPersistenceRevisionScenario();
	RunPersistenceNestingScenario();
	RunPersistenceRejectionScenario();
	RunPersistenceLegacyScenario();
	RunPersistenceContractScenario();
	RunPersistenceWriteGateScenario();
	RunPersistenceLoadPlanScenario();
	RunPersistenceScheduleScenario();
}

} // namespace

// ---------------------------------------------------------------------------
// Calibration guide: the live feedback the modal shows during collection.
// ---------------------------------------------------------------------------

static std::vector<PoseSample> GuideStream(double seconds, double rate,
	const std::function<Eigen::Quaterniond(double)> &rotAt,
	const std::function<Eigen::Vector3d(double)> &posAt,
	std::mt19937 &rng, double noiseDeg)
{
	std::normal_distribution<double> noise(0.0, noiseDeg * EIGEN_PI / 180.0);
	std::vector<PoseSample> out;
	const double dt = 1.0 / rate;
	for (double t = 0.0; t < seconds; t += dt)
	{
		PoseSample s;
		s.time = t;
		Eigen::Vector3d jitter(noise(rng), noise(rng), noise(rng));
		const double a = jitter.norm();
		Eigen::Quaterniond q = rotAt(t);
		if (a > 1e-12)
			q = (q * Eigen::Quaterniond(Eigen::AngleAxisd(a, jitter / a))).normalized();
		s.rot = q;
		s.pos = posAt(t);
		s.vel = (posAt(t + dt) - posAt(t)) / dt;
		Eigen::Quaterniond dq = (rotAt(t).conjugate() * rotAt(t + dt)).normalized();
		if (dq.w() < 0.0)
			dq.coeffs() = -dq.coeffs();
		const double ang = 2.0 * std::acos(std::min(1.0, dq.w()));
		Eigen::Vector3d axis = dq.vec().norm() > 1e-12 ? dq.vec().normalized() : Eigen::Vector3d::UnitY();
		s.angVel = axis * (ang / dt);
		out.push_back(s);
	}
	return out;
}

// The legacy continuous method (hyblocker's CalibrationCalc port): the loop
// as the overlay drives it, on synthetic poses with a known calibration C and
// a known tracker pose S on the head.
void RunLegacyScenarios()
{
	using namespace questcal::legacy;
	char detail[256];
	std::mt19937 rng(777);

	const Eigen::Quaterniond Rc(Eigen::AngleAxisd(37.0 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
	const Eigen::Vector3d tc(0.8, 0.1, -0.5);
	const Eigen::Quaterniond Srot(Eigen::AngleAxisd(0.3, Eigen::Vector3d(0.2, 0.7, -0.3).normalized()));
	const Eigen::Vector3d Spos(0.05, -0.08, 0.03);

	auto headAt = [](double t, Eigen::Quaterniond &rot, Eigen::Vector3d &pos) {
		rot = Eigen::Quaterniond(
			Eigen::AngleAxisd(0.9 * std::sin(0.5 * t), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(0.45 * std::sin(0.9 * t + 1.0), Eigen::Vector3d::UnitX()));
		pos = Eigen::Vector3d(0.4 * std::sin(0.3 * t), 1.6, 0.4 * std::cos(0.2 * t));
	};
	// reference = C * target, and the tracker rides the head at S.
	auto makeSample = [&](double t, bool moving, double posNoiseM, double rotNoiseDeg) {
		std::normal_distribution<double> pn(0.0, posNoiseM);
		std::normal_distribution<double> rn(0.0, rotNoiseDeg * EIGEN_PI / 180.0);
		Eigen::Quaterniond hr;
		Eigen::Vector3d hp;
		headAt(moving ? t : 0.0, hr, hp);
		Eigen::Quaterniond refRot = hr * Srot;
		Eigen::Vector3d refPos = hr * Spos + hp;
		Eigen::Quaterniond tgtRot = Rc.conjugate() * refRot;
		Eigen::Vector3d tgtPos = Rc.conjugate() * (refPos - tc);
		Eigen::Vector3d axis = Eigen::Vector3d(pn(rng) + 1e-6, pn(rng), pn(rng) - 1e-6).normalized();
		tgtRot = (Eigen::Quaterniond(Eigen::AngleAxisd(rn(rng), axis)) * tgtRot).normalized();
		tgtPos += Eigen::Vector3d(pn(rng), pn(rng), pn(rng));
		return Sample(Pose(hr, hp), Pose(tgtRot, tgtPos), t);
	};
	auto yawErrorDeg = [&](const Eigen::AffineCompact3d &est) {
		Eigen::Quaterniond q(est.rotation());
		return q.angularDistance(Rc) * 180.0 / EIGEN_PI;
	};

	// 1. Motion: window of 100 at 20 Hz, re-solve on every full window, drop
	// a tenth afterwards, as the overlay does.
	{
		CalibrationCalc calc;
		calc.enableStaticRecalibration = false;
		bool lerp = false;
		int accepted = 0;
		for (int i = 0; i < 600; ++i)
		{
			calc.PushSample(makeSample(i * 0.05, true, 0.002, 0.2));
			if (calc.SampleCount() < 100)
				continue;
			while (calc.SampleCount() > 100)
				calc.ShiftSample();
			if (calc.ComputeIncremental(lerp, 1.5, 0.005, false))
				++accepted;
			for (int k = 0; k < 10; ++k)
				calc.ShiftSample();
		}
		double yawErr = calc.isValid() ? yawErrorDeg(calc.Transformation()) : 999.0;
		double posErr = calc.isValid() ? (calc.Transformation().translation() - tc).norm() : 999.0;
		snprintf(detail, sizeof detail, "accepted %d, yaw error %.2f deg, position error %.1f cm",
			accepted, yawErr, posErr * 100.0);
		Check("legacy: motion re-solve recovers the calibration",
			calc.isValid() && accepted > 0 && yawErr < 1.0 && posErr < 0.03, detail);
	}

	// 2. A one-axis window cannot identify the cross-universe rotation. The
	// first solve has no prior variance baseline, so it must still fail closed
	// instead of accepting an arbitrary finite transform from the position fit.
	{
		CalibrationCalc calc;
		calc.enableStaticRecalibration = false;
		std::normal_distribution<double> pn(0.0, 0.0005);
		for (int i = 0; i < 100; ++i)
		{
			const double t = i * 0.05;
			const Eigen::Quaterniond hr(
				Eigen::AngleAxisd(0.9 * std::sin(0.7 * t), Eigen::Vector3d::UnitY()));
			const Eigen::Vector3d hp(0.25 * std::sin(0.4 * t) + pn(rng),
				1.6 + pn(rng), 0.25 * std::cos(0.4 * t) + pn(rng));
			const Eigen::Quaterniond refRot = (hr * Srot).normalized();
			const Eigen::Vector3d refPos = hr * Spos + hp;
			const Eigen::Quaterniond tgtRot = (Rc.conjugate() * refRot).normalized();
			const Eigen::Vector3d tgtPos = Rc.conjugate() * (refPos - tc);
			calc.PushSample(Sample(Pose(hr, hp), Pose(tgtRot, tgtPos), t));
		}
		bool lerp = false;
		const bool ok = calc.ComputeIncremental(lerp, 1.5, 0.005, false);
		snprintf(detail, sizeof detail, "accepted %d, valid %d, axis variance %.3e",
			ok ? 1 : 0, calc.isValid() ? 1 : 0, calc.m_axisVariance);
		Check("legacy: low-diversity initial solve fails closed",
			!ok && !calc.isValid(), detail);
	}

	// 3. Static: no motion at all, the relative pose known; the re-solve
	// comes from the averaged relative pose.
	{
		CalibrationCalc calc;
		calc.enableStaticRecalibration = true;
		Eigen::AffineCompact3d S = Eigen::Translation3d(Spos) * Srot;
		calc.setRelativeTransformation(S, true);
		for (int i = 0; i < 100; ++i)
			calc.PushSample(makeSample(i * 0.05, false, 0.001, 0.1));
		bool lerp = false;
		bool ok = calc.ComputeIncremental(lerp, 1.5, 0.005, false);
		double yawErr = calc.isValid() ? yawErrorDeg(calc.Transformation()) : 999.0;
		double posErr = calc.isValid() ? (calc.Transformation().translation() - tc).norm() : 999.0;
		snprintf(detail, sizeof detail, "accepted %d, yaw error %.2f deg, position error %.1f cm",
			ok ? 1 : 0, yawErr, posErr * 100.0);
		Check("legacy: static re-solve from the relative pose",
			ok && calc.isValid() && yawErr < 0.5 && posErr < 0.02, detail);
	}

	// 4. The delta the overlay applies reproduces the re-solved calibration.
	{
		Eigen::Quaterniond oldR(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()));
		Eigen::Vector3d oldT(0.2, 0.0, -0.1);
		Eigen::Quaterniond dR;
		Eigen::Vector3d dT;
		DeltaBetweenCalibrations(oldR, oldT, Rc, tc, dR, dT);
		Eigen::Quaterniond back = (dR * oldR).normalized();
		Eigen::Vector3d backT = dR * oldT + dT;
		snprintf(detail, sizeof detail, "rotation error %.2e rad, translation error %.2e m",
			back.angularDistance(Rc), (backT - tc).norm());
		Check("legacy: delta reproduces the new calibration",
			back.angularDistance(Rc) < 1e-9 && (backT - tc).norm() < 1e-9, detail);
	}
}

void RunGuideScenarios()
{
	char detail[256];
	std::mt19937 rng(4242);
	auto still = [](double) { return Eigen::Vector3d(0.0, 1.2, 0.0); };

	// 1. Coverage: rotation about one axis stays near empty however long it
	// runs; rotation about two axes fills the ring.
	{
		auto yawOnly = [](double t) {
			return Eigen::Quaterniond(Eigen::AngleAxisd(1.2 * std::sin(0.8 * t), Eigen::Vector3d::UnitY()));
		};
		auto twoAxis = [](double t) {
			return Eigen::Quaterniond(
				Eigen::AngleAxisd(1.2 * std::sin(0.8 * t), Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(0.9 * std::sin(1.3 * t + 0.7), Eigen::Vector3d::UnitX()));
		};
		auto single = GuideStream(10.0, 90.0, yawOnly, still, rng, 0.2);
		auto varied = GuideStream(10.0, 90.0, twoAxis, still, rng, 0.2);
		GuideMetrics a = ComputeGuideMetrics(single, single);
		GuideMetrics b = ComputeGuideMetrics(varied, varied);
		snprintf(detail, sizeof detail, "single-axis %.2f  two-axis %.2f", a.coverage, b.coverage);
		Check("guide: coverage tells one axis from two",
			a.valid && b.valid && a.coverage < 0.25 && b.coverage > 0.8, detail);
	}

	// 2. Speed: the share of the last second's samples over the engine's gates.
	{
		auto slowRot = [](double t) {
			return Eigen::Quaterniond(Eigen::AngleAxisd(0.3 * std::sin(t), Eigen::Vector3d::UnitY()));
		};
		// 3 m/s for the last half second only.
		auto fastMove = [](double t) {
			return Eigen::Vector3d(t < 9.5 ? 0.0 : 3.0 * (t - 9.5), 1.2, 0.0);
		};
		auto stream = GuideStream(10.0, 90.0, slowRot, fastMove, rng, 0.1);
		GuideMetrics m = ComputeGuideMetrics(stream, stream);
		snprintf(detail, sizeof detail, "gated fraction %.2f (expect ~0.5)", m.gatedFraction);
		Check("guide: speed gate fraction",
			m.valid && m.gatedFraction > 0.35 && m.gatedFraction < 0.65, detail);
	}

	// 3. Rigidity: a fixed relative rotation reads tight; a wobbling one loose.
	{
		auto rot = [](double t) {
			return Eigen::Quaterniond(
				Eigen::AngleAxisd(0.8 * std::sin(0.9 * t), Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(0.5 * std::sin(1.4 * t), Eigen::Vector3d::UnitX()));
		};
		const Eigen::Quaterniond mount(Eigen::AngleAxisd(0.6, Eigen::Vector3d(0.3, 0.8, 0.2).normalized()));
		auto rigid = [&](double t) { return (rot(t) * mount).normalized(); };
		auto loose = [&](double t) {
			return (rot(t) * mount *
				Eigen::Quaterniond(Eigen::AngleAxisd(0.25 * std::sin(6.0 * t), Eigen::Vector3d::UnitZ()))).normalized();
		};
		auto ref = GuideStream(4.0, 90.0, rot, still, rng, 0.1);
		auto tightPair = GuideStream(4.0, 72.0, rigid, still, rng, 0.1);
		auto loosePair = GuideStream(4.0, 72.0, loose, still, rng, 0.1);
		auto crossUniversePair = tightPair;
		const Eigen::Quaterniond targetToReference(
			Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, 0.9, -0.3).normalized()));
		for (auto &sample : crossUniversePair)
			sample.rot = (targetToReference.conjugate() * sample.rot).normalized();
		GuideMetrics a = ComputeGuideMetrics(ref, tightPair);
		GuideMetrics b = ComputeGuideMetrics(ref, loosePair);
		GuideMetrics c = ComputeGuideMetrics(ref, crossUniversePair);
		snprintf(detail, sizeof detail, "rigid %.2f deg  loose %.2f deg  cross-universe %.2f deg",
			a.rigidityDeg, b.rigidityDeg, c.rigidityDeg);
		Check("guide: rigidity spread",
			a.rigidityValid && b.rigidityValid && c.rigidityValid &&
			a.rigidityDeg < 1.5 && b.rigidityDeg > 5.0 && c.rigidityDeg < 1.5,
			detail);
	}
}

// ---------------------------------------------------------------------------
// Update feed policy: network and installation stay outside this harness, while
// every rule that decides whether a remote asset is executable input is pure.
// ---------------------------------------------------------------------------

picojson::value UpdateReleaseValue(const std::string &tag, bool draft,
	bool prerelease, const std::string &assetName, const std::string &digest,
	bool duplicate = false)
{
	picojson::object asset;
	asset["name"] = picojson::value(assetName);
	asset["size"] = picojson::value(1329433.0);
	asset["digest"] = picojson::value(digest);
	asset["browser_download_url"] = picojson::value(
		"https://github.com/VividNightmareUnleashed/QuestCalibrator/releases/download/" +
		tag + "/" + assetName);
	picojson::value assetValue(asset);
	picojson::array assets;
	assets.push_back(assetValue);
	if (duplicate)
		assets.push_back(assetValue);

	picojson::object release;
	release["tag_name"] = picojson::value(tag);
	release["draft"] = picojson::value(draft);
	release["prerelease"] = picojson::value(prerelease);
	release["html_url"] = picojson::value(
		"https://github.com/VividNightmareUnleashed/QuestCalibrator/releases/tag/" + tag);
	release["assets"] = picojson::value(assets);
	return picojson::value(release);
}

void RunUpdatePolicyScenarios()
{
	using namespace questcal::update;
	const std::string digest =
		"sha256:d768f19e0724ef00432d694bf6e5ad23c9010de5a32b087d56e0c6bd094d8072";
	Version parsed;
	const bool validTag = ParseReleaseTag("questcalibrator-v12.34.56", parsed);
	Check("updates: only canonical stable tags parse",
		validTag && parsed.major == 12 && parsed.minor == 34 && parsed.patch == 56 &&
		!ParseReleaseTag("v12.34.56", parsed) &&
		!ParseReleaseTag("questcalibrator-v12.34.56-alpha.1", parsed) &&
		!ParseReleaseTag("questcalibrator-v12.034.56", parsed) &&
		!ParseReleaseTag("questcalibrator-v42949672960.0.0", parsed), "");

	std::array<unsigned char, 32> digestBytes{};
	Check("updates: SHA-256 metadata is strict",
		ParseSha256Digest(digest, digestBytes) && digestBytes[0] == 0xd7 &&
		digestBytes[31] == 0x72 &&
		!ParseSha256Digest("sha256:d768", digestBytes) &&
		!ParseSha256Digest(
			"sha256:z768f19e0724ef00432d694bf6e5ad23c9010de5a32b087d56e0c6bd094d8072",
			digestBytes), "");

	picojson::array releases;
	releases.push_back(UpdateReleaseValue("questcalibrator-v9.0.0", true, false,
		"QuestCalibrator-9.0.0.zip", digest));
	releases.push_back(UpdateReleaseValue("questcalibrator-v8.0.0", false, true,
		"QuestCalibrator-8.0.0.zip", digest));
	releases.push_back(UpdateReleaseValue("v99.0.0", false, false,
		"QuestCalibrator-99.0.0.zip", digest));
	releases.push_back(UpdateReleaseValue("questcalibrator-v1.2.0", false, false,
		"QuestCalibrator-1.2.0.zip", digest));
	picojson::value feed;
	feed.set<picojson::array>(std::move(releases));
	ReleaseCandidate candidate;
	bool available = false;
	std::string error;
	const bool selected = SelectReleaseCandidate(feed.serialize(),
		Version{ 1, 1, 0 }, candidate, available, error);
	Check("updates: newest eligible stable release wins",
		selected && available && error.empty() &&
		VersionString(candidate.version) == "1.2.0" &&
		candidate.packageName == "QuestCalibrator-1.2.0.zip" &&
		candidate.size == 1329433, error.c_str());

	ReleaseCandidate none;
	bool newerAvailable = true;
	error.clear();
	const bool current = SelectReleaseCandidate(feed.serialize(),
		Version{ 1, 2, 0 }, none, newerAvailable, error);
	Check("updates: current version does not redownload",
		current && !newerAvailable && error.empty(), error.c_str());

	picojson::array missingDigest;
	missingDigest.push_back(UpdateReleaseValue("questcalibrator-v2.0.0", false,
		false, "QuestCalibrator-2.0.0.zip", ""));
	picojson::value missingDigestFeed;
	missingDigestFeed.set<picojson::array>(std::move(missingDigest));
	available = false;
	error.clear();
	const bool acceptedMissingDigest = SelectReleaseCandidate(
		missingDigestFeed.serialize(), Version{ 1, 1, 0 }, candidate, available, error);
	Check("updates: package without digest fails closed",
		!acceptedMissingDigest && available && !error.empty(), error.c_str());

	picojson::array duplicate;
	duplicate.push_back(UpdateReleaseValue("questcalibrator-v2.0.0", false,
		false, "QuestCalibrator-2.0.0.zip", digest, true));
	picojson::value duplicateFeed;
	duplicateFeed.set<picojson::array>(std::move(duplicate));
	available = false;
	error.clear();
	const bool acceptedDuplicate = SelectReleaseCandidate(duplicateFeed.serialize(),
		Version{ 1, 1, 0 }, candidate, available, error);
	Check("updates: duplicate canonical packages fail closed",
		!acceptedDuplicate && available && !error.empty(), error.c_str());
}

int main(int argc, char **argv)
{
	// Unbuffered: an abort discards a buffered stdout, so a harness that dies
	// mid-run reports nothing at all about where. That is precisely how a
	// zero-sigma normal_distribution kept every Debug build of this harness
	// from ever completing without leaving a single line of evidence.
	setvbuf(stdout, nullptr, _IONBF, 0);
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
	RunDriverSyncScenarios();
	RunDriverSessionScenarios();
	RunDriverWorkerScenario();
	RunDriverSyncStateScenarios();
	RunPoseChannelScenarios();
	RunSolverPrimitiveScenarios();
	RunSolverRobustnessScenarios();
	RunSolverPropertyScenarios(propertyTrials, propertySeed);

	// Accuracy bands below are derived, not guessed. Measurement: 12 seeds x
	// {Debug, Release} on MSVC 14.44. Two results shaped every number here.
	// First, the deterministic scenarios are bit-identical across seeds AND
	// across optimization level - only the property scenario varies with
	// --property-seed - so these bands were never absorbing observed noise;
	// the 12x-43x headroom was pure slack. Second, where draws DO change (the
	// property scenario, 12 seeds) the worst-case error moves about 1.3x above
	// its mean.
	//
	// So each band is ~4x its measured error: comfortably past the 1.3x that
	// changing draws costs, while turning an assertion that could not fail
	// into one that can. 4x rather than 2x because std::normal_distribution is
	// implementation-defined - a different STL generates different scenes
	// entirely, which is the one axis this machine cannot measure. Bands
	// already inside 4x were left alone; none were loosened.
	// Floors of 0.02 deg / 0.001 m keep the exact-recovery cases off
	// bit-for-bit equality.

	// 1. Clean data: near-exact recovery (measured 0.0000 deg / 0.0000 m).
	{
		SceneConfig scene;
		Expectation e;
		e.maxRotErrDeg = 0.02;
		e.maxTransErrM = 0.001;
		RunScenario("clean", scene, truth, config, e);
	}

	// 2. Realistic noise (measured 0.0117 deg / 0.0006 m; was 43x / 25x).
	{
		SceneConfig scene;
		scene.posNoise = 0.002;
		scene.rotNoiseDeg = 0.2;
		Expectation e;
		e.maxRotErrDeg = 0.05;
		e.maxTransErrM = 0.0025;
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
		// Measured 0.0297 deg / 0.0008 m; was 24x / 25x.
		Expectation e;
		e.maxRotErrDeg = 0.12;
		e.maxTransErrM = 0.0035;
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
		// Measured 0.0592 deg / 0.0016 m; was 17x / 12x.
		Expectation e;
		e.maxRotErrDeg = 0.24;
		e.maxTransErrM = 0.007;
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
		// Measured 0.0135 deg / 0.0007 m; was 44x / 29x - the loosest band here.
		Expectation e;
		e.maxRotErrDeg = 0.06;
		e.maxTransErrM = 0.003;
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
		// Measured 0.0256 deg / 0.0011 m; was 14x / 18x.
		Expectation e;
		e.maxRotErrDeg = 0.11;
		e.maxTransErrM = 0.005;
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
		// Measured 0.0487 deg / 0.0012 m; was 12x / 17x. This is the scenario
		// built to pin the near-pi pair gate, so slack here is the least
		// affordable: removing the gate has to move the number past this band.
		Expectation e;
		e.maxRotErrDeg = 0.20;
		e.maxTransErrM = 0.005;
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

		// The opposite frequency ordering is non-physical too. Smoothing the
		// target makes reference/target fine gain exceed gross gain; it must be
		// neutralized, never accepted merely because it is not the familiar
		// fine-below-gross signature.
		EngineConfig oppositeCfg = sc;
		oppositeCfg.gainSmoothingMargin = 0.01;
		EngineResult opposite = CalibrationEngine::Solve(
			ref, SmoothStreamZeroPhase(tgt, 0.12, 0.02), oppositeCfg);
		bool oppositeGuarded = opposite.valid && opposite.motionGainValid &&
			opposite.motionGainHigh > opposite.motionGainLow + oppositeCfg.gainSmoothingMargin &&
			opposite.motionGainInconsistent && !opposite.motionSmoothingDetected &&
			opposite.scaleGuard == ScaleGuard::NeutralizedForSmoothing &&
			std::abs(opposite.scale - 1.0) < 1e-9;

		// Exercise the conditional-information gate independently of the band
		// estimator. A deliberately stricter confidence contract rejects the
		// free fit; the default-safe guard can still produce a neutral transform.
		EngineConfig confidenceCfg = sc;
		confidenceCfg.minScaleCondition = 0.9;
		confidenceCfg.pinScaleOnSmoothing = false;
		EngineResult rejectedConfidence = CalibrationEngine::Solve(ref, tgt,
			confidenceCfg);
		confidenceCfg.pinScaleOnSmoothing = true;
		EngineResult neutralConfidence = CalibrationEngine::Solve(ref, tgt,
			confidenceCfg);
		bool confidenceGated = !rejectedConfidence.valid &&
			!rejectedConfidence.scaleIdentifiable && neutralConfidence.valid &&
			!neutralConfidence.scaleIdentifiable &&
			neutralConfidence.scaleGuard == ScaleGuard::NeutralizedForSmoothing &&
			neutralConfidence.scale == 1.0;

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
			std::abs(cleanGross.scale - 1.0) <= grossCfg.maxCleanGrossDeviation + 1e-6 &&
			oppositeGuarded && confidenceGated;
		printf("%-28s %s  short %d/%d/%.3f/%d  gross %d %.3f  opposite %.3f/%.3f guard %d  confidence %d\n",
			"scale diagnostic branches", pass ? "PASS" : "FAIL",
			shortResult.valid, shortResult.motionGainValid, shortResult.scale,
			static_cast<int>(shortResult.scaleGuard),
			cleanGrossSeen, cleanGross.scale, opposite.motionGainLow,
			opposite.motionGainHigh, static_cast<int>(opposite.scaleGuard), confidenceGated);
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
	RunGuideScenarios();
	RunLegacyScenarios();
	RunUpdatePolicyScenarios();
	RunReviewRegressionScenarios(Check);

	// ---- Profile persistence: codec, write gates, load plan ----
	RunPersistenceScenarios();

	printf("\n%d scenario(s) ran, %d failed\n", checksRun, failures);
	return failures;
}
