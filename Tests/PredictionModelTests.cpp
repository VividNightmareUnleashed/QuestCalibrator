// What a game sees is not the driver pose QuestCalibrator rewrites but
// vrserver's extrapolation of it: the position advanced along the reported
// velocity and acceleration, the orientation along the angular velocity, to
// the moment the frame will be shown, all in driver space, then carried into
// world space by worldFromDriver and into head space by driverFromHead. That
// model is written down here from the contract in openvr_driver.h (Valve's
// code is closed; the prediction horizon it picks is unknown and is swept).
// The invariant it checks is the one the rewrite in PoseTransform.h relies
// on: calibrating the pose commutes with predicting it, so the game sees the
// calibrated version of the pose it would have seen, at every horizon, and
// the latency correction shifts only which instant that is.
#include "../Driver/PoseTransform.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

namespace
{

Eigen::Quaterniond Q(const vr::HmdQuaternion_t &q) { return Eigen::Quaterniond(q.w, q.x, q.y, q.z); }
Eigen::Vector3d V(const double (&v)[3]) { return Eigen::Vector3d(v[0], v[1], v[2]); }

struct WorldPose
{
	Eigen::Vector3d position;      // the tracked origin in world space
	Eigen::Quaterniond rotation;
	Eigen::Vector3d head;          // the head/render origin: position plus the rotated driverFromHead offset
};

// vrserver's extrapolation as the header describes it. The pose is
// `poseTimeOffset` seconds from the PoseUpdated call; a frame `horizon`
// seconds after that call needs the pose advanced by their difference. The
// angular step is applied in the frame the driver reports its angular
// velocity in; the invariants below hold under either convention because
// the rewrite never touches the device-local orientation.
WorldPose PredictWorld(const vr::DriverPose_t &p, double horizon)
{
	const double dt = horizon - p.poseTimeOffset;
	Eigen::Vector3d position = V(p.vecPosition) + V(p.vecVelocity) * dt +
		0.5 * V(p.vecAcceleration) * dt * dt;
	Eigen::Vector3d w = V(p.vecAngularVelocity);
	Eigen::Quaterniond spin = Eigen::Quaterniond::Identity();
	if (w.norm() > 1e-12)
		spin = Eigen::Quaterniond(Eigen::AngleAxisd(w.norm() * dt, w.normalized()));
	Eigen::Quaterniond local = spin * Q(p.qRotation);

	WorldPose out;
	out.position = Q(p.qWorldFromDriverRotation) * position + V(p.vecWorldFromDriverTranslation);
	out.rotation = Q(p.qWorldFromDriverRotation) * local;
	out.head = out.position + out.rotation * V(p.vecDriverFromHeadTranslation);
	return out;
}

Eigen::Quaterniond RandomRotation(std::mt19937 &rng)
{
	std::uniform_real_distribution<double> u(-1.0, 1.0);
	std::uniform_real_distribution<double> angle(-EIGEN_PI, EIGEN_PI);
	Eigen::Vector3d axis;
	do
		axis = Eigen::Vector3d(u(rng), u(rng), u(rng));
	while (axis.squaredNorm() < 1e-8);
	return Eigen::Quaterniond(Eigen::AngleAxisd(angle(rng), axis.normalized()));
}

Eigen::Vector3d RandomVector(std::mt19937 &rng, double magnitude)
{
	std::uniform_real_distribution<double> u(-magnitude, magnitude);
	return Eigen::Vector3d(u(rng), u(rng), u(rng));
}

vr::DriverPose_t RandomPose(std::mt19937 &rng, const Eigen::Vector3d &driverFromHead)
{
	vr::DriverPose_t pose{};
	Eigen::Quaterniond wfd = RandomRotation(rng), local = RandomRotation(rng);
	Eigen::Vector3d wfdT = RandomVector(rng, 2.0), position = RandomVector(rng, 2.0);
	Eigen::Vector3d velocity = RandomVector(rng, 4.0), acceleration = RandomVector(rng, 10.0);
	Eigen::Vector3d angular = RandomVector(rng, 6.0);
	std::uniform_real_distribution<double> offset(-0.05, 0.05);
	pose.poseTimeOffset = offset(rng);
	pose.qWorldFromDriverRotation = { wfd.w(), wfd.x(), wfd.y(), wfd.z() };
	pose.qRotation = { local.w(), local.x(), local.y(), local.z() };
	pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
	for (int i = 0; i < 3; ++i)
	{
		pose.vecWorldFromDriverTranslation[i] = wfdT(i);
		pose.vecPosition[i] = position(i);
		pose.vecVelocity[i] = velocity(i);
		pose.vecAcceleration[i] = acceleration(i);
		pose.vecAngularVelocity[i] = angular(i);
		pose.vecDriverFromHeadTranslation[i] = driverFromHead(i);
	}
	return pose;
}

struct Calibration
{
	Eigen::Quaterniond rotation;
	Eigen::Vector3d translation;
	double scale;
	Eigen::Vector3d Apply(const Eigen::Vector3d &world) const { return rotation * (scale * world) + translation; }
};

} // namespace

void RunPredictionModelScenarios(void (*check)(const char *, bool, const char *))
{
	std::mt19937 rng(0x5EED1234u);
	std::uniform_real_distribution<double> scaleDist(0.85, 1.15);
	std::uniform_real_distribution<double> shiftDist(-0.05, 0.05);
	const double horizons[] = { -0.02, 0.0, 0.011, 0.03, 0.06 };
	char detail[256];

	// 1. With no latency correction, calibrating then predicting equals
	//    predicting then calibrating, at every horizon, for position and
	//    orientation alike.
	double worstPosition = 0.0, worstRotation = 0.0;
	for (int trial = 0; trial < 128; ++trial)
	{
		Calibration cal{ RandomRotation(rng), RandomVector(rng, 3.0), scaleDist(rng) };
		vr::DriverPose_t raw = RandomPose(rng, Eigen::Vector3d::Zero());
		vr::DriverPose_t rewritten = raw;
		double t[3] = { cal.translation.x(), cal.translation.y(), cal.translation.z() };
		vr::HmdQuaternion_t r{ cal.rotation.w(), cal.rotation.x(), cal.rotation.y(), cal.rotation.z() };
		questcal::driverpose::Apply(rewritten, r, t, cal.scale, 0.0);
		for (double h : horizons)
		{
			WorldPose seen = PredictWorld(rewritten, h);
			WorldPose expected = PredictWorld(raw, h);
			worstPosition = std::max(worstPosition, (seen.position - cal.Apply(expected.position)).norm());
			worstRotation = std::max(worstRotation,
				seen.rotation.angularDistance(cal.rotation * expected.rotation));
		}
	}
	snprintf(detail, sizeof detail, "position %.2e m, rotation %.2e rad over 5 horizons", worstPosition, worstRotation);
	check("prediction model: the rewrite commutes with vrserver's extrapolation",
		worstPosition < 1e-9 && worstRotation < 1e-9, detail);

	// 2. The latency correction shifts poseTimeOffset by delta. Predicting
	//    the rewritten pose to horizon h then shows the calibrated pose the
	//    raw stream had at h - delta: the correction moves the instant, not
	//    the geometry.
	double worstShifted = 0.0, worstUnshifted = 0.0;
	for (int trial = 0; trial < 128; ++trial)
	{
		Calibration cal{ RandomRotation(rng), RandomVector(rng, 3.0), scaleDist(rng) };
		const double delta = shiftDist(rng);
		vr::DriverPose_t raw = RandomPose(rng, Eigen::Vector3d::Zero());
		vr::DriverPose_t rewritten = raw;
		double t[3] = { cal.translation.x(), cal.translation.y(), cal.translation.z() };
		vr::HmdQuaternion_t r{ cal.rotation.w(), cal.rotation.x(), cal.rotation.y(), cal.rotation.z() };
		questcal::driverpose::Apply(rewritten, r, t, cal.scale, delta);
		for (double h : horizons)
		{
			WorldPose seen = PredictWorld(rewritten, h);
			worstShifted = std::max(worstShifted,
				(seen.position - cal.Apply(PredictWorld(raw, h - delta).position)).norm());
			// The same comparison without the shift must fail whenever the
			// pose is moving, or the check above proves nothing.
			if (std::abs(delta) > 0.005)
				worstUnshifted = std::max(worstUnshifted,
					(seen.position - cal.Apply(PredictWorld(raw, h).position)).norm());
		}
	}
	snprintf(detail, sizeof detail, "shifted %.2e m; the unshifted comparison differs by up to %.3f m",
		worstShifted, worstUnshifted);
	check("prediction model: a latency shift moves the instant, not the pose",
		worstShifted < 1e-9 && worstUnshifted > 0.01, detail);

	// 3. The head offset is deliberately left unscaled (PoseTransform.h), so
	//    the head/render origin lands (1 - s) times that offset away from the
	//    calibrated head; exactly that, and under a millimetre for the
	//    corrections the solver actually produces.
	double worstMismatch = 0.0, worstRealistic = 0.0;
	for (int trial = 0; trial < 128; ++trial)
	{
		Calibration cal{ RandomRotation(rng), RandomVector(rng, 3.0), scaleDist(rng) };
		Eigen::Vector3d driverFromHead = RandomVector(rng, 0.4);
		vr::DriverPose_t raw = RandomPose(rng, driverFromHead);
		vr::DriverPose_t rewritten = raw;
		double t[3] = { cal.translation.x(), cal.translation.y(), cal.translation.z() };
		vr::HmdQuaternion_t r{ cal.rotation.w(), cal.rotation.x(), cal.rotation.y(), cal.rotation.z() };
		questcal::driverpose::Apply(rewritten, r, t, cal.scale, 0.0);
		for (double h : horizons)
		{
			double away = (PredictWorld(rewritten, h).head - cal.Apply(PredictWorld(raw, h).head)).norm();
			double predicted = std::abs(1.0 - cal.scale) * driverFromHead.norm();
			worstMismatch = std::max(worstMismatch, std::abs(away - predicted));
		}
		// A 2 % scale correction on a 5 cm offset.
		Calibration realistic{ cal.rotation, cal.translation, 1.02 };
		Eigen::Vector3d smallOffset = driverFromHead.normalized() * 0.05;
		vr::DriverPose_t rawSmall = RandomPose(rng, smallOffset);
		vr::DriverPose_t rewrittenSmall = rawSmall;
		questcal::driverpose::Apply(rewrittenSmall, r, t, realistic.scale, 0.0);
		worstRealistic = std::max(worstRealistic,
			(PredictWorld(rewrittenSmall, 0.011).head - realistic.Apply(PredictWorld(rawSmall, 0.011).head)).norm());
	}
	snprintf(detail, sizeof detail, "|away - (1-s)|offset|| %.2e m; 2%% scale on a 5 cm offset: %.2f mm",
		worstMismatch, worstRealistic * 1000.0);
	check("prediction model: the unscaled head offset lands (1-s) x offset away",
		worstMismatch < 1e-9 && worstRealistic <= 0.001 + 1e-9, detail);
}
