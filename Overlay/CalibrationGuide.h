#pragma once

// Live feedback for the calibration modal, computed from the run's own sample
// buffers so the player sees the solver's gates filling before they fire:
// how much rotation variety the pair has covered, whether the motion is too
// fast for the sample rate, and whether the two devices are moving as one.
//
// Pure Eigen + CalibrationEngine reuse; no OpenVR or UI dependencies, so the
// synthetic test harness compiles exactly the code the overlay ships.

#include "CalibrationEngine.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <vector>

namespace questcal
{

struct GuideMetrics
{
	bool valid = false;          // enough samples to say anything at all
	double coverage = 0.0;       // 0..1: rotation-axis diversity, 1 = well past the gate
	double gatedFraction = 0.0;  // share of recent target samples over the speed gates
	bool rigidityValid = false;
	double rigidityDeg = 0.0;    // RMS spread of the reference->target relative rotation
};

struct GuideConfig
{
	double minPairAngle = 0.4;     // rad; same band the engine pairs over
	double maxPairAngle = 2.9;
	// Second/first eigenvalue ratio of the delta-axis scatter at which the ring
	// reads full. The engine refuses below 0.010; this is well above that so a
	// full ring means a comfortable solve, not a marginal one.
	double fullSpread = 0.15;
	double maxLinearSpeed = 1.6;   // m/s; EngineConfig's gates
	double maxAngularSpeed = 8.0;  // rad/s
	double windowSeconds = 1.0;    // recent window for speed and rigidity
	double maxInterpolationGap = 0.06;
};

// Rotation-axis diversity over the collection so far, mirroring the engine's
// axis-spread gate: delta rotations between samples a few lags apart, their
// axes accumulated into a scatter matrix, and the second/first eigenvalue
// ratio of that matrix. Rotation about one axis alone leaves the ratio near
// zero however long it goes on; that is exactly the failure this is meant to
// show before the solve does.
//
// The helpers below take the collection ComputeGuideMetrics has already sized:
// at least 8 target samples.
inline double GuideAxisCoverage(const std::vector<PoseSample> &stream,
                                const GuideConfig &config)
{
	// Subsample to ~200 points so the cost stays flat over a 35 s run.
	const size_t stride = std::max<size_t>(1, stream.size() / 200);
	std::vector<Eigen::Quaterniond> q;
	q.reserve(stream.size() / stride + 1);
	for (size_t i = 0; i < stream.size(); i += stride)
		q.push_back(stream[i].rot);

	Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
	size_t pairs = 0;
	static const size_t lags[] = { 1, 2, 4, 8, 16, 32 };
	for (size_t lag : lags)
	{
		for (size_t i = lag; i < q.size(); ++i)
		{
			Eigen::Quaterniond d = (q[i - lag].conjugate() * q[i]).normalized();
			if (d.w() < 0.0)
				d.coeffs() = -d.coeffs();
			const double angle = 2.0 * std::acos(std::min(1.0, d.w()));
			if (angle < config.minPairAngle || angle > config.maxPairAngle)
				continue;
			// |axis| = sin(angle / 2) >= sin(minPairAngle / 2).
			const Eigen::Vector3d axis = d.vec().normalized();
			scatter += axis * axis.transpose();
			++pairs;
		}
	}
	if (pairs < 5)
		return 0.0;

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(scatter);
	const Eigen::Vector3d ev = es.eigenvalues();   // ascending; ev(2) >= pairs / 3
	const double spread = ev(1) / ev(2);
	return std::min(1.0, spread / config.fullSpread);
}

// Share of the recent target samples the engine would drop for speed.
inline double GuideGatedFraction(const std::vector<PoseSample> &target,
                                 const GuideConfig &config)
{
	const double cutoff = target.back().time - config.windowSeconds;
	size_t total = 0, gated = 0;
	for (size_t i = target.size(); i-- > 0;)
	{
		const PoseSample &s = target[i];
		if (s.time < cutoff)
			break;
		++total;
		if (s.vel.norm() > config.maxLinearSpeed || s.angVel.norm() > config.maxAngularSpeed)
			++gated;
	}
	return static_cast<double>(gated) / static_cast<double>(total);   // the last sample counts
}

// How differently the two devices rotate over short, timestamp-aligned
// intervals in the recent window. The raw streams are expressed in different
// tracking universes, so their direct relative rotation is not constant even
// for a rigid pair. Rotation angle is invariant under both the unknown
// cross-universe rotation and the fixed device-to-device mount rotation; a
// loose pair changes that angle independently.
inline bool GuideRigidity(const std::vector<PoseSample> &ref,
                          const std::vector<PoseSample> &target,
                          const GuideConfig &config, double &rmsDegOut)
{
	rmsDegOut = 0.0;
	const double cutoff = target.back().time - config.windowSeconds;
	constexpr double comparisonSeconds = 0.2;
	double sq = 0.0;
	size_t comparisons = 0;
	for (const PoseSample &currentTarget : target)
	{
		if (currentTarget.time < cutoff + comparisonSeconds)
			continue;
		const double previousTime = currentTarget.time - comparisonSeconds;
		PoseSample previousRef, currentRef, previousTarget;
		if (!CalibrationEngine::InterpolateAt(ref, previousTime,
				config.maxInterpolationGap, previousRef) ||
			!CalibrationEngine::InterpolateAt(ref, currentTarget.time,
				config.maxInterpolationGap, currentRef) ||
			!CalibrationEngine::InterpolateAt(target, previousTime,
				config.maxInterpolationGap, previousTarget))
			continue;

		const double referenceAngle = previousRef.rot.angularDistance(currentRef.rot);
		const double targetAngle = previousTarget.rot.angularDistance(currentTarget.rot);
		const double difference = referenceAngle - targetAngle;
		sq += difference * difference;
		++comparisons;
	}
	if (comparisons < 4)
		return false;
	rmsDegOut = std::sqrt(sq / static_cast<double>(comparisons)) * 180.0 / EIGEN_PI;
	return true;
}

inline GuideMetrics ComputeGuideMetrics(const std::vector<PoseSample> &ref,
                                        const std::vector<PoseSample> &target,
                                        const GuideConfig &config = GuideConfig())
{
	GuideMetrics m;
	if (target.size() < 8)
		return m;
	m.valid = true;
	m.coverage = GuideAxisCoverage(target, config);
	m.gatedFraction = GuideGatedFraction(target, config);
	m.rigidityValid = GuideRigidity(ref, target, config, m.rigidityDeg);
	return m;
}

} // namespace questcal
