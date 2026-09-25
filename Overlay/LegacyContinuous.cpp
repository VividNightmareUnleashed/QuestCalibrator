// hyblocker/OpenVR-SpaceCalibrator (legacy-1-5-1) CalibrationCalc.cpp, math
// unchanged. See LegacyContinuous.h for what was removed around it.
#include "stdafx.h"
#include "LegacyContinuous.h"

#include <cmath>

namespace questcal {
namespace legacy {

namespace {

	struct DSample
	{
		bool valid;
		Eigen::Vector3d ref, target;
	};

	Eigen::Vector3d AxisFromRotationMatrix3(Eigen::Matrix3d rot)
	{
		return Eigen::Vector3d(rot(2, 1) - rot(1, 2), rot(0, 2) - rot(2, 0), rot(1, 0) - rot(0, 1));
	}

	double AngleFromRotationMatrix3(Eigen::Matrix3d rot)
	{
		return acos((rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) / 2.0);
	}

	// The source built an OpenVR quaternion from Euler degrees (Z, Y, X) and
	// turned it back into a matrix; the same rotation, without the detour.
	Eigen::Matrix3d RotationFromEulerDeg(const Eigen::Vector3d &eulerdeg)
	{
		auto euler = eulerdeg * EIGEN_PI / 180.0;
		Eigen::Quaterniond rotQuat =
			Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
		return rotQuat.toRotationMatrix();
	}

	DSample DeltaRotationSamples(const Sample &s1, const Sample &s2)
	{
		auto dref = s1.ref.rot * s2.ref.rot.transpose();
		auto dtarget = s1.target.rot * s2.target.rot.transpose();

		// When stuck together, the two tracked objects rotate as a pair,
		// therefore their axes of rotation must be equal between any given pair of samples.
		DSample ds;
		ds.ref = AxisFromRotationMatrix3(dref);
		ds.target = AxisFromRotationMatrix3(dtarget);

		// Reject samples that were too close to each other.
		auto refA = AngleFromRotationMatrix3(dref);
		auto targetA = AngleFromRotationMatrix3(dtarget);
		ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

		ds.ref.normalize();
		ds.target.normalize();
		return ds;
	}

	Pose ApplyTransform(const Pose &originalPose, const Eigen::AffineCompact3d &transform)
	{
		Pose pose(originalPose);
		pose.rot = transform.rotation() * pose.rot;
		pose.trans = transform * pose.trans;
		return pose;
	}

	class PoseAverager
	{
	private:
		Eigen::Matrix<double, 4, Eigen::Dynamic> quatAvg;
		Eigen::Vector3d accum = Eigen::Vector3d::Zero();
		int i = 0;
	public:
		PoseAverager(size_t n_samples)
		{
			quatAvg.resize(4, n_samples);
		}

		template<typename P>
		void Push(const P &pose)
		{
			const Eigen::Quaterniond rot(pose.rotation());
			quatAvg.col(i++) = Eigen::Vector4d(rot.w(), rot.x(), rot.y(), rot.z());
			accum += pose.translation();
		}

		Eigen::AffineCompact3d Average()
		{
			// https://stackoverflow.com/a/27410865/36723
			auto quatT = quatAvg.transpose();
			Eigen::Matrix4d quatMul = quatAvg * quatT;
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
			solver.compute(quatMul);

			Eigen::Vector4d quatAvgV = solver.eigenvectors().col(3).real().normalized();
			Eigen::Quaterniond avgQ(quatAvgV(0), quatAvgV(1), quatAvgV(2), quatAvgV(3));
			avgQ.normalize();

			Eigen::AffineCompact3d pose(avgQ);
			pose.pretranslate(accum * (1.0 / i));

			return pose;
		}

		template<typename XS, typename F>
		static Eigen::AffineCompact3d AverageFor(const XS &samples, const F &poseProvider)
		{
			int sampleCount = 0;

			for (auto &sample : samples)
			{
				if (!sample.valid) continue;
				sampleCount++;
			}

			PoseAverager accum(sampleCount);

			for (auto &sample : samples)
			{
				if (!sample.valid) continue;
				auto pose = poseProvider(sample);
				accum.Push(pose);
			}

			return accum.Average();
		}
	};
}

const double CalibrationCalc::AxisVarianceThreshold = 0.001;

void CalibrationCalc::PushSample(const Sample &sample)
{
	m_samples.push_back(sample);
}

void CalibrationCalc::Clear()
{
	m_estimatedTransformation.setIdentity();
	m_isValid = false;
	m_samples.clear();
	m_axisVariance = 0.0;
	m_refToTargetPose = Eigen::AffineCompact3d::Identity();
	m_relativePosCalibrated = false;
}

Eigen::Vector3d CalibrationCalc::CalibrateRotation() const
{
	std::vector<DSample> deltas;
	for (size_t i = 0; i < m_samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(m_samples[i], m_samples[j]);
			if (delta.valid)
				deltas.push_back(delta);
		}
	}

	// Kabsch algorithm on the horizontal (x, z) components of the rotation axes.
	Eigen::MatrixXd refPoints(deltas.size(), 2), targetPoints(deltas.size(), 2);
	Eigen::Vector2d refCentroid(0, 0), targetCentroid(0, 0);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) << deltas[i].ref[0], deltas[i].ref[2];
		refCentroid += refPoints.row(i);

		targetPoints.row(i) << deltas[i].target[0], deltas[i].target[2];
		targetCentroid += targetPoints.row(i);
	}

	refCentroid /= (double)deltas.size();
	targetCentroid /= (double)deltas.size();

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) -= refCentroid;
		targetPoints.row(i) -= targetCentroid;
	}

	auto crossCV = refPoints.transpose() * targetPoints;

	Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(crossCV);

	Eigen::Matrix2d i = Eigen::Matrix2d::Identity();
	Eigen::Matrix2d rot = svd.matrixV() * i * svd.matrixU().transpose();

	double yaw = std::atan2(rot(1, 0), rot(0, 0));

	Eigen::Vector3d euler(0.0, yaw * 180.0 / EIGEN_PI, 0.0);   // degrees
	return euler;
}

Eigen::Vector3d CalibrationCalc::CalibrateTranslation(const Eigen::Matrix3d &rotation) const
{
	const size_t count = m_samples.size();
	Eigen::VectorXd constants(count * 6);
	Eigen::MatrixXd coefficients(count * 6, 3);
	for (int family = 0; family < 2; ++family)
	{
		Eigen::Matrix3d meanQ = Eigen::Matrix3d::Zero();
		Eigen::Vector3d meanB = Eigen::Vector3d::Zero();
		for (size_t i = 0; i < count; ++i)
		{
			const auto &s = m_samples[i];
			Eigen::Matrix3d q = family == 0 ? Eigen::Matrix3d(s.ref.rot.transpose())
				: Eigen::Matrix3d((rotation * s.target.rot).transpose());
			Eigen::Vector3d b = q * (s.ref.trans - rotation * s.target.trans);
			const size_t row = (family * count + i) * 3;
			coefficients.block<3, 3>(row, 0) = q;
			constants.segment<3>(row) = b;
			meanQ += q;
			meanB += b;
		}
		meanQ /= static_cast<double>(count);
		meanB /= static_cast<double>(count);
		for (size_t i = 0; i < count; ++i)
		{
			const size_t row = (family * count + i) * 3;
			coefficients.block<3, 3>(row, 0) -= meanQ;
			constants.segment<3>(row) -= meanB;
		}
	}
	// sum(i<j) ||e_i-e_j||^2 = n * sum(i) ||e_i-mean(e)||^2.
	// Center each frame family separately: this is the same least-squares
	// problem with 6n rows instead of 3n(n-1), without squaring its condition
	// number by forming normal equations.
	return coefficients.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(constants);
}

Eigen::AffineCompact3d CalibrationCalc::ComputeCalibration() const
{
	Eigen::Vector3d rotation = CalibrateRotation();
	Eigen::Matrix3d rotationMat = RotationFromEulerDeg(rotation);
	Eigen::Vector3d translation = CalibrateTranslation(rotationMat);

	Eigen::AffineCompact3d rot(rotationMat);
	Eigen::Translation3d trans(translation);

	return trans * rot;
}

double CalibrationCalc::RetargetingErrorRMS(
	const Eigen::Vector3d &hmdToTargetPos,
	const Eigen::AffineCompact3d &calibration) const
{
	double errorAccum = 0;
	int sampleCount = 0;

	for (auto &sample : m_samples)
	{
		if (!sample.valid) continue;

		const auto updatedPose = ApplyTransform(sample.target, calibration);
		const Eigen::Vector3d hmdPoseSpace = sample.ref.rot * hmdToTargetPos + sample.ref.trans;
		double error = (updatedPose.trans - hmdPoseSpace).squaredNorm();
		errorAccum += error;
		sampleCount++;
	}

	return sqrt(errorAccum / sampleCount);
}

Eigen::Vector3d CalibrationCalc::ComputeRefToTargetOffset(const Eigen::AffineCompact3d &calibration) const
{
	Eigen::Vector3d accum = Eigen::Vector3d::Zero();
	int sampleCount = 0;

	for (auto &sample : m_samples)
	{
		if (!sample.valid) continue;

		const auto updatedPose = ApplyTransform(sample.target, calibration);

		// Now move the transform from world to HMD space
		const auto hmdOriginPos = updatedPose.trans - sample.ref.trans;
		const auto hmdSpace = sample.ref.rot.inverse() * hmdOriginPos;

		accum += hmdSpace;
		sampleCount++;
	}

	accum /= sampleCount;

	return accum;
}

Eigen::Vector4d CalibrationCalc::ComputeAxisVariance() const
{
	// We want to determine if the user rotated in enough axis to find a unique solution.
	// It's sufficient to rotate in two axis - this is because once we constrain the mapping
	// of those two orthogonal basis vectors, the third is determined by the cross product of
	// those two basis vectors. So, the question we then have to answer is - after accounting for
	// translational movement of the HMD itself, are we too close to having only moved on a plane?

	// To determine this, we perform primary component analysis on the rotation quaternions themselves.
	// Since an angle axis quaternion is defined as the sum of Qidentity*cos(angle/2) + Qaxis*sin(angle/2),
	// we expect that rotations around a single axis will have two primary components: One corresponding
	// to the identity component, and one to the axis component. Thus, we check the variance (eigenvalue) of
	// the third primary component to see if we've moved in two axis.
	std::vector<Eigen::Vector4d> points;

	Eigen::Vector4d mean = Eigen::Vector4d::Zero();

	for (auto &sample : m_samples)
	{
		if (!sample.valid) continue;

		auto q = Eigen::Quaterniond(sample.target.rot);
		auto point = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
		mean += point;

		points.push_back(point);
	}
	mean /= (double)points.size();   // a full window, at least 100 samples

	Eigen::Matrix4d covMatrix = Eigen::Matrix4d::Zero();

	for (auto &point : points)
	{
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				covMatrix(i, j) += (point(i) - mean(i)) * (point(j) - mean(j));
	}
	covMatrix /= (double)points.size();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
	solver.compute(covMatrix);

	return solver.eigenvalues();
}

[[nodiscard]] bool CalibrationCalc::ValidateCalibration(const Eigen::AffineCompact3d &calibration, double *error)
{
	if (!calibration.matrix().allFinite())
		return false;

	bool ok = true;

	const auto posOffset = ComputeRefToTargetOffset(calibration);
	if (!posOffset.allFinite())
		return false;

	double rmsError = RetargetingErrorRMS(posOffset, calibration);
	if (!std::isfinite(rmsError) || rmsError > 0.1) ok = false;

	if (error) *error = rmsError;

	return ok;
}

// Given:
//   R - the reference pose (in reference world space)
//   T - the target pose (in target world space)
//   C - the true calibration (target world -> reference world)
// We assume that there is some "static target pose" S s.t.:
// R * S = C * T (we'll call this the static target pose)
// To compute S:
// S = R^-1 * C * T
// To compute C:
// R * S * T^-1 = C

// S = R^-1 * C * T
Eigen::AffineCompact3d CalibrationCalc::EstimateRefToTargetPose(const Eigen::AffineCompact3d &calibration) const
{
	auto avg = PoseAverager::AverageFor(m_samples, [&](const auto &sample) {
		return Eigen::Affine3d(sample.ref.ToAffine().inverse() * calibration * sample.target.ToAffine());
	});
	return avg;
}

// C = R * S * T^-1, from the estimated refToTargetPose: this works even when
// the devices are not moving.
bool CalibrationCalc::CalibrateByRelPose(Eigen::AffineCompact3d &out) const
{
	out = PoseAverager::AverageFor(m_samples, [&](const auto &sample) {
		return Eigen::AffineCompact3d(sample.ref.ToAffine() * m_refToTargetPose * sample.target.ToAffine().inverse());
	});

	return true;
}

bool CalibrationCalc::ComputeOneshot(const bool)
{
	auto calibration = ComputeCalibration();
	if (!ValidateCalibration(calibration, &m_lastError))
		return false;
	m_estimatedTransformation = calibration;
	m_isValid = true;
	return true;
}

bool CalibrationCalc::ComputeIncremental(bool &lerp, double threshold, double relPoseMaxError, const bool)
{
	if (lockRelativePosition)
	{
		Eigen::AffineCompact3d byRelPose;
		double relPoseError = INFINITY;
		if (CalibrateByRelPose(byRelPose) &&
			ValidateCalibration(byRelPose, &relPoseError))
		{
			m_isValid = true;
			m_estimatedTransformation = byRelPose;
			m_lastError = relPoseError;
			return true;
		}
	}

	// Only the error it writes is wanted here.
	double priorCalibrationError = INFINITY;
	if (m_isValid)
		(void)ValidateCalibration(m_estimatedTransformation, &priorCalibrationError);

	double newError = INFINITY;
	bool newCalibrationValid = false;
	Eigen::AffineCompact3d byRelPose;
	Eigen::AffineCompact3d calibration;
	bool usingRelPose = false;
	double relPoseError = INFINITY;

	if (enableStaticRecalibration && CalibrateByRelPose(byRelPose))
	{
		if (ValidateCalibration(byRelPose, &relPoseError))
		{
			if (relPoseError < 0.010 || (m_relativePosCalibrated && relPoseError < 0.025))
			{
				if (relPoseError * threshold >= priorCalibrationError)
					return false;

				if (relPoseError > relPoseMaxError)
					return false;

				newCalibrationValid = true;
				usingRelPose = true;
				newError = relPoseError;
				calibration = byRelPose;
			}
		}
	}

	double newVariance = 0;
	bool shouldRapidCorrect = true;
	if (!newCalibrationValid)
	{
		// Axis diversity is observable directly from the samples. Check it before
		// running either SVD: an initial low-diversity window has no prior
		// variance to compare against and would accept an arbitrary finite
		// calibration from jitter or single-axis motion.
		newVariance = ComputeAxisVariance()(1);
		if (!std::isfinite(newVariance) || newVariance < AxisVarianceThreshold)
		{
			newCalibrationValid = false;
			shouldRapidCorrect = false;
		}
		else
		{
			calibration = ComputeCalibration();
			newCalibrationValid = ValidateCalibration(calibration, &newError);

			if (m_isValid)
			{
				if (priorCalibrationError < newError * threshold)
				{
					// If we have a more noisy calibration than before, avoid updating.
					newCalibrationValid = false;
					shouldRapidCorrect = false;
				}
			}
		}
	}

	// Now, can we use the relative pose to perform a rapid correction?
	if (!newCalibrationValid && shouldRapidCorrect)
	{
		double existingPoseErrorUsingRelPosition = RetargetingErrorRMS(m_refToTargetPose.translation(), m_estimatedTransformation);
		if (relPoseError * threshold < existingPoseErrorUsingRelPosition)
		{
			newCalibrationValid = true;
			usingRelPose = true;
			newError = relPoseError;
			calibration = byRelPose;
		}
	}

	if (newCalibrationValid)
	{
		lerp = m_isValid;
		m_relativePosCalibrated = m_relativePosCalibrated || newError < 0.005;

		m_isValid = true;
		m_estimatedTransformation = calibration;
		m_axisVariance = newVariance;
		m_lastError = newError;

		if (!usingRelPose)
			m_refToTargetPose = EstimateRefToTargetPose(m_estimatedTransformation);

		return true;
	}
	else
	{
		return false;
	}
}

} // namespace legacy
} // namespace questcal
