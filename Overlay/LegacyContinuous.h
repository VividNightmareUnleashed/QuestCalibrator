#pragma once

// The "legacy" continuous-calibration method: hyblocker's CalibrationCalc from
// OpenVR-SpaceCalibrator (branch legacy-1-5-1), preserving its solve objective.
// It re-solves the whole calibration from a rolling window of reference and
// target poses (yaw-only Kabsch on rotation deltas, least-squares
// translation), validates it by retargeting error, and optionally refreshes
// it from the averaged relative pose when the pair is still. Kept alongside
// QuestCalibrator's own model so a player whose setup worked with the
// original can choose it, and so the two can be compared on the same data.
//
// The translation solve uses the equivalent centered system instead of all
// pairwise differences. Poses have no OpenVR types, the metrics sink is gone,
// and log lines go to an optional callback instead of a global context.

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <deque>
#include <functional>
#include <vector>

namespace questcal {
namespace legacy {

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(const Eigen::AffineCompact3d &transform)
	{
		rot = transform.rotation();
		trans = transform.translation();
	}
	Pose(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation)
		: rot(rotation.toRotationMatrix()), trans(translation) { }
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x, y, z)) { }

	Eigen::Matrix4d ToAffine() const
	{
		Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				matrix(i, j) = rot(i, j);
			matrix(i, 3) = trans(i);
		}
		return matrix;
	}
};

struct Sample
{
	Pose ref, target;
	bool valid;
	double timestamp;
	Sample() : valid(false), timestamp(0) { }
	Sample(Pose ref, Pose target, double timestamp) : ref(ref), target(target), valid(true), timestamp(timestamp) { }
};

class CalibrationCalc
{
public:
	static const double AxisVarianceThreshold;

	bool enableStaticRecalibration;
	bool lockRelativePosition = false;

	// Where the source wrote to the application log.
	std::function<void(const char *)> log;

	const Eigen::AffineCompact3d Transformation() const { return m_estimatedTransformation; }

	const Eigen::Vector3d EulerRotation() const
	{
		auto rot = m_estimatedTransformation.rotation();
		return rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
	}

	bool isValid() const { return m_isValid; }

	const Eigen::AffineCompact3d RelativeTransformation() const { return m_refToTargetPose; }
	bool isRelativeTransformationCalibrated() const { return m_relativePosCalibrated; }

	void setRelativeTransformation(const Eigen::AffineCompact3d transform, bool calibrated)
	{
		m_refToTargetPose = transform;
		m_relativePosCalibrated = calibrated;
	}

	void PushSample(const Sample &sample);
	void Clear();

	double ReferenceJitter() const;
	double TargetJitter() const;

	bool ComputeOneshot(const bool ignoreOutliers);
	bool ComputeIncremental(bool &lerp, double threshold, double relPoseMaxError, const bool ignoreOutliers);

	size_t SampleCount() const { return m_samples.size(); }

	void ShiftSample()
	{
		if (!m_samples.empty())
			m_samples.pop_front();
	}

	CalibrationCalc() : enableStaticRecalibration(true), m_calcCycle(0), m_isValid(false) { }

	// Debug fields
	Eigen::Vector3d m_posOffset;
	double m_axisVariance = 0.0;
	long m_calcCycle;
	// The retargeting error of the last accepted estimate (meters).
	double m_lastError = 0.0;

private:
	bool m_isValid;
	Eigen::AffineCompact3d m_estimatedTransformation;
	bool m_relativePosCalibrated = false;

	/*
	 * This affine transform estimates the pose of the target within the reference device's local pose space.
	 * That is to say, it's given by transforming the target world pose by the inverse reference pose.
	 */
	Eigen::AffineCompact3d m_refToTargetPose = Eigen::AffineCompact3d::Identity();

	std::deque<Sample> m_samples;

	std::vector<bool> DetectOutliers() const;
	Eigen::Vector3d CalibrateRotation(const bool ignoreOutliers) const;
	Eigen::Vector3d CalibrateTranslation(const Eigen::Matrix3d &rotation) const;

	Eigen::AffineCompact3d ComputeCalibration(const bool ignoreOutliers) const;

	double RetargetingErrorRMS(const Eigen::Vector3d &hmdToTargetPos, const Eigen::AffineCompact3d &calibration) const;
	Eigen::Vector3d ComputeRefToTargetOffset(const Eigen::AffineCompact3d &calibration) const;

	Eigen::Vector4d ComputeAxisVariance(const Eigen::AffineCompact3d &calibration) const;

	[[nodiscard]] bool ValidateCalibration(const Eigen::AffineCompact3d &calibration, double *errorOut = nullptr, Eigen::Vector3d *posOffsetV = nullptr);

	Eigen::AffineCompact3d EstimateRefToTargetPose(const Eigen::AffineCompact3d &calibration) const;
	bool CalibrateByRelPose(Eigen::AffineCompact3d &out) const;
};

// The left delta D with newCal = D o oldCal, which is how the overlay applies
// a re-solved calibration: every consumer of the base transform (anchors,
// chaperone, driver slew) already speaks deltas.
inline void DeltaBetweenCalibrations(
	const Eigen::Quaterniond &oldRotation, const Eigen::Vector3d &oldTranslation,
	const Eigen::Quaterniond &newRotation, const Eigen::Vector3d &newTranslation,
	Eigen::Quaterniond &deltaRotation, Eigen::Vector3d &deltaTranslation)
{
	deltaRotation = (newRotation * oldRotation.conjugate()).normalized();
	deltaTranslation = newTranslation - deltaRotation * oldTranslation;
}

} // namespace legacy
} // namespace questcal
