#pragma once

// Overlay-side mirror of the driver's spatial-correction-field blend
// (Driver/AlignmentField.cpp BlendAt), over the overlay's absolute anchors.
// Pure Eigen, so the harness checks it against alignfield::BlendAt.
//
// The continuous-calibration loop compares its estimate against the transform
// the field actually produces at the mounted tracker's spot: against the raw
// base it would read every anchor's own delta as a deviation and drag the base
// toward one spot's local deformation. Comparing against delta_blend o base
// keeps continuous corrections orthogonal to the field.
//
// A yaw/translation correction D (ApplyCalibrationDelta) shifts the base AND
// the absolute anchors by D, so the blended expectation moves by exactly D
// (delta' = D delta D^-1, expected' = D o expected).

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace questcal
{

constexpr double FieldBlendSigmaMeters = 1.5;
constexpr double FieldBlendIdentityFloor = 0.05;

// The per-anchor delta the driver blends: delta_i = anchor_i o base^-1, the
// correction that, applied after the base calibration, reproduces the absolute
// solve at that anchor's spot. The single source for SendAlignmentField,
// BlendedFieldCalibration and the harness, which must agree bit for bit.
inline void AnchorDelta(const Eigen::Quaterniond &anchorRot,
                        const Eigen::Vector3d &anchorTrans,
                        const Eigen::Quaterniond &baseRotInv,
                        const Eigen::Vector3d &baseTrans,
                        Eigen::Quaterniond &dRot, Eigen::Vector3d &dTrans)
{
	dRot = (anchorRot * baseRotInv).normalized();
	dTrans = anchorTrans - dRot * baseTrans;
}

// Expected local calibration delta_blend(queryBasePos) o base. Anchors carry
// absolute per-spot solves (.position in reference space, .rotation /
// .translationMeters the absolute transform). With zero anchors the result is
// the base calibration itself. sigmaMeters must be the width
// SendAlignmentField put on the wire, which the driver blends with.
template <class AnchorVec>
inline void BlendedFieldCalibration(const AnchorVec &anchors,
                                    const Eigen::Quaterniond &baseRot,
                                    const Eigen::Vector3d &baseTrans,
                                    const Eigen::Vector3d &queryBasePos,
                                    Eigen::Quaterniond &rotOut,
                                    Eigen::Vector3d &transOut,
                                    double sigmaMeters = FieldBlendSigmaMeters)
{
	double wSum = FieldBlendIdentityFloor;
	Eigen::Vector4d q(0.0, 0.0, 0.0, FieldBlendIdentityFloor);   // (x, y, z, w)
	Eigen::Vector3d p = FieldBlendIdentityFloor * queryBasePos;

	const double invTwoSigmaSq = 1.0 / (2.0 * sigmaMeters * sigmaMeters);

	Eigen::Quaterniond baseInv = baseRot.conjugate();
	for (const auto &a : anchors)
	{
		double dx = queryBasePos.x() - a.position.x();
		double dz = queryBasePos.z() - a.position.z();
		double w = std::exp(-(dx * dx + dz * dz) * invTwoSigmaSq);

		Eigen::Quaterniond dR;
		Eigen::Vector3d dT;
		AnchorDelta(a.rotation, a.translationMeters, baseInv, baseTrans, dR, dT);

		// Hemisphere alignment: deltas are small, w >= 0 is the short way.
		Eigen::Vector4d qa = dR.coeffs();
		if (qa(3) < 0.0)
			qa = -qa;

		q += w * qa;
		p += w * (dR * queryBasePos + dT);
		wSum += w;
	}

	q.normalize();
	Eigen::Quaterniond dBlend(q(3), q(0), q(1), q(2));
	Eigen::Vector3d tBlend = p / wSum - dBlend * queryBasePos;

	rotOut = (dBlend * baseRot).normalized();
	transOut = dBlend * baseTrans + tBlend;
}

} // namespace questcal
