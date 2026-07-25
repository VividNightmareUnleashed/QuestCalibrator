#pragma once

// Overlay-side mirror of the driver's spatial-correction-field blend
// (Driver/AlignmentField.cpp BlendAt), over the overlay's absolute anchors.
//
// The continuous-calibration loop needs the transform the field actually
// produces at the mounted tracker's spot: with anchors present the true local
// alignment legitimately differs from the base calibration, so measuring the
// windowed estimate against raw base would read every anchor's own delta as a
// "deviation" — freezing on it near large anchors and, below the freeze
// threshold, emitting corrections that drag the base (and with it every
// anchor) toward one spot's local deformation. Blending here and comparing
// against delta_blend o base keeps continuous corrections orthogonal to the
// field: only genuinely global movement of the universe produces a deviation.
//
// Applying a resulting correction D through ApplyAlignmentDelta stays exact:
// it shifts the base AND the absolute anchors by D, so the blended expectation
// itself moves by exactly D (delta' = D delta D^-1, expected' = D o expected).
//
// Pure Eigen so the synthetic test harness compiles exactly this code and
// checks it against alignfield::BlendAt. The constants mirror the driver's:
// protocol::SetAlignmentField::sigmaMeters default and
// alignfield::IdentityFloorWeight (asserted equal in the tests).

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace questcal
{

constexpr double FieldBlendSigmaMeters = 1.5;
constexpr double FieldBlendIdentityFloor = 0.05;

// Expected local calibration delta_blend(queryBasePos) o base. Anchors carry
// absolute per-spot solves (.position in reference space, .rotation /
// .translationMeters the absolute transform); deltas against base are derived
// here exactly as SendAlignmentField ships them. With zero anchors the result
// is the base calibration itself.
template <class AnchorVec>
inline void BlendedFieldCalibration(const AnchorVec &anchors,
                                    const Eigen::Quaterniond &baseRot,
                                    const Eigen::Vector3d &baseTrans,
                                    const Eigen::Vector3d &queryBasePos,
                                    Eigen::Quaterniond &rotOut,
                                    Eigen::Vector3d &transOut)
{
	double wSum = FieldBlendIdentityFloor;
	Eigen::Vector4d q(0.0, 0.0, 0.0, FieldBlendIdentityFloor);   // (x, y, z, w)
	Eigen::Vector3d t = Eigen::Vector3d::Zero();

	constexpr double invTwoSigmaSq =
		1.0 / (2.0 * FieldBlendSigmaMeters * FieldBlendSigmaMeters);

	Eigen::Quaterniond baseInv = baseRot.conjugate();
	for (const auto &a : anchors)
	{
		double dx = queryBasePos.x() - a.position.x();
		double dz = queryBasePos.z() - a.position.z();
		double w = std::exp(-(dx * dx + dz * dz) * invTwoSigmaSq);

		// delta_i = anchor_i o base^-1, the same derivation SendAlignmentField
		// ships to the driver.
		Eigen::Quaterniond dR = (a.rotation * baseInv).normalized();
		Eigen::Vector3d dT = a.translationMeters - dR * baseTrans;

		// Hemisphere alignment: deltas are small, w >= 0 is the short way.
		Eigen::Vector4d qa = dR.coeffs();
		if (qa(3) < 0.0)
			qa = -qa;

		q += w * qa;
		t += w * dT;
		wSum += w;
	}

	q.normalize();
	Eigen::Quaterniond dBlend(q(3), q(0), q(1), q(2));
	Eigen::Vector3d tBlend = t / wSum;

	rotOut = (dBlend * baseRot).normalized();
	transOut = dBlend * baseTrans + tBlend;
}

} // namespace questcal
