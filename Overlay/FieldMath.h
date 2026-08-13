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
// checks it against alignfield::BlendAt. That purity is why this stays a
// mirror rather than a thin adapter over the driver's blend: an adapter would
// have to build a protocol::SetAlignmentField, pulling protocol and OpenVR
// types into a layer that is deliberately dependency-free. The cost of the
// mirror is that the blend width must be passed in rather than assumed - see
// BlendedFieldCalibration.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace questcal
{

constexpr double FieldBlendSigmaMeters = 1.5;
constexpr double FieldBlendIdentityFloor = 0.05;

// The per-anchor delta the driver blends: delta_i = anchor_i o base^-1, the
// correction that, applied after the base calibration, reproduces the absolute
// solve at that anchor's spot. SINGLE SOURCE for the derivation:
// SendAlignmentField ships exactly this, BlendedFieldCalibration mirrors it,
// and the test harness builds reference fields from it — the continuous loop's
// "corrections orthogonal to the field" invariant needs all three bit-equal.
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
// .translationMeters the absolute transform); deltas against base are derived
// here exactly as SendAlignmentField ships them. With zero anchors the result
// is the base calibration itself.
// sigmaMeters must be the value the driver is blending with, i.e. the one
// SendAlignmentField put on the wire - not merely the default that happens to
// match it. The wire field is real, per-message and range-validated, so the
// moment sigma becomes configurable a hard-coded constant here would give the
// driver one field shape and the continuous loop's expectation another, with
// nothing failing: healthy anchor gradients would then read as universe
// deviation, which is the exact failure the field-blended comparison exists to
// prevent. Passing it in keeps the two blends coupled by construction.
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
	Eigen::Vector3d t = Eigen::Vector3d::Zero();

	// Mirrors the driver's clamp (AlignmentField.cpp): a non-positive or
	// absurd sigma must not divide by ~zero here either.
	double sigma = sigmaMeters > 0.01 ? sigmaMeters : 1.5;
	const double invTwoSigmaSq = 1.0 / (2.0 * sigma * sigma);

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
