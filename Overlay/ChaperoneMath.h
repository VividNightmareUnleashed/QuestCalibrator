#pragma once

// Chaperone snapshot math, kept free of OpenVR *calls* (types only) so the
// synthetic test harness can verify the row-major HmdMatrix34_t conventions.
//
// Frames: collision-bound quads and the play-area rect are expressed relative
// to the standing/play-area center (openvr.h: "Tracking space center (0,0,0)
// is the center of the Play Area"), while standingZeroPoseToRawTrackingPose
// maps that frame into the raw tracking universe. A universe jump re-bases
// the raw frame only, so the quads are jump-invariant and just the standing
// center pose has to be re-anchored.

#include <Eigen/Core>
#include <Eigen/Geometry>

// The flattened OpenVR headers both define the vr types but cannot coexist in
// one TU. The harness (PCH-free) reaches openvr_driver.h through Protocol.h,
// so this include must stay conditional.
#if !defined(_OPENVR_API) && !defined(_OPENVR_DRIVER_API)
#include <openvr.h>
#endif

#include <cmath>
#include <vector>

#include "PoseMath.h"
#include "ProfileValidation.h"
#include "../common/TransformLimits.h"

namespace questcal
{

// The gravity-preserving (yaw + translation) raw-universe delta between two
// worldFromDriver transforms, which come from trusted ring samples or a
// validated snapshot. Fails when the delta's translation is out of bounds.
inline bool WorldFromDriverDelta(
	const Eigen::Quaterniond &oldRotation, const Eigen::Vector3d &oldTranslation,
	const Eigen::Quaterniond &newRotation, const Eigen::Vector3d &newTranslation,
	Eigen::Quaterniond &deltaRotation, Eigen::Vector3d &deltaTranslation)
{
	Eigen::Quaterniond fullDelta =
		(newRotation.normalized() * oldRotation.normalized().conjugate()).normalized();
	Eigen::Quaterniond yaw = YawOnlyRotation(fullDelta);

	deltaRotation = yaw;
	deltaTranslation = newTranslation - yaw * oldTranslation;
	return IsBoundedVector(deltaTranslation, protocol::limits::MaxAbsTranslationMeters);
}


// Left-compose a rigid universe delta (R, T) onto a standing-center pose:
// out = [R|T] * m, i.e. the same physical pose expressed in the post-jump
// raw frame.
inline vr::HmdMatrix34_t DeltaTimesPose(
	const Eigen::Quaterniond &R, const Eigen::Vector3d &T, const vr::HmdMatrix34_t &m)
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			rot(i, j) = m.m[i][j];
		trans(i) = m.m[i][3];
	}

	Eigen::Matrix3d outRot = R.toRotationMatrix() * rot;
	Eigen::Vector3d outTrans = R * trans + T;

	vr::HmdMatrix34_t out;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			out.m[i][j] = static_cast<float>(outRot(i, j));
		out.m[i][3] = static_cast<float>(outTrans(i));
	}
	return out;
}

// Corner-wise geometry comparison with a tolerance: the snapshot round-trips
// through JSON and SteamVR's live copy, so exact float equality is too strict.
// A non-finite corner (SteamVR's live copy is unchecked) never matches.
inline bool QuadsMatch(const std::vector<vr::HmdQuad_t> &a,
	const std::vector<vr::HmdQuad_t> &b, float epsMeters)
{
	if (a.size() != b.size())
		return false;

	for (size_t i = 0; i < a.size(); ++i)
		for (int c = 0; c < 4; ++c)
			for (int k = 0; k < 3; ++k)
			{
				float x = a[i].vCorners[c].v[k];
				float y = b[i].vCorners[c].v[k];
				if (!std::isfinite(x) || !std::isfinite(y) ||
					std::fabs(x - y) > epsMeters)
					return false;
			}

	return true;
}

// Broad trust-boundary validation for persisted/live room geometry. Values
// are intentionally generous enough for warehouses, while rejecting FLT_MAX
// payloads and non-rigid standing transforms before they can be auto-committed.
inline bool IsPlausibleChaperone(const std::vector<vr::HmdQuad_t> &geometry,
	const vr::HmdMatrix34_t &standingCenter, const vr::HmdVector2_t &playSpaceSize)
{
	for (float size : playSpaceSize.v)
		if (!std::isfinite(size) || size <= 0.0f ||
			size > protocol::limits::MaxPlayAreaSizeMeters)
			return false;

	Eigen::Matrix3d basis;
	for (int row = 0; row < 3; ++row)
	{
		for (int column = 0; column < 3; ++column)
		{
			float value = standingCenter.m[row][column];
			if (!std::isfinite(value) || std::abs(value) > 2.0f)
				return false;
			basis(row, column) = value;
		}
		float translation = standingCenter.m[row][3];
		if (!std::isfinite(translation) ||
			std::abs(translation) > protocol::limits::MaxAbsChaperoneCoordinateMeters)
			return false;
	}
	Eigen::Matrix3d orthogonality = basis.transpose() * basis;
	if ((orthogonality - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() >
		protocol::limits::MaxStandingBasisError ||
		std::abs(basis.determinant() - 1.0) > protocol::limits::MaxStandingBasisError)
		return false;

	for (const auto &quad : geometry)
		for (const auto &corner : quad.vCorners)
			for (float value : corner.v)
				if (!std::isfinite(value) ||
					std::abs(value) > protocol::limits::MaxAbsChaperoneCoordinateMeters)
					return false;
	return true;
}

} // namespace questcal
