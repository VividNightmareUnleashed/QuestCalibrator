#pragma once

#include "Protocol.h"
#include "TransformLimits.h"

#include <algorithm>
#include <cmath>

// The predicates both processes validate wire values with, over the protocol's
// `double[3]` and OpenVR quaternion types, against the TransformLimits.h
// bounds. Overlay/ProfileValidation.h has the Eigen-typed equivalents (kept
// separate so vrserver needs no Eigen).
namespace questcal
{
namespace numeric
{

inline bool IsFiniteBounded(double value, double maxAbs)
{
	return std::isfinite(value) && std::abs(value) <= maxAbs;
}

inline bool IsBoundedVector3(const double (&value)[3], double maxAbs)
{
	return IsFiniteBounded(value[0], maxAbs)
		&& IsFiniteBounded(value[1], maxAbs)
		&& IsFiniteBounded(value[2], maxAbs);
}

// The two quaternion gates differ on purpose; do not merge them.
// NormalizeQuaternion (pipe into vrserver) sanitizes: it scales by the largest
// component first, so a huge-but-finite quaternion is stored as its unit
// direction. IsAcceptableQuaternion (ring into overlay) accepts or drops: the
// same quaternion overflows its squared norm and the sample is dropped rather
// than fed to the solver. Their near-zero floors differ too.
inline bool NormalizeQuaternion(vr::HmdQuaternion_t &value)
{
	if (!std::isfinite(value.w) || !std::isfinite(value.x)
		|| !std::isfinite(value.y) || !std::isfinite(value.z))
		return false;

	double maxComponent = std::max(std::max(std::abs(value.w), std::abs(value.x)),
		std::max(std::abs(value.y), std::abs(value.z)));
	if (maxComponent < protocol::limits::MinQuaternionComponentNorm)
		return false;

	// After this scaling one component is exactly +/-1, so the norm is in [1, 2].
	double w = value.w / maxComponent;
	double x = value.x / maxComponent;
	double y = value.y / maxComponent;
	double z = value.z / maxComponent;
	double norm = std::sqrt(w * w + x * x + y * y + z * z);
	value = { w / norm, x / norm, y / norm, z / norm };
	return true;
}

inline bool IsAcceptableQuaternion(const vr::HmdQuaternion_t &q)
{
	if (!std::isfinite(q.w) || !std::isfinite(q.x) ||
		!std::isfinite(q.y) || !std::isfinite(q.z))
		return false;
	double normSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
	return std::isfinite(normSq) && normSq > 1e-12;
}

} // namespace numeric
} // namespace questcal
