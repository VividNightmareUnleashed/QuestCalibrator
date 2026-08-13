#pragma once

#include "Protocol.h"
#include "TransformLimits.h"

#include <algorithm>
#include <cmath>

// The numeric vocabulary both processes validate wire values with, over the two
// C-array-shaped types the protocol carries (`double[3]` and the OpenVR
// quaternion). TransformLimits.h centralized the BOUNDS; the predicates that
// consume them stayed behind in Driver/ProtocolValidation.h and
// Overlay/RingPoseMath.h as two independent spellings, so a bound added next
// door reached two call sites written two different ways. They live here so a
// new bound reaches one.
//
// Overlay/ProfileValidation.h holds a third, Eigen-typed set over the same
// bounds and deliberately stays where it is: merging it would force Eigen into
// vrserver. Its verdicts already match IsAcceptableQuaternion /
// IsBoundedVector3 value for value (IsValidRotation and IsBoundedVector), so
// the divergence is one of types, not of policy.
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

// The two quaternion policies below are NOT one function, and merging them
// would change what one of the two gates decides.
//
// NormalizeQuaternion is the pipe-into-vrserver gate: it SANITIZES and then
// publishes, so it scales by the largest component before forming the norm and
// a huge-but-finite wire quaternion is accepted as its unit direction. A
// caller has already decided this message is worth applying; the job here is to
// make sure what gets stored is a unit quaternion.
//
// IsAcceptableQuaternion is the ring-into-overlay gate: it ACCEPTS OR DROPS,
// and a rejected sample is tracking absence rather than data. It forms the
// squared norm directly, so the same huge-but-finite quaternion overflows and
// the sample is dropped - which is the conservative answer for a monitor that
// would otherwise feed a fabricated pose to the solver. The component-magnitude
// floor (MinQuaternionComponentNorm) and the squared-norm floor (1e-12) are
// likewise not the same threshold at the boundary, and both are load-bearing
// where they are.
inline bool NormalizeQuaternion(vr::HmdQuaternion_t &value)
{
	if (!std::isfinite(value.w) || !std::isfinite(value.x)
		|| !std::isfinite(value.y) || !std::isfinite(value.z))
		return false;

	double maxComponent = std::max(std::max(std::abs(value.w), std::abs(value.x)),
		std::max(std::abs(value.y), std::abs(value.z)));
	if (maxComponent < protocol::limits::MinQuaternionComponentNorm)
		return false;

	// Scale before taking the norm so even very large finite wire values cannot
	// overflow the check. The stored quaternion is always unit length.
	double w = value.w / maxComponent;
	double x = value.x / maxComponent;
	double y = value.y / maxComponent;
	double z = value.z / maxComponent;
	double norm = std::sqrt(w * w + x * x + y * y + z * z);
	if (!std::isfinite(norm) || norm <= 0.0)
		return false;

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
