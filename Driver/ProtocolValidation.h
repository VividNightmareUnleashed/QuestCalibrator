#pragma once

#include "../common/Protocol.h"
#include "../common/TransformLimits.h"

#include <algorithm>
#include <cmath>

// Pure validation/sanitization for messages crossing from the overlay process
// into vrserver. Setters publish only the returned copy, so a malformed request
// cannot partially replace a previously good transform.
namespace questcal
{
namespace driverinput
{

inline bool IsFiniteBounded(double value, double maxAbs)
{
	return std::isfinite(value) && std::abs(value) <= maxAbs;
}

inline bool IsFiniteVector(const double (&value)[3], double maxAbs)
{
	return IsFiniteBounded(value[0], maxAbs)
		&& IsFiniteBounded(value[1], maxAbs)
		&& IsFiniteBounded(value[2], maxAbs);
}

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

inline bool ValidateAndSanitize(const protocol::SetDeviceTransform &input,
	protocol::SetDeviceTransform &output)
{
	if (input.openVRID >= vr::k_unMaxTrackedDeviceCount)
		return false;
	if (input.enabled > 1 || input.hidden > 1)
		return false;
	if (!IsFiniteVector(input.translation.v, protocol::limits::MaxAbsTranslationMeters))
		return false;
	if (!std::isfinite(input.scale) || input.scale < protocol::limits::MinScale ||
		input.scale > protocol::limits::MaxScale)
		return false;
	if (!IsFiniteBounded(input.timeOffset, protocol::limits::MaxAbsTimeOffsetSeconds))
		return false;

	output = input;
	return NormalizeQuaternion(output.rotation);
}

inline bool ValidateAndSanitize(const protocol::SetAlignmentField &input,
	protocol::SetAlignmentField &output)
{
	if (input.enabled > 1)
		return false;
	if (input.anchorCount > protocol::SetAlignmentField::MaxAnchors)
		return false;
	if (!std::isfinite(input.sigmaMeters)
		|| input.sigmaMeters < protocol::limits::MinFieldSigmaMeters
		|| input.sigmaMeters > protocol::limits::MaxFieldSigmaMeters)
		return false;

	// Value-initialize unused anchors rather than copying untrusted bytes that
	// are outside anchorCount. This keeps every stored snapshot numerically sane.
	output = protocol::SetAlignmentField{};
	output.enabled = input.enabled;
	output.generation = input.generation;
	output.anchorCount = input.anchorCount;
	output.sigmaMeters = input.sigmaMeters;

	for (uint32_t i = 0; i < input.anchorCount; ++i)
	{
		const protocol::FieldAnchor &source = input.anchors[i];
		protocol::FieldAnchor &destination = output.anchors[i];
		if (!IsFiniteVector(source.position, protocol::limits::MaxAbsAnchorPositionMeters)
			|| !IsFiniteVector(source.translationDelta, protocol::limits::MaxAbsAnchorDeltaMeters))
			return false;

		destination = source;
		if (!NormalizeQuaternion(destination.rotationDelta))
			return false;
	}

	return true;
}

} // namespace driverinput
} // namespace questcal
