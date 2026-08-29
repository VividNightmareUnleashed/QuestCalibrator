#pragma once

#include "../common/NumericValidation.h"
#include "../common/Protocol.h"
#include "../common/TransformLimits.h"

#include <cmath>

// Pure validation/sanitization for messages crossing from the overlay process
// into vrserver. Setters publish only the returned copy, so a malformed request
// cannot partially replace a previously good transform.
//
// The finite/bounded/normalize predicates themselves live in
// common/NumericValidation.h alongside the bounds they read, because the ring
// gate in the other direction (Overlay/RingPoseMath.h) asks the same questions
// of the same two wire types. What stays here is the message shape: which field
// is checked against which bound, and the transactional copy-out.
namespace questcal
{
namespace driverinput
{

using questcal::numeric::IsBoundedVector3;
using questcal::numeric::IsFiniteBounded;
// The sanitize-and-publish half of the shared quaternion pair; see
// NumericValidation.h for why the ring's accept-or-drop half is not this one.
using questcal::numeric::NormalizeQuaternion;

inline bool ValidateAndSanitize(const protocol::SetDeviceTransform &input,
	protocol::SetDeviceTransform &output)
{
	if (input.openVRID >= vr::k_unMaxTrackedDeviceCount)
		return false;
	if (input.enabled > 1 || input.hidden > 1)
		return false;
	if (!IsBoundedVector3(input.translation.v, protocol::limits::MaxAbsTranslationMeters))
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
		if (!IsBoundedVector3(source.position, protocol::limits::MaxAbsAnchorPositionMeters)
			|| !IsBoundedVector3(source.translationDelta, protocol::limits::MaxAbsAnchorDeltaMeters))
			return false;

		destination = source;
		if (!NormalizeQuaternion(destination.rotationDelta))
			return false;
	}

	return true;
}

inline bool ValidateAndSanitize(const protocol::SetRuntimeState &input,
	protocol::SetRuntimeState &output)
{
	if ((input.hiddenMask & ~input.enabledMask) != 0 ||
		input.transform.openVRID != 0 || input.transform.enabled != 1 ||
		input.transform.hidden != 0)
		return false;

	protocol::SetDeviceTransform transform;
	protocol::SetAlignmentField field;
	if (!ValidateAndSanitize(input.transform, transform) ||
		!ValidateAndSanitize(input.field, field))
		return false;
	if (field.enabled != 0 && input.enabledMask == 0)
		return false;

	output = protocol::SetRuntimeState{};
	output.enabledMask = input.enabledMask;
	output.hiddenMask = input.hiddenMask;
	output.transform = transform;
	output.field = field;
	return true;
}

} // namespace driverinput
} // namespace questcal
