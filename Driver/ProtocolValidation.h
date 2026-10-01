#pragma once

#include "../common/NumericValidation.h"
#include "../common/Protocol.h"
#include "../common/TransformLimits.h"

#include <cmath>

// Pure validation/sanitization for messages crossing from the overlay process
// into vrserver: which field is checked against which bound (predicates in
// common/NumericValidation.h). Setters publish only the returned copy, so a
// malformed request cannot partially replace a previously good transform.
namespace questcal
{
namespace driverinput
{

using questcal::numeric::IsBoundedVector3;
using questcal::numeric::IsFiniteBounded;
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

// Keep these scalar assignments explicit: the formal checker expands aggregate
// copies containing arrays into costly byte-level operations.
inline void CopyFrameCorrectionScalars(const protocol::FrameCorrection &input,
	protocol::FrameCorrection &output)
{
	output.rotation.w = input.rotation.w;
	output.rotation.x = input.rotation.x;
	output.rotation.y = input.rotation.y;
	output.rotation.z = input.rotation.z;
	output.translation.v[0] = input.translation.v[0];
	output.translation.v[1] = input.translation.v[1];
	output.translation.v[2] = input.translation.v[2];
}

inline bool ValidateAndSanitize(const protocol::FrameCorrection &input,
	protocol::FrameCorrection &output)
{
	protocol::FrameCorrection clean;
	CopyFrameCorrectionScalars(input, clean);
	if (!IsBoundedVector3(clean.translation.v, protocol::limits::MaxAbsTranslationMeters) ||
		!NormalizeQuaternion(clean.rotation))
		return false;
	CopyFrameCorrectionScalars(clean, output);
	return true;
}

inline bool ValidateFrameCorrections(
	const protocol::FrameCorrection (&input)[vr::k_unMaxTrackedDeviceCount],
	protocol::FrameCorrection (&output)[vr::k_unMaxTrackedDeviceCount])
{
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		if (!ValidateAndSanitize(input[id], output[id]))
			return false;
	return true;
}

inline void CopyFrameCorrections(
	const protocol::FrameCorrection (&input)[vr::k_unMaxTrackedDeviceCount],
	protocol::FrameCorrection (&output)[vr::k_unMaxTrackedDeviceCount])
{
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		CopyFrameCorrectionScalars(input[id], output[id]);
}

inline void CopyFrameSerialKeys(const uint64_t (&input)[vr::k_unMaxTrackedDeviceCount],
	uint64_t (&output)[vr::k_unMaxTrackedDeviceCount])
{
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) output[id] = input[id];
}

// The post-validation publication copy has its own direct proof. Explicit
// scalar assignments avoid the checker's expensive byte-level array copies.
inline void CopyAlignmentField(const protocol::SetAlignmentField &input,
	protocol::SetAlignmentField &output)
{
	output.enabled = input.enabled;
	output.generation = input.generation;
	output.anchorCount = input.anchorCount;
	output.sigmaMeters = input.sigmaMeters;
	for (uint32_t i = 0; i < protocol::SetAlignmentField::MaxAnchors; ++i)
	{
		const auto &source = input.anchors[i];
		auto &destination = output.anchors[i];
		for (uint32_t component = 0; component < 3; ++component)
		{
			destination.position[component] = source.position[component];
			destination.translationDelta[component] = source.translationDelta[component];
		}
		destination.rotationDelta.w = source.rotationDelta.w;
		destination.rotationDelta.x = source.rotationDelta.x;
		destination.rotationDelta.y = source.rotationDelta.y;
		destination.rotationDelta.z = source.rotationDelta.z;
	}
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

	protocol::FrameCorrection frames[vr::k_unMaxTrackedDeviceCount];
	if (!ValidateFrameCorrections(input.frames, frames))
		return false;
	output.enabledMask = input.enabledMask;
	output.hiddenMask = input.hiddenMask;
	output.transform = transform;
	CopyAlignmentField(field, output.field);
	output.frameProfileKey = input.frameProfileKey;
	output.expectedSessionId = input.expectedSessionId;
	CopyFrameSerialKeys(input.frameSerialKeys, output.frameSerialKeys);
	CopyFrameCorrections(frames, output.frames);
	return true;
}

} // namespace driverinput
} // namespace questcal
