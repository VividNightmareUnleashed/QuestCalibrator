#pragma once

#include "../common/NumericValidation.h"
#include "../common/Protocol.h"
#include "../common/TransformLimits.h"

#include <cmath>
#include <cstdint>

// The accept-or-drop gate a raw ring sample clears before anything composes
// it. Kept free of Eigen, unlike RingPoseMath.h which includes it, so the
// model checker in VirtualQuest/formal/input-validation (which cannot parse
// Eigen) checks this code as written.

// The per-field checks are the wire vocabulary shared with the driver's inbound
// gate (common/NumericValidation.h). IsAcceptableQuaternion accepts or drops;
// the driver's sanitizing NormalizeQuaternion would turn a huge finite
// quaternion into accepted data instead of tracking absence.
using questcal::numeric::IsAcceptableQuaternion;
using questcal::numeric::IsBoundedVector3;

// A driver can mark a pose Running_OK while still supplying malformed numeric
// fields. Validate the complete raw sample before any solver or runtime monitor
// composes it; invalid samples are treated as tracking absence. qpcToSeconds is
// the positive QPC period.
inline bool IsUsableRingSample(const protocol::DevicePoseSample &s, double qpcToSeconds)
{
	if (!std::isfinite(s.poseTimeOffset) ||
		std::abs(s.poseTimeOffset) > protocol::limits::MaxAbsTimeOffsetSeconds ||
		!IsAcceptableQuaternion(s.worldFromDriverRotation) ||
		!IsAcceptableQuaternion(s.rotation) ||
		!IsBoundedVector3(s.worldFromDriverTranslation,
			protocol::limits::MaxAbsTranslationMeters) ||
		!IsBoundedVector3(s.position, protocol::limits::MaxAbsPosePositionMeters) ||
		!IsBoundedVector3(s.velocity,
			protocol::limits::MaxAbsLinearVelocityMetersPerSecond) ||
		!IsBoundedVector3(s.angularVelocity,
			protocol::limits::MaxAbsAngularVelocityRadiansPerSecond))
		return false;

	double composedTime = static_cast<double>(s.sampleTimeQpc) * qpcToSeconds + s.poseTimeOffset;
	return std::abs(composedTime) <= protocol::limits::MaxAbsPoseTimestampSeconds;
}

// The complete overlay-side trust boundary for one ring sample: the driver must
// claim the pose is valid AND that the device is actually tracking, and the
// numeric fields must survive validation. Every consumer (chaperone baseline,
// collector, runtime monitor, continuous loop) asks this one question.
inline bool IsTrustedRingSample(
	const protocol::DevicePoseSample &s, double qpcToSeconds)
{
	return s.deviceIsConnected && s.poseIsValid &&
		s.trackingResult == static_cast<uint32_t>(vr::TrackingResult_Running_OK) &&
		IsUsableRingSample(s, qpcToSeconds);
}
