#pragma once

#include "../common/TransformLimits.h"

#include <cmath>

// The scalar field bounds of a persisted profile. Kept free of Eigen, unlike
// ProfileValidation.h which includes it, so the model checker in
// VirtualQuest/formal/input-validation (which cannot parse Eigen) checks this
// code as written.
namespace questcal
{

inline bool IsValidScale(double scale)
{
	// The solver normally stays within [0.85, 1.15]. This wider range
	// preserves intentional manual edits while rejecting destructive input.
	return std::isfinite(scale) && scale >= protocol::limits::MinScale &&
		scale <= protocol::limits::MaxScale;
}

inline bool IsValidResidual(double value)
{
	return std::isfinite(value) && value >= 0.0;
}

inline bool IsValidTimeOffset(double seconds)
{
	return std::isfinite(seconds) &&
		std::abs(seconds) <= protocol::limits::MaxAbsTimeOffsetSeconds;
}

// Unix seconds of the last successful solve, or of the room copy; 0 means
// "unknown", which older records legitimately carry.
inline bool IsValidRecordUnixTime(double seconds)
{
	return std::isfinite(seconds) && seconds >= 0.0 &&
		seconds <= protocol::limits::MaxPlausibleUnixTimeSeconds;
}

} // namespace questcal
