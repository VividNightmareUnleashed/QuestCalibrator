#pragma once

// Numeric limits shared by profile/UI validation and the vrserver trust
// boundary. Keeping them protocol-owned prevents the overlay from accepting a
// value that the driver will later reject after reporting a successful edit.
namespace protocol
{
namespace limits
{
	constexpr double MinScale = 0.25;
	constexpr double MaxScale = 4.0;
	constexpr double MaxAbsTranslationMeters = 10000.0;
	constexpr double MaxAbsTimeOffsetSeconds = 1.0;
	constexpr double MaxAbsPosePositionMeters = 10000.0;
	constexpr double MaxAbsLinearVelocityMetersPerSecond = 1000.0;
	constexpr double MaxAbsAngularVelocityRadiansPerSecond = 10000.0;
	constexpr double MaxPlausibleUnixTimeSeconds = 32503680000.0; // 3000-01-01 UTC
	// QPC-derived pose times are seconds since boot. This is deliberately far
	// beyond a plausible Windows uptime while still excluding finite magnitudes
	// whose subtraction/interpolation would overflow or lose all resolution.
	constexpr double MaxAbsPoseTimestampSeconds = 1.0e12;
	constexpr double MaxPlayAreaSizeMeters = 1000.0;
	constexpr double MaxAbsChaperoneCoordinateMeters = 10000.0;
	constexpr double MaxStandingBasisError = 0.05;
	constexpr double MinFieldSigmaMeters = 0.05;
	constexpr double MaxFieldSigmaMeters = 100.0;
	constexpr double MaxAbsAnchorPositionMeters = 10000.0;
	constexpr double MaxAbsAnchorDeltaMeters = 100.0;
	constexpr double MinQuaternionComponentNorm = 1e-6;
}
}
