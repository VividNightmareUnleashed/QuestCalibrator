#pragma once

#include "../common/Protocol.h"

// Spatial-correction-field evaluation.
//
// The overlay ships per-anchor deltas delta_i = anchor_i o base^-1: each is the
// small correction that, applied AFTER the base calibration, reproduces the
// absolute calibration solved at that anchor's spot. The driver blends the
// deltas with unnormalized Gaussian RBF weights over the device's own
// base-calibrated position (horizontal XZ distance) plus a constant identity
// floor, then applies world = delta_blend(base(raw)).
//
// Pure protocol types + hand-rolled double math, no vrserver dependencies:
// SolverTests compiles this file standalone and checks it against an Eigen
// reference implementation.
namespace alignfield
{
	// Identity floor weight w0. Far from every anchor the correction fades to
	// the base calibration instead of the nearest anchor winning outright
	// meters from where it was measured; at an anchor the applied correction
	// reaches 1/(1+w0) (~95%) of the measured delta.
	constexpr double IdentityFloorWeight = 0.05;

	// Slew limits for the applied delta. Deltas are <= ~3 deg / 5 cm, so these
	// complete any transition well under a second while suppressing pops; they
	// also sit far above the natural field gradient under walking speed
	// (~0.05 m/s), so steady traversal is never rate-limited.
	constexpr double MaxTranslationSlewPerSec = 0.25;   // m/s
	constexpr double MaxRotationSlewPerSec = 1.0;       // rad/s

	// Gaps larger than this between evaluations snap instead of slewing
	// (device standby, tracking loss - there is no motion to smooth across).
	constexpr double MaxSlewGapSeconds = 0.25;

	// Rate-limit profile for SlewToward. The field and the base calibration
	// share the slew implementation but move at very different scales.
	struct SlewLimits
	{
		double maxTranslationPerSec;
		double maxRotationPerSec;
		double maxGapSeconds;
	};

	constexpr SlewLimits FieldSlewLimits{ MaxTranslationSlewPerSec, MaxRotationSlewPerSec, MaxSlewGapSeconds };

	// Base-calibration slew (continuous calibration, protocol v5). Continuous
	// corrections arrive as <= ~1 cm / 0.5 deg steps every couple of seconds;
	// these limits complete a worst-case step in ~1-2 s while staying below
	// perception. Intentional discontinuities (recalibration, universe jump)
	// bump SetDeviceTransform::generation and snap instead.
	constexpr SlewLimits BaseSlewLimits{ 0.01, 0.00873 /* 0.5 deg/s */, MaxSlewGapSeconds };

	// Per-device smoothing state. Owned by that device's pose thread (each
	// OpenVR id updates on a single device-driver thread), so no locking.
	struct EvalState
	{
		bool hasCurrent = false;
		uint32_t generation = 0;
		double lastTime = 0.0;               // seconds, caller's clock
		vr::HmdQuaternion_t rot{ 1.0, 0.0, 0.0, 0.0 };
		double trans[3] = { 0.0, 0.0, 0.0 };
	};

	// Raw blended delta at a base-calibrated position (no smoothing). Always
	// yields a normalized quaternion; with zero anchors it is the identity.
	void BlendAt(const protocol::SetAlignmentField &field, const double (&basePos)[3],
	             vr::HmdQuaternion_t &rotOut, double (&transOut)[3]);

	// Blend + slew: advances `state` toward the blended delta, rate-limited by
	// the time since the previous call. Snaps when the field generation
	// changed - a recalibration or universe-jump compensation is an
	// intentional discontinuity that must not be smeared over time.
	void Evaluate(const protocol::SetAlignmentField &field, const double (&basePos)[3],
	              double nowSeconds, EvalState &state);

	// Advance `state` toward an arbitrary target transform, rate-limited by
	// `limits` and the time since the previous call. Snaps on the first
	// evaluation, on a generation change, and across evaluation gaps larger
	// than limits.maxGapSeconds. Shared by the field (via Evaluate) and the
	// driver's base-calibration slew.
	void SlewToward(const vr::HmdQuaternion_t &targetRot, const double (&targetTrans)[3],
	                double nowSeconds, const SlewLimits &limits, uint32_t generation,
	                EvalState &state);
}
