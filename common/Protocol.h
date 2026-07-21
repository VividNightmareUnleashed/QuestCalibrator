#pragma once

#include <cstdint>

#ifndef _OPENVR_API
#include <openvr_driver.h>
#endif

#define QUESTCALIBRATOR_PIPE_NAME "\\\\.\\pipe\\QuestCalibratorDriver"
#define QUESTCALIBRATOR_SHMEM_NAME "Local\\QuestCalibratorPoseRing"

namespace protocol
{
	// v3: SetDeviceTransform always carries the full transform (no partial-update
	// flags — a partial update against unknown driver-side state is how positions
	// used to collapse to zero), and the shared-memory pose ring was added.
	// v4: SetDeviceTransform carries timeOffset (runtime latency re-prediction),
	// and SetAlignmentField ships the spatial-correction anchor set.
	// v5: SetDeviceTransform carries `generation` (snap/slew discriminator for
	// continuous calibration) and `hidden` (displace the HMD-mounted tracker out
	// of games' reach). The complete wire format is defined up front (the
	// handshake requires exact version equality, so every bump costs users a
	// driver reinstall + SteamVR restart); the driver may store-and-ignore
	// fields until the matching feature lands.
	const uint32_t Version = 5;

	enum RequestType : uint32_t
	{
		RequestInvalid,
		RequestHandshake,
		RequestSetDeviceTransform,
		RequestSetAlignmentField,
	};

	enum ResponseType : uint32_t
	{
		ResponseInvalid,
		ResponseHandshake,
		ResponseSuccess,
	};

	struct Protocol
	{
		uint32_t version = Version;
	};

	// Always a complete transform. Defaults are the neutral transform so a
	// value-initialized message is harmless.
	struct SetDeviceTransform
	{
		uint32_t openVRID = 0xFFFFFFFF;   // k_unTrackedDeviceIndexInvalid
		bool enabled = false;
		vr::HmdVector3d_t translation{ { 0.0, 0.0, 0.0 } };
		vr::HmdQuaternion_t rotation{ 1.0, 0.0, 0.0, 0.0 };
		double scale = 1.0;
		// Seconds added to DriverPose_t::poseTimeOffset. Positive declares the
		// pose newer, shortening vrserver's forward prediction (a delay); this is
		// how the solved inter-system time offset is applied to target devices.
		double timeOffset = 0.0;
		// Snap/slew discriminator for the base transform, mirroring
		// SetAlignmentField::generation: a changed generation marks an intentional
		// discontinuity (recalibration, universe jump, profile edit) and the
		// driver snaps to the new transform; an unchanged generation with a
		// changed transform is a continuous-calibration correction and the driver
		// slews toward it instead.
		uint32_t generation = 0;
		// Displace this device's forwarded pose far away so applications ignore
		// it (the HMD-mounted continuous-calibration tracker). The raw pose the
		// overlay's solver consumes is published before the displacement.
		bool hidden = false;

		SetDeviceTransform() = default;

		SetDeviceTransform(uint32_t id, bool enabled)
			: openVRID(id), enabled(enabled) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation, double scale, double timeOffset = 0.0)
			: openVRID(id), enabled(enabled), translation(translation), rotation(rotation), scale(scale), timeOffset(timeOffset) { }
	};

	// One spatial-correction anchor: a small delta transform relative to the base
	// calibration, valid around `position` (reference/world space, meters).
	struct FieldAnchor
	{
		double position[3] = { 0.0, 0.0, 0.0 };
		vr::HmdQuaternion_t rotationDelta{ 1.0, 0.0, 0.0, 0.0 };
		double translationDelta[3] = { 0.0, 0.0, 0.0 };
	};

	// Spatial correction field. The driver blends the anchor deltas
	// with Gaussian RBF weights over each device's own base-calibrated position
	// and applies base ∘ blendedDelta. `generation` bumps on any recalibration
	// or universe-jump compensation so driver-side smoothing snaps instead of
	// smearing an intentional change over time.
	struct SetAlignmentField
	{
		static const uint32_t MaxAnchors = 8;

		bool enabled = false;
		uint32_t generation = 0;
		uint32_t anchorCount = 0;
		double sigmaMeters = 1.5;         // RBF falloff in the horizontal plane
		FieldAnchor anchors[MaxAnchors];
	};

	// Raw driver-space pose as captured by the pose hook inside vrserver, stamped
	// with QueryPerformanceCounter at capture. Field set mirrors what the solver
	// needs from vr::DriverPose_t; kept protocol-owned because the overlay compiles
	// against openvr.h, which has no DriverPose_t.
	struct DevicePoseSample
	{
		int64_t sampleTimeQpc = 0;
		uint32_t deviceId = 0xFFFFFFFF;   // k_unTrackedDeviceIndexInvalid
		uint32_t trackingResult = 0;      // vr::ETrackingResult
		bool poseIsValid = false;
		bool deviceIsConnected = false;
		double poseTimeOffset = 0.0;      // seconds; driver's own age estimate for the pose
		vr::HmdQuaternion_t worldFromDriverRotation{ 1.0, 0.0, 0.0, 0.0 };
		double worldFromDriverTranslation[3] = { 0.0, 0.0, 0.0 };
		vr::HmdQuaternion_t rotation{ 1.0, 0.0, 0.0, 0.0 };   // driver-space orientation
		double position[3] = { 0.0, 0.0, 0.0 };               // driver-space position, meters
		double velocity[3] = { 0.0, 0.0, 0.0 };               // m/s
		double angularVelocity[3] = { 0.0, 0.0, 0.0 };        // rad/s
	};

	// Fixed-size wire messages; the whole struct crosses the pipe via sizeof.
	// Every field has an initializer so no meaningful byte is ever indeterminate
	// (padding bytes are not scrubbed — the pipe never leaves this machine).
	struct Request
	{
		RequestType type = RequestInvalid;
		SetDeviceTransform setDeviceTransform;
		SetAlignmentField setAlignmentField;

		Request() = default;
		explicit Request(RequestType type) : type(type) { }
	};

	struct Response
	{
		ResponseType type = ResponseInvalid;
		Protocol protocol;

		Response() = default;
		explicit Response(ResponseType type) : type(type) { }
	};
}
