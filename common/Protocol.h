#pragma once

#include <cstdint>

#ifndef _OPENVR_API
#include <openvr_driver.h>
#endif

#define QUESTCALIBRATOR_PIPE_NAME "\\\\.\\pipe\\QuestCalibratorDriver"
// Versioned by PoseRing::LayoutVersion, independently of the pipe protocol: a
// named mapping outlives the process that made it, so an old name could strand
// an upgraded driver behind an incompatible overlay-held mapping.
#define QUESTCALIBRATOR_SHMEM_NAME "Local\\QuestCalibratorPoseRing.v6.layout4"

namespace protocol
{
	// The handshake requires exact version equality, so every bump costs users a
	// driver reinstall and a SteamVR restart. v9: the field blends
	// field-transformed query positions, and both peers must share those
	// semantics because continuous calibration predicts the driver's output.
	const uint32_t Version = 9;
	const uint32_t PoseHook005 = 1u << 0;
	const uint32_t PoseHook006 = 1u << 1;

	enum RequestType : uint32_t
	{
		RequestInvalid,
		RequestHandshake,
		RequestSetDeviceTransform,
		RequestSetRuntimeState,
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
		uint32_t enabled = 0;          // canonical wire flag: validation requires 0 or 1
		vr::HmdVector3d_t translation{ { 0.0, 0.0, 0.0 } };
		vr::HmdQuaternion_t rotation{ 1.0, 0.0, 0.0, 0.0 };
		double scale = 1.0;
		// Seconds added to DriverPose_t::poseTimeOffset. Positive declares the
		// pose newer, shortening vrserver's forward prediction (a delay); this is
		// how the solved inter-system time offset is applied to target devices.
		double timeOffset = 0.0;
		// Snap/slew discriminator (SetAlignmentField::generation works the same
		// way): a changed generation marks an intentional discontinuity
		// (recalibration, universe jump, profile edit) and the driver snaps; an
		// unchanged generation with a changed transform is a continuous-calibration
		// correction and the driver slews toward it.
		uint32_t generation = 0;
		// Displace this device's forwarded pose far away so applications ignore
		// it (the HMD-mounted continuous-calibration tracker). The raw pose the
		// overlay's solver consumes is published before the displacement.
		uint32_t hidden = 0;           // canonical wire flag: validation requires 0 or 1

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

		bool operator==(const FieldAnchor &other) const noexcept
		{
			return position[0] == other.position[0] &&
				position[1] == other.position[1] &&
				position[2] == other.position[2] &&
				rotationDelta.w == other.rotationDelta.w &&
				rotationDelta.x == other.rotationDelta.x &&
				rotationDelta.y == other.rotationDelta.y &&
				rotationDelta.z == other.rotationDelta.z &&
				translationDelta[0] == other.translationDelta[0] &&
				translationDelta[1] == other.translationDelta[1] &&
				translationDelta[2] == other.translationDelta[2];
		}
	};

	// Spatial correction field. The driver blends the anchor deltas with
	// Gaussian RBF weights over each device's own base-calibrated position and
	// applies blendedDelta ∘ base. `generation` snaps or slews exactly as
	// SetDeviceTransform::generation does.
	struct SetAlignmentField
	{
		static const uint32_t MaxAnchors = 8;

		uint32_t enabled = 0;          // canonical wire flag: validation requires 0 or 1
		uint32_t generation = 0;
		uint32_t anchorCount = 0;
		double sigmaMeters = 1.5;         // RBF falloff in the horizontal plane
		FieldAnchor anchors[MaxAnchors];

		bool operator==(const SetAlignmentField &other) const noexcept
		{
			if (enabled != other.enabled || generation != other.generation ||
				anchorCount != other.anchorCount || sigmaMeters != other.sigmaMeters)
				return false;
			for (uint32_t i = 0; i < MaxAnchors; ++i)
				if (!(anchors[i] == other.anchors[i]))
					return false;
			return true;
		}
	};

	struct SetRuntimeState
	{
		uint64_t enabledMask = 0;
		uint64_t hiddenMask = 0;
		// The payload shared by every enabled slot. openVRID/enabled/hidden are
		// canonical placeholders; the driver derives their per-slot values from
		// the masks only after validating the complete message.
		SetDeviceTransform transform{ 0, true };
		SetAlignmentField field;
	};

	// Raw driver-space pose as captured by the pose hook inside vrserver, stamped
	// with QueryPerformanceCounter at capture. Protocol-owned because the overlay
	// compiles against openvr.h, which has no DriverPose_t.
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
	// Every field has an initializer (padding is not scrubbed; the pipe never
	// leaves this machine).
	struct Request
	{
		Protocol protocol;
		RequestType type = RequestInvalid;
		SetDeviceTransform setDeviceTransform;
		SetRuntimeState setRuntimeState;

		Request() = default;
		explicit Request(RequestType type) : type(type) { }
	};

	struct Response
	{
		ResponseType type = ResponseInvalid;
		Protocol protocol;
		uint32_t poseHookMask = 0;

		Response() = default;
		explicit Response(ResponseType type) : type(type) { }
	};

	// The pipe carries a version but no layout identity, and the two ends get the
	// OpenVR types from different headers (openvr_driver.h vs openvr.h), so equal
	// versions do not prove equal bytes. When one of these fires on purpose, bump
	// protocol::Version and update the size. x64 is the only target, so these
	// sizes are exact.
	static_assert(sizeof(SetDeviceTransform) == 88, "SetDeviceTransform wire layout changed");
	static_assert(sizeof(FieldAnchor) == 80, "FieldAnchor wire layout changed");
	static_assert(sizeof(SetAlignmentField) == 664, "SetAlignmentField wire layout changed");
	static_assert(sizeof(SetRuntimeState) == 768, "SetRuntimeState wire layout changed");
	static_assert(sizeof(Request) == 864, "Request wire layout changed");
	static_assert(sizeof(Response) == 12, "Response wire layout changed");
	// Crosses the shared-memory ring: bump PoseRing::LayoutVersion instead.
	static_assert(sizeof(DevicePoseSample) == 192, "DevicePoseSample layout changed");
}
