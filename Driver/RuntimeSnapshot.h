#pragma once
#include "AtomicSnapshot.h"
#include "../common/Protocol.h"
namespace questcal { namespace runtimesnapshot {
	// The envelope: fields the pose path honours whether or not calibration is
	// enabled for the device.
	struct DeviceControl
	{
		bool enabled = false;
		bool hidden = false;       // displace forwarded pose out of games' reach
		uint32_t generation = 0;   // snap/slew discriminator (see Protocol.h)
	};

	// One coherent slot read. `calibration` is meaningful only while
	// `control.enabled`; otherwise it keeps the protocol's neutral defaults
	// (identity quaternion, scale 1.0), so the slots must never be memset.
	struct DeviceTransform
	{
		DeviceControl control;
		protocol::SetDeviceTransform calibration;
		protocol::FrameCorrection frame;
	};

	// Protected by runtimeSequence: the IPC thread writes, pose callbacks read.
	// Payload scalars are individually lock-free atomic so a discarded mixed
	// generation is still race-free under the C++ memory model.
	struct TransformSlot
	{
		// Envelope - always stored, always loaded.
		std::atomic<uint32_t> enabled{ 0 };
		std::atomic<uint32_t> hidden{ 0 };
		std::atomic<uint32_t> generation{ 0 };
		// Calibration payload - loaded only while enabled. Explicit initialisers
		// (see atomicsnapshot::Double).
		questcal::atomicsnapshot::Vector3 translation;
		questcal::atomicsnapshot::Quaternion rotation;
		questcal::atomicsnapshot::Double scale{ 1.0 };
		questcal::atomicsnapshot::Double timeOffset{ 0.0 };
		questcal::atomicsnapshot::Quaternion frameRotation;
		questcal::atomicsnapshot::Vector3 frameTranslation;

		void Store(const protocol::SetDeviceTransform &source,
			const protocol::FrameCorrection &frame = {}) noexcept
		{
			enabled.store(source.enabled ? 1u : 0u, std::memory_order_release);
			hidden.store(source.hidden ? 1u : 0u, std::memory_order_release);
			generation.store(source.generation, std::memory_order_release);
			translation.Store(source.translation.v);
			rotation.Store(source.rotation);
			scale.store(source.scale, std::memory_order_release);
			timeOffset.store(source.timeOffset, std::memory_order_release);
			frameRotation.Store(frame.rotation);
			frameTranslation.Store(frame.translation.v);
		}

		DeviceTransform Load() const noexcept
		{
			DeviceTransform result;
			result.control.enabled = enabled.load(std::memory_order_acquire) != 0;
			result.control.hidden = hidden.load(std::memory_order_acquire) != 0;
			result.control.generation = generation.load(std::memory_order_acquire);
			result.calibration.enabled = result.control.enabled ? 1u : 0u;
			result.calibration.hidden = result.control.hidden ? 1u : 0u;
			result.calibration.generation = result.control.generation;
			if (!result.control.enabled)
				return result;   // payload stays at its neutral defaults

			translation.Load(result.calibration.translation.v);
			result.calibration.rotation = rotation.Load<vr::HmdQuaternion_t>();
			result.calibration.scale = scale.load(std::memory_order_acquire);
			result.calibration.timeOffset = timeOffset.load(std::memory_order_acquire);
			result.frame.rotation = frameRotation.Load<vr::HmdQuaternion_t>();
			frameTranslation.Load(result.frame.translation.v);
			return result;
		}

		// Store/Load mirror the wire struct by hand; this turns a new protocol
		// field into a build break rather than a silent default on the pose path.
		static_assert(sizeof(protocol::SetDeviceTransform) == 88,
			"protocol::SetDeviceTransform changed; mirror the new field in TransformSlot::Store/Load "
			"and place it in DeviceControl or the calibration payload");
	};

	struct AtomicFieldAnchor
	{
		questcal::atomicsnapshot::Vector3 position;
		questcal::atomicsnapshot::Quaternion rotationDelta;
		questcal::atomicsnapshot::Vector3 translationDelta;

		void Store(const protocol::FieldAnchor &source) noexcept
		{
			position.Store(source.position);
			rotationDelta.Store(source.rotationDelta);
			translationDelta.Store(source.translationDelta);
		}

		void Load(protocol::FieldAnchor &destination) const noexcept
		{
			position.Load(destination.position);
			destination.rotationDelta = rotationDelta.Load<vr::HmdQuaternion_t>();
			translationDelta.Load(destination.translationDelta);
		}
	};

	struct AtomicAlignmentField
	{
		std::atomic<uint32_t> enabled{ 0 };
		std::atomic<uint32_t> generation{ 0 };
		std::atomic<uint32_t> anchorCount{ 0 };
		questcal::atomicsnapshot::Double sigmaMeters{ 1.5 };   // the protocol default
		AtomicFieldAnchor anchors[protocol::SetAlignmentField::MaxAnchors];

		void Store(const protocol::SetAlignmentField &source) noexcept
		{
			enabled.store(source.enabled ? 1u : 0u, std::memory_order_release);
			generation.store(source.generation, std::memory_order_release);
			anchorCount.store(source.anchorCount, std::memory_order_release);
			sigmaMeters.store(source.sigmaMeters, std::memory_order_release);
			for (uint32_t i = 0; i < protocol::SetAlignmentField::MaxAnchors; ++i)
				anchors[i].Store(source.anchors[i]);
		}

		protocol::SetAlignmentField Load() const noexcept
		{
			protocol::SetAlignmentField result;
			result.enabled = enabled.load(std::memory_order_acquire) != 0;
			if (!result.enabled)
				return result;

			result.generation = generation.load(std::memory_order_acquire);
			// Only validated counts (<= MaxAnchors) are ever stored, and this is one
			// atomic, so even a torn snapshot cannot read past the array.
			result.anchorCount = anchorCount.load(std::memory_order_acquire);
			result.sigmaMeters = sigmaMeters.load(std::memory_order_acquire);
			for (uint32_t i = 0; i < result.anchorCount; ++i)
				anchors[i].Load(result.anchors[i]);
			return result;
		}
	};

struct Snapshot { DeviceTransform transform; protocol::SetAlignmentField field; };
inline Snapshot Read(const std::atomic<uint32_t> &sequence, const TransformSlot &slot,
    const AtomicAlignmentField &field, Snapshot &lastGood)
{
    for (int attempt = 0; attempt < 8; ++attempt) {
        const uint32_t before = sequence.load(std::memory_order_acquire);
        if (before & 1u) continue;
        Snapshot snapshot{slot.Load(), field.Load()};
        const uint32_t after = sequence.load(std::memory_order_acquire);
        if (before == after) { lastGood = snapshot; return snapshot; }
    }
    return lastGood;
}
} }
