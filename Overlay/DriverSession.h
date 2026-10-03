#pragma once

#include "DriverSyncPolicy.h"
#include "FrameRecovery.h"
#include "../common/Protocol.h"

#include <cstdint>
#include <array>
#include <algorithm>
#include <functional>
#include <string>
#include <utility>

// One driver conversation: handshake, complete state application, targeted
// neutralization, and error debouncing. Injected transport and enumeration keep
// connection handling testable without Windows or CalibrationContext.
namespace questcal
{

// One round-trip's outcome, as the session reasons about it. IPCClient throws
// on I/O failure and exposes its connection generation separately; both become
// values here.
struct DriverTransportResult
{
	// False means the transport failed outright and no response was received.
	bool completed = false;
	// Populated only when !completed.
	std::string error;
	// With !completed: the driver is from another release, which reinstalling
	// fixes and retrying does not.
	bool versionMismatch = false;
	// Valid only when completed.
	protocol::Response response;
	// The transport's connection generation AFTER the attempt, whether or not
	// it succeeded: SendBlocking reconnects and replays internally, so a request
	// can land on a different pipe than the one the batch started on.
	uint64_t connectionGeneration = 0;
};

using DriverTransport = std::function<DriverTransportResult(const protocol::Request &)>;

// Enumerate the OpenVR device at `id` (the result carries that id) as the slot
// decision reads it. Called once per slot.
using DriverDeviceEnumerator =
	std::function<SyncDevice(uint32_t id, const DriverSyncDesired &desired)>;

// Everything one reconciliation needs, as plain values.
struct DriverApplyRequest
{
	// Whether the profile should be applied at all. The caller's validity gates
	// have already run by this point; a slot decision can still withdraw it.
	bool enabled = false;
	DriverSyncDesired desired;
	// The field the caller would ship; `enabled` is only the caller's half of the
	// predicate (profile live, anchors present). The session also requires at
	// least one enabled slot and builds the canonical disable itself.
	protocol::SetAlignmentField field;
	std::array<protocol::FrameCorrection, vr::k_unMaxTrackedDeviceCount> frames{};
	FrameSerialKeys frameSerialKeys{};
	// Parked recovery entries retain identity but cannot drive a slot until
	// enumeration has bound that slot to the same physical serial.
	uint64_t frameBoundMask = 0;
	uint64_t frameProfileKey = 0;
	uint64_t driverSessionId = 0;
	bool recoverFrames = false;

	bool operator==(const DriverApplyRequest &other) const
	{
		return enabled == other.enabled && desired == other.desired &&
			field == other.field && frames == other.frames && frameSerialKeys == other.frameSerialKeys &&
			frameBoundMask == other.frameBoundMask && frameProfileKey == other.frameProfileKey &&
			driverSessionId == other.driverSessionId &&
			recoverFrames == other.recoverFrames;
	}
};

// Why the session stopped applying the profile; the UI tells the user which.
enum class DriverDisableCause
{
	None,
	// The live headset reports a different tracking system than the profile's
	// reference.
	HmdMismatch,
	// The batch did not complete on one connection.
	DriverUnreachable,
	// The batch failed because the driver is from another release.
	DriverVersionMismatch,
};

// What the caller mirrors into its own state. Fail-closed: short of a complete
// batch, the masks, the tracker id and `enabled` come back cleared.
struct DriverApplyResult
{
	bool enabled = false;
	// The complete desired state reached one driver connection.
	bool synchronized = false;
	DriverDisableCause cause = DriverDisableCause::None;
	bool referenceDeviceMask[vr::k_unMaxTrackedDeviceCount] = {};
	bool targetDeviceMask[vr::k_unMaxTrackedDeviceCount] = {};
	uint32_t continuousTrackerId = vr::k_unTrackedDeviceIndexInvalid;
	uint32_t poseHookMask = 0;
	uint64_t driverSessionId = 0;
	uint64_t frameProfileKey = 0;
	bool recoveryChecked = false;
	bool framesRecovered = false;
	RecoveredFrames frames{};
	FrameSerialKeys frameSerialKeys{};
};

// The per-slot half of one reconciliation, derived from the enumerated devices
// alone. Pure, so the submitting thread and the session derive the same
// identities from the same devices.
struct DriverSlotState
{
	uint64_t enabledMask = 0;
	uint64_t hiddenMask = 0;
	bool hmdMismatch = false;
	bool referenceDeviceMask[vr::k_unMaxTrackedDeviceCount] = {};
	bool targetDeviceMask[vr::k_unMaxTrackedDeviceCount] = {};
	uint32_t continuousTrackerId = vr::k_unTrackedDeviceIndexInvalid;
};

inline DriverSlotState DeriveDriverSlotState(const DriverSyncDesired &desired,
	const DriverDeviceEnumerator &enumerate)
{
	DriverSlotState state;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		const SyncDevice device = enumerate(id, desired);
		const SlotDecision decision = DecideSlot(desired, device);
		state.referenceDeviceMask[id] = decision.referenceDevice;
		state.targetDeviceMask[id] = decision.targetDevice;
		if (decision.continuousTracker)
			state.continuousTrackerId = id;
		if (decision.disableProfile)
			state.hmdMismatch = true;
		if (decision.action != SlotAction::ApplyTransform)
			continue;
		state.enabledMask |= uint64_t{ 1 } << id;
		if (decision.transform.hidden)
			state.hiddenMask |= uint64_t{ 1 } << id;
	}
	return state;
}

class DriverSession
{
	struct Batch
	{
		bool connectionReady = false;
		uint64_t connectionGeneration = 0;
		uint32_t poseHookMask = 0;
		uint64_t driverSessionId = 0;
	};

public:
	void SetTransport(DriverTransport newTransport) { transport = std::move(newTransport); }
	void SetDeviceEnumerator(DriverDeviceEnumerator newEnumerator)
	{
		enumerate = std::move(newEnumerator);
	}
	// Error presentation belongs to the caller rather than the transport layer.
	void SetErrorSink(std::function<void(const std::string &)> report,
		std::function<void()> clear)
	{
		reportError = std::move(report);
		clearError = std::move(clear);
	}

	// Drive the driver to `request`, and report what the caller may now believe.
	DriverApplyResult Apply(const DriverApplyRequest &request, double atTime)
	{
		now = atTime;
		versionMismatch = false;
		const Batch batch = Begin();
		DriverApplyResult result;
		result.enabled = request.enabled;
		result.poseHookMask = batch.poseHookMask;
		result.driverSessionId = batch.driverSessionId;
		if (!batch.connectionReady || (request.driverSessionId != 0 &&
			request.driverSessionId != batch.driverSessionId))
		{
			result.enabled = false;
			result.cause = UnreachableCause();
			return result;
		}
		uint64_t batchConnectionGeneration = batch.connectionGeneration;

		protocol::SetRuntimeState desiredState;
		desiredState.transform = protocol::SetDeviceTransform(0, true,
			WireVector(request.desired.translationMeters),
			WireQuaternion(request.desired.rotation), request.desired.scale,
			request.desired.timeShift);
		desiredState.transform.generation = request.desired.baseGeneration;
		desiredState.field = request.field;
		std::copy(request.frames.begin(), request.frames.end(), desiredState.frames);
		desiredState.frameProfileKey = request.frameProfileKey;
		desiredState.expectedSessionId = batch.driverSessionId;
		std::copy(request.frameSerialKeys.begin(), request.frameSerialKeys.end(), desiredState.frameSerialKeys);
		if (request.recoverFrames)
		{
			protocol::Response snapshot;
			if (!SendRequest(protocol::Request(protocol::RequestGetRuntimeState),
				"recovering tracker frame corrections", &batchConnectionGeneration, &snapshot))
			{
				result.enabled = false;
				result.cause = UnreachableCause();
				return result;
			}
			result.recoveryChecked = true;
			result.frameProfileKey = request.frameProfileKey;
			result.frameSerialKeys = request.frameSerialKeys;
			result.framesRecovered = RecoverTrackerFrames(snapshot, batch.driverSessionId,
				request.frameProfileKey, request.frameSerialKeys, result.frames, &result.frameSerialKeys);
			if (!result.framesRecovered && request.frameProfileKey != 0 &&
				snapshot.runtimeState.frameProfileKey == request.frameProfileKey)
			{
				ReportThrottled("The driver returned invalid tracker-frame recovery state; calibration was not overwritten\n");
				result.enabled = false;
				result.cause = UnreachableCause();
				return result;
			}
			if (result.framesRecovered)
			{
				std::copy(result.frames.begin(), result.frames.end(), desiredState.frames);
				std::copy(result.frameSerialKeys.begin(), result.frameSerialKeys.end(), desiredState.frameSerialKeys);
			}
		}

		if (result.enabled)
		{
			const DriverSlotState slots = DeriveDriverSlotState(request.desired, enumerate);
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				result.referenceDeviceMask[id] = slots.referenceDeviceMask[id];
				result.targetDeviceMask[id] = slots.targetDeviceMask[id];
			}
			result.continuousTrackerId = slots.continuousTrackerId;
			desiredState.enabledMask = slots.enabledMask;
			desiredState.hiddenMask = slots.hiddenMask;
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				const uint64_t bit = uint64_t{1} << id;
				if (desiredState.frameSerialKeys[id] == 0 || (request.frameBoundMask & bit) != 0)
					continue;
				desiredState.enabledMask &= ~bit;
				desiredState.hiddenMask &= ~bit;
				result.targetDeviceMask[id] = false;
				if (result.continuousTrackerId == id)
					result.continuousTrackerId = vr::k_unTrackedDeviceIndexInvalid;
			}
			if (slots.hmdMismatch)
			{
				result.enabled = false;
				result.cause = DriverDisableCause::HmdMismatch;
			}
		}

		if (!result.enabled)
		{
			desiredState.enabledMask = 0;
			desiredState.hiddenMask = 0;
			for (bool &value : result.referenceDeviceMask) value = false;
			for (bool &value : result.targetDeviceMask) value = false;
			result.continuousTrackerId = vr::k_unTrackedDeviceIndexInvalid;
		}
		desiredState.field.enabled = desiredState.field.enabled &&
			desiredState.enabledMask != 0;
		if (!desiredState.field.enabled)
			desiredState.field.anchorCount = 0;

		protocol::Request stateRequest(protocol::RequestSetRuntimeState);
		stateRequest.setRuntimeState = desiredState;
		bool driverSynchronized = batch.connectionReady && SendRequest(stateRequest,
			"applying the complete driver state", &batchConnectionGeneration);

		// Short of a complete batch the monitors must not believe the driver
		// matches the profile; the next periodic scan retries the whole state.
		result.synchronized = driverSynchronized;
		if (!driverSynchronized)
		{
			result.enabled = false;
			result.cause = UnreachableCause();
			result.continuousTrackerId = vr::k_unTrackedDeviceIndexInvalid;
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				result.referenceDeviceMask[id] = false;
				result.targetDeviceMask[id] = false;
			}
		}
		else
		{
			clearError();
			lastErrorTime = -1e9;
		}
		return result;
	}

	// Retire one slot outside any batch: the pre-collection reset, which must
	// clear a stale transform off the device about to be sampled.
	bool DisableDeviceTransform(uint32_t id, double atTime)
	{
		now = atTime;
		versionMismatch = false;
		protocol::Request req(protocol::RequestSetDeviceTransform);
		req.setDeviceTransform = protocol::SetDeviceTransform(id, false);
		return SendRequest(req, "disabling a device transform", nullptr);
	}

private:
	DriverDisableCause UnreachableCause() const
	{
		return versionMismatch ? DriverDisableCause::DriverVersionMismatch
			: DriverDisableCause::DriverUnreachable;
	}

	Batch Begin()
	{
		Batch batch;
		protocol::Request handshake(protocol::RequestHandshake);
		protocol::Response response;
		batch.connectionReady = SendRequest(handshake, "checking the driver connection",
			&batch.connectionGeneration, &response);
		if (batch.connectionReady)
		{
			batch.poseHookMask = response.poseHookMask;
			batch.driverSessionId = response.driverSessionId;
		}
		return batch;
	}

	// Every request goes through here. A refused response and a failed transport
	// are one verdict (the state was not applied) and share one 30 s debounce,
	// so a dead pipe cannot rewrite the user's banner on every scan.
	bool SendRequest(const protocol::Request &request, const char *operation,
		uint64_t *batchConnectionGeneration,
		protocol::Response *acceptedResponse = nullptr)
	{
		const DriverTransportResult result = transport(request);
		if (result.completed)
		{
			bool accepted = (result.response.type == protocol::ResponseSuccess &&
				request.type != protocol::RequestGetRuntimeState) ||
				(request.type == protocol::RequestGetRuntimeState &&
					result.response.type == protocol::ResponseRuntimeState &&
					result.response.protocol.version == protocol::Version) ||
				(request.type == protocol::RequestHandshake &&
					result.response.type == protocol::ResponseHandshake &&
					result.response.protocol.version == protocol::Version);
			if (accepted)
			{
				if (acceptedResponse)
					*acceptedResponse = result.response;
				if (batchConnectionGeneration)
				{
					if (*batchConnectionGeneration == 0)
						*batchConnectionGeneration = result.connectionGeneration;
					else if (*batchConnectionGeneration != result.connectionGeneration)
					{
						// Accepted by a different connection than the batch began
						// on; not an error to report.
						return false;
					}
				}
				return true;
			}
			ReportThrottled(std::string("QuestCalibrator driver rejected ") + operation +
				"; the requested live state was not applied\n");
			return false;
		}
		versionMismatch = versionMismatch || result.versionMismatch;
		ReportThrottled(std::string("QuestCalibrator driver communication failed while ") +
			operation + ": " + result.error + "\n");
		return false;
	}

	void ReportThrottled(const std::string &message)
	{
		if (now - lastErrorTime >= 30.0)
		{
			reportError(message);
			lastErrorTime = now;
		}
	}

	// All four are installed before use (DriverWorker, and every test fixture).
	DriverTransport transport;
	DriverDeviceEnumerator enumerate;
	std::function<void(const std::string &)> reportError;
	std::function<void()> clearError;

	// Error debounce clock, in the caller's tick time. -1e9 means "re-armed":
	// the next failure reports immediately.
	double lastErrorTime = -1e9;
	// The caller's tick time for the request in flight, set at each entry point.
	double now = 0.0;
	// A request at this entry point failed because the driver is from another
	// release.
	bool versionMismatch = false;
};

} // namespace questcal
