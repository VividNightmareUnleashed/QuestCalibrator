#pragma once

#include "DriverSyncPolicy.h"
#include "../common/Protocol.h"

#include <cstdint>
#include <functional>
#include <string>
#include <utility>

// The overlay's side of one driver conversation: one handshake, one atomic
// desired-state request, and error debouncing.
//
// This is the transport half of what SynchronizeDriverState used to be. The
// decision half already lives in DriverSyncPolicy.h; what stayed unreachable
// was the connection sequencing and fail-closed behavior. Both seams are
// injected instead:
//
//   - the transport, so a refusal, a dead pipe and a silent reconnect are
//     scriptable values;
//   - the device enumerator, because the enumeration is lazy and the laziness
//     is load-bearing: the neutral path performs none at all, and the live loop
//     stops paying for OpenVR property reads the moment a send fails.
//
// Deliberately free of openvr.h, windows.h and CalibrationContext. Protocol.h
// already picks openvr.h or openvr_driver.h from whichever translation unit
// includes it (the two conflict, and the harness carries the driver one), the
// IPC client stays behind the transport function so this header never needs a
// Windows type, and everything the caller owns crosses the seam as a plain
// value.
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
	// Valid only when completed.
	protocol::Response response;
	// The transport's connection generation AFTER the attempt, reported whether
	// or not the attempt succeeded. IPCClient::SendBlocking reconnects and
	// replays internally, so a request can be accepted on a pipe other than the
	// one the batch started on — and a request that ultimately failed can still
	// have advanced the generation on the way. Every batch rule below is written
	// against this number, so a transport that only reported it on success would
	// silently disable the reconnect handling.
	uint64_t connectionGeneration = 0;
};

using DriverTransport = std::function<DriverTransportResult(const protocol::Request &)>;

// Enumerate one OpenVR device as the slot decision reads it. Called at most
// once per slot, and only for slots the loop actually reaches.
using DriverDeviceEnumerator =
	std::function<SyncDevice(uint32_t id, const DriverSyncDesired &desired)>;

// The handshake that opens one reconciliation batch, and the connection
// generation it landed on. Produced by Begin() and consumed by Apply().
struct DriverBatch
{
	bool connectionReady = false;
	uint64_t connectionGeneration = 0;
	uint32_t poseHookMask = 0;
};

// Everything one reconciliation needs, as plain values.
struct DriverApplyRequest
{
	// Whether the profile should be applied at all. The caller's validity gates
	// have already run by this point; a slot decision can still withdraw it.
	bool enabled = false;
	DriverSyncDesired desired;
	// The spatial correction field the caller would ship, with `enabled`
	// carrying only the caller's own half of the predicate: the profile is live
	// and there is something to blend. The session ANDs in the half only it
	// knows — a complete base-transform batch on this same connection — and
	// builds the canonical disable itself, so a bad base can still clear a stale
	// field.
	protocol::SetAlignmentField field;
};

// Why the session stopped applying the profile. Distinct causes rather than one
// "disabled" flag because the UI tells the user what to go and check: a dead
// pipe and a foreign headset send them to different places.
enum class DriverDisableCause
{
	None,
	// The live headset reports a different tracking system than the profile's
	// reference. This is a different rig.
	HmdMismatch,
	// The batch did not complete on one connection.
	DriverUnreachable,
};

// What the caller mirrors into its own derived state. Already fail-closed: on
// anything short of a complete batch the masks, the tracker id and `enabled`
// come back cleared, so a caller that simply assigns this cannot leave the
// monitors believing the driver matches the profile.
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
};

class DriverSession
{
public:
	void SetTransport(DriverTransport newTransport) { transport = std::move(newTransport); }
	void SetDeviceEnumerator(DriverDeviceEnumerator newEnumerator)
	{
		enumerate = std::move(newEnumerator);
	}
	// Where a driver failure surfaces, and where a recovered batch withdraws it.
	// Both are the caller's, because "an error banner" is a UI concept and this
	// class must not name the context that owns one.
	void SetErrorSink(std::function<void(const std::string &)> report,
		std::function<void()> clear)
	{
		reportError = std::move(report);
		clearError = std::move(clear);
	}

	// Open a batch: one handshake, whose reply stamps the connection generation
	// every later request in this batch is checked against.
	//
	// Separate from Apply on purpose. The caller runs its own validity gates
	// between the two and can report its own error there, and error banners are
	// last-writer-wins — so this handshake's failure has to be reported before
	// those gates run, exactly as it was when one function owned both halves.
	DriverBatch Begin(double atTime)
	{
		now = atTime;
		DriverBatch batch;
		protocol::Request handshake(protocol::RequestHandshake);
		protocol::Response response;
		batch.connectionReady = SendRequest(handshake, "checking the driver connection",
			&batch.connectionGeneration, &response);
		if (batch.connectionReady)
			batch.poseHookMask = response.poseHookMask;
		return batch;
	}

	// Drive the driver to `request`, and report what the caller may now believe.
	DriverApplyResult Apply(const DriverBatch &batch, const DriverApplyRequest &request,
		double atTime)
	{
		now = atTime;
		DriverApplyResult result;
		result.enabled = request.enabled;
		result.poseHookMask = batch.poseHookMask;
		uint64_t batchConnectionGeneration = batch.connectionGeneration;

		protocol::SetRuntimeState desiredState;
		desiredState.transform = protocol::SetDeviceTransform(0, true,
			WireVector(request.desired.translationMeters),
			WireQuaternion(request.desired.rotation), request.desired.scale,
			request.desired.timeShift);
		desiredState.transform.generation = request.desired.baseGeneration;
		desiredState.field = request.field;

		if (result.enabled)
		{
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				SyncDevice device;
				device.id = id;
				if (enumerate)
					device = enumerate(id, request.desired);

				SlotDecision decision = DecideSlot(request.desired, device);
				result.referenceDeviceMask[id] = decision.referenceDevice;
				result.targetDeviceMask[id] = decision.targetDevice;
				if (decision.continuousTracker)
					result.continuousTrackerId = id;
				if (decision.disableProfile)
				{
					result.enabled = false;
					result.cause = DriverDisableCause::HmdMismatch;
				}
				if (decision.action != SlotAction::ApplyTransform)
					continue;
				desiredState.enabledMask |= uint64_t{ 1 } << id;
				if (decision.transform.hidden)
					desiredState.hiddenMask |= uint64_t{ 1 } << id;
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

		// Never let the jump/drift/continuous monitors infer that the live driver
		// matches the profile after a partial pipe failure. The next periodic scan
		// rebuilds and retries the complete desired state, including after
		// vrserver restarts; until then, all device identities are deliberately
		// unavailable.
		result.synchronized = driverSynchronized;
		if (!driverSynchronized)
		{
			result.enabled = false;
			result.cause = DriverDisableCause::DriverUnreachable;
			result.continuousTrackerId = vr::k_unTrackedDeviceIndexInvalid;
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				result.referenceDeviceMask[id] = false;
				result.targetDeviceMask[id] = false;
			}
		}
		else
		{
			if (clearError)
				clearError();
			lastErrorTime = -1e9;
		}
		return result;
	}

	// Retire one slot outside any batch: the pre-collection reset, which must
	// clear a stale transform off the device about to be sampled. No batch
	// generation to check, and deliberately no ledger update — this path does not
	// speak for what the profile left live, and a mark here would let a lost
	// response be forgotten without a confirmed disable behind it.
	bool DisableDeviceTransform(uint32_t id, double atTime)
	{
		now = atTime;
		return SendDisable(id, nullptr);
	}

private:
	// Every request in this file goes through here. A refused response and a
	// failed transport are one verdict — the requested state was not applied —
	// and share one 30 s debounce, so a dead pipe cannot rewrite the user's
	// banner 65 times a second.
	bool SendRequest(const protocol::Request &request, const char *operation,
		uint64_t *batchConnectionGeneration,
		protocol::Response *acceptedResponse = nullptr)
	{
		DriverTransportResult result;
		if (transport)
			result = transport(request);
		else
			result.error = "no driver transport is installed";
		if (result.completed)
		{
			bool accepted = result.response.type == protocol::ResponseSuccess ||
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
						// Accepted, but by a different driver connection than the one
						// this batch has been building state on. Not an error to
						// report — the caller reacts to the generation itself.
						return false;
					}
				}
				return true;
			}
			ReportThrottled(std::string("QuestCalibrator driver rejected ") + operation +
				"; the requested live state was not applied\n");
			return false;
		}
		ReportThrottled(std::string("QuestCalibrator driver communication failed while ") +
			operation + ": " + result.error + "\n");
		return false;
	}

	void ReportThrottled(const std::string &message)
	{
		if (now - lastErrorTime >= 30.0)
		{
			if (reportError)
				reportError(message);
			lastErrorTime = now;
		}
	}

	bool SendDisable(uint32_t id, uint64_t *batchConnectionGeneration)
	{
		protocol::Request req(protocol::RequestSetDeviceTransform);
		req.setDeviceTransform = protocol::SetDeviceTransform(id, false);
		return SendRequest(req, "disabling a device transform", batchConnectionGeneration);
	}

	DriverTransport transport;
	DriverDeviceEnumerator enumerate;
	std::function<void(const std::string &)> reportError;
	std::function<void()> clearError;

	// The transport's generation as of the last request, successful or not. Read
	// after a send to notice a reconnect the response itself does not announce.
	// Error debounce clock, in the caller's tick time. -1e9 means "re-armed":
	// the next failure reports immediately.
	double lastErrorTime = -1e9;
	// The caller's tick time for the request in flight, set at each entry point.
	double now = 0.0;
};

} // namespace questcal
