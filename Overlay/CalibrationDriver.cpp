#include "stdafx.h"
#include "CalibrationDriver.h"

#include "Calibration.h"
#include "DriverSyncTracker.h"
#include "DriverWorker.h"
#include "FieldMath.h"
#include "IPCClient.h"
#include "ProfileValidation.h"

#include <algorithm>
#include <cmath>

namespace questcal
{
namespace
{

IPCClient Client;
DriverWorker Worker;
DriverSyncTracker Tracker;
std::optional<DriverNeutralizationResult> NeutralizationCompletion;

protocol::SetAlignmentField BuildAlignmentField(const CalibrationContext &ctx)
{
	protocol::SetAlignmentField field;
	const auto &anchors = ctx.ActiveFieldAnchors();
	field.enabled = ctx.enabled && !anchors.empty();
	field.generation = ctx.fieldGeneration;
	field.sigmaMeters = FieldBlendSigmaMeters;
	if (!field.enabled)
		return field;

	field.anchorCount = static_cast<uint32_t>((std::min)(
		anchors.size(), static_cast<size_t>(protocol::SetAlignmentField::MaxAnchors)));
	const Eigen::Quaterniond baseInverse = ctx.transform.rotation.conjugate();
	for (uint32_t i = 0; i < field.anchorCount; ++i)
	{
		const auto &anchor = anchors[i];
		Eigen::Quaterniond rotation;
		Eigen::Vector3d translation;
		AnchorDelta(anchor.rotation, anchor.translationMeters, baseInverse,
			ctx.transform.translationMeters, rotation, translation);
		for (int axis = 0; axis < 3; ++axis)
		{
			field.anchors[i].position[axis] = anchor.position(axis);
			field.anchors[i].translationDelta[axis] = translation(axis);
		}
		field.anchors[i].rotationDelta = WireQuaternion(rotation);
	}
	return field;
}

SyncDevice EnumerateDevice(uint32_t id, const DriverSyncDesired &desired)
{
	SyncDevice device;
	if (id >= vr::k_unMaxTrackedDeviceCount)
		return device;

	device.id = id;
	auto system = vr::VRSystem();
	if (!system)
		return device;

	switch (system->GetTrackedDeviceClass(id))
	{
	case vr::TrackedDeviceClass_Invalid:
		return device;
	case vr::TrackedDeviceClass_HMD:
		device.deviceClass = SyncDeviceClass::Hmd;
		break;
	default:
		device.deviceClass = SyncDeviceClass::Other;
		break;
	}

	device.trackingSystemKnown = ReadTrackedDeviceString(id,
		vr::Prop_TrackingSystemName_String, device.trackingSystem);
	if (SlotNeedsSerial(desired, device))
		device.serialKnown = ReadTrackedDeviceString(id,
			vr::Prop_SerialNumber_String, device.serial);
	return device;
}

void AssignDeviceIdentities(CalibrationContext &ctx, uint32_t continuousTrackerId,
	const bool (&referenceDeviceMask)[vr::k_unMaxTrackedDeviceCount],
	const bool (&targetDeviceMask)[vr::k_unMaxTrackedDeviceCount])
{
	ctx.continuousTrackerId = continuousTrackerId;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		ctx.referenceDeviceMask[id] = referenceDeviceMask[id];
		ctx.targetDeviceMask[id] = targetDeviceMask[id];
	}
}

void ClearDeviceIdentities(CalibrationContext &ctx)
{
	static const bool none[vr::k_unMaxTrackedDeviceCount] = {};
	AssignDeviceIdentities(ctx, vr::k_unTrackedDeviceIndexInvalid, none, none);
}

void ApplyCompletion(CalibrationContext &ctx,
	const DriverCompletion &completion)
{
	if (!completion.error.empty())
		ctx.ReportError(completion.error, CalibrationContext::ErrorSource::Driver);
	else if (completion.clearError)
		ctx.ClearError(CalibrationContext::ErrorSource::Driver);

	const auto &result = completion.result;
	ctx.enabled = result.enabled;
	ctx.driverPoseHookMask = result.poseHookMask;
	switch (result.cause)
	{
	case DriverDisableCause::HmdMismatch:
		ctx.disableReason = CalibrationContext::DisableReason::HmdMismatch;
		break;
	case DriverDisableCause::DriverUnreachable:
		ctx.disableReason = CalibrationContext::DisableReason::DriverUnreachable;
		break;
	case DriverDisableCause::None:
		if (result.enabled)
			ctx.disableReason = CalibrationContext::DisableReason::None;
		break;
	}
	AssignDeviceIdentities(ctx, result.continuousTrackerId,
		result.referenceDeviceMask, result.targetDeviceMask);
}

} // namespace

bool ReadTrackedDeviceString(uint32_t id,
	vr::ETrackedDeviceProperty property, std::string &value)
{
	value.clear();
	auto system = vr::VRSystem();
	if (!system || id >= vr::k_unMaxTrackedDeviceCount)
		return false;

	// Only tracking-system names and serials cross this boundary; oversized
	// properties fail closed instead of allocating on every device scan.
	char buffer[256] = {};
	vr::ETrackedPropertyError error = vr::TrackedProp_Success;
	const uint32_t size = system->GetStringTrackedDeviceProperty(id, property,
		buffer, static_cast<uint32_t>(sizeof buffer), &error);
	if (error != vr::TrackedProp_Success || size <= 1 || size > sizeof buffer ||
		buffer[size - 1] != '\0')
		return false;

	value.assign(buffer, size - 1);
	return !value.empty();
}

bool ReadCurrentHmdIdentity(std::string &trackingSystem, std::string &serial)
{
	return ReadTrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd,
		vr::Prop_TrackingSystemName_String, trackingSystem) &&
		ReadTrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd,
			vr::Prop_SerialNumber_String, serial);
}

void StartCalibrationDriver()
{
	Tracker = DriverSyncTracker{};
	NeutralizationCompletion.reset();
	Worker.Start([](const protocol::Request &request)
	{
		DriverTransportResult result;
		try
		{
			result.response = Client.SendBlocking(request);
			result.completed = true;
		}
		catch (const std::exception &e)
		{
			result.error = e.what();
		}
		result.connectionGeneration = Client.ConnectionGeneration();
		return result;
	});
}

void StopCalibrationDriver()
{
	Worker.Stop();
}

void SynchronizeCalibrationDriver(CalibrationContext &ctx)
{
	ctx.enabled = ctx.validProfile && !ctx.profileUniverseUnsafe;
	ctx.disableReason = ctx.enabled
		? CalibrationContext::DisableReason::None
		: CalibrationContext::DisableReason::UniverseUnsafe;

	if (ctx.enabled && !IsValidTrackingSystemPair(
		ctx.referenceTrackingSystem, ctx.targetTrackingSystem))
	{
		ctx.ReportError(
			"The live profile has invalid tracking-system identities and was disabled before sending it to the driver\n");
		ctx.enabled = false;
		ctx.disableReason = CalibrationContext::DisableReason::InvalidIdentity;
	}

	if (ctx.enabled)
	{
		std::string hmdTrackingSystem;
		std::string hmdSerial;
		if (vr::VRSystem()->GetTrackedDeviceClass(vr::k_unTrackedDeviceIndex_Hmd) !=
				vr::TrackedDeviceClass_HMD ||
			!ReadCurrentHmdIdentity(hmdTrackingSystem, hmdSerial) ||
			hmdTrackingSystem != ctx.referenceTrackingSystem ||
			(!ctx.profileHmdSerial.empty() &&
				!ProfileHmdIdentityMatches(ctx.profileHmdSerial, hmdSerial)))
		{
			ctx.enabled = false;
			ctx.disableReason = CalibrationContext::DisableReason::HmdMismatch;
		}
	}

	double timeShift = 0.0;
	if (ctx.useManualTimeOffset)
		timeShift = ctx.manualTimeOffsetMs / 1000.0;
	else if (ctx.applyTimeOffset)
		timeShift = ComputeAppliedTimeOffset(ctx.transform.timeOffset);
	if (!std::isfinite(timeShift) ||
		std::abs(timeShift) > protocol::limits::MaxAbsTimeOffsetSeconds ||
		(ctx.enabled && !IsValidCalibrationTransform(ctx.transform.rotation,
			ctx.transform.translationMeters, ctx.transform.scale)))
	{
		ctx.ReportError(
			"The live calibration contains invalid numeric values and was disabled before sending it to the driver\n");
		ctx.enabled = false;
		ctx.disableReason = CalibrationContext::DisableReason::InvalidTransform;
		timeShift = 0.0;
	}
	ctx.appliedTimeOffset = timeShift;

	DriverStateJob job;
	job.request.enabled = ctx.enabled;
	auto &desired = job.request.desired;
	desired.referenceTrackingSystem = ctx.referenceTrackingSystem;
	desired.targetTrackingSystem = ctx.targetTrackingSystem;
	desired.rotation = ctx.transform.rotation;
	desired.translationMeters = ctx.transform.translationMeters;
	desired.scale = ctx.transform.scale;
	desired.timeShift = timeShift;
	desired.baseGeneration = ctx.baseGeneration;
	desired.continuousArmed = ctx.ContinuousArmed();
	desired.hideMountedTracker = ctx.hideMountedTracker;
	desired.continuousTrackerSerial = ctx.continuousTrackerSerial;
	job.request.field = BuildAlignmentField(ctx);
	job.time = ctx.timeLastTick;
	if (job.request.enabled)
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			job.devices[id] = EnumerateDevice(id, desired);

	// The identities the monitors steer by are read off the enumerated devices,
	// not off the driver's answer: derive them now, from the very devices the
	// worker ships, so a sync in flight never reads as a disabled profile and
	// the jump/drift/continuous monitors keep their history across the round
	// trip. The completion re-applies the same identities -- the same function
	// of the same devices -- and adds only the driver's verdict.
	ClearDeviceIdentities(ctx);
	if (job.request.enabled)
	{
		const DriverSlotState slots = DeriveDriverSlotState(desired,
			[&job](uint32_t id, const DriverSyncDesired &)
			{
				return id < job.devices.size() ? job.devices[id] : SyncDevice();
			});
		if (slots.hmdMismatch)
		{
			ctx.enabled = false;
			ctx.disableReason = CalibrationContext::DisableReason::HmdMismatch;
		}
		else
		{
			AssignDeviceIdentities(ctx, slots.continuousTrackerId,
				slots.referenceDeviceMask, slots.targetDeviceMask);
		}
	}

	const DriverStateSubmission submission = Worker.Submit(job);
	if (Tracker.NoteSubmission(submission.sequence, submission.stateChanged) &&
		ctx.enabled)
	{
		// The driver refused this exact state and has not been asked anything
		// different since: stay disabled rather than flip the profile back on
		// for the length of a round trip on every scan.
		ctx.enabled = false;
		ctx.disableReason = CalibrationContext::DisableReason::DriverUnreachable;
		ClearDeviceIdentities(ctx);
	}
}

void PollCalibrationDriver(CalibrationContext &ctx)
{
	DriverCompletion completion;
	while (Worker.Poll(completion))
	{
		if (completion.kind == DriverWorkKind::Neutralize)
		{
			if (!completion.error.empty())
				ctx.ReportError(completion.error,
					CalibrationContext::ErrorSource::Driver);
			else if (completion.clearError)
				ctx.ClearError(CalibrationContext::ErrorSource::Driver);
			NeutralizationCompletion = DriverNeutralizationResult{
				completion.sequence, completion.succeeded };
		}
		else if (Tracker.NoteVerdict(completion.sequence,
			completion.result.synchronized))
		{
			// Only the latest submission's verdict is taken; an older one
			// answers a state that is no longer on the wire.
			ApplyCompletion(ctx, completion);
		}
	}
}

std::optional<DriverNeutralizationResult> TakeCalibrationNeutralization(
	uint64_t sequence)
{
	if (!NeutralizationCompletion || NeutralizationCompletion->sequence != sequence)
		return std::nullopt;
	auto result = NeutralizationCompletion;
	NeutralizationCompletion.reset();
	return result;
}

uint64_t NeutralizeCalibrationDevices(
	const std::array<uint32_t, 2> &deviceIds, double time)
{
	return Worker.Neutralize(deviceIds, time);
}

void ReleaseCalibrationDeviceNeutralization()
{
	Worker.ReleaseNeutralization();
}

} // namespace questcal
