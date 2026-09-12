#include "stdafx.h"
#include "CalibrationSpace.h"

#include "Calibration.h"
#include "CalibrationDriver.h"
#include "ChaperoneMath.h"
#include "Configuration.h"
#include "JumpDetector.h"
#include "PoseStreamHub.h"
#include "ProfileValidation.h"
#include "RingPoseMath.h"
#include "../common/PoseChannel.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <memory>
#include <utility>
#include <vector>

namespace
{

enum class ChaperoneOwnerStatus
{
	Match,
	Mismatch,
	Unowned,
	Unavailable,
};

struct HmdUniverseObservation
{
	enum class State { Empty, EndpointOnly, Usable };

	State state = State::Empty;
	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };
	Eigen::Vector3d translation{ 0, 0, 0 };
	double sampleTime = -1e9;
	double captureTime = -1e9;
	ringpose::DriverLocalPoseSample localPose;

	bool HasEndpoint() const noexcept { return state != State::Empty; }
	bool IsUsable() const noexcept { return state == State::Usable; }
	void Reset() noexcept { *this = HmdUniverseObservation{}; }

	void BreakContinuity() noexcept
	{
		if (HasEndpoint())
			state = State::EndpointOnly;
	}

	void Accept(const Eigen::Quaterniond &acceptedRotation,
		const Eigen::Vector3d &acceptedTranslation, double acceptedSampleTime,
		double acceptedCaptureTime,
		const ringpose::DriverLocalPoseSample &acceptedLocalPose)
	{
		state = State::Usable;
		rotation = acceptedRotation;
		translation = acceptedTranslation;
		sampleTime = acceptedSampleTime;
		captureTime = acceptedCaptureTime;
		localPose = acceptedLocalPose;
	}
};

struct SpaceState
{
	PoseStreamHub *poseHub = nullptr;
	double qpcToSeconds = 0.0;
	int consumer = -1;
	std::vector<protocol::DevicePoseSample> scratch;
	std::unique_ptr<JumpDetector> jumps;

	HmdUniverseObservation hmd;
	double mismatchSince = -1e9;
	double compensatedJumpAwaitingEndpoint = -1e9;

	double lastOwnerCheck = -1e9;
	std::string checkedTrackingSystem;
	std::string checkedHmdSerial;
	ChaperoneOwnerStatus owner = ChaperoneOwnerStatus::Unavailable;
	double lastSetupFailure = -1e9;
	double lastReadFailure = -1e9;

	bool VerdictPending() const noexcept { return mismatchSince >= 0.0; }
	void ClearVerdict() noexcept { mismatchSince = -1e9; }
	void ResetContinuity() noexcept
	{
		hmd.Reset();
		mismatchSince = -1e9;
		compensatedJumpAwaitingEndpoint = -1e9;
	}
};

struct HmdWorldTransition
{
	bool worldChanged = false;
	bool localPoseContinuous = false;
	Eigen::Quaterniond previousRotation{ 1, 0, 0, 0 };
	Eigen::Vector3d previousTranslation{ 0, 0, 0 };
	Eigen::Quaterniond currentRotation{ 1, 0, 0, 0 };
	Eigen::Vector3d currentTranslation{ 0, 0, 0 };
};

SpaceState Space;
constexpr double UniverseVerdictGraceSeconds = 0.5;
constexpr float ChaperoneCompareTolerance = 0.002f;

bool CacheHmdWorldFromDriver(const protocol::DevicePoseSample &sample,
	HmdWorldTransition &transition)
{
	transition = HmdWorldTransition{};
	if (sample.deviceId != vr::k_unTrackedDeviceIndex_Hmd ||
		!IsTrustedRingSample(sample, Space.qpcToSeconds))
		return false;

	const double sampleTime = RingSampleTime(sample, Space.qpcToSeconds);
	if (Space.hmd.HasEndpoint() && sampleTime <= Space.hmd.sampleTime)
		return false;

	const RingSampleParts parts = UnpackRingSample(sample);
	ringpose::DriverLocalPoseSample localPose;
	localPose.time = sampleTime;
	localPose.rotation = parts.drvRot;
	localPose.position = parts.drvPos;
	localPose.velocity = Eigen::Vector3d(
		sample.velocity[0], sample.velocity[1], sample.velocity[2]);
	localPose.angularVelocity = Eigen::Vector3d(
		sample.angularVelocity[0], sample.angularVelocity[1],
		sample.angularVelocity[2]);

	if (Space.hmd.HasEndpoint() && questcal::WorldFromDriverChanged(
		Space.hmd.rotation, Space.hmd.translation, parts.wfdRot, parts.wfdTrans))
	{
		transition.worldChanged = true;
		transition.previousRotation = Space.hmd.rotation;
		transition.previousTranslation = Space.hmd.translation;
		transition.currentRotation = parts.wfdRot;
		transition.currentTranslation = parts.wfdTrans;
		transition.localPoseContinuous = Space.hmd.IsUsable() &&
			ringpose::IsDriverLocalPoseContinuous(Space.hmd.localPose, localPose);
	}

	Space.hmd.Accept(parts.wfdRot, parts.wfdTrans, sampleTime,
		RingCaptureTime(sample, Space.qpcToSeconds), localPose);
	return true;
}

bool HasFreshHmdWorldFromDriver()
{
	LARGE_INTEGER qpcNow;
	if (!QueryPerformanceCounter(&qpcNow))
		return false;
	const double sampleClockNow =
		static_cast<double>(qpcNow.QuadPart) * Space.qpcToSeconds;
	return Space.poseHub->RingOpen() && Space.hmd.IsUsable() &&
		ringpose::IsFreshCaptureTime(Space.hmd.captureTime, sampleClockNow, 2.0);
}

bool ChaperoneBaselineIsCurrent(const CalibrationContext::Chaperone &snapshot)
{
	return snapshot.worldFromDriverValid &&
		snapshot.baselineVerifiedThisSession && HasFreshHmdWorldFromDriver() &&
		!questcal::WorldFromDriverChanged(
			snapshot.worldFromDriverRotation, snapshot.worldFromDriverTranslation,
			Space.hmd.rotation, Space.hmd.translation);
}

bool CopyCurrentHmdWorldFromDriver(CalibrationContext::Chaperone &snapshot)
{
	if (!HasFreshHmdWorldFromDriver())
		return false;
	snapshot.worldFromDriverRotation = Space.hmd.rotation;
	snapshot.worldFromDriverTranslation = Space.hmd.translation;
	snapshot.worldFromDriverValid = true;
	snapshot.baselineVerifiedThisSession = true;
	return true;
}

bool LiveGeometryMatches(vr::IVRChaperoneSetup *setup,
	const std::vector<vr::HmdQuad_t> &snapshot, uint32_t &liveQuadCount,
	bool &readOk)
{
	readOk = false;
	liveQuadCount = 0;
	uint32_t quadCount = 0;
	if (!setup->GetLiveCollisionBoundsInfo(nullptr, &quadCount))
		return false;
	liveQuadCount = quadCount;
	if (quadCount != snapshot.size())
	{
		readOk = true;
		return false;
	}

	std::vector<vr::HmdQuad_t> live(quadCount);
	uint32_t returnedCount = quadCount;
	if (!setup->GetLiveCollisionBoundsInfo(live.data(), &returnedCount))
		return false;
	if (returnedCount != quadCount)
	{
		liveQuadCount = returnedCount;
		return false;
	}
	readOk = true;
	return questcal::QuadsMatch(live, snapshot, ChaperoneCompareTolerance);
}

ChaperoneOwnerStatus CurrentChaperoneOwner(
	const CalibrationContext::Chaperone &snapshot)
{
	if (snapshot.ownerTrackingSystem.empty() || snapshot.ownerHmdSerial.empty())
		return ChaperoneOwnerStatus::Unowned;

	std::string trackingSystem;
	std::string serial;
	if (!questcal::ReadCurrentHmdIdentity(trackingSystem, serial))
		return ChaperoneOwnerStatus::Unavailable;
	return trackingSystem == snapshot.ownerTrackingSystem &&
		serial == snapshot.ownerHmdSerial
		? ChaperoneOwnerStatus::Match : ChaperoneOwnerStatus::Mismatch;
}

void DisarmChaperoneAndPersist(CalibrationContext &ctx, double now,
	const char *reason)
{
	ctx.DisarmChaperone();
	ctx.persistence.MarkSettings(now);
	ctx.ReportError(reason, CalibrationContext::ErrorSource::Chaperone);
	SaveSettings(ctx);
}

void ProfileUniverseTick(CalibrationContext &ctx, double now);
void DrainJumpObservations(CalibrationContext &ctx);
bool ApplyUniverseDelta(CalibrationContext &ctx,
	const JumpDetector::UniverseDelta &delta, double now);

} // namespace

namespace questcal
{

void StartCalibrationSpace(PoseStreamHub &poseHub, double qpcToSeconds)
{
	Space = SpaceState{};
	Space.poseHub = &poseHub;
	Space.qpcToSeconds = qpcToSeconds;
	Space.consumer = poseHub.CreateConsumer();
	Space.jumps = std::make_unique<JumpDetector>(qpcToSeconds);
}

void StopCalibrationSpace()
{
	Space = SpaceState{};
}

void CheckProtectedChaperone(CalibrationContext &ctx)
{
	const bool shouldCheck = ctx.chaperone.valid && ctx.chaperone.autoApply &&
		!ctx.chaperone.geometry.empty() &&
		ChaperoneBaselineIsCurrent(ctx.chaperone);
	if (!shouldCheck)
	{
		ctx.ClearError(CalibrationContext::ErrorSource::ChaperoneMonitor);
		return;
	}

	auto setup = vr::VRChaperoneSetup();
	if (!setup)
	{
		if (ctx.timeLastTick - Space.lastSetupFailure >= 30.0)
		{
			ctx.ReportError(
				"Couldn't check the protected chaperone: SteamVR's chaperone setup isn't available right now.\n",
				CalibrationContext::ErrorSource::ChaperoneMonitor);
			Space.lastSetupFailure = ctx.timeLastTick;
		}
		return;
	}
	Space.lastSetupFailure = -1e9;

	uint32_t quadCount = 0;
	bool liveRead = false;
	const bool differs = !LiveGeometryMatches(
		setup, ctx.chaperone.geometry, quadCount, liveRead);
	if (!liveRead)
	{
		if (ctx.timeLastTick - Space.lastReadFailure >= 30.0)
		{
			ctx.ReportError(
				"Couldn't read the current chaperone, so the protected one wasn't changed.\n",
				CalibrationContext::ErrorSource::ChaperoneMonitor);
			Space.lastReadFailure = ctx.timeLastTick;
		}
		return;
	}
	Space.lastReadFailure = -1e9;
	ctx.ClearError(CalibrationContext::ErrorSource::ChaperoneMonitor);

	if (differs && ctx.timeLastTick - ctx.chaperone.lastRestoreTime >= 5.0)
	{
		ctx.chaperone.lastRestoreTime = ctx.timeLastTick;
		if (ApplyChaperoneBounds(false))
		{
			char message[128];
			snprintf(message, sizeof message,
				"Chaperone changed outside the app (%u live / %zu saved walls); restored the protected bounds\n",
				quadCount, ctx.chaperone.geometry.size());
			ctx.Log(message);
		}
	}
}

void CalibrationSpaceTick(CalibrationContext &ctx, double now)
{
	std::vector<HmdWorldTransition> transitions;
	if (!Space.poseHub->RingOpen())
	{
		Space.ResetContinuity();
		ctx.chaperone.baselineVerifiedThisSession = false;
	}
	else
	{
		const uint64_t dropped = Space.poseHub->Drain(Space.consumer, Space.scratch);
		if (dropped > 0)
		{
			ctx.chaperone.baselineVerifiedThisSession = false;
			Space.hmd.BreakContinuity();
		}
		for (const auto &sample : Space.scratch)
		{
			HmdWorldTransition transition;
			if (!CacheHmdWorldFromDriver(sample, transition))
			{
				if (sample.deviceId == vr::k_unTrackedDeviceIndex_Hmd)
				{
					Space.hmd.BreakContinuity();
					ctx.chaperone.baselineVerifiedThisSession = false;
				}
				continue;
			}
			if (transition.worldChanged)
			{
				if (!transition.localPoseContinuous)
					ctx.chaperone.baselineVerifiedThisSession = false;
				transitions.push_back(std::move(transition));
			}
		}
		if (!HasFreshHmdWorldFromDriver())
			ctx.chaperone.baselineVerifiedThisSession = false;
	}

	ProfileUniverseTick(ctx, now);
	if (!ctx.chaperone.valid)
	{
		Space.owner = ChaperoneOwnerStatus::Unavailable;
		Space.checkedTrackingSystem.clear();
		Space.checkedHmdSerial.clear();
		Space.lastOwnerCheck = -1e9;
	}
	else if (ctx.chaperone.ownerTrackingSystem != Space.checkedTrackingSystem ||
		ctx.chaperone.ownerHmdSerial != Space.checkedHmdSerial ||
		now - Space.lastOwnerCheck >= 1.0)
	{
		Space.checkedTrackingSystem = ctx.chaperone.ownerTrackingSystem;
		Space.checkedHmdSerial = ctx.chaperone.ownerHmdSerial;
		Space.owner = CurrentChaperoneOwner(ctx.chaperone);
		Space.lastOwnerCheck = now;
	}

	if (ctx.chaperone.valid && Space.owner == ChaperoneOwnerStatus::Mismatch)
	{
		DisarmChaperoneAndPersist(ctx, now,
			"The protected chaperone was saved for a different headset, so it was switched off. "
			"Press Protect chaperone again on the main screen.\n");
		return;
	}
	if (ctx.chaperone.valid && Space.owner == ChaperoneOwnerStatus::Unowned)
	{
		DisarmChaperoneAndPersist(ctx, now,
			"The protected chaperone is missing information about the room it was saved in, so it was switched off. "
			"Press Protect chaperone again on the main screen.\n");
		return;
	}
	if (!ctx.chaperone.valid || Space.owner != ChaperoneOwnerStatus::Match ||
		!HasFreshHmdWorldFromDriver())
		return;
	if (!ctx.chaperone.worldFromDriverValid)
	{
		DisarmChaperoneAndPersist(ctx, now,
			"The protected chaperone is missing information about the room it was saved in, so it was switched off. "
			"Press Protect chaperone again on the main screen.\n");
		return;
	}

	if (!WorldFromDriverChanged(
		ctx.chaperone.worldFromDriverRotation,
		ctx.chaperone.worldFromDriverTranslation,
		Space.hmd.rotation, Space.hmd.translation))
	{
		ctx.chaperone.baselineVerifiedThisSession = true;
		return;
	}

	if (ctx.validProfile)
	{
		if (Space.VerdictPending())
		{
			ctx.chaperone.baselineVerifiedThisSession = false;
			return;
		}
		const bool autoApplyChanged = ctx.chaperone.autoApply;
		ctx.chaperone.autoApply = false;
		ctx.chaperone.baselineVerifiedThisSession = false;
		if (autoApplyChanged)
		{
			ctx.persistence.MarkSettings(now);
			ctx.ReportError(
				"The headset re-centred, so the protected chaperone no longer lines up and was turned off. "
				"Protect it again from the main screen.\n",
				CalibrationContext::ErrorSource::Chaperone);
			SaveSettings(ctx);
		}
		return;
	}

	bool reanchored = false;
	bool continuityLost = false;
	double accumulatedShift = 0.0;
	for (const auto &transition : transitions)
	{
		if (WorldFromDriverChanged(
			ctx.chaperone.worldFromDriverRotation,
			ctx.chaperone.worldFromDriverTranslation,
			transition.previousRotation, transition.previousTranslation))
			continue;
		if (!transition.localPoseContinuous)
		{
			continuityLost = true;
			break;
		}

		Eigen::Quaterniond deltaRotation;
		Eigen::Vector3d deltaTranslation;
		if (!WorldFromDriverDelta(
			transition.previousRotation, transition.previousTranslation,
			transition.currentRotation, transition.currentTranslation,
			deltaRotation, deltaTranslation))
		{
			continuityLost = true;
			break;
		}

		ctx.chaperone.standingCenter = DeltaTimesPose(
			deltaRotation, deltaTranslation, ctx.chaperone.standingCenter);
		ctx.chaperone.worldFromDriverRotation =
			transition.currentRotation.normalized();
		ctx.chaperone.worldFromDriverTranslation = transition.currentTranslation;
		ctx.chaperone.worldFromDriverValid = true;
		ctx.chaperone.baselineVerifiedThisSession = true;
		accumulatedShift += deltaTranslation.norm();
		reanchored = true;
	}

	if (continuityLost || WorldFromDriverChanged(
		ctx.chaperone.worldFromDriverRotation,
		ctx.chaperone.worldFromDriverTranslation,
		Space.hmd.rotation, Space.hmd.translation))
	{
		DisarmChaperoneAndPersist(ctx, now,
			"The headset re-centred while QuestCalibrator couldn't follow it, so the protected chaperone was switched off. "
			"Press Protect chaperone again on the main screen.\n");
		return;
	}
	if (!reanchored)
		return;

	ctx.persistence.AdvanceRevision();
	ctx.persistence.MarkSettings(now);
	char message[192];
	snprintf(message, sizeof message,
		"Protected chaperone re-anchored after HMD universe rebase (shift %.3f m)\n",
		accumulatedShift);
	ctx.Log(message);
	if (SaveSettings(ctx))
		ctx.ClearError(CalibrationContext::ErrorSource::Chaperone);
	else
	{
		ctx.chaperone.autoApply = false;
		ctx.chaperone.baselineVerifiedThisSession = false;
		ctx.persistence.MarkSettings(now);
	}
}

void ObserveUniversePose(const protocol::DevicePoseSample &sample)
{
	Space.jumps->Push(sample);
}

void ResetUniverseObservations(CalibrationContext &ctx)
{
	DrainJumpObservations(ctx);
	Space.jumps->Reset();
}

bool FinishUniverseObservations(CalibrationContext &ctx, double now)
{
	DrainJumpObservations(ctx);
	JumpDetector::UniverseDelta delta;
	bool jumped = false;
	while (Space.jumps->PollDelta(delta))
		jumped |= ApplyUniverseDelta(ctx, delta, now);
	return jumped;
}

void RebindCalibrationUniverse(CalibrationContext &ctx,
	const std::string &hmdSerial, bool priorUniverseUnsafe)
{
	ctx.profileUniverseUnsafe = false;
	ctx.profileUniverseValid = false;
	ctx.profileHmdSerial = hmdSerial;
	std::string currentHmdSerial;
	if (HasFreshHmdWorldFromDriver() &&
		ReadTrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd,
			vr::Prop_SerialNumber_String, currentHmdSerial) &&
		ProfileHmdIdentityMatches(ctx.profileHmdSerial, currentHmdSerial))
	{
		ctx.profileWorldFromDriverRotation = Space.hmd.rotation;
		ctx.profileWorldFromDriverTranslation = Space.hmd.translation;
		ctx.profileUniverseValid = true;
	}

	if (ctx.chaperone.valid &&
		(ctx.chaperone.ownerTrackingSystem != ctx.referenceTrackingSystem ||
			priorUniverseUnsafe || !ChaperoneBaselineIsCurrent(ctx.chaperone)))
	{
		ctx.DisarmChaperone();
		ctx.ReportError(
			"The protected chaperone belonged to the previous calibration and was turned off. "
			"Protect it again from the main screen.\n",
			CalibrationContext::ErrorSource::Chaperone);
	}
}

bool ApplyCalibrationDelta(CalibrationContext &ctx,
	const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation,
	bool snap, double now)
{
	if (!IsValidRotation(rotation) ||
		!IsBoundedVector(translation, protocol::limits::MaxAbsTranslationMeters))
		return false;

	const Eigen::Quaterniond newRotation =
		(rotation * ctx.transform.rotation).normalized();
	const Eigen::Vector3d newTranslation =
		rotation * ctx.transform.translationMeters + translation;
	if (!IsValidCalibrationTransform(
		newRotation, newTranslation, ctx.transform.scale))
		return false;

	for (const auto &anchor : ctx.fieldAnchors)
	{
		const Eigen::Vector3d position =
			rotation * anchor.position + translation;
		const Eigen::Quaterniond anchorRotation =
			(rotation * anchor.rotation).normalized();
		const Eigen::Vector3d anchorTranslation =
			rotation * anchor.translationMeters + translation;
		if (!IsValidFieldAnchor(position, anchorRotation, anchorTranslation,
			newRotation, newTranslation))
			return false;
	}

	vr::HmdMatrix34_t newStandingCenter{};
	if (snap && ctx.chaperone.valid)
	{
		newStandingCenter = DeltaTimesPose(
			rotation, translation, ctx.chaperone.standingCenter);
		if (!IsPlausibleChaperone(ctx.chaperone.geometry,
			newStandingCenter, ctx.chaperone.playSpaceSize))
			return false;
	}

	for (auto &anchor : ctx.fieldAnchors)
	{
		anchor.position = rotation * anchor.position + translation;
		anchor.rotation = (rotation * anchor.rotation).normalized();
		anchor.translationMeters =
			rotation * anchor.translationMeters + translation;
	}

	if (snap)
	{
		ctx.SetCalibration(newRotation, newTranslation, ctx.transform.scale);
		if (!ctx.fieldAnchors.empty())
			ctx.fieldGeneration++;
		ctx.persistence.AdvanceRevision();
		if (ctx.chaperone.valid)
			ctx.chaperone.standingCenter = newStandingCenter;
		ctx.persistence.MarkSettings(now);
	}
	else
	{
		ctx.SetCalibrationContinuous(
			newRotation, newTranslation, ctx.transform.scale);
	}

	ctx.persistence.MarkProfile(now);
	SynchronizeCalibrationDriver(ctx);
	return true;
}

} // namespace questcal

namespace
{

bool AdoptObservedUniverseAfterJump(CalibrationContext &ctx, double now)
{
	if (Space.hmd.sampleTime < Space.compensatedJumpAwaitingEndpoint ||
		Space.hmd.sampleTime - Space.compensatedJumpAwaitingEndpoint >
			UniverseVerdictGraceSeconds)
		return false;
	Space.compensatedJumpAwaitingEndpoint = -1e9;

	ctx.profileWorldFromDriverRotation = Space.hmd.rotation;
	ctx.profileWorldFromDriverTranslation = Space.hmd.translation;
	ctx.profileUniverseValid = true;
	ctx.persistence.MarkProfile(now);
	if (ctx.chaperone.valid)
	{
		ctx.chaperone.worldFromDriverRotation = Space.hmd.rotation;
		ctx.chaperone.worldFromDriverTranslation = Space.hmd.translation;
		ctx.chaperone.worldFromDriverValid = true;
		ctx.chaperone.baselineVerifiedThisSession = true;
		ctx.persistence.MarkSettings(now);
	}
	return true;
}

bool ApplyUniverseDelta(CalibrationContext &ctx,
	const JumpDetector::UniverseDelta &delta, double now)
{
	if (!questcal::ApplyCalibrationDelta(
		ctx, delta.rotation, delta.translation, true, now))
	{
		ctx.ReportError(
			"A headset re-centre was too large to compensate safely. Recalibrate before continuing.\n");
		return false;
	}

	if (delta.exact)
	{
		if (ctx.profileUniverseValid)
		{
			ctx.profileWorldFromDriverRotation =
				delta.worldFromDriverRotation.normalized();
			ctx.profileWorldFromDriverTranslation =
				delta.worldFromDriverTranslation;
		}
		if (ctx.chaperone.valid)
		{
			ctx.chaperone.worldFromDriverRotation =
				delta.worldFromDriverRotation.normalized();
			ctx.chaperone.worldFromDriverTranslation =
				delta.worldFromDriverTranslation;
			ctx.chaperone.worldFromDriverValid = true;
			ctx.chaperone.baselineVerifiedThisSession = true;
		}
	}
	else
	{
		Space.compensatedJumpAwaitingEndpoint = delta.time;
	}

	ctx.jumpsCompensated++;
	const double yawDegrees = 2.0 *
		std::atan2(delta.rotation.y(), delta.rotation.w()) * 180.0 / EIGEN_PI;
	// A jump seconds after the reference stream came back is the headset's
	// wake sequence as often as a moved universe (see recentResumeSeconds),
	// so the log line carries the age for whoever reads it later.
	char resumeNote[96] = "";
	if (delta.secondsSinceStreamResume >= 0.0 &&
		delta.secondsSinceStreamResume <= JumpDetector::Config().recentResumeSeconds)
		snprintf(resumeNote, sizeof resumeNote, ", %.1f s after the reference stream resumed",
			delta.secondsSinceStreamResume);
	// A controller that followed the headset's step seconds later confirmed a
	// held candidate (a Quest Pro map switch); the compensation is applied
	// that late, and the log says so.
	char lagNote[64] = "";
	if (delta.confirmationLagSeconds > 0.0)
		snprintf(lagNote, sizeof lagNote, ", confirmed %.1f s later",
			delta.confirmationLagSeconds);
	char message[256];
	snprintf(message, sizeof message,
		"Universe jump compensated (%s): yaw %+.2f deg, shift %.3f m, %d device(s)%s%s\n",
		delta.exact ? "exact" : "estimated", yawDegrees,
		delta.translation.norm(), delta.devicesAgreeing, resumeNote, lagNote);
	ctx.Log(message);
	return true;
}

void ProfileUniverseTick(CalibrationContext &ctx, double now)
{
	if (ctx.state != CalibrationState::None || !ctx.validProfile ||
		ctx.profileUniverseUnsafe)
	{
		Space.ClearVerdict();
		return;
	}
	if (!HasFreshHmdWorldFromDriver())
		return;

	const bool changed = !ctx.profileUniverseValid || questcal::WorldFromDriverChanged(
		ctx.profileWorldFromDriverRotation, ctx.profileWorldFromDriverTranslation,
		Space.hmd.rotation, Space.hmd.translation);
	if (!changed)
	{
		Space.ClearVerdict();
		return;
	}

	std::string trackingSystem;
	std::string serial;
	if (!questcal::ReadCurrentHmdIdentity(trackingSystem, serial) ||
		trackingSystem != ctx.referenceTrackingSystem)
		return;
	if (!ctx.profileHmdSerial.empty() && !questcal::ProfileHmdIdentityMatches(
		ctx.profileHmdSerial, serial))
	{
		Space.ClearVerdict();
		return;
	}

	if (!ctx.profileUniverseValid)
	{
		const bool fromSnapshot = ctx.chaperone.valid &&
			ctx.chaperone.worldFromDriverValid &&
			ctx.chaperone.ownerTrackingSystem == trackingSystem &&
			ctx.chaperone.ownerHmdSerial == serial;
		ctx.profileHmdSerial = serial;
		ctx.profileWorldFromDriverRotation = fromSnapshot
			? ctx.chaperone.worldFromDriverRotation : Space.hmd.rotation;
		ctx.profileWorldFromDriverTranslation = fromSnapshot
			? ctx.chaperone.worldFromDriverTranslation : Space.hmd.translation;
		ctx.profileUniverseValid = true;
		ctx.persistence.MarkProfile(now);
		Space.ClearVerdict();
		if (!fromSnapshot)
			return;
	}

	if (!questcal::WorldFromDriverChanged(
		ctx.profileWorldFromDriverRotation, ctx.profileWorldFromDriverTranslation,
		Space.hmd.rotation, Space.hmd.translation))
	{
		Space.ClearVerdict();
		return;
	}
	if (AdoptObservedUniverseAfterJump(ctx, now))
	{
		Space.ClearVerdict();
		return;
	}

	if (!Space.VerdictPending())
		Space.mismatchSince = now;
	if (now - Space.mismatchSince < UniverseVerdictGraceSeconds)
	{
		ctx.chaperone.baselineVerifiedThisSession = false;
		return;
	}
	Space.ClearVerdict();

	ctx.profileUniverseUnsafe = true;
	ctx.enabled = false;
	ctx.persistence.MarkProfile(now);
	const bool autoApplyChanged = ctx.chaperone.autoApply;
	ctx.chaperone.autoApply = false;
	ctx.chaperone.baselineVerifiedThisSession = false;
	if (autoApplyChanged)
		ctx.persistence.MarkSettings(now);
	ctx.ReportError(autoApplyChanged
		? "The headset re-centred while QuestCalibrator couldn't follow it. "
			"The calibration and the protected chaperone are off until you recalibrate.\n"
		: "The headset re-centred while QuestCalibrator couldn't follow it. "
			"The calibration is off until you recalibrate.\n",
		CalibrationContext::ErrorSource::Chaperone);
	questcal::SynchronizeCalibrationDriver(ctx);
	SavePendingChanges(ctx);
}

void DrainJumpObservations(CalibrationContext &ctx)
{
	std::string note;
	while (Space.jumps->PollNote(note))
		ctx.Log(note + "\n");

	JumpDetector::GapEvent gap;
	while (Space.jumps->PollGap(gap))
	{
		ctx.referenceGapEvents++;
		char message[256];
		snprintf(message, sizeof message,
			"Reference tracking gap (%.1f s) on device %u -- the universe may have moved; recalibrate if alignment looks off\n",
			gap.duration, gap.deviceId);
		ctx.Log(message);
	}
}

bool FailClosedChaperoneCapture(const std::string &message)
{
	const bool persistDisarm = CalCtx.chaperone.valid;
	if (persistDisarm)
	{
		CalCtx.chaperone.autoApply = false;
		CalCtx.chaperone.baselineVerifiedThisSession = false;
		CalCtx.persistence.MarkSettings(CalCtx.timeLastTick);
	}
	CalCtx.ReportError(message, CalibrationContext::ErrorSource::Chaperone);
	if (persistDisarm && !SaveSettings(CalCtx))
		CalCtx.persistence.MarkSettings(CalCtx.timeLastTick);
	return false;
}

} // namespace

bool LoadChaperoneBounds()
{
	if (CalCtx.profileUniverseUnsafe)
		return FailClosedChaperoneCapture(
			"Couldn't protect the chaperone:run a new base calibration first because the previous profile lost raw-universe continuity\n");

	auto setup = vr::VRChaperoneSetup();
	if (!setup)
		return FailClosedChaperoneCapture(
			"Couldn't protect the chaperone:OpenVR chaperone setup is unavailable\n");

	setup->RevertWorkingCopy();
	CalibrationContext::Chaperone snapshot;
	snapshot.autoApply = true;
	if (!questcal::ReadCurrentHmdIdentity(
		snapshot.ownerTrackingSystem, snapshot.ownerHmdSerial))
		return FailClosedChaperoneCapture(
			"Couldn't protect the chaperone:the current headset/runtime identity is unavailable\n");
	if (CalCtx.validProfile &&
		snapshot.ownerTrackingSystem != CalCtx.referenceTrackingSystem)
		return FailClosedChaperoneCapture(
			"Couldn't protect the chaperone:the active profile uses a different reference "
			"tracking system than the headset which owns this play area\n");
	if (!CopyCurrentHmdWorldFromDriver(snapshot))
		return FailClosedChaperoneCapture(
			"Couldn't protect the chaperone:no fresh validated HMD raw-universe pose is available yet\n");

	uint32_t quadCount = 0;
	if (!setup->GetLiveCollisionBoundsInfo(nullptr, &quadCount) || quadCount > 16384)
		return FailClosedChaperoneCapture(
			"Couldn't protect the chaperone:failed to read the live wall count\n");

	snapshot.geometry.resize(quadCount);
	uint32_t returnedCount = quadCount;
	vr::HmdQuad_t *walls = snapshot.geometry.empty()
		? nullptr : snapshot.geometry.data();
	if (!setup->GetLiveCollisionBoundsInfo(walls, &returnedCount) ||
		returnedCount != quadCount)
		return FailClosedChaperoneCapture(
			"Couldn't protect the chaperone:the live walls changed while they were being captured\n");

	if (!setup->GetWorkingStandingZeroPoseToRawTrackingPose(
			&snapshot.standingCenter) ||
		!setup->GetWorkingPlayAreaSize(
			&snapshot.playSpaceSize.v[0], &snapshot.playSpaceSize.v[1]) ||
		!questcal::IsPlausibleChaperone(
			snapshot.geometry, snapshot.standingCenter, snapshot.playSpaceSize))
		return FailClosedChaperoneCapture(
			"Couldn't protect the chaperone:play-area data is missing or invalid\n");

	const std::time_t copyTime = std::time(nullptr);
	const double copyUnixTime = static_cast<double>(copyTime);
	if (copyTime == static_cast<std::time_t>(-1) ||
		!std::isfinite(copyUnixTime) || copyUnixTime < 0.0 ||
		copyUnixTime > protocol::limits::MaxPlausibleUnixTimeSeconds)
		return FailClosedChaperoneCapture(
			"Couldn't protect the chaperone:the capture time is unavailable or invalid\n");

	snapshot.copyUnixTime = copyUnixTime;
	snapshot.valid = true;
	if (CalCtx.chaperone.valid)
	{
		CalCtx.chaperone.autoApply = false;
		CalCtx.chaperone.baselineVerifiedThisSession = false;
		CalCtx.persistence.MarkSettings(CalCtx.timeLastTick);
		if (!SaveSettings(CalCtx))
		{
			CalCtx.persistence.MarkSettings(CalCtx.timeLastTick);
			CalCtx.ReportError(
				"Couldn't protect the chaperone: the previous snapshot couldn't be turned off first, "
				"so nothing was changed.\n",
				CalibrationContext::ErrorSource::Chaperone);
			return false;
		}
	}

	CalCtx.chaperone = std::move(snapshot);
	if (!SaveSettings(CalCtx))
	{
		CalCtx.chaperone.autoApply = false;
		CalCtx.chaperone.baselineVerifiedThisSession = false;
		CalCtx.persistence.MarkSettings(CalCtx.timeLastTick);
		CalCtx.Log(
			"The new chaperone snapshot was kept disarmed because it could not be persisted\n");
		return false;
	}
	CalCtx.ClearError(CalibrationContext::ErrorSource::Chaperone);

	// Said on the main screen, where the button is: a protect that changed
	// nothing visible looked like a button that did nothing.
	char message[160];
	if (quadCount == 0)
		snprintf(message, sizeof message,
			"Chaperone saved without walls, so auto-restore stays off until Room Setup adds some.");
	else
		snprintf(message, sizeof message,
			"Chaperone saved: %u wall%s, %.1f x %.1f m.",
			quadCount, quadCount == 1 ? "" : "s", CalCtx.chaperone.playSpaceSize.v[0],
			CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.Tell(message, quadCount == 0 ? CalibrationContext::Tone::Warn : CalibrationContext::Tone::Good);
	return true;
}

bool ApplyChaperoneBounds(bool logSuccess)
{
	if (!CalCtx.chaperone.valid || !questcal::IsPlausibleChaperone(
		CalCtx.chaperone.geometry, CalCtx.chaperone.standingCenter,
		CalCtx.chaperone.playSpaceSize))
	{
		CalCtx.ReportError(
			"Couldn't restore the chaperone: the protected snapshot is damaged. Protect it again.\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}

	const ChaperoneOwnerStatus owner = CurrentChaperoneOwner(CalCtx.chaperone);
	if (owner == ChaperoneOwnerStatus::Unavailable)
	{
		CalCtx.ReportError(
			"Couldn't restore the chaperone: the headset isn't reporting to SteamVR yet.\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}
	if (owner == ChaperoneOwnerStatus::Mismatch)
	{
		DisarmChaperoneAndPersist(CalCtx, CalCtx.timeLastTick,
			"The protected chaperone was saved for a different headset, so it was switched off. "
			"Press Protect chaperone again on the main screen.\n");
		return false;
	}
	if (owner == ChaperoneOwnerStatus::Unowned)
	{
		DisarmChaperoneAndPersist(CalCtx, CalCtx.timeLastTick,
			"The protected chaperone is missing information about the room it was saved in, so it was switched off. "
			"Press Protect chaperone again on the main screen.\n");
		return false;
	}
	if (!ChaperoneBaselineIsCurrent(CalCtx.chaperone))
	{
		CalCtx.ReportError(
			"Couldn't restore the chaperone yet: the headset hasn't been tracked this session. Put it on and try again.\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}

	auto setup = vr::VRChaperoneSetup();
	if (!setup)
	{
		CalCtx.ReportError(
			"Couldn't restore the chaperone: SteamVR's chaperone setup isn't available right now.\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}

	setup->RevertWorkingCopy();
	if (!CalCtx.chaperone.geometry.empty())
		setup->SetWorkingCollisionBoundsInfo(CalCtx.chaperone.geometry.data(),
			static_cast<uint32_t>(CalCtx.chaperone.geometry.size()));
	setup->SetWorkingStandingZeroPoseToRawTrackingPose(
		&CalCtx.chaperone.standingCenter);
	setup->SetWorkingPlayAreaSize(
		CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	if (!setup->CommitWorkingCopy(vr::EChaperoneConfigFile_Live))
	{
		setup->RevertWorkingCopy();
		CalCtx.ReportError(
			"Couldn't restore the chaperone: SteamVR rejected the update.\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}

	bool verified = true;
	if (!CalCtx.chaperone.geometry.empty())
	{
		uint32_t liveQuadCount = 0;
		bool readOk = false;
		verified = LiveGeometryMatches(setup, CalCtx.chaperone.geometry,
			liveQuadCount, readOk);
	}
	setup->RevertWorkingCopy();
	vr::HmdMatrix34_t standing{};
	vr::HmdVector2_t size{};
	verified = verified &&
		setup->GetWorkingStandingZeroPoseToRawTrackingPose(&standing) &&
		setup->GetWorkingPlayAreaSize(&size.v[0], &size.v[1]);
	for (int row = 0; verified && row < 3; ++row)
		for (int column = 0; verified && column < 4; ++column)
			verified = std::abs(standing.m[row][column] -
				CalCtx.chaperone.standingCenter.m[row][column]) <=
				ChaperoneCompareTolerance;
	verified = verified &&
		std::abs(size.v[0] - CalCtx.chaperone.playSpaceSize.v[0]) <=
			ChaperoneCompareTolerance &&
		std::abs(size.v[1] - CalCtx.chaperone.playSpaceSize.v[1]) <=
			ChaperoneCompareTolerance;
	if (!verified)
	{
		CalCtx.ReportError(
			"Couldn't verify the restored chaperone. Check it in SteamVR Room Setup before relying on it.\n",
			CalibrationContext::ErrorSource::Chaperone);
		return false;
	}
	if (logSuccess)
		CalCtx.Tell("Chaperone restored.", CalibrationContext::Tone::Good);
	CalCtx.ClearError(CalibrationContext::ErrorSource::Chaperone);
	return true;
}
