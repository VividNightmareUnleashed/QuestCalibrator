#include "../Overlay/Calibration.h"
#include "../Overlay/CalibrationDriver.h"

bool CalibrationEulerRoundTripScenario()
{
	for (double yaw : {-179.0, -90.0, -0.1, 0.0, 90.0, 179.0})
		for (double pitch : {-90.0, -89.9, 0.0, 89.9, 90.0})
		{
			CalibrationTransform transform;
			transform.rotation = CalibrationContext::RebuildRotationFromEuler(Eigen::Vector3d(yaw, pitch, 27.0));
			const auto rebuilt = CalibrationContext::RebuildRotationFromEuler(transform.RotationEulerDegrees());
			if (transform.rotation.angularDistance(rebuilt) > 1e-9)
				return false;
		}
	return true;
}

bool CalibrationContextResetScenario()
{
	CalibrationContext ctx;
	ctx.applyTimeOffset = false;
	ctx.solveScale = true;
	ctx.uiAdvanced = true;
	ctx.continuousEnabled = true;
	ctx.continuousTrackerSerial = "tracker";
	ctx.fieldEnabled = false;
	ctx.fieldGeneration = 9;
	ctx.baseGeneration = 11;
	ctx.pendingReferenceTrackingSystem = "pending";
	ctx.chaperone.valid = true;
	ctx.chaperone.geometry.push_back(vr::HmdQuad_t{});

	ctx.transform.translationMeters = Eigen::Vector3d(1, 2, 3);
	ctx.appliedTimeOffset = 0.02;
	ctx.fieldAnchors.push_back(CalibrationContext::FieldAnchor{});
	ctx.mountExtrinsic.valid = true;
	ctx.continuousTrackerId = 4;
	ctx.autoCorrectionsApplied = 3;
	ctx.jumpsCompensated = 2;
	ctx.driftScore = 0.7;
	ctx.referenceTrackingSystem = "reference";
	ctx.targetTrackingSystem = "target";
	ctx.enabled = true;
	ctx.validProfile = true;
	ctx.profileUniverseUnsafe = true;
	ctx.profileUniverseValid = true;
	ctx.profileHmdSerial = "hmd";
	ctx.lastResult.valid = true;
	ctx.continuousCorrectionGate.Offer({}, false);

	ctx.Clear();
	return !ctx.applyTimeOffset && ctx.solveScale && ctx.uiAdvanced &&
		ctx.continuousEnabled && ctx.continuousTrackerSerial == "tracker" &&
		!ctx.fieldEnabled && ctx.fieldGeneration == 10 &&
		ctx.baseGeneration == 11 &&
		ctx.pendingReferenceTrackingSystem == "pending" &&
		ctx.chaperone.valid && ctx.chaperone.geometry.size() == 1 &&
		ctx.transform.translationMeters.isZero() && ctx.appliedTimeOffset == 0.0 &&
		ctx.fieldAnchors.empty() && !ctx.mountExtrinsic.valid &&
		ctx.continuousTrackerId == vr::k_unTrackedDeviceIndexInvalid &&
		ctx.autoCorrectionsApplied == 0 && ctx.jumpsCompensated == 0 &&
		ctx.driftScore == 0.0 && ctx.referenceTrackingSystem.empty() &&
		ctx.targetTrackingSystem.empty() && !ctx.enabled && !ctx.validProfile &&
		!ctx.profileUniverseUnsafe && !ctx.profileUniverseValid &&
		ctx.profileHmdSerial.empty() && !ctx.lastResult.valid &&
		!ctx.continuousCorrectionGate.HasPending();
}

bool ControllerTriggerAxisScenario()
{
	vr::VRControllerState_t state{};
	std::array<int32_t, vr::k_unControllerStateAxisCount> types{};
	types[1] = vr::k_eControllerAxis_Trigger;
	types[3] = vr::k_eControllerAxis_Joystick;
	unsigned reads = 0;
	auto readAxisType = [&](vr::ETrackedDeviceProperty property)
	{
		++reads;
		return types.at(property - vr::Prop_Axis0Type_Int32);
	};
	if (questcal::ControllerTriggerPressed(state, readAxisType) || reads != 0)
		return false;

	state.rAxis[1].x = 1.0f;
	if (!questcal::ControllerTriggerPressed(state, readAxisType))
		return false;
	state.rAxis[1].x = 0.75f;
	state.rAxis[3].x = 1.0f;
	if (questcal::ControllerTriggerPressed(state, readAxisType))
		return false;

	types[1] = vr::k_eControllerAxis_None;
	types[4] = vr::k_eControllerAxis_Trigger;
	state.rAxis[4].x = 0.751f;
	if (!questcal::ControllerTriggerPressed(state, readAxisType))
		return false;

	return !questcal::ControllerTriggerPressed(state,
		[](vr::ETrackedDeviceProperty) { return vr::k_eControllerAxis_None; });
}

bool CalibrationContextCadenceScenario()
{
	CalibrationContext ctx;
	if (ctx.IdleUpdateInterval() != 1.0)
		return false;
	ctx.enabled = ctx.validProfile = ctx.poseRingOpen = true;
	ctx.continuousEnabled = true;
	ctx.continuousTrackerSerial = "tracker";
	ctx.continuousTrackerId = 3;
	ctx.referenceDeviceMask[vr::k_unTrackedDeviceIndex_Hmd] = true;
	ctx.continuousMode = ContinuousMode::Legacy;
	if (ctx.IdleUpdateInterval() != 0.05)
		return false;
	ctx.poseRingOpen = false;
	if (ctx.IdleUpdateInterval() != 0.05)
		return false;
	ctx.poseRingOpen = true;
	ctx.continuousTrackerSerial.clear();
	if (ctx.IdleUpdateInterval() != 0.05)
		return false;

	ctx.continuousMode = ContinuousMode::Quest;
	ctx.mountExtrinsic.valid = true;
	ctx.continuousRequireTrigger = true;
	if (ctx.IdleUpdateInterval() != 0.05)
		return false;
	ctx.continuousCorrectionGate.Offer({}, false);
	if (ctx.IdleUpdateInterval() != 0.05)
		return false;
	ctx.continuousRequireTrigger = false;
	if (ctx.IdleUpdateInterval() != 0.05)
		return false;
	ctx.continuousRequireTrigger = true;
	ctx.state = CalibrationState::Editing;
	if (ctx.IdleUpdateInterval() != 0.05)
		return false;
	ctx.state = CalibrationState::None;
	ctx.continuousCorrectionGate.Clear();
	ctx.continuousEnabled = false;
	if (ctx.IdleUpdateInterval() != 0.05)
		return false;
	ctx.enabled = false;
	if (ctx.IdleUpdateInterval() != 1.0)
		return false;
	ctx.chaperone.valid = true;
	if (ctx.IdleUpdateInterval() != 0.05)
		return false;
	ctx.chaperone.autoApply = false;
	return ctx.IdleUpdateInterval() == 1.0;
}

bool CalibrationContextCorrectionBasisScenario()
{
	CalibrationContext ctx;
	questcal::ContinuousAlignment::Correction correction;
	correction.translation = Eigen::Vector3d(0.01, 0, 0);
	ctx.continuousCorrectionGate.Offer(correction, false);
	ctx.SetCalibration(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1, 0, 0), 1.0);
	if (ctx.continuousCorrectionGate.HasPending() ||
		ctx.continuousCorrectionGate.Take(true, true, correction) || ctx.baseGeneration != 1)
		return false;

	ctx.continuousCorrectionGate.Offer(correction, false);
	ctx.SetCalibrationContinuous(Eigen::Quaterniond::Identity(), Eigen::Vector3d(2, 0, 0), 1.0);
	return !ctx.continuousCorrectionGate.HasPending() &&
		!ctx.continuousCorrectionGate.Take(true, true, correction) && ctx.baseGeneration == 1;
}
