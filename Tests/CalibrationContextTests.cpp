#include "../Overlay/Calibration.h"

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
		ctx.profileHmdSerial.empty() && !ctx.lastResult.valid;
}
