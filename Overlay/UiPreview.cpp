// -uipreview: fake devices and fake calibration state, no SteamVR.
#include "stdafx.h"
#include "UiInternal.h"

// ---------------------------------------------------------------------------
// VR state
// ---------------------------------------------------------------------------

// Preview only: point fake devices at real SteamVR icon files when the local
// install has them (vector fallbacks otherwise).
std::string PreviewIconPath(const char *driverRelative)
{
	static std::string base;
	if (base.empty())
	{
		char buf[MAX_PATH] = {};
		DWORD len = sizeof buf;
		if (RegGetValueA(HKEY_CURRENT_USER, "Software\\Valve\\Steam", "SteamPath", RRF_RT_REG_SZ, nullptr, buf, &len) == ERROR_SUCCESS)
			base = std::string(buf) + "\\steamapps\\common\\SteamVR\\drivers\\";
		else
			base = "C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR\\drivers\\";
	}
	std::string path = base + driverRelative;
	return FileExists(path) ? path : std::string();
}

// The fake devices behind -uipreview / -uipreview-many.
VRState PreviewVRState()
{
	VRState state;
	state.trackingSystems = { "oculus", "lighthouse" };

	VRDevice hmd;
	hmd.id = 0;
	hmd.deviceClass = vr::TrackedDeviceClass_HMD;
	hmd.model = "Meta Quest Pro";
	hmd.serial = "1PASH5D1P17365";
	hmd.trackingSystem = "oculus";
	hmd.iconPath = PreviewIconPath("oculus\\resources\\icons\\quest_headset_ready_2x.png");
	hmd.tracking = true;
	state.devices.push_back(hmd);

	VRDevice touchPro;
	touchPro.id = 3 + kPreviewManyTrackerCount;
	touchPro.deviceClass = vr::TrackedDeviceClass_Controller;
	touchPro.model = "Touch Pro Right";
	touchPro.serial = "PREVIEW-TOUCH-PRO-RIGHT";
	touchPro.trackingSystem = "oculus";
	touchPro.controllerRole = vr::TrackedControllerRole_RightHand;
	touchPro.battery = 0.75f;
	touchPro.tracking = true;
	state.devices.push_back(touchPro);

	VRDevice right;
	right.id = 1;
	right.deviceClass = vr::TrackedDeviceClass_Controller;
	right.model = "Knuckles Right";
	right.serial = "LHR-A3C36EA5";
	right.trackingSystem = "lighthouse";
	right.controllerRole = vr::TrackedControllerRole_RightHand;
	right.battery = 0.80f;
	right.iconPath = PreviewIconPath("indexcontroller\\resources\\icons\\right_controller_status_ready_2x.png");
	right.tracking = true;
	state.devices.push_back(right);

	VRDevice left;
	left.id = 2;
	left.deviceClass = vr::TrackedDeviceClass_Controller;
	left.model = "Knuckles Left";
	left.serial = "LHR-841C98C3";
	left.trackingSystem = "lighthouse";
	left.controllerRole = vr::TrackedControllerRole_LeftHand;
	left.battery = 0.12f;
	left.iconPath = PreviewIconPath("indexcontroller\\resources\\icons\\left_controller_status_ready_low_2x.png");
	left.tracking = true;
	state.devices.push_back(left);

	int trackerCount = g_uiPreviewMany ? kPreviewManyTrackerCount : 1;
	for (int i = 0; i < trackerCount; ++i)
	{
		VRDevice tracker;
		tracker.id = 3 + i;
		tracker.deviceClass = vr::TrackedDeviceClass_GenericTracker;
		tracker.model = "VIVE Tracker 3.0";
		char serial[32];
		snprintf(serial, sizeof serial, "LHR-77E5A2%02X", 0x11 + i);
		tracker.serial = serial;
		tracker.trackingSystem = "lighthouse";
		// Demo the row states: the lone tracker sits disconnected; with
		// -uipreview-many they fan out across battery levels instead, the
		// last one staying disconnected to keep that row state on screen.
		tracker.connected = g_uiPreviewMany ? (i != kPreviewManyTrackerCount - 1) : false;
		tracker.tracking = tracker.connected;
		if (tracker.connected)
			tracker.battery = 0.95f - 0.12f * (float)i;
		const char *art =
			!tracker.connected ? "htc\\resources\\icons\\tracker_status_off_2x.png" :
			tracker.battery < kLowBattery ? "htc\\resources\\icons\\tracker_status_ready_low_2x.png" :
			"htc\\resources\\icons\\tracker_status_ready.png"; // no 2x ready ships
		tracker.iconPath = PreviewIconPath(art);
		state.devices.push_back(tracker);
	}

	return state;
}

// UI preview (-uipreview): plausible fake state so every part of the interface
// renders without SteamVR. Fake devices come from LoadVRState; profile saves
// are disabled while previewing.
void SetupPreviewState()
{
	CalCtx.validProfile = true;
	CalCtx.enabled = true;
	CalCtx.referenceTrackingSystem = "oculus";
	CalCtx.targetTrackingSystem = "lighthouse";

	// Two of the fake trackers carry player-given names so the named and
	// unnamed row treatments sit side by side.
	if (g_uiPreviewMany)
	{
		CalCtx.deviceNames["LHR-77E5A212"] = "Hip";
		CalCtx.deviceNames["LHR-77E5A213"] = "Left foot";
	}

	CalCtx.lastResult.valid = true;
	CalCtx.lastResult.rotationRmsDeg = 2.53;
	CalCtx.lastResult.translationRmsMeters = 0.010;
	CalCtx.lastResult.timeOffset = 0.0038;
	CalCtx.lastResult.scale = 1.002;
	CalCtx.transform.scale = 1.002;

	CalCtx.calibrationUnixTime = static_cast<double>(std::time(nullptr)) - 180.0;
	CalCtx.alignment = CalibrationContext::AlignmentHealth::Stale;
	CalCtx.driftScore = 0.70;
	CalCtx.driftSlideEvents = 21;
	CalCtx.driftMaxSlideM = 0.08;
	CalCtx.discontinuousLossEvents = 0;

	CalCtx.appliedTimeOffset = -0.0038;
	CalCtx.applyTimeOffset = true;
	CalCtx.fieldEnabled = true;
	CalCtx.chaperone.valid = true;
	CalCtx.chaperone.geometry.resize(26);
	CalCtx.chaperone.playSpaceSize.v[0] = 2.1f;
	CalCtx.chaperone.playSpaceSize.v[1] = 2.4f;
	CalCtx.chaperone.copyUnixTime = static_cast<double>(std::time(nullptr)) - 840.0;

	CalibrationContext::FieldAnchor anchor;
	anchor.position = Eigen::Vector3d(1.2, 1.1, -0.8);
	anchor.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.004, Eigen::Vector3d::UnitY()));
	anchor.translationMeters = Eigen::Vector3d(0.02, 0.0, -0.01);
	CalCtx.fieldAnchors.push_back(anchor);

	// Continuous calibration in its healthy maintaining state (the tracker
	// serial matches the first fake VIVE tracker in -uipreview-many).
	CalCtx.continuousEnabled = true;
	CalCtx.continuousTrackerSerial = "LHR-77E5A211";
	CalCtx.hideMountedTracker = true;
	CalCtx.mountExtrinsic.valid = true;
	CalCtx.mountExtrinsic.rot = Eigen::Quaterniond(
		Eigen::AngleAxisd(2.7, Eigen::Vector3d(0.2, 0.7, -0.3).normalized()));
	CalCtx.mountExtrinsic.pos = Eigen::Vector3d(0.05, -0.08, 0.03);
	CalCtx.mountExtrinsic.rotRmsDeg = 0.21;
	CalCtx.mountExtrinsic.posRmsM = 0.004;
	CalCtx.continuousState = questcal::ContinuousAlignment::State::Tracking;
	CalCtx.continuousDeviation.valid = true;
	CalCtx.continuousDeviation.yawDeg = 0.08;
	CalCtx.continuousDeviation.tiltDeg = 0.11;
	CalCtx.continuousDeviation.posM = 0.004;
	CalCtx.continuousScatterRotDeg = 0.19;
	CalCtx.continuousScatterPosM = 0.006;
	CalCtx.autoCorrectionsApplied = 14;
	CalCtx.lastAutoCorrectionUnixTime = static_cast<double>(std::time(nullptr)) - 42.0;

	switch (g_uiPreviewScenario)
	{
	case PreviewScenario::Guide:
	case PreviewScenario::Result:
		CalCtx.referenceID = 3 + kPreviewManyTrackerCount;
		CalCtx.targetID = 3;
		CalCtx.pendingReferenceTrackingSystem = "oculus";
		CalCtx.pendingTargetTrackingSystem = "lighthouse";
		OpenGuide(false, false);
		if (g_uiPreviewScenario == PreviewScenario::Result)
		{
			CalCtx.lastRunHint = CalibrationContext::GuideHint::Success;
			CalCtx.Outcome("Calibration complete", "Check that the tracker positions line up in VR.",
				"", "Rotation RMS 2.53 degrees; position RMS 1.0 cm", CalibrationContext::Tone::Good);
			s_guide.stage = GuideStage::Done;
		}
		break;
	case PreviewScenario::Frozen:
		// The loop measured a deviation too large to correct and stopped:
		// the band shows its two actions and the activity card the event.
		CalCtx.continuousState = questcal::ContinuousAlignment::State::Frozen;
		CalCtx.continuousDeviation.yawDeg = 2.6;
		CalCtx.continuousDeviation.tiltDeg = 0.9;
		CalCtx.continuousDeviation.posM = 0.11;
		CalCtx.Tell("Continuous calibration paused: readings drifted too far from the calibration to correct safely.",
			CalibrationContext::Tone::Warn);
		break;
	case PreviewScenario::Empty:
		// First launch: no profile, no chaperone, nothing measured.
		CalCtx.validProfile = false;
		CalCtx.enabled = false;
		CalCtx.lastResult.valid = false;
		CalCtx.fieldAnchors.clear();
		CalCtx.chaperone.valid = false;
		CalCtx.chaperone.geometry.clear();
		CalCtx.continuousEnabled = false;
		CalCtx.continuousTrackerSerial.clear();
		CalCtx.mountExtrinsic.valid = false;
		CalCtx.continuousState = questcal::ContinuousAlignment::State::Inactive;
		CalCtx.continuousDeviation.valid = false;
		CalCtx.autoCorrectionsApplied = 0;
		break;
	case PreviewScenario::Failed:
	case PreviewScenario::Healthy:
		break;
	}
}
