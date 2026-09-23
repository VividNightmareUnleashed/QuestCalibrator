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
	hmd.offIconPath = PreviewIconPath("oculus\\resources\\icons\\quest_headset_off_2x.png");
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
	touchPro.offIconPath = PreviewIconPath("oculus\\resources\\icons\\rifts_right_controller_off_2x.png");
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

	// Where everything stands, for the 3D View's feed: the player in the middle of
	// the play area facing forward, the left hand reaching out towards the
	// wall it is down to one station at, the trackers on the body.
	auto facingYaw = [](double degrees)
	{
		const double r = degrees * EIGEN_PI / 180.0;
		return Eigen::Vector3d(-std::sin(r), 0.0, -std::cos(r));
	};
	const std::map<std::string, Eigen::Vector3d> spots = {
		{ "1PASH5D1P17365", { 0.15, 1.62, 0.05 } },
		{ "PREVIEW-TOUCH-PRO-RIGHT", { 0.45, 1.20, -0.20 } },
		{ "LHR-A3C36EA5", { 0.50, 1.15, -0.25 } },
		{ "LHR-841C98C3", { -0.75, 1.30, 0.55 } },
		{ "LHR-77E5A211", { 0.15, 1.72, 0.12 } },
		{ "LHR-77E5A212", { 0.12, 1.00, 0.10 } },
		{ "LHR-77E5A213", { 0.02, 0.10, 0.12 } },
		{ "LHR-77E5A214", { 0.30, 0.10, 0.08 } },
		{ "LHR-77E5A215", { 0.72, 0.85, -0.55 } },
	};
	for (auto &dev : state.devices)
	{
		auto spot = spots.find(dev.serial);
		if (spot == spots.end() || !dev.connected)
			continue;
		dev.placed = true;
		dev.position = spot->second;
		dev.facing = facingYaw(dev.id == 0 ? 20.0 : 0.0);
	}

	// Four stations high in the corners of a 4.4 by 3.8 m room, each aimed at
	// a point a metre above the middle; the ids match the log lines below.
	struct PreviewStation { const char *serial; const char *mode; Eigen::Vector3d at; };
	const PreviewStation stations[] = {
		{ "LHB-D3D4E73B", "5", { -2.1, 2.25, -1.8 } },
		{ "LHB-170EE067", "8", { 2.2, 2.10, -1.7 } },
		{ "LHB-F210FBA6", "9", { 2.0, 2.30, 1.9 } },
		{ "LHB-04D47FB4", "16", { -1.9, 1.35, 1.8 } },
	};
	for (int i = 0; i < 4; ++i)
	{
		VRStation st;
		st.id = 20 + i;
		st.serial = stations[i].serial;
		st.modeLabel = stations[i].mode;
		st.iconPath = PreviewIconPath("lighthouse\\resources\\icons\\base2_status_ready_2x.png");
		st.connected = true;
		st.placed = true;
		st.position = stations[i].at;
		st.facing = (Eigen::Vector3d(0.0, 1.0, 0.0) - st.position).normalized();
		state.stations.push_back(st);
	}
	// The last station is mounted low and turned away from the middle.
	state.stations.back().facing = (Eigen::Vector3d(1.4, 0.9, -1.2) - state.stations.back().position).normalized();
	// -uipreview-lighthouse-conflict: a fifth station left on channel 8, so
	// two of the player's stations share it.
	if (g_uiPreviewScenario == PreviewScenario::LighthouseConflict)
	{
		VRStation extra = state.stations[1];
		extra.id = 24;
		extra.serial = "LHB-2A91C0D4";
		extra.position = Eigen::Vector3d(0.1, 2.2, -1.9);
		extra.facing = (Eigen::Vector3d(0.0, 1.0, 0.0) - extra.position).normalized();
		state.stations.push_back(extra);
	}
	// Upright mounts: no roll, so right is level and up follows the tilt.
	for (auto &st : state.stations)
	{
		st.right = st.facing.cross(Eigen::Vector3d::UnitY()).normalized();
		st.up = st.right.cross(st.facing).normalized();
	}

	// An irregular boundary around a 2.1 by 2.4 m play area.
	const ImVec2 outline[] = {
		{ -1.05f, -1.20f }, { 0.70f, -1.20f }, { 1.05f, -0.85f }, { 1.05f, 1.20f },
		{ -0.60f, 1.20f }, { -1.05f, 0.75f } };
	for (int k = 0; k < 6; ++k)
	{
		state.floorEdges.push_back(outline[k]);
		state.floorEdges.push_back(outline[(k + 1) % 6]);
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

	// Base station visibility as the lighthouse log would have reported it:
	// every device learned four stations, the left controller is down to
	// one (the red figure), one tracker lost one, and two stations carry
	// most of the drops so the line under the panes has an order.
	{
		using K = lighthouselog::Event::Kind;
		static const std::map<int, uint32_t> ids = {
			{ 5, 0xD3D4E73Bu }, { 8, 0x170EE067u }, { 9, 0xF210FBA6u }, { 16, 0x04D47FB4u } };
		auto line = [](K kind, const std::string &serial, int channel, std::vector<int> visible)
		{
			lighthouselog::Event e;
			e.kind = kind;
			e.serial = serial;
			e.channel = channel;
			e.stationId = ids.at(channel);
			e.visibleKnown = true;
			e.visibleChannels = std::move(visible);
			for (int c : e.visibleChannels)
				e.visibleIds.push_back(ids.at(c));
			e.historical = true;
			return e;
		};
		auto &vis = CalCtx.lighthouse;
		std::vector<std::string> serials = { "LHR-A3C36EA5", "LHR-841C98C3" };
		for (int i = 0; i < kPreviewManyTrackerCount; ++i)
			serials.push_back(FormatString("LHR-77E5A2%02X", 0x11 + i));
		for (const auto &s : serials)
		{
			vis.Apply(line(K::StationAdded, s, 5, { 5 }), 0.0);
			vis.Apply(line(K::StationAdded, s, 8, { 5, 8 }), 0.0);
			vis.Apply(line(K::StationAdded, s, 9, { 5, 8, 9 }), 0.0);
			vis.Apply(line(K::StationAdded, s, 16, { 5, 8, 9, 16 }), 0.0);
		}
		for (int n = 0; n < 7; ++n)
		{
			vis.Apply(line(K::StationDropped, "LHR-77E5A211", 16, { 5, 8, 9 }), 0.0);
			vis.Apply(line(K::StationAdded, "LHR-77E5A211", 16, { 5, 8, 9, 16 }), 0.0);
		}
		for (int n = 0; n < 3; ++n)
		{
			vis.Apply(line(K::StationDropped, "LHR-A3C36EA5", 5, { 8, 9, 16 }), 0.0);
			vis.Apply(line(K::StationAdded, "LHR-A3C36EA5", 5, { 5, 8, 9, 16 }), 0.0);
		}
		vis.Apply(line(K::StationDropped, "LHR-77E5A212", 16, { 5, 8, 9 }), 0.0);
		vis.Apply(line(K::StationDropped, "LHR-841C98C3", 16, { 5, 8, 9 }), 0.0);
		vis.Apply(line(K::StationDropped, "LHR-841C98C3", 8, { 5, 9 }), 0.0);
		vis.Apply(line(K::StationDropped, "LHR-841C98C3", 5, { 9 }), 0.0);
		// And one live drop half a second ago, so the tab shows a fresh
		// loss next to the ones that have lasted.
		LARGE_INTEGER frequency, counter;
		if (QueryPerformanceFrequency(&frequency) && QueryPerformanceCounter(&counter) && frequency.QuadPart != 0)
		{
			auto fresh = line(K::StationDropped, "LHR-A3C36EA5", 8, { 5, 9, 16 });
			fresh.historical = false;
			vis.Apply(fresh, static_cast<double>(counter.QuadPart) / static_cast<double>(frequency.QuadPart) - 0.5);
		}
		CalCtx.lighthouseLogAvailable = true;
		CalCtx.lighthouseLogPath = lighthouselog::DefaultLogPath();
	}

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
	case PreviewScenario::Lighthouse:
	case PreviewScenario::LighthouseConflict:
		s_mainTab = MainTab::Lighthouse;
		break;
	case PreviewScenario::Failed:
	case PreviewScenario::Healthy:
		break;
	}
}
