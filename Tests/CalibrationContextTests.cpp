#include "../Overlay/Calibration.h"
#include "../Overlay/CalibrationDriver.h"
#include "../Overlay/Diagnostics.h"
#include <filesystem>
#include <fstream>
#include <sstream>

bool DiagnosticsExportScenario()
{
	const auto root = std::filesystem::temp_directory_path() /
		(L"QuestCalDiagnosticsTest-" + std::to_wstring(GetCurrentProcessId()) + L"-" + std::to_wstring(GetTickCount64()));
	std::filesystem::create_directory(root);
	const DWORD length = GetEnvironmentVariableW(L"LOCALAPPDATA", nullptr, 0);
	std::wstring previous(length, L'\0');
	if (length)
	{
		GetEnvironmentVariableW(L"LOCALAPPDATA", previous.data(), length);
		previous.resize(length - 1);
	}
	struct RestoreEnvironment
	{
		const std::wstring &previous;
		bool existed;
		~RestoreEnvironment() { SetEnvironmentVariableW(L"LOCALAPPDATA", existed ? previous.c_str() : nullptr); }
	} restore{ previous, length != 0 };
	if (!SetEnvironmentVariableW(L"LOCALAPPDATA", root.c_str()))
		return false;
	CalibrationContext ctx;
	ctx.lastResult.valid = true;
	ctx.lastResult.scaleCondition = 0.0001;
	ctx.mountExtrinsic.valid = true;
	DiagnosticCapture capture;
	capture.poseStream.devices[16].received = 1;
	capture.poseStream.devices[16].latest.deviceId = 16;
	capture.poseStream.devices[16].latest.position[1] = -0.25;
	std::string path, error;
	const bool saved = WriteDiagnosticsFile(ctx, path, error, nullptr, capture);
	std::string report;
	if (saved)
	{
		std::ifstream file(std::filesystem::u8path(path), std::ios::binary);
		std::ostringstream contents;
		contents << file.rdbuf();
		report = contents.str();
		file.close();
		std::filesystem::remove(std::filesystem::u8path(path));
	}
	// Only remove the exact directories created by this fixture, never recursively.
	std::filesystem::remove(root / L"QuestCalibrator" / L"diagnostics");
	std::filesystem::remove(root / L"QuestCalibrator");
	std::filesystem::remove(root);
	return saved && error.empty() && report.find("SHA-256: unavailable") == std::string::npos &&
		report.find("[driver synchronization]") != std::string::npos &&
		report.find("scale identifiable: off, condition 0.0001") != std::string::npos &&
		report.find("mount rotation (w x y z)") != std::string::npos &&
		report.find("device 16: received 1") != std::string::npos &&
		report.find("local position (m) 0 -0.25 0") != std::string::npos &&
		report.find("[runtime poses at export]\nSteamVR unavailable") != std::string::npos;
}

bool ContinuousInputDiagnosticsScenario()
{
	CalibrationContext ctx;
	ctx.continuousTrackerId = 9;
	auto &input = ctx.continuousDiagnostics;
	protocol::DevicePoseSample raw{};
	raw.deviceId = 0;
	raw.sampleTimeQpc = 10000;
	raw.deviceIsConnected = raw.poseIsValid = true;
	raw.trackingResult = vr::TrackingResult_Running_OK;
	raw.rotation.w = raw.worldFromDriverRotation.w = 1.0;
	questcal::PoseSample composed;
	auto &device = input.devices[0];
	if (!device.Compose(raw, 0.001, composed))
		return false;
	raw.sampleTimeQpc = 10010;
	raw.poseIsValid = false;
	if (device.Compose(raw, 0.001, composed))
		return false;
	raw.poseIsValid = true;
	raw.rotation.w = 0.0;
	if (device.Compose(raw, 0.001, composed) || device.received != 3 ||
		device.accepted != 1 || device.trackingRejected != 1 || device.numericRejected != 1)
		return false;
	const std::string report = DescribeContinuousDiagnostics(ctx, 10.02);
	if (report.find("device 0: received 3, accepted 1, rejected tracking 1 / numeric 1") == std::string::npos ||
		report.find("device 9: received 0, accepted 0") == std::string::npos ||
		report.find("accepted capture age 20.0 ms") == std::string::npos)
		return false;
	ctx.Clear();
	ctx.continuousMode = ContinuousMode::Legacy;
	const std::string afterReset = DescribeContinuousDiagnostics(ctx, 20.0);
	return input.devices[0].received == 3 &&
		afterReset.find("method: legacy") != std::string::npos &&
		afterReset.find("accepted capture age 10000.0 ms") != std::string::npos &&
		afterReset.find("valid solve off") != std::string::npos;
}

bool PoseStreamDiagnosticsScenario()
{
	PoseStreamHub hub;
	const int consumer = hub.CreateConsumer();
	protocol::DevicePoseSample raw{};
	raw.deviceId = 9;
	raw.sampleTimeQpc = 10000;
	hub.AppendSampleForTest(raw);
	raw.deviceId = 16;
	raw.position[1] = -0.25;
	hub.AppendSampleForTest(raw);
	hub.AppendGapForTest(7);
	raw.sampleTimeQpc = 10010;
	hub.AppendSampleForTest(raw);
	const auto snapshot = hub.ReadDiagnostics();
	if (snapshot.devices[9].received != 1 || snapshot.devices[16].received != 2 ||
		snapshot.devices[16].latest.position[1] != -0.25 || snapshot.gapMarkers != 1 || snapshot.reportedLoss != 7)
		return false;
	std::vector<protocol::DevicePoseSample> samples;
	if (hub.Drain(consumer, samples) != 0 || samples.size() != 2)
		return false;
	return hub.Drain(consumer, samples) == 7 && samples.size() == 1 &&
		samples.front().sampleTimeQpc == 10010 && hub.ReadDiagnostics().devices[16].received == 2;
}

bool ContinuousWindowDiagnosticsScenario()
{
	using Engine = questcal::ContinuousAlignment;
	Engine engine;
	questcal::MountExtrinsic mount;
	mount.valid = true;
	engine.SetExtrinsic(mount);
	auto feed = [&](double time) {
		questcal::PoseSample sample;
		sample.time = time;
		engine.PushReference(sample);
		engine.PushTarget(sample);
		engine.Update(time, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 1.0, 0.0);
	};
	// Repeated loss prevents warmup; its evidence must survive the same resets.
	for (int interval = 0; interval < 3; ++interval)
	{
		for (int frame = 0; frame < 100; ++frame)
			feed(10.0 + interval + frame * 0.01);
		engine.Reset(Engine::ResetReason::StreamGap);
	}
	const auto starved = engine.GetDiagnostics();
	if (starved.resets[static_cast<size_t>(Engine::ResetReason::StreamGap)] != 3 ||
		starved.observations != 0 || starved.referenceSamples != 0 || starved.targetSamples != 0 ||
		starved.observationsKept == 0 || engine.GetState() != Engine::State::Inactive)
		return false;
	for (int frame = 0; frame < 700; ++frame)
		feed(20.0 + frame * 0.01);
	const auto recovered = engine.GetDiagnostics();
	return engine.GetState() == Engine::State::Tracking &&
		recovered.observations >= recovered.requiredObservations &&
		recovered.observationsKept > starved.observationsKept &&
		recovered.resets == starved.resets;
}

bool ContinuousPairingDiagnosticsScenario()
{
	using Engine = questcal::ContinuousAlignment;
	Engine engine;
	questcal::MountExtrinsic mount;
	mount.valid = true;
	engine.SetExtrinsic(mount);
	questcal::PoseSample sample;
	sample.time = 10.0;
	engine.PushTarget(sample);
	engine.Update(10.0, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 1.0, 0.0);
	if (engine.GetDiagnostics().referenceWaitUpdates != 1 || engine.GetDiagnostics().pendingTargets != 1)
		return false;
	sample.time = 9.99;
	engine.PushReference(sample);
	sample.time = 10.01;
	engine.PushReference(sample);
	engine.PushReference(sample);
	engine.Update(10.01, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 1.0, 0.0);
	sample.time = 10.02;
	sample.vel.x() = 10.0;
	engine.PushTarget(sample);
	engine.PushTarget(sample);
	engine.Update(10.02, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 1.0, 0.0);
	const auto result = engine.GetDiagnostics();
	return result.observationsFormed == 1 && result.observationsKept == 1 &&
		result.referenceOutOfOrder == 1 && result.targetOutOfOrder == 1 &&
		result.targetSpeedRejected == 1 && result.referenceWaitUpdates == 1 &&
		result.pendingTargets == 0;
}

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
