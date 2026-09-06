#pragma once

// The diagnostics file: one text file a player can attach to a bug report,
// written on request from Settings. It holds the version, the settings and
// profile summary, the recent activity, and the current and previous session
// logs, with the account name, the profile directories and the computer name
// replaced before anything is written. Device serials stay: they identify
// hardware, not people, and a report about "the tracker that keeps pausing"
// needs them.

#include <openvr.h>
#include <string>
#include "PoseStreamHub.h"
#include "DriverSyncTracker.h"

struct CalibrationContext;
namespace vr { class IVRSystem; }

// The same snapshot goes to the export and the throttled detailed log.
// qpcNow uses the raw pose clock, or zero when that clock is unavailable.
std::string DescribeContinuousDiagnostics(const CalibrationContext &ctx, double qpcNow);
struct DiagnosticCapture
{
	PoseStreamHub::Diagnostics poseStream;
	questcal::DriverSyncTracker driverSync;
	double sampleClock = 0.0;
	float steamVrWorldScale = 0.0f;
	vr::EVRSettingsError worldScaleError = vr::VRSettingsError_ReadFailed;
};
DiagnosticCapture CaptureCalibrationDiagnostics();

// Writes the file under %LOCALAPPDATA%\QuestCalibrator\diagnostics. On
// success pathOut is the file's UTF-8 path; on failure error says why.
bool WriteDiagnosticsFile(const CalibrationContext &ctx, std::string &pathOut, std::string &error,
	vr::IVRSystem *system, const DiagnosticCapture &capture);

// Opens Explorer with the file selected, so "send this file" needs no search.
void RevealInExplorer(const std::string &utf8Path);

// The substitution the file applies, exposed for the test harness.
std::string AnonymiseDiagnosticsText(const std::string &text,
	const std::string &userProfileDir, const std::string &userName, const std::string &computerName);
