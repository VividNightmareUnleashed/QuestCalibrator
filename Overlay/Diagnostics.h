#pragma once

// The diagnostics file: one text file a player can attach to a bug report,
// written on request from Settings. It holds the version, the settings and
// profile summary, the recent activity, and the current and previous session
// logs, with the account name, the profile directories and the computer name
// replaced before anything is written. Device serials stay: they identify
// hardware, not people, and a report about "the tracker that keeps pausing"
// needs them.

#include <string>

struct CalibrationContext;

// Writes the file under %LOCALAPPDATA%\QuestCalibrator\diagnostics. On
// success pathOut is the file's UTF-8 path; on failure error says why.
bool WriteDiagnosticsFile(const CalibrationContext &ctx, std::string &pathOut, std::string &error);

// Opens Explorer with the file selected, so "send this file" needs no search.
void RevealInExplorer(const std::string &utf8Path);

// The substitution the file applies, exposed for the test harness.
std::string AnonymiseDiagnosticsText(const std::string &text,
	const std::string &userProfileDir, const std::string &userName, const std::string &computerName);
