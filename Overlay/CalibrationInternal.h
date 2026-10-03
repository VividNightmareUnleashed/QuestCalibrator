#pragma once

// What Calibration.cpp shares with the files split from it
// (CalibrationMonitor.cpp, CalibrationContinuous.cpp, CalibrationSolve.cpp):
// the state more than one of them uses, defined in Calibration.cpp, and the
// steps one of them calls in another. Nothing else includes it.

#include "Calibration.h"
#include "ContinuousAlignment.h"
#include "DriftMonitor.h"
#include "LighthouseFrameWatch.h"
#include "PoseStreamHub.h"
#include "StreamEvents.h"

#include <Eigen/Dense>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace calibration_internal
{
// The ring's clock: QPC seconds per count (set by InitCalibrator), and now.
extern double QpcToSeconds;
double QpcNowSeconds();

extern PoseStreamHub PoseHub;
extern std::unique_ptr<DriftMonitor> Drift;
extern LighthouseFrameWatch FrameWatch;
extern int MonitorConsumer;

// Everything the per-tick runtime monitors keep between ticks that does not
// live inside a detector object. Collected here so "what is the monitor's
// state" has one answer and the half that is inferred from an unbroken pose
// stream is cleared wherever the detectors themselves are reset.
struct MonitorState
{
	// Per-device composed-time watermarks (see RuntimeMonitorTick).
	bool hasComposedTime[vr::k_unMaxTrackedDeviceCount] = {};
	double lastComposedTime[vr::k_unMaxTrackedDeviceCount] = {};

	// The HMD's latest raw position, for the worn-device heuristic. Only rough
	// currency is needed, but a stale one suppresses drift evidence, so it must
	// not survive a hole the detectors were reset across.
	Eigen::Vector3d hmdRawPosition{ 0, 0, 0 };
	double hmdRawTime = -1e9;

	// One notification per calibration: re-armed only by a successful solve
	// (the freeze one also by a resume event).
	bool staleNotified = false;
	bool freezeNotified = false;
	bool unstableNotified = false;

	// Only the stream-derived half: the one-shot flags are per-calibration
	// policy, so a drain hole must not clear them.
	void ResetObservations()
	{
		for (bool &hasTime : hasComposedTime)
			hasTime = false;
		hmdRawPosition = Eigen::Vector3d::Zero();
		hmdRawTime = -1e9;
	}

	void ReArmNotifications()
	{
		staleNotified = false;
		freezeNotified = false;
		unstableNotified = false;
	}
};

extern MonitorState Monitors;

extern std::unique_ptr<questcal::ContinuousAlignment> Continuous;
extern int ContinuousConsumer;
extern std::vector<protocol::DevicePoseSample> ContinuousPending;
extern int64_t FrameObservedThrough[vr::k_unMaxTrackedDeviceCount];
// A failure capture a notable frame move asked for (detailed logging only).
extern bool FrameCapturePending;
extern double FrameCaptureDue;
extern double LastFrameCapture;

// Supplied by the app shell (see Calibration.h); absent under -uipreview.
extern std::function<void(const char *)> ToastSink;

// Why a run stopped, in the player's words: what happened (naming what they
// can see) and one thing to do. `detail` is the engineer's reason, kept
// verbatim for the session log and the modal's details toggle.
struct StopReason
{
	std::string body;
	std::string action;
	std::string detail;
};

void ResetContinuousObservations(CalibrationContext &ctx,
	questcal::ContinuousAlignment::ResetReason reason);
void DispatchStreamEvent(CalibrationContext &ctx, questcal::StreamEvent event);
bool EndCalibrationRun(CalibrationContext &ctx);
void AbortCalibration(CalibrationContext &ctx, const StopReason &reason);
std::string DeviceName(const CalibrationContext &ctx, const std::string &model,
	const std::string &serial, bool reference);
uint32_t LighthouseRestarts(const CalibrationContext &ctx, const std::string &serial);
void NotifyOnce(CalibrationContext &ctx, bool &notified, const char *line,
	CalibrationContext::Tone tone, const char *toast, bool showToast);

// The per-tick steps CalibrationTick runs from the other files.
void LighthouseTick(CalibrationContext &ctx, double time);
void RuntimeMonitorTick(CalibrationContext &ctx, double now);
void ContinuousTick(CalibrationContext &ctx, double now);
void FinishCalibration(CalibrationContext &ctx);
}
