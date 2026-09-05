// Rating and continuous-status logic, the status band and the main screen.
#include "stdafx.h"
#include "UiInternal.h"

// ---------------------------------------------------------------------------
// Plain-language calibration rating (simple mode)
// ---------------------------------------------------------------------------

bool s_showSettings = false;

// The driver's pose channel feeds every runtime monitor. Preview mode has no
// driver at all, so it must not display a fault for it.
bool PoseChannelDown()
{
	return !g_uiPreviewMode && !CalCtx.poseRingOpen;
}

// ---------------------------------------------------------------------------
// Continuous-calibration status
// ---------------------------------------------------------------------------

ContinuousStatus ContinuousStatusNow()
{
	using CA = questcal::ContinuousAlignment;

	if (!CalCtx.continuousEnabled)
		return ContinuousStatus::Off;
	if (CalCtx.continuousTrackerSerial.empty())
		return ContinuousStatus::NoTracker;
	if (!CalCtx.ContinuousArmed())
		return ContinuousStatus::NeedsMount;

	// The rest of ContinuousTick's shouldRun conjunction. The tracker id is
	// re-resolved by the driver sync and reset to invalid whenever that batch
	// fails, so a sleeping tracker or a dropped pipe lands here rather than
	// looking like a loop that is still warming up. Preview mode has no driver
	// at all and so can satisfy none of these (see PoseChannelDown).
	bool running = g_uiPreviewMode ||
		(CalCtx.state == CalibrationState::None &&
			CalCtx.enabled && CalCtx.validProfile && CalCtx.poseRingOpen &&
			CalCtx.continuousTrackerId < vr::k_unMaxTrackedDeviceCount &&
			CalCtx.referenceDeviceMask[vr::k_unTrackedDeviceIndex_Hmd]);
	if (!running)
		return ContinuousStatus::NotRunning;

	switch (CalCtx.continuousState)
	{
	case CA::State::Tracking: return ContinuousStatus::Tracking;
	case CA::State::Coasting: return ContinuousStatus::Coasting;
	case CA::State::Frozen:   return ContinuousStatus::Frozen;
	case CA::State::Holding:  return ContinuousStatus::Holding;
	default:                  return ContinuousStatus::Gathering;
	}
}

// The state as one or two words, for the settings row where the feature's
// name is the row title and the setup button beneath says what to do.
const char *ContinuousStateWord(ContinuousStatus status)
{
	switch (status)
	{
	case ContinuousStatus::Off:        return "Off";
	case ContinuousStatus::NoTracker:  return "Needs a tracker";
	case ContinuousStatus::NeedsMount: return "Needs setup";
	case ContinuousStatus::NotRunning: return "Paused";
	case ContinuousStatus::Tracking:   return "Active";
	case ContinuousStatus::Coasting:   return "Waiting";
	case ContinuousStatus::Frozen:     return "Paused";
	case ContinuousStatus::Holding:    return "Waiting";
	default:                           return "Warming up";
	}
}

// The rest of the sentence that starts "Continuous calibration", for the
// main screen's band. Written for the question the player has ("will this
// fix itself?"): waiting states resolve on their own, paused ones name what
// they need. One sentence, no colons: the band already has a label.
const char *ContinuousStatusLine(ContinuousStatus status)
{
	switch (status)
	{
	case ContinuousStatus::Off:        return " is off.";
	case ContinuousStatus::NoTracker:  return " needs a headset tracker. Pick one in Settings.";
	case ContinuousStatus::NeedsMount: return " needs one run with the headset tracker.";
	case ContinuousStatus::NotRunning: return " paused. The headset tracker is off or asleep.";
	case ContinuousStatus::Tracking:   return " is active.";
	case ContinuousStatus::Coasting:   return " is waiting. The headset tracker isn't being seen.";
	case ContinuousStatus::Frozen:     return " paused. Readings drifted too far to correct.";
	case ContinuousStatus::Holding:    return " is waiting. Tracking is too noisy here.";
	default:                           return " is warming up.";
	}
}

// Whether the loop is doing its job, paused for a reason that clears itself,
// or needs the player: the colour every rendering of the status shares.
ImVec4 ContinuousStatusColor(ContinuousStatus status)
{
	switch (status)
	{
	case ContinuousStatus::Tracking:   return Pal::Good;
	case ContinuousStatus::Frozen:     return Pal::Bad;
	case ContinuousStatus::Off:        return Pal::Dim;
	default:                           return Pal::Warn;
	}
}

// ---------------------------------------------------------------------------
// Rating
// ---------------------------------------------------------------------------

CalRating ComputeCalibrationRating(ContinuousStatus continuous)
{
	int r = Rating_Good;

	// Solve quality caps the rating; staleness/drift then degrade it further.
	if (CalCtx.lastResult.valid)
	{
		double rot = CalCtx.lastResult.rotationRmsDeg;
		double pos = CalCtx.lastResult.translationRmsMeters * 100.0;
		int q =
			(rot <= 3.0 && pos <= 1.5) ? Rating_Good :
			(rot <= 6.0 && pos <= 3.0) ? Rating_Decent :
			(rot <= 12.0 && pos <= 6.0) ? Rating_Poor : Rating_VeryPoor;
		if (q > r)
			r = q;
	}

	// A live measurement outranks the solve's history: with the headset
	// tracker running, the deviation between what it sees and the calibration
	// is the actual misalignment right now, so a paused loop with a large
	// deviation reads as the misalignment it is instead of "Usable".
	if ((continuous == ContinuousStatus::Tracking || continuous == ContinuousStatus::Frozen) &&
		CalCtx.continuousDeviation.valid)
	{
		double yaw = CalCtx.continuousDeviation.yawDeg + CalCtx.continuousDeviation.tiltDeg;
		double pos = CalCtx.continuousDeviation.posM * 100.0;
		int q =
			(yaw <= 0.75 && pos <= 2.0) ? Rating_Good :
			(yaw <= 1.5 && pos <= 4.0) ? Rating_Decent :
			(yaw <= 3.0 && pos <= 8.0) ? Rating_Poor : Rating_VeryPoor;
		if (q > r)
			r = q;
	}

	// Without the pose channel the monitors that would demote this rating
	// cannot run at all: no jump compensation, no drift evidence, and the solve
	// itself fell back to tick-rate runtime poses. Never claim Good on that;
	// and with no solve this session either, nothing has been measured at all,
	// so say so rather than asserting "Usable" while SteamVR is still starting.
	if (PoseChannelDown())
	{
		if (!CalCtx.lastResult.valid)
			return Rating_Unknown;
		if (r < Rating_Decent)
			r = Rating_Decent;
	}

	// Staleness/drift only degrade the rating when nothing is maintaining the
	// alignment; a healthy continuous loop re-measures it constantly. A frozen
	// loop is reported on its own line with its own action, not folded into
	// this verdict: a calibration that solved well ten minutes ago is still
	// good when the headset tracker gets nudged, and calling it Poor sent
	// people to redo a good calibration or to switch the feature off.
	bool continuouslyMaintained = continuous == ContinuousStatus::Tracking;
	if (!continuouslyMaintained)
	{
		if (CalCtx.alignment == CalibrationContext::AlignmentHealth::Aging && r < Rating_Decent)
			r = Rating_Decent;
		if (CalCtx.alignment == CalibrationContext::AlignmentHealth::Stale && r < Rating_Poor)
			r = Rating_Poor;
		if (CalCtx.driftScore >= 0.85)
			r = Rating_VeryPoor;
	}

	// Solve residuals live only in lastResult, which FinishCalibration sets and
	// nothing persists, so a profile restored from the registry has no evidence
	// behind the quality half of this verdict. If none of the monitors above
	// found a reason to demote it, what we have is an absence of evidence, not
	// a good measurement -- say so instead of asserting the best label.
	// A running continuous loop is a live measurement of the same thing, so
	// after a restart it is evidence enough: "Not measured" beside "maintained
	// continuously" would deny a measurement the app has.
	const bool measured = CalCtx.lastResult.valid ||
		((continuous == ContinuousStatus::Tracking || continuous == ContinuousStatus::Frozen) &&
			CalCtx.continuousDeviation.valid);
	if (!measured && r == Rating_Good)
		return Rating_Unknown;

	return (CalRating)r;
}

// The ladder answers "should I do anything?", so the two bad rungs read as a
// verdict a player can act on rather than a grade.
static const char *RatingLabels[] = { "Good", "Usable", "Rough", "Bad" };

const char *RatingLabel(CalRating r)
{
	return r == Rating_Unknown ? "Not measured" : RatingLabels[r];
}

// The band a fresh solve's residuals land in; shared with the last-calibration
// row so its colour follows the numbers instead of a hardcoded green.
CalRating SolveQualityRating(const questcal::EngineResult &result)
{
	if (!result.valid)
		return Rating_Unknown;
	double rot = result.rotationRmsDeg;
	double pos = result.translationRmsMeters * 100.0;
	return (rot <= 3.0 && pos <= 1.5) ? Rating_Good :
		(rot <= 6.0 && pos <= 3.0) ? Rating_Decent :
		(rot <= 12.0 && pos <= 6.0) ? Rating_Poor : Rating_VeryPoor;
}

ImVec4 RatingColor(CalRating r)
{
	switch (r)
	{
	// Unknown claims nothing in either direction, so it gets the neutral ink
	// rather than the green of a verdict we cannot support or the red of one
	// we have no reason to give.
	case Rating_Unknown: return Pal::Dim;
	case Rating_Good:    return Pal::Good;
	case Rating_Decent:  return Pal::Warn;
	case Rating_Poor:    return Pal::Bad;
	default:             return Pal::VeryBad;
	}
}

// The one recalibration nudge both screens render. The rating already folds
// solve quality, staleness, drift and a bumped mount into a single verdict;
// re-deriving "should I nag" from alignment on one screen and from the rating
// on the other let identical state produce contradictory advice. Returns null
// when there is nothing to advise.
const char *RecalibrationNudge(CalRating rating)
{
	if (rating < Rating_Poor)
		return nullptr;
	return rating == Rating_VeryPoor
		? "Your trackers won't line up like this. Run a new calibration."
		: "Run a new calibration.";
}

// Empty when the timestamp is unusable. "Unknown" is a property of the data,
// not a rendered phrase: returning it as an empty optional lets every caller
// word its own fallback, instead of the wording being baked into a buffer and
// recovered downstream by comparing against the literal.
std::optional<std::string> FormatUnixAge(double unixTime)
{
	double now = static_cast<double>(std::time(nullptr));
	if (!std::isfinite(unixTime) || unixTime <= 0.0 || now <= 0.0)
		return std::nullopt;

	double seconds = now - unixTime;
	// Small clock corrections must not read as a future timestamp.
	if (seconds < -300.0)
		return std::string("time is in the future");

	double hours = std::max(0.0, seconds) / 3600.0;
	if (seconds < 90.0)
		return std::string("just now");
	if (hours < 1.0)
		return FormatString("%d min ago", static_cast<int>(hours * 60.0));
	if (hours < 48.0)
		return FormatString("%.1f h ago", hours);
	return FormatString("%.0f days ago", hours / 24.0);
}

// The age of the alignment, from the same base UpdateDriftScore ages from:
// the later of the manual solve and the last auto-correction, because a
// continuously maintained calibration is not aging. Anything rendered next to
// a score-derived verdict must use that base and say which one it is --
// showing the solve time alone put "calibrated 3.5 days ago" beside
// "Alignment fresh" whenever the loop had just corrected it.
std::optional<std::string> FormatAlignmentAge()
{
	if (CalCtx.lastAutoCorrectionUnixTime > CalCtx.calibrationUnixTime)
	{
		if (auto adjusted = FormatUnixAge(CalCtx.lastAutoCorrectionUnixTime))
			return "adjusted " + *adjusted;
	}
	if (auto calibrated = FormatUnixAge(CalCtx.calibrationUnixTime))
		return "calibrated " + *calibrated;
	return std::nullopt;
}

// Snapshot the live chaperone and arm auto-restore (the "protect" action).
// In preview mode there is no VR system to read bounds from.
bool ProtectChaperone()
{
	if (g_uiPreviewMode)
		return true;
	return LoadChaperoneBounds();
}

// When the one-time drift warning modal was opened; gates its accept button.
double g_chapWarnOpenedAt = 0.0;

float s_bottomReserve = 48.0f;
void BuildStatusBand(const VRState &state)
{
	// ---- Status ----
	// The verdict strip renders in both modes; advanced mode adds the
	// numbers as quiet lines under the verdict instead of replacing it.
	const ContinuousStatus continuous = ContinuousStatusNow();
	if (CalCtx.validProfile && continuous == ContinuousStatus::Frozen)
	{
		const float width = ImGui::GetWindowContentRegionWidth();
		const float actionW = 280.0f;
		const float bandH = CalCtx.uiAdvanced ? 220.0f : 184.0f;
		s_bottomReserve = bandH;
		ImGui::SetCursorPosY(ImGui::GetWindowHeight() - bandH + 18.0f);
		const ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::PushFont(g_fontTitle);
		ImGui::TextUnformatted("Continuous calibration paused");
		ImGui::PopFont();
		ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + width - actionW - 36.0f);
		ImGui::TextWrapped("Tracking no longer matches the saved alignment. Recalibrate with the tracker attached to your headset.");
		ImGui::PopTextWrapPos();
		ImGui::SetCursorScreenPos(ImVec2(p.x + width - actionW, p.y));
		if (IconButton("fixmount", "Recalibrate", IconPlay, ImVec2(actionW, 46.0f), BtnKind::Primary))
			StartMountSetup(state);
		ImGui::SetCursorScreenPos(ImVec2(p.x + width - actionW, p.y + 54.0f));
		if (IconButton("stopcont", "Turn off continuous", nullptr, ImVec2(actionW, 38.0f), BtnKind::Ghost))
			SaveProfileFieldEdit(CalCtx, [](questcal::ProfileRecord &candidate) {
				candidate.continuousEnabled = false;
			});
		ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + 108.0f));
		if (CalCtx.uiAdvanced && CalCtx.continuousDeviation.valid)
		{
			ImGui::PushFont(g_fontSmall);
			ImGui::TextColored(Pal::Dim, "Difference: %.1f deg yaw, %.1f deg tilt, %.1f cm position",
				CalCtx.continuousDeviation.yawDeg, CalCtx.continuousDeviation.tiltDeg,
				CalCtx.continuousDeviation.posM * 100.0);
			ImGui::PopFont();
		}
		return;
	}
	const CalRating rating = CalCtx.validProfile ? ComputeCalibrationRating(continuous) : Rating_Unknown;
	const char *nudge = CalCtx.validProfile ? RecalibrationNudge(rating) : nullptr;

	struct DetailLine
	{
		std::string text;
		ImVec4 color;
	};
	std::vector<DetailLine> details;
	if (CalCtx.uiAdvanced && CalCtx.validProfile)
	{
		if (CalCtx.lastResult.valid)
		{
			// Colour follows the solve's own quality band; a hardcoded
			// green tick called an 11-degree solve a success.
			CalRating solve = SolveQualityRating(CalCtx.lastResult);
			// Detail lines share one quiet ink and a role word each; the
			// verdict above owns the colour. A green solve row under a red
			// verdict read as a second opinion.
			(void)solve;
			details.push_back({ FormatString("Solve: %.2f deg / %.1f cm, %+.1f ms%s",
					CalCtx.lastResult.rotationRmsDeg,
					CalCtx.lastResult.translationRmsMeters * 100.0,
					CalCtx.lastResult.timeOffset * 1000.0,
					CalCtx.lastResult.scale != 1.0 ? ", scaled" : ""), Pal::Dim });
			if (CalCtx.solveScale)
				details.push_back({ FormatString("Scale: %s (condition %.4f, uncertainty %.4f)",
						CalCtx.lastResult.scaleIdentifiable ? "identifiable" : "insufficient",
						CalCtx.lastResult.scaleCondition, CalCtx.lastResult.scaleStdDev), Pal::Dim });
		}

		if (CalCtx.jumpsCompensated > 0 || CalCtx.referenceGapEvents > 0)
		{
			details.push_back({ FormatString("Jumps: %u compensated%s",
					CalCtx.jumpsCompensated,
					CalCtx.referenceGapEvents > 0 ? " (tracking gaps seen; recalibrate if alignment looks off)" : ""),
				Pal::Dim });
		}

		{
			const ImVec4 healthColor = Pal::Dim;
			const char *healthLabel =
				CalCtx.alignment == CalibrationContext::AlignmentHealth::Stale ? "stale" :
				CalCtx.alignment == CalibrationContext::AlignmentHealth::Aging ? "aging" : "fresh";
			// The age is on the verdict line already.
			std::string line = FormatString("Drift: %s", healthLabel);
			// Evidence only when there is some: "0 tracking glitch(es)" was
			// a developer's plural on a zero.
			if (CalCtx.driftSlideEvents > 0)
				line += FormatString("; %u slip%s up to %.1f cm while standing still",
					CalCtx.driftSlideEvents, CalCtx.driftSlideEvents == 1 ? "" : "s",
					CalCtx.driftMaxSlideM * 100.0);
			if (CalCtx.discontinuousLossEvents > 0)
				line += FormatString("; %u tracking dropout%s with a position change",
					CalCtx.discontinuousLossEvents, CalCtx.discontinuousLossEvents == 1 ? "" : "s");
			details.push_back({ line, healthColor });
		}

		// Only while the loop is running: a correction count under "needs
		// setup" contradicts it, and the count is reset when the tracker
		// changes.
		const bool loopRunning = continuous == ContinuousStatus::Tracking ||
			continuous == ContinuousStatus::Coasting || continuous == ContinuousStatus::Frozen ||
			continuous == ContinuousStatus::Holding;
		if (loopRunning)
		{
			if (continuous == ContinuousStatus::Tracking && CalCtx.continuousDeviation.valid)
				details.push_back({ FormatString(
						"Upkeep: scatter %.2f deg / %.1f cm, deviation %.2f deg / %.1f cm, %u corrections",
						CalCtx.continuousScatterRotDeg, CalCtx.continuousScatterPosM * 100.0,
						CalCtx.continuousDeviation.yawDeg + CalCtx.continuousDeviation.tiltDeg,
						CalCtx.continuousDeviation.posM * 100.0,
						CalCtx.autoCorrectionsApplied), Pal::Dim });
			else
				details.push_back({ FormatString("Upkeep: %u corrections", CalCtx.autoCorrectionsApplied), Pal::Dim });
		}
	}

	// Bottom band: the verdict, the advanced detail lines, the
	// continuous-calibration line with its own action when it has paused, and the
	// nudge.
	{
		const bool showContinuous = CalCtx.validProfile && continuous != ContinuousStatus::Off;
		const float lineH = g_fontBody->FontSize + 6.0f;
		const float detailH = g_fontSmall->FontSize + 6.0f;
		int lines = CalCtx.validProfile ? 1 + (showContinuous ? 1 : 0) + (nudge ? 1 : 0) : 1;
		float stripH = lineH * (float)lines + (lines > 1 ? 4.0f : 0.0f)
			+ detailH * (float)details.size() + (details.empty() ? 0.0f : 4.0f);

		// Full-bleed inset surface with a hairline top edge, so the status
		// text sits on something instead of floating.
		// Anchored to the bottom when there is room, pushed down by the
		// content when there isn't; the surface is painted either way so
		// the band never loses its inset when the screen is busy.
		const float bandH = stripH + 14.0f + 44.0f;
		const float bandTop = ImGui::GetWindowHeight() - bandH;
		s_bottomReserve = bandH;
		{
			ImVec2 wp = ImGui::GetWindowPos();
			float ww = ImGui::GetWindowWidth();
			float top = wp.y + bandTop - ImGui::GetScrollY();
			ImDrawList *dlb = ImGui::GetWindowDrawList();
			dlb->AddRectFilled(ImVec2(wp.x, top),
				ImVec2(wp.x + ww, top + bandH + 40.0f), Pal::U32(Pal::Inset));
			dlb->AddLine(ImVec2(wp.x, top), ImVec2(wp.x + ww, top), Pal::U32(Pal::Border), 1.0f);
			ImGui::SetCursorPosY(bandTop + 14.0f);
		}
		ImVec2 p = ImGui::GetCursorScreenPos();
		ImDrawList *dl = ImGui::GetWindowDrawList();
		float y = p.y;

		if (CalCtx.validProfile)
		{
			// Line 1: the verdict word carries the colour; the age beside it
			// is information, so it gets Dim rather than Faint.
			float ty = y + lineH * 0.5f - g_fontBody->FontSize * 0.5f;
			float x = p.x;
			const char *label = RatingLabel(rating);
			dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(x, ty), Pal::U32(Pal::Text), "Tracking quality: ");
			x += ImGui::CalcTextSize("Tracking quality: ").x;
			dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(x, ty), Pal::U32(RatingColor(rating)), label);
			x += ImGui::CalcTextSize(label).x;

			std::string ageLine;
			if (continuous == ContinuousStatus::Tracking)
			{
				std::optional<std::string> adjusted;
				if (CalCtx.autoCorrectionsApplied > 0)
					adjusted = FormatUnixAge(CalCtx.lastAutoCorrectionUnixTime);
				ageLine = adjusted ? FormatString("last adjusted %s", adjusted->c_str())
					: std::string("maintained continuously");
			}
			else
			{
				auto age = FormatAlignmentAge();
				ageLine = age ? *age : std::string("calibration time unknown");
			}
			dl->AddText(g_fontSmall, g_fontSmall->FontSize,
				ImVec2(x + 18.0f, y + lineH * 0.5f - g_fontSmall->FontSize * 0.5f + 2.0f),
				Pal::U32(Pal::Dim), ageLine.c_str());
			y += lineH;

			// Advanced mode: the numbers, small and in the colour of the
			// verdict they support.
			for (const auto &detail : details)
			{
				dl->AddText(g_fontSmall, g_fontSmall->FontSize,
					ImVec2(p.x, y + detailH * 0.5f - g_fontSmall->FontSize * 0.5f),
					Pal::U32(detail.color), detail.text.c_str());
				y += detailH;
			}
			if (!details.empty())
				y += 4.0f;

			// Line 2: the loop's own state in its own colour and, when it
			// has paused, the two things the player can do about it.
			if (showContinuous)
			{
				y += 4.0f;
				ty = y + lineH * 0.5f - g_fontBody->FontSize * 0.5f;
				x = p.x;
				dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(x, ty), Pal::U32(Pal::Text), "Continuous calibration");
				x += ImGui::CalcTextSize("Continuous calibration").x;
				dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(x, ty),
					Pal::U32(ContinuousStatusColor(continuous)), ContinuousStatusLine(continuous));
				y += lineH;
			}

			if (nudge)
			{
				if (!showContinuous)
					y += 4.0f;
				dl->AddText(g_fontBody, g_fontBody->FontSize,
					ImVec2(p.x, y + lineH * 0.5f - g_fontBody->FontSize * 0.5f),
					Pal::U32(Pal::Violet), nudge);
			}
		}
		else
		{
			dl->AddText(g_fontBody, g_fontBody->FontSize,
				ImVec2(p.x, y + lineH * 0.5f - g_fontBody->FontSize * 0.5f),
				Pal::U32(Pal::Dim),
				"Not calibrated yet. Pick a device on each side and press Start calibration.");
		}

		ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + stripH));
		ImGui::Dummy(ImVec2(0, 0));
	}
}

void BuildMainScreen(const VRState &state)
{
	float cw = ImGui::GetWindowContentRegionWidth();
	const float gap = 12.0f;

	{
		{
			std::vector<StatusRowData> warn;
			if (CalCtx.validProfile && !CalCtx.enabled)
			{
				// Six conditions disable a calibration and each wants a
				// different action; naming the headset for all of them sent
				// people to check hardware that was working.
				using Reason = CalibrationContext::DisableReason;
				std::string why;
				switch (CalCtx.disableReason)
				{
				case Reason::HmdMismatch:
					why = FormatString("%s headset not detected; calibration disabled until it's back",
						FriendlySystemName(CalCtx.referenceTrackingSystem).c_str());
					break;
				case Reason::DriverUnreachable:
					why = "SteamVR isn't accepting the calibration. Restart SteamVR.";
					break;
				case Reason::InvalidIdentity:
					why = "The saved calibration doesn't match the connected hardware. Recalibrate.";
					break;
				case Reason::InvalidTransform:
					why = "The saved calibration is corrupt. Recalibrate.";
					break;
				case Reason::UniverseUnsafe:
					why = "The headset re-centred while QuestCalibrator wasn't watching, so the saved alignment is off. Recalibrate.";
					break;
				case Reason::None:
					// Never borrow another cause's sentence: a universe change
					// that was not observed is not something to assert.
					why = "Calibration disabled. Restart SteamVR or recalibrate.";
					break;
				}
				warn.push_back({ IconInfo, Pal::Bad, why });
			}
			if (PoseChannelDown())
			{
				std::string why = "QuestCalibrator isn't getting tracking data from SteamVR, so it can't watch for drift. Restart SteamVR.";
				if (CalCtx.uiAdvanced)
					why += FormatString(" (host hooks: 005 %s, 006 %s)",
						CalCtx.driverPoseHookMask & protocol::PoseHook005 ? "active" : "missing",
						CalCtx.driverPoseHookMask & protocol::PoseHook006 ? "active" : "missing");
				warn.push_back({ IconInfo, Pal::Warn, why });
			}
			const questcal::update::Snapshot update =
				questcal::update::AppUpdater.GetSnapshot();
			if (update.state == questcal::update::State::Ready)
			{
				warn.push_back({ IconDownload, Pal::Good,
					"QuestCalibrator " + update.version +
					" is ready. Open Settings to install it." });
			}
			if (!warn.empty())
			{
				DrawStatusCard(warn);
				ImGui::Spacing();
			}
		}

		// ---- Row A: Start dominates; the measurement length and Clear ride
		// beside it so the whole screen fits without scrolling ----
		bool haveProfile = CalCtx.validProfile;
		const float bh = 56.0f;
		const float clearW = 190.0f;
		const float segItemW = 80.0f;
		const float segW = segItemW * 3.0f + 8.0f;
		float startW = cw - segW - gap - (haveProfile ? clearW + gap : 0.0f);

		ImVec2 rowA = ImGui::GetCursorScreenPos();
		const bool recovering = ContinuousStatusNow() == ContinuousStatus::Frozen;
		if (IconButton("start", "Start calibration", IconPlay, ImVec2(startW, bh),
			recovering ? BtnKind::Ghost : BtnKind::Primary))
			OpenGuide(false, false);

		// Measurement length. A duration, not a speed: "Fast" read as the good
		// choice and fought the instruction to move slowly.
		{
			const char *speeds[] = { "10 s", "20 s", "35 s" };
			ImVec2 sp = ImVec2(rowA.x + startW + gap, rowA.y);
			ImGui::SetCursorScreenPos(ImVec2(sp.x, sp.y + (bh - 46.0f) * 0.5f));
			auto previousSpeed = CalCtx.calibrationSpeed;
			CalCtx.calibrationSpeed = static_cast<CalibrationContext::Speed>(
				Segmented("speed", static_cast<int>(CalCtx.calibrationSpeed), speeds, 3, segItemW, 46.0f));
			if (CalCtx.calibrationSpeed != previousSpeed)
				SaveSettingOrRestore(CalCtx.calibrationSpeed, previousSpeed);
			if (ImGui::IsMouseHoveringRect(sp, ImVec2(sp.x + segW, sp.y + bh)) && !ImGui::IsAnyItemActive())
				ShowTip("How long to measure. Longer is more accurate.");
		}

		if (haveProfile)
		{
			ImGui::SetCursorScreenPos(ImVec2(rowA.x + startW + gap + segW + gap, rowA.y));
			// Irreversible and one laser-click from the primary button: confirm.
			if (IconButton("clear", "Clear calibration", IconTrash, ImVec2(clearW, bh), BtnKind::Ghost))
				ImGui::OpenPopup("Clear calibration?");
		}
		ImGui::SetCursorScreenPos(ImVec2(rowA.x, rowA.y + bh));
		ImGui::Dummy(ImVec2(0, 0));

		// ---- Row B: the secondary tools, one row ----
		{
			const bool anchors = CalCtx.validProfile && !CalCtx.profileUniverseUnsafe;
			const int count = anchors ? 3 : 2;
			const float bw = (cw - gap * (float)(count - 1)) / (float)count;
			const float rh = 46.0f;

			IdentifyButton(ImVec2(bw, rh));

			// One button covers the whole chaperone story: snapshot the current
			// bounds AND arm auto-restore. People who rely on the Quest boundary
			// transferring in each session simply never press it (or disarm the
			// restore in settings).
			ImGui::SameLine(0.0f, gap);
			const char *chapLabel = CalCtx.chaperone.valid ? "Update protected chaperone" : "Protect chaperone";
			if (IconButton("copychap", chapLabel, IconCopy, ImVec2(bw, rh), BtnKind::Ghost))
			{
				if (CalCtx.chaperoneWarningAck)
				{
					ProtectChaperone();
				}
				else
				{
					// First use: the drift warning modal takes over; it calls
					// ProtectChaperone() itself once acknowledged.
					g_chapWarnOpenedAt = ImGui::GetTime();
					ImGui::OpenPopup("Chaperone Drift Warning");
				}
			}
			if (ImGui::IsItemHovered())
			{
				ShowTip(
					"Saves your current chaperone bounds (SteamVR's walls) and puts\n"
					"them back automatically if SteamVR or the headset ever loses them.\n"
					"Redrew your chaperone? Press again to save the new one.\n"
					"Prefer the Quest's Guardian imported fresh each session? Don't use this.");
			}
			if (anchors)
			{
				ImGui::SameLine(0.0f, gap);
				if (IconButton("addanchor", "Add field anchor", IconPin, ImVec2(bw, rh), BtnKind::Ghost))
					OpenGuide(true, false);
				if (ImGui::IsItemHovered())
				{
					ShowTip(
						"Aligned in one spot but slightly off in another?\n"
						"Stand at the bad spot, press this, and do a quick calibration there.\n"
						"That spot gets its own correction, blended in as you walk around.");
				}
			}
		}

		// Current state stays in the pinned band; historical events are optional.
		ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0, 0, 0, 0));
		ImGui::PushStyleColor(ImGuiCol_HeaderHovered, Pal::CardHov);
		ImGui::PushStyleColor(ImGuiCol_HeaderActive, Pal::Inset);
		const bool showActivity = !CalCtx.activity.empty() && ImGui::CollapsingHeader("Recent activity");
		ImGui::PopStyleColor(3);
		if (showActivity)
		{
			const size_t shown = std::min<size_t>(CalCtx.activity.size(), CalCtx.uiAdvanced ? 2 : 3);
			for (size_t i = CalCtx.activity.size() - shown; i < CalCtx.activity.size(); ++i)
			{
				const auto &entry = CalCtx.activity[i];
				char stamp[16] = "";
				std::time_t t = static_cast<std::time_t>(entry.unixTime);
				std::tm tm;
				if (localtime_s(&tm, &t) == 0)
					std::strftime(stamp, sizeof stamp, "%H:%M", &tm);
				ImGui::PushFont(g_fontSmall);
				ImGui::TextColored(Pal::Dim, "%s", stamp);
				ImGui::PopFont();
				ImGui::TextWrapped("%s", entry.text.c_str());
				ImGui::Spacing();
			}
		}

	}
}
