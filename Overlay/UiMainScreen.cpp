// Rating and continuous-status logic, the status band and the main screen.
#include "stdafx.h"
#include "UiInternal.h"

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
	case ContinuousStatus::NotRunning: return "Waiting";
	case ContinuousStatus::Tracking:   return "Active";
	case ContinuousStatus::Coasting:   return "Waiting";
	case ContinuousStatus::Frozen:     return "Paused";
	case ContinuousStatus::Holding:    return "Waiting";
	default:                           return "Warming up";
	}
}

// The main screen's line about the loop, as a whole sentence so it reads
// (and translates) as one. Written for the question the player has ("will
// this fix itself?"): waiting states resolve on their own, paused ones name
// what they need. NotRunning covers a sleeping tracker, a disabled
// calibration and a closed pose channel alike, so it claims no cause.
const char *ContinuousStatusLine(ContinuousStatus status)
{
	switch (status)
	{
	case ContinuousStatus::Off:        return "Continuous calibration is off.";
	case ContinuousStatus::NoTracker:  return "Continuous calibration needs a headset tracker. Pick one in Settings.";
	case ContinuousStatus::NeedsMount: return "Continuous calibration needs the headset tracker set up. Do it in Settings.";
	case ContinuousStatus::NotRunning: return "Continuous calibration is waiting. It resumes when tracking is available.";
	case ContinuousStatus::Tracking:   return "Continuous calibration is active.";
	case ContinuousStatus::Coasting:   return "Continuous calibration is waiting. The headset tracker isn't being seen.";
	case ContinuousStatus::Frozen:     return "Continuous calibration is paused. Readings drifted too far to correct.";
	case ContinuousStatus::Holding:    return "Continuous calibration is waiting. Tracking is too noisy here.";
	default:                           return "Continuous calibration is warming up.";
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
	// loop gets its own line and action instead: a calibration that solved
	// well is still good when the headset tracker gets nudged.
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

	// Solve residuals are not persisted, so a restored profile has no evidence
	// behind the quality half of this verdict. With nothing demoting it, say
	// "Not measured" rather than Good, unless the continuous loop is measuring
	// live.
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

ImVec4 RatingColor(CalRating r)
{
	switch (r)
	{
	// Unknown claims nothing in either direction, so it gets the neutral ink.
	case Rating_Unknown: return Pal::Dim;
	case Rating_Good:    return Pal::Good;
	case Rating_Decent:  return Pal::Warn;
	case Rating_Poor:    return Pal::Bad;
	default:             return Pal::VeryBad;
	}
}

// The recalibration nudge, derived from the rating alone so every screen gives
// the same advice. Null when there is nothing to advise.
const char *RecalibrationNudge(CalRating rating)
{
	if (rating < Rating_Poor)
		return nullptr;
	return rating == Rating_VeryPoor
		? "Your trackers won't line up like this. Recalibrate."
		: "Recalibrate to tighten the alignment.";
}

// Empty when the timestamp (0 = unknown) or the clock is unusable; each caller
// words its own fallback.
std::optional<std::string> FormatUnixAge(double unixTime)
{
	double now = static_cast<double>(std::time(nullptr));
	if (unixTime <= 0.0 || now <= 0.0)
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
// the later of the manual solve and the last auto-correction. Anything shown
// beside a score-derived verdict must use that base and say which one it is.
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

// Pinned below the scrolling content by BuildMainWindow. Its height goes
// through s_bottomReserve so the next frame's content child leaves room for it.
float s_bottomReserve = 48.0f;
void BuildStatusBand(const VRState &state)
{
	// ---- Status ----
	// The verdict strip renders in both modes; advanced mode adds the
	// numbers as quiet lines under the verdict instead of replacing it.
	const ContinuousStatus continuous = ContinuousStatusNow();
	if (CalCtx.validProfile && continuous == ContinuousStatus::Frozen)
	{
		const float width = ImGui::GetContentRegionAvail().x;
		const float actionW = 280.0f;
		const float bandH = CalCtx.uiAdvanced ? 220.0f : 184.0f;
		s_bottomReserve = bandH;
		ImGui::SetCursorPosY(ImGui::GetWindowHeight() - bandH + 18.0f);
		const ImVec2 p = ImGui::GetCursorScreenPos();
		ImGui::PushFont(g_fontTitle);
		ImGui::TextUnformatted(Tr("Continuous calibration paused"));
		ImGui::PopFont();
		ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + width - actionW - 36.0f);
		ImGui::TextWrapped("%s", Tr("Tracking no longer matches the saved alignment. Recalibrate with the headset tracker."));
		ImGui::PopTextWrapPos();
		ImGui::SetCursorScreenPos(ImVec2(p.x + width - actionW, p.y));
		if (IconButton("fixmount", "Recalibrate", IconPlay, ImVec2(actionW, 46.0f), BtnKind::Primary))
			StartMountSetup(state);
		ImGui::SetCursorScreenPos(ImVec2(p.x + width - actionW, p.y + 54.0f));
		if (IconButton("stopcont", "Turn off continuous calibration", nullptr, ImVec2(actionW, 38.0f), BtnKind::Ghost))
			SaveProfileFieldEdit(CalCtx, [](questcal::ProfileRecord &candidate) {
				candidate.continuousEnabled = false;
			});
		ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + 108.0f));
		if (CalCtx.uiAdvanced && CalCtx.continuousDeviation.valid)
		{
			ImGui::PushFont(g_fontSmall);
			ImGui::TextColored(Pal::Dim, "%s", Tr(FormatString(
				"Difference: %.1f deg yaw, %.1f deg tilt, %.1f cm position",
				CalCtx.continuousDeviation.yawDeg, CalCtx.continuousDeviation.tiltDeg,
				CalCtx.continuousDeviation.posM * 100.0)).c_str());
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
	// Detail lines share one quiet ink and a role word each; the verdict owns
	// the colour, so a detail line never reads as a second opinion.
	std::vector<DetailLine> details;
	if (CalCtx.uiAdvanced && CalCtx.validProfile)
	{
		if (CalCtx.lastResult.valid)
		{
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
			// Evidence only when there is some.
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
	// continuous-calibration line and the nudge.
	{
		const bool showContinuous = CalCtx.validProfile && continuous != ContinuousStatus::Off;
		const float lineH = g_fontBody->LegacySize + 6.0f;
		const float detailH = g_fontSmall->LegacySize + 6.0f;
		int lines = CalCtx.validProfile ? 1 + (showContinuous ? 1 : 0) + (nudge ? 1 : 0) : 1;
		float stripH = lineH * (float)lines + (lines > 1 ? 4.0f : 0.0f)
			+ detailH * (float)details.size() + (details.empty() ? 0.0f : 4.0f);

		// Full-bleed inset surface with a hairline top edge, anchored to the
		// bottom of the window.
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
			float ty = y + lineH * 0.5f - g_fontBody->LegacySize * 0.5f;
			float x = p.x;
			const char *label = Tr(RatingLabel(rating));
			const char *heading = Tr("Alignment: ");
			dl->AddText(g_fontBody, g_fontBody->LegacySize, ImVec2(x, ty), Pal::U32(Pal::Text), heading);
			x += ImGui::CalcTextSize(heading).x;
			dl->AddText(g_fontBody, g_fontBody->LegacySize, ImVec2(x, ty), Pal::U32(RatingColor(rating)), label);
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
			dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
				ImVec2(x + 18.0f, y + lineH * 0.5f - g_fontSmall->LegacySize * 0.5f + 2.0f),
				Pal::U32(Pal::Dim), Tr(ageLine.c_str()));
			y += lineH;

			// Advanced mode: the numbers, small and quiet under the verdict.
			for (const auto &detail : details)
			{
				dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
					ImVec2(p.x, y + detailH * 0.5f - g_fontSmall->LegacySize * 0.5f),
					Pal::U32(detail.color), Tr(detail.text.c_str()));
				y += detailH;
			}
			if (!details.empty())
				y += 4.0f;

			// Line 2: the loop's own state, one sentence in its own colour.
			if (showContinuous)
			{
				y += 4.0f;
				ty = y + lineH * 0.5f - g_fontBody->LegacySize * 0.5f;
				dl->AddText(g_fontBody, g_fontBody->LegacySize, ImVec2(p.x, ty),
					Pal::U32(ContinuousStatusColor(continuous)), Tr(ContinuousStatusLine(continuous)));
				y += lineH;
			}

			if (nudge)
			{
				if (!showContinuous)
					y += 4.0f;
				dl->AddText(g_fontBody, g_fontBody->LegacySize,
					ImVec2(p.x, y + lineH * 0.5f - g_fontBody->LegacySize * 0.5f),
					Pal::U32(Pal::Violet), Tr(nudge));
			}
		}
		else
		{
			dl->AddText(g_fontBody, g_fontBody->LegacySize,
				ImVec2(p.x, y + lineH * 0.5f - g_fontBody->LegacySize * 0.5f),
				Pal::U32(Pal::Dim),
				Tr("Not calibrated yet. Pick a device on each side and press Start calibration."));
		}

		ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + stripH));
		ImGui::Dummy(ImVec2(0, 0));
	}
}

void BuildMainScreen(const VRState &state)
{
	float cw = ImGui::GetContentRegionAvail().x;
	const float gap = 12.0f;

	{
		{
			std::vector<StatusRowData> warn;
			if (CalCtx.validProfile && !CalCtx.enabled)
			{
				// Each disable reason wants a different action, so each gets
				// its own sentence.
				using Reason = CalibrationContext::DisableReason;
				std::string why;
				switch (CalCtx.disableReason)
				{
				case Reason::HmdMismatch:
					why = FormatString("%s headset isn't connected. Calibration is off until it's back.",
						FriendlySystemName(CalCtx.referenceTrackingSystem).c_str());
					break;
				case Reason::DriverUnreachable:
					why = "SteamVR isn't accepting the calibration. Restart SteamVR.";
					break;
				case Reason::InvalidIdentity:
					why = "The saved calibration doesn't match the connected hardware. Recalibrate.";
					break;
				case Reason::InvalidTransform:
					why = "The saved calibration is damaged. Recalibrate.";
					break;
				case Reason::UniverseUnsafe:
					why = "The headset re-centered while QuestCalibrator wasn't watching, so the saved alignment is off. Recalibrate.";
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
		const float clearW = ButtonWidthFor("Clear calibration", true, 190.0f);
		const float segItemW = 112.0f;
		const float segW = segItemW * 3.0f + 8.0f;
		float startW = cw - segW - gap - (haveProfile ? clearW + gap : 0.0f);

		ImVec2 rowA = ImGui::GetCursorScreenPos();
		const bool recovering = ContinuousStatusNow() == ContinuousStatus::Frozen;
		if (IconButton("start", "Start calibration", IconPlay, ImVec2(startW, bh),
			recovering ? BtnKind::Ghost : BtnKind::Primary))
			OpenGuide(false, false);

		{
			// Labelled by length: "Slow" reads as an instruction to move slowly.
			std::string speedLabels[3];
			const char *speeds[3];
			for (int i = 0; i < 3; ++i)
			{
				speedLabels[i] = FormatString("%.0f s", CalibrationContext::CollectionSecondsFor(
					static_cast<CalibrationContext::Speed>(i)));
				speeds[i] = speedLabels[i].c_str();
			}
			ImVec2 sp = ImVec2(rowA.x + startW + gap, rowA.y);
			ImGui::SetCursorScreenPos(ImVec2(sp.x, sp.y + (bh - 46.0f) * 0.5f));
			auto previousSpeed = CalCtx.calibrationSpeed;
			CalCtx.calibrationSpeed = static_cast<CalibrationContext::Speed>(
				Segmented("speed", static_cast<int>(CalCtx.calibrationSpeed), speeds, 3, segItemW, 46.0f));
			if (CalCtx.calibrationSpeed != previousSpeed)
				SaveSettingOrRestore(CalCtx.calibrationSpeed, previousSpeed);
			if (ImGui::IsMouseHoveringRect(sp, ImVec2(sp.x + segW, sp.y + bh)) && !ImGui::IsAnyItemActive())
				ShowTip("How long calibration collects tracking data. Longer can be more accurate.\nMove gently at every setting.");
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
					"Saves your current chaperone (SteamVR's walls) and puts it\n"
					"back automatically if SteamVR or the headset ever loses it.\n"
					"Redrew your chaperone? Press again to protect the new one.\n"
					"Prefer the Quest boundary imported fresh each session? Don't use this.");
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
		const bool showActivity = !CalCtx.activity.empty() && ImGui::CollapsingHeader(
			(std::string(Tr("Recent activity")) + "###recentactivity").c_str());
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
				ImGui::TextWrapped("%s", Tr(entry.text.c_str()));
				ImGui::Spacing();
			}
		}

	}
}
