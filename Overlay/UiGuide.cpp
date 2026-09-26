// The guided calibration modal (art, meters, stages) and the other modals.
#include "stdafx.h"
#include "UiInternal.h"

// ---------------------------------------------------------------------------
// Main menu (default state)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Guided calibration
//
// The modal walks the player through four stages: get set (what to hold and
// how, with the two picks' tracking state), a short countdown, the run with
// live feedback from CalibrationGuide.h, and the outcome with a picture of
// what to change. Presentation state only; the run itself is the context's.
// ---------------------------------------------------------------------------

GuideState s_guide;

// Whether the modal shows the engineer lines (ids, serials, residuals). Opens
// the way advanced mode is set; the toggle at the bottom of the modal flips it.
bool s_modalDetails = false;

void OpenGuide(bool anchor, bool mountRun)
{
	s_modalDetails = CalCtx.uiAdvanced;
	s_guide = GuideState();
	s_guide.anchor = anchor;
	s_guide.mountRun = mountRun;
	s_guide.stage = GuideStage::GetSet;
	CalCtx.ClearMessages();
	ImGui::OpenPopup("Calibration Progress");
}

// Continuous calibration needs one run with the headset as the reference and
// the headset tracker as the target. Nobody should have to know that: this
// makes both picks and starts the guided run.
void StartMountSetup(const VRState &state)
{
	const VRDevice *hmd = nullptr;
	const VRDevice *tracker = nullptr;
	for (const auto &d : state.devices)
	{
		if (d.deviceClass == vr::TrackedDeviceClass_HMD && !hmd)
			hmd = &d;
		if (!CalCtx.continuousTrackerSerial.empty() && d.serial == CalCtx.continuousTrackerSerial)
			tracker = &d;
	}
	if (!hmd)
	{
		CalCtx.ReportError("The headset isn't showing up in SteamVR, so the tracker position can't be measured yet.\n");
		return;
	}
	if (!tracker || !tracker->connected)
	{
		CalCtx.ReportError("The headset tracker isn't connected. Switch it on, then try again.\n");
		return;
	}
	CalCtx.pendingReferenceTrackingSystem = hmd->trackingSystem;
	CalCtx.referenceID = static_cast<uint32_t>(hmd->id);
	CalCtx.pendingTargetTrackingSystem = tracker->trackingSystem;
	CalCtx.targetID = static_cast<uint32_t>(tracker->id);
	s_showSettings = false;
	OpenGuide(false, true);
}

// The countdown ended: start the real run, or fake one for the preview so
// the running stage can be styled without a runtime.
bool BeginGuidedRun()
{
	if (g_uiPreviewMode)
	{
		CalCtx.ClearMessages();
		CalCtx.Log("Reference device ID: 0, serial: 1PASH5D1P17365\n"
			"Target device ID: 3, serial: LHR-77E5A211\n"
			"Sampling raw driver poses (timestamped)\n");
		if (s_guide.mountRun)
		{
			CalCtx.Instruct("Look around slowly.");
			CalCtx.Note("Side to side, up and down, and tilt your head. Keep the tracker tracking the whole time.");
		}
		else
		{
			CalCtx.Instruct("Hold them together and keep rotating.");
			CalCtx.Note("Rotate around more than one axis and move around a little. Keep both devices tracking.");
		}
		CalCtx.Progress(650, 1000);
		s_guide.metrics.valid = true;
		s_guide.metrics.coverage = 0.55;
		s_guide.metrics.gatedFraction = 0.08;
		s_guide.metrics.rigidityValid = true;
		s_guide.metrics.rigidityDeg = 1.4;
		return true;
	}
	return s_guide.anchor ? StartAnchorCalibration() : StartCalibration();
}

// Procedural art for the guide: line work in the palette, so it reads at
// overlay scale and needs no assets.
namespace GuideArt
{
	// A tracker (rounded square) held against a controller (capsule) as one
	// rigid pair, rotated by `angle` about `c` and foreshortened by `tilt`.
	static void Pair(ImDrawList *dl, ImVec2 c, float s, float angle, float tilt, ImU32 col)
	{
		const float ca = cosf(angle), sa = sinf(angle), ct = cosf(tilt);
		auto P = [&](float x, float y) {
			return ImVec2(c.x + (x * ca - y * sa) * s, c.y + (x * sa + y * ca) * s * ct);
		};
		ImVec2 a = P(-0.95f, 0.05f), b = P(0.0f, 0.05f);
		dl->AddLine(a, b, col, s * 0.32f);
		dl->AddCircleFilled(a, s * 0.16f, col, 12);
		dl->AddCircleFilled(b, s * 0.16f, col, 12);
		dl->AddCircle(P(-0.2f, -0.32f), s * 0.40f, col, 24, 2.2f);
		ImVec2 quad[4] = { P(0.12f, -0.62f), P(0.82f, -0.62f), P(0.82f, 0.08f), P(0.12f, 0.08f) };
		dl->AddPolyline(quad, 4, col, true, 2.4f);
		dl->AddCircleFilled(P(0.47f, -0.27f), s * 0.08f, col, 8);
	}

	// Headset with the tracker strapped on top; `yaw` fakes the head turning.
	static void HeadsetWithTracker(ImDrawList *dl, ImVec2 c, float s, float yaw, ImU32 col, ImU32 accent)
	{
		const float shift = sinf(yaw) * s * 0.25f;
		dl->PathRect(ImVec2(c.x - s, c.y - s * 0.45f), ImVec2(c.x + s, c.y + s * 0.45f), s * 0.3f);
		dl->PathStroke(col, true, 2.4f);
		dl->AddCircleFilled(ImVec2(c.x - s * 0.42f + shift, c.y), s * 0.15f, col, 12);
		dl->AddCircleFilled(ImVec2(c.x + s * 0.42f + shift, c.y), s * 0.15f, col, 12);
		// Strap up over the head, tracker sitting on it.
		dl->AddLine(ImVec2(c.x - s * 0.55f, c.y - s * 0.45f), ImVec2(c.x - s * 0.35f, c.y - s * 1.05f), col, 2.0f);
		dl->AddLine(ImVec2(c.x + s * 0.55f, c.y - s * 0.45f), ImVec2(c.x + s * 0.35f, c.y - s * 1.05f), col, 2.0f);
		ImVec2 t0(c.x - s * 0.32f + shift * 0.5f, c.y - s * 1.55f);
		ImVec2 t1(c.x + s * 0.32f + shift * 0.5f, c.y - s * 1.0f);
		dl->PathRect(t0, t1, s * 0.12f);
		dl->PathStroke(accent, true, 2.4f);
		dl->AddCircleFilled(ImVec2((t0.x + t1.x) * 0.5f, (t0.y + t1.y) * 0.5f), s * 0.07f, accent, 8);
	}

	// Arc with an arrowhead at its end: "rotate this way".
	static void CurvedArrow(ImDrawList *dl, ImVec2 c, float r, float a0, float a1, ImU32 col, float th = 2.4f)
	{
		dl->PathArcTo(c, r, a0, a1, 24);
		dl->PathStroke(col, false, th);
		ImVec2 tip(c.x + cosf(a1) * r, c.y + sinf(a1) * r);
		const float dir = a1 > a0 ? 1.0f : -1.0f;
		ImVec2 tan(-sinf(a1) * dir, cosf(a1) * dir);
		ImVec2 nrm(cosf(a1), sinf(a1));
		const float h = 9.0f;
		dl->AddTriangleFilled(
			ImVec2(tip.x + tan.x * h * 0.6f, tip.y + tan.y * h * 0.6f),
			ImVec2(tip.x - tan.x * h + nrm.x * h * 0.7f, tip.y - tan.y * h + nrm.y * h * 0.7f),
			ImVec2(tip.x - tan.x * h - nrm.x * h * 0.7f, tip.y - tan.y * h - nrm.y * h * 0.7f), col);
	}

	static void BaseStation(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
	{
		dl->PathRect(ImVec2(c.x - s * 0.5f, c.y - s * 0.5f), ImVec2(c.x + s * 0.5f, c.y + s * 0.5f), s * 0.12f);
		dl->PathStroke(col, true, 2.2f);
		dl->AddCircleFilled(c, s * 0.14f, col, 10);
	}
}

// The looping "what to do" picture: the pair turning through a figure-eight,
// or the head turning with the tracker on it.
void DrawGuideAnimation(ImDrawList *dl, ImVec2 origin, ImVec2 size, double t, bool mountRun)
{
	ImVec2 c(origin.x + size.x * 0.5f, origin.y + size.y * 0.5f);
	dl->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y), Pal::U32(Pal::Inset), 12.0f);
	const ImU32 ink = Pal::U32(Pal::Text);
	const float ft = static_cast<float>(t);
	if (mountRun)
	{
		GuideArt::HeadsetWithTracker(dl, ImVec2(c.x, c.y + 12.0f), 34.0f, ft * 1.1f, ink, Pal::U32(Pal::Accent));
		GuideArt::CurvedArrow(dl, ImVec2(c.x, c.y + 8.0f), 64.0f, IM_PI * 1.15f, IM_PI * 1.85f, Pal::U32(Pal::Dim));
		return;
	}
	// Figure-eight drift plus continuous rotation and a slow tilt: the motion
	// the solver wants, shown rather than described.
	ImVec2 pc(c.x + cosf(ft * 0.7f) * 26.0f, c.y + sinf(ft * 1.4f) * 12.0f);
	GuideArt::Pair(dl, pc, 30.0f, ft * 1.3f, sinf(ft * 0.8f) * 0.9f, ink);
	// Radius plus the +-12 px drift and the arrowhead stays inside the 150 px
	// panel; at 58 the arc intermittently crossed its top edge.
	GuideArt::CurvedArrow(dl, pc, 50.0f, ft * 0.5f, ft * 0.5f + IM_PI * 0.9f, Pal::U32(Pal::Dim));
}

// A still picture of the mistake a refused run points at.
void DrawGuideHint(ImDrawList *dl, ImVec2 origin, ImVec2 size, CalibrationContext::GuideHint hint)
{
	using Hint = CalibrationContext::GuideHint;
	ImVec2 c(origin.x + size.x * 0.5f, origin.y + size.y * 0.5f);
	dl->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y), Pal::U32(Pal::Inset), 12.0f);
	const ImU32 ink = Pal::U32(Pal::Text);
	const ImU32 accent = Pal::U32(Pal::Violet);
	const ImU32 bad = Pal::U32(Pal::Bad);
	switch (hint)
	{
	case Hint::RotateMore:
		GuideArt::Pair(dl, c, 26.0f, 0.4f, 0.3f, ink);
		GuideArt::CurvedArrow(dl, c, 52.0f, -IM_PI * 0.2f, IM_PI * 1.2f, accent, 3.0f);
		break;
	case Hint::TwoAxes:
		GuideArt::Pair(dl, c, 24.0f, 0.4f, 0.3f, ink);
		GuideArt::CurvedArrow(dl, c, 50.0f, IM_PI * 0.15f, IM_PI * 0.85f, accent, 3.0f);
		GuideArt::CurvedArrow(dl, c, 50.0f, IM_PI * 1.15f, IM_PI * 1.85f, accent, 3.0f);
		break;
	case Hint::HoldTogether:
	{
		GuideArt::Pair(dl, c, 26.0f, 0.0f, 0.0f, ink);
		// Chevrons pressing the pair together.
		ImVec2 l[3] = { ImVec2(c.x - 62.0f, c.y - 16.0f), ImVec2(c.x - 48.0f, c.y), ImVec2(c.x - 62.0f, c.y + 16.0f) };
		ImVec2 r[3] = { ImVec2(c.x + 62.0f, c.y - 16.0f), ImVec2(c.x + 48.0f, c.y), ImVec2(c.x + 62.0f, c.y + 16.0f) };
		dl->AddPolyline(l, 3, accent, false, 3.0f);
		dl->AddPolyline(r, 3, accent, false, 3.0f);
		break;
	}
	case Hint::SlowDown:
		GuideArt::Pair(dl, ImVec2(c.x - 34.0f, c.y), 22.0f, 0.3f, 0.2f, ink);
		IconGauge(dl, ImVec2(c.x + 50.0f, c.y), 22.0f, accent);
		break;
	case Hint::KeepTracking:
	case Hint::TrackingLost:
	{
		ImVec2 bs(origin.x + 36.0f, origin.y + 30.0f);
		GuideArt::BaseStation(dl, bs, 24.0f, ink);
		// A dashed cone from the base station to the pair.
		ImVec2 target(c.x + 26.0f, c.y + 18.0f);
		for (int i = -2; i <= 2; ++i)
		{
			float spread = static_cast<float>(i) * 0.12f;
			ImVec2 dir(target.x - bs.x, target.y - bs.y);
			float len = sqrtf(dir.x * dir.x + dir.y * dir.y);
			dir.x /= len; dir.y /= len;
			ImVec2 side(-dir.y, dir.x);
			for (float f = 0.2f; f < 0.85f; f += 0.16f)
			{
				ImVec2 p0(bs.x + dir.x * len * f + side.x * spread * len * f, bs.y + dir.y * len * f + side.y * spread * len * f);
				ImVec2 p1(bs.x + dir.x * len * (f + 0.08f) + side.x * spread * len * (f + 0.08f),
					bs.y + dir.y * len * (f + 0.08f) + side.y * spread * len * (f + 0.08f));
				dl->AddLine(p0, p1, Pal::U32(Pal::Dim), 1.5f);
			}
		}
		GuideArt::Pair(dl, target, 22.0f, 0.5f, 0.3f, ink);
		if (hint == Hint::TrackingLost)
			dl->AddLine(ImVec2(c.x - 30.0f, c.y - 26.0f), ImVec2(c.x - 2.0f, c.y + 10.0f), bad, 4.0f);
		break;
	}
	case Hint::WrongPick:
		IconHMD(dl, ImVec2(c.x - 52.0f, c.y - 6.0f), 24.0f, ink);
		IconTracker(dl, ImVec2(c.x + 52.0f, c.y - 6.0f), 24.0f, ink);
		dl->AddText(g_fontSmall, g_fontSmall->FontSize, ImVec2(c.x - 86.0f, c.y + 26.0f), Pal::U32(Pal::Dim), "reference");
		dl->AddText(g_fontSmall, g_fontSmall->FontSize, ImVec2(c.x + 30.0f, c.y + 26.0f), Pal::U32(Pal::Dim), "target");
		break;
	case Hint::WaitForTracking:
		dl->PathArcTo(c, 26.0f, 0.0f, IM_PI * 1.5f, 24);
		dl->PathStroke(accent, false, 3.0f);
		break;
	case Hint::Success:
		dl->AddCircle(c, 30.0f, Pal::U32(Pal::Good), 32, 3.0f);
		IconCheck(dl, c, 16.0f, Pal::U32(Pal::Good));
		break;
	default:
		break;
	}
}

// The modal's eyebrow: which run this is and where in it the player stands.
// A step count does work a stage name above a stage headline did not.
std::string GuideStepLabel(bool anchor, bool mountRun, int step)
{
	const char *run = anchor ? "FIELD ANCHOR, " : mountRun ? "HEADSET TRACKER, " : "";
	return FormatString("%sSTEP %d OF 3", run, step);
}

const char *GuideHintCaption(CalibrationContext::GuideHint hint)
{
	using Hint = CalibrationContext::GuideHint;
	switch (hint)
	{
	case Hint::RotateMore:      return "Bigger turns";
	case Hint::TwoAxes:         return "Twist, tilt and roll";
	case Hint::HoldTogether:    return "Firmly together, in one hand";
	case Hint::SlowDown:        return "Walking speed";
	case Hint::KeepTracking:    return "Keep both devices tracking";
	case Hint::TrackingLost:    return "Keep both devices tracking";
	case Hint::WrongPick:       return "Headset side on the left, tracker on the right";
	case Hint::WaitForTracking: return "Let tracking settle first";
	case Hint::Success:         return nullptr;  // the tick and the headline already say it
	default:                    return nullptr;
	}
}

// The three live indicators during the run: rotation variety as a ring, pace
// and rigidity as bars, each with a word the player can act on.
void DrawGuideIndicators(ImDrawList *dl, ImVec2 origin, float width, const questcal::GuideMetrics &m, bool mountRun)
{
	(void)width;
	const float rowH = 48.0f;
	// A word only when there is something to change: a full green meter says
	// "good" by itself, and the amber words stand out for being the only ones.
	auto label = [&](float y, const char *name, const char *word, const ImVec4 &col) {
		// Without a word beneath it the name centres on the meter, as the
		// two-line block did.
		const float nameY = word ? y + 2.0f : y + rowH * 0.5f - g_fontBody->FontSize * 0.5f;
		dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(origin.x + 70.0f, nameY),
			Pal::U32(Pal::Text), name);
		if (word)
			dl->AddText(g_fontSmall, g_fontSmall->FontSize, ImVec2(origin.x + 70.0f, y + 26.0f),
				Pal::U32(col), word);
	};
	auto bar = [&](float y, float fill, const ImVec4 &col) {
		ImVec2 b0(origin.x + 10.0f, y + rowH * 0.5f - 5.0f);
		ImVec2 b1(origin.x + 50.0f, y + rowH * 0.5f + 5.0f);
		dl->AddRectFilled(b0, b1, Pal::U32(Pal::Border), 4.0f);
		if (fill > 0.02f)
			dl->AddRectFilled(b0, ImVec2(b0.x + (b1.x - b0.x) * fill, b1.y), Pal::U32(col), 4.0f);
	};

	// 1. Rotation variety: a ring that fills as the delta axes spread out.
	{
		const float y = origin.y;
		ImVec2 rc(origin.x + 30.0f, y + rowH * 0.5f);
		dl->AddCircle(rc, 18.0f, Pal::U32(Pal::Border), 32, 4.0f);
		const float f = m.valid ? static_cast<float>(m.coverage) : 0.0f;
		const bool full = f >= 0.99f;
		const ImVec4 col = !m.valid ? Pal::Dim : full ? Pal::Good : f >= 0.4f ? Pal::Warn : Pal::Accent;
		if (f > 0.01f)
		{
			dl->PathArcTo(rc, 18.0f, -IM_PI * 0.5f, -IM_PI * 0.5f + f * 2.0f * IM_PI, 40);
			dl->PathStroke(Pal::U32(col), false, 4.0f);
		}
		const char *word = !m.valid ? "measuring" : full ? nullptr : f >= 0.4f ? "more directions" : "keep turning";
		label(y, "Rotation variety", word, !m.valid ? Pal::Dim : full ? Pal::Good : Pal::Warn);
	}
	// 2. Pace: how much of the last second the solver would throw away.
	{
		const float y = origin.y + rowH;
		const float g = m.valid ? static_cast<float>(m.gatedFraction) : 0.0f;
		const ImVec4 col = !m.valid ? Pal::Dim : g < 0.15f ? Pal::Good : g < 0.4f ? Pal::Warn : Pal::Bad;
		bar(y, m.valid ? 1.0f - g : 0.0f, col);
		const char *word = !m.valid ? "measuring" : g < 0.15f ? nullptr : g < 0.4f ? "a little slower" : "slow down";
		label(y, "Pace", word, col);
	}
	// 3. Rigidity: whether the pair (or the tracker on the head) moves as one.
	{
		const float y = origin.y + rowH * 2.0f;
		const bool ok = m.rigidityValid;
		const double r = m.rigidityDeg;
		const ImVec4 col = !ok ? Pal::Dim : r < 3.0 ? Pal::Good : r < 6.0 ? Pal::Warn : Pal::Bad;
		const float fill = ok ? static_cast<float>(std::max(0.0, std::min(1.0, 1.0 - (r - 1.0) / 8.0))) : 0.0f;
		bar(y, fill, col);
		const char *word = !ok ? "measuring" : r < 3.0 ? nullptr
			: r < 6.0 ? "loosening" : (mountRun ? "wobbling" : "coming apart");
		label(y, mountRun ? "Tracker steady" : "Held together", word, col);
	}
}

// The bottom band: the verdict, the advanced detail lines, the
// continuous-calibration line and the nudge. Rendered by BuildMainWindow
// after the scrolling content child, so it never scrolls away or loses
// its surface when the screen is busy. Its height goes through
// s_bottomReserve so the next frame's child leaves room for it.

void BuildMenu(const VRState &state, bool runningInOverlay)
{
	auto &io = ImGui::GetIO();
	float cw = ImGui::GetWindowContentRegionWidth();

	if (CalCtx.state == CalibrationState::None)
	{
		if (s_showSettings)
			BuildSettingsScreen(state);
		else
			BuildMainScreen(state);
	}
	else if (CalCtx.state == CalibrationState::Editing)
	{
		bool transformValid = BuildProfileEditor();

		ImGui::Spacing();
		const float gap = 10.0f;
		const float cancelWidth = 130.0f;
		const char *saveLabel = transformValid ? "Save profile" : "Fix invalid values before saving";
		if (IconButton("saveprofile", saveLabel, IconCheck,
			ImVec2(cw - cancelWidth - gap, 52.0f),
			transformValid ? BtnKind::Primary : BtnKind::Ghost) && transformValid)
		{
			// Saving keeps the editor open, so re-seed from the context the save
			// just wrote: the fields then show the stored values (persistence may
			// have normalized them) and the sticky rotation-edited flag clears.
			if (SaveProfileEditorDraft())
				SeedTransformEditorDraft();
		}
		ImGui::SameLine(0.0f, gap);
		if (IconButton("cancelprofile", "Cancel", nullptr,
			ImVec2(cancelWidth, 52.0f), BtnKind::Ghost))
		{
			// Nothing to discard: leaving Editing is enough, because re-entering
			// it seeds the draft again.
			CalCtx.state = CalibrationState::None;
			CalCtx.timeLastScan = -1e9;
		}
	}
	else
	{
		// The modal normally covers this; if it ever does not, the same two
		// facts are here: what is happening and how to stop it.
		ImVec2 p = BeginRowCard(64.0f);
		ImDrawList *dl = ImGui::GetWindowDrawList();
		dl->AddText(g_fontBody, g_fontBody->FontSize,
			ImVec2(p.x + 20.0f, p.y + 32.0f - g_fontBody->FontSize * 0.5f),
			Pal::U32(Pal::Text), "Calibrating...");
		ImGui::SetCursorScreenPos(ImVec2(p.x + cw - 150.0f, p.y + 13.0f));
		if (IconButton("cancelcard", "Cancel", nullptr, ImVec2(130.0f, 38.0f), BtnKind::Ghost))
			CancelCalibration();
		EndRowCard(p, 64.0f);
	}

	// ---- Calibration progress modal ----
	float modalW = 720.0f;
	ImGui::SetNextWindowPos(ImVec2((io.DisplaySize.x - modalW) * 0.5f, 180.0f), ImGuiSetCond_Always);
	ImGui::SetNextWindowSize(ImVec2(modalW, 0.0f), ImGuiSetCond_Always);
	if (ImGui::BeginPopupModal("Calibration Progress", nullptr,
		ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove))
	{
		using Msg = CalibrationContext::Message;
		const double now = ImGui::GetTime();
		const float mw = ImGui::GetWindowContentRegionWidth();
		ImDrawList *mdl = ImGui::GetWindowDrawList();
		// The animation and the countdown need frames; the idle loop otherwise
		// waits up to a second between them.
		CalCtx.wantedUpdateInterval = std::min(CalCtx.wantedUpdateInterval, 1.0 / 60.0);

		// Stage transitions the context drives: a run that ended, any way,
		// moves to the outcome; a finished countdown starts the run.
		if (s_guide.stage == GuideStage::Running && !g_uiPreviewMode &&
			CalCtx.state == CalibrationState::None)
			s_guide.stage = GuideStage::Done;
		if (s_guide.stage == GuideStage::Running && g_uiPreviewMode &&
			now - s_guide.countdownStart > kCountdownSeconds + 6.0)
		{
			// Preview: a fake outcome so the result stage can be styled, a
			// refused solve under -uipreview-failed and a success otherwise.
			if (g_uiPreviewScenario == PreviewScenario::Failed)
			{
				CalCtx.lastRunHint = CalibrationContext::GuideHint::RotateMore;
				CalCtx.Outcome("That didn't work", "Not enough rotation to tell the axes apart.",
					"Turn the pair through more directions and try again.",
					"Rotation coverage 0.21 of 1.00 (need 0.60)", CalibrationContext::Tone::Warn);
			}
			else
			{
				CalCtx.lastRunHint = CalibrationContext::GuideHint::Success;
				CalCtx.Outcome("Calibration done", "Good.", "", "", CalibrationContext::Tone::Good);
			}
			s_guide.stage = GuideStage::Done;
		}
		if (s_guide.stage == GuideStage::Countdown && now - s_guide.countdownStart >= kCountdownSeconds)
		{
			if (BeginGuidedRun())
				s_guide.stage = GuideStage::Running;
			else
			{
				// StartCalibration already said why, in the banner.
				s_guide.stage = GuideStage::Idle;
				ImGui::CloseCurrentPopup();
			}
		}

		const ImVec2 artSize(240.0f, 150.0f);
		switch (s_guide.stage)
		{
		case GuideStage::GetSet:
		case GuideStage::Countdown:
		{
			SectionLabel(GuideStepLabel(s_guide.anchor, s_guide.mountRun, 1).c_str());
			ImGui::Spacing();
			ImVec2 row = ImGui::GetCursorScreenPos();
			DrawGuideAnimation(mdl, row, artSize, now, s_guide.mountRun);

			// Text column beside the picture: what to hold, how long, and
			// whether the two picks are tracking right now. ImGui starts every
			// new line at the window's left edge, so each item re-enters the
			// column explicitly.
			const float tx = row.x + artSize.x + 24.0f;
			auto column = [&]() {
				ImGui::SetCursorScreenPos(ImVec2(tx, ImGui::GetCursorScreenPos().y));
			};
			ImGui::SetCursorScreenPos(ImVec2(tx, row.y));
			// Wrap positions are window-local; a screen x here would put the
			// wrap point past the modal's edge and clip the headline instead.
			ImGui::PushTextWrapPos(row.x + mw - ImGui::GetWindowPos().x);
			ImGui::PushFont(g_fontTitle);
			ImGui::TextWrapped("%s", s_guide.mountRun ? "Put the headset on with the tracker strapped to it."
				: s_guide.anchor ? "Stand at the spot that feels off." : "Hold the two devices together.");
			ImGui::PopFont();
			ImGui::Spacing();
			column();
			std::string how = s_guide.mountRun
				? FormatString("You'll look around slowly for %.0f seconds.", CalCtx.CollectionSeconds())
				: FormatString("One hand, firmly. You'll rotate them for %.0f seconds.", CalCtx.CollectionSeconds());
			ImGui::TextWrapped("%s", how.c_str());
			ImGui::PopTextWrapPos();
			ImGui::Spacing();

			const VRDevice *picks[2] = { nullptr, nullptr };
			for (const auto &d : state.devices)
			{
				if (static_cast<uint32_t>(d.id) == CalCtx.referenceID)
					picks[0] = &d;
				if (static_cast<uint32_t>(d.id) == CalCtx.targetID)
					picks[1] = &d;
			}
			bool ready = true;
			for (int i = 0; i < 2; ++i)
			{
				ImVec2 lp = ImGui::GetCursorScreenPos();
				lp.x = tx;
				const bool ok = picks[i] && picks[i]->tracking;
				ready = ready && ok;
				mdl->AddCircleFilled(ImVec2(lp.x + 7.0f, lp.y + g_fontBody->FontSize * 0.5f), 5.0f,
					Pal::U32(ok ? Pal::Good : Pal::Bad), 12);
				std::string who = picks[i] ? DeviceDisplayName(*picks[i])
					: std::string(i == 0 ? "Reference device" : "Target device");
				// The dot already says "tracking"; words only when it isn't.
				std::string line = ok ? who : who + " -- not tracking";
				ImGui::SetCursorScreenPos(ImVec2(lp.x + 22.0f, lp.y));
				ImGui::TextColored(ok ? Pal::Text : Pal::Bad, "%s", line.c_str());
			}
			float bottom = std::max(ImGui::GetCursorScreenPos().y, row.y + artSize.y);
			ImGui::SetCursorScreenPos(ImVec2(row.x, bottom + 12.0f));

			if (s_guide.stage == GuideStage::Countdown)
			{
				int remain = static_cast<int>(std::ceil(kCountdownSeconds - (now - s_guide.countdownStart)));
				std::string label = FormatString("Starting in %d...", std::max(1, remain));
				ImGui::PushFont(g_fontTitle);
				ImGui::TextUnformatted(label.c_str());
				ImGui::PopFont();
				ImGui::Spacing();
				if (IconButton("guidecancel", "Cancel", nullptr, ImVec2(mw, 46.0f), BtnKind::Ghost) || EscapePressed())
				{
					s_guide.stage = GuideStage::Idle;
					ImGui::CloseCurrentPopup();
				}
			}
			else
			{
				const float cancelW = 180.0f;
				if (IconButton("guidestart", ready ? "Start" : "Start anyway", IconPlay,
					ImVec2(mw - cancelW - 12.0f, 46.0f), ready ? BtnKind::Primary : BtnKind::Ghost))
				{
					s_guide.stage = GuideStage::Countdown;
					s_guide.countdownStart = now;
				}
				ImGui::SameLine(0.0f, 12.0f);
				if (IconButton("guidecancel", "Cancel", nullptr, ImVec2(cancelW, 46.0f), BtnKind::Ghost) || EscapePressed())
				{
					s_guide.stage = GuideStage::Idle;
					ImGui::CloseCurrentPopup();
				}
			}
			break;
		}
		case GuideStage::Running:
		{
			SectionLabel(GuideStepLabel(s_guide.anchor, s_guide.mountRun, 2).c_str());
			ImGui::Spacing();
			// Live feedback from the run's own buffers, at 5 Hz.
			if (!g_uiPreviewMode && CalCtx.state == CalibrationState::Collecting &&
				now - s_guide.lastMetricsTime >= 0.2)
			{
				s_guide.lastMetricsTime = now;
				s_guide.metrics = questcal::ComputeGuideMetrics(
					CalCtx.run.referenceSamples, CalCtx.run.targetSamples);
			}
			for (auto &message : CalCtx.messages)
			{
				if (message.kind == Msg::Instruction)
				{
					ImGui::PushFont(g_fontTitle);
					ImGui::TextWrapped("%s", message.str.c_str());
					ImGui::PopFont();
				}
				else if (message.kind == Msg::Info)
					ImGui::TextWrapped("%s", message.str.c_str());
			}
			ImGui::Spacing();
			ImVec2 row = ImGui::GetCursorScreenPos();
			DrawGuideAnimation(mdl, row, artSize, now, s_guide.mountRun);
			DrawGuideIndicators(mdl, ImVec2(row.x + artSize.x + 24.0f, row.y + 4.0f),
				mw - artSize.x - 24.0f, s_guide.metrics, s_guide.mountRun);
			ImGui::SetCursorScreenPos(ImVec2(row.x, row.y + artSize.y + 8.0f));
			ImGui::Dummy(ImVec2(0, 0));
			for (auto &message : CalCtx.messages)
			{
				if (message.kind != Msg::Progress)
					continue;
				float fraction = message.target > 0
					? (float)message.progress / (float)message.target : 0.0f;
				ImGui::Spacing();
				ImGui::PushStyleColor(ImGuiCol_FrameBg, Pal::Inset);
				ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 5.0f);
				ImGui::ProgressBar(fraction, ImVec2(-1.0f, 10.0f), "");
				ImGui::PopStyleVar();
				ImGui::PopStyleColor();
				// Seconds remaining, not a percentage of an unnamed quantity.
				// Progress counts hundredths of a second (see CalibrationTick).
				double secondsLeft = std::max(0.0, (message.target - message.progress) / 100.0);
				ImGui::PushFont(g_fontSmall);
				ImGui::TextColored(Pal::Dim, "%.0f s left", std::ceil(secondsLeft));
				ImGui::PopFont();
			}
			ImGui::Spacing();
			// Otherwise captive: Escape and clicking outside do nothing to a
			// modal, and the run only ends on its own timer.
			if (IconButton("cancelprogress", "Cancel", nullptr, ImVec2(mw, 46.0f), BtnKind::Ghost) || EscapePressed())
			{
				if (g_uiPreviewMode)
				{
					s_guide.stage = GuideStage::Idle;
					ImGui::CloseCurrentPopup();
				}
				else
					CancelCalibration();
			}
			break;
		}
		case GuideStage::Done:
		default:
		{
			SectionLabel(GuideStepLabel(s_guide.anchor, s_guide.mountRun, 3).c_str());
			ImGui::Spacing();
			// The outcome block starts at the last headline; the run's own
			// instruction and note before it are not part of the result.
			// Detail lines are shown from anywhere, behind the toggle.
			size_t outcomeStart = 0;
			for (size_t i = 0; i < CalCtx.messages.size(); ++i)
				if (CalCtx.messages[i].kind == Msg::Headline)
					outcomeStart = i;
			bool anyDetail = false;
			for (size_t i = 0; i < CalCtx.messages.size(); ++i)
			{
				const auto &message = CalCtx.messages[i];
				switch (message.kind)
				{
				case Msg::Headline:
					if (i < outcomeStart)
						break;
					ImGui::PushFont(g_fontTitle);
					ImGui::TextWrapped("%s", message.str.c_str());
					ImGui::PopFont();
					ImGui::Spacing();
					break;
				case Msg::Info:
					if (i > outcomeStart)
						ImGui::TextWrapped("%s", message.str.c_str());
					break;
				case Msg::Action:
					if (i > outcomeStart)
					{
						ImGui::PushStyleColor(ImGuiCol_Text, Pal::Violet);
						ImGui::TextWrapped("%s", message.str.c_str());
						ImGui::PopStyleColor();
					}
					break;
				case Msg::Detail:
					anyDetail = true;
					break;
				default:
					break;
				}
			}
			// The engineer lines come after the outcome, whatever order they
			// were logged in: the result is what the modal is for.
			if (anyDetail && s_modalDetails)
			{
				ImGui::Spacing();
				ImGui::PushFont(g_fontSmall);
				ImGui::PushStyleColor(ImGuiCol_Text, Pal::Dim);
				for (const auto &message : CalCtx.messages)
					if (message.kind == Msg::Detail)
						ImGui::TextWrapped("%s", message.str.c_str());
				ImGui::PopStyleColor();
				ImGui::PopFont();
			}

			// The picture of what to change (or the tick), centred.
			const auto hint = CalCtx.lastRunHint;
			if (hint != CalibrationContext::GuideHint::None)
			{
				ImGui::Spacing();
				ImVec2 hp = ImGui::GetCursorScreenPos();
				const ImVec2 hs(220.0f, 120.0f);
				ImVec2 ho(hp.x + (mw - hs.x) * 0.5f, hp.y);
				DrawGuideHint(mdl, ho, hs, hint);
				float captionH = 0.0f;
				if (const char *cap = GuideHintCaption(hint))
				{
					ImGui::PushFont(g_fontSmall);
					ImVec2 cs = ImGui::CalcTextSize(cap);
					ImGui::PopFont();
					mdl->AddText(g_fontSmall, g_fontSmall->FontSize,
						ImVec2(hp.x + (mw - cs.x) * 0.5f, ho.y + hs.y + 6.0f), Pal::U32(Pal::Dim), cap);
					captionH = g_fontSmall->FontSize + 10.0f;
				}
				ImGui::SetCursorScreenPos(ImVec2(hp.x, ho.y + hs.y + captionH + 6.0f));
				ImGui::Dummy(ImVec2(0, 0));
			}

			ImGui::Spacing();
			const bool failed = hint != CalibrationContext::GuideHint::Success;
			const float detailsW = anyDetail ? 170.0f : 0.0f;
			if (anyDetail)
			{
				if (IconButton("modaldetails", s_modalDetails ? "Hide details" : "Show details", nullptr,
					ImVec2(detailsW, 46.0f), BtnKind::Quiet))
					s_modalDetails = !s_modalDetails;
				ImGui::SameLine(0.0f, 12.0f);
			}
			const float remaining = mw - (anyDetail ? detailsW + 12.0f : 0.0f);
			if (failed)
			{
				// Back to get-set with the picture still in mind, not to a log.
				const float closeW = 180.0f;
				if (IconButton("guideretry", "Try again", IconPlay, ImVec2(remaining - closeW - 12.0f, 46.0f), BtnKind::Primary))
				{
					CalCtx.ClearMessages();
					s_guide.stage = GuideStage::GetSet;
				}
				ImGui::SameLine(0.0f, 12.0f);
				if (IconButton("closeprogress", "Close", nullptr, ImVec2(closeW, 46.0f), BtnKind::Ghost) || EscapePressed())
				{
					s_guide.stage = GuideStage::Idle;
					ImGui::CloseCurrentPopup();
				}
			}
			else if (IconButton("closeprogress", "Close", nullptr, ImVec2(remaining, 46.0f), BtnKind::Ghost) || EscapePressed())
			{
				s_guide.stage = GuideStage::Idle;
				ImGui::CloseCurrentPopup();
			}
			break;
		}
		}

		ImGui::EndPopup();
	}

	// ---- Clear calibration confirmation ----
	ImGui::SetNextWindowPos(ImVec2((io.DisplaySize.x - modalW) * 0.5f, 180.0f), ImGuiSetCond_Always);
	ImGui::SetNextWindowSize(ImVec2(modalW, 0.0f), ImGuiSetCond_Always);
	if (ImGui::BeginPopupModal("Clear calibration?", nullptr,
		ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove))
	{
		ImGui::PushFont(g_fontTitle);
		ImGui::TextUnformatted("Clear this calibration?");
		ImGui::PopFont();
		ImGui::Spacing();
		ImGui::TextWrapped("Field anchors and the headset tracker measurement go with it.");
		ImGui::Spacing();
		ImGui::Spacing();
		float bw = ImGui::GetWindowContentRegionWidth();
		// The safe choice carries the accent and the width; the destructive
		// one is outlined in the error colour, so a laser pointer that lands
		// on the big blue button keeps the calibration.
		float clearW = 210.0f, bgap = 12.0f;
		if (IconButton("clearkeep", "Keep", nullptr, ImVec2(bw - clearW - bgap, 46.0f), BtnKind::Primary) || EscapePressed())
			ImGui::CloseCurrentPopup();
		ImGui::SameLine(0.0f, bgap);
		if (IconButton("clearconfirm", "Clear", IconTrash, ImVec2(clearW, 46.0f), BtnKind::Danger))
		{
			// The write can be refused; a destructive button that did nothing
			// has to say so instead of leaving the screen unchanged.
			if (!ClearSavedProfile(CalCtx))
				CalCtx.ReportError("Couldn't clear the calibration, so it's still saved. Restart QuestCalibrator and try again.\n",
					CalibrationContext::ErrorSource::ProfilePersistence);
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}

	// ---- One-time chaperone drift warning ----
	// Shown before the first "Protect chaperone" ever runs; the accept button
	// unlocks after a short countdown so the caveat actually gets read.
	// Narrower than the other modals: three paragraphs of prose want a
	// 60-70 character measure, and the buttons should span the text.
	const float proseW = 640.0f;
	ImGui::SetNextWindowPos(ImVec2((io.DisplaySize.x - proseW) * 0.5f, 180.0f), ImGuiSetCond_Always);
	ImGui::SetNextWindowSize(ImVec2(proseW, 0.0f), ImGuiSetCond_Always);
	if (ImGui::BeginPopupModal("Chaperone Drift Warning", nullptr,
		ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove))
	{
		SectionLabel("BEFORE YOU RELY ON THIS");
		ImGui::Spacing();

		ImGui::TextWrapped(
			"Playspace drift is inevitable due to how the Quest tracks your position "
			"relative to SteamVR devices. It continuously re-adjusts your position within "
			"the playspace, and although QuestCalibrator substantially improves upon the "
			"original, small errors will still build up over time, causing your chaperone "
			"bounds (SteamVR's walls) to become misaligned.");
		ImGui::Spacing();
		ImGui::TextWrapped(
			"For the best physical safety, keep the Quest's own Guardian enabled: it is "
			"anchored to the real room by the headset's cameras and cannot drift out of "
			"place. The protected chaperone is a fallback for when Guardian is off, and "
			"the only walls your SteamVR-tracked devices can see.");
		ImGui::Spacing();
		ImGui::PushStyleColor(ImGuiCol_Text, Pal::Violet);
		ImGui::TextWrapped(
			"Always check that your chaperone bounds are aligned with the real room "
			"before engaging in activities such as dancing.");
		ImGui::PopStyleColor();
		if (!CalCtx.uiError.empty())
		{
			ImGui::Spacing();
			ImGui::TextColored(Pal::Bad, "%s", CalCtx.uiError.c_str());
		}
		ImGui::Spacing();
		ImGui::Spacing();

		float bw = ImGui::GetWindowContentRegionWidth();
		float cancelW = 210.0f, bgap = 12.0f;
		int remain = static_cast<int>(std::ceil(5.0 - (ImGui::GetTime() - g_chapWarnOpenedAt)));
		if (remain > 0)
		{
			// Disabled look: ghost chrome, dim countdown label, no hover/click.
			ImVec2 p = ImGui::GetCursorScreenPos();
			ImVec2 sz(bw - cancelW - bgap, 46.0f);
			ImDrawList *mdl = ImGui::GetWindowDrawList();
			mdl->AddRectFilled(p, ImVec2(p.x + sz.x, p.y + sz.y), Pal::U32(Pal::Card), 10.0f);
			mdl->AddRect(p, ImVec2(p.x + sz.x, p.y + sz.y), Pal::U32(Pal::Border), 10.0f);
			std::string lbl = FormatString("I understand (%d s)", remain);
			ImVec2 ts = ImGui::CalcTextSize(lbl.c_str());
			mdl->AddText(g_fontBody, g_fontBody->FontSize,
				ImVec2(p.x + (sz.x - ts.x) * 0.5f, p.y + sz.y * 0.5f - g_fontBody->FontSize * 0.5f),
				Pal::U32(Pal::Dim), lbl.c_str());
			ImGui::Dummy(sz);
		}
		else if (IconButton("chapwarnok", "I understand", nullptr, ImVec2(bw - cancelW - bgap, 46.0f), BtnKind::Primary))
		{
			bool previousAck = CalCtx.chaperoneWarningAck;
			CalCtx.chaperoneWarningAck = true;
			if (ProtectChaperone())
				ImGui::CloseCurrentPopup();
			else
			{
				CalCtx.chaperoneWarningAck = previousAck;
				// A failed capture may have persisted the fail-closed chaperone
				// state immediately; keep the acknowledgement rollback consistent.
				if (!SaveSettings(CalCtx))
				{
					CalCtx.persistence.MarkSettings(CalCtx.timeLastTick);
				}
			}
		}
		ImGui::SameLine(0.0f, bgap);
		if (IconButton("chapwarncancel", "Cancel", nullptr, ImVec2(cancelW, 46.0f), BtnKind::Ghost) || EscapePressed())
			ImGui::CloseCurrentPopup();

		ImGui::EndPopup();
	}
}
