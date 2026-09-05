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
static GLuint s_guideTexture = 0;
static int s_guideTextureKind = -1;

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
	s_guide.openRequested = true;
	BOOL animate = TRUE;
	if (SystemParametersInfoW(SPI_GETCLIENTAREAANIMATION, 0, &animate, 0))
		s_guide.animate = animate != FALSE;
	if (!s_guide.animate)
		s_guide.animationTime = 3.3;
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
	s_guide.metrics = questcal::GuideMetrics();
	s_guide.lastMetricsTime = 0.0;
	if (g_uiPreviewMode)
	{
		CalCtx.ClearMessages();
		CalCtx.Log(FormatString("Preview run: reference device %u, target device %u\n",
			CalCtx.referenceID, CalCtx.targetID));
		if (s_guide.demo != GuideDemo::Wrist)
		{
			CalCtx.Instruct("Look around slowly.");
			CalCtx.Note("Look left and right, then up and down. Gently tilt your head to each side. Keep the tracker in view of the base stations.");
		}
		else
		{
			CalCtx.Instruct("Keep both devices firmly together.");
			CalCtx.Note("Move both devices in a figure eight, gently turning and tilting as you go. Keep them firmly together and in view of their tracking cameras or base stations.");
		}
		CalCtx.Progress(0, static_cast<int>(CalCtx.CollectionSeconds() * 100.0));
		s_guide.metrics.valid = true;
		s_guide.metrics.coverage = 0.55;
		s_guide.metrics.gatedFraction = 0.08;
		s_guide.metrics.rigidityValid = true;
		s_guide.metrics.rigidityDeg = 1.4;
		return true;
	}
	return s_guide.anchor ? StartAnchorCalibration() : StartCalibration();
}

// One atlas is resident at a time (at most 112.5 MiB).
// Resources are embedded so a moved executable cannot lose its instructions.
void DrawGuideAnimation(ImDrawList *dl, ImVec2 origin, ImVec2 size, double t, GuideDemo demo)
{
	auto &texture = s_guideTexture;
	auto &loadedKind = s_guideTextureKind;
	const int kind = static_cast<int>(demo);
	if (kind != loadedKind)
	{
		if (texture)
			glDeleteTextures(1, &texture);
		texture = 0;
		LoadGuideTexture(demo, &texture);
		loadedKind = kind;
	}
	if (!texture)
	{
		dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(origin.x, origin.y + 24.0f),
			Pal::U32(Pal::Warn), "Motion demos couldn't load. Reinstall QuestCalibrator to restore them.");
		return;
	}
	const bool wrist = demo == GuideDemo::Wrist;
	const char *headLabels[] = { "Look left and right", "Look up and down", "Tilt side to side" };
	const float gap = 16.0f;
	const float cellW = (size.x - gap * 2.0f) / 3.0f;
	const float imageH = size.y - 34.0f;
	const float imageW = imageH * (wrist ? 5.0f / 3.0f : 8.0f / 7.0f);
	const int columns = wrist ? 16 : 24;
	const float frameW = wrist ? 320.0f : 192.0f;
	const float frameH = wrist ? 192.0f : 168.0f;
	const float atlasW = wrist ? 5120.0f : 4608.0f;
	const float atlasH = wrist ? 5760.0f : 5880.0f;
	if (demo == GuideDemo::Mounted)
		t += 2.3;
	const float blend = static_cast<float>(std::clamp((t - 2.0) / 0.3, 0.0, 1.0));
	const float motionAlpha = blend * blend * (3.0f - 2.0f * blend);
	auto drawFrame = [&](int index, ImVec2 top, float opacity)
	{
		if (!texture || opacity <= 0.0f)
			return;
		const int column = index % columns;
		const int row = index / columns;
		// Half-texel inset keeps linear filtering inside this frame.
		const ImVec2 uv0((column * frameW + 0.5f) / atlasW, (row * frameH + 0.5f) / atlasH);
		const ImVec2 uv1((column * frameW + frameW - 0.5f) / atlasW, (row * frameH + frameH - 0.5f) / atlasH);
		dl->AddImage(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(texture)), top,
			ImVec2(top.x + imageW, top.y + imageH), uv0, uv1,
			ImGui::ColorConvertFloat4ToU32(ImVec4(1, 1, 1, opacity)));
	};
	if (wrist)
	{
		const int frame = t < 2.0 ? std::min(119, static_cast<int>(t * 60.0))
			: 120 + static_cast<int>(std::fmod(t - 2.0, 6.0) * 60.0) % 360;
		drawFrame(frame, ImVec2(origin.x + (size.x - imageW) * 0.5f, origin.y),
			static_cast<float>(std::clamp(t / 0.2, 0.0, 1.0)));
		const char *label = t < 2.0 ? "Bring the controller to the wrist tracker" : "Turn and tilt as you move in a figure eight";
		const ImVec2 textSize = g_fontBody->CalcTextSizeA(g_fontBody->FontSize, FLT_MAX, 0.0f, label);
		dl->AddText(g_fontBody, g_fontBody->FontSize,
			ImVec2(origin.x + (size.x - textSize.x) * 0.5f, origin.y + imageH + 8.0f), Pal::U32(Pal::Text), label);
		return;
	}
	if (motionAlpha < 1.0f)
	{
		const float opacity = (1.0f - motionAlpha) * static_cast<float>(std::clamp(t / 0.2, 0.0, 1.0));
		drawFrame(std::min(119, static_cast<int>(t * 60.0)),
			ImVec2(origin.x + (size.x - imageW) * 0.5f, origin.y), opacity);
		const char *label = demo == GuideDemo::HeadsetContact ? "Rest the controller against the visor" : "Bring the controller to the wrist tracker";
		const ImVec2 textSize = g_fontBody->CalcTextSizeA(g_fontBody->FontSize, FLT_MAX, 0.0f, label);
		ImVec4 color = Pal::Text;
		color.w *= opacity;
		dl->AddText(g_fontBody, g_fontBody->FontSize,
			ImVec2(origin.x + (size.x - textSize.x) * 0.5f, origin.y + imageH + 8.0f), Pal::U32(color), label);
	}
	const int frame = static_cast<int>(std::fmod(std::max(0.0, t - 2.3), 4.0) * 60.0) % 240;
	for (int axis = 0; axis < 3; ++axis)
	{
		const float x = origin.x + (cellW + gap) * static_cast<float>(axis);
		const ImVec2 top(x + (cellW - imageW) * 0.5f, origin.y);
		drawFrame(120 + axis * 240 + frame, top, motionAlpha);
		const char *label = headLabels[axis];
		const ImVec2 textSize = g_fontBody->CalcTextSizeA(g_fontBody->FontSize, FLT_MAX, 0.0f, label);
		ImVec4 color = Pal::Text;
		color.w *= motionAlpha;
		dl->AddText(g_fontBody, g_fontBody->FontSize,
			ImVec2(x + (cellW - textSize.x) * 0.5f, origin.y + imageH + 8.0f), Pal::U32(color), label);
	}
}

void DrawGuideIndicators(ImDrawList *dl, ImVec2 origin, float width, const questcal::GuideMetrics &m, bool mountRun)
{
	const float cellW = (width - 32.0f) / 3.0f;
	const char *names[] = { "Motion variety", "Movement speed", mountRun ? "Tracker stability" : "Device stability" };
	const char *states[] = {
		!m.valid ? "Measuring..." : m.coverage >= 0.99 ? "Enough variety" : "Keep turning and tilting",
		!m.valid ? "Measuring..." : m.gatedFraction < 0.15 ? "Good pace" : "Move more slowly",
		!m.rigidityValid ? "Measuring..." : m.rigidityDeg < 3.0 ? "Moving together" : "Movement doesn't match"
	};
	const double values[] = { m.coverage, m.valid ? 1.0 - m.gatedFraction : 0.0,
		m.rigidityValid ? std::clamp(1.0 - (m.rigidityDeg - 1.0) / 8.0, 0.0, 1.0) : 0.0 };
	for (int i = 0; i < 3; ++i)
	{
		const float x = origin.x + (cellW + 16.0f) * static_cast<float>(i);
		dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(x, origin.y), Pal::U32(Pal::Text), names[i]);
		dl->AddText(g_fontSmall, g_fontSmall->FontSize, ImVec2(x, origin.y + 28.0f), Pal::U32(Pal::Dim), states[i]);
		const ImVec2 a(x, origin.y + 50.0f), b(x + cellW, origin.y + 54.0f);
		dl->AddRectFilled(a, b, Pal::U32(Pal::Border), 2.0f);
		const float fill = static_cast<float>(std::clamp(values[i], 0.0, 1.0));
		if (fill > 0.0f)
			dl->AddRectFilled(a, ImVec2(x + cellW * fill, b.y), Pal::U32(Pal::Accent), 2.0f);
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
	if ((s_guide.stage == GuideStage::Done || s_guide.stage == GuideStage::Idle) && s_guideTexture)
	{
		glDeleteTextures(1, &s_guideTexture);
		s_guideTexture = 0;
		s_guideTextureKind = -1;
	}


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
	// Popup IDs depend on the current window. The pinned recovery button is
	// outside this content child, so open its request in the modal's scope.
	if (s_guide.openRequested)
	{
		s_guide.demo = s_guide.mountRun ? GuideDemo::Mounted : GuideDemo::Wrist;
		bool headsetReference = false;
		bool controllerTarget = false;
		for (const auto &device : state.devices)
		{
			if (static_cast<uint32_t>(device.id) == CalCtx.referenceID)
				headsetReference = device.deviceClass == vr::TrackedDeviceClass_HMD;
			if (static_cast<uint32_t>(device.id) == CalCtx.targetID)
				controllerTarget = device.deviceClass == vr::TrackedDeviceClass_Controller;
		}
		if (!s_guide.mountRun && headsetReference)
			s_guide.demo = controllerTarget ? GuideDemo::HeadsetContact : GuideDemo::Mounted;
		ImGui::OpenPopup("Calibration Progress");
		s_guide.openRequested = false;
	}
	const double now = ImGui::GetTime();
	// Stage transitions the context drives: a run that ended, any way,
	// moves to the outcome; a finished countdown starts the run.
	if (s_guide.stage == GuideStage::Running && !g_uiPreviewMode &&
		CalCtx.state == CalibrationState::None)
		s_guide.stage = GuideStage::Done;
	if (s_guide.stage == GuideStage::Running && g_uiPreviewMode &&
		now - s_guide.countdownStart > kCountdownSeconds + CalCtx.CollectionSeconds())
	{
		// Preview: a fake outcome so the result stage can be styled, a
		// refused solve under -uipreview-failed and a success otherwise.
		if (g_uiPreviewScenario == PreviewScenario::Failed)
		{
			CalCtx.lastRunHint = CalibrationContext::GuideHint::RotateMore;
			CalCtx.Outcome("Calibration failed", "The devices didn't rotate in enough directions.",
				"Turn and tilt both devices together, then try again.",
				"Rotation coverage 0.21 of 1.00 (need 0.60)", CalibrationContext::Tone::Warn);
		}
		else
		{
			CalCtx.lastRunHint = CalibrationContext::GuideHint::Success;
			CalCtx.Outcome("Calibration complete", "Check that the tracker positions line up in VR.", "", "", CalibrationContext::Tone::Good);
		}
		s_guide.stage = GuideStage::Done;
	}
	const bool showingResult = s_guide.stage == GuideStage::Done;
	float modalW = showingResult ? 660.0f : 940.0f;
	ImGui::SetNextWindowPos(ImVec2((io.DisplaySize.x - modalW) * 0.5f, showingResult ? 180.0f : 60.0f), ImGuiSetCond_Always);
	ImGui::SetNextWindowSize(ImVec2(modalW, 0.0f), ImGuiSetCond_Always);
	if (ImGui::BeginPopupModal("Calibration Progress", nullptr,
		ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove))
	{
		using Msg = CalibrationContext::Message;
		const float mw = ImGui::GetWindowContentRegionWidth();
		ImDrawList *mdl = ImGui::GetWindowDrawList();
		if (s_guide.stage == GuideStage::Countdown || s_guide.stage == GuideStage::Running ||
			(s_guide.stage == GuideStage::GetSet && s_guide.animate))
			CalCtx.wantedUpdateInterval = std::min(CalCtx.wantedUpdateInterval, 1.0 / 60.0);

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

		if (s_guide.animate)
			s_guide.animationTime += std::min(io.DeltaTime, 0.05f);
		const ImVec2 artSize(mw, s_guide.demo == GuideDemo::Wrist ? 330.0f : 230.0f);
		switch (s_guide.stage)
		{
		case GuideStage::GetSet:
		case GuideStage::Countdown:
		{
			ImGui::TextColored(Pal::Dim, "Step 1 of 3");
			ImGui::Spacing();
			ImGui::PushFont(g_fontTitle);
			ImGui::TextWrapped("%s", s_guide.anchor ? "Stand where the trackers look misaligned."
				: s_guide.demo == GuideDemo::HeadsetContact ? "Hold the controller against the visor."
				: s_guide.demo == GuideDemo::Mounted ? "Keep the tracker fixed to your headset."
				: "Hold the controller against the wrist tracker.");
			ImGui::PopFont();
			const std::string how = s_guide.demo == GuideDemo::HeadsetContact
				? FormatString("Hold the controller upright, with the trigger side against the front of your headset. Keep it in place as you turn and tilt your head for %.0f seconds.", CalCtx.CollectionSeconds())
				: s_guide.demo == GuideDemo::Mounted
				? FormatString("Move your head gently for %.0f seconds, keeping your body relaxed. Keep the tracker sensors uncovered.", CalCtx.CollectionSeconds())
				: FormatString("Hold the controller against the wrist tracker with your other hand. Move both in a figure eight, gently turning and tilting, for %.0f seconds.", CalCtx.CollectionSeconds());
			ImGui::TextWrapped("%s", how.c_str());
			ImGui::Spacing();
			const ImVec2 row = ImGui::GetCursorScreenPos();
			DrawGuideAnimation(mdl, row, artSize, s_guide.animationTime, s_guide.demo);
			ImGui::SetCursorScreenPos(ImVec2(row.x, row.y + artSize.y + 12.0f));
			const ImVec2 readiness = ImGui::GetCursorScreenPos();

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
				const bool ok = picks[i] && picks[i]->tracking;
				ready = ready && ok;
				const std::string who = picks[i] ? DeviceDisplayName(*picks[i])
					: std::string(i == 0 ? "Reference device" : "Target device");
				ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + mw - 332.0f);
				ImGui::TextWrapped("%s is %stracking", who.c_str(), ok ? "" : "not ");
				ImGui::PopTextWrapPos();
			}
			const float readinessBottom = ImGui::GetCursorScreenPos().y;
			ImGui::SetCursorScreenPos(ImVec2(readiness.x + mw - 312.0f, readiness.y));
			if (IconButton("motionreplay", "Replay", nullptr, ImVec2(110.0f, 38.0f), BtnKind::Quiet))
			{
				s_guide.animationTime = 0.0;
				s_guide.animate = true;
			}
			ImGui::SameLine(0.0f, 12.0f);
			if (IconButton("motiontoggle", s_guide.animate ? "Pause motion" : "Play motion", nullptr,
				ImVec2(190.0f, 38.0f), BtnKind::Ghost))
				s_guide.animate = !s_guide.animate;
			ImGui::SetCursorScreenPos(ImVec2(row.x, std::max(readinessBottom, readiness.y + 38.0f) + 8.0f));

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
			if (g_uiPreviewMode)
				CalCtx.Progress(static_cast<int>((now - s_guide.countdownStart - kCountdownSeconds) * 100.0),
					static_cast<int>(CalCtx.CollectionSeconds() * 100.0));
			ImGui::TextColored(Pal::Dim, "Step 2 of 3");
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
			DrawGuideAnimation(mdl, row, artSize, s_guide.animationTime, s_guide.demo);
			DrawGuideIndicators(mdl, ImVec2(row.x, row.y + artSize.y + 12.0f), mw, s_guide.metrics, s_guide.mountRun);
			ImGui::SetCursorScreenPos(ImVec2(row.x, row.y + artSize.y + 80.0f));
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
			if (IconButton("cancelprogress", "Cancel", nullptr, ImVec2(mw - 202.0f, 46.0f), BtnKind::Ghost) || EscapePressed())
			{
				if (g_uiPreviewMode)
				{
					s_guide.stage = GuideStage::Idle;
					ImGui::CloseCurrentPopup();
				}
				else
					CancelCalibration();
			}
			ImGui::SameLine(0.0f, 12.0f);
			if (IconButton("runningmotion", s_guide.animate ? "Pause motion" : "Play motion", nullptr,
				ImVec2(190.0f, 46.0f), BtnKind::Ghost))
				s_guide.animate = !s_guide.animate;
			break;
		}
		case GuideStage::Done:
		default:
		{
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

			const auto hint = CalCtx.lastRunHint;
			ImGui::Dummy(ImVec2(0, 16.0f));

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
				const float closeW = 120.0f;
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
			else
			{
				ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - 150.0f);
				if (IconButton("closeprogress", "Done", nullptr, ImVec2(150.0f, 46.0f), BtnKind::Primary) || EscapePressed())
				{
					s_guide.stage = GuideStage::Idle;
					ImGui::CloseCurrentPopup();
				}
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
		ImGui::TextWrapped("This also removes the saved field anchors and headset tracker setup.");
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
		ImGui::PushFont(g_fontTitle);
		ImGui::TextUnformatted("Check your room boundaries");
		ImGui::PopFont();
		ImGui::Spacing();

		ImGui::TextWrapped(
			"QuestCalibrator saves and restores your SteamVR chaperone. "
			"Tracking drift can still move those virtual walls away from the real room boundaries.");
		ImGui::Spacing();
		ImGui::TextWrapped(
			"Keep the Quest's own boundary enabled too. A saved chaperone "
			"doesn't guarantee that your play area is clear or correctly aligned.");
		ImGui::Spacing();
		ImGui::PushStyleColor(ImGuiCol_Text, Pal::Violet);
		ImGui::TextWrapped(
			"Before playing, check that the virtual walls match your room and leave "
			"enough space to move safely, especially when dancing.");
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
