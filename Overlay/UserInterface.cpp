// Window entry point: header, content child, pinned band and footer.
#include "stdafx.h"
#include "UiInternal.h"


bool g_uiPreviewMode = false;
bool g_uiPreviewMany = false;
PreviewScenario g_uiPreviewScenario = PreviewScenario::Healthy;
ImFont *g_fontBody = nullptr;
ImFont *g_fontSmall = nullptr;
ImFont *g_fontTitle = nullptr;

IdentifyPulseState g_identifyPulse;

void UpdateIdentifyPulse(double now)
{
	if (!g_identifyPulse.active)
		return;
	auto system = vr::VRSystem();
	if (!system || g_identifyPulse.pulsesRemaining == 0)
	{
		g_identifyPulse = IdentifyPulseState();
		return;
	}

	// Main-loop ownership makes shutdown safe. Wake at the original 5 ms
	// cadence without creating detached workers or overlapping pulse trains.
	CalCtx.wantedUpdateInterval = std::min(CalCtx.wantedUpdateInterval, 0.005);
	if (now < g_identifyPulse.nextPulseTime)
		return;
	if (g_identifyPulse.targetId < vr::k_unMaxTrackedDeviceCount)
		system->TriggerHapticPulse(g_identifyPulse.targetId, 0, 2000);
	if (g_identifyPulse.referenceId < vr::k_unMaxTrackedDeviceCount)
		system->TriggerHapticPulse(g_identifyPulse.referenceId, 0, 2000);
	--g_identifyPulse.pulsesRemaining;
	g_identifyPulse.nextPulseTime = now + 0.005;
}

// ---------------------------------------------------------------------------
// Header
// ---------------------------------------------------------------------------

void BuildHeader()
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImVec2 p = ImGui::GetCursorScreenPos();
	float cw = ImGui::GetWindowContentRegionWidth();
	const float h = 44.0f;

	// Logo mark
	ImVec2 lp = ImVec2(p.x, p.y + (h - 38.0f) * 0.5f);
	dl->AddRectFilled(lp, ImVec2(lp.x + 38, lp.y + 38), Pal::U32(Pal::Card), 11.0f);
	dl->AddRect(lp, ImVec2(lp.x + 38, lp.y + 38), Pal::U32(Pal::Border), 11.0f);
	IconLogo(dl, ImVec2(lp.x + 19, lp.y + 19), 12.0f, Pal::U32(Pal::Text));

	dl->AddText(g_fontTitle, g_fontTitle->FontSize,
		ImVec2(p.x + 52.0f, p.y + (h - g_fontTitle->FontSize) * 0.5f),
		Pal::U32(Pal::Text), "QuestCalibrator");

	// Gear (settings) toggle at the far right
	const float gearS = 38.0f;
	ImVec2 gp = ImVec2(p.x + cw - gearS, p.y + (h - gearS) * 0.5f);
	ImGui::SetCursorScreenPos(gp);
	if (ImGui::InvisibleButton("##settingsgear", ImVec2(gearS, gearS)))
		s_showSettings = !s_showSettings;
	bool gearHov = ImGui::IsItemHovered();
	{
		ImVec4 bg = s_showSettings ? ImVec4(Pal::Accent.x, Pal::Accent.y, Pal::Accent.z, 0.20f)
			: (gearHov ? Pal::CardHov : Pal::Card);
		dl->AddRectFilled(gp, ImVec2(gp.x + gearS, gp.y + gearS), Pal::U32(bg), 10.0f);
		dl->AddRect(gp, ImVec2(gp.x + gearS, gp.y + gearS),
			Pal::U32(s_showSettings ? Pal::Accent : (gearHov ? Pal::BorderHov : Pal::Border)), 10.0f);
		IconGear(dl, ImVec2(gp.x + gearS * 0.5f, gp.y + gearS * 0.5f), 9.0f,
			Pal::U32(s_showSettings || gearHov ? Pal::Text : Pal::Dim));
	}

	ImGui::SetCursorScreenPos(p);
	ImGui::Dummy(ImVec2(cw, h));
}

void BuildFooter(bool runningInOverlay)
{
	auto &io = ImGui::GetIO();
	float cw = ImGui::GetWindowContentRegionWidth();
	// ---- Footer ----
	{
		float footerY = ImGui::GetWindowHeight() - 40.0f;
		if (ImGui::GetCursorPosY() < footerY)
			ImGui::SetCursorPosY(footerY);
		else
			ImGui::Spacing();

		ImGui::PushFont(g_fontSmall);
		ImGui::TextColored(Pal::Faint, "QuestCalibrator v" QUESTCAL_VERSION_STRING "  -  by");
		ImGui::SameLine(0.0f, 4.0f);
		LinkText("VividNightmareUnleashed", "https://github.com/VividNightmareUnleashed");
		ImGui::SameLine(0.0f, 4.0f);
		ImGui::TextColored(Pal::Faint, "(");
		ImGui::SameLine(0.0f, 0.0f);
		LinkText("Jinxxy", "https://jinxxy.com/VividNightmare");
		ImGui::SameLine(0.0f, 0.0f);
		ImGui::TextColored(Pal::Faint, ")  -  based on");
		ImGui::SameLine(0.0f, 4.0f);
		LinkText("OpenVR-SpaceCalibrator", "https://github.com/pushrax/OpenVR-SpaceCalibrator");
		ImGui::SameLine(0.0f, 4.0f);
		ImGui::TextColored(Pal::Faint, "by pushrax");
		// The keyboard works (arrows and Enter), and nothing else says so:
		// a quiet line once the pointer has rested, or once it is in use.
		static double s_lastMouseMove = 0.0;
		if (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f)
			s_lastMouseMove = ImGui::GetTime();
		const bool keyboardHint = io.NavVisible || ImGui::GetTime() - s_lastMouseMove > 6.0;
		if (!runningInOverlay && keyboardHint)
		{
			const char *hint = "Arrow keys move, Enter presses";
			ImVec2 ts = ImGui::CalcTextSize(hint);
			ImGui::SameLine(cw - ts.x);
			ImGui::TextColored(Pal::Faint, hint);
		}
		if (runningInOverlay)
		{
			const char *hint = "Close VR overlay to use mouse";
			ImVec2 ts = ImGui::CalcTextSize(hint);
			ImGui::SameLine(cw - ts.x);
			ImGui::TextColored(Pal::Faint, hint);
		}
		ImGui::PopFont();
	}
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

// The main window never scrolls itself: the content child does. Without
// NoScrollbar the pinned band below the child makes the window reserve a
// scrollbar column and everything narrows by it.
static const ImGuiWindowFlags bareWindowFlags =
	ImGuiWindowFlags_NoTitleBar |
	ImGuiWindowFlags_NoResize |
	ImGuiWindowFlags_NoMove |
	ImGuiWindowFlags_NoScrollbar |
	ImGuiWindowFlags_NoScrollWithMouse;

void BuildMainWindow(bool runningInOverlay)
{
	auto &io = ImGui::GetIO();

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiSetCond_Always);
	ImGui::SetNextWindowSize(io.DisplaySize, ImGuiSetCond_Always);

	if (!ImGui::Begin("MainWindow", nullptr, bareWindowFlags))
	{
		ImGui::End();
		return;
	}

	// The device list changes on the order of minutes, but this runs at the
	// ~90 Hz dashboard frame rate — re-querying vrserver's properties (plus
	// per-device icon disk stats) every frame is a cross-process call storm.
	// A 1 Hz refresh keeps the panes current without it.
	static VRState state;
	static double lastStateRefresh = -1e9;
	double now = ImGui::GetTime();
	UpdateIdentifyPulse(now);
	if (now - lastStateRefresh >= 1.0)
	{
		state = LoadVRState();
		lastStateRefresh = now;
	}

	BuildHeader();
	ImGui::Spacing();
	if (!CalCtx.uiError.empty())
	{
		ImGui::PushStyleColor(ImGuiCol_Text, Pal::Bad);
		ImGui::TextWrapped("%s", CalCtx.uiError.c_str());
		ImGui::PopStyleColor();
		if (ImGui::SmallButton("Dismiss error"))
		{
			CalCtx.uiError.clear();
			CalCtx.uiErrorSource = CalibrationContext::ErrorSource::None;
		}
		ImGui::Spacing();
	}
	// Everything below the header scrolls in its own region, so the header
	// and its way back out of Settings stay put when the settings list runs
	// past the window.
	// NavFlattened: keyboard focus walks straight from the header into the
	// content's controls instead of stopping on the child as one item.
	ImGui::BeginChild("##content",
		ImVec2(0.0f, ImGui::GetWindowHeight() - ImGui::GetCursorPosY() - s_bottomReserve), false,
		ImGuiWindowFlags_NavFlattened);
	// The settings screen replaces the whole content area; keeping the device
	// panes above it buried the settings below the fold for no benefit.
	bool inSettings = (CalCtx.state == CalibrationState::None && s_showSettings);
	if (!inSettings)
	{
		BuildSpacesSection(state);
		ImGui::Spacing();
	}
	BuildMenu(state, runningInOverlay);
	ImGui::EndChild();

	// The pinned bottom block: the band on the main screen, the footer
	// everywhere.
	if (!inSettings && CalCtx.state == CalibrationState::None)
		BuildStatusBand(state);
	else
		s_bottomReserve = 48.0f;
	BuildFooter(runningInOverlay);

	ImGui::End();
}
