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
MainTab s_mainTab = MainTab::Calibration;

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
	float cw = ImGui::GetContentRegionAvail().x;
	const float h = 44.0f;
	static const char *const tabs[] = { "Calibration", "Lighthouse", "Smoothing" };
	static const char *const disabledTips[] = { nullptr,
		"Lighthouse module is currently not installed. Select it during installation.",
		"Work in progress" };

	// One row: the brand at the left, the gear at the right, and the tab
	// switch centred on the row as a whole rather than on what is left
	// between them, so it stays put when the title's width changes.
	FlexLayout fl;
	YGNodeRef row = fl.Root();
	YGNodeStyleSetHeight(row, h);
	YGNodeStyleSetAlignItems(row, YGAlignCenter);
	YGNodeStyleSetJustifyContent(row, YGJustifySpaceBetween);

	YGNodeRef brand = fl.Row(row);
	YGNodeStyleSetAlignItems(brand, YGAlignCenter);
	YGNodeStyleSetGap(brand, YGGutterColumn, 14.0f);
	YGNodeRef logo = fl.Add(brand);
	YGNodeStyleSetWidth(logo, 38.0f);
	YGNodeStyleSetHeight(logo, 38.0f);
	YGNodeRef title = fl.Text(brand, g_fontTitle, "QuestCalibrator");

	YGNodeRef tabsNode = fl.Add(row);
	YGNodeStyleSetPositionType(tabsNode, YGPositionTypeAbsolute);
	YGNodeStyleSetPositionPercent(tabsNode, YGEdgeLeft, 50.0f);
	YGNodeStyleSetWidth(tabsNode, SegmentedTabsWidth(tabs, 3));
	YGNodeStyleSetHeight(tabsNode, SegmentedTabsHeight());
	YGNodeStyleSetMargin(tabsNode, YGEdgeLeft, -SegmentedTabsWidth(tabs, 3) * 0.5f);

	YGNodeRef gear = fl.Add(row);
	YGNodeStyleSetWidth(gear, 38.0f);
	YGNodeStyleSetHeight(gear, 38.0f);
	fl.Compute(p, cw, h);

	// Logo mark
	const FlexRect lr = fl.Rect(logo);
	dl->AddRectFilled(lr.min, lr.max, Pal::U32(Pal::Card), 11.0f);
	dl->AddRect(lr.min, lr.max, Pal::U32(Pal::Border), 11.0f);
	IconLogo(dl, lr.Center(), 12.0f, Pal::U32(Pal::Text));

	dl->AddText(g_fontTitle, g_fontTitle->LegacySize, fl.Rect(title).min,
		Pal::U32(Pal::Text), "QuestCalibrator");

	// The tab switch. Picking a tab is also the way back out of Settings.
	// Lighthouse and Smoothing stay greyed out: they will ship as modules
	// picked in the installer.
	{
		ImGui::SetCursorScreenPos(fl.Rect(tabsNode).min);
		const int picked = SegmentedTabs("maintab", static_cast<int>(s_mainTab), tabs, 3,
			(1u << static_cast<int>(MainTab::Lighthouse)) | (1u << static_cast<int>(MainTab::Smoothing)),
			disabledTips);
		if (picked != static_cast<int>(s_mainTab))
		{
			s_mainTab = static_cast<MainTab>(picked);
			s_showSettings = false;
		}
	}

	// Gear (settings) toggle at the far right
	const FlexRect gr = fl.Rect(gear);
	ImGui::SetCursorScreenPos(gr.min);
	if (ImGui::InvisibleButton("##settingsgear", gr.Size(), ImGuiButtonFlags_EnableNav))
		s_showSettings = !s_showSettings;
	bool gearHov = ImGui::IsItemHovered();
	{
		ImVec4 bg = s_showSettings ? ImVec4(Pal::Accent.x, Pal::Accent.y, Pal::Accent.z, 0.20f)
			: (gearHov ? Pal::CardHov : Pal::Card);
		dl->AddRectFilled(gr.min, gr.max, Pal::U32(bg), 10.0f);
		dl->AddRect(gr.min, gr.max,
			Pal::U32(s_showSettings ? Pal::Accent : (gearHov ? Pal::BorderHov : Pal::Border)), 10.0f);
		IconGear(dl, gr.Center(), 9.0f,
			Pal::U32(s_showSettings || gearHov ? Pal::Text : Pal::Dim));
	}

	ImGui::SetCursorScreenPos(p);
	ImGui::Dummy(ImVec2(cw, h));
}

void BuildFooter(bool runningInOverlay)
{
	auto &io = ImGui::GetIO();
	float cw = ImGui::GetContentRegionAvail().x;
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
			const char *hint = Tr("Arrow keys to move \xC2\xB7 Enter to select");
			ImVec2 ts = ImGui::CalcTextSize(hint);
			ImGui::SameLine(cw - ts.x);
			ImGui::TextColored(Pal::Faint, hint);
		}
		if (runningInOverlay)
		{
			const char *hint = Tr("Close the SteamVR dashboard to use the mouse");
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

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
	ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);

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
		ImGui::TextWrapped("%s", Tr(CalCtx.uiError.c_str()));
		ImGui::PopStyleColor();
		if (ImGui::SmallButton((std::string(Tr("Dismiss")) + "###dismisserror").c_str()))
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
		ImVec2(0.0f, ImGui::GetWindowHeight() - ImGui::GetCursorPosY() - s_bottomReserve),
		ImGuiChildFlags_NavFlattened);
	// The settings screen replaces the whole content area; keeping the device
	// panes above it buried the settings below the fold for no benefit.
	bool inSettings = (CalCtx.state == CalibrationState::None && s_showSettings);
	// The Lighthouse tab owns the content area only while nothing else
	// does: a calibration in progress or the profile editor keeps its
	// screen whatever the tab says.
	const bool lighthouseTab = !inSettings && CalCtx.state == CalibrationState::None &&
		s_mainTab == MainTab::Lighthouse;
	if (lighthouseTab)
	{
		BuildLighthouseScreen(state);
	}
	else
	{
		if (!inSettings)
		{
			BuildSpacesSection(state);
			ImGui::Spacing();
		}
		BuildMenu(state, runningInOverlay);
	}
	ImGui::EndChild();

	// The pinned bottom block: the band on the calibration screen, the
	// footer everywhere.
	if (!inSettings && !lighthouseTab && CalCtx.state == CalibrationState::None)
		BuildStatusBand(state);
	else
		s_bottomReserve = 48.0f;
	BuildFooter(runningInOverlay);

	ImGui::End();
}
