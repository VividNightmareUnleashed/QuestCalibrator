#pragma once

struct ImFont;

// UI preview mode (-uipreview): run the window with fake devices and no
// SteamVR, for iterating on the interface. Profile saves are disabled.
// -uipreview-many additionally fakes a batch of trackers to stress the device
// list past its scroll threshold (count: kPreviewManyTrackerCount).
extern bool g_uiPreviewMode;
extern bool g_uiPreviewMany;

// Preview scenarios cover frozen alignment, a failed solve, an empty profile,
// guide setup, and a completed result. Each implies -uipreview-many.
enum class PreviewScenario { Healthy, Frozen, Failed, Empty, Guide, Result };
extern PreviewScenario g_uiPreviewScenario;

// Fonts loaded by QuestCalibrator.cpp at window creation.
extern ImFont *g_fontBody;
extern ImFont *g_fontSmall;
extern ImFont *g_fontTitle;

void ApplyTheme();
void SetupPreviewState();
void BuildMainWindow(bool runningInOverlay);
void RequestApplicationExit();
