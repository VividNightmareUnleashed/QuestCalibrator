#pragma once

struct ImFont;

// UI preview mode (-uipreview): run the window with fake devices and no
// SteamVR, for iterating on the interface. Profile saves are disabled.
// -uipreview-many additionally fakes 8 trackers to stress the device list.
extern bool g_uiPreviewMode;
extern bool g_uiPreviewMany;

// Fonts loaded by QuestCalibrator.cpp at window creation.
extern ImFont *g_fontBody;
extern ImFont *g_fontSmall;
extern ImFont *g_fontTitle;

void ApplyTheme();
void BuildMainWindow(bool runningInOverlay);
