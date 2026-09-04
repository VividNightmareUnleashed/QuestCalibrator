#pragma once

// Internal to the overlay UI: shared types, palette, widgets and the
// screen builders, split across the Ui*.cpp files. Nothing here is an
// API for the rest of the program; that is UserInterface.h.

#include "stdafx.h"
#include "UserInterface.h"
#include "Calibration.h"
#include "CalibrationGuide.h"
#include "Configuration.h"
#include "ProfileValidation.h"
#include "Updater.h"
#include "../common/Protocol.h"
#include "../common/Version.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <GL/gl3w.h>
#include <wincodec.h>
#include <shellapi.h>
#pragma comment(lib, "windowscodecs.lib")
#pragma comment(lib, "shell32.lib")

struct VRDevice
{
	int id = -1;
	vr::TrackedDeviceClass deviceClass;
	std::string model;
	std::string serial;
	std::string trackingSystem;
	std::string iconPath;   // absolute path to the SteamVR device icon, matched to state (ready/low/off)
	vr::ETrackedControllerRole controllerRole = vr::TrackedControllerRole_Invalid;
	bool connected = true;
	bool tracking = false;       // pose valid and Running_OK at the last 1 Hz refresh
	float battery = -1.0f;       // 0..1, or -1 when the device reports none
	bool charging = false;
};

struct VRState
{
	std::vector<std::string> trackingSystems;
	std::vector<VRDevice> devices;
};

// Below this level the device icon swaps to its red low-battery art and the
// battery bar fill goes red.
static const float kLowBattery = 0.15f;

template<typename T>
static void SaveSettingOrRestore(T &value, const T &previous)
{
	if (!SaveSettingsWithResult(CalCtx).settingsSaved)
		value = previous;
}

// The profile-backed toggles do not have a counterpart here: they all go
// through Configuration.cpp's SaveProfileFieldEdit, which persists a candidate
// record before touching live state, so the UI expresses the edit and never a
// rollback. Every such call site sits inside an `if (validProfile)` block.

struct IdentifyPulseState
{
	bool active = false;
	uint32_t targetId = vr::k_unTrackedDeviceIndexInvalid;
	uint32_t referenceId = vr::k_unTrackedDeviceIndexInvalid;
	unsigned pulsesRemaining = 0;
	double nextPulseTime = 0.0;
};

namespace Pal
{
	const ImVec4 Bg       (0.039f, 0.043f, 0.051f, 1.00f);
	const ImVec4 Card     (0.075f, 0.080f, 0.093f, 1.00f);
	const ImVec4 CardHov  (0.100f, 0.107f, 0.124f, 1.00f);
	const ImVec4 Inset    (0.055f, 0.059f, 0.069f, 1.00f);
	const ImVec4 Border   (0.150f, 0.158f, 0.180f, 1.00f);
	// 3:1 on Card: the outline that says "this is a button" on a Ghost
	// button; card hairlines stay at Border.
	const ImVec4 GhostBorder(0.370f, 0.385f, 0.425f, 1.00f);
	const ImVec4 BorderHov(0.230f, 0.242f, 0.272f, 1.00f);
	const ImVec4 Text     (0.925f, 0.933f, 0.950f, 1.00f);
	const ImVec4 Dim      (0.520f, 0.550f, 0.610f, 1.00f);
	// 4.6:1 on Card: the quietest ink that still passes as body text in a
	// headset. Reserved for decoration and disabled controls; anything that
	// carries information (an age, a serial, a live state) uses Dim.
	const ImVec4 Faint    (0.470f, 0.500f, 0.560f, 1.00f);
	const ImVec4 Accent   (0.188f, 0.404f, 0.867f, 1.00f);
	const ImVec4 AccentHov(0.250f, 0.470f, 0.920f, 1.00f);
	const ImVec4 AccentAct(0.150f, 0.340f, 0.780f, 1.00f);
	const ImVec4 Good     (0.350f, 0.780f, 0.450f, 1.00f);
	const ImVec4 Warn     (0.930f, 0.700f, 0.300f, 1.00f);
	const ImVec4 Bad      (0.900f, 0.420f, 0.380f, 1.00f);
	const ImVec4 VeryBad  (0.920f, 0.300f, 0.280f, 1.00f);
	const ImVec4 Violet   (0.560f, 0.510f, 0.950f, 1.00f);
	const ImVec4 White    (1.000f, 1.000f, 1.000f, 1.00f);

	inline ImU32 U32(const ImVec4 &c, float alphaMul = 1.0f)
	{
		ImVec4 v = c;
		v.w *= alphaMul;
		return ImGui::GetColorU32(v);
	}
}

// Inline text that opens a URL in the default browser; underlined on hover.

typedef void (*IconFn)(ImDrawList *, ImVec2, float, ImU32);

struct DeviceIconTex
{
	GLuint tex = 0;
	int w = 0, h = 0;
	bool failed = false;
};

// Danger: ghost chrome in the error colour, for the destructive choice in a
// confirmation, so the accent stays the safe one.
enum class BtnKind { Primary, Ghost, Quiet, Danger };

static const float kRowHeight = 52.0f;
static const float kRowInsetX = 16.0f;
static const float kRowControlY = 14.0f;

// Remembers the height it opened with, so the end call cannot disagree with
// the begin call and silently overlap the next row.
ImVec2 BeginRowCard(float height);
void EndRowCard(ImVec2 p, float height);

struct RowCard
{
	explicit RowCard(float rowHeight) : pos(BeginRowCard(rowHeight)), height(rowHeight) {}
	RowCard(const RowCard &) = delete;
	RowCard &operator=(const RowCard &) = delete;
	~RowCard() { EndRowCard(pos, height); }

	ImVec2 pos;
	float height;
};

// One-sentence explanation under a row's label. Permanent rather than a
// tooltip: the only hover target a tooltip had was the 24 px checkbox, and
// the explanation then covered the very controls it described.
static const float kRowSubLineH = 22.0f;

struct StatusRowData
{
	IconFn icon;
	ImVec4 color;
	std::string text;
};

// Unknown is not a severity, it is the absence of one: nothing has been
// measured to rate. Sorting it below Good keeps every "bad enough to say
// something" test (>= Rating_Poor) reading false for it with no special case.
enum CalRating { Rating_Unknown = -1, Rating_Good = 0, Rating_Decent, Rating_Poor, Rating_VeryPoor };

// One derived answer to "what is the continuous loop actually doing", built
// from the same terms ContinuousTick gates its own shouldRun on. The status
// text, the rating, the advanced row's colour, the mount advisory and both
// recalibration nudges all read this instead of each re-deriving a slice of
// it -- which is how the state meaning "not running at all" came to render as
// "gathering" forever, and how the two screens came to nudge from two rules.
enum class ContinuousStatus
{
	Off,         // the feature is switched off
	NoTracker,   // enabled, but no mounted tracker picked
	NeedsMount,  // picked, but no mount offset learned for it yet
	NotRunning,  // armed, yet the loop's runtime preconditions do not hold
	Gathering,   // running, not enough observations yet (State::Inactive)
	Tracking,
	Coasting,
	Frozen,
	Holding,
};

// Pure over context fields, so asking again is free and no cached mirror of it
// can go stale.

enum class GuideStage { Idle, GetSet, Countdown, Running, Done };

struct GuideState
{
	GuideStage stage = GuideStage::Idle;
	bool anchor = false;       // field-anchor run
	bool mountRun = false;     // head-referenced run for the headset tracker
	double countdownStart = 0.0;
	double lastMetricsTime = -1.0;
	questcal::GuideMetrics metrics;
};

static const float kCountdownSeconds = 3.0f;

// How many trackers -uipreview-many fabricates. The only statement of the
// count: enough to push a tracking system's device list past BuildDeviceList's
// four-row scroll threshold, with room to fan out across battery levels.
static const int kPreviewManyTrackerCount = 6;

// Shared state, each owned by one file.
extern IdentifyPulseState g_identifyPulse;
extern bool s_showSettings;
extern double g_chapWarnOpenedAt;
extern GuideState s_guide;
extern bool s_modalDetails;
extern float s_bottomReserve;

// Functions shared across the Ui*.cpp files.
void LinkText(const char *label, const char *url);
float LetterSpacedWidth(ImFont *font, const char *text, float spacing);
void LetterSpacedTextAt(ImDrawList *dl, ImFont *font, ImVec2 pos, ImU32 col, const char *text, float spacing);
void SectionLabel(const char *text);
void IconLogo(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconHMD(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconController(ImDrawList *dl, ImVec2 c, float s, ImU32 col, bool leftHand);
void IconTracker(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconPlay(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconPencil(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconTrash(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconCopy(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconCrosshair(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconCheck(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconClock(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconDownload(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconInfo(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconPin(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconScale(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconGauge(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconField(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
void IconGear(ImDrawList *dl, ImVec2 c, float s, ImU32 col);
bool LoadTextureFromFile(const char *path, GLuint *outTex, int *outW, int *outH);
const DeviceIconTex *GetDeviceIconTex(const std::string &path);
bool FileExists(const std::string &path);
std::string Prefer2x(const std::string &path);
void DrawFocusRing(ImDrawList *dl, ImVec2 a, ImVec2 b, float rounding);
bool IconButton(const char *id, const char *label, IconFn icon, ImVec2 size, BtnKind kind, bool smallCaps = false);
bool QCCheckbox(const char *id, bool *v);
ImVec2 BeginRowCard(float height);
void EndRowCard(ImVec2 p, float height);
void RowIconLabel(ImVec2 rowPos, IconFn icon, const char *label);
void RowSubLine(ImVec2 rowPos, const char *text);
bool ToggleRow(const char *id, IconFn icon, const char *label, bool &value, const char *subline = nullptr);
bool EscapePressed();
void ShowTip(const char *text, bool leftOfCursor = false);
bool NestedToggle(const char *id, ImVec2 pos, float width, const char *label, bool &value, const char *tooltip);
int Segmented(const char *id, int value, const char *const items[], int count, float itemW, float h);
void DrawStatusCard(const std::vector<StatusRowData> &rows);
std::string FormatString(const char *fmt, ...);
bool PoseChannelDown();
ContinuousStatus ContinuousStatusNow();
const char *ContinuousStateWord(ContinuousStatus status);
const char *ContinuousStatusLine(ContinuousStatus status);
ImVec4 ContinuousStatusColor(ContinuousStatus status);
CalRating ComputeCalibrationRating(ContinuousStatus continuous);
const char *RatingLabel(CalRating r);
CalRating SolveQualityRating(const questcal::EngineResult &result);
ImVec4 RatingColor(CalRating r);
const char *RecalibrationNudge(CalRating rating);
std::optional<std::string> FormatUnixAge(double unixTime);
std::optional<std::string> FormatAlignmentAge();
bool ProtectChaperone();
void BuildStatusBand(const VRState &state);
void BuildMainScreen(const VRState &state);
void DeviceIcon(ImDrawList *dl, const VRDevice &dev, ImVec2 c, float s, ImU32 col);
const std::string *FindDeviceName(const std::string &serial);
std::string DeviceDisplayName(const VRDevice &dev);
void CommitDeviceName(const VRDevice &dev, const char *text);
bool DeviceRow(const VRDevice &dev, bool selected, float w, bool first, bool last);
void EnsureDeviceSelection(const VRState &state, uint32_t &selected, const std::string &system);
void BuildDeviceList(const VRState &state, uint32_t &selected, const std::string &system, float paneW);
std::string FriendlySystemName(const std::string &raw);
void PickTrackingSystem(const char *id, const std::vector<std::string> &candidates, int fallback, float width, std::string &selection);
void BuildSpacesSection(const VRState &state);
void IdentifyButton(ImVec2 size);
void OpenGuide(bool anchor, bool mountRun);
void StartMountSetup(const VRState &state);
bool BeginGuidedRun();
void DrawGuideAnimation(ImDrawList *dl, ImVec2 origin, ImVec2 size, double t, bool mountRun);
void DrawGuideHint(ImDrawList *dl, ImVec2 origin, ImVec2 size, CalibrationContext::GuideHint hint);
std::string GuideStepLabel(bool anchor, bool mountRun, int step);
const char *GuideHintCaption(CalibrationContext::GuideHint hint);
void DrawGuideIndicators(ImDrawList *dl, ImVec2 origin, float width, const questcal::GuideMetrics &m, bool mountRun);
void BuildMenu(const VRState &state, bool runningInOverlay);
void BuildSettingsScreen(const VRState &state);
void SeedTransformEditorDraft();
bool BuildProfileEditor();
bool SaveProfileEditorDraft();
std::string PreviewIconPath(const char *driverRelative);
void UpdateIdentifyPulse(double now);
void BuildHeader();
void BuildFooter(bool runningInOverlay);
VRState LoadVRState();
VRState PreviewVRState();
