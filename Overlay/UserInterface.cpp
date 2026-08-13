#include "stdafx.h"
#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "ProfileValidation.h"
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

bool g_uiPreviewMode = false;
bool g_uiPreviewMany = false;
ImFont *g_fontBody = nullptr;
ImFont *g_fontSmall = nullptr;
ImFont *g_fontTitle = nullptr;

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

VRState LoadVRState();
static void BuildSpacesSection(const VRState &state);
static bool BuildProfileEditor();
static bool SaveProfileEditorDraft();
static void SeedTransformEditorDraft();
static void BuildMenu(const VRState &state, bool runningInOverlay);

template<typename T>
static void SaveSettingOrRestore(T &value, const T &previous)
{
	if (!SaveSettings(CalCtx))
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

static IdentifyPulseState g_identifyPulse;

static void UpdateIdentifyPulse(double now)
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
// Palette + theme
// ---------------------------------------------------------------------------

namespace Pal
{
	const ImVec4 Bg       (0.039f, 0.043f, 0.051f, 1.00f);
	const ImVec4 Card     (0.075f, 0.080f, 0.093f, 1.00f);
	const ImVec4 CardHov  (0.100f, 0.107f, 0.124f, 1.00f);
	const ImVec4 Inset    (0.055f, 0.059f, 0.069f, 1.00f);
	const ImVec4 Border   (0.150f, 0.158f, 0.180f, 1.00f);
	const ImVec4 BorderHov(0.230f, 0.242f, 0.272f, 1.00f);
	const ImVec4 Text     (0.925f, 0.933f, 0.950f, 1.00f);
	const ImVec4 Dim      (0.520f, 0.550f, 0.610f, 1.00f);
	const ImVec4 Faint    (0.360f, 0.383f, 0.440f, 1.00f);
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
static void LinkText(const char *label, const char *url)
{
	ImGui::TextColored(Pal::Dim, label);
	if (ImGui::IsItemHovered())
	{
		ImVec2 mn = ImGui::GetItemRectMin(), mx = ImGui::GetItemRectMax();
		ImGui::GetWindowDrawList()->AddLine(
			ImVec2(mn.x, mx.y - 1.0f), ImVec2(mx.x, mx.y - 1.0f), Pal::U32(Pal::Dim));
		if (ImGui::IsItemClicked())
			ShellExecuteA(nullptr, "open", url, nullptr, nullptr, SW_SHOWNORMAL);
	}
}

void ApplyTheme()
{
	ImGuiStyle &s = ImGui::GetStyle();

	s.WindowPadding     = ImVec2(28, 24);
	s.FramePadding      = ImVec2(14, 10);
	s.ItemSpacing       = ImVec2(12, 8);
	s.ItemInnerSpacing  = ImVec2(8, 6);
	s.WindowRounding    = 0.0f;
	s.FrameRounding     = 9.0f;
	s.PopupRounding     = 14.0f;
	s.ChildRounding     = 12.0f;
	s.GrabRounding      = 9.0f;
	s.ScrollbarRounding = 8.0f;
	s.ScrollbarSize     = 10.0f;
	s.WindowBorderSize  = 0.0f;
	s.ChildBorderSize   = 0.0f;
	s.PopupBorderSize   = 1.0f;
	s.FrameBorderSize   = 1.0f;

	ImVec4 *c = s.Colors;
	c[ImGuiCol_Text]                 = Pal::Text;
	c[ImGuiCol_TextDisabled]         = Pal::Faint;
	c[ImGuiCol_WindowBg]             = Pal::Bg;
	c[ImGuiCol_ChildBg]              = ImVec4(0, 0, 0, 0);
	c[ImGuiCol_PopupBg]              = ImVec4(0.060f, 0.064f, 0.075f, 1.0f);
	c[ImGuiCol_Border]               = Pal::Border;
	c[ImGuiCol_BorderShadow]         = ImVec4(0, 0, 0, 0);
	c[ImGuiCol_FrameBg]              = Pal::Card;
	c[ImGuiCol_FrameBgHovered]       = Pal::CardHov;
	c[ImGuiCol_FrameBgActive]        = Pal::CardHov;
	c[ImGuiCol_ScrollbarBg]          = ImVec4(0, 0, 0, 0);
	c[ImGuiCol_ScrollbarGrab]        = ImVec4(1, 1, 1, 0.08f);
	c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(1, 1, 1, 0.15f);
	c[ImGuiCol_ScrollbarGrabActive]  = ImVec4(1, 1, 1, 0.22f);
	c[ImGuiCol_CheckMark]            = Pal::Accent;
	c[ImGuiCol_SliderGrab]           = Pal::Accent;
	c[ImGuiCol_SliderGrabActive]     = Pal::AccentHov;
	c[ImGuiCol_Button]               = Pal::Card;
	c[ImGuiCol_ButtonHovered]        = Pal::CardHov;
	c[ImGuiCol_ButtonActive]         = Pal::Inset;
	c[ImGuiCol_Header]               = ImVec4(Pal::Accent.x, Pal::Accent.y, Pal::Accent.z, 0.35f);
	c[ImGuiCol_HeaderHovered]        = ImVec4(Pal::Accent.x, Pal::Accent.y, Pal::Accent.z, 0.25f);
	c[ImGuiCol_HeaderActive]         = ImVec4(Pal::Accent.x, Pal::Accent.y, Pal::Accent.z, 0.45f);
	c[ImGuiCol_Separator]            = Pal::Border;
	c[ImGuiCol_PlotHistogram]        = Pal::Accent;
	c[ImGuiCol_PlotHistogramHovered] = Pal::AccentHov;
	c[ImGuiCol_TextSelectedBg]       = ImVec4(Pal::Accent.x, Pal::Accent.y, Pal::Accent.z, 0.35f);
	c[ImGuiCol_ModalWindowDarkening] = ImVec4(0, 0, 0, 0.65f);
	c[ImGuiCol_NavHighlight]         = Pal::Accent;
}

// ---------------------------------------------------------------------------
// Small drawing helpers
// ---------------------------------------------------------------------------

typedef void (*IconFn)(ImDrawList *, ImVec2, float, ImU32);

static float LetterSpacedWidth(ImFont *font, const char *text, float spacing)
{
	float w = 0.0f;
	for (const char *p = text; *p; ++p)
	{
		const ImFontGlyph *g = font->FindGlyph((ImWchar)*p);
		w += (g ? g->AdvanceX : font->FontSize * 0.5f) + spacing;
	}
	return w > 0.0f ? w - spacing : 0.0f;
}

static void LetterSpacedTextAt(ImDrawList *dl, ImFont *font, ImVec2 pos, ImU32 col, const char *text, float spacing)
{
	float x = pos.x;
	char buf[2] = { 0, 0 };
	for (const char *p = text; *p; ++p)
	{
		buf[0] = *p;
		dl->AddText(font, font->FontSize, ImVec2(x, pos.y), col, buf);
		const ImFontGlyph *g = font->FindGlyph((ImWchar)*p);
		x += (g ? g->AdvanceX : font->FontSize * 0.5f) + spacing;
	}
}

// Section label ("REFERENCE SPACE") as an inline widget.
static void SectionLabel(const char *text)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImVec2 p = ImGui::GetCursorScreenPos();
	LetterSpacedTextAt(dl, g_fontSmall, p, Pal::U32(Pal::Dim), text, 2.0f);
	ImGui::Dummy(ImVec2(LetterSpacedWidth(g_fontSmall, text, 2.0f), g_fontSmall->FontSize + 4.0f));
}

// ---------------------------------------------------------------------------
// Icons (line style, s = half-extent in pixels)
// ---------------------------------------------------------------------------

static void IconLogo(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.78f, col, 24, 2.2f);
	dl->AddCircleFilled(c, s * 0.26f, col, 12);
}

static void IconHMD(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	ImVec2 a = ImVec2(c.x - s, c.y - s * 0.60f);
	ImVec2 b = ImVec2(c.x + s, c.y + s * 0.44f);
	dl->PathRect(a, b, s * 0.34f);
	dl->PathStroke(col, true, 2.2f);
	dl->AddCircleFilled(ImVec2(c.x - s * 0.42f, c.y - s * 0.06f), s * 0.16f, col, 12);
	dl->AddCircleFilled(ImVec2(c.x + s * 0.42f, c.y - s * 0.06f), s * 0.16f, col, 12);
}

static void IconController(ImDrawList *dl, ImVec2 c, float s, ImU32 col, bool leftHand)
{
	float m = leftHand ? -1.0f : 1.0f;
	// Handle first, then the tracking ring drawn over it (same color, so the
	// overlap disappears and the silhouette reads as one piece).
	ImVec2 h0 = ImVec2(c.x + m * s * 0.16f, c.y - s * 0.10f);
	ImVec2 h1 = ImVec2(c.x + m * s * 0.34f, c.y + s * 0.78f);
	float th = s * 0.38f;
	dl->AddLine(h0, h1, col, th);
	dl->AddCircleFilled(h0, th * 0.5f, col, 12);
	dl->AddCircleFilled(h1, th * 0.5f, col, 12);
	ImVec2 ringC = ImVec2(c.x - m * s * 0.10f, c.y - s * 0.32f);
	dl->AddCircle(ringC, s * 0.52f, col, 24, 2.4f);
}

static void IconTracker(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.68f, col, 24, 2.2f);
	dl->AddCircleFilled(c, s * 0.15f, col, 10);
	for (int i = 0; i < 3; ++i)
	{
		float a = -IM_PI * 0.5f + (float)i * (2.0f * IM_PI / 3.0f);
		dl->AddCircleFilled(ImVec2(c.x + cosf(a) * s * 0.40f, c.y + sinf(a) * s * 0.40f), s * 0.09f, col, 8);
	}
}

static void IconPlay(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddTriangleFilled(
		ImVec2(c.x - s * 0.42f, c.y - s * 0.62f),
		ImVec2(c.x - s * 0.42f, c.y + s * 0.62f),
		ImVec2(c.x + s * 0.66f, c.y), col);
}

static void IconPencil(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	ImVec2 tip = ImVec2(c.x - s * 0.58f, c.y + s * 0.58f);
	ImVec2 top = ImVec2(c.x + s * 0.50f, c.y - s * 0.50f);
	dl->AddLine(ImVec2(tip.x + s * 0.22f, tip.y - s * 0.06f), top, col, 2.4f);
	dl->AddTriangleFilled(tip,
		ImVec2(tip.x + s * 0.30f, tip.y - s * 0.02f),
		ImVec2(tip.x + s * 0.02f, tip.y - s * 0.30f), col);
	dl->AddLine(ImVec2(top.x - s * 0.28f, top.y - s * 0.02f), ImVec2(top.x - s * 0.02f, top.y + s * 0.28f), col, 2.0f);
}

static void IconTrash(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddLine(ImVec2(c.x - s * 0.62f, c.y - s * 0.36f), ImVec2(c.x + s * 0.62f, c.y - s * 0.36f), col, 2.2f);
	dl->AddLine(ImVec2(c.x - s * 0.20f, c.y - s * 0.60f), ImVec2(c.x + s * 0.20f, c.y - s * 0.60f), col, 2.2f);
	dl->PathRect(ImVec2(c.x - s * 0.45f, c.y - s * 0.36f), ImVec2(c.x + s * 0.45f, c.y + s * 0.62f), s * 0.16f, ImDrawCornerFlags_Bot);
	dl->PathStroke(col, true, 2.0f);
	dl->AddLine(ImVec2(c.x - s * 0.15f, c.y - s * 0.12f), ImVec2(c.x - s * 0.15f, c.y + s * 0.36f), col, 1.8f);
	dl->AddLine(ImVec2(c.x + s * 0.15f, c.y - s * 0.12f), ImVec2(c.x + s * 0.15f, c.y + s * 0.36f), col, 1.8f);
}

static void IconCopy(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->PathRect(ImVec2(c.x - s * 0.62f, c.y - s * 0.62f), ImVec2(c.x + s * 0.14f, c.y + s * 0.14f), s * 0.14f);
	dl->PathStroke(col, true, 2.0f);
	dl->PathRect(ImVec2(c.x - s * 0.14f, c.y - s * 0.14f), ImVec2(c.x + s * 0.62f, c.y + s * 0.62f), s * 0.14f);
	dl->PathStroke(col, true, 2.0f);
}

static void IconCrosshair(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.55f, col, 24, 2.0f);
	dl->AddCircleFilled(c, s * 0.12f, col, 8);
	dl->AddLine(ImVec2(c.x, c.y - s * 0.90f), ImVec2(c.x, c.y - s * 0.55f), col, 2.0f);
	dl->AddLine(ImVec2(c.x, c.y + s * 0.55f), ImVec2(c.x, c.y + s * 0.90f), col, 2.0f);
	dl->AddLine(ImVec2(c.x - s * 0.90f, c.y), ImVec2(c.x - s * 0.55f, c.y), col, 2.0f);
	dl->AddLine(ImVec2(c.x + s * 0.55f, c.y), ImVec2(c.x + s * 0.90f, c.y), col, 2.0f);
}

static void IconCheck(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	ImVec2 pts[3] = {
		ImVec2(c.x - s * 0.65f, c.y + s * 0.05f),
		ImVec2(c.x - s * 0.15f, c.y + s * 0.55f),
		ImVec2(c.x + s * 0.70f, c.y - s * 0.50f)
	};
	dl->AddPolyline(pts, 3, col, false, 2.4f);
}

static void IconClock(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.85f, col, 20, 2.0f);
	dl->AddLine(c, ImVec2(c.x, c.y - s * 0.52f), col, 2.0f);
	dl->AddLine(c, ImVec2(c.x + s * 0.40f, c.y + s * 0.14f), col, 2.0f);
}

static void IconInfo(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.85f, col, 20, 2.0f);
	dl->AddCircleFilled(ImVec2(c.x, c.y - s * 0.38f), s * 0.11f, col, 8);
	dl->AddLine(ImVec2(c.x, c.y - s * 0.08f), ImVec2(c.x, c.y + s * 0.42f), col, 2.2f);
}

static void IconPin(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	ImVec2 hc = ImVec2(c.x, c.y - s * 0.22f);
	dl->AddCircle(hc, s * 0.42f, col, 20, 2.0f);
	dl->AddCircleFilled(hc, s * 0.11f, col, 8);
	dl->AddLine(ImVec2(hc.x - s * 0.29f, hc.y + s * 0.30f), ImVec2(c.x, c.y + s * 0.72f), col, 2.0f);
	dl->AddLine(ImVec2(hc.x + s * 0.29f, hc.y + s * 0.30f), ImVec2(c.x, c.y + s * 0.72f), col, 2.0f);
}

static void IconScale(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddLine(ImVec2(c.x, c.y - s * 0.62f), ImVec2(c.x, c.y + s * 0.46f), col, 2.0f);
	dl->AddLine(ImVec2(c.x - s * 0.58f, c.y - s * 0.40f), ImVec2(c.x + s * 0.58f, c.y - s * 0.40f), col, 2.0f);
	dl->AddLine(ImVec2(c.x - s * 0.30f, c.y + s * 0.46f), ImVec2(c.x + s * 0.30f, c.y + s * 0.46f), col, 2.0f);
	dl->PathArcTo(ImVec2(c.x - s * 0.58f, c.y - s * 0.16f), s * 0.24f, 0.0f, IM_PI, 12);
	dl->PathStroke(col, false, 2.0f);
	dl->PathArcTo(ImVec2(c.x + s * 0.58f, c.y - s * 0.16f), s * 0.24f, 0.0f, IM_PI, 12);
	dl->PathStroke(col, false, 2.0f);
}

static void IconGauge(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->PathArcTo(ImVec2(c.x, c.y + s * 0.25f), s * 0.72f, IM_PI, 2.0f * IM_PI, 20);
	dl->PathStroke(col, false, 2.2f);
	dl->AddLine(ImVec2(c.x, c.y + s * 0.25f), ImVec2(c.x + s * 0.38f, c.y - s * 0.22f), col, 2.2f);
	dl->AddCircleFilled(ImVec2(c.x, c.y + s * 0.25f), s * 0.12f, col, 8);
}

static void IconField(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	// Anchor spot: pin over ground ripple.
	dl->AddCircle(ImVec2(c.x, c.y - s * 0.30f), s * 0.34f, col, 16, 2.0f);
	dl->AddLine(ImVec2(c.x, c.y + s * 0.04f), ImVec2(c.x, c.y + s * 0.44f), col, 2.0f);
	dl->PathArcTo(ImVec2(c.x, c.y + s * 0.44f), s * 0.55f, IM_PI * 0.15f, IM_PI * 0.85f, 12);
	dl->PathStroke(col, false, 2.0f);
}

static void IconGear(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.52f, col, 20, 2.2f);
	dl->AddCircleFilled(c, s * 0.16f, col, 10);
	for (int i = 0; i < 8; ++i)
	{
		float a = (float)i * (IM_PI / 4.0f) + IM_PI / 8.0f;
		ImVec2 dir = ImVec2(cosf(a), sinf(a));
		dl->AddLine(ImVec2(c.x + dir.x * s * 0.55f, c.y + dir.y * s * 0.55f),
			ImVec2(c.x + dir.x * s * 0.85f, c.y + dir.y * s * 0.85f), col, s * 0.28f);
	}
}

// ---------------------------------------------------------------------------
// SteamVR device icon textures
// ---------------------------------------------------------------------------

// Owns one COM interface for the rest of its scope. Each acquisition below is
// then one line plus one guard: a new step cannot land its Release in the
// wrong place, because there is no unwinding ladder to place it in.
template<typename T>
struct ComScoped
{
	ComScoped() = default;
	ComScoped(const ComScoped &) = delete;
	ComScoped &operator=(const ComScoped &) = delete;
	~ComScoped() { if (ptr) ptr->Release(); }

	T **Put() { return &ptr; }
	T *Get() const { return ptr; }
	T *operator->() const { return ptr; }

	T *ptr = nullptr;
};

static bool LoadTextureFromFile(const char *path, GLuint *outTex, int *outW, int *outH)
{
	// One apartment init per process. The original hand-rolled flag ignored the
	// result and so does this: a failure surfaces as the CoCreateInstance below.
	static const HRESULT comInit = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
	(void)comInit;

	ComScoped<IWICImagingFactory> factory;
	if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
		IID_PPV_ARGS(factory.Put()))))
		return false;

	wchar_t wpath[MAX_PATH];
	MultiByteToWideChar(CP_UTF8, 0, path, -1, wpath, MAX_PATH);

	ComScoped<IWICBitmapDecoder> dec;
	if (FAILED(factory->CreateDecoderFromFilename(wpath, nullptr, GENERIC_READ,
		WICDecodeMetadataCacheOnDemand, dec.Put())))
		return false;

	ComScoped<IWICBitmapFrameDecode> frame;
	if (FAILED(dec->GetFrame(0, frame.Put())))
		return false;

	ComScoped<IWICFormatConverter> conv;
	if (FAILED(factory->CreateFormatConverter(conv.Put())))
		return false;
	if (FAILED(conv->Initialize(frame.Get(), GUID_WICPixelFormat32bppRGBA,
		WICBitmapDitherTypeNone, nullptr, 0.0, WICBitmapPaletteTypeCustom)))
		return false;

	// Device icons are small art; the bound keeps a malformed or hostile file
	// from allocating unbounded pixel memory on the render thread.
	UINT w = 0, h = 0;
	conv->GetSize(&w, &h);
	if (w == 0 || h == 0 || w > 1024 || h > 1024)
		return false;

	std::vector<unsigned char> pixels((size_t)w * h * 4);
	if (FAILED(conv->CopyPixels(nullptr, w * 4, (UINT)pixels.size(), pixels.data())))
		return false;

	GLuint tex = 0;
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	*outTex = tex;
	*outW = (int)w;
	*outH = (int)h;
	return true;
}

struct DeviceIconTex
{
	GLuint tex = 0;
	int w = 0, h = 0;
	bool failed = false;
};

// Keyed by absolute path; loaded lazily on the render thread (GL context current).
static std::map<std::string, DeviceIconTex> s_deviceIconCache;

static const DeviceIconTex *GetDeviceIconTex(const std::string &path)
{
	if (path.empty())
		return nullptr;

	auto it = s_deviceIconCache.find(path);
	if (it == s_deviceIconCache.end())
	{
		DeviceIconTex t;
		if (!LoadTextureFromFile(path.c_str(), &t.tex, &t.w, &t.h))
			t.failed = true;
		it = s_deviceIconCache.emplace(path, t).first;
	}
	return it->second.failed ? nullptr : &it->second;
}

static bool FileExists(const std::string &path)
{
	DWORD attrs = GetFileAttributesA(path.c_str());
	return attrs != INVALID_FILE_ATTRIBUTES && !(attrs & FILE_ATTRIBUTE_DIRECTORY);
}

// SteamVR ships icons in 1x and 2x; prefer the sharper one when present.
static std::string Prefer2x(const std::string &path)
{
	size_t dot = path.find_last_of('.');
	if (dot != std::string::npos)
	{
		std::string hi = path.substr(0, dot) + "_2x" + path.substr(dot);
		if (FileExists(hi))
			return hi;
	}
	return path;
}

// ---------------------------------------------------------------------------
// Widgets
// ---------------------------------------------------------------------------

enum class BtnKind { Primary, Ghost, Quiet };

static bool IconButton(const char *id, const char *label, IconFn icon, ImVec2 size, BtnKind kind, bool smallCaps = false)
{
	ImVec2 p = ImGui::GetCursorScreenPos();
	bool pressed = ImGui::InvisibleButton(id, size);
	bool hov = ImGui::IsItemHovered();
	bool act = ImGui::IsItemActive();
	ImDrawList *dl = ImGui::GetWindowDrawList();

	ImVec4 bg, txt;
	if (kind == BtnKind::Primary)
	{
		bg = act ? Pal::AccentAct : (hov ? Pal::AccentHov : Pal::Accent);
		txt = Pal::White;
	}
	else if (kind == BtnKind::Quiet)
	{
		// No chrome at rest; a faint wash on hover keeps it discoverable.
		bg = ImVec4(1, 1, 1, act ? 0.06f : (hov ? 0.04f : 0.0f));
		txt = hov ? Pal::Text : Pal::Dim;
	}
	else
	{
		bg = act ? Pal::Inset : (hov ? Pal::CardHov : Pal::Card);
		txt = hov ? Pal::Text : (smallCaps ? Pal::Dim : Pal::Text);
	}

	if (bg.w > 0.0f)
		dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), Pal::U32(bg), 10.0f);
	if (kind == BtnKind::Ghost)
		dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), Pal::U32(hov ? Pal::BorderHov : Pal::Border), 10.0f);

	ImFont *font = smallCaps ? g_fontSmall : g_fontBody;
	float spacing = smallCaps ? 2.0f : 0.0f;
	float textW = smallCaps ? LetterSpacedWidth(font, label, spacing) : ImGui::CalcTextSize(label).x;
	float iconS = 9.0f;
	float iconBlock = icon ? iconS * 2.0f + 10.0f : 0.0f;
	float cx = p.x + (size.x - textW - iconBlock) * 0.5f;
	float cy = p.y + size.y * 0.5f;
	ImU32 tcol = Pal::U32(txt);

	if (icon)
	{
		icon(dl, ImVec2(cx + iconS, cy), iconS, tcol);
		cx += iconBlock;
	}
	if (smallCaps)
		LetterSpacedTextAt(dl, font, ImVec2(cx, cy - font->FontSize * 0.5f), tcol, label, spacing);
	else
		dl->AddText(font, font->FontSize, ImVec2(cx, cy - font->FontSize * 0.5f), tcol, label);

	return pressed;
}

static bool QCCheckbox(const char *id, bool *v)
{
	const float sz = 24.0f;
	ImVec2 p = ImGui::GetCursorScreenPos();
	bool pressed = ImGui::InvisibleButton(id, ImVec2(sz, sz));
	if (pressed)
		*v = !*v;
	bool hov = ImGui::IsItemHovered();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImVec2 b = ImVec2(p.x + sz, p.y + sz);

	if (*v)
	{
		dl->AddRectFilled(p, b, Pal::U32(hov ? Pal::AccentHov : Pal::Accent), 6.0f);
		ImVec2 pts[3] = {
			ImVec2(p.x + sz * 0.24f, p.y + sz * 0.52f),
			ImVec2(p.x + sz * 0.43f, p.y + sz * 0.72f),
			ImVec2(p.x + sz * 0.78f, p.y + sz * 0.30f)
		};
		dl->AddPolyline(pts, 3, Pal::U32(Pal::White), false, 2.6f);
	}
	else
	{
		dl->AddRectFilled(p, b, Pal::U32(Pal::Inset), 6.0f);
		dl->AddRect(p, b, Pal::U32(hov ? Pal::BorderHov : Pal::Border), 6.0f, ImDrawCornerFlags_All, 1.2f);
	}
	return pressed;
}

// Row card container for the settings list.
static ImVec2 BeginRowCard(float height)
{
	ImVec2 p = ImGui::GetCursorScreenPos();
	float w = ImGui::GetWindowContentRegionWidth();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	dl->AddRectFilled(p, ImVec2(p.x + w, p.y + height), Pal::U32(Pal::Card), 12.0f);
	dl->AddRect(p, ImVec2(p.x + w, p.y + height), Pal::U32(Pal::Border), 12.0f);
	return p;
}

static void EndRowCard(ImVec2 p, float height)
{
	ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + height));
	ImGui::Dummy(ImVec2(0, 0));
}

static void RowIconLabel(ImVec2 rowPos, IconFn icon, const char *label)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImVec2 iconC = ImVec2(rowPos.x + 66.0f, rowPos.y + 26.0f);
	icon(dl, iconC, 9.0f, Pal::U32(Pal::Dim));
	dl->AddText(g_fontBody, g_fontBody->FontSize,
		ImVec2(rowPos.x + 92.0f, rowPos.y + 26.0f - g_fontBody->FontSize * 0.5f),
		Pal::U32(Pal::Text), label);
}

// Settings-row geometry, stated once: every row is this tall and puts its
// control at this inset. Taller composite rows add their extra body to the
// same base instead of restating it.
static const float kRowHeight = 52.0f;
static const float kRowInsetX = 16.0f;
static const float kRowControlY = 14.0f;

// Remembers the height it opened with, so the end call cannot disagree with
// the begin call and silently overlap the next row.
struct RowCard
{
	explicit RowCard(float rowHeight) : pos(BeginRowCard(rowHeight)), height(rowHeight) {}
	RowCard(const RowCard &) = delete;
	RowCard &operator=(const RowCard &) = delete;
	~RowCard() { EndRowCard(pos, height); }

	ImVec2 pos;
	float height;
};

// A plain settings toggle, whole: card, checkbox at the shared inset, optional
// tooltip on it, icon + label. Returns whether the value changed this frame --
// persisting it stays with the caller, since the settings and profile
// save-or-restore paths are different templates.
static bool ToggleRow(const char *id, IconFn icon, const char *label, bool &value,
	const char *tooltip = nullptr)
{
	RowCard row(kRowHeight);
	ImGui::SetCursorScreenPos(ImVec2(row.pos.x + kRowInsetX, row.pos.y + kRowControlY));
	bool changed = QCCheckbox(id, &value);
	if (tooltip && ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", tooltip);
	RowIconLabel(row.pos, icon, label);
	return changed;
}

static int Segmented(const char *id, int value, const char *const items[], int count, float itemW, float h)
{
	ImGui::PushID(id);
	ImVec2 p = ImGui::GetCursorScreenPos();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	float pad = 4.0f;
	ImVec2 b = ImVec2(p.x + itemW * count + pad * 2.0f, p.y + h);
	dl->AddRectFilled(p, b, Pal::U32(Pal::Inset), 7.0f);
	dl->AddRect(p, b, Pal::U32(Pal::Border), 7.0f);

	for (int i = 0; i < count; ++i)
	{
		ImGui::PushID(i);
		ImVec2 ip = ImVec2(p.x + pad + i * itemW, p.y + pad);
		ImVec2 isz = ImVec2(itemW, h - pad * 2.0f);
		ImGui::SetCursorScreenPos(ip);
		if (ImGui::InvisibleButton("seg", isz))
			value = i;
		bool hov = ImGui::IsItemHovered();

		if (i == value)
			dl->AddRectFilled(ip, ImVec2(ip.x + isz.x, ip.y + isz.y), Pal::U32(ImVec4(1, 1, 1, 0.10f)), 5.0f);
		else if (hov)
			dl->AddRectFilled(ip, ImVec2(ip.x + isz.x, ip.y + isz.y), Pal::U32(ImVec4(1, 1, 1, 0.04f)), 5.0f);

		ImVec2 ts = ImGui::CalcTextSize(items[i]);
		dl->AddText(g_fontBody, g_fontBody->FontSize,
			ImVec2(ip.x + (isz.x - ts.x) * 0.5f, ip.y + (isz.y - g_fontBody->FontSize) * 0.5f),
			Pal::U32(i == value ? Pal::Text : Pal::Dim), items[i]);
		ImGui::PopID();
	}

	ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + h));
	ImGui::Dummy(ImVec2(0, 0));
	ImGui::PopID();
	return value;
}

// ---------------------------------------------------------------------------
// Status rows
// ---------------------------------------------------------------------------

struct StatusRowData
{
	IconFn icon;
	ImVec4 color;
	std::string text;
};

static void DrawStatusCard(const std::vector<StatusRowData> &rows)
{
	if (rows.empty())
		return;

	const float rowH = 34.0f, padY = 12.0f, padX = 16.0f;
	float h = padY * 2.0f + rowH * (float)rows.size();
	ImVec2 p = BeginRowCard(h);
	ImDrawList *dl = ImGui::GetWindowDrawList();

	for (size_t i = 0; i < rows.size(); ++i)
	{
		const StatusRowData &r = rows[i];
		float cy = p.y + padY + rowH * (float)i + rowH * 0.5f;
		ImVec2 c = ImVec2(p.x + padX + 13.0f, cy);
		ImVec4 bg = r.color; bg.w = 0.15f;
		dl->AddCircleFilled(c, 13.0f, Pal::U32(bg), 20);
		r.icon(dl, c, 8.5f, Pal::U32(r.color));
		dl->AddText(g_fontBody, g_fontBody->FontSize,
			ImVec2(c.x + 23.0f, cy - g_fontBody->FontSize * 0.5f),
			Pal::U32(r.color), r.text.c_str());
	}

	EndRowCard(p, h);
}

static std::string FormatString(const char *fmt, ...)
{
	char buf[512];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof buf, fmt, args);
	va_end(args);
	return std::string(buf);
}

// ---------------------------------------------------------------------------
// Plain-language calibration rating (simple mode)
// ---------------------------------------------------------------------------

static bool s_showSettings = false;

// Unknown is not a severity, it is the absence of one: nothing has been
// measured to rate. Sorting it below Good keeps every "bad enough to say
// something" test (>= Rating_Poor) reading false for it with no special case.
enum CalRating { Rating_Unknown = -1, Rating_Good = 0, Rating_Decent, Rating_Poor, Rating_VeryPoor };

// The driver's pose channel feeds every runtime monitor. Preview mode has no
// driver at all, so it must not display a fault for it.
static bool PoseChannelDown()
{
	return !g_uiPreviewMode && !CalCtx.poseRingOpen;
}

// ---------------------------------------------------------------------------
// Continuous-calibration status
// ---------------------------------------------------------------------------

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
static ContinuousStatus ContinuousStatusNow()
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

static const char *ContinuousStatusText(ContinuousStatus status)
{
	switch (status)
	{
	case ContinuousStatus::Off:        return "off";
	case ContinuousStatus::NoTracker:  return "no tracker selected";
	case ContinuousStatus::NeedsMount: return "needs one calibration with the tracker mounted";
	case ContinuousStatus::NotRunning: return "not running -- mounted tracker or driver unavailable";
	case ContinuousStatus::Tracking:   return "maintaining";
	case ContinuousStatus::Coasting:   return "paused -- tracker not tracking";
	case ContinuousStatus::Frozen:     return "on hold -- check the mount";
	case ContinuousStatus::Holding:    return "paused -- tracker tracking unstable";
	default:                           return "gathering";
	}
}

// ---------------------------------------------------------------------------
// Rating
// ---------------------------------------------------------------------------

static CalRating ComputeCalibrationRating(ContinuousStatus continuous)
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

	// Without the pose channel the monitors that would demote this rating
	// cannot run at all: no jump compensation, no drift evidence, and the solve
	// itself fell back to tick-rate runtime poses. Never claim Good on that.
	if (PoseChannelDown() && r < Rating_Decent)
		r = Rating_Decent;

	// Staleness/drift only degrade the rating when nothing is maintaining the
	// alignment; a healthy continuous loop re-measures it constantly. A frozen
	// loop (bumped mount) is itself a Poor signal.
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
	if (continuous == ContinuousStatus::Frozen && r < Rating_Poor)
		r = Rating_Poor;

	// Solve residuals live only in lastResult, which FinishCalibration sets and
	// nothing persists, so a profile restored from the registry has no evidence
	// behind the quality half of this verdict. If none of the monitors above
	// found a reason to demote it, what we have is an absence of evidence, not
	// a good measurement -- say so instead of asserting the best label.
	if (!CalCtx.lastResult.valid && r == Rating_Good)
		return Rating_Unknown;

	return (CalRating)r;
}

static const char *RatingLabels[] = { "Good", "Decent", "Poor", "Very Poor" };

static const char *RatingLabel(CalRating r)
{
	return r == Rating_Unknown ? "Unknown" : RatingLabels[r];
}

static ImVec4 RatingColor(CalRating r)
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
static const char *RecalibrationNudge(CalRating rating, ContinuousStatus continuous)
{
	if (rating < Rating_Poor)
		return nullptr;
	return continuous == ContinuousStatus::Frozen
		? "Check the headset-mounted tracker -- recalibrating is recommended."
		: "Recalibrating is recommended.";
}

// Empty when the timestamp is unusable. "Unknown" is a property of the data,
// not a rendered phrase: returning it as an empty optional lets every caller
// word its own fallback, instead of the wording being baked into a buffer and
// recovered downstream by comparing against the literal.
static std::optional<std::string> FormatUnixAge(double unixTime)
{
	double now = static_cast<double>(std::time(nullptr));
	if (!std::isfinite(unixTime) || unixTime <= 0.0 || now <= 0.0)
		return std::nullopt;

	double seconds = now - unixTime;
	// Small clock corrections must not read as a future timestamp.
	if (seconds < -300.0)
		return std::string("time is in the future");

	double hours = std::max(0.0, seconds) / 3600.0;
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
static std::optional<std::string> FormatAlignmentAge()
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
static bool ProtectChaperone()
{
	if (g_uiPreviewMode)
		return true;
	return LoadChaperoneBounds();
}

// When the one-time drift warning modal was opened; gates its accept button.
static double g_chapWarnOpenedAt = 0.0;

// ---------------------------------------------------------------------------
// Header
// ---------------------------------------------------------------------------

static void BuildHeader()
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

// ---------------------------------------------------------------------------
// Device cards
// ---------------------------------------------------------------------------

static void DeviceIcon(ImDrawList *dl, const VRDevice &dev, ImVec2 c, float s, ImU32 col)
{
	switch (dev.deviceClass)
	{
	case vr::TrackedDeviceClass_HMD:
		IconHMD(dl, c, s, col);
		break;
	case vr::TrackedDeviceClass_Controller:
		IconController(dl, c, s, col, dev.controllerRole == vr::TrackedControllerRole_LeftHand);
		break;
	default:
		IconTracker(dl, c, s, col);
		break;
	}
}

// A flat row inside the list container: no chrome of its own — an inset
// divider above (except the first), a hover wash, and selection as an accent
// rail + tint. Rounding only ever belongs to the container's outer corners.
static bool DeviceRow(const VRDevice &dev, bool selected, float w, bool first, bool last)
{
	const float h = 52.0f;
	ImGui::PushID(dev.id);
	ImVec2 p = ImGui::GetCursorScreenPos();
	bool pressed = ImGui::InvisibleButton("row", ImVec2(w, h));
	bool hov = ImGui::IsItemHovered();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImVec2 b = ImVec2(p.x + w, p.y + h);

	if (!first)
		dl->AddLine(ImVec2(p.x + 16.0f, p.y), ImVec2(b.x - 16.0f, p.y), Pal::U32(Pal::Border), 1.0f);

	int corners = (first ? ImDrawCornerFlags_Top : 0) | (last ? ImDrawCornerFlags_Bot : 0);
	if (selected)
	{
		ImVec4 tint = Pal::Accent; tint.w = 0.08f;
		dl->AddRectFilled(p, b, Pal::U32(tint), 11.0f, corners);
		dl->AddRectFilled(ImVec2(p.x + 1.0f, p.y + 10.0f), ImVec2(p.x + 4.5f, b.y - 10.0f),
			Pal::U32(Pal::Accent), 2.0f);
	}
	else if (hov)
	{
		dl->AddRectFilled(p, b, Pal::U32(ImVec4(1, 1, 1, 0.03f)), 11.0f, corners);
	}

	// Device icon: real SteamVR icon when resolvable, vector fallback otherwise.
	ImVec2 iconC = ImVec2(p.x + 30.0f, p.y + h * 0.5f);
	const DeviceIconTex *tex = GetDeviceIconTex(dev.iconPath);
	if (tex)
	{
		const float boxW = 40.0f, boxH = 34.0f;
		float scale = boxW / (float)tex->w;
		if (scale * (float)tex->h > boxH)
			scale = boxH / (float)tex->h;
		ImVec2 half = ImVec2(tex->w * scale * 0.5f, tex->h * scale * 0.5f);
		dl->AddImage((ImTextureID)(intptr_t)tex->tex,
			ImVec2(iconC.x - half.x, iconC.y - half.y),
			ImVec2(iconC.x + half.x, iconC.y + half.y));
	}
	else
	{
		DeviceIcon(dl, dev, iconC, 13.0f, Pal::U32(selected ? Pal::Text : Pal::Dim));
	}

	// Battery, right-aligned where a device reports one: a small glyph whose
	// fill is the level, red at the same threshold that swaps the device icon
	// to its low-battery art.
	float textRight = b.x - 16.0f;
	if (dev.connected && dev.battery >= 0.0f)
	{
		const float bw = 26.0f, bh = 12.0f, nub = 2.5f;
		ImVec2 g0 = ImVec2(b.x - 16.0f - bw - nub, p.y + (h - bh) * 0.5f);
		ImVec2 g1 = ImVec2(g0.x + bw, g0.y + bh);
		bool low = dev.battery < kLowBattery && !dev.charging;
		dl->AddRect(g0, g1, Pal::U32(Pal::Faint), 3.0f, ImDrawCornerFlags_All, 1.0f);
		dl->AddRectFilled(ImVec2(g1.x + 1.0f, iconC.y - 2.5f),
			ImVec2(g1.x + 1.0f + nub, iconC.y + 2.5f), Pal::U32(Pal::Faint), 1.0f);
		float level = dev.battery > 1.0f ? 1.0f : dev.battery;
		float fw = (bw - 4.0f) * level;
		if (fw > 0.5f)
			dl->AddRectFilled(ImVec2(g0.x + 2.0f, g0.y + 2.0f), ImVec2(g0.x + 2.0f + fw, g1.y - 2.0f),
				Pal::U32(low ? Pal::Bad : Pal::Good), 1.5f);
		textRight = g0.x - 10.0f;
	}

	ImVec4 clip = ImVec4(p.x, p.y, textRight, b.y);
	float tx = p.x + 58.0f;
	dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(tx, p.y + 6.0f),
		Pal::U32(dev.connected ? Pal::Text : Pal::Dim), dev.model.c_str(), nullptr, 0.0f, &clip);
	dl->AddText(g_fontSmall, g_fontSmall->FontSize, ImVec2(tx, p.y + 31.0f),
		Pal::U32(dev.connected ? Pal::Dim : Pal::Faint), dev.serial.c_str(), nullptr, 0.0f, &clip);

	ImGui::PopID();
	return pressed;
}

// Preferred default: left-hand controller, else first device of the system.
// Reconciles the context's selection against the live device list in place --
// it is the only owner of that selection, so there is no mirror to go stale
// when a tracking system disappears between frames.
static void EnsureDeviceSelection(const VRState &state, uint32_t &selected, const std::string &system)
{
	const uint32_t none = vr::k_unTrackedDeviceIndexInvalid;

	if (selected != none)
	{
		bool matched = false;
		for (auto &device : state.devices)
			if (device.trackingSystem == system && selected == (uint32_t)device.id)
			{
				matched = true;
				break;
			}
		if (!matched)
			selected = none;
	}

	if (selected == none)
	{
		for (auto &device : state.devices)
			if (device.trackingSystem == system && device.controllerRole == vr::TrackedControllerRole_LeftHand)
			{
				selected = (uint32_t)device.id;
				break;
			}
	}

	if (selected == none)
	{
		for (auto &device : state.devices)
			if (device.trackingSystem == system)
			{
				selected = (uint32_t)device.id;
				break;
			}
	}
}

static void BuildDeviceList(const VRState &state, uint32_t &selected, const std::string &system, float paneW)
{
	EnsureDeviceSelection(state, selected, system);

	const float rowH = 52.0f;
	const int maxVisible = 4;
	ImVec2 origin = ImGui::GetCursorScreenPos();
	ImDrawList *dl = ImGui::GetWindowDrawList();

	std::vector<const VRDevice *> devices;
	devices.reserve(state.devices.size());
	for (auto &device : state.devices)
		if (device.trackingSystem == system)
			devices.push_back(&device);

	if (devices.empty())
	{
		ImVec2 b = ImVec2(origin.x + paneW, origin.y + rowH);
		dl->AddRectFilled(origin, b, Pal::U32(Pal::Card), 12.0f);
		dl->AddRect(origin, b, Pal::U32(Pal::Border), 12.0f);
		const char *msg = "No devices in this space";
		ImVec2 ts = ImGui::CalcTextSize(msg);
		dl->AddText(g_fontBody, g_fontBody->FontSize,
			ImVec2(origin.x + (paneW - ts.x) * 0.5f, origin.y + (rowH - ts.y) * 0.5f),
			Pal::U32(Pal::Faint), msg);
		ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + rowH));
		ImGui::Dummy(ImVec2(0, 0));
		return;
	}

	// One container, flat rows inside it separated by hairlines — a grouped
	// list, not stacked cards. More devices than fit -> fixed-height scrolling
	// list; the rows narrow a little to leave room for the scrollbar.
	bool scrolls = (int)devices.size() > maxVisible;
	int visible = scrolls ? maxVisible : (int)devices.size();
	float listH = visible * rowH;
	float rowW = paneW - (scrolls ? 12.0f : 0.0f);

	ImVec2 listB = ImVec2(origin.x + paneW, origin.y + listH);
	dl->AddRectFilled(origin, listB, Pal::U32(Pal::Card), 12.0f);

	ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
	ImGui::BeginChild(system.empty() ? "devlist" : system.c_str(), ImVec2(paneW, listH), false);
	for (size_t i = 0; i < devices.size(); ++i)
	{
		const VRDevice &device = *devices[i];
		if (DeviceRow(device, selected == (uint32_t)device.id, rowW, i == 0, i + 1 == devices.size()))
			selected = (uint32_t)device.id;
	}
	ImGui::EndChild();
	ImGui::PopStyleVar();

	dl->AddRect(origin, listB, Pal::U32(Pal::Border), 12.0f);
	ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + listH));
	ImGui::Dummy(ImVec2(0, 0));
}

// ---------------------------------------------------------------------------
// Spaces + devices section
// ---------------------------------------------------------------------------

// Users know their hardware, not driver names; show friendly labels for the
// systems we recognize and fall back to the raw name for anything else.
static std::string FriendlySystemName(const std::string &raw)
{
	if (raw == "oculus")     return "Meta Quest";
	if (raw == "lighthouse") return "SteamVR Lighthouse";
	if (raw == "holographic") return "Windows Mixed Reality";
	return raw;
}

// Both space panes are the same control over a different candidate list: the
// friendly labels, the index recovery for the current pick, the Combo, and the
// write-back. `fallback` is the index to commit when `selection` is not among
// the candidates (-1 to leave it alone) -- that is how an emptied or vanished
// pick settles onto a real system instead of lingering as a name nothing shows.
static void PickTrackingSystem(const char *id, const std::vector<std::string> &candidates,
	int fallback, float width, std::string &selection)
{
	int current = -1;
	for (size_t i = 0; i < candidates.size(); ++i)
		if (candidates[i] == selection)
		{
			current = (int)i;
			break;
		}
	if (current == -1)
		current = fallback;

	std::vector<std::string> display;
	std::vector<const char *> items;
	display.reserve(candidates.size());
	items.reserve(candidates.size());
	for (const auto &raw : candidates)
		display.push_back(FriendlySystemName(raw));
	for (const auto &label : display)
		items.push_back(label.c_str());

	ImGui::PushItemWidth(width);
	ImGui::Combo(id, &current, items.data(), (int)items.size());
	ImGui::PopItemWidth();

	if (current >= 0 && current < (int)candidates.size())
		selection = candidates[current];
}

static void BuildSpacesSection(const VRState &state)
{
	if (state.trackingSystems.empty())
	{
		const float h = 120.0f;
		ImVec2 p = BeginRowCard(h);
		ImDrawList *dl = ImGui::GetWindowDrawList();
		float cw = ImGui::GetWindowContentRegionWidth();
		const char *msg = "No tracked devices are present";
		ImVec2 ts = ImGui::CalcTextSize(msg);
		IconHMD(dl, ImVec2(p.x + cw * 0.5f, p.y + 42.0f), 16.0f, Pal::U32(Pal::Faint));
		dl->AddText(g_fontBody, g_fontBody->FontSize,
			ImVec2(p.x + (cw - ts.x) * 0.5f, p.y + 70.0f), Pal::U32(Pal::Dim), msg);
		EndRowCard(p, h);
		return;
	}

	// The panes edit pending calibration choices. Active profile system names
	// are committed only by FinishCalibration after a successful base solve.
	// The reference pane offers every system in the list (HMD system first, as
	// LoadVRState ordered it); when the current pick is gone it falls back to
	// the first system that is not already the target.
	int firstReferenceSystemNotTargetSystem = -1;
	for (size_t i = 0; i < state.trackingSystems.size(); ++i)
	{
		const std::string &str = state.trackingSystems[i];
		if (str != CalCtx.pendingReferenceTrackingSystem &&
			str != CalCtx.pendingTargetTrackingSystem)
		{
			firstReferenceSystemNotTargetSystem = (int)i;
			break;
		}
	}

	float cw = ImGui::GetWindowContentRegionWidth();
	const float paneGap = 24.0f;
	float paneW = (cw - paneGap) * 0.5f;

	ImVec2 top = ImGui::GetCursorScreenPos();

	// ---- Left pane: reference space ----
	ImGui::SetCursorScreenPos(top);
	ImGui::BeginGroup();
	SectionLabel("REFERENCE SPACE");
	PickTrackingSystem("##ReferenceTrackingSystem", state.trackingSystems,
		firstReferenceSystemNotTargetSystem, paneW, CalCtx.pendingReferenceTrackingSystem);

	// One space cannot be both sides of the calibration. The reference pane
	// wins and the target pick is dropped; the target list below is rebuilt
	// from the reference, so it can never collide in the other direction.
	if (CalCtx.pendingReferenceTrackingSystem == CalCtx.pendingTargetTrackingSystem)
		CalCtx.pendingTargetTrackingSystem = "";

	ImGui::Spacing();
	BuildDeviceList(state, CalCtx.referenceID, CalCtx.pendingReferenceTrackingSystem, paneW);
	ImGui::EndGroup();
	float leftBottom = ImGui::GetItemRectMax().y;

	// ---- Right pane: target space ----
	std::vector<std::string> targetSystems;
	targetSystems.reserve(state.trackingSystems.size());
	for (auto &str : state.trackingSystems)
		if (str != CalCtx.pendingReferenceTrackingSystem)
			targetSystems.push_back(str);

	ImGui::SetCursorScreenPos(ImVec2(top.x + paneW + paneGap, top.y));
	ImGui::BeginGroup();
	SectionLabel("TARGET SPACE");
	if (!targetSystems.empty())
	{
		// An emptied pick (the collision rule above) settles on the first
		// remaining system; a pick that is simply absent this frame is left
		// alone, so a system that flickers out does not silently retarget.
		PickTrackingSystem("##TargetTrackingSystem", targetSystems,
			CalCtx.pendingTargetTrackingSystem.empty() ? 0 : -1, paneW,
			CalCtx.pendingTargetTrackingSystem);

		ImGui::Spacing();
		BuildDeviceList(state, CalCtx.targetID, CalCtx.pendingTargetTrackingSystem, paneW);
	}
	else
	{
		ImGui::TextDisabled("No second tracking system detected");
		// The pane is showing nothing, so nothing may stay selected: this id is
		// what the identify pulse buzzes and what StartCalibration freezes.
		CalCtx.targetID = vr::k_unTrackedDeviceIndexInvalid;
	}
	ImGui::EndGroup();
	float rightBottom = ImGui::GetItemRectMax().y;

	ImGui::SetCursorScreenPos(ImVec2(top.x, (leftBottom > rightBottom ? leftBottom : rightBottom)));
	ImGui::Dummy(ImVec2(0, 0));

	// ---- Identify ----
	ImGui::Spacing();
	bool identifyPressed = IconButton("identify", "IDENTIFY SELECTED DEVICES", IconCrosshair,
		ImVec2(cw, 38.0f), BtnKind::Quiet, true);
	if (ImGui::IsItemHovered())
	{
		ImGui::SetTooltip(
			"Blinks the LED or vibrates the selected devices\n"
			"so you can tell which is which.");
	}
	if (identifyPressed)
	{
		g_identifyPulse.active = true;
		g_identifyPulse.targetId = CalCtx.targetID;
		g_identifyPulse.referenceId = CalCtx.referenceID;
		g_identifyPulse.pulsesRemaining = 100;
		g_identifyPulse.nextPulseTime = ImGui::GetTime();
	}
}

// ---------------------------------------------------------------------------
// Main menu (default state)
// ---------------------------------------------------------------------------

static void OpenCalibrationPopup(bool anchor)
{
	if (g_uiPreviewMode)
	{
		// Preview: fake progress so the modal can be styled without VR.
		CalCtx.ClearMessages();
		CalCtx.Log("Keep the selected devices rigidly together.\n"
			"Move them through wide, varied rotations around at least two different axes and across the play area.\n\n");
		CalCtx.Progress(65, 100);
		ImGui::OpenPopup("Calibration Progress");
		return;
	}

	bool started = anchor ? StartAnchorCalibration() : StartCalibration();
	if (started)
		ImGui::OpenPopup("Calibration Progress");
}

static void BuildMainScreen()
{
	float cw = ImGui::GetWindowContentRegionWidth();
	const float gap = 12.0f;

	{
		{
			std::vector<StatusRowData> warn;
			if (CalCtx.validProfile && !CalCtx.enabled)
			{
				// Six conditions disable a profile and each wants a different
				// action from the user; naming the headset for all of them sent
				// people to check hardware that was working.
				using Reason = CalibrationContext::DisableReason;
				std::string why;
				switch (CalCtx.disableReason)
				{
				case Reason::HmdMismatch:
					why = FormatString("%s headset not detected -- profile disabled",
						FriendlySystemName(CalCtx.referenceTrackingSystem).c_str());
					break;
				case Reason::DriverUnreachable:
					why = "Driver did not accept the profile -- profile disabled until the next scan";
					break;
				case Reason::InvalidIdentity:
					why = "The profile's tracking systems are no longer valid -- recalibrate";
					break;
				case Reason::InvalidTransform:
					why = "The stored calibration contains invalid values -- recalibrate";
					break;
				case Reason::UniverseUnsafe:
				case Reason::None:
					why = "Raw tracking universe changed while monitoring was offline -- recalibrate before using this profile";
					break;
				}
				warn.push_back({ IconInfo, Pal::Bad, why });
			}
			if (PoseChannelDown())
			{
				warn.push_back({ IconInfo, Pal::Warn,
					"Pose channel unavailable -- runtime monitoring is off (universe jumps, drift and continuous calibration)" });
			}
			if (!warn.empty())
			{
				DrawStatusCard(warn);
				ImGui::Spacing();
			}
		}

		// ---- Action buttons: Start dominates; Clear is a narrow sidecar ----
		bool haveProfile = CalCtx.validProfile;
		const float bh = 56.0f;
		const float clearW = 210.0f;
		float startW = haveProfile ? cw - clearW - gap : cw;

		if (IconButton("start", "Start calibration", IconPlay, ImVec2(startW, bh), BtnKind::Primary))
			OpenCalibrationPopup(false);

		if (haveProfile)
		{
			ImGui::SameLine(0.0f, gap);
			if (IconButton("clear", "Clear calibration", IconTrash, ImVec2(clearW, bh), BtnKind::Ghost))
				ClearSavedProfile(CalCtx);
		}

		// ---- Calibration speed (quiet row, no card chrome) ----
		{
			const float rowH = 46.0f;
			ImVec2 p = ImGui::GetCursorScreenPos();
			ImDrawList *dl = ImGui::GetWindowDrawList();
			IconGauge(dl, ImVec2(p.x + 14.0f, p.y + rowH * 0.5f), 9.0f, Pal::U32(Pal::Dim));
			dl->AddText(g_fontBody, g_fontBody->FontSize,
				ImVec2(p.x + 38.0f, p.y + rowH * 0.5f - g_fontBody->FontSize * 0.5f),
				Pal::U32(Pal::Text), "Calibration speed");
			const char *speeds[] = { "Fast", "Slow", "Very Slow" };
			float itemW = 110.0f, segH = 38.0f;
			ImGui::SetCursorScreenPos(ImVec2(p.x + cw - (itemW * 3.0f + 8.0f), p.y + (rowH - segH) * 0.5f));
			auto previousSpeed = CalCtx.calibrationSpeed;
			CalCtx.calibrationSpeed = static_cast<CalibrationContext::Speed>(
				Segmented("speed", static_cast<int>(CalCtx.calibrationSpeed), speeds, 3, itemW, segH));
			if (CalCtx.calibrationSpeed != previousSpeed)
				SaveSettingOrRestore(CalCtx.calibrationSpeed, previousSpeed);
			ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + rowH));
			ImGui::Dummy(ImVec2(0, 0));
		}

		// ---- Chaperone + field anchor ----
		// One button covers the whole chaperone story: snapshot the current
		// bounds AND arm auto-restore. People who rely on the Quest boundary
		// transferring in each session simply never press it (or disarm the
		// restore in settings).
		float chapW = CalCtx.validProfile && !CalCtx.profileUniverseUnsafe
			? (cw - gap) * 0.5f : cw;
		const char *chapLabel = CalCtx.chaperone.valid ? "Update protected chaperone" : "Protect chaperone";
		if (IconButton("copychap", chapLabel, IconCopy, ImVec2(chapW, 46.0f), BtnKind::Ghost))
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
			ImGui::SetTooltip(
				"Saves your current chaperone bounds (SteamVR's walls) and puts\n"
				"them back automatically if SteamVR or the headset ever loses them.\n"
				"Redrew your chaperone? Press again to save the new one.\n"
				"Prefer the Quest's Guardian imported fresh each session? Don't use this.");
		}
		if (CalCtx.validProfile && !CalCtx.profileUniverseUnsafe)
		{
			ImGui::SameLine(0.0f, gap);
			if (IconButton("addanchor", "Add field anchor", IconPin, ImVec2(chapW, 46.0f), BtnKind::Ghost))
				OpenCalibrationPopup(true);
			if (ImGui::IsItemHovered())
			{
				ImGui::SetTooltip(
					"Feels aligned in one spot but slightly off in another?\n"
					"Stand at the bad spot, press this, and do a quick calibration there.\n"
					"That spot gets its own correction, blended in as you walk around.");
			}
		}

		// ---- Status ----
		std::vector<StatusRowData> rows;
		const ContinuousStatus continuous = ContinuousStatusNow();
		if (!CalCtx.uiAdvanced)
		{
			// Quiet strip anchored just above the footer: no card, a rating-colored
			// dot, and the recommendation on its own violet line when it matters.
			CalRating rating = CalCtx.validProfile ? ComputeCalibrationRating(continuous) : Rating_Unknown;
			const char *nudge = CalCtx.validProfile ? RecalibrationNudge(rating, continuous) : nullptr;
			const float lineH = g_fontBody->FontSize + 8.0f;
			float stripH = nudge ? lineH * 2.0f + 4.0f : lineH;

			// Bottom status band: full-bleed inset surface with a hairline top
			// edge, so the status text sits on something instead of floating.
			float bandH = stripH + 16.0f + 46.0f;
			float bandTop = ImGui::GetWindowHeight() - bandH;
			if (ImGui::GetCursorPosY() < bandTop)
			{
				ImVec2 wp = ImGui::GetWindowPos();
				float ww = ImGui::GetWindowWidth();
				ImDrawList *dlb = ImGui::GetWindowDrawList();
				dlb->AddRectFilled(ImVec2(wp.x, wp.y + bandTop),
					ImVec2(wp.x + ww, wp.y + ImGui::GetWindowHeight()), Pal::U32(Pal::Inset));
				dlb->AddLine(ImVec2(wp.x, wp.y + bandTop),
					ImVec2(wp.x + ww, wp.y + bandTop), Pal::U32(Pal::Border), 1.0f);
				ImGui::SetCursorPosY(bandTop + 16.0f);
			}
			ImVec2 p = ImGui::GetCursorScreenPos();
			ImDrawList *dl = ImGui::GetWindowDrawList();
			float cy = p.y + lineH * 0.5f;
			float ty = cy - g_fontBody->FontSize * 0.5f;

			if (CalCtx.validProfile)
			{
				// The colored rating word is the sole status indicator; line 1
				// names the symptom, the violet line names the cure.
				ImVec4 rcol = RatingColor(rating);
				float x = p.x;
				const char *label = RatingLabel(rating);
				dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(x, ty), Pal::U32(Pal::Text), "Tracking quality: ");
				x += ImGui::CalcTextSize("Tracking quality: ").x;
				dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(x, ty), Pal::U32(rcol), label);
				x += ImGui::CalcTextSize(label).x;

				std::string ageLine;
				if (continuous == ContinuousStatus::Tracking)
				{
					std::optional<std::string> adjusted;
					if (CalCtx.autoCorrectionsApplied > 0)
						adjusted = FormatUnixAge(CalCtx.lastAutoCorrectionUnixTime);
					ageLine = adjusted
						? FormatString("maintained continuously -- last adjusted %s", adjusted->c_str())
						: std::string("maintained continuously");
				}
				else
				{
					auto age = FormatAlignmentAge();
					ageLine = age ? *age : std::string("calibration time unknown");
				}
				dl->AddText(g_fontSmall, g_fontSmall->FontSize,
					ImVec2(x + 18.0f, cy - g_fontSmall->FontSize * 0.5f + 2.0f),
					Pal::U32(Pal::Faint), ageLine.c_str());

				if (nudge)
					dl->AddText(g_fontBody, g_fontBody->FontSize,
						ImVec2(p.x, ty + lineH + 4.0f),
						Pal::U32(Pal::Violet), nudge);
			}
			else
			{
				dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(p.x, ty),
					Pal::U32(Pal::Dim), "No calibration yet -- press Start calibration");
			}

			ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + stripH));
			ImGui::Dummy(ImVec2(0, 0));
		}
		else
		{
			if (CalCtx.validProfile && CalCtx.lastResult.valid)
			{
				rows.push_back({ IconCheck, Pal::Good,
					FormatString("Last calibration: rotation RMS %.2f deg, position RMS %.1f cm, time offset %+.1f ms%s",
						CalCtx.lastResult.rotationRmsDeg,
						CalCtx.lastResult.translationRmsMeters * 100.0,
						CalCtx.lastResult.timeOffset * 1000.0,
						CalCtx.lastResult.scale != 1.0 ? ", scaled" : "") });
			}

			if (CalCtx.jumpsCompensated > 0 || CalCtx.referenceGapEvents > 0)
			{
				rows.push_back({ IconInfo, ImVec4(0.55f, 0.65f, 0.90f, 1.0f),
					FormatString("Universe jumps compensated: %u%s",
						CalCtx.jumpsCompensated,
						CalCtx.referenceGapEvents > 0 ? " (tracking gaps seen -- recalibrate if alignment looks off)" : "") });
			}

			if (CalCtx.validProfile)
			{
				ImVec4 healthColor =
					CalCtx.alignment == CalibrationContext::AlignmentHealth::Stale ? Pal::Warn :
					CalCtx.alignment == CalibrationContext::AlignmentHealth::Aging ? Pal::Warn :
					Pal::Good;
				IconFn healthIcon =
					CalCtx.alignment == CalibrationContext::AlignmentHealth::Fresh ? IconCheck : IconClock;
				const char *healthLabel =
					CalCtx.alignment == CalibrationContext::AlignmentHealth::Stale ? "stale" :
					CalCtx.alignment == CalibrationContext::AlignmentHealth::Aging ? "aging" : "fresh";

				// Same base as the health label beside it (see FormatAlignmentAge).
				auto alignmentAge = FormatAlignmentAge();
				const char *age = alignmentAge ? alignmentAge->c_str() : "age unknown";

				if (CalCtx.driftSlideEvents > 0 || CalCtx.discontinuousLossEvents > 0)
				{
					rows.push_back({ healthIcon, healthColor,
						FormatString("Alignment %s (%s): %u stationary slide(s) up to %.1f cm, %u tracking glitch(es)",
							healthLabel, age,
							CalCtx.driftSlideEvents, CalCtx.driftMaxSlideM * 100.0,
							CalCtx.discontinuousLossEvents) });
				}
				else
				{
					rows.push_back({ healthIcon, healthColor,
						FormatString("Alignment %s (%s)", healthLabel, age) });
				}

				if (continuous != ContinuousStatus::Off)
				{
					ImVec4 col = continuous == ContinuousStatus::Tracking ? Pal::Good :
						continuous == ContinuousStatus::Frozen ? Pal::Bad : Pal::Warn;
					std::string line;
					if (continuous == ContinuousStatus::Tracking && CalCtx.continuousDeviation.valid)
						line = FormatString(
							"Continuous: maintaining -- scatter %.2f deg / %.1f cm, deviation %.2f deg / %.1f cm, %u corrections",
							CalCtx.continuousScatterRotDeg, CalCtx.continuousScatterPosM * 100.0,
							CalCtx.continuousDeviation.yawDeg + CalCtx.continuousDeviation.tiltDeg,
							CalCtx.continuousDeviation.posM * 100.0,
							CalCtx.autoCorrectionsApplied);
					else
						line = FormatString("Continuous: %s -- %u corrections applied",
							ContinuousStatusText(continuous), CalCtx.autoCorrectionsApplied);
					rows.push_back({ IconCrosshair, col, line });
				}

				if (const char *nudge = RecalibrationNudge(ComputeCalibrationRating(continuous), continuous))
					rows.push_back({ IconInfo, Pal::Violet, nudge });
			}
		}

		if (!rows.empty())
			DrawStatusCard(rows);

	}
}

static void BuildSettingsScreen(const VRState &state)
{
	float cw = ImGui::GetWindowContentRegionWidth();
	const float gap = 12.0f;
	(void)gap;

	{
		SectionLabel("SETTINGS");
		ImGui::Spacing();

		// Advanced mode
		{
			bool previous = CalCtx.uiAdvanced;
			if (ToggleRow("##uiAdvanced", IconGauge,
				"Advanced mode (show raw calibration and drift stats)", CalCtx.uiAdvanced))
				SaveSettingOrRestore(CalCtx.uiAdvanced, previous);
		}

		// Poor calibration notification
		{
			bool previous = CalCtx.notifyPoorCalibration;
			if (ToggleRow("##notifyPoorCalibration", IconInfo,
				"Poor calibration notification", CalCtx.notifyPoorCalibration,
				"Show a SteamVR notification when alignment monitoring recommends\n"
				"recalibration. The status and session log still update when disabled."))
				SaveSettingOrRestore(CalCtx.notifyPoorCalibration, previous);
		}

		// Spatial correction field
		if (CalCtx.validProfile)
		{
			size_t anchorCount = CalCtx.fieldAnchors.size();
			RowCard row(kRowHeight + (anchorCount > 0 ? anchorCount * 24.0f + 6.0f : 0.0f));
			const ImVec2 p = row.pos;

			ImGui::SetCursorScreenPos(ImVec2(p.x + kRowInsetX, p.y + kRowControlY));
			// Toggled through a local: the live member changes only after the
			// transaction has persisted the candidate that carries this value.
			bool fieldEnabled = CalCtx.fieldEnabled;
			if (QCCheckbox("##fieldEnabled", &fieldEnabled))
			{
				if (SaveProfileFieldEdit(CalCtx,
					[&](questcal::ProfileRecord &candidate) {
						candidate.fieldEnabled = fieldEnabled;
					},
					true))
					ResyncDriverState();
			}
			if (ImGui::IsItemHovered())
			{
				ImGui::SetTooltip(
					"Blends per-spot calibration anchors so the correction follows the\n"
					"room instead of assuming one rigid transform fits everywhere.\n"
					"Collect anchors at spots where alignment is visibly off.");
			}
			RowIconLabel(p, IconField, "Spatial correction field");

			if (anchorCount > 0)
			{
				float btnW = 150.0f;
				ImGui::SetCursorScreenPos(ImVec2(p.x + cw - kRowInsetX - btnW, p.y + 9.0f));
				if (IconButton("clearanchors", "Clear anchors", IconTrash, ImVec2(btnW, 34.0f), BtnKind::Ghost))
				{
					if (SaveProfileFieldEdit(CalCtx,
						[](questcal::ProfileRecord &candidate) {
							candidate.fieldAnchors.clear();
						},
						true))
						ResyncDriverState();
				}

				ImDrawList *dl = ImGui::GetWindowDrawList();
				for (size_t i = 0; i < CalCtx.fieldAnchors.size(); ++i)
				{
					const auto &a = CalCtx.fieldAnchors[i];
					// Delta vs base, evaluated at the anchor's own spot.
					Eigen::Vector3d targetPt = a.rotation.conjugate() * (a.position - a.translationMeters);
					Eigen::Vector3d basePos = CalCtx.calibratedRotationQ * targetPt + CalCtx.TranslationMeters();
					double posDeltaCm = (a.position - basePos).norm() * 100.0;
					double rotDeltaDeg = a.rotation.angularDistance(CalCtx.calibratedRotationQ) * 180.0 / EIGEN_PI;
					std::string line = FormatString("Anchor %zu at (%+.1f, %+.1f): %.1f cm / %.2f deg from base",
						i + 1, a.position.x(), a.position.z(), posDeltaCm, rotDeltaDeg);
					dl->AddText(g_fontSmall, g_fontSmall->FontSize,
						ImVec2(p.x + 92.0f, p.y + kRowHeight + i * 24.0f), Pal::U32(Pal::Dim), line.c_str());
				}
			}
		}

		// Solve playspace scale
		{
			bool previous = CalCtx.solveScale;
			if (ToggleRow("##solveScale", IconScale,
				"Solve playspace scale (experimental)", CalCtx.solveScale))
				SaveSettingOrRestore(CalCtx.solveScale, previous);
		}

		// Time offset + nested manual override
		{
			RowCard row(104.0f);
			const ImVec2 p = row.pos;
			ImDrawList *dl = ImGui::GetWindowDrawList();

			ImGui::SetCursorScreenPos(ImVec2(p.x + kRowInsetX, p.y + kRowControlY));
			bool previous = CalCtx.applyTimeOffset;
			if (QCCheckbox("##applyTimeOffset", &CalCtx.applyTimeOffset))
				SaveSettingOrRestore(CalCtx.applyTimeOffset, previous);
			if (ImGui::IsItemHovered())
			{
				ImGui::SetTooltip(
					"Shifts the target devices' timeline to match the reference system,\n"
					"removing relative wobble during motion. Held controllers on the target\n"
					"side will feel slightly delayed (affects throws in physics games).");
			}
			RowIconLabel(p, IconClock, "Apply time offset (latency re-prediction)");

			if (CalCtx.validProfile && (CalCtx.applyTimeOffset || CalCtx.useManualTimeOffset))
			{
				std::string applied = FormatString("applied %+.1f ms", CalCtx.appliedTimeOffset * 1000.0);
				ImVec2 ts = ImGui::CalcTextSize(applied.c_str());
				dl->AddText(g_fontBody, g_fontBody->FontSize,
					ImVec2(p.x + cw - kRowInsetX - ts.x, p.y + 26.0f - g_fontBody->FontSize * 0.5f),
					Pal::U32(Pal::Faint), applied.c_str());
			}

			// Nested inset: manual override spike tool (verifies the poseTimeOffset
			// sign convention against a live session; bypasses the solved value).
			ImVec2 np = ImVec2(p.x + 12.0f, p.y + kRowHeight);
			ImVec2 nb = ImVec2(p.x + cw - 12.0f, p.y + row.height - 10.0f);
			dl->AddRectFilled(np, nb, Pal::U32(Pal::Inset), 9.0f);

			ImGui::SetCursorScreenPos(ImVec2(np.x + 12.0f, np.y + 9.0f));
			QCCheckbox("##manualOverride", &CalCtx.useManualTimeOffset);
			dl->AddText(g_fontBody, g_fontBody->FontSize,
				ImVec2(np.x + 48.0f, (np.y + nb.y) * 0.5f - g_fontBody->FontSize * 0.5f),
				Pal::U32(CalCtx.useManualTimeOffset ? Pal::Text : Pal::Dim), "Manual time offset override (debug)");

			ImVec2 msSize = ImGui::CalcTextSize("ms");
			float inputW = 110.0f;
			dl->AddText(g_fontBody, g_fontBody->FontSize,
				ImVec2(nb.x - 14.0f - msSize.x, (np.y + nb.y) * 0.5f - g_fontBody->FontSize * 0.5f),
				Pal::U32(Pal::Dim), "ms");
			ImGui::SetCursorScreenPos(ImVec2(nb.x - 14.0f - msSize.x - 10.0f - inputW, np.y + 5.0f));
			ImGui::PushItemWidth(inputW);
			float manualMs = (float)CalCtx.manualTimeOffsetMs;
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10, 6));
			if (ImGui::InputFloat("##manualTimeOffset", &manualMs, 0.0f, 0.0f, 1))
			{
				if (!std::isfinite(manualMs)) manualMs = 0.0f;
				if (manualMs > 50.0f) manualMs = 50.0f;
				if (manualMs < -50.0f) manualMs = -50.0f;
				CalCtx.manualTimeOffsetMs = manualMs;
			}
			ImGui::PopStyleVar();
			ImGui::PopItemWidth();
		}

		// Continuous calibration: enable + nested tracker pick / hide / latency
		if (CalCtx.validProfile)
		{
			ContinuousStatus continuous = ContinuousStatusNow();
			bool advisory = continuous == ContinuousStatus::NoTracker ||
				continuous == ContinuousStatus::NeedsMount;
			const float nestedH = CalCtx.continuousEnabled ? 116.0f : 0.0f;
			const float advisoryH = advisory ? 26.0f : 0.0f;
			RowCard row(kRowHeight + nestedH + advisoryH);
			const ImVec2 p = row.pos;
			ImDrawList *dl = ImGui::GetWindowDrawList();

			ImGui::SetCursorScreenPos(ImVec2(p.x + kRowInsetX, p.y + kRowControlY));
			bool continuousEnabled = CalCtx.continuousEnabled;
			if (QCCheckbox("##continuousEnabled", &continuousEnabled))
				SaveProfileFieldEdit(CalCtx,
					[&](questcal::ProfileRecord &candidate) {
						candidate.continuousEnabled = continuousEnabled;
					});
			if (ImGui::IsItemHovered())
			{
				ImGui::SetTooltip(
					"Keeps the calibration aligned automatically while you play, using a\n"
					"spare tracker mounted firmly on the headset. Small corrections are\n"
					"applied smoothly and continuously; if the tracker is bumped or lost,\n"
					"updates pause and you get a notification instead.");
			}
			RowIconLabel(p, IconCrosshair, "Continuous calibration (experimental)");

			if (CalCtx.continuousEnabled)
			{
				// The toggle above changes what the loop is doing; the row height
				// for this frame is already committed, but the text is not.
				continuous = ContinuousStatusNow();
				std::string status = ContinuousStatusText(continuous);
				ImVec2 ts = ImGui::CalcTextSize(status.c_str());
				dl->AddText(g_fontBody, g_fontBody->FontSize,
					ImVec2(p.x + cw - kRowInsetX - ts.x, p.y + 26.0f - g_fontBody->FontSize * 0.5f),
					Pal::U32(Pal::Faint), status.c_str());

				// Nested inset: tracker pick, game visibility, opt-in latency.
				ImVec2 np = ImVec2(p.x + 12.0f, p.y + kRowHeight);
				ImVec2 nb = ImVec2(p.x + cw - 12.0f, p.y + kRowHeight + nestedH - 8.0f);
				dl->AddRectFilled(np, nb, Pal::U32(Pal::Inset), 9.0f);

				dl->AddText(g_fontBody, g_fontBody->FontSize,
					ImVec2(np.x + 12.0f, np.y + 17.0f - g_fontBody->FontSize * 0.5f),
					Pal::U32(Pal::Text), "Mounted tracker");

				// Candidates: every connected target-system device except the HMD.
				std::vector<const VRDevice *> candidates;
				candidates.reserve(state.devices.size());
				for (const auto &d : state.devices)
					if (d.trackingSystem == CalCtx.targetTrackingSystem &&
						d.deviceClass != vr::TrackedDeviceClass_HMD)
						candidates.push_back(&d);

				std::vector<std::string> labels;
				labels.reserve(candidates.size() + 1);
				int sel = -1;
				for (size_t i = 0; i < candidates.size(); ++i)
				{
					labels.push_back(candidates[i]->model + " -- " + candidates[i]->serial);
					if (candidates[i]->serial == CalCtx.continuousTrackerSerial)
						sel = (int)i;
				}
				if (sel < 0 && !CalCtx.continuousTrackerSerial.empty())
				{
					labels.push_back(CalCtx.continuousTrackerSerial + " (not connected)");
					sel = (int)labels.size() - 1;
				}
				std::vector<const char *> items;
				items.reserve(labels.size());
				for (const auto &l : labels)
					items.push_back(l.c_str());

				float comboW = 340.0f;
				ImGui::SetCursorScreenPos(ImVec2(nb.x - 14.0f - comboW, np.y + 5.0f));
				ImGui::PushItemWidth(comboW);
				ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10, 6));
				if (ImGui::Combo("##contTracker", &sel, items.data(), (int)items.size()) &&
					sel >= 0 && sel < (int)candidates.size())
				{
					if (CalCtx.continuousTrackerSerial != candidates[sel]->serial)
					{
						// The serial and the extrinsic are one edit -- the learned
						// mount offset described the previous tracker, and a
						// different physical device needs a fresh calibration --
						// so they land together or not at all. Stating both on the
						// candidate is what makes that atomic; there is no
						// two-member rollback left to get wrong.
						std::string serial = candidates[sel]->serial;
						SaveProfileFieldEdit(CalCtx,
							[&](questcal::ProfileRecord &candidate) {
								candidate.continuousTrackerSerial = serial;
								candidate.mountExtrinsic =
									questcal::MountExtrinsicRecord();
							});
					}
				}
				ImGui::PopStyleVar();
				ImGui::PopItemWidth();

				ImGui::SetCursorScreenPos(ImVec2(np.x + 12.0f, np.y + 40.0f));
				bool hideMountedTracker = CalCtx.hideMountedTracker;
				if (QCCheckbox("##hideTracker", &hideMountedTracker))
					SaveProfileFieldEdit(CalCtx,
						[&](questcal::ProfileRecord &candidate) {
							candidate.hideMountedTracker = hideMountedTracker;
						});
				if (ImGui::IsItemHovered())
				{
					ImGui::SetTooltip(
						"Moves the mounted tracker far out of reach in games so full-body\n"
						"setups never mistake it for a body tracker. Calibration still sees it.");
				}
				dl->AddText(g_fontBody, g_fontBody->FontSize,
					ImVec2(np.x + 48.0f, np.y + 53.0f - g_fontBody->FontSize * 0.5f),
					Pal::U32(CalCtx.hideMountedTracker ? Pal::Text : Pal::Dim),
					"Hide the mounted tracker from games");

				ImGui::SetCursorScreenPos(ImVec2(np.x + 12.0f, np.y + 74.0f));
				bool latencyReestimation = CalCtx.continuousLatencyReestimation;
				if (QCCheckbox("##contLatency", &latencyReestimation))
					SaveProfileFieldEdit(CalCtx,
						[&](questcal::ProfileRecord &candidate) {
							candidate.continuousLatencyReestimation = latencyReestimation;
						});
				if (ImGui::IsItemHovered())
				{
					ImGui::SetTooltip(
						"Also keeps the solved time offset up to date during play, measured\n"
						"from the mounted tracker. Off keeps the value from calibration.");
				}
				dl->AddText(g_fontBody, g_fontBody->FontSize,
					ImVec2(np.x + 48.0f, np.y + 87.0f - g_fontBody->FontSize * 0.5f),
					Pal::U32(CalCtx.continuousLatencyReestimation ? Pal::Text : Pal::Dim),
					"Re-estimate time offset continuously");
			}

			if (advisory)
			{
				dl->AddText(g_fontBody, g_fontBody->FontSize,
					ImVec2(p.x + kRowInsetX, p.y + kRowHeight + nestedH + 2.0f),
					Pal::U32(Pal::Violet),
					"Mount a spare tracker firmly on the headset, then calibrate using the headset as the reference device.");
			}
		}

		// Chaperone: auto-restore arm/disarm + manual restore + snapshot info
		if (CalCtx.chaperone.valid)
		{
			RowCard row(76.0f);
			const ImVec2 p = row.pos;
			ImGui::SetCursorScreenPos(ImVec2(p.x + kRowInsetX, p.y + kRowControlY));
			if (QCCheckbox("##chapAuto", &CalCtx.chaperone.autoApply))
			{
				// Disarming is fail-closed: a failed write must never roll the
				// in-memory state back to armed. A failed enable is likewise
				// reverted to false until Settings can be persisted truthfully.
				if (!SaveSettings(CalCtx))
				{
					CalCtx.chaperone.autoApply = false;
					CalCtx.MarkSettingsDirty(CalCtx.timeLastTick);
				}
			}
			if (ImGui::IsItemHovered())
			{
				ImGui::SetTooltip(
					"Turn off to let the Quest's Guardian import win each session;\n"
					"the saved chaperone then only applies when you press Restore.");
			}
			RowIconLabel(p, IconCopy, "Restore protected chaperone automatically if it changes");

			std::string info;
			if (CalCtx.chaperone.geometry.empty())
			{
				info = "Snapshot: play area center only (no chaperone walls) -- nothing to restore automatically";
			}
			else
			{
				auto copied = FormatUnixAge(CalCtx.chaperone.copyUnixTime);
				info = FormatString("Snapshot: %zu wall segment(s), %.1f x %.1f m play area, saved %s",
					CalCtx.chaperone.geometry.size(),
					CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1],
					copied ? copied->c_str() : "age unknown");
			}
			ImGui::GetWindowDrawList()->AddText(g_fontSmall, g_fontSmall->FontSize,
				ImVec2(p.x + 92.0f, p.y + 46.0f), Pal::U32(Pal::Dim), info.c_str());

			float btnW = 224.0f;
			ImGui::SetCursorScreenPos(ImVec2(p.x + cw - kRowInsetX - btnW, p.y + 9.0f));
			if (IconButton("pastechap", "Restore chaperone now", IconCopy, ImVec2(btnW, 34.0f), BtnKind::Ghost))
				ApplyChaperoneBounds();
		}

		// Raw transform editor -- power users only.
		if (CalCtx.validProfile && !CalCtx.profileUniverseUnsafe)
		{
			ImGui::Spacing();
			if (IconButton("edit", "Edit calibration (advanced)", IconPencil, ImVec2(cw, 46.0f), BtnKind::Ghost))
			{
				SeedTransformEditorDraft();
				CalCtx.state = CalibrationState::Editing;
			}
		}
	}
}

static void BuildMenu(const VRState &state, bool runningInOverlay)
{
	auto &io = ImGui::GetIO();
	float cw = ImGui::GetWindowContentRegionWidth();

	if (CalCtx.state == CalibrationState::None)
	{
		if (s_showSettings)
			BuildSettingsScreen(state);
		else
			BuildMainScreen();
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
		ImVec2 p = BeginRowCard(52.0f);
		ImDrawList *dl = ImGui::GetWindowDrawList();
		const char *msg = "Calibration in progress...";
		ImVec2 ts = ImGui::CalcTextSize(msg);
		dl->AddText(g_fontBody, g_fontBody->FontSize,
			ImVec2(p.x + (cw - ts.x) * 0.5f, p.y + 26.0f - g_fontBody->FontSize * 0.5f),
			Pal::U32(Pal::Dim), msg);
		EndRowCard(p, 52.0f);
	}

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
		if (runningInOverlay)
		{
			const char *hint = "Close VR overlay to use mouse";
			ImVec2 ts = ImGui::CalcTextSize(hint);
			ImGui::SameLine(cw - ts.x);
			ImGui::TextColored(Pal::Faint, hint);
		}
		ImGui::PopFont();
	}

	// ---- Calibration progress modal ----
	float modalW = 720.0f;
	ImGui::SetNextWindowPos(ImVec2((io.DisplaySize.x - modalW) * 0.5f, 180.0f), ImGuiSetCond_Always);
	ImGui::SetNextWindowSize(ImVec2(modalW, 0.0f), ImGuiSetCond_Always);
	if (ImGui::BeginPopupModal("Calibration Progress", nullptr,
		ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove))
	{
		SectionLabel("CALIBRATION");
		ImGui::Spacing();

		for (auto &message : CalCtx.messages)
		{
			switch (message.type)
			{
			case CalibrationContext::Message::String:
				ImGui::TextWrapped("%s", message.str.c_str());
				break;
			case CalibrationContext::Message::Progress:
				float fraction = (float)message.progress / (float)message.target;
				ImGui::Spacing();
				ImGui::PushStyleColor(ImGuiCol_FrameBg, Pal::Inset);
				ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 5.0f);
				ImGui::ProgressBar(fraction, ImVec2(-1.0f, 10.0f), "");
				ImGui::PopStyleVar();
				ImGui::PopStyleColor();
				ImGui::PushFont(g_fontSmall);
				ImGui::TextColored(Pal::Dim, "%d%%", (int)(fraction * 100));
				ImGui::PopFont();
				break;
			}
		}

		if (CalCtx.state == CalibrationState::None)
		{
			ImGui::Spacing();
			if (IconButton("closeprogress", "Close", nullptr, ImVec2(ImGui::GetWindowContentRegionWidth(), 46.0f), BtnKind::Ghost))
				ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

	// ---- One-time chaperone drift warning ----
	// Shown before the first "Protect chaperone" ever runs; the accept button
	// unlocks after a short countdown so the caveat actually gets read.
	ImGui::SetNextWindowPos(ImVec2((io.DisplaySize.x - modalW) * 0.5f, 180.0f), ImGuiSetCond_Always);
	ImGui::SetNextWindowSize(ImVec2(modalW, 0.0f), ImGuiSetCond_Always);
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
			std::string lbl = FormatString("I understand (%d)", remain);
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
					CalCtx.MarkSettingsDirty(CalCtx.timeLastTick);
				}
			}
		}
		ImGui::SameLine(0.0f, bgap);
		if (IconButton("chapwarncancel", "Cancel", nullptr, ImVec2(cancelW, 46.0f), BtnKind::Ghost))
			ImGui::CloseCurrentPopup();

		ImGui::EndPopup();
	}
}

// ---------------------------------------------------------------------------
// Profile editor
// ---------------------------------------------------------------------------

struct TransformEditorDraft
{
	bool valid = true;
	bool rotationEdited = false;
	Eigen::Quaterniond rotationQ{ 1, 0, 0, 0 };
	Eigen::Vector3d rotationEuler{ 0, 0, 0 };
	Eigen::Vector3d translationCm{ 0, 0, 0 };
	double scale = 1.0;
};

static TransformEditorDraft g_transformDraft;

// Seeding belongs with the state transition that puts the editor on screen:
// every path into CalibrationState::Editing calls this immediately before
// setting the state, which is why the draft needs no "is it seeded" flag of
// its own -- the two used to be separate pieces of state that three call sites
// had to change together.
static void SeedTransformEditorDraft()
{
	g_transformDraft = TransformEditorDraft();
	g_transformDraft.rotationQ = CalCtx.calibratedRotationQ;
	g_transformDraft.rotationEuler = CalCtx.calibratedRotation;
	g_transformDraft.translationCm = CalCtx.calibratedTranslation;
	g_transformDraft.scale = CalCtx.calibratedScale;
}

static bool BuildProfileEditor()
{
	ImGuiStyle &style = ImGui::GetStyle();
	float cw = ImGui::GetWindowContentRegionWidth();
	float width = cw / 3.0f - style.FramePadding.x;
	float widthF = width - style.FramePadding.x;

	SectionLabel("ROTATION (DEGREES)");

	ImGui::PushItemWidth(widthF);
	bool rotationEdited = false;
	rotationEdited |= ImGui::InputDouble("Yaw##Yaw", &g_transformDraft.rotationEuler(1), 0.1, 1.0, "%.8f");
	ImGui::SameLine();
	rotationEdited |= ImGui::InputDouble("Pitch##Pitch", &g_transformDraft.rotationEuler(2), 0.1, 1.0, "%.8f");
	ImGui::SameLine();
	rotationEdited |= ImGui::InputDouble("Roll##Roll", &g_transformDraft.rotationEuler(0), 0.1, 1.0, "%.8f");

	ImGui::Spacing();
	SectionLabel("TRANSLATION (CENTIMETERS)");

	bool edited = rotationEdited;
	edited |= ImGui::InputDouble("X##X", &g_transformDraft.translationCm(0), 1.0, 10.0, "%.8f");
	ImGui::SameLine();
	edited |= ImGui::InputDouble("Y##Y", &g_transformDraft.translationCm(1), 1.0, 10.0, "%.8f");
	ImGui::SameLine();
	edited |= ImGui::InputDouble("Z##Z", &g_transformDraft.translationCm(2), 1.0, 10.0, "%.8f");

	ImGui::Spacing();
	SectionLabel("SCALE");

	edited |= ImGui::InputDouble("##Scale", &g_transformDraft.scale, 0.0001, 0.01, "%.8f");
	ImGui::PopItemWidth();

	if (edited)
	{
		if (rotationEdited)
		{
			g_transformDraft.rotationEdited = true;
			if (g_transformDraft.rotationEuler.allFinite())
			{
				g_transformDraft.rotationQ = CalibrationContext::RebuildRotationFromEuler(
					g_transformDraft.rotationEuler);
			}
			else
			{
				g_transformDraft.rotationQ.coeffs().setConstant(
					std::numeric_limits<double>::quiet_NaN());
			}
		}

		Eigen::Vector3d candidateTranslation = g_transformDraft.translationCm * 0.01;
		g_transformDraft.valid = questcal::IsValidCalibrationTransform(
			g_transformDraft.rotationQ,
			candidateTranslation, g_transformDraft.scale);
		for (const auto &anchor : CalCtx.fieldAnchors)
		{
			g_transformDraft.valid = g_transformDraft.valid &&
				questcal::IsValidFieldAnchor(anchor.position, anchor.rotation,
					anchor.translationMeters, g_transformDraft.rotationQ,
					candidateTranslation);
		}
	}

	if (!g_transformDraft.valid)
	{
		ImGui::Spacing();
		ImGui::TextColored(Pal::Bad,
			"Invalid transform or field-anchor delta: enter bounded finite values. The profile remains unchanged.");
	}
	return g_transformDraft.valid;
}

static bool SaveProfileEditorDraft()
{
	if (!g_transformDraft.valid)
		return false;

	// Persistence validates and writes a narrow profile candidate before
	// touching the live transform. Translation/scale-only edits keep the exact
	// quaternion bits; Euler conversion occurs only after a rotation edit.
	return SaveProfileTransformEdit(CalCtx, g_transformDraft.rotationQ,
		g_transformDraft.translationCm * 0.01, g_transformDraft.scale,
		g_transformDraft.rotationEdited);
}

// ---------------------------------------------------------------------------
// VR state
// ---------------------------------------------------------------------------

// How many trackers -uipreview-many fabricates. The only statement of the
// count: enough to push a tracking system's device list past BuildDeviceList's
// four-row scroll threshold, with room to fan out across battery levels.
static const int kPreviewManyTrackerCount = 6;

// Preview only: point fake devices at real SteamVR icon files when the local
// install has them (vector fallbacks otherwise).
static std::string PreviewIconPath(const char *driverRelative)
{
	static std::string base;
	if (base.empty())
	{
		char buf[MAX_PATH] = {};
		DWORD len = sizeof buf;
		if (RegGetValueA(HKEY_CURRENT_USER, "Software\\Valve\\Steam", "SteamPath", RRF_RT_REG_SZ, nullptr, buf, &len) == ERROR_SUCCESS)
			base = std::string(buf) + "\\steamapps\\common\\SteamVR\\drivers\\";
		else
			base = "C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR\\drivers\\";
	}
	std::string path = base + driverRelative;
	return FileExists(path) ? path : std::string();
}

VRState LoadVRState()
{
	VRState state;
	state.trackingSystems.reserve(vr::k_unMaxTrackedDeviceCount);
	state.devices.reserve(vr::k_unMaxTrackedDeviceCount);

	if (g_uiPreviewMode)
	{
		state.trackingSystems = { "oculus", "lighthouse" };

		VRDevice hmd;
		hmd.id = 0;
		hmd.deviceClass = vr::TrackedDeviceClass_HMD;
		hmd.model = "Meta Quest Pro";
		hmd.serial = "1PASH5D1P17365";
		hmd.trackingSystem = "oculus";
		hmd.iconPath = PreviewIconPath("oculus\\resources\\icons\\quest_headset_ready_2x.png");
		state.devices.push_back(hmd);

		VRDevice right;
		right.id = 1;
		right.deviceClass = vr::TrackedDeviceClass_Controller;
		right.model = "Knuckles Right";
		right.serial = "LHR-A3C36EA5";
		right.trackingSystem = "lighthouse";
		right.controllerRole = vr::TrackedControllerRole_RightHand;
		right.battery = 0.80f;
		right.iconPath = PreviewIconPath("indexcontroller\\resources\\icons\\right_controller_status_ready_2x.png");
		state.devices.push_back(right);

		VRDevice left;
		left.id = 2;
		left.deviceClass = vr::TrackedDeviceClass_Controller;
		left.model = "Knuckles Left";
		left.serial = "LHR-841C98C3";
		left.trackingSystem = "lighthouse";
		left.controllerRole = vr::TrackedControllerRole_LeftHand;
		left.battery = 0.12f;
		left.iconPath = PreviewIconPath("indexcontroller\\resources\\icons\\left_controller_status_ready_low_2x.png");
		state.devices.push_back(left);

		int trackerCount = g_uiPreviewMany ? kPreviewManyTrackerCount : 1;
		for (int i = 0; i < trackerCount; ++i)
		{
			VRDevice tracker;
			tracker.id = 3 + i;
			tracker.deviceClass = vr::TrackedDeviceClass_GenericTracker;
			tracker.model = "VIVE Tracker 3.0";
			char serial[32];
			snprintf(serial, sizeof serial, "LHR-77E5A2%02X", 0x11 + i);
			tracker.serial = serial;
			tracker.trackingSystem = "lighthouse";
			// Demo the row states: the lone tracker sits disconnected; with
			// -uipreview-many they fan out across battery levels instead, the
			// last one staying disconnected to keep that row state on screen.
			tracker.connected = g_uiPreviewMany ? (i != kPreviewManyTrackerCount - 1) : false;
			if (tracker.connected)
				tracker.battery = 0.95f - 0.12f * (float)i;
			const char *art =
				!tracker.connected ? "htc\\resources\\icons\\tracker_status_off_2x.png" :
				tracker.battery < kLowBattery ? "htc\\resources\\icons\\tracker_status_ready_low_2x.png" :
				"htc\\resources\\icons\\tracker_status_ready.png"; // no 2x ready ships
			tracker.iconPath = PreviewIconPath(art);
			state.devices.push_back(tracker);
		}

		return state;
	}

	if (!vr::VRSystem())
		return state;

	auto &trackingSystems = state.trackingSystems;
	auto readStringProperty = [](uint32_t id, vr::ETrackedDeviceProperty property)
	{
		char value[vr::k_unMaxPropertyStringSize] = {};
		vr::ETrackedPropertyError error = vr::TrackedProp_Success;
		uint32_t size = vr::VRSystem()->GetStringTrackedDeviceProperty(id,
			property, value, static_cast<uint32_t>(sizeof value), &error);
		if (error != vr::TrackedProp_Success || size <= 1 || size > sizeof value ||
			value[size - 1] != '\0')
			return std::string();
		return std::string(value, size - 1);
	};

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		if (deviceClass != vr::TrackedDeviceClass_TrackingReference)
		{
			std::string system = readStringProperty(id, vr::Prop_TrackingSystemName_String);
			if (!system.empty())
			{
				auto existing = std::find(trackingSystems.begin(), trackingSystems.end(), system);
				if (existing != trackingSystems.end())
				{
					if (deviceClass == vr::TrackedDeviceClass_HMD)
					{
						trackingSystems.erase(existing);
						trackingSystems.insert(trackingSystems.begin(), system);
					}
				}
				else
				{
					trackingSystems.push_back(system);
				}

				VRDevice device;
				device.id = id;
				device.deviceClass = deviceClass;
				device.trackingSystem = system;

				device.model = readStringProperty(id, vr::Prop_ModelNumber_String);
				device.serial = readStringProperty(id, vr::Prop_SerialNumber_String);

				vr::ETrackedPropertyError roleError = vr::TrackedProp_Success;
				int32_t role = vr::VRSystem()->GetInt32TrackedDeviceProperty(
					id, vr::Prop_ControllerRoleHint_Int32, &roleError);
				if (roleError == vr::TrackedProp_Success)
					device.controllerRole = static_cast<vr::ETrackedControllerRole>(role);

				device.connected = vr::VRSystem()->IsTrackedDeviceConnected(id);

				vr::ETrackedPropertyError perr = vr::TrackedProp_Success;
				if (vr::VRSystem()->GetBoolTrackedDeviceProperty(id, vr::Prop_DeviceProvidesBatteryStatus_Bool, &perr) && perr == vr::TrackedProp_Success)
				{
					float pct = vr::VRSystem()->GetFloatTrackedDeviceProperty(id, vr::Prop_DeviceBatteryPercentage_Float, &perr);
					if (perr == vr::TrackedProp_Success)
						device.battery = pct;
					device.charging = vr::VRSystem()->GetBoolTrackedDeviceProperty(id, vr::Prop_DeviceIsCharging_Bool, &perr);
				}

				// SteamVR's own device icon, matched to state: green "ready"
				// art while connected, red low-battery art below kLowBattery,
				// gray "off" art when the device drops out. Falls back down
				// the chain when a driver doesn't ship a variant.
				// "{driver}/icons/x.png" resolves through IVRResources.
				auto resolveIcon = [&](vr::ETrackedDeviceProperty prop) -> std::string {
					std::string resource = readStringProperty(id, prop);
					if (resource.empty() || !vr::VRResources())
						return std::string();
					char fullPath[MAX_PATH] = {};
					uint32_t len = vr::VRResources()->GetResourceFullPath(
						resource.c_str(), "", fullPath, MAX_PATH);
					if (len == 0 || len >= MAX_PATH || !FileExists(fullPath))
						return std::string();
					return Prefer2x(fullPath);
				};

				if (!device.connected)
					device.iconPath = resolveIcon(vr::Prop_NamedIconPathDeviceOff_String);
				else if (device.battery >= 0.0f && device.battery < kLowBattery && !device.charging)
					device.iconPath = resolveIcon(vr::Prop_NamedIconPathDeviceAlertLow_String);
				if (device.iconPath.empty())
					device.iconPath = resolveIcon(vr::Prop_NamedIconPathDeviceReady_String);
				if (device.iconPath.empty())
					device.iconPath = resolveIcon(vr::Prop_NamedIconPathDeviceOff_String);

				state.devices.push_back(device);
			}
			else
			{
				// The Release build is a GUI binary, so stdout goes nowhere --
				// the session log is the only channel a user or a bug report can
				// read. This loader runs once a second, so report each device
				// once: the latch records what has been said, not device state.
				static bool reportedMissingSystem[vr::k_unMaxTrackedDeviceCount] = {};
				if (!reportedMissingSystem[id])
				{
					reportedMissingSystem[id] = true;
					AppendSessionLog(FormatString(
						"Device %u has no tracking system name and is not shown in the device panes", id));
				}
			}
		}
	}

	return state;
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

static const ImGuiWindowFlags bareWindowFlags =
	ImGuiWindowFlags_NoTitleBar |
	ImGuiWindowFlags_NoResize |
	ImGuiWindowFlags_NoMove;

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
	// The settings screen replaces the whole content area; keeping the device
	// panes above it buried the settings below the fold for no benefit.
	bool inSettings = (CalCtx.state == CalibrationState::None && s_showSettings);
	if (!inSettings)
	{
		BuildSpacesSection(state);
		ImGui::Spacing();
	}
	BuildMenu(state, runningInOverlay);

	ImGui::End();
}
