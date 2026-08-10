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
static void ResetTransformEditorDraft();
static void BuildMenu(const VRState &state, bool runningInOverlay);

template<typename T>
static void SaveSettingOrRestore(T &value, const T &previous)
{
	if (!SaveSettings(CalCtx))
		value = previous;
}

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

static bool LoadTextureFromFile(const char *path, GLuint *outTex, int *outW, int *outH)
{
	static bool comInit = false;
	if (!comInit)
	{
		CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
		comInit = true;
	}

	IWICImagingFactory *factory = nullptr;
	if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factory))))
		return false;

	wchar_t wpath[MAX_PATH];
	MultiByteToWideChar(CP_UTF8, 0, path, -1, wpath, MAX_PATH);

	bool ok = false;
	IWICBitmapDecoder *dec = nullptr;
	if (SUCCEEDED(factory->CreateDecoderFromFilename(wpath, nullptr, GENERIC_READ, WICDecodeMetadataCacheOnDemand, &dec)))
	{
		IWICBitmapFrameDecode *frame = nullptr;
		if (SUCCEEDED(dec->GetFrame(0, &frame)))
		{
			IWICFormatConverter *conv = nullptr;
			if (SUCCEEDED(factory->CreateFormatConverter(&conv)))
			{
				if (SUCCEEDED(conv->Initialize(frame, GUID_WICPixelFormat32bppRGBA, WICBitmapDitherTypeNone, nullptr, 0.0, WICBitmapPaletteTypeCustom)))
				{
					UINT w = 0, h = 0;
					conv->GetSize(&w, &h);
					if (w > 0 && h > 0 && w <= 1024 && h <= 1024)
					{
						std::vector<unsigned char> pixels((size_t)w * h * 4);
						if (SUCCEEDED(conv->CopyPixels(nullptr, w * 4, (UINT)pixels.size(), pixels.data())))
						{
							GLuint tex = 0;
							glGenTextures(1, &tex);
							glBindTexture(GL_TEXTURE_2D, tex);
							glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
							glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
							glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
							*outTex = tex;
							*outW = (int)w;
							*outH = (int)h;
							ok = true;
						}
					}
				}
				conv->Release();
			}
			frame->Release();
		}
		dec->Release();
	}
	factory->Release();
	return ok;
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

enum CalRating { Rating_Good = 0, Rating_Decent, Rating_Poor, Rating_VeryPoor };

static CalRating ComputeCalibrationRating()
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

	// Staleness/drift only degrade the rating when nothing is maintaining the
	// alignment; a healthy continuous loop re-measures it constantly. A frozen
	// loop (bumped mount) is itself a Poor signal.
	bool continuouslyMaintained = CalCtx.continuousEnabled && CalCtx.mountExtrinsic.valid &&
		(questcal::ContinuousAlignment::State)CalCtx.continuousState ==
			questcal::ContinuousAlignment::State::Tracking;
	if (!continuouslyMaintained)
	{
		if (CalCtx.alignment == CalibrationContext::AlignmentHealth::Aging && r < Rating_Decent)
			r = Rating_Decent;
		if (CalCtx.alignment == CalibrationContext::AlignmentHealth::Stale && r < Rating_Poor)
			r = Rating_Poor;
		if (CalCtx.driftScore >= 0.85)
			r = Rating_VeryPoor;
	}
	if (CalCtx.continuousEnabled && CalCtx.mountExtrinsic.valid &&
		(questcal::ContinuousAlignment::State)CalCtx.continuousState ==
			questcal::ContinuousAlignment::State::Frozen && r < Rating_Poor)
		r = Rating_Poor;

	return (CalRating)r;
}

static const char *RatingLabels[] = { "Good", "Decent", "Poor", "Very Poor" };

// Continuous-calibration status helpers (see ContinuousAlignment.h).
static bool ContinuousHealthy()
{
	return CalCtx.continuousEnabled && CalCtx.mountExtrinsic.valid &&
		(questcal::ContinuousAlignment::State)CalCtx.continuousState ==
			questcal::ContinuousAlignment::State::Tracking;
}

static bool ContinuousFrozen()
{
	return CalCtx.continuousEnabled && CalCtx.mountExtrinsic.valid &&
		(questcal::ContinuousAlignment::State)CalCtx.continuousState ==
			questcal::ContinuousAlignment::State::Frozen;
}

static const char *ContinuousStatusText()
{
	using CA = questcal::ContinuousAlignment;
	if (CalCtx.continuousTrackerSerial.empty())
		return "no tracker selected";
	if (!CalCtx.mountExtrinsic.valid)
		return "needs one calibration with the tracker mounted";
	switch ((CA::State)CalCtx.continuousState)
	{
	case CA::State::Tracking: return "maintaining";
	case CA::State::Coasting: return "paused -- tracker not tracking";
	case CA::State::Frozen:   return "on hold -- check the mount";
	case CA::State::Holding:  return "paused -- tracker tracking unstable";
	default:                  return "gathering";
	}
}

static ImVec4 RatingColor(CalRating r)
{
	switch (r)
	{
	case Rating_Good:   return Pal::Good;
	case Rating_Decent: return Pal::Warn;
	case Rating_Poor:   return Pal::Bad;
	default:            return Pal::VeryBad;
	}
}

static void FormatUnixAge(char *buf, size_t len, double unixTime)
{
	double now = static_cast<double>(std::time(nullptr));
	if (std::isfinite(unixTime) && unixTime > 0.0 && now > 0.0)
	{
		double seconds = now - unixTime;
		if (seconds < -300.0)
		{
			snprintf(buf, len, "time is in the future");
			return;
		}
		double hours = std::max(0.0, seconds) / 3600.0;
		if (hours < 1.0)
			snprintf(buf, len, "%d min ago", static_cast<int>(hours * 60.0));
		else if (hours < 48.0)
			snprintf(buf, len, "%.1f h ago", hours);
		else
			snprintf(buf, len, "%.0f days ago", hours / 24.0);
	}
	else
		snprintf(buf, len, "age unknown");
}

static void FormatCalibrationAge(char *buf, size_t len)
{
	FormatUnixAge(buf, len, CalCtx.calibrationUnixTime);
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
static void EnsureDeviceSelection(const VRState &state, int &selected, const std::string &system)
{
	if (selected != -1)
	{
		bool matched = false;
		for (auto &device : state.devices)
			if (device.trackingSystem == system && selected == device.id)
			{
				matched = true;
				break;
			}
		if (!matched)
			selected = -1;
	}

	if (selected == -1)
	{
		for (auto &device : state.devices)
			if (device.trackingSystem == system && device.controllerRole == vr::TrackedControllerRole_LeftHand)
			{
				selected = device.id;
				break;
			}
	}

	if (selected == -1)
	{
		for (auto &device : state.devices)
			if (device.trackingSystem == system)
			{
				selected = device.id;
				break;
			}
	}
}

static void BuildDeviceList(const VRState &state, int &selected, const std::string &system, float paneW)
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
		if (DeviceRow(device, selected == device.id, rowW, i == 0, i + 1 == devices.size()))
			selected = device.id;
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
	int currentReferenceSystem = -1;
	int currentTargetSystem = -1;
	int firstReferenceSystemNotTargetSystem = -1;

	std::vector<std::string> referenceRaw, referenceDisp;
	referenceRaw.reserve(state.trackingSystems.size());
	referenceDisp.reserve(state.trackingSystems.size());
	for (auto &str : state.trackingSystems)
	{
		if (str == CalCtx.pendingReferenceTrackingSystem)
			currentReferenceSystem = (int)referenceRaw.size();
		else if (firstReferenceSystemNotTargetSystem == -1 && str != CalCtx.pendingTargetTrackingSystem)
			firstReferenceSystemNotTargetSystem = (int)referenceRaw.size();
		referenceRaw.push_back(str);
		referenceDisp.push_back(FriendlySystemName(str));
	}

	if (currentReferenceSystem == -1)
		currentReferenceSystem = firstReferenceSystemNotTargetSystem;

	float cw = ImGui::GetWindowContentRegionWidth();
	const float paneGap = 24.0f;
	float paneW = (cw - paneGap) * 0.5f;

	ImVec2 top = ImGui::GetCursorScreenPos();

	// ---- Left pane: reference space ----
	ImGui::SetCursorScreenPos(top);
	ImGui::BeginGroup();
	SectionLabel("REFERENCE SPACE");
	std::vector<const char *> referenceItems;
	referenceItems.reserve(referenceDisp.size());
	for (auto &s : referenceDisp)
		referenceItems.push_back(s.c_str());
	ImGui::PushItemWidth(paneW);
	ImGui::Combo("##ReferenceTrackingSystem", &currentReferenceSystem, referenceItems.data(), (int)referenceItems.size());
	ImGui::PopItemWidth();

	if (currentReferenceSystem != -1 && currentReferenceSystem < (int)referenceRaw.size())
	{
		CalCtx.pendingReferenceTrackingSystem = referenceRaw[currentReferenceSystem];
		if (CalCtx.pendingReferenceTrackingSystem == CalCtx.pendingTargetTrackingSystem)
			CalCtx.pendingTargetTrackingSystem = "";
	}

	static int selectedRefDevice = -1;
	ImGui::Spacing();
	BuildDeviceList(state, selectedRefDevice, CalCtx.pendingReferenceTrackingSystem, paneW);
	CalCtx.referenceID = selectedRefDevice;
	ImGui::EndGroup();
	float leftBottom = ImGui::GetItemRectMax().y;

	// ---- Right pane: target space ----
	if (CalCtx.pendingTargetTrackingSystem == "")
		currentTargetSystem = 0;

	std::vector<std::string> targetRaw, targetDisp;
	targetRaw.reserve(state.trackingSystems.size());
	targetDisp.reserve(state.trackingSystems.size());
	for (auto &str : state.trackingSystems)
	{
		if (str != CalCtx.pendingReferenceTrackingSystem)
		{
			if (str != "" && str == CalCtx.pendingTargetTrackingSystem)
				currentTargetSystem = (int)targetRaw.size();
			targetRaw.push_back(str);
			targetDisp.push_back(FriendlySystemName(str));
		}
	}

	ImGui::SetCursorScreenPos(ImVec2(top.x + paneW + paneGap, top.y));
	ImGui::BeginGroup();
	SectionLabel("TARGET SPACE");
	if (!targetRaw.empty())
	{
		std::vector<const char *> targetItems;
		targetItems.reserve(targetDisp.size());
		for (auto &s : targetDisp)
			targetItems.push_back(s.c_str());
		ImGui::PushItemWidth(paneW);
		ImGui::Combo("##TargetTrackingSystem", &currentTargetSystem, targetItems.data(), (int)targetItems.size());
		ImGui::PopItemWidth();

		if (currentTargetSystem != -1 && currentTargetSystem < (int)targetRaw.size())
			CalCtx.pendingTargetTrackingSystem = targetRaw[currentTargetSystem];

		static int selectedCalDevice = -1;
		ImGui::Spacing();
		BuildDeviceList(state, selectedCalDevice, CalCtx.pendingTargetTrackingSystem, paneW);
		CalCtx.targetID = selectedCalDevice;
	}
	else
	{
		ImGui::TextDisabled("No second tracking system detected");
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
		CalCtx.messages.clear();
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
		if (CalCtx.validProfile && !CalCtx.enabled)
		{
			std::vector<StatusRowData> warn;
			warn.push_back({ IconInfo, Pal::Bad,
				CalCtx.profileUniverseUnsafe
					? "Raw tracking universe changed while monitoring was offline -- recalibrate before using this profile"
					: FormatString("%s headset not detected -- profile disabled",
						FriendlySystemName(CalCtx.referenceTrackingSystem).c_str()) });
			DrawStatusCard(warn);
			ImGui::Spacing();
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
		if (!CalCtx.uiAdvanced)
		{
			// Quiet strip anchored just above the footer: no card, a rating-colored
			// dot, and the recommendation on its own violet line when it matters.
			CalRating rating = CalCtx.validProfile ? ComputeCalibrationRating() : Rating_Good;
			bool nudge = CalCtx.validProfile && rating >= Rating_Poor;
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
				dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(x, ty), Pal::U32(Pal::Text), "Tracking quality: ");
				x += ImGui::CalcTextSize("Tracking quality: ").x;
				dl->AddText(g_fontBody, g_fontBody->FontSize, ImVec2(x, ty), Pal::U32(rcol), RatingLabels[rating]);
				x += ImGui::CalcTextSize(RatingLabels[rating]).x;

				std::string ageLine;
				if (ContinuousHealthy())
				{
					if (CalCtx.autoCorrectionsApplied > 0 && CalCtx.lastAutoCorrectionUnixTime > 0.0)
					{
						char adj[64];
						FormatUnixAge(adj, sizeof adj, CalCtx.lastAutoCorrectionUnixTime);
						ageLine = FormatString("maintained continuously -- last adjusted %s", adj);
					}
					else
						ageLine = "maintained continuously";
				}
				else
				{
					char age[64];
					FormatCalibrationAge(age, sizeof age);
					ageLine = (strcmp(age, "age unknown") == 0)
						? std::string("calibration time unknown")
						: FormatString("calibrated %s", age);
				}
				dl->AddText(g_fontSmall, g_fontSmall->FontSize,
					ImVec2(x + 18.0f, cy - g_fontSmall->FontSize * 0.5f + 2.0f),
					Pal::U32(Pal::Faint), ageLine.c_str());

				if (nudge)
					dl->AddText(g_fontBody, g_fontBody->FontSize,
						ImVec2(p.x, ty + lineH + 4.0f),
						Pal::U32(Pal::Violet), ContinuousFrozen()
							? "Check the headset-mounted tracker -- recalibrating is recommended."
							: "Recalibrating is recommended.");
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

				char age[64];
				FormatCalibrationAge(age, sizeof age);

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

				if (CalCtx.continuousEnabled)
				{
					using CAState = questcal::ContinuousAlignment::State;
					CAState cs = (CAState)CalCtx.continuousState;
					ImVec4 col = ContinuousHealthy() ? Pal::Good :
						cs == CAState::Frozen ? Pal::Bad : Pal::Warn;
					std::string line;
					if (ContinuousHealthy() && CalCtx.continuousDeviation.valid)
						line = FormatString(
							"Continuous: maintaining -- scatter %.2f deg / %.1f cm, deviation %.2f deg / %.1f cm, %u corrections",
							CalCtx.continuousScatterRotDeg, CalCtx.continuousScatterPosM * 100.0,
							CalCtx.continuousDeviation.yawDeg + CalCtx.continuousDeviation.tiltDeg,
							CalCtx.continuousDeviation.posM * 100.0,
							CalCtx.autoCorrectionsApplied);
					else
						line = FormatString("Continuous: %s -- %u corrections applied",
							ContinuousStatusText(), CalCtx.autoCorrectionsApplied);
					rows.push_back({ IconCrosshair, col, line });
				}

				if (CalCtx.alignment == CalibrationContext::AlignmentHealth::Stale &&
					!ContinuousHealthy())
					rows.push_back({ IconInfo, Pal::Violet, "Recalibrating is recommended." });
				if (ContinuousFrozen())
					rows.push_back({ IconInfo, Pal::Violet,
						"Check the headset-mounted tracker -- recalibrating is recommended." });
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
			ImVec2 p = BeginRowCard(52.0f);
			ImGui::SetCursorScreenPos(ImVec2(p.x + 16.0f, p.y + 14.0f));
			bool previous = CalCtx.uiAdvanced;
			if (QCCheckbox("##uiAdvanced", &CalCtx.uiAdvanced))
				SaveSettingOrRestore(CalCtx.uiAdvanced, previous);
			RowIconLabel(p, IconGauge, "Advanced mode (show raw calibration and drift stats)");
			EndRowCard(p, 52.0f);
		}

		// Poor calibration notification
		{
			ImVec2 p = BeginRowCard(52.0f);
			ImGui::SetCursorScreenPos(ImVec2(p.x + 16.0f, p.y + 14.0f));
			bool previous = CalCtx.notifyPoorCalibration;
			if (QCCheckbox("##notifyPoorCalibration", &CalCtx.notifyPoorCalibration))
				SaveSettingOrRestore(CalCtx.notifyPoorCalibration, previous);
			if (ImGui::IsItemHovered())
			{
				ImGui::SetTooltip(
					"Show a SteamVR notification when alignment monitoring recommends\n"
					"recalibration. The status and session log still update when disabled.");
			}
			RowIconLabel(p, IconInfo, "Poor calibration notification");
			EndRowCard(p, 52.0f);
		}

		// Spatial correction field
		if (CalCtx.validProfile)
		{
			size_t anchorCount = CalCtx.fieldAnchors.size();
			float rowH = 52.0f + (anchorCount > 0 ? anchorCount * 24.0f + 6.0f : 0.0f);
			ImVec2 p = BeginRowCard(rowH);

			ImGui::SetCursorScreenPos(ImVec2(p.x + 16.0f, p.y + 14.0f));
			if (QCCheckbox("##fieldEnabled", &CalCtx.fieldEnabled))
			{
				bool enabled = !CalCtx.fieldEnabled;
				uint32_t generation = CalCtx.fieldGeneration;
				CalCtx.fieldGeneration++;
				if (!SaveProfile(CalCtx))
				{
					CalCtx.fieldEnabled = enabled;
					CalCtx.fieldGeneration = generation;
				}
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
				ImGui::SetCursorScreenPos(ImVec2(p.x + cw - 16.0f - btnW, p.y + 9.0f));
				if (IconButton("clearanchors", "Clear anchors", IconTrash, ImVec2(btnW, 34.0f), BtnKind::Ghost))
				{
					auto anchors = CalCtx.fieldAnchors;
					uint32_t generation = CalCtx.fieldGeneration;
					CalCtx.fieldAnchors.clear();
					CalCtx.fieldGeneration++;
					if (!SaveProfile(CalCtx))
					{
						CalCtx.fieldAnchors = std::move(anchors);
						CalCtx.fieldGeneration = generation;
					}
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
						ImVec2(p.x + 92.0f, p.y + 52.0f + i * 24.0f), Pal::U32(Pal::Dim), line.c_str());
				}
			}

			EndRowCard(p, rowH);
		}

		// Solve playspace scale
		{
			ImVec2 p = BeginRowCard(52.0f);
			ImGui::SetCursorScreenPos(ImVec2(p.x + 16.0f, p.y + 14.0f));
			bool previous = CalCtx.solveScale;
			if (QCCheckbox("##solveScale", &CalCtx.solveScale))
				SaveSettingOrRestore(CalCtx.solveScale, previous);
			RowIconLabel(p, IconScale, "Solve playspace scale (experimental)");
			EndRowCard(p, 52.0f);
		}

		// Time offset + nested manual override
		{
			const float rowH = 104.0f;
			ImVec2 p = BeginRowCard(rowH);
			ImDrawList *dl = ImGui::GetWindowDrawList();

			ImGui::SetCursorScreenPos(ImVec2(p.x + 16.0f, p.y + 14.0f));
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
					ImVec2(p.x + cw - 16.0f - ts.x, p.y + 26.0f - g_fontBody->FontSize * 0.5f),
					Pal::U32(Pal::Faint), applied.c_str());
			}

			// Nested inset: manual override spike tool (verifies the poseTimeOffset
			// sign convention against a live session; bypasses the solved value).
			ImVec2 np = ImVec2(p.x + 12.0f, p.y + 52.0f);
			ImVec2 nb = ImVec2(p.x + cw - 12.0f, p.y + rowH - 10.0f);
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

			EndRowCard(p, rowH);
		}

		// Continuous calibration: enable + nested tracker pick / hide / latency
		if (CalCtx.validProfile)
		{
			bool advisory = CalCtx.continuousEnabled && !CalCtx.mountExtrinsic.valid;
			const float nestedH = CalCtx.continuousEnabled ? 116.0f : 0.0f;
			const float advisoryH = advisory ? 26.0f : 0.0f;
			float rowH = 52.0f + nestedH + advisoryH;
			ImVec2 p = BeginRowCard(rowH);
			ImDrawList *dl = ImGui::GetWindowDrawList();

			ImGui::SetCursorScreenPos(ImVec2(p.x + 16.0f, p.y + 14.0f));
			bool previousContinuousEnabled = CalCtx.continuousEnabled;
			if (QCCheckbox("##continuousEnabled", &CalCtx.continuousEnabled) &&
				!SaveProfile(CalCtx))
				CalCtx.continuousEnabled = previousContinuousEnabled;
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
				std::string status = ContinuousStatusText();
				ImVec2 ts = ImGui::CalcTextSize(status.c_str());
				dl->AddText(g_fontBody, g_fontBody->FontSize,
					ImVec2(p.x + cw - 16.0f - ts.x, p.y + 26.0f - g_fontBody->FontSize * 0.5f),
					Pal::U32(Pal::Faint), status.c_str());

				// Nested inset: tracker pick, game visibility, opt-in latency.
				ImVec2 np = ImVec2(p.x + 12.0f, p.y + 52.0f);
				ImVec2 nb = ImVec2(p.x + cw - 12.0f, p.y + 52.0f + nestedH - 8.0f);
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
						std::string previousSerial = CalCtx.continuousTrackerSerial;
						auto previousExtrinsic = CalCtx.mountExtrinsic;
						CalCtx.continuousTrackerSerial = candidates[sel]->serial;
						// The learned mount offset described the previous tracker;
						// a different physical device needs a fresh calibration.
						CalCtx.mountExtrinsic = questcal::MountExtrinsic();
						if (!SaveProfile(CalCtx))
						{
							CalCtx.continuousTrackerSerial = std::move(previousSerial);
							CalCtx.mountExtrinsic = previousExtrinsic;
						}
					}
				}
				ImGui::PopStyleVar();
				ImGui::PopItemWidth();

				ImGui::SetCursorScreenPos(ImVec2(np.x + 12.0f, np.y + 40.0f));
				bool previousHide = CalCtx.hideMountedTracker;
				if (QCCheckbox("##hideTracker", &CalCtx.hideMountedTracker) &&
					!SaveProfile(CalCtx))
					CalCtx.hideMountedTracker = previousHide;
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
				bool previousLatency = CalCtx.continuousLatencyReestimation;
				if (QCCheckbox("##contLatency", &CalCtx.continuousLatencyReestimation) &&
					!SaveProfile(CalCtx))
					CalCtx.continuousLatencyReestimation = previousLatency;
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
					ImVec2(p.x + 16.0f, p.y + 52.0f + nestedH + 2.0f),
					Pal::U32(Pal::Violet),
					"Mount a spare tracker firmly on the headset, then calibrate using the headset as the reference device.");
			}

			EndRowCard(p, rowH);
		}

		// Chaperone: auto-restore arm/disarm + manual restore + snapshot info
		if (CalCtx.chaperone.valid)
		{
			const float rowH = 76.0f;
			ImVec2 p = BeginRowCard(rowH);
			ImGui::SetCursorScreenPos(ImVec2(p.x + 16.0f, p.y + 14.0f));
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
				char age[64];
				FormatUnixAge(age, sizeof age, CalCtx.chaperone.copyUnixTime);
				info = FormatString("Snapshot: %zu wall segment(s), %.1f x %.1f m play area, saved %s",
					CalCtx.chaperone.geometry.size(),
					CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1], age);
			}
			ImGui::GetWindowDrawList()->AddText(g_fontSmall, g_fontSmall->FontSize,
				ImVec2(p.x + 92.0f, p.y + 46.0f), Pal::U32(Pal::Dim), info.c_str());

			float btnW = 224.0f;
			ImGui::SetCursorScreenPos(ImVec2(p.x + cw - 16.0f - btnW, p.y + 9.0f));
			if (IconButton("pastechap", "Restore chaperone now", IconCopy, ImVec2(btnW, 34.0f), BtnKind::Ghost))
				ApplyChaperoneBounds();
			EndRowCard(p, rowH);
		}

		// Raw transform editor -- power users only.
		if (CalCtx.validProfile && !CalCtx.profileUniverseUnsafe)
		{
			ImGui::Spacing();
			if (IconButton("edit", "Edit calibration (advanced)", IconPencil, ImVec2(cw, 46.0f), BtnKind::Ghost))
			{
				ResetTransformEditorDraft();
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
			if (SaveProfileEditorDraft())
			{
				ResetTransformEditorDraft();
			}
		}
		ImGui::SameLine(0.0f, gap);
		if (IconButton("cancelprofile", "Cancel", nullptr,
			ImVec2(cancelWidth, 52.0f), BtnKind::Ghost))
		{
			ResetTransformEditorDraft();
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
	bool active = false;
	bool valid = true;
	bool rotationEdited = false;
	Eigen::Quaterniond rotationQ{ 1, 0, 0, 0 };
	Eigen::Vector3d rotationEuler{ 0, 0, 0 };
	Eigen::Vector3d translationCm{ 0, 0, 0 };
	double scale = 1.0;
};

static TransformEditorDraft g_transformDraft;

static void ResetTransformEditorDraft()
{
	g_transformDraft.active = false;
}

static bool BuildProfileEditor()
{
	if (!g_transformDraft.active)
	{
		g_transformDraft.active = true;
		g_transformDraft.rotationQ = CalCtx.calibratedRotationQ;
		g_transformDraft.rotationEuler = CalCtx.calibratedRotation;
		g_transformDraft.translationCm = CalCtx.calibratedTranslation;
		g_transformDraft.scale = CalCtx.calibratedScale;
		g_transformDraft.rotationEdited = false;
		g_transformDraft.valid = true;
	}

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
				Eigen::Vector3d radians = g_transformDraft.rotationEuler * EIGEN_PI / 180.0;
				g_transformDraft.rotationQ =
					Eigen::AngleAxisd(radians(0), Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd(radians(1), Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd(radians(2), Eigen::Vector3d::UnitX());
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
	if (!g_transformDraft.active || !g_transformDraft.valid)
		return false;

	// Persistence validates and writes a narrow profile candidate before
	// touching the live transform. Translation/scale-only edits keep the exact
	// quaternion bits; Euler conversion occurs only after a rotation edit.
	if (!SaveProfileTransformEdit(CalCtx, g_transformDraft.rotationQ,
		g_transformDraft.translationCm * 0.01, g_transformDraft.scale,
		g_transformDraft.rotationEdited))
		return false;
	return true;
}

// ---------------------------------------------------------------------------
// VR state
// ---------------------------------------------------------------------------

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

		int trackerCount = g_uiPreviewMany ? 6 : 1;
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
			// -uipreviewmany they fan out across battery levels instead.
			tracker.connected = g_uiPreviewMany ? (i != 5) : false;
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
				printf("failed to get tracking system name for id %d\n", id);
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
