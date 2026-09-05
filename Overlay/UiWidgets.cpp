// Palette, theme, icon glyphs, textures and the hand-painted widgets.
#include "stdafx.h"
#include "UiInternal.h"

// ---------------------------------------------------------------------------
// Palette + theme
// ---------------------------------------------------------------------------

void LinkText(const char *label, const char *url)
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
	// The window is a fixed 1200x800 and the settings screen overflows it, so
	// the scrollbar is the only sign that more exists below the fold. It has
	// to be visible at rest.
	c[ImGuiCol_ScrollbarBg]          = ImVec4(1, 1, 1, 0.04f);
	c[ImGuiCol_ScrollbarGrab]        = ImVec4(1, 1, 1, 0.40f);  // 3.4:1 on the track
	c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(1, 1, 1, 0.50f);
	c[ImGuiCol_ScrollbarGrabActive]  = ImVec4(1, 1, 1, 0.60f);
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
	// Dark enough that the coloured status rows behind a modal stop competing
	// with it for attention.
	c[ImGuiCol_ModalWindowDarkening] = ImVec4(0, 0, 0, 0.80f);
	c[ImGuiCol_NavHighlight]         = Pal::Accent;
}

// ---------------------------------------------------------------------------
// Small drawing helpers
// ---------------------------------------------------------------------------

float LetterSpacedWidth(ImFont *font, const char *text, float spacing)
{
	float w = 0.0f;
	for (const char *p = text; *p; ++p)
	{
		const ImFontGlyph *g = font->FindGlyph((ImWchar)(unsigned char)*p);
		w += (g ? g->AdvanceX : font->FontSize * 0.5f) + spacing;
	}
	return w > 0.0f ? w - spacing : 0.0f;
}

// Labels must be ASCII; non-ASCII text needs UTF-8 decoding before glyph lookup.
void LetterSpacedTextAt(ImDrawList *dl, ImFont *font, ImVec2 pos, ImU32 col, const char *text, float spacing)
{
	float x = pos.x;
	char buf[2] = { 0, 0 };
	for (const char *p = text; *p; ++p)
	{
		buf[0] = *p;
		dl->AddText(font, font->FontSize, ImVec2(x, pos.y), col, buf);
		const ImFontGlyph *g = font->FindGlyph((ImWchar)(unsigned char)*p);
		x += (g ? g->AdvanceX : font->FontSize * 0.5f) + spacing;
	}
}

// Section label ("REFERENCE SPACE") as an inline widget.
void SectionLabel(const char *text)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImVec2 p = ImGui::GetCursorScreenPos();
	LetterSpacedTextAt(dl, g_fontSmall, p, Pal::U32(Pal::Dim), text, 2.0f);
	ImGui::Dummy(ImVec2(LetterSpacedWidth(g_fontSmall, text, 2.0f), g_fontSmall->FontSize + 4.0f));
}

// ---------------------------------------------------------------------------
// Icons (line style, s = half-extent in pixels)
// ---------------------------------------------------------------------------

void IconLogo(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.78f, col, 24, 2.2f);
	dl->AddCircleFilled(c, s * 0.26f, col, 12);
}

void IconHMD(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	ImVec2 a = ImVec2(c.x - s, c.y - s * 0.60f);
	ImVec2 b = ImVec2(c.x + s, c.y + s * 0.44f);
	dl->PathRect(a, b, s * 0.34f);
	dl->PathStroke(col, true, 2.2f);
	dl->AddCircleFilled(ImVec2(c.x - s * 0.42f, c.y - s * 0.06f), s * 0.16f, col, 12);
	dl->AddCircleFilled(ImVec2(c.x + s * 0.42f, c.y - s * 0.06f), s * 0.16f, col, 12);
}

void IconController(ImDrawList *dl, ImVec2 c, float s, ImU32 col, bool leftHand)
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

void IconTracker(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.68f, col, 24, 2.2f);
	dl->AddCircleFilled(c, s * 0.15f, col, 10);
	for (int i = 0; i < 3; ++i)
	{
		float a = -IM_PI * 0.5f + (float)i * (2.0f * IM_PI / 3.0f);
		dl->AddCircleFilled(ImVec2(c.x + cosf(a) * s * 0.40f, c.y + sinf(a) * s * 0.40f), s * 0.09f, col, 8);
	}
}

void IconPlay(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddTriangleFilled(
		ImVec2(c.x - s * 0.42f, c.y - s * 0.62f),
		ImVec2(c.x - s * 0.42f, c.y + s * 0.62f),
		ImVec2(c.x + s * 0.66f, c.y), col);
}

void IconPencil(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	// Heavier body and a longer tip than the other glyphs: at row size a thin
	// diagonal read as a stray stroke, not a pencil.
	ImVec2 tip = ImVec2(c.x - s * 0.62f, c.y + s * 0.62f);
	ImVec2 top = ImVec2(c.x + s * 0.50f, c.y - s * 0.50f);
	dl->AddLine(ImVec2(tip.x + s * 0.30f, tip.y - s * 0.10f), top, col, 3.0f);
	dl->AddTriangleFilled(tip,
		ImVec2(tip.x + s * 0.40f, tip.y - s * 0.04f),
		ImVec2(tip.x + s * 0.04f, tip.y - s * 0.40f), col);
	dl->AddLine(ImVec2(top.x - s * 0.34f, top.y - s * 0.02f), ImVec2(top.x - s * 0.02f, top.y + s * 0.34f), col, 2.2f);
}

void IconTrash(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddLine(ImVec2(c.x - s * 0.62f, c.y - s * 0.36f), ImVec2(c.x + s * 0.62f, c.y - s * 0.36f), col, 2.2f);
	dl->AddLine(ImVec2(c.x - s * 0.20f, c.y - s * 0.60f), ImVec2(c.x + s * 0.20f, c.y - s * 0.60f), col, 2.2f);
	dl->PathRect(ImVec2(c.x - s * 0.45f, c.y - s * 0.36f), ImVec2(c.x + s * 0.45f, c.y + s * 0.62f), s * 0.16f, ImDrawCornerFlags_Bot);
	dl->PathStroke(col, true, 2.0f);
	dl->AddLine(ImVec2(c.x - s * 0.15f, c.y - s * 0.12f), ImVec2(c.x - s * 0.15f, c.y + s * 0.36f), col, 1.8f);
	dl->AddLine(ImVec2(c.x + s * 0.15f, c.y - s * 0.12f), ImVec2(c.x + s * 0.15f, c.y + s * 0.36f), col, 1.8f);
}

void IconCopy(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->PathRect(ImVec2(c.x - s * 0.62f, c.y - s * 0.62f), ImVec2(c.x + s * 0.14f, c.y + s * 0.14f), s * 0.14f);
	dl->PathStroke(col, true, 2.0f);
	dl->PathRect(ImVec2(c.x - s * 0.14f, c.y - s * 0.14f), ImVec2(c.x + s * 0.62f, c.y + s * 0.62f), s * 0.14f);
	dl->PathStroke(col, true, 2.0f);
}

void IconCrosshair(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.55f, col, 24, 2.0f);
	dl->AddCircleFilled(c, s * 0.12f, col, 8);
	dl->AddLine(ImVec2(c.x, c.y - s * 0.90f), ImVec2(c.x, c.y - s * 0.55f), col, 2.0f);
	dl->AddLine(ImVec2(c.x, c.y + s * 0.55f), ImVec2(c.x, c.y + s * 0.90f), col, 2.0f);
	dl->AddLine(ImVec2(c.x - s * 0.90f, c.y), ImVec2(c.x - s * 0.55f, c.y), col, 2.0f);
	dl->AddLine(ImVec2(c.x + s * 0.55f, c.y), ImVec2(c.x + s * 0.90f, c.y), col, 2.0f);
}

void IconCheck(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	ImVec2 pts[3] = {
		ImVec2(c.x - s * 0.65f, c.y + s * 0.05f),
		ImVec2(c.x - s * 0.15f, c.y + s * 0.55f),
		ImVec2(c.x + s * 0.70f, c.y - s * 0.50f)
	};
	dl->AddPolyline(pts, 3, col, false, 2.4f);
}

void IconClock(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.85f, col, 20, 2.0f);
	dl->AddLine(c, ImVec2(c.x, c.y - s * 0.52f), col, 2.0f);
	dl->AddLine(c, ImVec2(c.x + s * 0.40f, c.y + s * 0.14f), col, 2.0f);
}

void IconDownload(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddLine(ImVec2(c.x, c.y - s * 0.72f),
		ImVec2(c.x, c.y + s * 0.30f), col, 2.2f);
	dl->AddLine(ImVec2(c.x - s * 0.42f, c.y - s * 0.02f),
		ImVec2(c.x, c.y + s * 0.38f), col, 2.2f);
	dl->AddLine(ImVec2(c.x + s * 0.42f, c.y - s * 0.02f),
		ImVec2(c.x, c.y + s * 0.38f), col, 2.2f);
	dl->AddLine(ImVec2(c.x - s * 0.66f, c.y + s * 0.68f),
		ImVec2(c.x + s * 0.66f, c.y + s * 0.68f), col, 2.2f);
}

void IconInfo(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddCircle(c, s * 0.85f, col, 20, 2.0f);
	dl->AddCircleFilled(ImVec2(c.x, c.y - s * 0.38f), s * 0.11f, col, 8);
	dl->AddLine(ImVec2(c.x, c.y - s * 0.08f), ImVec2(c.x, c.y + s * 0.42f), col, 2.2f);
}

void IconPin(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	ImVec2 hc = ImVec2(c.x, c.y - s * 0.22f);
	dl->AddCircle(hc, s * 0.42f, col, 20, 2.0f);
	dl->AddCircleFilled(hc, s * 0.11f, col, 8);
	dl->AddLine(ImVec2(hc.x - s * 0.29f, hc.y + s * 0.30f), ImVec2(c.x, c.y + s * 0.72f), col, 2.0f);
	dl->AddLine(ImVec2(hc.x + s * 0.29f, hc.y + s * 0.30f), ImVec2(c.x, c.y + s * 0.72f), col, 2.0f);
}

void IconScale(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddLine(ImVec2(c.x, c.y - s * 0.62f), ImVec2(c.x, c.y + s * 0.46f), col, 2.0f);
	dl->AddLine(ImVec2(c.x - s * 0.58f, c.y - s * 0.40f), ImVec2(c.x + s * 0.58f, c.y - s * 0.40f), col, 2.0f);
	dl->AddLine(ImVec2(c.x - s * 0.30f, c.y + s * 0.46f), ImVec2(c.x + s * 0.30f, c.y + s * 0.46f), col, 2.0f);
	dl->PathArcTo(ImVec2(c.x - s * 0.58f, c.y - s * 0.16f), s * 0.24f, 0.0f, IM_PI, 12);
	dl->PathStroke(col, false, 2.0f);
	dl->PathArcTo(ImVec2(c.x + s * 0.58f, c.y - s * 0.16f), s * 0.24f, 0.0f, IM_PI, 12);
	dl->PathStroke(col, false, 2.0f);
}

void IconGauge(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->PathArcTo(ImVec2(c.x, c.y + s * 0.25f), s * 0.72f, IM_PI, 2.0f * IM_PI, 20);
	dl->PathStroke(col, false, 2.2f);
	dl->AddLine(ImVec2(c.x, c.y + s * 0.25f), ImVec2(c.x + s * 0.38f, c.y - s * 0.22f), col, 2.2f);
	dl->AddCircleFilled(ImVec2(c.x, c.y + s * 0.25f), s * 0.12f, col, 8);
}

void IconField(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	// Anchor spot: pin over ground ripple.
	dl->AddCircle(ImVec2(c.x, c.y - s * 0.30f), s * 0.34f, col, 16, 2.0f);
	dl->AddLine(ImVec2(c.x, c.y + s * 0.04f), ImVec2(c.x, c.y + s * 0.44f), col, 2.0f);
	dl->PathArcTo(ImVec2(c.x, c.y + s * 0.44f), s * 0.55f, IM_PI * 0.15f, IM_PI * 0.85f, 12);
	dl->PathStroke(col, false, 2.0f);
}

void IconGear(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
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

static bool DecodeTexture(IWICImagingFactory *factory, IWICBitmapDecoder *decoder,
                         GLuint *outTex, int *outW, int *outH, bool guide)
{
	ComScoped<IWICBitmapFrameDecode> frame;
	if (FAILED(decoder->GetFrame(0, frame.Put())))
		return false;

	ComScoped<IWICFormatConverter> conv;
	if (FAILED(factory->CreateFormatConverter(conv.Put())))
		return false;
	if (FAILED(conv->Initialize(frame.Get(), GUID_WICPixelFormat32bppRGBA,
		WICBitmapDitherTypeNone, nullptr, 0.0, WICBitmapPaletteTypeCustom)))
		return false;

	// External device icons retain their small allocation bound. The embedded
	// guides have fixed, bounded atlas layouts for the wrist and head demos.
	UINT w = 0, h = 0;
	if (FAILED(conv->GetSize(&w, &h)) ||
		(guide ? !((w == 4608 && h == 5880) || (w == 5120 && h == 5760))
			: (w == 0 || h == 0 || w > 1024 || h > 1024)))
		return false;

	std::vector<unsigned char> pixels((size_t)w * h * 4);
	if (FAILED(conv->CopyPixels(nullptr, w * 4, (UINT)pixels.size(), pixels.data())))
		return false;

	GLuint tex = 0;
	glGenTextures(1, &tex);
	if (!tex)
		return false;
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
	GLint allocatedWidth = 0;
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &allocatedWidth);
	if (allocatedWidth != static_cast<GLint>(w))
	{
		glDeleteTextures(1, &tex);
		return false;
	}
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	*outTex = tex;
	*outW = (int)w;
	*outH = (int)h;
	return true;
}

bool LoadTextureFromFile(const char *path, GLuint *outTex, int *outW, int *outH)
{
	static const HRESULT comInit = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
	(void)comInit;
	ComScoped<IWICImagingFactory> factory;
	if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
		IID_PPV_ARGS(factory.Put()))))
		return false;
	wchar_t wpath[MAX_PATH];
	if (!MultiByteToWideChar(CP_UTF8, 0, path, -1, wpath, MAX_PATH))
		return false;
	ComScoped<IWICBitmapDecoder> decoder;
	if (FAILED(factory->CreateDecoderFromFilename(wpath, nullptr, GENERIC_READ,
		WICDecodeMetadataCacheOnDemand, decoder.Put())))
		return false;
	return DecodeTexture(factory.Get(), decoder.Get(), outTex, outW, outH, false);
}

bool LoadGuideTexture(GuideDemo demo, GLuint *outTex)
{
	const char *name = demo == GuideDemo::Mounted ? "GUIDE_HEADSET"
		: demo == GuideDemo::HeadsetContact ? "GUIDE_CONTACT" : "GUIDE_HANDHELD";
	HRSRC resource = FindResourceA(nullptr, name, MAKEINTRESOURCEA(10));
	if (!resource)
		return false;
	HGLOBAL loaded = LoadResource(nullptr, resource);
	auto bytes = static_cast<BYTE *>(LockResource(loaded));
	DWORD size = SizeofResource(nullptr, resource);
	if (!bytes || size == 0)
		return false;
	static const HRESULT comInit = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
	(void)comInit;
	ComScoped<IWICImagingFactory> factory;
	if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
		IID_PPV_ARGS(factory.Put()))))
		return false;
	ComScoped<IWICStream> stream;
	if (FAILED(factory->CreateStream(stream.Put())) ||
		FAILED(stream->InitializeFromMemory(bytes, size)))
		return false;
	ComScoped<IWICBitmapDecoder> decoder;
	if (FAILED(factory->CreateDecoderFromStream(stream.Get(), nullptr,
		WICDecodeMetadataCacheOnDemand, decoder.Put())))
		return false;
	int width = 0, height = 0;
	return DecodeTexture(factory.Get(), decoder.Get(), outTex, &width, &height, true);
}

const std::string &GuideModelCredits()
{
	static const std::string credits = []() -> std::string {
		HRSRC resource = FindResourceA(nullptr, "GUIDE_CREDITS", MAKEINTRESOURCEA(10));
		if (resource)
		{
			const char *bytes = static_cast<const char *>(LockResource(LoadResource(nullptr, resource)));
			if (bytes)
				return std::string(bytes, SizeofResource(nullptr, resource));
		}
		return "Motion demo credits couldn't load. Reinstall QuestCalibrator to restore them.";
	}();
	return credits;
}

// Keyed by absolute path; loaded lazily on the render thread (GL context current).
static std::map<std::string, DeviceIconTex> s_deviceIconCache;

const DeviceIconTex *GetDeviceIconTex(const std::string &path)
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

bool FileExists(const std::string &path)
{
	DWORD attrs = GetFileAttributesA(path.c_str());
	return attrs != INVALID_FILE_ATTRIBUTES && !(attrs & FILE_ATTRIBUTE_DIRECTORY);
}

// SteamVR ships icons in 1x and 2x; prefer the sharper one when present.
std::string Prefer2x(const std::string &path)
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

// Keyboard focus, drawn the same way on every hand-painted control: the
// navigation highlight ImGui would draw on its own widgets never reaches an
// InvisibleButton.
void DrawFocusRing(ImDrawList *dl, ImVec2 a, ImVec2 b, float rounding)
{
	if (ImGui::IsItemFocused())
		dl->AddRect(ImVec2(a.x - 2.0f, a.y - 2.0f), ImVec2(b.x + 2.0f, b.y + 2.0f),
			Pal::U32(Pal::Accent), rounding + 2.0f, ImDrawCornerFlags_All, 2.0f);
}

bool IconButton(const char *id, const char *label, IconFn icon, ImVec2 size, BtnKind kind, bool smallCaps)
{
	ImVec2 p = ImGui::GetCursorScreenPos();
	bool pressed = ImGui::InvisibleButton(id, size);
	bool hov = ImGui::IsItemHovered();
	bool act = ImGui::IsItemActive();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	DrawFocusRing(dl, p, ImVec2(p.x + size.x, p.y + size.y), 10.0f);

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
	else if (kind == BtnKind::Danger)
	{
		bg = act ? Pal::Inset : (hov ? Pal::CardHov : Pal::Card);
		txt = Pal::VeryBad;
	}
	else
	{
		bg = act ? Pal::Inset : (hov ? Pal::CardHov : Pal::Card);
		txt = hov ? Pal::Text : (smallCaps ? Pal::Dim : Pal::Text);
	}

	if (bg.w > 0.0f)
		dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), Pal::U32(bg), 10.0f);
	if (kind == BtnKind::Ghost)
		dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), Pal::U32(hov ? Pal::Text : Pal::GhostBorder), 10.0f);
	else if (kind == BtnKind::Danger)
		dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), Pal::U32(Pal::VeryBad), 10.0f);

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

bool QCCheckbox(const char *id, bool *v)
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
		dl->AddRect(p, b, Pal::U32(hov ? Pal::BorderHov : Pal::GhostBorder), 6.0f, ImDrawCornerFlags_All, 1.2f);
	}
	DrawFocusRing(dl, p, b, 6.0f);
	return pressed;
}

// Row card container for the settings list.
ImVec2 BeginRowCard(float height)
{
	ImVec2 p = ImGui::GetCursorScreenPos();
	float w = ImGui::GetWindowContentRegionWidth();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	dl->AddRectFilled(p, ImVec2(p.x + w, p.y + height), Pal::U32(Pal::Card), 12.0f);
	dl->AddRect(p, ImVec2(p.x + w, p.y + height), Pal::U32(Pal::Border), 12.0f);
	return p;
}

void EndRowCard(ImVec2 p, float height)
{
	ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + height));
	ImGui::Dummy(ImVec2(0, 0));
}

void RowIconLabel(ImVec2 rowPos, IconFn icon, const char *label)
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

void RowSubLine(ImVec2 rowPos, const char *text)
{
	ImGui::GetWindowDrawList()->AddText(g_fontSmall, g_fontSmall->FontSize,
		ImVec2(rowPos.x + 92.0f, rowPos.y + kRowHeight - 6.0f), Pal::U32(Pal::Dim), text);
}

// A plain settings toggle, whole: card, checkbox at the shared inset, icon +
// label, optional sub-line. Returns whether the value changed this frame --
// persisting it stays with the caller, since the settings and profile
// save-or-restore paths are different templates.
bool ToggleRow(const char *id, IconFn icon, const char *label, bool &value, const char *subline)
{
	RowCard row(kRowHeight + (subline ? kRowSubLineH : 0.0f));
	ImGui::SetCursorScreenPos(ImVec2(row.pos.x + kRowInsetX, row.pos.y + kRowControlY));
	bool changed = QCCheckbox(id, &value);
	RowIconLabel(row.pos, icon, label);
	if (subline)
		RowSubLine(row.pos, subline);
	return changed;
}

// A tooltip that stays out of the way: above the cursor when the cursor is in
// the lower half of the window, so it cannot cover the controls beneath it.
// Escape as the keyboard form of a modal's Cancel / Keep / Close: every
// modal has one, and a key that visibly does nothing reads as a hang.
bool EscapePressed()
{
	return ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape), false);
}

void ShowTip(const char *text, bool leftOfCursor)
{
	// Nothing behind a modal may raise a tooltip: a rect-based hover test does
	// not know the modal is there, and a tooltip window appearing takes focus
	// from it.
	if (ImGuiWindow *modal = ImGui::GetFrontMostPopupModal())
	{
		ImGuiWindow *current = ImGui::GetCurrentWindowRead();
		if (!current || current->RootWindow != modal->RootWindow)
			return;
	}
	ImGuiIO &io = ImGui::GetIO();
	int lines = 1;
	for (const char *p = text; *p; ++p)
		if (*p == '\n')
			++lines;
	float h = lines * g_fontBody->FontSize + 24.0f;
	// Above the cursor in the lower half of the window, and never past the
	// right edge: the window is fixed-size, so a tooltip raised near the
	// edge would otherwise be cut mid-sentence.
	ImGui::PushFont(g_fontBody);
	float w = ImGui::CalcTextSize(text).x + ImGui::GetStyle().WindowPadding.x * 2.0f + 2.0f;
	ImGui::PopFont();
	// Raised by keyboard focus rather than the pointer, the tip anchors to
	// the control itself: the mouse may be parked over something else.
	const ImVec2 itemMin = ImGui::GetItemRectMin(), itemMax = ImGui::GetItemRectMax();
	const bool byFocus = io.NavVisible && !ImGui::IsMouseHoveringRect(itemMin, itemMax);
	const ImVec2 at = byFocus ? ImVec2(itemMin.x, itemMax.y) : io.MousePos;
	// Tools inside a list ask for the tip beside the cursor, so it never
	// lands on the row beneath.
	float x = leftOfCursor ? at.x - w - 16.0f
		: std::min(at.x + 16.0f, io.DisplaySize.x - w - 12.0f);
	float y = leftOfCursor ? at.y - h * 0.5f
		: at.y > io.DisplaySize.y * 0.5f ? (byFocus ? itemMin.y : at.y) - h - 8.0f : at.y + (byFocus ? 8.0f : 20.0f);
	ImGui::SetNextWindowPos(ImVec2(std::max(8.0f, x), y));
	ImGui::BeginTooltip();
	ImGui::TextUnformatted(text);
	ImGui::EndTooltip();
}

// A checkbox + label inside a nested inset whose whole line is the hover and
// click target.
bool NestedToggle(const char *id, ImVec2 pos, float width, const char *label, bool &value, const char *tooltip)
{
	ImGui::SetCursorScreenPos(pos);
	bool changed = QCCheckbox(id, &value);
	bool hovered = ImGui::IsItemHovered();
	ImGui::GetWindowDrawList()->AddText(g_fontBody, g_fontBody->FontSize,
		ImVec2(pos.x + 36.0f, pos.y + 12.0f - g_fontBody->FontSize * 0.5f),
		Pal::U32(Pal::Text), label);
	ImGui::SetCursorScreenPos(ImVec2(pos.x + 30.0f, pos.y - 4.0f));
	std::string labelId = std::string(id) + "_label";
	if (ImGui::InvisibleButton(labelId.c_str(), ImVec2(width - 30.0f, 32.0f)))
	{
		value = !value;
		changed = true;
	}
	hovered = hovered || ImGui::IsItemHovered();
	if (hovered && tooltip)
		ShowTip(tooltip);
	return changed;
}

int Segmented(const char *id, int value, const char *const items[], int count, float itemW, float h)
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
		{
			// Fill plus an accent rail: the fill alone was a 1.3:1 state cue.
			dl->AddRectFilled(ip, ImVec2(ip.x + isz.x, ip.y + isz.y), Pal::U32(ImVec4(1, 1, 1, 0.18f)), 5.0f);
			dl->AddRectFilled(ImVec2(ip.x + 8.0f, ip.y + isz.y - 3.0f), ImVec2(ip.x + isz.x - 8.0f, ip.y + isz.y - 1.0f),
				Pal::U32(Pal::Accent), 1.0f);
		}
		else if (hov)
			dl->AddRectFilled(ip, ImVec2(ip.x + isz.x, ip.y + isz.y), Pal::U32(ImVec4(1, 1, 1, 0.04f)), 5.0f);
		DrawFocusRing(dl, ip, ImVec2(ip.x + isz.x, ip.y + isz.y), 5.0f);

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

void DrawStatusCard(const std::vector<StatusRowData> &rows)
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

std::string FormatString(const char *fmt, ...)
{
	char buf[512];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof buf, fmt, args);
	va_end(args);
	return std::string(buf);
}
