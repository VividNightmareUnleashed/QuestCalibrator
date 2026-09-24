// The Lighthouse tab: which base stations each lighthouse device has in view,
// from SteamVR's own log (LighthouseVisibility.h), and whether two of the
// player's stations share a channel, from the stations' OpenVR mode labels.
// Nothing here is measured by QuestCalibrator; it is the driver's account of
// its own tracking, laid out as one grid of devices against stations. The
// room in 3D is left to the optional 3D View module.
#include "stdafx.h"
#include "UiInternal.h"

namespace
{

// Stations in channel order: the columns have to keep their order while the
// counts change, and Stations() sorts by drops.
std::vector<LighthouseVisibility::Station> StationsByChannel()
{
	auto stations = CalCtx.lighthouse.Stations();
	std::sort(stations.begin(), stations.end(),
		[](const LighthouseVisibility::Station &a, const LighthouseVisibility::Station &b) {
			return a.channel < b.channel;
		});
	return stations;
}

bool Sees(const LighthouseVisibility::Device *seen, int channel)
{
	return seen && seen->visibleKnown &&
		std::find(seen->visible.begin(), seen->visible.end(), channel) != seen->visible.end();
}

float TextWidth(ImFont *font, const char *text)
{
	return font->CalcTextSizeA(font->LegacySize, std::numeric_limits<float>::max(), 0.0f, text).x;
}

float WrappedHeight(ImFont *font, const std::string &text, float width)
{
	return font->CalcTextSizeA(font->LegacySize, std::numeric_limits<float>::max(), width, text.c_str()).y;
}

void WrappedText(ImDrawList *dl, ImFont *font, ImVec2 pos, ImU32 col, const std::string &text, float width)
{
	dl->AddText(font, font->LegacySize, pos, col, text.c_str(), nullptr, width);
}

// One centred card for the states with nothing to lay out: what is missing
// and what brings the tab to life.
void MessageCard(IconFn icon, const char *title, const std::string &body)
{
	const float h = 132.0f;
	ImVec2 p = BeginRowCard(h);
	const float cw = ImGui::GetContentRegionAvail().x;
	ImDrawList *dl = ImGui::GetWindowDrawList();
	icon(dl, ImVec2(p.x + cw * 0.5f, p.y + 34.0f), 15.0f, Pal::U32(Pal::Faint));
	dl->AddText(g_fontBody, g_fontBody->LegacySize,
		ImVec2(p.x + (cw - TextWidth(g_fontBody, title)) * 0.5f, p.y + 58.0f),
		Pal::U32(Pal::Text), title);
	dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
		ImVec2(p.x + (cw - TextWidth(g_fontSmall, body.c_str())) * 0.5f, p.y + 90.0f),
		Pal::U32(Pal::Dim), body.c_str());
	EndRowCard(p, h);
}

// The device icon as DeviceRow draws it: SteamVR's own art when it resolves,
// the vector glyph otherwise.
void RowDeviceIcon(ImDrawList *dl, const VRDevice &dev, ImVec2 c, ImU32 fallback)
{
	const DeviceIconTex *tex = GetDeviceIconTex(dev.iconPath);
	if (!tex)
	{
		DeviceIcon(dl, dev, c, 13.0f, fallback);
		return;
	}
	const float boxW = 40.0f, boxH = 34.0f;
	float scale = boxW / (float)tex->w;
	if (scale * (float)tex->h > boxH)
		scale = boxH / (float)tex->h;
	ImVec2 half = ImVec2(tex->w * scale * 0.5f, tex->h * scale * 0.5f);
	dl->AddImage((ImTextureID)(intptr_t)tex->tex,
		ImVec2(c.x - half.x, c.y - half.y), ImVec2(c.x + half.x, c.y + half.y));
}

// A base station in SteamVR's own art, fitted into a square box; a plain
// station-shaped block when the art does not resolve.
void StationIcon(ImDrawList *dl, const std::string &path, ImVec2 c, float box)
{
	const DeviceIconTex *tex = GetDeviceIconTex(path);
	if (tex && tex->w > 0 && tex->h > 0)
	{
		const float scale = std::min(box / (float)tex->w, box / (float)tex->h);
		const ImVec2 half(tex->w * scale * 0.5f, tex->h * scale * 0.5f);
		dl->AddImage((ImTextureID)(intptr_t)tex->tex, ImVec2(c.x - half.x, c.y - half.y),
			ImVec2(c.x + half.x, c.y + half.y));
		return;
	}
	const ImVec2 half(box * 0.34f, box * 0.44f);
	dl->AddRectFilled(ImVec2(c.x - half.x, c.y - half.y), ImVec2(c.x + half.x, c.y + half.y),
		Pal::U32(Pal::Dim), 3.0f);
	dl->AddCircleFilled(ImVec2(c.x, c.y - half.y + box * 0.22f), box * 0.09f, Pal::U32(Pal::Card), 12);
}

void SectionLabelAt(ImDrawList *dl, const FlexRect &r, const char *text)
{
	LetterSpacedTextAt(dl, g_fontSmall, r.min, Pal::U32(Pal::Dim), text, 2.0f);
}

// A warning triangle, drawn to sit where IconCheck sits.
void WarnGlyph(ImDrawList *dl, ImVec2 c, float s, ImU32 col)
{
	dl->AddTriangle(ImVec2(c.x, c.y - s), ImVec2(c.x + s * 1.1f, c.y + s * 0.85f),
		ImVec2(c.x - s * 1.1f, c.y + s * 0.85f), col, 1.8f);
	dl->AddLine(ImVec2(c.x, c.y - s * 0.3f), ImVec2(c.x, c.y + s * 0.25f), col, 1.8f);
	dl->AddCircleFilled(ImVec2(c.x, c.y + s * 0.55f), 1.3f, col, 8);
}

// A control for something not built yet: its outline and label greyed out,
// and a tooltip saying so. The Smoothing tab is announced the same way.
void UnbuiltButton(const char *id, const char *label, ImVec2 pos, ImVec2 size, const char *tip)
{
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImGui::SetCursorScreenPos(pos);
	ImGui::InvisibleButton(id, size);
	const ImVec2 end(pos.x + size.x, pos.y + size.y);
	dl->AddRect(pos, end, Pal::U32(Pal::Border), 10.0f);
	dl->AddText(g_fontBody, g_fontBody->LegacySize,
		ImVec2(pos.x + (size.x - TextWidth(g_fontBody, label)) * 0.5f, pos.y + (size.y - g_fontBody->LegacySize) * 0.5f),
		Pal::U32(Pal::Faint), label);
	if (ImGui::IsItemHovered())
		ShowTip(tip, true);
}

// ---------------------------------------------------------------------------
// The stations as OpenVR lists them
// ---------------------------------------------------------------------------

// The OpenVR base station behind a log channel: by id, which the serial
// carries, and by the mode label while the log has not named the id yet.
const VRStation *StationDevice(const VRState &state, const LighthouseVisibility::Station &s)
{
	if (s.id != 0)
	{
		const std::string serial = "LHB-" + LighthouseVisibility::IdName(s.id);
		for (const auto &st : state.stations)
			if (_stricmp(st.serial.c_str(), serial.c_str()) == 0)
				return &st;
	}
	const std::string channel = std::to_string(s.channel);
	for (const auto &st : state.stations)
		if (st.modeLabel == channel)
			return &st;
	return nullptr;
}

// The eight hex digits of a station serial ("LHB-2A91C0D4"), the id the log
// prints; the whole serial when it has another shape.
std::string StationIdOf(const VRStation &st)
{
	return st.serial.rfind("LHB-", 0) == 0 ? st.serial.substr(4) : st.serial;
}

// Two or more of the player's stations on one channel, and the channel one
// of them could move to. Base station 2.0 has channels 1 to 16; a free one is
// used by no station OpenVR lists and none the log has named.
struct ChannelConflict
{
	int channel = -1;                     // -1: no conflict
	std::vector<const VRStation *> sharing;
	const VRStation *move = nullptr;      // the one to move
	int suggested = -1;                   // -1: every channel is taken
};

ChannelConflict FindChannelConflict(const VRState &state,
	const std::vector<LighthouseVisibility::Station> &logged)
{
	ChannelConflict found;
	std::map<int, std::vector<const VRStation *>> byChannel;
	for (const auto &st : state.stations)
	{
		char *end = nullptr;
		const long channel = std::strtol(st.modeLabel.c_str(), &end, 10);
		if (!st.modeLabel.empty() && end && *end == '\0' && channel >= 1 && channel <= 16)
			byChannel[(int)channel].push_back(&st);
	}
	for (const auto &entry : byChannel)
	{
		if (entry.second.size() < 2)
			continue;
		found.channel = entry.first;
		found.sharing = entry.second;
		break;
	}
	if (found.channel < 0)
		return found;

	// Move the one the log has not been naming on that channel, so the
	// station every device already knows by it keeps it.
	std::string loggedSerial;
	for (const auto &s : logged)
		if (s.channel == found.channel && s.id != 0)
			loggedSerial = "LHB-" + LighthouseVisibility::IdName(s.id);
	found.move = found.sharing.back();
	for (const VRStation *st : found.sharing)
		if (_stricmp(st->serial.c_str(), loggedSerial.c_str()) != 0)
			found.move = st;

	for (int channel = 1; channel <= 16 && found.suggested < 0; ++channel)
	{
		bool taken = byChannel.count(channel) > 0;
		for (const auto &s : logged)
			taken = taken || s.channel == channel;
		if (!taken)
			found.suggested = channel;
	}
	return found;
}

// ---------------------------------------------------------------------------
// The grid
// ---------------------------------------------------------------------------

// How long a freshly lost station glows behind its ring.
constexpr double kLossFreshSeconds = 4.0;

// The ring clock (QPC seconds) the visibility model stamps live lines with.
double RingNow()
{
	LARGE_INTEGER frequency, counter;
	if (!QueryPerformanceFrequency(&frequency) || !QueryPerformanceCounter(&counter) || frequency.QuadPart == 0)
		return 0.0;
	return static_cast<double>(counter.QuadPart) / static_cast<double>(frequency.QuadPart);
}

enum class Cell { InView, Lost, OutOfView, Off };

// One device's cell under one station: in view, lost (it had the station
// and dropped it), out of view (it never had it from where it stands), or
// nothing to say while the device is off or not in the log yet.
Cell CellFor(const LighthouseVisibility::Device *seen, bool reporting, int channel)
{
	if (!reporting)
		return Cell::Off;
	if (Sees(seen, channel))
		return Cell::InView;
	return seen->lost.count(channel) ? Cell::Lost : Cell::OutOfView;
}

void DrawCell(ImDrawList *dl, ImVec2 c, Cell cell, float freshness)
{
	switch (cell)
	{
	case Cell::InView:
		dl->AddCircleFilled(c, 6.0f, Pal::U32(Pal::Text), 20);
		break;
	case Cell::Lost:
		if (freshness > 0.0f)
			dl->AddCircleFilled(c, 11.0f, Pal::U32(Pal::Bad, 0.25f * freshness), 24);
		dl->AddCircle(c, 5.5f, Pal::U32(Pal::Bad), 20, 2.0f);
		break;
	case Cell::OutOfView:
		dl->AddCircle(c, 5.5f, Pal::U32(Pal::Faint), 20, 1.2f);
		break;
	case Cell::Off:
		dl->AddCircleFilled(c, 2.0f, Pal::U32(Pal::BorderHov), 8);
		break;
	}
}

// One line under a device's name, for the rows with something to say. The
// figure at the right carries the count; this names what changed.
std::string RowNote(const LighthouseVisibility::Device *seen, bool reporting, int clean, double now,
	ImVec4 &color)
{
	color = Pal::Dim;
	if (!reporting)
		return {};
	if (seen->visible.empty())
	{
		color = Pal::Bad;
		return "Sees no base station";
	}
	if ((int)seen->visible.size() < clean)
	{
		color = Pal::Bad;
		return "Down to one station";
	}
	std::string fresh, lasting;
	for (const auto &lost : seen->lost)
	{
		std::string &into = now - lost.second < kLossFreshSeconds ? fresh : lasting;
		into += (into.empty() ? "S-" : ", S-") + std::to_string(lost.first);
	}
	if (!fresh.empty())
		return "Lost " + fresh + " just now";
	if (!lasting.empty())
		return "Lost " + lasting;
	if (seen->drops >= 5)
		return FormatString("Lost a station %u times", seen->drops);
	return {};
}

// Problems first: no station or one, then a lost station, then the rest, and
// the devices that are off or not in the log yet last.
int RowRank(const VRDevice &dev, const LighthouseVisibility::Device *seen, int clean)
{
	if (!dev.connected)
		return 5;
	if (!seen || !seen->visibleKnown)
		return 4;
	if ((int)seen->visible.size() < clean)
		return 0;
	if (!seen->lost.empty())
		return 1;
	return 2;
}

// ---------------------------------------------------------------------------
// The side cards
// ---------------------------------------------------------------------------

constexpr float kCardPad = 20.0f;
constexpr float kButtonH = 44.0f;

// The channel card's text, measured once so the layout knows its height
// before anything is drawn.
struct ChannelCard
{
	bool conflict = false;
	std::string title, body, suggestion, suggestionNote, steamvr, button;
	float height = 0.0f;
};

ChannelCard BuildChannelCard(const ChannelConflict &c, int stationCount, float width)
{
	ChannelCard card;
	const float textW = width - kCardPad * 2.0f;
	const float titleH = g_fontBody->LegacySize + 4.0f;
	if (c.channel < 0)
	{
		card.title = "No channel conflicts";
		card.body = stationCount == 0 ? std::string("No base station is listed by SteamVR yet.")
			: stationCount == 1 ? std::string("Your one base station has a channel to itself.")
			: FormatString("Each of your %d base stations has its own channel.", stationCount);
		card.height = kCardPad + titleH + 8.0f + WrappedHeight(g_fontSmall, card.body, textW) + kCardPad;
		return card;
	}

	card.conflict = true;
	card.title = c.sharing.size() == 2 ? FormatString("Two stations share channel %d", c.channel)
		: FormatString("%d stations share channel %d", (int)c.sharing.size(), c.channel);
	card.body = "Stations on the same channel get in each other's way, so devices lose them.";
	card.steamvr = "Change it in SteamVR: Devices > Base Station Settings > Configure Base Station Channels.";
	float h = kCardPad + titleH + 8.0f + WrappedHeight(g_fontSmall, card.body, textW) + 14.0f;
	if (c.suggested > 0)
	{
		card.suggestion = FormatString("Move S-%d (%s) to channel %d", c.channel, StationIdOf(*c.move).c_str(), c.suggested);
		card.suggestionNote = FormatString("None of your stations uses channel %d.", c.suggested);
		card.button = FormatString("Switch to channel %d", c.suggested);
		const float innerW = textW - 28.0f - 36.0f;
		h += 14.0f + WrappedHeight(g_fontBody, card.suggestion, innerW) + 4.0f +
			WrappedHeight(g_fontSmall, card.suggestionNote, innerW) + 14.0f + 12.0f + kButtonH + 12.0f;
	}
	else
	{
		card.suggestion = "Every channel from 1 to 16 is in use, so turn off a station you can spare.";
		h += WrappedHeight(g_fontSmall, card.suggestion, textW) + 12.0f;
	}
	h += WrappedHeight(g_fontSmall, card.steamvr, textW) + kCardPad;
	card.height = h;
	return card;
}

void DrawChannelCard(ImDrawList *dl, const FlexRect &r, const ChannelCard &card, const ChannelConflict &c)
{
	dl->AddRectFilled(r.min, r.max, Pal::U32(Pal::Card), 12.0f);
	dl->AddRect(r.min, r.max, Pal::U32(card.conflict ? Pal::Warn : Pal::Border, card.conflict ? 0.45f : 1.0f), 12.0f);
	const float x = r.min.x + kCardPad;
	const float textW = r.W() - kCardPad * 2.0f;
	float y = r.min.y + kCardPad;

	const ImVec2 glyph(x + 9.0f, y + g_fontBody->LegacySize * 0.5f + 1.0f);
	if (card.conflict)
		WarnGlyph(dl, glyph, 8.0f, Pal::U32(Pal::Warn));
	else
		IconCheck(dl, glyph, 8.0f, Pal::U32(Pal::Good));
	dl->AddText(g_fontBody, g_fontBody->LegacySize, ImVec2(x + 28.0f, y), Pal::U32(Pal::Text), card.title.c_str());
	y += g_fontBody->LegacySize + 4.0f + 8.0f;
	WrappedText(dl, g_fontSmall, ImVec2(x, y), Pal::U32(Pal::Dim), card.body, textW);
	y += WrappedHeight(g_fontSmall, card.body, textW);
	if (!card.conflict)
		return;
	y += 14.0f;

	if (c.suggested > 0)
	{
		// The suggestion in an inset box: the station to move, the channel
		// to move it to, and the switch itself.
		const float innerW = textW - 28.0f - 36.0f;
		const float boxH = 14.0f + WrappedHeight(g_fontBody, card.suggestion, innerW) + 4.0f +
			WrappedHeight(g_fontSmall, card.suggestionNote, innerW) + 14.0f + 12.0f + kButtonH + 12.0f;
		const ImVec2 b0(x, y), b1(x + textW, y + boxH);
		dl->AddRectFilled(b0, b1, Pal::U32(Pal::Inset), 10.0f);
		dl->AddRect(b0, b1, Pal::U32(Pal::Border), 10.0f);
		StationIcon(dl, c.move->iconPath, ImVec2(b0.x + 14.0f + 13.0f, b0.y + 14.0f + 16.0f), 26.0f);
		const float tx = b0.x + 14.0f + 36.0f;
		float ty = b0.y + 14.0f;
		WrappedText(dl, g_fontBody, ImVec2(tx, ty), Pal::U32(Pal::Text), card.suggestion, innerW);
		ty += WrappedHeight(g_fontBody, card.suggestion, innerW) + 4.0f;
		WrappedText(dl, g_fontSmall, ImVec2(tx, ty), Pal::U32(Pal::Dim), card.suggestionNote, innerW);
		ty += WrappedHeight(g_fontSmall, card.suggestionNote, innerW) + 14.0f;
		UnbuiltButton("switch", card.button.c_str(), ImVec2(b0.x + 12.0f, ty), ImVec2(textW - 24.0f, kButtonH),
			"Switching from here is not built yet. Change the channel in SteamVR for now.");
		y = b1.y + 12.0f;
	}
	else
	{
		WrappedText(dl, g_fontSmall, ImVec2(x, y), Pal::U32(Pal::Text), card.suggestion, textW);
		y += WrappedHeight(g_fontSmall, card.suggestion, textW) + 12.0f;
	}
	WrappedText(dl, g_fontSmall, ImVec2(x, y), Pal::U32(Pal::Dim), card.steamvr, textW);
}

// The 3D View card's sketch: a floor in three-quarter view, a station high
// in each corner and their lines of sight to one spot, one of them blocked.
// `c` is the middle of the floor; the sketch reaches about 96 px above it
// and 39 px below.
void DrawRoomSketch(ImDrawList *dl, ImVec2 c, float alpha)
{
	const float k = 26.0f;   // px per metre
	auto iso = [c, k](float x, float y, float z) {
		return ImVec2(c.x + (x - z) * 0.87f * k, c.y + (x + z) * 0.5f * k - y * k);
	};
	const ImVec2 floor[4] = { iso(-1.6f, 0, -1.4f), iso(1.6f, 0, -1.4f), iso(1.6f, 0, 1.4f), iso(-1.6f, 0, 1.4f) };
	dl->AddConvexPolyFilled(floor, 4, Pal::U32(Pal::Inset, alpha));
	dl->AddPolyline(floor, 4, Pal::U32(Pal::BorderHov, alpha), ImDrawFlags_Closed, 1.0f);
	for (float x : { -0.8f, 0.0f, 0.8f })
		dl->AddLine(iso(x, 0, -1.4f), iso(x, 0, 1.4f), Pal::U32(Pal::Border, alpha), 1.0f);
	for (float z : { -0.7f, 0.0f, 0.7f })
		dl->AddLine(iso(-1.6f, 0, z), iso(1.6f, 0, z), Pal::U32(Pal::Border, alpha), 1.0f);

	const ImVec2 target = iso(0.1f, 1.1f, 0.2f);
	const float corners[4][3] = { { -1.6f, 2.2f, -1.4f }, { 1.6f, 2.1f, -1.4f }, { 1.6f, 2.3f, 1.4f }, { -1.6f, 1.4f, 1.4f } };
	dl->AddCircleFilled(iso(-0.9f, 0.9f, 0.9f), 10.0f, Pal::U32(Pal::Bad, 0.16f * alpha), 20);
	for (int i = 0; i < 4; ++i)
	{
		const ImVec2 p = iso(corners[i][0], corners[i][1], corners[i][2]);
		const ImVec2 base = iso(corners[i][0], 0, corners[i][2]);
		dl->AddLine(base, p, Pal::U32(Pal::BorderHov, alpha), 1.0f);
		if (i == 3)
			dl->AddLine(p, target, Pal::U32(Pal::Bad, 0.9f * alpha), 1.4f);
		else
			dl->AddLine(p, target, Pal::U32(Pal::Text, 0.35f * alpha), 1.4f);
		dl->AddRectFilled(ImVec2(p.x - 4.5f, p.y - 6.0f), ImVec2(p.x + 4.5f, p.y + 6.0f), Pal::U32(Pal::Good, alpha), 2.0f);
	}
	dl->AddCircleFilled(target, 4.0f, Pal::U32(Pal::Good, alpha), 12);
}

constexpr float kSketchH = 150.0f;
const char *const kModuleText =
	"Your room in 3D, in your browser: the stations, your devices, and where their view gets blocked.";

float ModuleCardHeight(float width)
{
	return kCardPad + kSketchH + 10.0f + WrappedHeight(g_fontSmall, kModuleText, width - kCardPad * 2.0f) +
		14.0f + kButtonH + kCardPad;
}

void DrawModuleCard(ImDrawList *dl, const FlexRect &r)
{
	dl->AddRectFilled(r.min, r.max, Pal::U32(Pal::Card), 12.0f);
	dl->AddRect(r.min, r.max, Pal::U32(Pal::Border), 12.0f);
	const float x = r.min.x + kCardPad;
	const float textW = r.W() - kCardPad * 2.0f;
	float y = r.min.y + kCardPad;
	DrawRoomSketch(dl, ImVec2(r.min.x + r.W() * 0.5f, y + 104.0f), 0.55f);
	y += kSketchH + 10.0f;
	WrappedText(dl, g_fontSmall, ImVec2(x, y), Pal::U32(Pal::Dim), kModuleText, textW);
	y += WrappedHeight(g_fontSmall, kModuleText, textW) + 14.0f;
	UnbuiltButton("3dview", "Open 3D View", ImVec2(x, y), ImVec2(textW, kButtonH),
		"The 3D View is an optional module that is not built yet.");
}

} // namespace

void BuildLighthouseScreen(const VRState &state)
{
	const float cw = ImGui::GetContentRegionAvail().x;
	ImDrawList *dl = ImGui::GetWindowDrawList();

	std::vector<const VRDevice *> devices;
	for (const auto &dev : state.devices)
		if (dev.trackingSystem == "lighthouse")
			devices.push_back(&dev);

	if (devices.empty())
	{
		MessageCard(IconTracker, "No base station devices",
			"Turn on a tracker or controller that uses base stations.");
		return;
	}
	if (!CalCtx.lighthouseLogAvailable)
	{
		MessageCard(IconClock, "Waiting for SteamVR's log",
			"Base stations appear here once SteamVR has written " +
			(CalCtx.lighthouseLogPath.empty() ? std::string("its vrserver log.") : CalCtx.lighthouseLogPath));
		return;
	}

	const auto stations = StationsByChannel();
	const int clean = CalCtx.lighthouse.Settings().cleanStations;
	const double now = RingNow();
	std::stable_sort(devices.begin(), devices.end(), [clean](const VRDevice *a, const VRDevice *b) {
		return RowRank(*a, CalCtx.lighthouse.Find(a->serial), clean) <
			RowRank(*b, CalCtx.lighthouse.Find(b->serial), clean);
	});

	int listedStations = 0;
	for (const auto &st : state.stations)
		listedStations += st.connected ? 1 : 0;
	const ChannelConflict conflict = FindChannelConflict(state, stations);

	// ---- Layout: the grid on the left; the channels and the 3D View beside it.
	const float sideW = 360.0f;
	const float gap = 20.0f;
	const float labelH = g_fontSmall->LegacySize + 4.0f;
	const float headH = 76.0f, rowH = 56.0f, footH = 44.0f;
	const float gridH = headH + rowH * (float)devices.size() + footH;
	const ChannelCard channelCard = BuildChannelCard(conflict, listedStations, sideW);

	FlexLayout fl;
	YGNodeRef root = fl.Root();
	YGNodeStyleSetFlexDirection(root, YGFlexDirectionRow);
	YGNodeStyleSetGap(root, YGGutterColumn, gap);
	YGNodeStyleSetAlignItems(root, YGAlignFlexStart);

	YGNodeRef gridCol = fl.Column(root);
	YGNodeStyleSetFlexGrow(gridCol, 1.0f);
	YGNodeRef gridLabel = fl.Add(gridCol);
	YGNodeStyleSetHeight(gridLabel, labelH);
	YGNodeStyleSetMargin(gridLabel, YGEdgeBottom, 6.0f);
	YGNodeRef grid = fl.Add(gridCol);
	YGNodeStyleSetHeight(grid, gridH);
	YGNodeRef legend = fl.Add(gridCol);
	YGNodeStyleSetHeight(legend, g_fontSmall->LegacySize + 4.0f);
	YGNodeStyleSetMargin(legend, YGEdgeTop, 10.0f);

	YGNodeRef sideCol = fl.Column(root);
	YGNodeStyleSetWidth(sideCol, sideW);
	YGNodeStyleSetFlexShrink(sideCol, 0.0f);
	YGNodeRef channelLabel = fl.Add(sideCol);
	YGNodeStyleSetHeight(channelLabel, labelH);
	YGNodeStyleSetMargin(channelLabel, YGEdgeBottom, 6.0f);
	YGNodeRef channelNode = fl.Add(sideCol);
	YGNodeStyleSetHeight(channelNode, channelCard.height);
	YGNodeRef moduleLabel = fl.Add(sideCol);
	YGNodeStyleSetHeight(moduleLabel, labelH);
	YGNodeStyleSetMargin(moduleLabel, YGEdgeTop, 20.0f);
	YGNodeStyleSetMargin(moduleLabel, YGEdgeBottom, 6.0f);
	YGNodeRef moduleNode = fl.Add(sideCol);
	YGNodeStyleSetHeight(moduleNode, ModuleCardHeight(sideW));

	const ImVec2 origin = ImGui::GetCursorScreenPos();
	fl.Compute(origin, cw, YGUndefined);

	// ---- The grid ----
	SectionLabelAt(dl, fl.Rect(gridLabel), "WHICH BASE STATIONS SEE EACH DEVICE");
	const FlexRect gr = fl.Rect(grid);
	const float gw = gr.W();
	dl->AddRectFilled(gr.min, gr.max, Pal::U32(Pal::Card), 12.0f);
	dl->AddRect(gr.min, gr.max, Pal::U32(Pal::Border), 12.0f);

	const float padL = 12.0f, padR = 20.0f, figureW = 96.0f, nameX = 62.0f;
	const float colW = stations.empty() ? 72.0f
		: std::min(72.0f, (gw - nameX - 180.0f - figureW - padR) / (float)stations.size());
	const float colsX = gr.max.x - padR - figureW - colW * (float)stations.size();
	const float nameEnd = colsX - 8.0f;
	auto colCenter = [&](size_t k) { return colsX + colW * ((float)k + 0.5f); };

	// The header: one column per station, in SteamVR's art, hoverable for its
	// id and drop count.
	const float headBase = gr.min.y + headH - 12.0f;
	LetterSpacedTextAt(dl, g_fontSmall, ImVec2(gr.min.x + padL + 8.0f, headBase - g_fontSmall->LegacySize),
		Pal::U32(Pal::Dim), "DEVICE", 2.0f);
	const float inViewW = LetterSpacedWidth(g_fontSmall, "IN VIEW", 2.0f);
	LetterSpacedTextAt(dl, g_fontSmall, ImVec2(gr.max.x - padR - inViewW, headBase - g_fontSmall->LegacySize),
		Pal::U32(Pal::Dim), "IN VIEW", 2.0f);
	for (size_t k = 0; k < stations.size(); ++k)
	{
		const auto &s = stations[k];
		const VRStation *st = StationDevice(state, s);
		const float cx = colCenter(k);
		StationIcon(dl, st ? st->iconPath : std::string(), ImVec2(cx, gr.min.y + 26.0f), 28.0f);
		const std::string name = FormatString("S-%d", s.channel);
		dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
			ImVec2(cx - TextWidth(g_fontSmall, name.c_str()) * 0.5f, headBase - g_fontSmall->LegacySize),
			Pal::U32(Pal::Text), name.c_str());

		ImGui::PushID(s.channel);
		ImGui::SetCursorScreenPos(ImVec2(cx - colW * 0.5f, gr.min.y));
		ImGui::InvisibleButton("station", ImVec2(colW, headH));
		if (ImGui::IsItemHovered())
		{
			std::string tip = CalCtx.lighthouse.StationName(s.channel);
			tip += s.drops == 0 ? std::string("\nNot lost this session")
				: FormatString("\nLost %u time%s this session", s.drops, s.drops == 1 ? "" : "s");
			if (s.losses > 0)
				tip += FormatString("\nThe last station a device lost %u time%s", s.losses, s.losses == 1 ? "" : "s");
			ShowTip(tip.c_str(), true);
		}
		ImGui::PopID();
	}
	dl->AddLine(ImVec2(gr.min.x, gr.min.y + headH), ImVec2(gr.max.x, gr.min.y + headH), Pal::U32(Pal::Border), 1.0f);

	// The device rows.
	std::vector<int> seeing(stations.size(), 0);
	int reportingCount = 0;
	for (size_t i = 0; i < devices.size(); ++i)
	{
		const VRDevice &dev = *devices[i];
		const auto *seen = CalCtx.lighthouse.Find(dev.serial);
		const bool reporting = dev.connected && seen && seen->visibleKnown;
		reportingCount += reporting ? 1 : 0;
		const ImVec2 p(gr.min.x, gr.min.y + headH + rowH * (float)i);
		const ImVec2 b(gr.max.x, p.y + rowH);
		if (i > 0)
			dl->AddLine(ImVec2(p.x + 16.0f, p.y), ImVec2(b.x - 16.0f, p.y), Pal::U32(Pal::Border), 1.0f);

		ImGui::PushID(dev.id);
		ImGui::SetCursorScreenPos(p);
		ImGui::InvisibleButton("row", ImVec2(gw, rowH));
		const bool hov = ImGui::IsItemHovered();
		ImGui::PopID();
		if (hov)
			dl->AddRectFilled(p, b, Pal::U32(ImVec4(1, 1, 1, 0.03f)), 0.0f);

		RowDeviceIcon(dl, dev, ImVec2(p.x + padL + 18.0f, p.y + rowH * 0.5f), Pal::U32(Pal::Dim));
		ImVec4 noteColor;
		// SteamVR only knows the device is gone, not whether it was switched off
		// or carried out of range, so the note says both and no figure repeats it.
		std::string note = "Off or out of range";
		noteColor = Pal::Dim;
		if (dev.connected)
			note = RowNote(seen, reporting, clean, now, noteColor);
		const std::string name = DeviceDisplayName(dev);
		dl->PushClipRect(ImVec2(p.x + nameX, p.y), ImVec2(nameEnd, b.y), true);
		const float nameY = note.empty() ? p.y + (rowH - g_fontBody->LegacySize) * 0.5f - 1.0f : p.y + 7.0f;
		dl->AddText(g_fontBody, g_fontBody->LegacySize, ImVec2(p.x + nameX, nameY),
			Pal::U32(dev.connected ? Pal::Text : Pal::Dim), name.c_str());
		if (!note.empty())
			dl->AddText(g_fontSmall, g_fontSmall->LegacySize, ImVec2(p.x + nameX, p.y + 32.0f),
				Pal::U32(noteColor), note.c_str());
		dl->PopClipRect();

		for (size_t k = 0; k < stations.size(); ++k)
		{
			const int channel = stations[k].channel;
			const Cell cell = CellFor(seen, reporting, channel);
			float freshness = 0.0f;
			if (cell == Cell::Lost)
			{
				const double age = now - seen->lost.at(channel);
				if (age >= 0.0 && age < kLossFreshSeconds)
					freshness = (float)(1.0 - age / kLossFreshSeconds);
			}
			if (cell == Cell::InView)
				++seeing[k];
			DrawCell(dl, ImVec2(colCenter(k), p.y + rowH * 0.5f), cell, freshness);
		}

		std::string figure;
		ImVec4 figureColor = Pal::Dim;
		if (dev.connected && !reporting)
			figure = "Not logged";
		else if (dev.connected)
		{
			const int inView = static_cast<int>(seen->visible.size());
			figure = FormatString("%d of %d", inView, std::max(CalCtx.lighthouse.StationCount(), inView));
			figureColor = inView < clean ? Pal::Bad : Pal::Text;
		}
		dl->AddText(g_fontBody, g_fontBody->LegacySize,
			ImVec2(b.x - padR - TextWidth(g_fontBody, figure.c_str()), p.y + (rowH - g_fontBody->LegacySize) * 0.5f - 1.0f),
			Pal::U32(figureColor), figure.c_str());

		if (hov && reporting)
		{
			std::string tip = dev.serial + "\nIn view:";
			if (seen->visible.empty())
				tip += " no base station";
			for (int c : seen->visible)
				tip += " " + CalCtx.lighthouse.StationName(c);
			if ((int)seen->visible.size() < clean)
				tip += "\nWith fewer than two stations in view its tracking can drift.";
			tip += seen->drops == 0 ? std::string("\nNo station lost this session")
				: FormatString("\nLost a station %u time%s this session", seen->drops, seen->drops == 1 ? "" : "s");
			if (seen->losses > 0)
				tip += FormatString(", lost all of them %u time%s", seen->losses, seen->losses == 1 ? "" : "s");
			if (seen->bootstraps > 0)
				tip += FormatString("\nStarted over %u time%s", seen->bootstraps, seen->bootstraps == 1 ? "" : "s");
			if (!seen->lastDisturbanceText.empty())
				tip += "\nLast change: " + seen->lastDisturbanceText;
			ShowTip(tip.c_str(), true);
		}
	}

	// The footer: how many of the reporting devices see each station. One no
	// device sees is the one to look at: blocked, unplugged, or aimed away.
	const float footY = gr.max.y - footH;
	dl->AddLine(ImVec2(gr.min.x, footY), ImVec2(gr.max.x, footY), Pal::U32(Pal::Border), 1.0f);
	const float footText = footY + (footH - g_fontSmall->LegacySize) * 0.5f - 1.0f;
	dl->AddText(g_fontSmall, g_fontSmall->LegacySize, ImVec2(gr.min.x + padL + 8.0f, footText),
		Pal::U32(Pal::Dim), "Seen by");
	for (size_t k = 0; k < stations.size(); ++k)
	{
		const bool unseen = reportingCount > 0 && seeing[k] == 0;
		const std::string text = reportingCount == 0 ? std::string("-") : FormatString("%d of %d", seeing[k], reportingCount);
		dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
			ImVec2(colCenter(k) - TextWidth(g_fontSmall, text.c_str()) * 0.5f, footText),
			Pal::U32(unseen ? Pal::Bad : Pal::Text), text.c_str());
	}
	if (stations.empty())
		dl->AddText(g_fontSmall, g_fontSmall->LegacySize, ImVec2(gr.min.x + nameX + 180.0f, gr.min.y + 26.0f),
			Pal::U32(Pal::Dim), "No base station named in the log yet.");

	// The legend under the grid.
	{
		const FlexRect lg = fl.Rect(legend);
		float x = lg.min.x + 4.0f;
		const float cy = lg.min.y + lg.H() * 0.5f;
		const struct { Cell cell; const char *text; } keys[] = {
			{ Cell::InView, "In view" }, { Cell::Lost, "Lost, not back yet" }, { Cell::OutOfView, "Out of view" } };
		for (const auto &key : keys)
		{
			DrawCell(dl, ImVec2(x + 6.0f, cy), key.cell, 0.0f);
			dl->AddText(g_fontSmall, g_fontSmall->LegacySize, ImVec2(x + 18.0f, lg.min.y + 1.0f),
				Pal::U32(Pal::Dim), key.text);
			x += 18.0f + TextWidth(g_fontSmall, key.text) + 24.0f;
		}
	}

	// ---- Channels ----
	SectionLabelAt(dl, fl.Rect(channelLabel), "CHANNELS");
	DrawChannelCard(dl, fl.Rect(channelNode), channelCard, conflict);

	// ---- The 3D View module ----
	SectionLabelAt(dl, fl.Rect(moduleLabel), "3D VIEW");
	DrawModuleCard(dl, fl.Rect(moduleNode));

	const FlexRect all = fl.Rect(root);
	ImGui::SetCursorScreenPos(origin);
	ImGui::Dummy(ImVec2(cw, all.H()));
}
