// The Lighthouse tab: each base station, who sees it, and what every
// lighthouse device has in view, from SteamVR's own log
// (LighthouseVisibility.h). Nothing here is measured by QuestCalibrator; it
// is the driver's account of its own tracking, laid out per station.
#include "stdafx.h"
#include "UiInternal.h"

namespace
{

// Stations in channel order: the strip has to keep its order while the
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

// Only for a device with visibleKnown set.
bool Sees(const LighthouseVisibility::Device &seen, int channel)
{
	return std::find(seen.visible.begin(), seen.visible.end(), channel) != seen.visible.end();
}

float TextWidth(ImFont *font, const char *text)
{
	return font->CalcTextSizeA(font->LegacySize, std::numeric_limits<float>::max(), 0.0f, text).x;
}

// One centred card for the states with nothing to lay out: what is missing
// and what brings the tab to life.
void MessageCard(IconFn icon, const char *english, const std::string &englishBody)
{
	const char *title = Tr(english);
	const std::string body = Tr(englishBody);
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

} // namespace

void BuildLighthouseScreen(const VRState &state)
{
	const float cw = ImGui::GetContentRegionAvail().x;
	ImDrawList *dl = ImGui::GetWindowDrawList();

	// The devices this tab is about, the switched-on ones first.
	std::vector<const VRDevice *> devices;
	for (const auto &dev : state.devices)
		if (dev.trackingSystem == "lighthouse")
			devices.push_back(&dev);
	std::stable_sort(devices.begin(), devices.end(),
		[](const VRDevice *a, const VRDevice *b) { return a->connected && !b->connected; });

	if (devices.empty())
	{
		MessageCard(IconTracker, "No Lighthouse devices",
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

	// ---- The stations ----
	const auto stations = StationsByChannel();
	SectionLabel("BASE STATIONS");
	if (stations.empty())
	{
		ImGui::TextColored(Pal::Dim, "%s", Tr("None named yet. Each base station appears as SteamVR reports it."));
	}
	else
	{
		const float gap = 12.0f;
		const int n = static_cast<int>(stations.size());
		const float cardW = (cw - gap * (float)(n - 1)) / (float)n;
		const float cardH = 118.0f;
		const ImVec2 p = ImGui::GetCursorScreenPos();
		for (int i = 0; i < n; ++i)
		{
			const auto &s = stations[i];
			const ImVec2 c0 = ImVec2(p.x + (float)i * (cardW + gap), p.y);
			const ImVec2 c1 = ImVec2(c0.x + cardW, c0.y + cardH);

			// How many of the switched-on devices have it in view right now.
			int seeing = 0, reporting = 0;
			for (const VRDevice *dev : devices)
			{
				if (!dev->connected)
					continue;
				const auto *seen = CalCtx.lighthouse.Find(dev->serial);
				if (!seen || !seen->visibleKnown)
					continue;
				++reporting;
				if (Sees(*seen, s.channel))
					++seeing;
			}
			// A station no device sees is the one to look at: blocked,
			// unplugged, or aimed away.
			const bool unseen = reporting > 0 && seeing == 0;

			dl->AddRectFilled(c0, c1, Pal::U32(Pal::Card), 12.0f);
			dl->AddRect(c0, c1, Pal::U32(unseen ? Pal::Bad : Pal::Border), 12.0f);

			const float x = c0.x + 18.0f;
			const std::string channel = FormatString("S-%d", s.channel);
			dl->AddText(g_fontTitle, g_fontTitle->LegacySize, ImVec2(x, c0.y + 14.0f),
				Pal::U32(Pal::Text), channel.c_str());
			const std::string id = s.id != 0 ? LighthouseVisibility::IdName(s.id) : std::string(Tr("id not logged yet"));
			dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
				ImVec2(x + TextWidth(g_fontTitle, channel.c_str()) + 10.0f,
					c0.y + 14.0f + g_fontTitle->LegacySize - g_fontSmall->LegacySize - 3.0f),
				Pal::U32(Pal::Dim), id.c_str());

			const std::string seenLine = reporting == 0 ? std::string("No reports yet")
				: FormatString("Seen by %d of %d", seeing, reporting);
			dl->AddText(g_fontBody, g_fontBody->LegacySize, ImVec2(x, c1.y - 56.0f),
				Pal::U32(unseen ? Pal::Bad : Pal::Text), Tr(seenLine.c_str()));

			// A loss is a drop that left a device with no station at all.
			std::string dropLine = s.drops == 0 ? std::string("No dropouts this session")
				: FormatString("Dropped out %u time%s", s.drops, s.drops == 1 ? "" : "s");
			if (s.losses > 0)
				dropLine += FormatString(" \xC2\xB7 the last one left %u time%s", s.losses, s.losses == 1 ? "" : "s");
			dl->AddText(g_fontSmall, g_fontSmall->LegacySize, ImVec2(x, c1.y - 28.0f),
				Pal::U32(Pal::Dim), Tr(dropLine.c_str()));
		}
		ImGui::Dummy(ImVec2(cw, cardH));
	}

	// ---- What each device sees ----
	ImGui::Spacing();
	ImGui::Spacing();
	SectionLabel("IN VIEW OF EACH DEVICE");
	const float rowH = 52.0f;
	const float listH = rowH * (float)devices.size();
	const ImVec2 lp = BeginRowCard(listH);
	const int clean = CalCtx.lighthouse.Settings().cleanStations;
	const float rightW = 150.0f;                 // the figure and its drop count
	const float dotStep = 36.0f;
	const float dotsW = dotStep * (float)stations.size();
	const float dotsX = lp.x + cw - 16.0f - rightW - 24.0f - dotsW;
	for (size_t i = 0; i < devices.size(); ++i)
	{
		const VRDevice &dev = *devices[i];
		const auto *seen = CalCtx.lighthouse.Find(dev.serial);
		const bool reporting = dev.connected && seen && seen->visibleKnown;
		ImGui::PushID(dev.id);
		const ImVec2 p = ImVec2(lp.x, lp.y + rowH * (float)i);
		const ImVec2 b = ImVec2(p.x + cw, p.y + rowH);
		if (i > 0)
			dl->AddLine(ImVec2(p.x + 16.0f, p.y), ImVec2(b.x - 16.0f, p.y), Pal::U32(Pal::Border), 1.0f);

		// The row is one hover target for its tooltip; nothing on it is
		// pressed.
		ImGui::SetCursorScreenPos(p);
		ImGui::InvisibleButton("row", ImVec2(cw, rowH));
		const bool hov = ImGui::IsItemHovered();
		if (hov)
			dl->AddRectFilled(p, b, Pal::U32(ImVec4(1, 1, 1, 0.03f)), 11.0f,
				(i == 0 ? ImDrawFlags_RoundCornersTop : 0) |
				(i + 1 == devices.size() ? ImDrawFlags_RoundCornersBottom : 0));

		const ImVec4 ink = dev.connected ? Pal::Text : Pal::Dim;
		RowDeviceIcon(dl, dev, ImVec2(p.x + 30.0f, p.y + rowH * 0.5f), Pal::U32(Pal::Dim));
		const std::string name = DeviceDisplayName(dev);
		dl->AddText(g_fontBody, g_fontBody->LegacySize, ImVec2(p.x + 62.0f, p.y + 7.0f),
			Pal::U32(ink), name.c_str());
		dl->AddText(g_fontSmall, g_fontSmall->LegacySize, ImVec2(p.x + 62.0f, p.y + 31.0f),
			Pal::U32(Pal::Faint), dev.serial.c_str());

		// One dot per station, in the strip's order: filled while this
		// device has it in view.
		for (size_t k = 0; k < stations.size(); ++k)
		{
			const int channel = stations[k].channel;
			const ImVec2 c = ImVec2(dotsX + dotStep * (float)k + dotStep * 0.5f, p.y + 18.0f);
			if (reporting && Sees(*seen, channel))
				dl->AddCircleFilled(c, 5.5f, Pal::U32(Pal::Good), 20);
			else
				dl->AddCircle(c, 5.5f, Pal::U32(reporting ? Pal::Dim : Pal::Faint), 20, 1.5f);
			const std::string label = FormatString("S-%d", channel);
			dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
				ImVec2(c.x - TextWidth(g_fontSmall, label.c_str()) * 0.5f, p.y + 28.0f),
				Pal::U32(Pal::Faint), label.c_str());
		}

		// The figure at the right: how many in view, coloured by whether
		// that is enough for a solid solution, with the drop count under it.
		std::string figure, detail;
		ImVec4 figureColor = Pal::Dim;
		if (!dev.connected)
		{
			figure = "Off";
		}
		else if (!reporting)
		{
			figure = "Not in the log yet";
		}
		else
		{
			const int inView = seen->InView();
			const int total = std::max(CalCtx.lighthouse.StationCount(), inView);
			figure = FormatString("%d of %d in view", inView, total);
			figureColor = inView < clean ? Pal::Bad : Pal::Text;
			detail = seen->drops == 0 ? std::string("no dropouts")
				: FormatString("%u dropout%s", seen->drops, seen->drops == 1 ? "" : "s");
			if (seen->losses > 0)
				detail += FormatString(", %u full loss%s", seen->losses, seen->losses == 1 ? "" : "es");
		}
		figure = Tr(figure);
		detail = Tr(detail);
		const float fx = b.x - 16.0f - TextWidth(g_fontBody, figure.c_str());
		dl->AddText(g_fontBody, g_fontBody->LegacySize,
			ImVec2(fx, p.y + (detail.empty() ? (rowH - g_fontBody->LegacySize) * 0.5f : 7.0f)),
			Pal::U32(figureColor), figure.c_str());
		if (!detail.empty())
			dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
				ImVec2(b.x - 16.0f - TextWidth(g_fontSmall, detail.c_str()), p.y + 31.0f),
				Pal::U32(Pal::Dim), detail.c_str());

		if (hov && reporting)
		{
			std::string tip = "In view:";
			if (seen->InView() == 0)
				tip += " no base station";
			for (int c : seen->visible)
				tip += " " + CalCtx.lighthouse.StationName(c);
			for (uint32_t id : seen->unmappedIds)
				tip += " " + LighthouseVisibility::IdName(id);
			if (!seen->lastDisturbanceText.empty())
				tip += "\nLast change: " + seen->lastDisturbanceText;
			if (seen->bootstraps > 0)
				tip += FormatString("\nTracking restarted %u time%s", seen->bootstraps, seen->bootstraps == 1 ? "" : "s");
			ShowTip(tip.c_str(), true);
		}
		ImGui::PopID();
	}
	EndRowCard(lp, listH);

	// ---- What the log did for the drift monitor ----
	ImGui::Spacing();
	ImGui::PushFont(g_fontSmall);
	if (CalCtx.lighthouseAttributedEvents > 0)
		ImGui::TextColored(Pal::Dim, "%s", Tr(CalCtx.lighthouseAttributedEvents == 1
			? std::string("1 drift check this session was skipped because a base station had just changed.")
			: FormatString("%u drift checks this session were skipped because a base station had just changed.",
				CalCtx.lighthouseAttributedEvents)).c_str());
	else
		ImGui::TextColored(Pal::Dim, "%s", Tr("Base station changes haven't affected drift checks this session."));
	if (!CalCtx.lighthouseLogPath.empty())
		ImGui::TextColored(Pal::Faint, "%s", Tr(FormatString("Read from %s", CalCtx.lighthouseLogPath.c_str())).c_str());
	ImGui::PopFont();
}
