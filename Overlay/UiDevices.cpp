// Device rows, the two tracking-system panes and the live VR state query.
#include "stdafx.h"
#include "UiInternal.h"

// ---------------------------------------------------------------------------
// Device cards
// ---------------------------------------------------------------------------

void DeviceIcon(ImDrawList *dl, const VRDevice &dev, ImVec2 c, float s, ImU32 col)
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

// Player-given names. The row shows the name in place of the model, with
// the model and serial beneath; everywhere else a device is named, the same
// lookup applies so "Hip" is "Hip" in the tracker pick and the guide too.
const std::string *FindDeviceName(const std::string &serial)
{
	auto it = CalCtx.deviceNames.find(serial);
	return it != CalCtx.deviceNames.end() && !it->second.empty() ? &it->second : nullptr;
}

std::string DeviceDisplayName(const VRDevice &dev)
{
	const std::string *name = FindDeviceName(dev.serial);
	return name ? *name : dev.model;
}

// Inline rename: one row at a time.
static int s_renameDeviceId = -1;
static bool s_renameFocus = false;
static char s_renameBuf[CalibrationContext::DeviceNameMaxBytes + 1] = {};

void CommitDeviceName(const VRDevice &dev, const char *text)
{
	std::string name(text);
	while (!name.empty() && isspace(static_cast<unsigned char>(name.back())))
		name.pop_back();
	size_t start = 0;
	while (start < name.size() && isspace(static_cast<unsigned char>(name[start])))
		++start;
	name.erase(0, start);

	auto previous = CalCtx.deviceNames;
	if (name.empty() || name == dev.model)
		CalCtx.deviceNames.erase(dev.serial);
	else if (CalCtx.deviceNames.count(dev.serial) ||
		CalCtx.deviceNames.size() < CalibrationContext::DeviceNameMaxCount)
		CalCtx.deviceNames[dev.serial] = name;
	if (CalCtx.deviceNames != previous)
		SaveSettingOrRestore(CalCtx.deviceNames, previous);
}

// A flat row inside the list container: no chrome of its own — an inset
// divider above (except the first), a hover wash, and selection as an accent
// rail + tint. Rounding only ever belongs to the container's outer corners.
bool DeviceRow(const VRDevice &dev, bool selected, float w, bool first, bool last)
{
	const float h = 52.0f;
	ImGui::PushID(dev.id);
	ImVec2 p = ImGui::GetCursorScreenPos();
	// The rename and identify tools sit on top of the row and must win the
	// hover.
	ImGui::SetNextItemAllowOverlap();
	bool pressed = ImGui::InvisibleButton("row", ImVec2(w, h), ImGuiButtonFlags_EnableNav);
	bool hov = ImGui::IsItemHovered();
	const bool rowFocused = ImGui::IsItemFocused();
	ImDrawList *dl = ImGui::GetWindowDrawList();
	ImVec2 b = ImVec2(p.x + w, p.y + h);

	if (!first)
		dl->AddLine(ImVec2(p.x + 16.0f, p.y), ImVec2(b.x - 16.0f, p.y), Pal::U32(Pal::Border), 1.0f);

	int corners = (first ? ImDrawFlags_RoundCornersTop : 0) | (last ? ImDrawFlags_RoundCornersBottom : 0);
	if (!corners)
		corners = ImDrawFlags_RoundCornersNone;
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
	const bool low = dev.connected && dev.battery >= 0.0f &&
		dev.battery < kLowBattery && !dev.charging;
	float textRight = b.x - 16.0f;
	if (dev.connected && dev.battery >= 0.0f)
	{
		const float bw = 26.0f, bh = 12.0f, nub = 2.5f;
		ImVec2 g0 = ImVec2(b.x - 16.0f - bw - nub, p.y + (h - bh) * 0.5f);
		ImVec2 g1 = ImVec2(g0.x + bw, g0.y + bh);
		dl->AddRect(g0, g1, Pal::U32(Pal::Faint), 3.0f, 1.0f, ImDrawFlags_RoundCornersAll);
		dl->AddRectFilled(ImVec2(g1.x + 1.0f, iconC.y - 2.5f),
			ImVec2(g1.x + 1.0f + nub, iconC.y + 2.5f), Pal::U32(Pal::Faint), 1.0f);
		float level = dev.battery > 1.0f ? 1.0f : dev.battery;
		float fw = (bw - 4.0f) * level;
		if (fw > 0.5f)
			dl->AddRectFilled(ImVec2(g0.x + 2.0f, g0.y + 2.0f), ImVec2(g0.x + 2.0f + fw, g1.y - 2.0f),
				Pal::U32(low ? Pal::Bad : Pal::Good), 1.5f);
		textRight = g0.x - 10.0f;
	}

	// The state in words, next to the glyphs that only hint at it: a dimmer
	// row and a red pill are not something a player can be expected to read.
	const char *stateWord = !dev.connected ? "Off" : (low ? "Low battery" : nullptr);
	if (stateWord)
	{
		ImGui::PushFont(g_fontSmall);
		ImVec2 sw = ImGui::CalcTextSize(stateWord);
		ImGui::PopFont();
		float sx = textRight - sw.x;
		dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
			ImVec2(sx, p.y + (h - g_fontSmall->LegacySize) * 0.5f),
			Pal::U32(dev.connected ? Pal::Bad : Pal::Dim), stateWord);
		textRight = sx - 10.0f;
	}

	// Two small tools, quiet at rest and bright on hover: rename, and buzz
	// this one device. With six identical trackers these are what tell the
	// rows apart.
	{
		// A headset can't buzz or blink on command, so it gets rename only.
		const bool canBuzz = dev.deviceClass != vr::TrackedDeviceClass_HMD;
		const float bs = 26.0f;
		const float toolsW = canBuzz ? bs * 2.0f + 8.0f : bs;
		float bx = textRight - toolsW;
		const float by = p.y + (h - bs) * 0.5f;
		auto tool = [&](const char *id, IconFn icon, float iconSize, const char *tip) {
			ImGui::SetCursorScreenPos(ImVec2(bx, by));
			bool pressedTool = ImGui::InvisibleButton(id, ImVec2(bs, bs), ImGuiButtonFlags_EnableNav);
			bool hovTool = ImGui::IsItemHovered();
			bool focused = ImGui::IsItemFocused();
			if (hovTool || focused)
				dl->AddRectFilled(ImVec2(bx, by), ImVec2(bx + bs, by + bs),
					Pal::U32(ImVec4(1, 1, 1, 0.08f)), 6.0f);
			if (focused)
				dl->AddRect(ImVec2(bx, by), ImVec2(bx + bs, by + bs), Pal::U32(Pal::Accent), 6.0f, 2.0f, ImDrawFlags_RoundCornersNone);
			// No tip while the field is open: it would sit on the field.
			if (hovTool && s_renameDeviceId != dev.id)
				ShowTip(tip, true);
			icon(dl, ImVec2(bx + bs * 0.5f, by + bs * 0.5f), iconSize,
				Pal::U32(hovTool || focused ? Pal::Text : Pal::Faint));
			bx += bs + 8.0f;
			return pressedTool;
		};
		if (tool("rename", IconPencil, 9.0f, "Rename this device"))
		{
			s_renameDeviceId = dev.id;
			const std::string *current = FindDeviceName(dev.serial);
			strncpy_s(s_renameBuf, current ? current->c_str() : "", _TRUNCATE);
			s_renameFocus = true;
		}
		if (canBuzz && tool("buzz", IconCrosshair, 7.5f, "Vibrate or blink this device"))
		{
			g_identifyPulse.active = true;
			g_identifyPulse.targetId = static_cast<uint32_t>(dev.id);
			g_identifyPulse.referenceId = vr::k_unTrackedDeviceIndexInvalid;
			g_identifyPulse.pulsesRemaining = 100;
			g_identifyPulse.nextPulseTime = ImGui::GetTime();
		}
		textRight -= toolsW + 10.0f;
	}

	ImVec4 clip = ImVec4(p.x, p.y, textRight, b.y);
	float tx = p.x + 58.0f;
	if (s_renameDeviceId == dev.id)
	{
		// The field takes the name line only; the model and serial stay
		// beneath it, because with six identical trackers they are what
		// says which one is being named.
		ImGui::SetCursorScreenPos(ImVec2(tx - 6.0f, p.y + 3.0f));
		ImGui::PushItemWidth(std::max(140.0f, textRight - tx - 2.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6, 3));
		if (s_renameFocus)
		{
			ImGui::SetKeyboardFocusHere();
			s_renameFocus = false;
		}
		bool entered = ImGui::InputText("##rename", s_renameBuf, sizeof s_renameBuf,
			ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
		// The VR keyboard may finish without an Enter, so leaving the field
		// commits too; Escape reverts the buffer first, which commits no change.
		bool finished = entered || ImGui::IsItemDeactivated();
		ImGui::PopStyleVar();
		ImGui::PopItemWidth();
		if (s_renameBuf[0] == '\0')
			dl->AddText(g_fontBody, g_fontBody->LegacySize, ImVec2(tx, p.y + 6.0f),
				Pal::U32(Pal::Faint), "Hip, Left foot, Chest...");
		std::string sub = dev.model + "  " + dev.serial;
		dl->AddText(g_fontSmall, g_fontSmall->LegacySize, ImVec2(tx, p.y + 31.0f),
			Pal::U32(Pal::Dim), sub.c_str(), nullptr, 0.0f, &clip);
		if (finished)
		{
			CommitDeviceName(dev, s_renameBuf);
			s_renameDeviceId = -1;
		}
	}
	else if (const std::string *name = FindDeviceName(dev.serial))
	{
		dl->AddText(g_fontBody, g_fontBody->LegacySize, ImVec2(tx, p.y + 6.0f),
			Pal::U32(dev.connected ? Pal::Text : Pal::Dim), name->c_str(), nullptr, 0.0f, &clip);
		std::string sub = dev.model + "  " + dev.serial;
		dl->AddText(g_fontSmall, g_fontSmall->LegacySize, ImVec2(tx, p.y + 31.0f),
			Pal::U32(dev.connected ? Pal::Dim : Pal::Faint), sub.c_str(), nullptr, 0.0f, &clip);
	}
	else
	{
		dl->AddText(g_fontBody, g_fontBody->LegacySize, ImVec2(tx, p.y + 6.0f),
			Pal::U32(dev.connected ? Pal::Text : Pal::Dim), dev.model.c_str(), nullptr, 0.0f, &clip);
		dl->AddText(g_fontSmall, g_fontSmall->LegacySize, ImVec2(tx, p.y + 31.0f),
			Pal::U32(dev.connected ? Pal::Dim : Pal::Faint), dev.serial.c_str(), nullptr, 0.0f, &clip);
	}

	if (rowFocused)
		dl->AddRect(ImVec2(p.x + 2.0f, p.y + 2.0f), ImVec2(b.x - 2.0f, b.y - 2.0f),
			Pal::U32(Pal::Accent), 6.0f, 2.0f, ImDrawFlags_RoundCornersAll);

	// The tools and the rename field moved the cursor; the next row starts
	// where this one ends, as it did when the row was a single button.
	ImGui::SetCursorScreenPos(ImVec2(p.x, b.y));
	ImGui::PopID();
	return pressed;
}

// Preferred default: left-hand controller, else first device of the system.
// Reconciles the context's selection against the live device list in place --
// it is the only owner of that selection, so there is no mirror to go stale
// when a tracking system disappears between frames.
void EnsureDeviceSelection(const VRState &state, uint32_t &selected, const std::string &system)
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

void BuildDeviceList(const VRState &state, uint32_t &selected, const std::string &system, float paneW)
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
		const char *msg = "No devices connected";
		ImVec2 ts = ImGui::CalcTextSize(msg);
		dl->AddText(g_fontBody, g_fontBody->LegacySize,
			ImVec2(origin.x + (paneW - ts.x) * 0.5f, origin.y + (rowH - ts.y) * 0.5f),
			Pal::U32(Pal::Dim), msg);
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
std::string FriendlySystemName(const std::string &raw)
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
void PickTrackingSystem(const char *id, const std::vector<std::string> &candidates, int fallback, float width, std::string &selection)
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

	if (candidates.size() <= 1)
	{
		// One system is not a choice: a combo that can only return its own
		// value still looks like something to open. Same frame as the combo
		// beside it, minus the arrow.
		ImVec2 p = ImGui::GetCursorScreenPos();
		float h = ImGui::GetFrameHeight();
		ImDrawList *dl = ImGui::GetWindowDrawList();
		dl->AddRectFilled(p, ImVec2(p.x + width, p.y + h), Pal::U32(Pal::Card), 9.0f);
		dl->AddRect(p, ImVec2(p.x + width, p.y + h), Pal::U32(Pal::Border), 9.0f);
		ImGui::Dummy(ImVec2(width, h));
		if (!items.empty())
			dl->AddText(g_fontBody, g_fontBody->LegacySize,
				ImVec2(p.x + 14.0f, p.y + (h - g_fontBody->LegacySize) * 0.5f),
				Pal::U32(Pal::Text), items[0]);
	}
	else
	{
		ImGui::PushItemWidth(width);
		ImGui::Combo(id, &current, items.data(), (int)items.size());
		ImGui::PopItemWidth();
	}

	if (current >= 0 && current < (int)candidates.size())
		selection = candidates[current];
}

void BuildSpacesSection(const VRState &state)
{
	if (state.trackingSystems.empty())
	{
		const float h = 120.0f;
		ImVec2 p = BeginRowCard(h);
		ImDrawList *dl = ImGui::GetWindowDrawList();
		float cw = ImGui::GetContentRegionAvail().x;
		const char *msg = "No tracked devices found";
		ImVec2 ts = ImGui::CalcTextSize(msg);
		IconHMD(dl, ImVec2(p.x + cw * 0.5f, p.y + 42.0f), 16.0f, Pal::U32(Pal::Faint));
		dl->AddText(g_fontBody, g_fontBody->LegacySize,
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

	float cw = ImGui::GetContentRegionAvail().x;
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
		ImGui::TextColored(Pal::Dim, "No trackers found. Turn on a tracker and make sure SteamVR sees it.");
		// The pane is showing nothing, so nothing may stay selected: this id is
		// what the identify pulse buzzes and what StartCalibration freezes.
		CalCtx.targetID = vr::k_unTrackedDeviceIndexInvalid;
	}
	ImGui::EndGroup();
	float rightBottom = ImGui::GetItemRectMax().y;

	ImGui::SetCursorScreenPos(ImVec2(top.x, (leftBottom > rightBottom ? leftBottom : rightBottom)));
	ImGui::Dummy(ImVec2(0, 0));

	// ---- What happens next, then Identify ----
	ImGui::Spacing();
	{
		std::string hint = FormatString(
			"Hold both devices together and rotate them for %.0f seconds.",
			CalCtx.CollectionSeconds());
		ImGui::PushFont(g_fontSmall);
		ImVec2 hs = ImGui::CalcTextSize(hint.c_str());
		ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, (cw - hs.x) * 0.5f));
		ImGui::TextColored(Pal::Dim, "%s", hint.c_str());
		ImGui::PopFont();
	}
}

// Vibrate or blink the two picks. A real button on the main screen: styled
// as a caption it was the one tool that tells six identical trackers apart,
// and it read as a section label.
void IdentifyButton(ImVec2 size)
{
	bool identifyPressed = IconButton("identify", "Identify selected devices", IconCrosshair,
		size, BtnKind::Ghost);
	if (ImGui::IsItemHovered())
		ShowTip("Vibrates or blinks the two selected devices so you can tell which is which.");
	if (identifyPressed)
	{
		g_identifyPulse.active = true;
		g_identifyPulse.targetId = CalCtx.targetID;
		g_identifyPulse.referenceId = CalCtx.referenceID;
		g_identifyPulse.pulsesRemaining = 100;
		g_identifyPulse.nextPulseTime = ImGui::GetTime();
	}
}

VRState LoadVRState()
{
	VRState state;
	state.trackingSystems.reserve(vr::k_unMaxTrackedDeviceCount);
	state.devices.reserve(vr::k_unMaxTrackedDeviceCount);

	if (g_uiPreviewMode)
		return PreviewVRState();

	if (!vr::VRSystem())
		return state;

	// One pose query per refresh for the readiness dots in the guide: raw
	// universe, no prediction, so it costs the same as the property reads.
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
		vr::TrackingUniverseRawAndUncalibrated, 0.0f, poses, vr::k_unMaxTrackedDeviceCount);

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
				device.tracking = device.connected && poses[id].bPoseIsValid &&
					poses[id].eTrackingResult == vr::TrackingResult_Running_OK;

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
