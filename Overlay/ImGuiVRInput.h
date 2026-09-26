#pragma once

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

namespace imgui_vr
{
inline const char *ActiveText()
{
	const auto *state = ImGui::GetInputTextState(ImGui::GetActiveID());
	return state && state->TextA.Data ? state->TextA.Data : "";
}

// Use the widget's input path so UTF-8, capacity, numeric filters and undo
// retain their normal behavior. Keep the widget active until the queue drains.
inline bool ReplaceActiveText(const char *text, ImGuiID owner)
{
	auto *state = ImGui::GetInputTextState(owner);
	if (!state || ImGui::GetActiveID() != owner)
		return false;
	state->SelectAll();
	auto &io = ImGui::GetIO();
	if (*text)
		io.AddInputCharactersUTF8(text);
	else
	{
		io.AddKeyEvent(ImGuiKey_Backspace, true);
		io.AddKeyEvent(ImGuiKey_Backspace, false);
	}
	return true;
}

inline bool HasPendingInput()
{
	return !GImGui->InputEventsQueue.empty();
}

inline void DesktopFocusEvent(bool focused, bool dashboardVisible)
{
	// AddFocusEvent bypasses SetAppAcceptingEvents in ImGui. Desktop focus
	// changes must not clear the active VR widget or its pending input.
	if (!dashboardVisible)
		ImGui::GetIO().AddFocusEvent(focused);
}

inline void ChangeInputSource(bool dashboardVisible, bool desktopFocused)
{
	auto &io = ImGui::GetIO();
	io.ClearEventsQueue();
	io.ClearInputKeys();
	io.ClearInputMouse();
	io.AddFocusEvent(dashboardVisible || desktopFocused);
}
}
