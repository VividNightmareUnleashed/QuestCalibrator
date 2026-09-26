#include "../Overlay/ImGuiVRInput.h"

#include <cstring>

namespace
{
struct UiContext
{
	bool focusLost = false;
	UiContext()
	{
		ImGui::CreateContext();
		auto &io = ImGui::GetIO();
		io.DisplaySize = ImVec2(640, 480);
		io.IniFilename = nullptr;
		unsigned char *pixels;
		int width, height;
		io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
	}
	~UiContext() { ImGui::DestroyContext(); }
	void Frame(char *buffer, size_t capacity, bool focus = false, ImGuiInputTextFlags flags = 0)
	{
		ImGui::NewFrame();
		focusLost = ImGui::GetIO().AppFocusLost;
		ImGui::SetNextWindowPos(ImVec2(0, 0));
		ImGui::SetNextWindowSize(ImVec2(640, 480));
		ImGui::Begin("Test");
		if (focus)
			ImGui::SetKeyboardFocusHere();
		ImGui::InputText("Name", buffer, capacity, flags);
		ImGui::End();
		ImGui::Render();
	}
	void Focus(char *buffer, size_t capacity, ImGuiInputTextFlags flags = 0)
	{
		Frame(buffer, capacity, true, flags);
		Frame(buffer, capacity, false, flags);
		Frame(buffer, capacity, false, flags);
	}
};
}

void RunImGuiInputScenarios(void (*check)(const char *, bool, const char *))
{
	{
		UiContext context;
		char buffer[32] = "Old tracker";
		context.Focus(buffer, sizeof buffer);
		check("ImGui: VR reads active text", std::strcmp(imgui_vr::ActiveText(), buffer) == 0, "");
		const char *replacement = "Left caf\xc3\xa9";
		imgui_vr::ReplaceActiveText(replacement, ImGui::GetActiveID());
		context.Frame(buffer, sizeof buffer);
		check("ImGui: VR replaces UTF-8 text", std::strcmp(buffer, replacement) == 0 && !imgui_vr::HasPendingInput(), "");
		imgui_vr::ReplaceActiveText("", ImGui::GetActiveID());
		context.Frame(buffer, sizeof buffer);
		context.Frame(buffer, sizeof buffer);
		check("ImGui: VR clears text", buffer[0] == 0 && !imgui_vr::HasPendingInput(), "");
		check("ImGui: stale keyboard result ignored",
			!imgui_vr::ReplaceActiveText("stale", ImGui::GetActiveID() + 1) && !imgui_vr::HasPendingInput(), "");
	}
	{
		UiContext context;
		char buffer[5] = "old";
		context.Focus(buffer, sizeof buffer);
		imgui_vr::ReplaceActiveText("\xc3\xa9\xc3\xa9\xc3\xa9", ImGui::GetActiveID());
		context.Frame(buffer, sizeof buffer);
		check("ImGui: UTF-8 capacity boundary", std::strcmp(buffer, "\xc3\xa9\xc3\xa9") == 0, "");
	}
	{
		UiContext context;
		char buffer[32] = "12";
		context.Focus(buffer, sizeof buffer, ImGuiInputTextFlags_CharsDecimal);
		imgui_vr::ReplaceActiveText("-3.5x", ImGui::GetActiveID());
		context.Frame(buffer, sizeof buffer, false, ImGuiInputTextFlags_CharsDecimal);
		check("ImGui: VR respects numeric filter", std::strcmp(buffer, "-3.5") == 0, "");
	}
	{
		UiContext context;
		char buffer[32] = "Old";
		context.Focus(buffer, sizeof buffer);
		auto &io = ImGui::GetIO();
		io.AddMouseButtonEvent(0, true);
		io.AddKeyEvent(ImGuiKey_A, true);
		context.Frame(buffer, sizeof buffer);
		imgui_vr::ChangeInputSource(true, false);
		io.SetAppAcceptingEvents(false);
		io.AddMousePosEvent(11, 12);
		imgui_vr::DesktopFocusEvent(false, true);
		io.SetAppAcceptingEvents(true);
		io.AddMousePosEvent(101, 102);
		context.Frame(buffer, sizeof buffer);
		check("ImGui: VR takes input ownership", !io.MouseDown[0] && !ImGui::IsKeyDown(ImGuiKey_A) &&
			!io.AppFocusLost && io.MousePos.x == 101 && io.MousePos.y == 102, "");
		imgui_vr::ChangeInputSource(false, false);
		context.Frame(buffer, sizeof buffer);
		check("ImGui: desktop focus restored", context.focusLost, "");
	}
}
