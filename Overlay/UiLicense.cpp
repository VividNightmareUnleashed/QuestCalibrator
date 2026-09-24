// The license agreement screen: shown instead of everything else until the
// player has agreed to the embedded LICENSE, through the installer or here.
// A manual install never runs the installer, so this is where it agrees.
#include "stdafx.h"
#include "UiInternal.h"
#include "LicenseAgreement.h"

// -uipreview-license shows the screen whatever the registry says, and agreeing
// there only dismisses it: a preview never records anything.
static bool s_previewAgreed = false;

bool LicenseScreenWanted()
{
	if (g_uiPreviewMode)
		return g_uiPreviewScenario == PreviewScenario::License && !s_previewAgreed;
	return !questcal::license::Accepted();
}

void BuildLicenseScreen(bool runningInOverlay)
{
	// The installer asks the same single question.
	const char *const quitLabel = "Decline and quit";
	const char *const agreeLabel = "Agree and continue";
	const float buttonH = 40.0f;
	const float gap = 12.0f;

	ImGui::PushFont(g_fontTitle);
	ImGui::TextUnformatted(Tr("License agreement"));
	ImGui::PopFont();
	ImGui::PushStyleColor(ImGuiCol_Text, Pal::Dim);
	ImGui::TextWrapped("%s", Tr("QuestCalibrator is free to use under this agreement. Read it, then agree to continue."));
	if (questcal::i18n::CurrentLanguage() != questcal::i18n::Language::English)
		ImGui::TextWrapped("%s", Tr("The agreement is in English only."));
	ImGui::PopStyleColor();
	ImGui::Spacing();

	// The text scrolls in its own inset; the buttons and the footer stay
	// pinned under it.
	const float reserve = buttonH + 40.0f + 36.0f;
	ImGui::PushStyleColor(ImGuiCol_ChildBg, Pal::Inset);
	ImGui::PushStyleColor(ImGuiCol_Border, Pal::Border);
	ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 10.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(22.0f, 18.0f));
	ImGui::BeginChild("##licensetext",
		ImVec2(0.0f, ImGui::GetWindowHeight() - ImGui::GetCursorPosY() - reserve),
		ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding);
	const std::string &text = questcal::license::Text();
	ImGui::PushTextWrapPos(0.0f);
	if (text.empty())
		ImGui::TextUnformatted(Tr("The license agreement couldn't load. Reinstall QuestCalibrator to restore it."));
	else
		ImGui::TextUnformatted(text.c_str(), text.c_str() + text.size());
	ImGui::PopTextWrapPos();
	ImGui::EndChild();
	ImGui::PopStyleVar(2);
	ImGui::PopStyleColor(2);
	ImGui::Spacing();

	const float quitW = ButtonWidthFor(quitLabel, false, 180.0f);
	const float agreeW = ButtonWidthFor(agreeLabel, true, 200.0f);
	ImGui::SetCursorPosX(ImGui::GetWindowWidth() - ImGui::GetStyle().WindowPadding.x - quitW - gap - agreeW);
	if (IconButton("##licensequit", quitLabel, nullptr, ImVec2(quitW, buttonH), BtnKind::Ghost))
		RequestApplicationExit();
	ImGui::SameLine(0.0f, gap);
	if (IconButton("##licenseagree", agreeLabel, IconCheck, ImVec2(agreeW, buttonH), BtnKind::Primary))
	{
		if (g_uiPreviewMode)
		{
			s_previewAgreed = true;
		}
		else
		{
			std::string detail;
			if (questcal::license::RecordAcceptance(detail))
			{
				AppendSessionLog("license: agreed in the overlay");
			}
			else
			{
				AppendSessionLog("license: agreed in the overlay, but not saved: " + detail);
				CalCtx.ReportError("Couldn't save your agreement to the license, so QuestCalibrator will ask again next time it starts.");
			}
		}
	}

	BuildFooter(runningInOverlay);
}
