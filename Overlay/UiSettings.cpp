// The settings screen and the advanced profile editor.
#include "stdafx.h"
#include "UiInternal.h"
#include "Diagnostics.h"

void BuildSettingsScreen(const VRState &state)
{
	float cw = ImGui::GetContentRegionAvail().x;
	const float gap = 12.0f;
	(void)gap;

	{
		// No "SETTINGS" eyebrow: the lit gear in the pinned header and the
		// list of switches already say where this is.
		ImGui::Spacing();

		// Advanced mode
		{
			bool previous = CalCtx.uiAdvanced;
			if (ToggleRow("##uiAdvanced", IconGauge, "Advanced mode", CalCtx.uiAdvanced,
				"Shows calibration measurements, drift readings and extra settings."))
				SaveSettingOrRestore(CalCtx.uiAdvanced, previous);
		}

		// Alignment notifications: stale calibration, paused continuous calibration,
		// noisy tracking. One switch for all three toasts.
		{
			bool previous = CalCtx.notifyPoorCalibration;
			if (ToggleRow("##notifyPoorCalibration", IconInfo,
				"Alignment notifications in VR", CalCtx.notifyPoorCalibration,
				"A SteamVR notification when the calibration looks off or continuous calibration pauses."))
				SaveSettingOrRestore(CalCtx.notifyPoorCalibration, previous);
		}

		// Spatial correction field
		if (CalCtx.validProfile)
		{
			size_t anchorCount = CalCtx.fieldAnchors.size();
			// One line per anchor for the advanced reader; a count otherwise.
			const size_t anchorLines = anchorCount == 0 ? 0 : CalCtx.uiAdvanced ? anchorCount : 1;
			RowCard row(kRowHeight + kRowSubLineH + (anchorLines > 0 ? anchorLines * 24.0f + 6.0f : 0.0f));
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
			RowIconLabel(p, IconField, "Spatial correction field");
			RowSubLine(p, "Uses saved field anchors to correct alignment in different parts of the room.");

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
				if (!CalCtx.uiAdvanced)
				{
					std::string line = FormatString("%zu anchor%s saved", anchorCount, anchorCount == 1 ? "" : "s");
					dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
						ImVec2(p.x + 92.0f, p.y + kRowHeight + kRowSubLineH), Pal::U32(Pal::Dim), line.c_str());
				}
				for (size_t i = 0; CalCtx.uiAdvanced && i < CalCtx.fieldAnchors.size(); ++i)
				{
					const auto &a = CalCtx.fieldAnchors[i];
					// Delta vs base, evaluated at the anchor's own spot.
					Eigen::Vector3d targetPt = a.rotation.conjugate() * (a.position - a.translationMeters);
					Eigen::Vector3d basePos = CalCtx.transform.rotation * targetPt + CalCtx.transform.translationMeters;
					double posDeltaCm = (a.position - basePos).norm() * 100.0;
					double rotDeltaDeg = a.rotation.angularDistance(CalCtx.transform.rotation) * 180.0 / EIGEN_PI;
					std::string line = FormatString("Anchor %zu at (%+.1f, %+.1f): %.1f cm / %.2f deg from base",
						i + 1, a.position.x(), a.position.z(), posDeltaCm, rotDeltaDeg);
					dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
						ImVec2(p.x + 92.0f, p.y + kRowHeight + kRowSubLineH + i * 24.0f), Pal::U32(Pal::Dim), line.c_str());
				}
			}
		}

		// Solve playspace scale
		{
			bool previous = CalCtx.solveScale;
			if (ToggleRow("##solveScale", IconScale,
				"Solve playspace scale (experimental)", CalCtx.solveScale,
				"Measures whether the two systems disagree on distances. Leave off unless you know you need it."))
				SaveSettingOrRestore(CalCtx.solveScale, previous);
		}

		// Time offset (+ the manual override spike tool, advanced mode only)
		{
			const float nestedH = CalCtx.uiAdvanced ? 52.0f : 0.0f;
			RowCard row(kRowHeight + kRowSubLineH + nestedH);
			const ImVec2 p = row.pos;
			ImDrawList *dl = ImGui::GetWindowDrawList();

			ImGui::SetCursorScreenPos(ImVec2(p.x + kRowInsetX, p.y + kRowControlY));
			bool previous = CalCtx.applyTimeOffset;
			if (QCCheckbox("##applyTimeOffset", &CalCtx.applyTimeOffset))
				SaveSettingOrRestore(CalCtx.applyTimeOffset, previous);
			RowIconLabel(p, IconClock, "Apply time offset");
			RowSubLine(p, "Compensates for the tracking delay measured during calibration.");

			// The number is for the advanced reader; the switch is the setting.
			if (CalCtx.uiAdvanced && CalCtx.validProfile && (CalCtx.applyTimeOffset || CalCtx.useManualTimeOffset))
			{
				std::string applied = FormatString("%+.1f ms", CalCtx.appliedTimeOffset * 1000.0);
				ImVec2 ts = ImGui::CalcTextSize(applied.c_str());
				dl->AddText(g_fontBody, g_fontBody->LegacySize,
					ImVec2(p.x + cw - kRowInsetX - ts.x, p.y + 26.0f - g_fontBody->LegacySize * 0.5f),
					Pal::U32(Pal::Dim), applied.c_str());
			}

			// Nested inset: manual override spike tool (verifies the poseTimeOffset
			// sign convention against a live session; bypasses the solved value).
			// Developer scaffolding, so it exists only in advanced mode.
			if (CalCtx.uiAdvanced)
			{
				ImVec2 np = ImVec2(p.x + 12.0f, p.y + kRowHeight + kRowSubLineH);
				ImVec2 nb = ImVec2(p.x + cw - 12.0f, p.y + row.height - 10.0f);
				dl->AddRectFilled(np, nb, Pal::U32(Pal::Inset), 9.0f);

				ImGui::SetCursorScreenPos(ImVec2(np.x + 12.0f, np.y + 9.0f));
				QCCheckbox("##manualOverride", &CalCtx.useManualTimeOffset);
				dl->AddText(g_fontBody, g_fontBody->LegacySize,
					ImVec2(np.x + 48.0f, (np.y + nb.y) * 0.5f - g_fontBody->LegacySize * 0.5f),
					Pal::U32(Pal::Text), "Manual time offset override (debug)");

				ImVec2 msSize = ImGui::CalcTextSize("ms");
				float inputW = 110.0f;
				dl->AddText(g_fontBody, g_fontBody->LegacySize,
					ImVec2(nb.x - 14.0f - msSize.x, (np.y + nb.y) * 0.5f - g_fontBody->LegacySize * 0.5f),
					Pal::U32(Pal::Dim), "ms");
				ImGui::SetCursorScreenPos(ImVec2(nb.x - 14.0f - msSize.x - 10.0f - inputW, np.y + 5.0f));
				ImGui::PushItemWidth(inputW);
				float manualMs = (float)CalCtx.manualTimeOffsetMs;
				ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10, 6));
				if (ImGui::InputFloat("##manualTimeOffset", &manualMs, 0.0f, 0.0f, "%.1f"))
				{
					if (!std::isfinite(manualMs)) manualMs = 0.0f;
					if (manualMs > 50.0f) manualMs = 50.0f;
					if (manualMs < -50.0f) manualMs = -50.0f;
					CalCtx.manualTimeOffsetMs = manualMs;
				}
				ImGui::PopStyleVar();
				ImGui::PopItemWidth();
			}
		}

		// Continuous calibration: enable + nested tracker pick / safety options
		if (CalCtx.validProfile)
		{
			ContinuousStatus continuous = ContinuousStatusNow();
			const bool needsSetup = continuous == ContinuousStatus::NeedsMount ||
				continuous == ContinuousStatus::Frozen;
			const bool needsPick = continuous == ContinuousStatus::NoTracker;
			const float nestedH = CalCtx.continuousEnabled ? 188.0f : 0.0f;
			const float actionH = CalCtx.continuousEnabled && (needsSetup || needsPick) ? 46.0f : 0.0f;
			RowCard row(kRowHeight + kRowSubLineH + nestedH + actionH);
			const ImVec2 p = row.pos;
			ImDrawList *dl = ImGui::GetWindowDrawList();

			ImGui::SetCursorScreenPos(ImVec2(p.x + kRowInsetX, p.y + kRowControlY));
			bool continuousEnabled = CalCtx.continuousEnabled;
			if (QCCheckbox("##continuousEnabled", &continuousEnabled))
				SaveProfileFieldEdit(CalCtx,
					[&](questcal::ProfileRecord &candidate) {
						candidate.continuousEnabled = continuousEnabled;
					});
			RowIconLabel(p, IconCrosshair, "Continuous calibration");
			RowSubLine(p, "Uses a tracker attached to your headset to keep devices aligned while you play.");

			if (CalCtx.continuousEnabled)
			{
				// The toggle above changes what the loop is doing; the row height
				// for this frame is already committed, but the text is not.
				continuous = ContinuousStatusNow();
				const char *status = ContinuousStateWord(continuous);
				ImVec2 ts = ImGui::CalcTextSize(status);
				dl->AddText(g_fontBody, g_fontBody->LegacySize,
					ImVec2(p.x + cw - kRowInsetX - ts.x, p.y + 26.0f - g_fontBody->LegacySize * 0.5f),
					Pal::U32(ContinuousStatusColor(continuous)), status);

				// Nested inset: tracker pick, game visibility, opt-in latency.
				ImVec2 np = ImVec2(p.x + 12.0f, p.y + kRowHeight + kRowSubLineH);
				ImVec2 nb = ImVec2(p.x + cw - 12.0f, np.y + nestedH - 8.0f);
				dl->AddRectFilled(np, nb, Pal::U32(Pal::Inset), 9.0f);
				const float nestedW = nb.x - np.x - 24.0f;

				dl->AddText(g_fontBody, g_fontBody->LegacySize,
					ImVec2(np.x + 12.0f, np.y + 17.0f - g_fontBody->LegacySize * 0.5f),
					Pal::U32(Pal::Text), "Headset tracker");

				// Candidates: every target-system device except the HMD. A
				// disconnected one stays listed (it is still the pick) and
				// says so, instead of posing as a healthy choice.
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
					labels.push_back(DeviceDisplayName(*candidates[i]) + "  " + candidates[i]->serial +
						(candidates[i]->connected ? "" : "  Off"));
					if (candidates[i]->serial == CalCtx.continuousTrackerSerial)
						sel = (int)i;
				}
				if (sel < 0 && !CalCtx.continuousTrackerSerial.empty())
				{
					labels.push_back(CalCtx.continuousTrackerSerial + "  Off");
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
						// The count described the previous tracker's loop.
						CalCtx.autoCorrectionsApplied = 0;
					}
				}
				ImGui::PopStyleVar();
				ImGui::PopItemWidth();

				// Which loop runs: a two-way switch, since both are a method and
				// neither is "on". Legacy is the original SpaceCalibrator's,
				// for players whose setup worked with it.
				{
					const float segItemW = 150.0f, segH = 32.0f;
					const float rowY = np.y + 40.0f;
					dl->AddText(g_fontBody, g_fontBody->LegacySize,
						ImVec2(np.x + 12.0f, rowY + segH * 0.5f - g_fontBody->LegacySize * 0.5f),
						Pal::U32(Pal::Text), "Method");
					const ImVec2 segPos(nb.x - 14.0f - (segItemW * 2.0f + 8.0f), rowY);
					ImGui::SetCursorScreenPos(segPos);
					const char *methods[] = { "QuestCalibrator", "Legacy" };
					const int mode = CalCtx.continuousMode == ContinuousMode::Legacy ? 1 : 0;
					const int picked = Segmented("contMethod", mode, methods, 2, segItemW, segH);
					if (ImGui::IsMouseHoveringRect(segPos,
						ImVec2(segPos.x + segItemW * 2.0f + 8.0f, rowY + segH)))
						// Beside the cursor, not below it: below is the rest of the inset.
						ShowTip("QuestCalibrator measures where the tracker sits on the headset once,\n"
							"then corrects from every pose. Legacy is OpenVR-SpaceCalibrator's\n"
							"method: it re-solves from head movement. Try it if the other keeps pausing.", true);
					if (picked != mode)
						SaveProfileFieldEdit(CalCtx,
							[&](questcal::ProfileRecord &candidate) {
								candidate.continuousMode = picked;
							});
				}

				bool hideMountedTracker = CalCtx.hideMountedTracker;
				if (NestedToggle("##hideTracker", ImVec2(np.x + 12.0f, np.y + 80.0f), nestedW,
					"Hide the headset tracker from games", hideMountedTracker,
					"Moves it far out of reach in games so full-body setups never\n"
					"mistake it for a body tracker. Calibration still sees it."))
					SaveProfileFieldEdit(CalCtx,
						[&](questcal::ProfileRecord &candidate) {
							candidate.hideMountedTracker = hideMountedTracker;
						});

				bool latencyReestimation = CalCtx.continuousLatencyReestimation;
				if (NestedToggle("##contLatency", ImVec2(np.x + 12.0f, np.y + 114.0f), nestedW,
					"Re-estimate time offset continuously", latencyReestimation,
					"Also keeps the solved time offset up to date during play, measured\n"
					"from the headset tracker. Off keeps the value from calibration."))
					SaveProfileFieldEdit(CalCtx,
						[&](questcal::ProfileRecord &candidate) {
							candidate.continuousLatencyReestimation = latencyReestimation;
						});

				bool requireTrigger = CalCtx.continuousRequireTrigger;
				if (NestedToggle("##contTrigger", ImVec2(np.x + 12.0f, np.y + 146.0f), nestedW,
					"Ask before applying each correction", requireTrigger,
					"Each correction waits until you pull a controller trigger.\n"
					"The main screen shows when one is waiting."))
					SaveProfileFieldEdit(CalCtx,
						[&](questcal::ProfileRecord &candidate) {
							candidate.continuousRequireTrigger = requireTrigger;
						});

				// What the loop needs from the player, as a button whenever the
				// app can do it for them.
				if (needsPick || needsSetup)
				{
					ImVec2 ap = ImVec2(p.x + kRowInsetX, nb.y + 6.0f);
					if (needsPick)
					{
						dl->AddText(g_fontBody, g_fontBody->LegacySize,
							ImVec2(ap.x, ap.y + 8.0f), Pal::U32(Pal::Violet),
							"Strap a spare tracker to the headset and pick it above.");
					}
					else
					{
						ImGui::SetCursorScreenPos(ap);
						std::string label = FormatString(
							continuous == ContinuousStatus::Frozen
								? "Recalibrate with the headset tracker (%.0f s)"
								: "Measure the tracker position (%.0f s)",
							CalCtx.CollectionSeconds());
						if (IconButton("mountsetup", label.c_str(), IconPlay,
							ImVec2(cw - kRowInsetX * 2.0f, 38.0f), BtnKind::Primary))
							StartMountSetup(state);
					}
				}
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
					CalCtx.persistence.MarkSettings(CalCtx.timeLastTick);
				}
			}
			if (ImGui::IsItemHovered())
			{
				ShowTip(
					"Turn off to let the Quest's Guardian import win each session;\n"
					"the saved chaperone then only applies when you press Restore.");
			}
			RowIconLabel(p, IconCopy, "Automatically restore saved chaperone");

			std::string info;
			if (CalCtx.chaperone.geometry.empty())
			{
				info = "Snapshot: play area center only (no chaperone walls) -- nothing to restore automatically";
			}
			else
			{
				auto copied = FormatUnixAge(CalCtx.chaperone.copyUnixTime);
				size_t walls = CalCtx.chaperone.geometry.size();
				info = FormatString("%zu wall%s, %.1f x %.1f m, saved %s",
					walls, walls == 1 ? "" : "s",
					CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1],
					copied ? copied->c_str() : "age unknown");
			}
			ImGui::GetWindowDrawList()->AddText(g_fontSmall, g_fontSmall->LegacySize,
				ImVec2(p.x + 92.0f, p.y + 46.0f), Pal::U32(Pal::Dim), info.c_str());

			float btnW = 224.0f;
			ImGui::SetCursorScreenPos(ImVec2(p.x + cw - kRowInsetX - btnW, p.y + 9.0f));
			if (IconButton("pastechap", "Restore chaperone now", IconCopy, ImVec2(btnW, 34.0f), BtnKind::Ghost))
				ApplyChaperoneBounds();
		}

		// Updates never touch the network until this persisted opt-in is on.
		// Checking and downloading are automatic; applying the verified package
		// remains explicit because Steam must close and Windows must elevate the
		// existing installer.
		{
			questcal::update::Snapshot update = questcal::update::AppUpdater.GetSnapshot();
			const bool expanded = CalCtx.automaticUpdates;
			RowCard row(kRowHeight + kRowSubLineH + (expanded ? 46.0f : 0.0f));
			const ImVec2 p = row.pos;
			ImDrawList *dl = ImGui::GetWindowDrawList();

			ImGui::SetCursorScreenPos(ImVec2(p.x + kRowInsetX, p.y + kRowControlY));
			const bool previous = CalCtx.automaticUpdates;
			if (QCCheckbox("##automaticUpdates", &CalCtx.automaticUpdates))
			{
				SaveSettingOrRestore(CalCtx.automaticUpdates, previous);
				if (CalCtx.automaticUpdates != previous && !g_uiPreviewMode)
					questcal::update::AppUpdater.SetEnabled(CalCtx.automaticUpdates);
			}
			RowIconLabel(p, IconDownload, "Automatic updates");
			RowSubLine(p, "Downloads verified updates. Close Steam before installing them.");

			if (expanded)
			{
				const float actionY = p.y + kRowHeight + kRowSubLineH + 6.0f;
				std::string status;
				ImVec4 color = Pal::Dim;
				switch (update.state)
				{
				case questcal::update::State::Checking:
					status = "Checking GitHub";
					break;
				case questcal::update::State::Downloading:
				{
					const unsigned percent = update.totalBytes == 0 ? 0 :
						static_cast<unsigned>((update.downloadedBytes * 100) / update.totalBytes);
					status = FormatString("Downloading %s  %u%%", update.version.c_str(), percent);
					break;
				}
				case questcal::update::State::UpToDate:
					status = "Version " + update.version + " is current";
					color = Pal::Good;
					break;
				case questcal::update::State::Ready:
					status = "Version " + update.version + " is downloaded and verified";
					color = Pal::Good;
					break;
				case questcal::update::State::Failed:
					status = "Update check failed";
					color = Pal::Bad;
					break;
				default:
					status = "Ready to check";
					break;
				}
				dl->AddText(g_fontSmall, g_fontSmall->LegacySize,
					ImVec2(p.x + 92.0f, actionY + 10.0f), Pal::U32(color), status.c_str());

				const bool installReady = update.state == questcal::update::State::Ready;
				const bool canRetry = update.state == questcal::update::State::Failed ||
					update.state == questcal::update::State::UpToDate ||
					update.state == questcal::update::State::Idle;
				if (installReady || canRetry)
				{
					const float buttonW = 180.0f;
					ImGui::SetCursorScreenPos(ImVec2(p.x + cw - kRowInsetX - buttonW, actionY));
					const std::string label = installReady ?
						("Install " + update.version) : "Check again";
					if (IconButton("updateAction", label.c_str(),
						installReady ? IconDownload : nullptr,
						ImVec2(buttonW, 34.0f), installReady ? BtnKind::Primary : BtnKind::Ghost))
					{
						if (installReady)
						{
							std::string error;
							if (questcal::update::AppUpdater.LaunchInstaller(error))
								RequestApplicationExit();
							else
								CalCtx.ReportError(error + "\n");
						}
						else
						{
							questcal::update::AppUpdater.CheckNow();
						}
					}
				}
				if (update.state == questcal::update::State::Failed &&
					ImGui::IsMouseHoveringRect(ImVec2(p.x, actionY),
						ImVec2(p.x + cw, actionY + 40.0f)))
					ShowTip(update.message.c_str());
			}
		}

		// Bug reports: extra detail in the session log while on, and one file
		// to send, written on request with personal folders taken out.
		{
			RowCard row(kRowHeight + kRowSubLineH);
			const ImVec2 p = row.pos;
			ImGui::SetCursorScreenPos(ImVec2(p.x + kRowInsetX, p.y + kRowControlY));
			bool previous = CalCtx.detailedLogging;
			if (QCCheckbox("##detailedLogging", &CalCtx.detailedLogging))
				SaveSettingOrRestore(CalCtx.detailedLogging, previous);
			RowIconLabel(p, IconInfo, "Detailed calibration logging");
			RowSubLine(p, "Records extra tracking and calibration details. Saved diagnostics remove your name and folder paths.");

			const float btnW = 210.0f;
			ImGui::SetCursorScreenPos(ImVec2(p.x + cw - kRowInsetX - btnW, p.y + kRowHeight * 0.5f - 17.0f));
			if (IconButton("savediag", "Save diagnostics file", IconCopy, ImVec2(btnW, 34.0f), BtnKind::Ghost))
			{
				std::string path, error;
				if (WriteDiagnosticsFile(CalCtx, path, error))
				{
					CalCtx.Tell("Diagnostics saved to " + path, CalibrationContext::Tone::Good);
					if (!g_uiPreviewMode)
						RevealInExplorer(path);
				}
				else
					CalCtx.ReportError(error + "\n");
			}
		}

		ImGui::Spacing();
		if (ImGui::CollapsingHeader("Motion demo credits"))
			ImGui::TextWrapped("%s", GuideModelCredits().c_str());

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

// Pinned under the content child by BuildMainWindow.

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
void SeedTransformEditorDraft()
{
	g_transformDraft = TransformEditorDraft();
	g_transformDraft.rotationQ = CalCtx.transform.rotation;
	g_transformDraft.rotationEuler = CalCtx.transform.RotationEulerDegrees();
	g_transformDraft.translationCm = CalCtx.transform.translationMeters * 100.0;
	g_transformDraft.scale = CalCtx.transform.scale;
}

bool BuildProfileEditor()
{
	ImGuiStyle &style = ImGui::GetStyle();
	float cw = ImGui::GetContentRegionAvail().x;
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
			"These values can't be applied. Use plain numbers within range. The saved profile is unchanged.");
	}
	return g_transformDraft.valid;
}

bool SaveProfileEditorDraft()
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
