#include "stdafx.h"
#include "Calibration.h"
#include "Configuration.h"
#include "EmbeddedFiles.h"
#include "UserInterface.h"

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
// glfw3native.h #undefs APIENTRY expecting windows.h to redefine it, but
// windows.h is already include-guarded via stdafx.h; wWinMain needs it back.
#ifndef APIENTRY
#define APIENTRY __stdcall
#endif
#include <dwmapi.h>
#pragma comment(lib, "dwmapi.lib")
// WIN32_LEAN_AND_MEAN keeps shellapi.h out of windows.h; CommandLineToArgvW
// needs it (shell32.lib is already linked by the project).
#include <shellapi.h>
#include <openvr.h>
#include <ctime>
#include <string>
#include <vector>

#pragma comment(linker,"\"/manifestdependency:type='win32' \
name='Microsoft.Windows.Common-Controls' version='6.0.0.0' \
processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

#define OPENVR_APPLICATION_KEY "burrow.QuestCalibrator"

extern "C" __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
extern "C" __declspec(dllexport) DWORD AmdPowerXpressRequestHighPerformance = 0x00000001;

void GLFWErrorCallback(int error, const char* description)
{
	fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

static void HandleCommandLine(LPWSTR lpCmdLine, bool appDirResolved);
static void SetupPreviewState();

static GLFWwindow *glfwWindow = nullptr;
static vr::VROverlayHandle_t overlayMainHandle = 0, overlayThumbnailHandle = 0;
static bool imguiContextInitialized = false;
static bool imguiGlfwInitialized = false;
static bool imguiOpenGLInitialized = false;

// The shell's half of the calibration layer's toast policy. The handle is read
// at call time, not captured: TryCreateVROverlay runs before InitCalibrator on
// the normal path but the overlay can also fail to create, and either way a
// zero handle has always meant "log only". Installed by InitCalibrator's caller
// below, which is why the calibration layer no longer declares an accessor for
// a resource this file owns.
static void ShowVRToast(const char *message)
{
	if (overlayMainHandle && vr::VRNotifications())
	{
		vr::VRNotificationId notifId = 0;
		vr::VRNotifications()->CreateNotification(
			overlayMainHandle, 0, vr::EVRNotificationType_Transient,
			message, vr::EVRNotificationStyle_Application, nullptr, &notifId);
	}
}
static GLuint fboHandle = 0, fboTextureHandle = 0;
static int fboTextureWidth = 0, fboTextureHeight = 0;

// Directory containing QuestCalibrator.exe. Everything we load or register by
// path (manifest.vrmanifest, icon.png) sits next to the executable, so this must
// NOT come from the process working directory - installers, Start Menu shortcuts
// and SteamVR auto-launch all start us from somewhere else, and registering a
// manifest path relative to the wrong directory fails silently.
static std::string appDir;   // UTF-8

// False means the module path could not be resolved at all. There is no usable
// fallback: substituting the working directory would register a manifest path
// SteamVR cannot launch and load the overlay icon from the wrong place, both
// silently, so callers must treat this as fatal.
static bool ResolveAppDir()
{
	// The OpenVR APIs this feeds (manifest registration, SetOverlayFromFile)
	// take UTF-8; the ANSI variants would hand SteamVR mojibake for any
	// non-ASCII install path, silently breaking auto-launch and the icon.
	std::vector<wchar_t> wide(MAX_PATH);
	for (;;)
	{
		DWORD len = GetModuleFileNameW(nullptr, wide.data(), static_cast<DWORD>(wide.size()));
		if (len == 0)
			return false;
		if (len < wide.size())
			break;
		// Returning exactly the buffer size means truncation, not success: an
		// install path longer than MAX_PATH must not silently become a
		// different directory. Retry on the heap.
		if (wide.size() >= 32768)
			return false;   // longer than any addressable NT path
		wide.resize(wide.size() * 2);
	}

	wchar_t *lastSlash = wcsrchr(wide.data(), L'\\');
	if (!lastSlash)
		return false;
	*lastSlash = L'\0';

	int bytes = WideCharToMultiByte(CP_UTF8, 0, wide.data(), -1, nullptr, 0, nullptr, nullptr);
	if (bytes <= 1)
		return false;
	std::string utf8(static_cast<size_t>(bytes), '\0');
	if (WideCharToMultiByte(CP_UTF8, 0, wide.data(), -1, &utf8[0], bytes, nullptr, nullptr) == 0)
		return false;
	utf8.resize(static_cast<size_t>(bytes) - 1);   // drop the terminating NUL
	appDir = std::move(utf8);
	return true;
}

// Everything we load or register by path sits next to the executable; composing
// those joins in one place keeps a new one from picking a different directory.
static std::string AppFile(const char *name)
{
	return appDir + "\\" + name;
}

// Release builds are a GUI binary with no console, so printf/cerr from the
// -installmanifest style commands go nowhere. Report through a message box
// instead, unless the caller passed -noui (the installer does, so a scripted
// install never blocks on a modal window and reads the exit code instead).
static bool g_cliNoUi = false;

// -frames N: render exactly N frames and return. Paired with -uipreview (which
// builds a complete fake VR state and needs no SteamVR) this makes the UI layer
// runnable as a smoke test: a crash on any of those frames leaves wWinMain with
// a non-empty fatal message and a non-zero exit code.
static int g_frameLimit = 0;

static void CliReport(const char *message, bool isError)
{
	if (isError)
		fprintf(stderr, "%s\n", message);
	else
		printf("%s\n", message);

	if (g_cliNoUi)
		return;

	MessageBoxA(nullptr, message, "QuestCalibrator",
		MB_OK | (isError ? MB_ICONERROR : MB_ICONINFORMATION));
}

struct ManifestInstallResult
{
	bool success = false;
	bool changed = false;
	std::string message;
};

// One rollback-safe registration path for the installer and for startup
// self-repair. OpenVR rejects duplicate application keys, so replacement must
// temporarily remove the old manifest; every failure after that restores it.
//
// Auto-launch is the user's setting once the app is registered: only the
// installer (`forceAutoLaunch`) and a first-ever registration turn it on. A
// moved install carries the previous choice across its re-registration, and a
// registration that already points here is left exactly as SteamVR has it, so
// a user who switched auto-launch off does not get it switched back on by
// every launch.
static ManifestInstallResult EnsureManifestRegistration(bool forceAutoLaunch)
{
	ManifestInstallResult result;
	const std::string manifestPath = AppFile("manifest.vrmanifest");
	if (GetFileAttributesA(manifestPath.c_str()) == INVALID_FILE_ATTRIBUTES)
	{
		result.message = "QuestCalibrator's application manifest is missing. The existing SteamVR registration was left unchanged.\n\n" + manifestPath;
		return result;
	}

	std::string oldManifest;
	bool oldAutoLaunch = false;
	if (vr::VRApplications()->IsApplicationInstalled(OPENVR_APPLICATION_KEY))
	{
		oldAutoLaunch = vr::VRApplications()->GetApplicationAutoLaunch(
			OPENVR_APPLICATION_KEY);
		char oldDirectory[MAX_PATH] = {};
		auto error = vr::VRApplicationError_None;
		vr::VRApplications()->GetApplicationPropertyString(OPENVR_APPLICATION_KEY,
			vr::VRApplicationProperty_WorkingDirectory_String, oldDirectory,
			MAX_PATH, &error);
		if (error != vr::VRApplicationError_None)
		{
			result.message = "Failed to locate the previously registered QuestCalibrator manifest. The old registration was left unchanged.\n\n" +
				std::string(vr::VRApplications()->GetApplicationsErrorNameFromEnum(error));
			return result;
		}
		oldManifest = std::string(oldDirectory) + "\\manifest.vrmanifest";
	}

	const bool replacing = !oldManifest.empty() &&
		_stricmp(oldManifest.c_str(), manifestPath.c_str()) != 0;
	const bool adding = replacing || oldManifest.empty();
	auto restoreOld = [&]()
	{
		if (oldManifest.empty())
			return true;
		auto error = vr::VRApplications()->AddApplicationManifest(oldManifest.c_str());
		return error == vr::VRApplicationError_None &&
			vr::VRApplications()->SetApplicationAutoLaunch(
				OPENVR_APPLICATION_KEY, oldAutoLaunch) == vr::VRApplicationError_None;
	};
	auto failed = [&](const std::string &message, bool restore)
	{
		result.message = message;
		if (restore)
			result.message += restoreOld()
				? "\n\nThe previous registration was restored."
				: "\n\nSteamVR also refused to restore the previous registration.";
		return result;
	};

	if (replacing)
	{
		auto error = vr::VRApplications()->RemoveApplicationManifest(oldManifest.c_str());
		if (error != vr::VRApplicationError_None)
			return failed("Failed to remove the previously registered QuestCalibrator manifest. The old registration was left unchanged.\n\n" + oldManifest + "\n\n" +
				vr::VRApplications()->GetApplicationsErrorNameFromEnum(error), false);
	}

	if (adding)
	{
		auto error = vr::VRApplications()->AddApplicationManifest(manifestPath.c_str());
		if (error != vr::VRApplicationError_None)
			return failed("Failed to register the application manifest with SteamVR.\n\n" +
				manifestPath + "\n\n" +
				vr::VRApplications()->GetApplicationsErrorNameFromEnum(error), replacing);
	}

	// A freshly added manifest gets its auto-launch set explicitly: on for a
	// first registration, the carried-over choice for a moved install. An
	// existing registration is only touched when the installer asks.
	const bool wantAutoLaunch = forceAutoLaunch || oldManifest.empty() || oldAutoLaunch;
	const bool settingAutoLaunch = adding || (forceAutoLaunch && !oldAutoLaunch);
	if (settingAutoLaunch)
	{
		auto error = vr::VRApplications()->SetApplicationAutoLaunch(
			OPENVR_APPLICATION_KEY, wantAutoLaunch);
		if (error != vr::VRApplicationError_None)
		{
			if (adding)
				vr::VRApplications()->RemoveApplicationManifest(manifestPath.c_str());
			return failed("SteamVR could not set QuestCalibrator auto-launch.\n\n" +
				std::string(vr::VRApplications()->GetApplicationsErrorNameFromEnum(error)),
				replacing);
		}
	}

	result.success = true;
	result.changed = adding || (settingAutoLaunch && wantAutoLaunch != oldAutoLaunch);
	result.message = "QuestCalibrator registered with SteamVR.\n\n" + manifestPath;
	return result;
}

void CreateGLFWWindow()
{
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, false);

	fboTextureWidth = 1200;
	fboTextureHeight = 800;

	glfwWindow = glfwCreateWindow(fboTextureWidth, fboTextureHeight, "QuestCalibrator", NULL, NULL);
	if (!glfwWindow)
		throw std::runtime_error("Failed to create window");

	glfwMakeContextCurrent(glfwWindow);
	glfwSwapInterval(1);
	if (gl3wInit() != 0)
		throw std::runtime_error("Failed to initialize OpenGL functions");

	// Dark titlebar on Windows 10 20H1+ (attribute 20 = DWMWA_USE_IMMERSIVE_DARK_MODE).
	BOOL darkTitlebar = TRUE;
	DwmSetWindowAttribute(glfwGetWin32Window(glfwWindow), 20, &darkTitlebar, sizeof darkTitlebar);

	if (!g_uiPreviewMode)
		glfwIconifyWindow(glfwWindow);

	imguiContextInitialized = ImGui::CreateContext() != nullptr;
	if (!imguiContextInitialized)
		throw std::runtime_error("Failed to initialize ImGui");
	ImGuiIO &io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
	io.IniFilename = nullptr;
	g_fontBody = io.Fonts->AddFontFromMemoryCompressedTTF(DroidSans_compressed_data, DroidSans_compressed_size, 21.0f);
	g_fontSmall = io.Fonts->AddFontFromMemoryCompressedTTF(DroidSans_compressed_data, DroidSans_compressed_size, 14.0f);
	g_fontTitle = io.Fonts->AddFontFromMemoryCompressedTTF(DroidSans_compressed_data, DroidSans_compressed_size, 27.0f);
	io.FontDefault = g_fontBody;

	imguiGlfwInitialized = ImGui_ImplGlfw_InitForOpenGL(glfwWindow, true);
	if (!imguiGlfwInitialized)
		throw std::runtime_error("Failed to initialize the ImGui GLFW backend");
	imguiOpenGLInitialized = ImGui_ImplOpenGL3_Init("#version 330");
	if (!imguiOpenGLInitialized)
		throw std::runtime_error("Failed to initialize the ImGui OpenGL backend");

	ApplyTheme();

	glGenTextures(1, &fboTextureHandle);
	glBindTexture(GL_TEXTURE_2D, fboTextureHandle);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, fboTextureWidth, fboTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glGenFramebuffers(1, &fboHandle);
	glBindFramebuffer(GL_FRAMEBUFFER, fboHandle);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, fboTextureHandle, 0);

	GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
	glDrawBuffers(1, drawBuffers);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		throw std::runtime_error("OpenGL framebuffer incomplete");
	}
}

void TryCreateVROverlay()
{
	if (overlayMainHandle || !vr::VROverlay())
		return;

	vr::VROverlayError error = vr::VROverlay()->CreateDashboardOverlay(
		"burrow.QuestCalibrator", "Quest Cal",
		&overlayMainHandle, &overlayThumbnailHandle
	);

	if (error == vr::VROverlayError_KeyInUse)
	{
		throw std::runtime_error("Another instance of QuestCalibrator is already running");
	}
	else if (error != vr::VROverlayError_None)
	{
		throw std::runtime_error("Error creating VR overlay: " + std::string(vr::VROverlay()->GetOverlayErrorNameFromEnum(error)));
	}

	vr::VROverlay()->SetOverlayWidthInMeters(overlayMainHandle, 3.0f);
	vr::VROverlay()->SetOverlayInputMethod(overlayMainHandle, vr::VROverlayInputMethod_Mouse);
	vr::VROverlay()->SetOverlayFlag(overlayMainHandle, vr::VROverlayFlags_SendVRDiscreteScrollEvents, true);

	vr::VROverlay()->SetOverlayFromFile(overlayThumbnailHandle, AppFile("icon.png").c_str());
}

void ActivateMultipleDrivers()
{
	vr::EVRSettingsError vrSettingsError;
	bool enabled = vr::VRSettings()->GetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool, &vrSettingsError);

	if (vrSettingsError != vr::VRSettingsError_None)
	{
		std::string err = "Could not read \"" + std::string(vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool) + "\" setting: "
			+ vr::VRSettings()->GetSettingsErrorNameFromEnum(vrSettingsError);

		throw std::runtime_error(err);
	}

	if (!enabled)
	{
		vr::VRSettings()->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool, true, &vrSettingsError);
		if (vrSettingsError != vr::VRSettingsError_None)
		{
			std::string err = "Could not set \"" + std::string(vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool) + "\" setting: "
				+ vr::VRSettings()->GetSettingsErrorNameFromEnum(vrSettingsError);

			throw std::runtime_error(err);
		}

		std::cerr << "Enabled \"" << vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool << "\" setting" << std::endl;
	}
	else
	{
		std::cerr << "\"" << vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool << "\" setting previously enabled" << std::endl;
	}
}

void InitVR(bool &initialized)
{
	auto initError = vr::VRInitError_None;
	vr::VR_Init(&initError, vr::VRApplication_Overlay);
	if (initError != vr::VRInitError_None)
	{
		auto error = vr::VR_GetVRInitErrorAsEnglishDescription(initError);
		throw std::runtime_error("OpenVR error:" + std::string(error));
	}
	// Publish successful OpenVR ownership immediately. Interface validation and
	// settings setup below can still throw; the caller must then shut this
	// session down even though InitVR itself did not return normally.
	initialized = true;

	if (!vr::VR_IsInterfaceVersionValid(vr::IVRSystem_Version))
	{
		throw std::runtime_error("OpenVR error: Outdated IVRSystem_Version");
	}
	else if (!vr::VR_IsInterfaceVersionValid(vr::IVRSettings_Version))
	{
		throw std::runtime_error("OpenVR error: Outdated IVRSettings_Version");
	}
	else if (!vr::VR_IsInterfaceVersionValid(vr::IVROverlay_Version))
	{
		throw std::runtime_error("OpenVR error: Outdated IVROverlay_Version");
	}

	ActivateMultipleDrivers();
}

void RunLoop()
{
	int framesRendered = 0;
	while (!glfwWindowShouldClose(glfwWindow))
	{
		TryCreateVROverlay();

		double time = glfwGetTime();
		CalibrationTick(time);

		bool dashboardVisible = false;
		int width, height;
		glfwGetFramebufferSize(glfwWindow, &width, &height);

		if (overlayMainHandle && vr::VROverlay())
		{
			auto &io = ImGui::GetIO();
			dashboardVisible = vr::VROverlay()->IsActiveDashboardOverlay(overlayMainHandle);

			// Closing the VR keyboard takes two frames to settle, so the phase is
			// named rather than encoded in flags whose combinations only a
			// comment explained. Both waits are deliberate, and io.WantTextInput
			// is read here BEFORE ImGui::NewFrame, so it always lags one frame
			// behind the widget state - which is why clearing the active widget
			// is not enough to stop an immediate reopen.
			enum class KeyboardPhase
			{
				Closed,
				Open,
				CommitText,         // Done pressed; give ImGui a frame to take SetActiveText
				AwaitInputRelease,  // widget cleared; wait for io.WantTextInput to catch up
			};
			static KeyboardPhase keyboardPhase = KeyboardPhase::Closed;

			switch (keyboardPhase)
			{
			case KeyboardPhase::CommitText:
				ImGui::ClearActiveID();
				keyboardPhase = KeyboardPhase::AwaitInputRelease;
				break;
			case KeyboardPhase::AwaitInputRelease:
				keyboardPhase = KeyboardPhase::Closed;
				break;
			case KeyboardPhase::Open:
				// The user might dismiss the keyboard without hitting Done; let
				// it open again in that case.
				if (!io.WantTextInput)
					keyboardPhase = KeyboardPhase::Closed;
				break;
			case KeyboardPhase::Closed:
				if (io.WantTextInput)
				{
					char buf[0x400];
					ImGui::GetActiveText(buf, sizeof buf);
					buf[0x3ff] = 0;
					uint32_t unFlags = 0; // EKeyboardFlags

					vr::EVROverlayError error = vr::VROverlay()->ShowKeyboardForOverlay(
						overlayMainHandle, vr::k_EGamepadTextInputModeNormal, vr::k_EGamepadTextInputLineModeSingleLine,
						unFlags, "QuestCalibrator Overlay", sizeof buf, buf, 0
					);
					if (error == vr::VROverlayError_None)
						keyboardPhase = KeyboardPhase::Open;
				}
				break;
			}

			vr::VREvent_t vrEvent;
			while (vr::VROverlay()->PollNextOverlayEvent(overlayMainHandle, &vrEvent, sizeof(vrEvent)))
			{
				switch (vrEvent.eventType) {
				case vr::VREvent_MouseMove:
					io.MousePos.x = vrEvent.data.mouse.x;
					io.MousePos.y = vrEvent.data.mouse.y;
					break;
				case vr::VREvent_MouseButtonDown:
					io.MouseDown[vrEvent.data.mouse.button == vr::VRMouseButton_Left ? 0 : 1] = true;
					break;
				case vr::VREvent_MouseButtonUp:
					io.MouseDown[vrEvent.data.mouse.button == vr::VRMouseButton_Left ? 0 : 1] = false;
					break;
				case vr::VREvent_ScrollDiscrete:
					io.MouseWheelH += vrEvent.data.scroll.xdelta * 360.0f * 8.0f;
					io.MouseWheel += vrEvent.data.scroll.ydelta * 360.0f * 8.0f;
					break;
				case vr::VREvent_KeyboardDone: {
					char buf[0x400] = {};
					uint32_t bytes = vr::VROverlay()->GetKeyboardText(buf, sizeof buf);
					if (bytes > 0)
					{
						buf[sizeof buf - 1] = 0;
						// buf_size is the capacity: the decoder writes at most
						// buf_size - 1 characters, so passing the string length
						// drops the last one typed.
						ImGui::SetActiveText(buf, static_cast<int>(sizeof buf));
					}
					// A Done for a keyboard we no longer consider open has no
					// widget to clear; only settle.
					keyboardPhase = keyboardPhase == KeyboardPhase::Open
						? KeyboardPhase::CommitText
						: KeyboardPhase::AwaitInputRelease;
					break;
				}
				case vr::VREvent_Quit:
					return;
				}
			}
		}

		ImGui::GetIO().DisplaySize = ImVec2((float) fboTextureWidth, (float) fboTextureHeight);

		ImGui_ImplGlfw_SetReadMouseFromGlfw(!dashboardVisible);
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		BuildMainWindow(dashboardVisible);

		ImGui::Render();

		glBindFramebuffer(GL_FRAMEBUFFER, fboHandle);
		glViewport(0, 0, fboTextureWidth, fboTextureHeight);
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT);

		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		if (width && height)
		{
			glBindFramebuffer(GL_READ_FRAMEBUFFER, fboHandle);
			glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
			glfwSwapBuffers(glfwWindow);
		}

		if (dashboardVisible)
		{
			vr::Texture_t vrTex;
			vrTex.eType = vr::TextureType_OpenGL;
			vrTex.eColorSpace = vr::ColorSpace_Auto;

			vrTex.handle = (void *)(uintptr_t) fboTextureHandle;

			vr::HmdVector2_t mouseScale = { (float) fboTextureWidth, (float) fboTextureHeight };

			vr::VROverlay()->SetOverlayTexture(overlayMainHandle, &vrTex);
			vr::VROverlay()->SetOverlayMouseScale(overlayMainHandle, &mouseScale);
		}

		const double dashboardInterval = 1.0 / 90.0; // fps
		double waitEventsTimeout = CalCtx.wantedUpdateInterval;

		if (dashboardVisible && waitEventsTimeout > dashboardInterval)
			waitEventsTimeout = dashboardInterval;

		// A zero interval (calibration collection) must not busy-spin: sample
		// ingestion happens on the PoseStreamHub thread and CalibrationTick
		// self-gates to 50 Hz, so nothing needs more than a short wait — and
		// the iconified window skips the vsync throttle that would otherwise
		// pace this loop.
		if (waitEventsTimeout < 0.005)
			waitEventsTimeout = 0.005;

		if (g_frameLimit > 0 && ++framesRendered >= g_frameLimit)
			return;

		glfwWaitEventsTimeout(waitEventsTimeout);
	}
}

// Two instances would each open a pose-ring reader and split the driver's pose
// stream disjointly between them - each seeing roughly half the samples, with
// no loss marker to say so, and both feeding the runtime monitors. The OpenVR
// dashboard key catches a duplicate too, but only once SteamVR is up and the
// overlay interface exists; this answers "am I already running" on its own and
// before anything is opened or rotated. Local\ is the session scope, which is
// the scope that shares the ring. The handle is deliberately held for the
// process lifetime and released by exit.
static HANDLE g_singleInstanceMutex = nullptr;

static bool ClaimSingleInstance()
{
	g_singleInstanceMutex = CreateMutexW(nullptr, FALSE,
		L"Local\\QuestCalibrator.SingleInstance");
	if (!g_singleInstanceMutex)
		return true;   // cannot prove a duplicate; never lock the user out on it
	if (GetLastError() == ERROR_ALREADY_EXISTS)
	{
		CloseHandle(g_singleInstanceMutex);
		g_singleInstanceMutex = nullptr;
		return false;
	}
	return true;
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
	// Resolve first: the CLI commands below register and load files by path.
	bool appDirResolved = ResolveAppDir();
	HandleCommandLine(lpCmdLine, appDirResolved);

	// After the CLI commands, so registering a manifest still works while the
	// overlay runs, and before the session log, so a duplicate launch cannot
	// rotate a live session's log away.
	if (!g_uiPreviewMode && !ClaimSingleInstance())
	{
		MessageBox(nullptr, L"QuestCalibrator is already running.",
			L"QuestCalibrator", MB_ICONINFORMATION | MB_OK);
		return -1;
	}

	// Before the try block so even InitVR/window-creation failures land in the
	// log; skipped for UI preview so a dev preview never rotates a real
	// session's log away.
	if (!g_uiPreviewMode)
		InitSessionLog();

	if (!glfwInit())
	{
		MessageBox(nullptr, L"Failed to initialize GLFW", L"", 0);
		return -1;
	}

	glfwSetErrorCallback(GLFWErrorCallback);
	bool vrInitialized = false;
	bool calibratorInitialized = false;
	auto shutdownRuntime = [&](bool cleanExit)
	{
		// Clear ownership before calling out so a cleanup exception cannot make
		// the catch path invoke the same shutdown operation twice.
		if (calibratorInitialized)
		{
			calibratorInitialized = false;
			ShutdownCalibrator(cleanExit);
		}
		if (vrInitialized)
		{
			vrInitialized = false;
			vr::VR_Shutdown();
		}
	};
	auto shutdownGraphics = [&]()
	{
		if (glfwWindow)
			glfwMakeContextCurrent(glfwWindow);
		if (fboHandle)
		{
			glDeleteFramebuffers(1, &fboHandle);
			fboHandle = 0;
		}
		if (fboTextureHandle)
		{
			glDeleteTextures(1, &fboTextureHandle);
			fboTextureHandle = 0;
		}
		if (imguiOpenGLInitialized)
		{
			imguiOpenGLInitialized = false;
			ImGui_ImplOpenGL3_Shutdown();
		}
		if (imguiGlfwInitialized)
		{
			imguiGlfwInitialized = false;
			ImGui_ImplGlfw_Shutdown();
		}
		if (imguiContextInitialized)
		{
			imguiContextInitialized = false;
			ImGui::DestroyContext();
		}
	};

	// Record what went wrong rather than acting on it inside the catch, so the
	// shutdown pair below exists exactly once for every path.
	std::string fatal;
	try {
		if (!g_uiPreviewMode)
		{
			InitVR(vrInitialized);
			ManifestInstallResult registration = EnsureManifestRegistration(false);
			if (!registration.success)
				AppendSessionLog("SteamVR manifest self-repair failed: " + registration.message);
			else if (registration.changed)
				AppendSessionLog("SteamVR application manifest/auto-launch registration repaired");
			// Take the single-instance guard (the dashboard overlay key) BEFORE
			// anything opens the driver's pose ring. Two overlay readers on one
			// ring split the sample stream between them with no loss marker, so
			// a second instance that reached InitCalibrator first would silently
			// decimate the running instance's stream - possibly mid-calibration
			// - for as long as it took to reach its own first RunLoop iteration.
			TryCreateVROverlay();
		}
		CreateGLFWWindow();
		if (g_uiPreviewMode)
		{
			SetupPreviewState();
		}
		else
		{
			SetToastSink(ShowVRToast);
			InitCalibrator();
			calibratorInitialized = true;
			LoadProfile(CalCtx);
		}
		RunLoop();
	}
	catch (const std::exception &e)
	{
		fatal = std::string("Runtime error: ") + e.what();
	}
	catch (...)
	{
		fatal = "QuestCalibrator stopped because of an unknown fatal error.";
	}

	// One shutdown pair for every path, and before the modal dialog below can
	// block this process indefinitely: ShutdownCalibrator flushes debounced
	// profile/settings updates before stopping the pose hub. The ownership
	// flags inside the lambdas are what make a throw out of one of them
	// recoverable - the completed half is already a no-op on the retry.
	try
	{
		shutdownRuntime(fatal.empty());
		shutdownGraphics();
	}
	catch (...)
	{
		// Not dead code: the ownership flags cleared whatever already completed,
		// so this retry finishes the rest rather than repeating it.
		shutdownRuntime(false);
		shutdownGraphics();
		if (fatal.empty())
			fatal = "QuestCalibrator hit a fatal error while shutting down.";
	}

	if (!fatal.empty())
	{
		std::cerr << fatal << std::endl;
		AppendSessionLog(fatal);
		wchar_t message[1024];
		swprintf(message, 1024, L"%hs", fatal.c_str());
		MessageBox(nullptr, message, L"Runtime Error", MB_OK | MB_ICONERROR);
	}

	if (glfwWindow)
		glfwDestroyWindow(glfwWindow);

	glfwTerminate();
	return fatal.empty() ? 0 : -1;
}

// UI preview (-uipreview): plausible fake state so every part of the interface
// renders without SteamVR. Fake devices come from LoadVRState; profile saves
// are disabled while previewing.
static void SetupPreviewState()
{
	CalCtx.validProfile = true;
	CalCtx.enabled = true;
	CalCtx.referenceTrackingSystem = "oculus";
	CalCtx.targetTrackingSystem = "lighthouse";

	CalCtx.lastResult.valid = true;
	CalCtx.lastResult.rotationRmsDeg = 2.53;
	CalCtx.lastResult.translationRmsMeters = 0.010;
	CalCtx.lastResult.timeOffset = 0.0038;
	CalCtx.lastResult.scale = 1.002;
	CalCtx.transform.scale = 1.002;

	CalCtx.calibrationUnixTime = static_cast<double>(std::time(nullptr)) - 180.0;
	CalCtx.alignment = CalibrationContext::AlignmentHealth::Stale;
	CalCtx.driftScore = 0.70;
	CalCtx.driftSlideEvents = 21;
	CalCtx.driftMaxSlideM = 0.08;
	CalCtx.discontinuousLossEvents = 0;

	CalCtx.appliedTimeOffset = -0.0038;
	CalCtx.applyTimeOffset = true;
	CalCtx.fieldEnabled = true;
	CalCtx.chaperone.valid = true;
	CalCtx.chaperone.geometry.resize(26);
	CalCtx.chaperone.playSpaceSize.v[0] = 2.1f;
	CalCtx.chaperone.playSpaceSize.v[1] = 2.4f;
	CalCtx.chaperone.copyUnixTime = static_cast<double>(std::time(nullptr)) - 840.0;

	CalibrationContext::FieldAnchor anchor;
	anchor.position = Eigen::Vector3d(1.2, 1.1, -0.8);
	anchor.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.004, Eigen::Vector3d::UnitY()));
	anchor.translationMeters = Eigen::Vector3d(0.02, 0.0, -0.01);
	CalCtx.fieldAnchors.push_back(anchor);

	// Continuous calibration in its healthy maintaining state (the tracker
	// serial matches the first fake VIVE tracker in -uipreview-many).
	CalCtx.continuousEnabled = true;
	CalCtx.continuousTrackerSerial = "LHR-77E5A211";
	CalCtx.hideMountedTracker = true;
	CalCtx.mountExtrinsic.valid = true;
	CalCtx.mountExtrinsic.rot = Eigen::Quaterniond(
		Eigen::AngleAxisd(2.7, Eigen::Vector3d(0.2, 0.7, -0.3).normalized()));
	CalCtx.mountExtrinsic.pos = Eigen::Vector3d(0.05, -0.08, 0.03);
	CalCtx.mountExtrinsic.rotRmsDeg = 0.21;
	CalCtx.mountExtrinsic.posRmsM = 0.004;
	CalCtx.continuousState = questcal::ContinuousAlignment::State::Tracking;
	CalCtx.continuousDeviation.valid = true;
	CalCtx.continuousDeviation.yawDeg = 0.08;
	CalCtx.continuousDeviation.tiltDeg = 0.11;
	CalCtx.continuousDeviation.posM = 0.004;
	CalCtx.continuousScatterRotDeg = 0.19;
	CalCtx.continuousScatterPosM = 0.006;
	CalCtx.autoCorrectionsApplied = 14;
	CalCtx.lastAutoCorrectionUnixTime = static_cast<double>(std::time(nullptr)) - 42.0;
}

// Shared exit path for the CLI commands: report, shut OpenVR down, and exit with
// a code the installer can act on.
static void CliExit(const std::string &message, bool isError)
{
	CliReport(message.c_str(), isError);
	vr::VR_Shutdown();
	exit(isError ? -2 : 0);
}

static std::string InitErrorMessage(vr::EVRInitError vrErr)
{
	return std::string("Failed to initialize OpenVR: ")
		+ vr::VR_GetVRInitErrorAsEnglishDescription(vrErr)
		+ "\n\nSteamVR must be installed. If it has never been run on this PC,"
		" start SteamVR once and try again.";
}

// Every command below reaches straight for the applications or settings
// interface, which is only valid inside an initialised session. Sharing the
// prologue means a new command cannot forget it. The preview flags must NOT
// call this: their whole point is running without SteamVR.
static void InitVRUtilityOrExit()
{
	auto vrErr = vr::VRInitError_None;
	vr::VR_Init(&vrErr, vr::VRApplication_Utility);
	if (vrErr != vr::VRInitError_None)
		CliExit(InitErrorMessage(vrErr), true);
}

static std::string Narrow(const std::wstring &wide)
{
	if (wide.empty())
		return std::string();
	int bytes = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(),
		static_cast<int>(wide.size()), nullptr, 0, nullptr, nullptr);
	if (bytes <= 0)
		return std::string();
	std::string out(static_cast<size_t>(bytes), '\0');
	WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), static_cast<int>(wide.size()),
		&out[0], bytes, nullptr, nullptr);
	return out;
}

static void HandleCommandLine(LPWSTR lpCmdLine, bool appDirResolved)
{
	// Tokenise instead of substring-matching: -noui must not be stripped out of
	// the middle of a longer token, and an argument we do not recognise has to
	// be reported rather than falling through to a full GUI launch with exit
	// code 0 - which a scripted install cannot tell from success.
	// CommandLineToArgvW applies program-name rules to the first token, so
	// prepend a placeholder for it.
	std::wstring full = L"QuestCalibrator.exe ";
	full += lpCmdLine ? lpCmdLine : L"";
	std::vector<std::wstring> args;
	int argc = 0;
	if (LPWSTR *argv = CommandLineToArgvW(full.c_str(), &argc))
	{
		for (int i = 1; i < argc; ++i)
			args.push_back(argv[i]);
		LocalFree(argv);
	}

	// -noui may accompany any command below. The installer passes it so a
	// scripted install never blocks on a modal dialog and reads the exit code
	// instead; without it (manual install) results are shown in a message box.
	std::wstring cmd, unrecognised;
	for (size_t i = 0; i < args.size(); ++i)
	{
		const std::wstring &arg = args[i];
		if (arg == L"-noui")
			g_cliNoUi = true;
		else if (arg == L"-frames" && i + 1 < args.size() &&
			_wtoi(args[i + 1].c_str()) > 0)
			g_frameLimit = _wtoi(args[++i].c_str());
		else if (cmd.empty())
			cmd = arg;
		else if (unrecognised.empty())
			unrecognised = arg;
	}

	// Everything registered or loaded by path hangs off the install directory,
	// so failing to resolve it must fail loudly here rather than register a
	// manifest SteamVR will later auto-launch from the wrong place.
	if (!appDirResolved)
		CliExit("QuestCalibrator could not determine its own install directory.", true);

	// An extra argument alongside a valid command is as much a mistake as a
	// mistyped command, and used to be ignored entirely.
	if (!unrecognised.empty())
		CliExit("Unrecognised command-line argument: " + Narrow(unrecognised), true);

	if (cmd == L"-uipreview")
	{
		g_uiPreviewMode = true;
	}
	else if (cmd == L"-uipreview-many")
	{
		g_uiPreviewMode = true;
		g_uiPreviewMany = true;
	}
	else if (cmd == L"-openvrpath")
	{
		InitVRUtilityOrExit();

		char stackRuntimePath[MAX_PATH] = { 0 };
		uint32_t requiredBytes = 0;
		bool pathRead = vr::VR_GetRuntimePath(stackRuntimePath,
			static_cast<uint32_t>(sizeof stackRuntimePath), &requiredBytes);
		char *runtimePath = stackRuntimePath;
		size_t runtimePathCapacity = sizeof stackRuntimePath;
		std::vector<char> extendedRuntimePath;
		constexpr uint32_t MaxRuntimePathBytes = 1024 * 1024;
		if (!pathRead && requiredBytes > sizeof stackRuntimePath &&
			requiredBytes <= MaxRuntimePathBytes)
		{
			extendedRuntimePath.resize(requiredBytes, '\0');
			uint32_t retryRequiredBytes = 0;
			pathRead = vr::VR_GetRuntimePath(extendedRuntimePath.data(),
				requiredBytes, &retryRequiredBytes);
			requiredBytes = retryRequiredBytes;
			runtimePath = extendedRuntimePath.data();
			runtimePathCapacity = extendedRuntimePath.size();
		}

		if (!pathRead || requiredBytes == 0 ||
			requiredBytes > runtimePathCapacity ||
			!std::memchr(runtimePath, '\0', runtimePathCapacity) ||
			runtimePath[0] == '\0')
		{
			CliExit("Failed to read the OpenVR runtime path.", true);
		}

		// Machine-readable, so no trailing newline: callers capture this on
		// stdout and use it directly as a path.
		printf("%s", runtimePath);
		if (!g_cliNoUi)
			MessageBoxA(nullptr, runtimePath, "QuestCalibrator", MB_OK | MB_ICONINFORMATION);
		vr::VR_Shutdown();
		exit(0);
	}
	else if (cmd == L"-installmanifest")
	{
		InitVRUtilityOrExit();
		ManifestInstallResult install = EnsureManifestRegistration(true);
		CliExit(install.message, !install.success);
	}
	else if (cmd == L"-removemanifest")
	{
		InitVRUtilityOrExit();

		std::string manifestPath = AppFile("manifest.vrmanifest");
		if (vr::VRApplications()->IsApplicationInstalled(OPENVR_APPLICATION_KEY))
		{
			auto vrAppErr = vr::VRApplications()->RemoveApplicationManifest(
				manifestPath.c_str());
			if (vrAppErr != vr::VRApplicationError_None)
			{
				CliExit("Failed to deregister QuestCalibrator from SteamVR.\n\n" +
					manifestPath + "\n\n" +
					vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr), true);
			}
		}

		CliExit("QuestCalibrator deregistered from SteamVR.", false);
	}
	else if (cmd == L"-activatemultipledrivers")
	{
		InitVRUtilityOrExit();

		try
		{
			ActivateMultipleDrivers();
		}
		catch (std::runtime_error &e)
		{
			CliExit(std::string("Failed to enable SteamVR's multiple-drivers setting.\n\n") + e.what(), true);
		}
		CliExit("SteamVR multiple-driver support enabled.", false);
	}
	else if (!cmd.empty())
	{
		// A mistyped command used to launch the full GUI and exit 0, which a
		// scripted install (Start-Process -Wait) cannot tell from success: it
		// blocks until a human closes a window that is iconified at creation.
		CliExit("Unrecognised command-line argument: " + Narrow(cmd), true);
	}
}
