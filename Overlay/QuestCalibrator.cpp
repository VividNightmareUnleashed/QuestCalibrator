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
#include <openvr.h>
#include <direct.h>
#include <ctime>
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

static void HandleCommandLine(LPWSTR lpCmdLine);
static void SetupPreviewState();

static GLFWwindow *glfwWindow = nullptr;
static vr::VROverlayHandle_t overlayMainHandle = 0, overlayThumbnailHandle = 0;
static bool imguiContextInitialized = false;
static bool imguiGlfwInitialized = false;
static bool imguiOpenGLInitialized = false;

vr::VROverlayHandle_t GetMainOverlayHandle()
{
	return overlayMainHandle;
}
static GLuint fboHandle = 0, fboTextureHandle = 0;
static int fboTextureWidth = 0, fboTextureHeight = 0;

// Directory containing QuestCalibrator.exe. Everything we load or register by
// path (manifest.vrmanifest, icon.png) sits next to the executable, so this must
// NOT come from the process working directory - installers, Start Menu shortcuts
// and SteamVR auto-launch all start us from somewhere else, and registering a
// manifest path relative to the wrong directory fails silently.
static char appDir[MAX_PATH * 3];   // UTF-8; up to 3 bytes per UTF-16 unit

static void ResolveAppDir()
{
	// The OpenVR APIs this feeds (manifest registration, SetOverlayFromFile)
	// take UTF-8; the ANSI variants would hand SteamVR mojibake for any
	// non-ASCII install path, silently breaking auto-launch and the icon.
	wchar_t wide[MAX_PATH];
	DWORD len = GetModuleFileNameW(nullptr, wide, MAX_PATH);
	if (len == 0 || len >= MAX_PATH)
	{
		if (!_wgetcwd(wide, MAX_PATH))
		{
			appDir[0] = '\0';
			return;
		}
	}
	else
	{
		wchar_t *lastSlash = wcsrchr(wide, L'\\');
		if (lastSlash)
			*lastSlash = L'\0';
	}
	if (WideCharToMultiByte(CP_UTF8, 0, wide, -1, appDir, sizeof appDir, nullptr, nullptr) == 0)
		appDir[0] = '\0';
}

// Release builds are a GUI binary with no console, so printf/cerr from the
// -installmanifest style commands go nowhere. Report through a message box
// instead, unless the caller passed -noui (the installer does, so a scripted
// install never blocks on a modal window and reads the exit code instead).
static bool g_cliNoUi = false;

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

	std::string iconPath = appDir;
	iconPath += "\\icon.png";
	vr::VROverlay()->SetOverlayFromFile(overlayThumbnailHandle, iconPath.c_str());
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
	vr::VR_Init(&initError, vr::VRApplication_Other);
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

			static bool keyboardOpen = false, keyboardJustClosed = false;

			// After closing the keyboard, this code waits one frame for ImGui to pick up the new text from SetActiveText
			// before clearing the active widget. Then it waits another frame before allowing the keyboard to open again,
			// otherwise it will do so instantly since WantTextInput is still true on the second frame.
			if (keyboardJustClosed && keyboardOpen)
			{
				ImGui::ClearActiveID();
				keyboardOpen = false;
			}
			else if (keyboardJustClosed)
			{
				keyboardJustClosed = false;
			}
			else if (!io.WantTextInput)
			{
				// User might close the keyboard without hitting Done, so we unset the flag to allow it to open again.
				keyboardOpen = false;
			}
			else if (io.WantTextInput && !keyboardOpen && !keyboardJustClosed)
			{
				char buf[0x400];
				ImGui::GetActiveText(buf, sizeof buf);
				buf[0x3ff] = 0;
				uint32_t unFlags = 0; // EKeyboardFlags 

				vr::VROverlay()->ShowKeyboardForOverlay(
					overlayMainHandle, vr::k_EGamepadTextInputModeNormal, vr::k_EGamepadTextInputLineModeSingleLine,
					unFlags, "QuestCalibrator Overlay", sizeof buf, buf, 0
				);
				keyboardOpen = true;
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
					char buf[0x400];
					vr::VROverlay()->GetKeyboardText(buf, sizeof buf);
					ImGui::SetActiveText(buf, sizeof buf);
					keyboardJustClosed = true;
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

			vrTex.handle = (void *)
#if defined _WIN64 || defined _LP64
			(uint64_t)
#endif
				fboTextureHandle;

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

		glfwWaitEventsTimeout(waitEventsTimeout);
	}
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
	ResolveAppDir();
	HandleCommandLine(lpCmdLine);

	// Before the try block so even InitVR/window-creation failures land in the
	// log; skipped for UI preview so a dev preview never rotates a real
	// session's log away.
	if (!g_uiPreviewMode)
		InitSessionLog();

	if (!glfwInit())
	{
		MessageBox(nullptr, L"Failed to initialize GLFW", L"", 0);
		return 0;
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

	try {
		if (!g_uiPreviewMode)
			InitVR(vrInitialized);
		CreateGLFWWindow();
		if (g_uiPreviewMode)
		{
			SetupPreviewState();
		}
		else
		{
			InitCalibrator();
			calibratorInitialized = true;
			LoadProfile(CalCtx);
		}
		RunLoop();
		shutdownRuntime(true);
		shutdownGraphics();
	}
	catch (const std::exception &e)
	{
		// Flush persistent state and release live runtime/graphics resources
		// before the modal error dialog can block this process indefinitely.
		shutdownRuntime(false);
		shutdownGraphics();
		std::cerr << "Runtime error: " << e.what() << std::endl;
		AppendSessionLog(std::string("Runtime error: ") + e.what());
		wchar_t message[1024];
		swprintf(message, 1024, L"%hs", e.what());
		MessageBox(nullptr, message, L"Runtime Error", 0);
	}
	catch (...)
	{
		shutdownRuntime(false);
		shutdownGraphics();
		const char *message = "QuestCalibrator stopped because of an unknown fatal error.";
		std::cerr << message << std::endl;
		AppendSessionLog(message);
		MessageBoxA(nullptr, message, "Runtime Error", MB_OK | MB_ICONERROR);
	}

	// Also covers every exception after initialization. In particular,
	// ShutdownCalibrator flushes debounced profile/settings updates before
	// stopping the pose hub. The ownership flags make this a no-op after the
	// normal shutdown above.
	shutdownRuntime(false);
	shutdownGraphics();

	if (glfwWindow)
		glfwDestroyWindow(glfwWindow);

	glfwTerminate();
	return 0;
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
	CalCtx.calibratedScale = 1.002;

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
	CalCtx.continuousState = static_cast<int>(questcal::ContinuousAlignment::State::Tracking);
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

static void HandleCommandLine(LPWSTR lpCmdLine)
{
	std::wstring cmd = lpCmdLine ? lpCmdLine : L"";

	// -noui may accompany any command below. The installer passes it so a
	// scripted install never blocks on a modal dialog and reads the exit code
	// instead; without it (manual install) results are shown in a message box.
	const std::wstring nouiFlag = L"-noui";
	auto nouiAt = cmd.find(nouiFlag);
	if (nouiAt != std::wstring::npos)
	{
		g_cliNoUi = true;
		cmd.erase(nouiAt, nouiFlag.size());
	}
	auto firstCh = cmd.find_first_not_of(L" \t");
	cmd = (firstCh == std::wstring::npos)
		? std::wstring()
		: cmd.substr(firstCh, cmd.find_last_not_of(L" \t") - firstCh + 1);

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
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr != vr::VRInitError_None)
			CliExit(InitErrorMessage(vrErr), true);

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
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr != vr::VRInitError_None)
			CliExit(InitErrorMessage(vrErr), true);

		if (vr::VRApplications()->IsApplicationInstalled(OPENVR_APPLICATION_KEY))
		{
			char oldWd[MAX_PATH] = { 0 };
			auto vrAppErr = vr::VRApplicationError_None;
			vr::VRApplications()->GetApplicationPropertyString(OPENVR_APPLICATION_KEY, vr::VRApplicationProperty_WorkingDirectory_String, oldWd, MAX_PATH, &vrAppErr);
			if (vrAppErr != vr::VRApplicationError_None)
			{
				CliExit("Failed to locate the previously registered QuestCalibrator manifest. "
					"The old registration was left unchanged.\n\n" +
					std::string(vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr)), true);
			}
			else
			{
				std::string oldManifest = oldWd;
				oldManifest += "\\manifest.vrmanifest";
				std::cout << "Removing old manifest path: " << oldManifest << std::endl;
				vrAppErr = vr::VRApplications()->RemoveApplicationManifest(
					oldManifest.c_str());
				if (vrAppErr != vr::VRApplicationError_None)
				{
					CliExit("Failed to remove the previously registered QuestCalibrator manifest. "
						"The old registration was left unchanged.\n\n" + oldManifest + "\n\n" +
						vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr), true);
				}
			}
		}

		std::string manifestPath = appDir;
		manifestPath += "\\manifest.vrmanifest";

		auto vrAppErr = vr::VRApplications()->AddApplicationManifest(manifestPath.c_str());
		if (vrAppErr != vr::VRApplicationError_None)
		{
			CliExit("Failed to register the application manifest with SteamVR.\n\n"
				+ manifestPath + "\n\n"
				+ vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr), true);
		}
		vrAppErr = vr::VRApplications()->SetApplicationAutoLaunch(
			OPENVR_APPLICATION_KEY, true);
		if (vrAppErr != vr::VRApplicationError_None)
		{
			CliExit("QuestCalibrator was registered, but SteamVR could not enable auto-launch.\n\n" +
				std::string(vr::VRApplications()->GetApplicationsErrorNameFromEnum(vrAppErr)), true);
		}
		CliExit("QuestCalibrator registered with SteamVR.\n\n" + manifestPath, false);
	}
	else if (cmd == L"-removemanifest")
	{
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr != vr::VRInitError_None)
			CliExit(InitErrorMessage(vrErr), true);

		std::string manifestPath = appDir;
		manifestPath += "\\manifest.vrmanifest";
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
		auto vrErr = vr::VRInitError_None;
		vr::VR_Init(&vrErr, vr::VRApplication_Utility);
		if (vrErr != vr::VRInitError_None)
			CliExit(InitErrorMessage(vrErr), true);

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
}
