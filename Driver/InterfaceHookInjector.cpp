#include "Logging.h"
#include "Hooking.h"
#include "InterfaceHookInjector.h"
#include "OpenVRHookLayout.h"
#include "HookLifecyclePolicy.h"
#include "ServerTrackedDeviceProvider.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>

#ifdef QUESTCAL_HOOK_INJECTOR_TEST_SEAM
void (*TryInstallAfterAcceptCheckForTest)() = nullptr;
void (*InsideDriverCallbackForTest)() = nullptr;
void (*BeforeDriverWaitForTest)() = nullptr;
#endif

namespace
{

std::atomic<ServerTrackedDeviceProvider *> Driver{ nullptr };
std::atomic<bool> AcceptingHookRequests{ false };
std::atomic<bool> PoseHook005Ready{ false };
std::atomic<bool> PoseHook006Ready{ false };
// Pose callbacks inside the driver, which teardown waits to leave.
questcal::hooks::CallbackActivity ActiveCallbacks;
// What the pose hook has seen since InjectHooks (protocol::HookStatus).
std::atomic<uint64_t> HookedDevices{ 0 };
std::atomic<uint32_t> MismatchedPoseUpdates{ 0 };
std::atomic<uint32_t> ReentrantPoseUpdates{ 0 };
// The device whose pose this thread is forwarding through a pose detour. A
// host whose TrackedDevicePoseUpdated calls another hooked version (005
// forwarding to 006, say) would otherwise transform the pose twice and publish
// the transformed one as raw.
thread_local uint32_t ForwardingDevice = vr::k_unTrackedDeviceIndexInvalid;
std::mutex HookSetupMutex;
bool MinHookInitialized = false;   // protected by HookSetupMutex
bool HooksInstalled = false;       // protected by HookSetupMutex
using CallbackGuard = questcal::hooks::CallbackActivity::Guard;

// HandleDevicePoseUpdated never blocks, so this only covers a pose thread that
// was preempted inside it.
constexpr std::chrono::milliseconds DriverReleaseWait{ 1000 };

// Once a hook has been enabled, a thread may be inside its detour or its
// trampoline at any later moment, and only stopping every thread in vrserver
// could prove otherwise. So neither ever goes away: this module stays loaded,
// and MinHook initialized with its trampolines, until the process exits.
// Teardown only disables the hooks, after which a detour still running only
// forwards.
bool PinThisModule()
{
	HMODULE module = nullptr;
	return GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_PIN |
		GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
		reinterpret_cast<LPCSTR>(&PinThisModule), &module) != FALSE;
}

Hook<void*(*)(vr::IVRDriverContext *, const char *, vr::EVRInitError *)>
	GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

using PoseUpdateHook =
	Hook<void(*)(void *, uint32_t, const vr::DriverPose_t &, uint32_t)>;
using PoseUpdateDetour =
	void (*)(void *, uint32_t, const vr::DriverPose_t &, uint32_t);

PoseUpdateHook TrackedDevicePoseUpdatedHook005("IVRServerDriverHost005::TrackedDevicePoseUpdated");
PoseUpdateHook TrackedDevicePoseUpdatedHook006("IVRServerDriverHost006::TrackedDevicePoseUpdated");

// The forwarding body every interface version shares. The detours stay
// distinct functions because each is the address MinHook patches in.
void ForwardPoseUpdate(PoseUpdateHook &hook, void *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	// Set before the hook was first enabled and never cleared, since hooks are
	// disabled but not removed. A call through null would take vrserver down,
	// so it is checked all the same.
	auto original = hook.originalFunc.load(std::memory_order_acquire);

	// The outer detour already handled this device's pose.
	if (ForwardingDevice == unWhichDevice)
	{
		ReentrantPoseUpdates.fetch_add(1, std::memory_order_relaxed);
		if (original)
			original(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}
	struct Forwarding
	{
		uint32_t outer;
		~Forwarding() { ForwardingDevice = outer; }
	} forwarding{ ForwardingDevice };
	ForwardingDevice = unWhichDevice;

	// Our copy is sized by the vendored DriverPose_t, so a caller passing a
	// different layout gets its own object forwarded untouched: no ring publish,
	// no transform. A layout change then degrades to "calibration not applied",
	// never to a corrupt device. No logging: stdio must not run on a pose thread.
	if (unPoseStructSize != sizeof(vr::DriverPose_t))
	{
		MismatchedPoseUpdates.fetch_add(1, std::memory_order_relaxed);
		if (original)
			original(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}

	if (unWhichDevice < vr::k_unMaxTrackedDeviceCount)
	{
		// A load first, so after a device's first pose this is all it costs.
		const uint64_t bit = uint64_t{ 1 } << unWhichDevice;
		if ((HookedDevices.load(std::memory_order_relaxed) & bit) == 0)
			HookedDevices.fetch_or(bit, std::memory_order_relaxed);
	}

	auto pose = newPose;
	{
		// Counted from before Driver is read until the driver is done. The count
		// and this load are sequentially consistent with DisableHooks' store, so
		// a callback is either counted when teardown looks or finds no driver.
		CallbackGuard callback(ActiveCallbacks);
		if (ServerTrackedDeviceProvider *driver = Driver.load(std::memory_order_seq_cst))
		{
#ifdef QUESTCAL_HOOK_INJECTOR_TEST_SEAM
			if (InsideDriverCallbackForTest)
				InsideDriverCallbackForTest();
#endif
			driver->HandleDevicePoseUpdated(unWhichDevice, pose);
		}
	}
	if (original)
		original(_this, unWhichDevice, pose, unPoseStructSize);
}

void DetourTrackedDevicePoseUpdated005(void *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	ForwardPoseUpdate(TrackedDevicePoseUpdatedHook005, _this, unWhichDevice, newPose,
		unPoseStructSize);
}

void DetourTrackedDevicePoseUpdated006(void *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	ForwardPoseUpdate(TrackedDevicePoseUpdatedHook006, _this, unWhichDevice, newPose,
		unPoseStructSize);
}

// Supporting another interface version is one row here.
struct PoseHookBinding
{
	const char *interfaceVersion;
	PoseUpdateHook *hook;
	PoseUpdateDetour detour;
	std::atomic<bool> *ready;
};

const PoseHookBinding PoseHookBindings[] = {
	{ "IVRServerDriverHost_005", &TrackedDevicePoseUpdatedHook005,
		&DetourTrackedDevicePoseUpdated005, &PoseHook005Ready },
	{ "IVRServerDriverHost_006", &TrackedDevicePoseUpdatedHook006,
		&DetourTrackedDevicePoseUpdated006, &PoseHook006Ready },
};

bool DisableEveryHook()
{
	// The context hook goes first: while it is enabled it can still install a new
	// pose hook behind the ones being disabled here.
	bool success = GetGenericInterfaceHook.Disable();
	for (const PoseHookBinding &binding : PoseHookBindings)
		success = binding.hook->Disable() && success;
	return success;
}

void TryInstallPoseHook(const char *interfaceVersion, void *originalInterface)
{
	if (!AcceptingHookRequests.load(std::memory_order_acquire))
		return;

	const PoseHookBinding *binding = nullptr;
	for (const PoseHookBinding &candidate : PoseHookBindings)
	{
		if (std::strcmp(interfaceVersion, candidate.interfaceVersion) == 0)
		{
			binding = &candidate;
			break;
		}
	}
	if (!binding)
		return;

	if (!originalInterface)
	{
		LOG("Cannot hook %s: GetGenericInterface returned null", interfaceVersion);
		return;
	}

	// Under the mutex, accepting implies MinHook is initialized.
	std::lock_guard<std::mutex> lock(HookSetupMutex);
	if (!AcceptingHookRequests.load(std::memory_order_acquire))
		return;
#ifdef QUESTCAL_HOOK_INJECTOR_TEST_SEAM
	if (TryInstallAfterAcceptCheckForTest)
		TryInstallAfterAcceptCheckForTest();
#endif

	if (binding->ready->load(std::memory_order_relaxed))
		return;
	if (binding->hook->CreateHookInObjectVTable(
		originalInterface, openvr_hook::PoseUpdateSlot, binding->detour))
	{
		binding->ready->store(true, std::memory_order_release);
	}
}

void *DetourGetGenericInterface(vr::IVRDriverContext *_this,
	const char *pchInterfaceVersion, vr::EVRInitError *peError)
{
	auto original = GetGenericInterfaceHook.originalFunc.load(std::memory_order_acquire);
	if (!original)
		return nullptr;

	if (!pchInterfaceVersion)
	{
		LOG("IVRDriverContext::GetGenericInterface called with a null version");
		if (peError)
			*peError = vr::VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}

	TRACE("ServerTrackedDeviceProvider::DetourGetGenericInterface(%s)", pchInterfaceVersion);
	void *originalInterface = original(_this, pchInterfaceVersion, peError);
	TryInstallPoseHook(pchInterfaceVersion, originalInterface);
	return originalInterface;
}

} // namespace

bool InjectHooks(ServerTrackedDeviceProvider *driver, vr::IVRDriverContext *pDriverContext)
{
	if (!pDriverContext)
	{
		LOG("InjectHooks: invalid driver context");
		return false;
	}

	std::lock_guard<std::mutex> lock(HookSetupMutex);
	if (HooksInstalled)
	{
		LOG("InjectHooks: hooks are already installed");
		return false;
	}
	HookedDevices.store(0, std::memory_order_relaxed);
	MismatchedPoseUpdates.store(0, std::memory_order_relaxed);
	ReentrantPoseUpdates.store(0, std::memory_order_relaxed);
	// Once per process; a later Init enables the hooks the first one created.
	if (!MinHookInitialized)
	{
		if (!PinThisModule())
		{
			LOG("InjectHooks: could not pin the driver module (error %u)", GetLastError());
			return false;
		}
		MH_STATUS err = MH_Initialize();
		if (err != MH_OK)
		{
			LOG("MH_Initialize error: %s", MH_StatusToString(err));
			return false;
		}
		MinHookInitialized = true;
	}
	Driver.store(driver, std::memory_order_release);

	if (!GetGenericInterfaceHook.CreateHookInObjectVTable(
		pDriverContext, openvr_hook::GetGenericInterfaceSlot, &DetourGetGenericInterface))
	{
		Driver.store(nullptr, std::memory_order_release);
		return false;
	}

	HooksInstalled = true;
	AcceptingHookRequests.store(true, std::memory_order_release);
	return true;
}

bool IsPoseUpdateHookInstalled()
{
	return PoseUpdateHookMask() != 0;
}

uint32_t PoseUpdateHookMask()
{
	uint32_t mask = 0;
	if (PoseHook005Ready.load(std::memory_order_acquire))
		mask |= protocol::PoseHook005;
	if (PoseHook006Ready.load(std::memory_order_acquire))
		mask |= protocol::PoseHook006;
	return mask;
}

protocol::HookStatus PoseHookStatus()
{
	protocol::HookStatus status;
	status.hookedDevices = HookedDevices.load(std::memory_order_relaxed);
	status.mismatchedPoseUpdates = MismatchedPoseUpdates.load(std::memory_order_relaxed);
	status.reentrantPoseUpdates = ReentrantPoseUpdates.load(std::memory_order_relaxed);
	return status;
}

bool DisableHooks()
{
	// Stop detours from starting new hook creation before waiting on the setup
	// mutex. A detour already waiting for the mutex rechecks this flag.
	AcceptingHookRequests.store(false, std::memory_order_release);
	// Sequentially consistent, like the callbacks' count and their load of
	// Driver: once the wait below finds no callback counted, every later
	// callback finds no driver.
	Driver.store(nullptr, std::memory_order_seq_cst);

	{
		std::lock_guard<std::mutex> lock(HookSetupMutex);
		// Under the mutex, not before it: a detour that passed its accept
		// check before the store above sets its ready flag while holding
		// the mutex. Cleared earlier, that flag outlives the hook it names,
		// and the next Init in this process skips installing the hook while
		// IsPoseUpdateHookInstalled still reports it.
		for (const PoseHookBinding &binding : PoseHookBindings)
			binding.ready->store(false, std::memory_order_release);
		HooksInstalled = false;
		if (MinHookInitialized && !DisableEveryHook())
		{
			LOG("One or more hooks could not be disabled individually; disabling all MinHook targets");
			MH_STATUS error = MH_DisableHook(MH_ALL_HOOKS);
			if (error != MH_OK && error != MH_ERROR_DISABLED)
				LOG("Failed to disable all MinHook targets: %s; their detours stay in place and only forward",
					MH_StatusToString(error));
			else
				DisableEveryHook();   // synchronize each hook's lifecycle state
		}
	}

	// A thread still inside a detour finishes safely (see PinThisModule); only
	// the driver it may have reached needs waiting for.
#ifdef QUESTCAL_HOOK_INJECTOR_TEST_SEAM
	if (BeforeDriverWaitForTest)
		BeforeDriverWaitForTest();
#endif
	if (ActiveCallbacks.WaitUntilIdle(DriverReleaseWait))
		return true;
	LOG("A pose callback was still inside the driver after %lld ms",
		static_cast<long long>(DriverReleaseWait.count()));
	return false;
}
