#include "Logging.h"
#include "Hooking.h"
#include "InterfaceHookInjector.h"
#include "ServerTrackedDeviceProvider.h"

#include <atomic>
#include <cstring>
#include <mutex>
#include <tlhelp32.h>
#include <vector>

namespace
{

std::atomic<ServerTrackedDeviceProvider *> Driver{ nullptr };
std::atomic<bool> AcceptingHookRequests{ false };
std::atomic<bool> PoseHook005Ready{ false };
std::atomic<bool> PoseHook006Ready{ false };
std::atomic<uint32_t> ActiveCallbacks{ 0 };
std::mutex HookSetupMutex;
bool MinHookInitialized = false;   // protected by HookSetupMutex

class CallbackGuard
{
public:
	CallbackGuard()
	{
		ActiveCallbacks.fetch_add(1, std::memory_order_seq_cst);
	}

	~CallbackGuard()
	{
		ActiveCallbacks.fetch_sub(1, std::memory_order_seq_cst);
	}

	CallbackGuard(const CallbackGuard &) = delete;
	CallbackGuard &operator=(const CallbackGuard &) = delete;
};

struct ModuleRange
{
	uintptr_t begin = 0;
	uintptr_t end = 0;

	bool IsValid() const
	{
		return begin != 0 && end > begin;
	}

	bool Contains(uintptr_t address) const
	{
		return address >= begin && address < end;
	}
};

// Resolved before the first target can be enabled and retained for the entire
// hook lifetime. Teardown cannot safely discover this information on demand:
// returning after a lookup failure would let the DLL unload while a disabled
// hook's already-entered detour frame can still return into this module.
ModuleRange DriverModuleRange;

bool GetThisModuleRange(ModuleRange &range)
{
	HMODULE module = nullptr;
	if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
		GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
		reinterpret_cast<LPCSTR>(&GetThisModuleRange), &module) || !module)
		return false;

	auto base = reinterpret_cast<uintptr_t>(module);
	auto dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(base);
	if (dos->e_magic != IMAGE_DOS_SIGNATURE)
		return false;
	auto nt = reinterpret_cast<const IMAGE_NT_HEADERS *>(base + dos->e_lfanew);
	if (nt->Signature != IMAGE_NT_SIGNATURE || nt->OptionalHeader.SizeOfImage == 0)
		return false;
	range = { base, base + nt->OptionalHeader.SizeOfImage };
	return true;
}

// Unwind a suspended x64 thread without symbols. Every detour frame and every
// return address back into one lives in this module; rejecting any such frame
// closes both the pre-CallbackGuard entry window and the post-guard epilogue
// window that a refcount alone cannot observe.
bool StackReferencesModule(HANDLE thread, const ModuleRange &module, bool &references)
{
	references = false;
	CONTEXT context{};
	context.ContextFlags = CONTEXT_FULL;
	if (!GetThreadContext(thread, &context))
		return false;

	for (int frame = 0; frame < 256 && context.Rip != 0; ++frame)
	{
		if (module.Contains(static_cast<uintptr_t>(context.Rip)))
		{
			references = true;
			return true;
		}

		DWORD64 previousRsp = context.Rsp;
		DWORD64 imageBase = 0;
		PRUNTIME_FUNCTION function = RtlLookupFunctionEntry(
			context.Rip, &imageBase, nullptr);
		if (function)
		{
			PVOID handlerData = nullptr;
			DWORD64 establisherFrame = 0;
			RtlVirtualUnwind(UNW_FLAG_NHANDLER, imageBase, context.Rip,
				function, &context, &handlerData, &establisherFrame, nullptr);
		}
		else
		{
			DWORD64 returnAddress = 0;
			SIZE_T bytesRead = 0;
			if (!ReadProcessMemory(GetCurrentProcess(),
				reinterpret_cast<const void *>(context.Rsp), &returnAddress,
				sizeof returnAddress, &bytesRead) || bytesRead != sizeof returnAddress)
				return false;
			context.Rip = returnAddress;
			context.Rsp += sizeof returnAddress;
		}

		if (context.Rsp <= previousRsp)
			return false;
	}
	return context.Rip == 0;
}

class SuspendedThreads
{
public:
	~SuspendedThreads() { Resume(); }

	bool SuspendAllOtherThreads()
	{
		HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
		if (snapshot == INVALID_HANDLE_VALUE)
			return false;

		// Open and store every handle before suspending anything. Growing a
		// std::vector after the first suspension can deadlock if an arbitrary
		// suspended thread owns the CRT heap lock.
		THREADENTRY32 entry{};
		entry.dwSize = sizeof entry;
		bool success = Thread32First(snapshot, &entry) != FALSE;
		DWORD processId = GetCurrentProcessId();
		DWORD currentThreadId = GetCurrentThreadId();
		while (success)
		{
			if (entry.th32OwnerProcessID == processId &&
				entry.th32ThreadID != currentThreadId)
			{
				HANDLE thread = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT |
					THREAD_QUERY_INFORMATION, FALSE, entry.th32ThreadID);
				if (!thread)
				{
					if (GetLastError() != ERROR_INVALID_PARAMETER)
					{
						CloseHandle(snapshot);
						CloseUnsuspendedHandles();
						return false;
					}
				}
				else
					threads.push_back(thread);
			}
			success = Thread32Next(snapshot, &entry) != FALSE;
		}
		DWORD enumerationError = GetLastError();
		CloseHandle(snapshot);
		if (enumerationError != ERROR_NO_MORE_FILES)
		{
			CloseUnsuspendedHandles();
			return false;
		}

		// The vector already contains every candidate handle, so this loop and
		// every error path below perform no heap allocation. Compact successfully
		// suspended handles in place while tolerating threads that exited after
		// the snapshot was taken.
		size_t suspendedCount = 0;
		for (size_t i = 0; i < threads.size(); ++i)
		{
			HANDLE thread = threads[i];
			if (SuspendThread(thread) != static_cast<DWORD>(-1))
			{
				threads[suspendedCount++] = thread;
				continue;
			}

			DWORD exitCode = STILL_ACTIVE;
			bool exited = GetExitCodeThread(thread, &exitCode) &&
				exitCode != STILL_ACTIVE;
			CloseHandle(thread);
			if (exited)
				continue;

			// Preserve the successfully suspended prefix for Resume(), and close
			// every candidate that has not yet been suspended.
			for (size_t remaining = i + 1; remaining < threads.size(); ++remaining)
				CloseHandle(threads[remaining]);
			threads.resize(suspendedCount);
			return false;
		}

		threads.resize(suspendedCount);
		return true;
	}

	bool AnyStackReferences(const ModuleRange &module, bool &references) const
	{
		references = false;
		for (HANDLE thread : threads)
		{
			bool oneReferences = false;
			if (!StackReferencesModule(thread, module, oneReferences))
				return false;
			if (oneReferences)
			{
				references = true;
				return true;
			}
		}
		return true;
	}

	void Resume()
	{
		for (HANDLE thread : threads)
		{
			ResumeThread(thread);
			CloseHandle(thread);
		}
		threads.clear();
	}

private:
	void CloseUnsuspendedHandles()
	{
		for (HANDLE thread : threads)
			CloseHandle(thread);
		threads.clear();
	}

	std::vector<HANDLE> threads;
};

// Quiescence can be unreachable rather than merely slow: a third-party thread
// parked in hand-written or generated code without unwind data fails the stack
// walk on every attempt, and nothing bounds how long it stays there. Retrying
// forever suspends every thread in vrserver once a millisecond for as long as
// that lasts, and hangs both SteamVR shutdown and (through Init's failure paths)
// SteamVR startup.
constexpr ULONGLONG QuiescenceWaitMs = 5000;

// Keeping this module resident is the safe way to stop waiting. Every hook is
// already disabled at this point, so the detours that remain only forward, and a
// pinned module can never unload under a frame that would return into it.
bool PinThisModule()
{
	HMODULE module = nullptr;
	return GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_PIN |
		GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
		reinterpret_cast<LPCSTR>(&GetThisModuleRange), &module) != FALSE;
}

// Returns false so callers know no quiescence proof was obtained; anything a
// detour can still touch must outlive this call.
bool AbandonTeardown(const char *reason)
{
	if (PinThisModule())
	{
		LOG("%s; keeping the driver module resident with its hooks disabled", reason);
		return false;
	}
	// Without the pin, returning would permit this DLL to unload under an active
	// detour frame, so waiting really is the only remaining safe outcome.
	LOG("%s, and the module could not be pinned (error %u); teardown must wait",
		reason, GetLastError());
	for (;;)
		Sleep(1000);
}

Hook<void*(*)(vr::IVRDriverContext *, const char *, vr::EVRInitError *)>
	GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

using PoseUpdateHook =
	Hook<void(*)(vr::IVRServerDriverHost *, uint32_t, const vr::DriverPose_t &, uint32_t)>;
using PoseUpdateDetour =
	void (*)(vr::IVRServerDriverHost *, uint32_t, const vr::DriverPose_t &, uint32_t);

PoseUpdateHook TrackedDevicePoseUpdatedHook005("IVRServerDriverHost005::TrackedDevicePoseUpdated");
PoseUpdateHook TrackedDevicePoseUpdatedHook006("IVRServerDriverHost006::TrackedDevicePoseUpdated");

// The one forwarding body every supported interface version shares. Keeping it
// here rather than once per detour is what stops the versions from silently
// diverging when the argument handling, the null-driver fallback or the guard
// changes. The detours themselves must stay distinct functions: each is the
// address MinHook patches in for its own target.
void ForwardPoseUpdate(PoseUpdateHook &hook, vr::IVRServerDriverHost *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	auto original = hook.originalFunc.load(std::memory_order_acquire);

	// unPoseStructSize is the caller's declaration of the layout behind
	// `newPose`, and it is the only signal that layout carries. Our copy below
	// is sized by the vendored vr::DriverPose_t: if a SteamVR build ever passes
	// a larger struct, forwarding that size over a smaller stack object makes
	// vrserver read past it, and the trailing bytes become pose fields. When the
	// size does not match, touch nothing - forward the caller's own object with
	// the caller's own size, and skip both the ring publish and the transform,
	// since the provider reads rotation/position/velocity from a layout we
	// cannot interpret. A layout change then degrades to "calibration not
	// applied", never to a corrupt or disappearing device. No logging here: this
	// is a pose thread, and stdio must not run on one.
	if (unPoseStructSize != sizeof(vr::DriverPose_t))
	{
		if (original)
			original(_this, unWhichDevice, newPose, unPoseStructSize);
		return;
	}

	ServerTrackedDeviceProvider *driver = Driver.load(std::memory_order_acquire);
	auto pose = newPose;
	if (!driver || driver->HandleDevicePoseUpdated(unWhichDevice, pose))
	{
		if (original)
			original(_this, unWhichDevice, pose, unPoseStructSize);
	}
}

void DetourTrackedDevicePoseUpdated005(vr::IVRServerDriverHost *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	CallbackGuard callback;
	ForwardPoseUpdate(TrackedDevicePoseUpdatedHook005, _this, unWhichDevice, newPose,
		unPoseStructSize);
}

void DetourTrackedDevicePoseUpdated006(vr::IVRServerDriverHost *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	CallbackGuard callback;
	ForwardPoseUpdate(TrackedDevicePoseUpdatedHook006, _this, unWhichDevice, newPose,
		unPoseStructSize);
}

// TrackedDevicePoseUpdated is the second virtual in the vendored
// IVRServerDriverHost_005 declaration, which is the only layout this repository
// can check. The 006 layout is not declared anywhere here, so the same index is
// an assumption for that branch; if a future revision reorders it, the detour is
// entered with mismatched arguments inside vrserver. Declaring 006 is what would
// settle it.
constexpr int PoseUpdateVTableIndex = 1;

// Adding a version means one row here rather than an edit at the install site,
// both teardown paths and the accessor.
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

bool DestroyEveryHook()
{
	bool success = GetGenericInterfaceHook.Destroy();
	for (const PoseHookBinding &binding : PoseHookBindings)
		success = binding.hook->Destroy() && success;
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

	std::lock_guard<std::mutex> lock(HookSetupMutex);
	if (!MinHookInitialized || !AcceptingHookRequests.load(std::memory_order_acquire))
		return;

	if (binding->ready->load(std::memory_order_relaxed))
		return;
	if (binding->hook->CreateHookInObjectVTable(
		originalInterface, PoseUpdateVTableIndex, binding->detour))
	{
		binding->ready->store(true, std::memory_order_release);
	}
}

void *DetourGetGenericInterface(vr::IVRDriverContext *_this,
	const char *pchInterfaceVersion, vr::EVRInitError *peError)
{
	CallbackGuard callback;
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
	if (!driver || !pDriverContext)
	{
		LOG("InjectHooks: invalid driver context");
		return false;
	}

	std::lock_guard<std::mutex> lock(HookSetupMutex);
	if (MinHookInitialized)
	{
		LOG("InjectHooks: hooks are already initialized");
		return false;
	}
	if (!DriverModuleRange.IsValid())
	{
		ModuleRange resolved;
		if (!GetThisModuleRange(resolved) || !resolved.IsValid())
		{
			LOG("InjectHooks: could not resolve the driver module range");
			return false;
		}
		DriverModuleRange = resolved;
	}

	MH_STATUS err = MH_Initialize();
	if (err != MH_OK)
	{
		LOG("MH_Initialize error: %s", MH_StatusToString(err));
		return false;
	}
	MinHookInitialized = true;
	Driver.store(driver, std::memory_order_release);

	if (!GetGenericInterfaceHook.CreateHookInObjectVTable(
		pDriverContext, 0, &DetourGetGenericInterface))
	{
		Driver.store(nullptr, std::memory_order_release);
		MH_Uninitialize();
		MinHookInitialized = false;
		return false;
	}

	AcceptingHookRequests.store(true, std::memory_order_release);
	return true;
}

bool IsPoseUpdateHookInstalled()
{
	for (const PoseHookBinding &binding : PoseHookBindings)
	{
		if (binding.ready->load(std::memory_order_acquire))
			return true;
	}
	return false;
}

bool DisableHooks()
{
	// Stop detours from starting new hook creation before waiting on the setup
	// mutex. A detour already waiting for the mutex rechecks this flag.
	AcceptingHookRequests.store(false, std::memory_order_release);
	Driver.store(nullptr, std::memory_order_release);
	for (const PoseHookBinding &binding : PoseHookBindings)
		binding.ready->store(false, std::memory_order_release);

	// Hook removal is only safe after every target is confirmed disabled. If
	// both the individual and MinHook-wide disable paths fail, keep the benign
	// forwarding detours installed and retry instead of taking a snapshot during
	// which a still-enabled target could form a new detour stack.
	ULONGLONG lastDisableWaitLog = 0;
	ModuleRange module;
	for (;;)
	{
		{
			std::lock_guard<std::mutex> lock(HookSetupMutex);
			if (!MinHookInitialized)
				return true;

			bool disabled = DisableEveryHook();
			if (!disabled)
			{
				LOG("One or more hooks could not be disabled individually; disabling all MinHook targets");
				MH_STATUS error = MH_DisableHook(MH_ALL_HOOKS);
				if (error != MH_OK && error != MH_ERROR_DISABLED)
					LOG("Failed to disable all MinHook targets: %s", MH_StatusToString(error));
				else
					disabled = DisableEveryHook();   // synchronize each hook's lifecycle state
			}
			if (disabled)
			{
				module = DriverModuleRange;
				break;
			}
		}

		ULONGLONG now = GetTickCount64();
		if (lastDisableWaitLog == 0 || now - lastDisableWaitLog >= 5000)
		{
			LOG("Hook targets are still enabled; teardown is waiting and will retry");
			lastDisableWaitLog = now;
		}
		// The lock guard must be destroyed before sleeping so an already-entered
		// GetGenericInterface detour can observe AcceptingHookRequests=false and
		// leave the module.
		Sleep(100);
	}

	// InjectHooks establishes this invariant before enabling even the context
	// detour. If memory corruption ever violates it, no stack can be inspected,
	// so the module has to stay loaded instead.
	if (!module.IsValid())
		return AbandonTeardown("Cached driver module range is unavailable, so detour "
			"stacks cannot be inspected");

	ULONGLONG quiescenceDeadline = GetTickCount64() + QuiescenceWaitMs;
	ULONGLONG lastWaitLog = GetTickCount64();
	for (;;)
	{
		std::unique_lock<std::mutex> lock(HookSetupMutex);
		if (!MinHookInitialized)
			return true;

		SuspendedThreads suspended;
		bool referencesModule = true;
		bool inspected = suspended.SuspendAllOtherThreads() &&
			suspended.AnyStackReferences(module, referencesModule);
		bool callbacksDrained = ActiveCallbacks.load(std::memory_order_seq_cst) == 0;
		if (inspected && !referencesModule && callbacksDrained)
		{
			// Hooks are disabled, so after this proven-empty snapshot no new
			// detour stack can form. Resume before freeing to avoid allocator or
			// loader-lock deadlocks against an unrelated suspended thread.
			suspended.Resume();
			if (!DestroyEveryHook())
			{
				LOG("One or more hooks could not be removed after quiescence; teardown will retry");
				lock.unlock();
				if (GetTickCount64() >= quiescenceDeadline)
					return AbandonTeardown("Hooks could not be removed after quiescence");
				Sleep(100);
				continue;
			}
			MH_STATUS err = MH_Uninitialize();
			if (err == MH_OK || err == MH_ERROR_NOT_INITIALIZED)
			{
				MinHookInitialized = false;
				DriverModuleRange = {};
				return true;
			}
			LOG("MH_Uninitialize error after quiescence: %s; teardown will retry",
				MH_StatusToString(err));
			lock.unlock();
			if (GetTickCount64() >= quiescenceDeadline)
				return AbandonTeardown("MinHook could not be uninitialized after quiescence");
			Sleep(100);
			continue;
		}

		suspended.Resume();
		lock.unlock();
		if (GetTickCount64() - lastWaitLog >= 5000)
		{
			LOG("Waiting for hook detour stacks to drain (inspect=%d, references=%d, active=%u)",
				inspected ? 1 : 0, referencesModule ? 1 : 0,
				ActiveCallbacks.load(std::memory_order_seq_cst));
			lastWaitLog = GetTickCount64();
		}
		// A failed inspection can be permanent, and a thread can sit in this
		// module for as long as its own work takes, so stop suspending every
		// thread in the process once the wait has run long enough.
		if (GetTickCount64() >= quiescenceDeadline)
			return AbandonTeardown("Hook detour stacks could not be proven drained");
		Sleep(1);
	}
}
