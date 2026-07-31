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

Hook<void*(*)(vr::IVRDriverContext *, const char *, vr::EVRInitError *)>
	GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

Hook<void(*)(vr::IVRServerDriverHost *, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook005("IVRServerDriverHost005::TrackedDevicePoseUpdated");

Hook<void(*)(vr::IVRServerDriverHost *, uint32_t, const vr::DriverPose_t &, uint32_t)>
	TrackedDevicePoseUpdatedHook006("IVRServerDriverHost006::TrackedDevicePoseUpdated");

bool DisableEveryHook()
{
	bool success = true;
	success = GetGenericInterfaceHook.Disable() && success;
	success = TrackedDevicePoseUpdatedHook005.Disable() && success;
	success = TrackedDevicePoseUpdatedHook006.Disable() && success;
	return success;
}

bool DestroyEveryHook()
{
	bool success = true;
	success = GetGenericInterfaceHook.Destroy() && success;
	success = TrackedDevicePoseUpdatedHook005.Destroy() && success;
	success = TrackedDevicePoseUpdatedHook006.Destroy() && success;
	return success;
}

void DetourTrackedDevicePoseUpdated005(vr::IVRServerDriverHost *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	CallbackGuard callback;
	auto original = TrackedDevicePoseUpdatedHook005.originalFunc.load(std::memory_order_acquire);
	ServerTrackedDeviceProvider *driver = Driver.load(std::memory_order_acquire);
	auto pose = newPose;
	if (!driver || driver->HandleDevicePoseUpdated(unWhichDevice, pose))
	{
		if (original)
			original(_this, unWhichDevice, pose, unPoseStructSize);
	}
}

void DetourTrackedDevicePoseUpdated006(vr::IVRServerDriverHost *_this,
	uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	CallbackGuard callback;
	auto original = TrackedDevicePoseUpdatedHook006.originalFunc.load(std::memory_order_acquire);
	ServerTrackedDeviceProvider *driver = Driver.load(std::memory_order_acquire);
	auto pose = newPose;
	if (!driver || driver->HandleDevicePoseUpdated(unWhichDevice, pose))
	{
		if (original)
			original(_this, unWhichDevice, pose, unPoseStructSize);
	}
}

void TryInstallPoseHook(const char *interfaceVersion, void *originalInterface)
{
	if (!AcceptingHookRequests.load(std::memory_order_acquire))
		return;

	const bool is005 = std::strcmp(interfaceVersion, "IVRServerDriverHost_005") == 0;
	const bool is006 = std::strcmp(interfaceVersion, "IVRServerDriverHost_006") == 0;
	if (!is005 && !is006)
		return;

	if (!originalInterface)
	{
		LOG("Cannot hook %s: GetGenericInterface returned null", interfaceVersion);
		return;
	}

	std::lock_guard<std::mutex> lock(HookSetupMutex);
	if (!MinHookInitialized || !AcceptingHookRequests.load(std::memory_order_acquire))
		return;

	if (is005 && !PoseHook005Ready.load(std::memory_order_relaxed))
	{
		if (TrackedDevicePoseUpdatedHook005.CreateHookInObjectVTable(
			originalInterface, 1, &DetourTrackedDevicePoseUpdated005))
		{
			PoseHook005Ready.store(true, std::memory_order_release);
		}
	}
	else if (is006 && !PoseHook006Ready.load(std::memory_order_relaxed))
	{
		if (TrackedDevicePoseUpdatedHook006.CreateHookInObjectVTable(
			originalInterface, 1, &DetourTrackedDevicePoseUpdated006))
		{
			PoseHook006Ready.store(true, std::memory_order_release);
		}
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
	return PoseHook005Ready.load(std::memory_order_acquire)
		|| PoseHook006Ready.load(std::memory_order_acquire);
}

void DisableHooks()
{
	// Stop detours from starting new hook creation before waiting on the setup
	// mutex. A detour already waiting for the mutex rechecks this flag.
	AcceptingHookRequests.store(false, std::memory_order_release);
	Driver.store(nullptr, std::memory_order_release);
	PoseHook005Ready.store(false, std::memory_order_release);
	PoseHook006Ready.store(false, std::memory_order_release);

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
				return;

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
	// detour. If memory corruption ever violates it, waiting is the only safe
	// outcome: returning would permit this DLL to unload under an active frame.
	if (!module.IsValid())
	{
		LOG("Cached driver module range is unavailable; hook teardown cannot continue safely");
		for (;;)
			Sleep(1000);
	}

	ULONGLONG lastWaitLog = GetTickCount64();
	for (;;)
	{
		std::unique_lock<std::mutex> lock(HookSetupMutex);
		if (!MinHookInitialized)
			return;

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
				Sleep(100);
				continue;
			}
			MH_STATUS err = MH_Uninitialize();
			if (err == MH_OK || err == MH_ERROR_NOT_INITIALIZED)
			{
				MinHookInitialized = false;
				DriverModuleRange = {};
				return;
			}
			LOG("MH_Uninitialize error after quiescence: %s; teardown will retry",
				MH_StatusToString(err));
			lock.unlock();
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
		Sleep(1);
	}
}
