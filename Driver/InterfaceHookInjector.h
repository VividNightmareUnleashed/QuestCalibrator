#pragma once

#include <openvr_driver.h>

class ServerTrackedDeviceProvider;

// Installs the driver-context hook. ServerTrackedDeviceProvider::Init invokes
// OpenVR context initialization through it, which must in turn install at least
// one supported pose hook before initialization is considered successful.
bool InjectHooks(ServerTrackedDeviceProvider *driver, vr::IVRDriverContext *pDriverContext);
bool IsPoseUpdateHookInstalled();
uint32_t PoseUpdateHookMask();
// Disables every hook and returns true once no pose callback can still reach
// the driver. The detours themselves stay in place, pinned with this module,
// and only forward. False means a callback was still inside the driver when
// the wait ran out: what it reads (the pose ring above all) must not be
// released by the caller.
bool DisableHooks();

#ifdef QUESTCAL_HOOK_INJECTOR_TEST_SEAM
// Called inside TryInstallPoseHook with the setup mutex held, after the accept
// recheck and before the ready flag is read.
extern void (*TryInstallAfterAcceptCheckForTest)();
// Called inside a pose detour once it has found the driver, before handing it
// the pose.
extern void (*InsideDriverCallbackForTest)();
// Called in DisableHooks once the hooks are disabled, before it waits for the
// pose callbacks inside the driver.
extern void (*BeforeDriverWaitForTest)();
#endif
