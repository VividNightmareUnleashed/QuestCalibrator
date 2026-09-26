#pragma once

#include <openvr_driver.h>

class ServerTrackedDeviceProvider;

// Installs the driver-context hook. ServerTrackedDeviceProvider::Init invokes
// OpenVR context initialization through it, which must in turn install at least
// one supported pose hook before initialization is considered successful.
bool InjectHooks(ServerTrackedDeviceProvider *driver, vr::IVRDriverContext *pDriverContext);
bool IsPoseUpdateHookInstalled();
uint32_t PoseUpdateHookMask();
// Disables every hook and returns true once no thread can still be inside this
// module's detours. False means the wait was abandoned: the module is pinned
// resident with its hooks disabled, and anything a detour reads (the pose ring
// above all) must not be released by the caller.
bool DisableHooks();
