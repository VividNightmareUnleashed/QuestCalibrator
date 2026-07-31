#pragma once

#include <openvr_driver.h>

class ServerTrackedDeviceProvider;

// Installs the driver-context hook. ServerTrackedDeviceProvider::Init invokes
// OpenVR context initialization through it, which must in turn install at least
// one supported pose hook before initialization is considered successful.
bool InjectHooks(ServerTrackedDeviceProvider *driver, vr::IVRDriverContext *pDriverContext);
bool IsPoseUpdateHookInstalled();
void DisableHooks();
