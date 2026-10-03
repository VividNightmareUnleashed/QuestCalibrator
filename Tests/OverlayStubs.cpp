// What the overlay sources linked into the tests expect from the parts of the
// app the tests leave out: openvr_api, the context and session log that
// Calibration.cpp owns, and Configuration.cpp's saves.
#include "../Overlay/Calibration.h"
#include "../Overlay/Configuration.h"

#include <openvr.h>

#include <string>

// openvr.h's accessors (vr::VRSystem() and the rest) reach these exports; the
// project defines OPENVR_BUILD_STATIC so plain definitions satisfy them. No
// runtime is present, so every interface is missing, as before SteamVR starts.
namespace vr
{
void *VR_CALLTYPE VR_GetGenericInterface(const char *, EVRInitError *error)
{
	if (error)
		*error = VRInitError_Init_NotInitialized;
	return nullptr;
}

uint32_t VR_CALLTYPE VR_GetInitToken()
{
	return 0;
}
}

CalibrationContext CalCtx;

void AppendSessionLog(const std::string &)
{
}

bool SaveSettings(CalibrationContext &)
{
	return true;
}

bool SavePendingChanges(CalibrationContext &)
{
	return true;
}
