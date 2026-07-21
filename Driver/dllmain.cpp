#include "Logging.h"
#include "../common/Version.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdio>

BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		OpenLogFile();
		LOG("QuestCalibratorDriver " QUESTCAL_VERSION_STRING " loaded");
		break;
	case DLL_PROCESS_DETACH:
		LOG("QuestCalibratorDriver unloaded");
		break;
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
		break;
	}
	return TRUE;
}

