#include "stdafx.h"
#include "Modules.h"

namespace questcal
{

namespace
{

ModuleStatus ReadModule(const char *name)
{
	DWORD value = 0;
	DWORD size = sizeof(value);
	const LSTATUS result = RegGetValueA(HKEY_LOCAL_MACHINE, "Software\\QuestCalibrator\\Modules", name,
		RRF_RT_REG_DWORD, nullptr, &value, &size);
	return result == ERROR_SUCCESS && value != 0 ? ModuleStatus::Installed : ModuleStatus::NotInstalled;
}

} // namespace

Modules ReadInstalledModules()
{
	Modules modules;
	modules.lighthouse = ReadModule("Lighthouse");
	return modules;
}

} // namespace questcal
