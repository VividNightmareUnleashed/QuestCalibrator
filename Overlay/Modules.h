#pragma once

// Optional parts of QuestCalibrator the installer lets the player leave out,
// so a plain install stays the classic calibrator (docs/modules.md).
//
// Install.ps1 records each module it installed as a DWORD under
// HKLM\Software\QuestCalibrator\Modules. The list is read once at startup
// and kept on CalibrationContext; everything a module gates asks it there.

namespace questcal
{

enum class ModuleStatus
{
	NotInstalled,
	Installed,
	NotBuilt, // announced in the UI, not shipped by any installer yet
};

struct Modules
{
	// Base station tools; the Lighthouse tab is the first, and more will
	// follow. Reading SteamVR's lighthouse log, and keeping a tracker's own
	// base station changes out of the drift evidence, is core and runs
	// either way (docs/modules.md says which side a new feature goes on).
	ModuleStatus lighthouse = ModuleStatus::NotInstalled;
	ModuleStatus smoothing = ModuleStatus::NotBuilt;

	static bool On(ModuleStatus s) { return s == ModuleStatus::Installed; }
};

// Reads what the installer recorded. A missing key or value is NotInstalled;
// modules that are not built stay NotBuilt whatever the registry says.
Modules ReadInstalledModules();

// Inline: Diagnostics.cpp, which SolverTests compiles, prints it.
inline const char *ModuleStatusName(ModuleStatus s)
{
	switch (s)
	{
	case ModuleStatus::Installed: return "installed";
	case ModuleStatus::NotBuilt: return "not built";
	case ModuleStatus::NotInstalled: break;
	}
	return "not installed";
}

} // namespace questcal
