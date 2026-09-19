// A stand-in for SteamVR's vrclient_x64.dll, for testing the installer on a
// machine without SteamVR. The real openvr_api.dll that ships with the overlay
// loads it from <runtime>\bin, exactly as it would load SteamVR's.
//
// It implements only what the overlay's -installmanifest, -removemanifest and
// -activatemultipledrivers commands touch: IVRApplications (manifest
// registration, auto-launch, working directory) and IVRSettings (booleans).
// IVRSystem is handed out only because VR_Init insists on it; every slot of
// it traps. Registrations and settings persist in <runtime>\fake-openvr\
// state.txt so separate install, upgrade and uninstall runs see one another,
// and every call is appended to calls.log. A call outside that set is logged
// as UNEXPECTED and answered with an error, so the test can fail on it rather
// than on whatever the overlay does with a made-up answer.
//
// Behaviour follows SteamVR where the overlay depends on it: a second
// manifest with an app key that is already registered from another path is
// refused, and removing a manifest that is not registered is an error.

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "openvr.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

namespace
{

std::mutex g_mutex;

std::string RuntimeDir()
{
	HMODULE self = nullptr;
	GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
		GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
		reinterpret_cast<LPCSTR>(&RuntimeDir), &self);
	char path[MAX_PATH] = {};
	GetModuleFileNameA(self, path, MAX_PATH);
	std::string dir(path);
	// <runtime>\bin\vrclient_x64.dll or <runtime>\bin\win64\vrclient_x64.dll
	for (int up = 0; up < 3; ++up)
	{
		dir = dir.substr(0, dir.find_last_of("\\/"));
		std::string name = dir.substr(dir.find_last_of("\\/") + 1);
		if (_stricmp(name.c_str(), "bin") == 0)
			return dir.substr(0, dir.find_last_of("\\/"));
	}
	return dir;
}

std::string StateDir()
{
	std::string dir = RuntimeDir() + "\\fake-openvr";
	CreateDirectoryA(dir.c_str(), nullptr);
	return dir;
}

void Log(const std::string &line)
{
	std::ofstream out(StateDir() + "\\calls.log", std::ios::app);
	out << GetCurrentProcessId() << '\t' << line << '\n';
}

std::string Str(const char *s)
{
	return s ? s : "(null)";
}

std::string Lower(std::string s)
{
	for (char &c : s)
		c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
	return s;
}

// ---- persisted state ------------------------------------------------------

struct App
{
	std::string manifest;
	bool autoLaunch = false;
};

struct State
{
	std::map<std::string, App> apps;                  // app key -> registration
	std::map<std::string, bool> bools;                // "section\tkey" -> value
};

State Load()
{
	State state;
	std::ifstream in(StateDir() + "\\state.txt");
	std::string line;
	while (std::getline(in, line))
	{
		std::istringstream fields(line);
		std::string kind, a, b, c;
		std::getline(fields, kind, '\t');
		std::getline(fields, a, '\t');
		std::getline(fields, b, '\t');
		std::getline(fields, c, '\t');
		if (kind == "app")
			state.apps[a] = App{ b, c == "1" };
		else if (kind == "bool")
			state.bools[a + "\t" + b] = c == "1";
	}
	return state;
}

void Save(const State &state)
{
	std::ofstream out(StateDir() + "\\state.txt", std::ios::trunc);
	for (const auto &app : state.apps)
		out << "app\t" << app.first << '\t' << app.second.manifest << '\t'
			<< (app.second.autoLaunch ? 1 : 0) << '\n';
	for (const auto &setting : state.bools)
		out << "bool\t" << setting.first << '\t' << (setting.second ? 1 : 0) << '\n';
}

// The manifest's first "app_key" value. Enough for a well-formed manifest;
// anything else is an invalid manifest, as SteamVR would call it.
bool ReadAppKey(const std::string &path, std::string &key)
{
	std::ifstream in(path);
	if (!in)
		return false;
	std::stringstream text;
	text << in.rdbuf();
	const std::string body = text.str();
	size_t at = body.find("\"app_key\"");
	if (at == std::string::npos)
		return false;
	at = body.find(':', at);
	size_t open = at == std::string::npos ? at : body.find('"', at);
	size_t close = open == std::string::npos ? open : body.find('"', open + 1);
	if (close == std::string::npos)
		return false;
	key = body.substr(open + 1, close - open - 1);
	return !key.empty();
}

// ---- IVRApplications ------------------------------------------------------

using namespace vr;

class FakeApplications : public IVRApplications
{
public:
	EVRApplicationError AddApplicationManifest(const char *path, bool temporary) override
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		std::string key;
		EVRApplicationError result = VRApplicationError_None;
		State state = Load();
		if (!path || !ReadAppKey(path, key))
			result = VRApplicationError_InvalidManifest;
		else if (state.apps.count(key) && Lower(state.apps[key].manifest) != Lower(path))
			result = VRApplicationError_AppKeyAlreadyExists;
		else
		{
			bool autoLaunch = state.apps.count(key) ? state.apps[key].autoLaunch : false;
			state.apps[key] = App{ path, autoLaunch };
			Save(state);
		}
		Log("AddApplicationManifest\t" + Str(path) + "\t" + (temporary ? "temporary" : "permanent") +
			"\t" + key + "\t" + ErrorName(result));
		return result;
	}

	EVRApplicationError RemoveApplicationManifest(const char *path) override
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		State state = Load();
		EVRApplicationError result = VRApplicationError_UnknownApplication;
		for (auto it = state.apps.begin(); path && it != state.apps.end(); ++it)
		{
			if (Lower(it->second.manifest) == Lower(path))
			{
				state.apps.erase(it);
				Save(state);
				result = VRApplicationError_None;
				break;
			}
		}
		Log("RemoveApplicationManifest\t" + Str(path) + "\t" + ErrorName(result));
		return result;
	}

	bool IsApplicationInstalled(const char *key) override
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		bool installed = key && Load().apps.count(key) > 0;
		Log("IsApplicationInstalled\t" + Str(key) + "\t" + (installed ? "1" : "0"));
		return installed;
	}

	EVRApplicationError SetApplicationAutoLaunch(const char *key, bool autoLaunch) override
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		State state = Load();
		EVRApplicationError result = VRApplicationError_UnknownApplication;
		if (key && state.apps.count(key))
		{
			state.apps[key].autoLaunch = autoLaunch;
			Save(state);
			result = VRApplicationError_None;
		}
		Log("SetApplicationAutoLaunch\t" + Str(key) + "\t" + (autoLaunch ? "1" : "0") +
			"\t" + ErrorName(result));
		return result;
	}

	bool GetApplicationAutoLaunch(const char *key) override
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		State state = Load();
		bool autoLaunch = key && state.apps.count(key) && state.apps[key].autoLaunch;
		Log("GetApplicationAutoLaunch\t" + Str(key) + "\t" + (autoLaunch ? "1" : "0"));
		return autoLaunch;
	}

	uint32_t GetApplicationPropertyString(const char *key, EVRApplicationProperty property,
		char *buffer, uint32_t length, EVRApplicationError *error) override
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		State state = Load();
		EVRApplicationError result = VRApplicationError_None;
		std::string value;
		if (!key || !state.apps.count(key))
			result = VRApplicationError_UnknownApplication;
		else if (property != VRApplicationProperty_WorkingDirectory_String)
			result = VRApplicationError_UnknownProperty;
		else
		{
			const std::string &manifest = state.apps[key].manifest;
			value = manifest.substr(0, manifest.find_last_of("\\/"));
			if (!buffer || length < value.size() + 1)
				result = VRApplicationError_BufferTooSmall;
			else
				memcpy(buffer, value.c_str(), value.size() + 1);
		}
		if (error)
			*error = result;
		Log("GetApplicationPropertyString\t" + Str(key) + "\t" + std::to_string(property) +
			"\t" + value + "\t" + ErrorName(result));
		return static_cast<uint32_t>(value.empty() ? 0 : value.size() + 1);
	}

	const char *GetApplicationsErrorNameFromEnum(EVRApplicationError error) override
	{
		return ErrorName(error);
	}

	// Everything below is outside the install path.
	uint32_t GetApplicationCount() override { return Unexpected("GetApplicationCount"), 0; }
	EVRApplicationError GetApplicationKeyByIndex(uint32_t, char *, uint32_t) override { return Fail("GetApplicationKeyByIndex"); }
	EVRApplicationError GetApplicationKeyByProcessId(uint32_t, char *, uint32_t) override { return Fail("GetApplicationKeyByProcessId"); }
	EVRApplicationError LaunchApplication(const char *) override { return Fail("LaunchApplication"); }
	EVRApplicationError LaunchTemplateApplication(const char *, const char *, const AppOverrideKeys_t *, uint32_t) override { return Fail("LaunchTemplateApplication"); }
	EVRApplicationError LaunchApplicationFromMimeType(const char *, const char *) override { return Fail("LaunchApplicationFromMimeType"); }
	EVRApplicationError LaunchDashboardOverlay(const char *) override { return Fail("LaunchDashboardOverlay"); }
	bool CancelApplicationLaunch(const char *) override { return Unexpected("CancelApplicationLaunch"), false; }
	EVRApplicationError IdentifyApplication(uint32_t, const char *) override { return Fail("IdentifyApplication"); }
	uint32_t GetApplicationProcessId(const char *) override { return Unexpected("GetApplicationProcessId"), 0; }
	bool GetApplicationPropertyBool(const char *, EVRApplicationProperty, EVRApplicationError *error) override { return Unexpected("GetApplicationPropertyBool", error), false; }
	uint64_t GetApplicationPropertyUint64(const char *, EVRApplicationProperty, EVRApplicationError *error) override { return Unexpected("GetApplicationPropertyUint64", error), 0; }
	EVRApplicationError SetDefaultApplicationForMimeType(const char *, const char *) override { return Fail("SetDefaultApplicationForMimeType"); }
	bool GetDefaultApplicationForMimeType(const char *, char *, uint32_t) override { return Unexpected("GetDefaultApplicationForMimeType"), false; }
	bool GetApplicationSupportedMimeTypes(const char *, char *, uint32_t) override { return Unexpected("GetApplicationSupportedMimeTypes"), false; }
	uint32_t GetApplicationsThatSupportMimeType(const char *, char *, uint32_t) override { return Unexpected("GetApplicationsThatSupportMimeType"), 0; }
	uint32_t GetApplicationLaunchArguments(uint32_t, char *, uint32_t) override { return Unexpected("GetApplicationLaunchArguments"), 0; }
	EVRApplicationError GetStartingApplication(char *, uint32_t) override { return Fail("GetStartingApplication"); }
	EVRSceneApplicationState GetSceneApplicationState() override { return Unexpected("GetSceneApplicationState"), EVRSceneApplicationState_None; }
	EVRApplicationError PerformApplicationPrelaunchCheck(const char *) override { return Fail("PerformApplicationPrelaunchCheck"); }
	const char *GetSceneApplicationStateNameFromEnum(EVRSceneApplicationState) override { return Unexpected("GetSceneApplicationStateNameFromEnum"), "unexpected"; }
	EVRApplicationError LaunchInternalProcess(const char *, const char *, const char *) override { return Fail("LaunchInternalProcess"); }
	EVRApplicationError RegisterSubprocess(uint32_t) override { return Fail("RegisterSubprocess"); }
	uint32_t GetCurrentSceneProcessId() override { return Unexpected("GetCurrentSceneProcessId"), 0; }

private:
	static const char *ErrorName(EVRApplicationError error)
	{
		switch (error)
		{
		case VRApplicationError_None: return "VRApplicationError_None";
		case VRApplicationError_AppKeyAlreadyExists: return "VRApplicationError_AppKeyAlreadyExists";
		case VRApplicationError_UnknownApplication: return "VRApplicationError_UnknownApplication";
		case VRApplicationError_InvalidManifest: return "VRApplicationError_InvalidManifest";
		case VRApplicationError_UnknownProperty: return "VRApplicationError_UnknownProperty";
		case VRApplicationError_BufferTooSmall: return "VRApplicationError_BufferTooSmall";
		case VRApplicationError_NotImplemented: return "VRApplicationError_NotImplemented";
		default: return "VRApplicationError_Other";
		}
	}
	static void Unexpected(const char *name, EVRApplicationError *error = nullptr)
	{
		if (error)
			*error = VRApplicationError_NotImplemented;
		Log(std::string("UNEXPECTED\tIVRApplications::") + name);
	}
	static EVRApplicationError Fail(const char *name)
	{
		Unexpected(name);
		return VRApplicationError_NotImplemented;
	}
};

// ---- IVRSettings ----------------------------------------------------------

class FakeSettings : public IVRSettings
{
public:
	const char *GetSettingsErrorNameFromEnum(EVRSettingsError error) override
	{
		switch (error)
		{
		case VRSettingsError_None: return "VRSettingsError_None";
		case VRSettingsError_ReadFailed: return "VRSettingsError_ReadFailed";
		case VRSettingsError_WriteFailed: return "VRSettingsError_WriteFailed";
		default: return "VRSettingsError_Other";
		}
	}

	void SetBool(const char *section, const char *key, bool value, EVRSettingsError *error) override
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		State state = Load();
		state.bools[Str(section) + "\t" + Str(key)] = value;
		Save(state);
		if (error)
			*error = VRSettingsError_None;
		Log("SetBool\t" + Str(section) + "\t" + Str(key) + "\t" + (value ? "1" : "0"));
	}

	// An unset key reads as false with no error, like a SteamVR setting whose
	// default file says false.
	bool GetBool(const char *section, const char *key, EVRSettingsError *error) override
	{
		std::lock_guard<std::mutex> lock(g_mutex);
		State state = Load();
		auto it = state.bools.find(Str(section) + "\t" + Str(key));
		bool value = it != state.bools.end() && it->second;
		if (error)
			*error = VRSettingsError_None;
		Log("GetBool\t" + Str(section) + "\t" + Str(key) + "\t" + (value ? "1" : "0"));
		return value;
	}

	void SetInt32(const char *, const char *, int32_t, EVRSettingsError *error) override { Unexpected("SetInt32", error); }
	void SetFloat(const char *, const char *, float, EVRSettingsError *error) override { Unexpected("SetFloat", error); }
	void SetString(const char *, const char *, const char *, EVRSettingsError *error) override { Unexpected("SetString", error); }
	int32_t GetInt32(const char *, const char *, EVRSettingsError *error) override { return Unexpected("GetInt32", error), 0; }
	float GetFloat(const char *, const char *, EVRSettingsError *error) override { return Unexpected("GetFloat", error), 0.0f; }
	void GetString(const char *, const char *, char *value, uint32_t length, EVRSettingsError *error) override
	{
		if (value && length)
			value[0] = '\0';
		Unexpected("GetString", error);
	}
	void RemoveSection(const char *, EVRSettingsError *error) override { Unexpected("RemoveSection", error); }
	void RemoveKeyInSection(const char *, const char *, EVRSettingsError *error) override { Unexpected("RemoveKeyInSection", error); }

private:
	static void Unexpected(const char *name, EVRSettingsError *error)
	{
		if (error)
			*error = VRSettingsError_ReadFailed;
		Log(std::string("UNEXPECTED\tIVRSettings::") + name);
	}
};

// ---- IVRSystem ------------------------------------------------------------

// VR_Init returns an IVRSystem pointer and refuses to succeed without one,
// and it tells the runtime which SDK it was built against. Nothing else in
// the install commands touches IVRSystem. Every other vtable slot is a
// distinct function that logs its index and terminates, so a slip is loud and
// names the method (slots follow the declaration order in openvr.h).
constexpr int SystemSlots = 128;
constexpr int SetSDKVersionSlot = 50;   // IVRSystem_026

static_assert(std::string_view(IVRSystem_Version) == "IVRSystem_026",
	"IVRSystem changed: recount SetSDKVersionSlot against openvr.h");

EVRInitError __cdecl SetSDKVersion(void *, uint32_t major, uint32_t minor, uint32_t build)
{
	Log("SetSDKVersion\t" + std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(build));
	return VRInitError_None;
}

template <int Slot>
void __cdecl TrapSlot()
{
	Log("UNEXPECTED\tIVRSystem slot " + std::to_string(Slot));
	TerminateProcess(GetCurrentProcess(), 0xC0DE0000u + Slot);
}

template <int Slot>
constexpr void (*SystemEntry())()
{
	if constexpr (Slot == SetSDKVersionSlot)
		return reinterpret_cast<void (*)()>(&SetSDKVersion);
	else
		return &TrapSlot<Slot>;
}

template <int... Slots>
struct TrapTable
{
	static inline void (*const entries[sizeof...(Slots)])() = { SystemEntry<Slots>()... };
};

template <int... Slots>
constexpr TrapTable<Slots...> MakeTraps(std::integer_sequence<int, Slots...>)
{
	return {};
}

using SystemTraps = decltype(MakeTraps(std::make_integer_sequence<int, SystemSlots>()));

struct TrapObject
{
	void (*const *vtable)();
};

TrapObject g_system{ SystemTraps::entries };
FakeApplications g_applications;
FakeSettings g_settings;

// ---- IVRClientCore --------------------------------------------------------

// The loader-facing interface openvr_api.dll asks vrclient for. It is not in
// the public header; this is its layout (IVRClientCore_003).
class IVRClientCore
{
public:
	virtual EVRInitError Init(EVRApplicationType type, const char *startupInfo) = 0;
	virtual void Cleanup() = 0;
	virtual EVRInitError IsInterfaceVersionValid(const char *version) = 0;
	virtual void *GetGenericInterface(const char *version, EVRInitError *error) = 0;
	virtual bool BIsHmdPresent() = 0;
	virtual const char *GetEnglishStringForHmdError(EVRInitError error) = 0;
	virtual const char *GetIDForVRInitError(EVRInitError error) = 0;
};

void *Lookup(const char *version)
{
	if (!version)
		return nullptr;
	if (strcmp(version, IVRSystem_Version) == 0)
		return &g_system;
	if (strcmp(version, IVRApplications_Version) == 0)
		return static_cast<IVRApplications *>(&g_applications);
	if (strcmp(version, IVRSettings_Version) == 0)
		return static_cast<IVRSettings *>(&g_settings);
	return nullptr;
}

class FakeClientCore : public IVRClientCore
{
public:
	EVRInitError Init(EVRApplicationType type, const char *startupInfo) override
	{
		Log("Init\t" + std::to_string(type) + "\t" + Str(startupInfo));
		// Only the utility sessions the install commands open are supported; a
		// scene or overlay launch means something tried to run the real app.
		if (type != VRApplication_Utility)
		{
			Log("UNEXPECTED\tInit with application type " + std::to_string(type));
			return VRInitError_Init_InvalidApplicationType;
		}
		return VRInitError_None;
	}

	void Cleanup() override { Log("Cleanup"); }

	EVRInitError IsInterfaceVersionValid(const char *version) override
	{
		EVRInitError result = Lookup(version) ? VRInitError_None : VRInitError_Init_InterfaceNotFound;
		Log("IsInterfaceVersionValid\t" + Str(version) + "\t" + std::to_string(result));
		return result;
	}

	void *GetGenericInterface(const char *version, EVRInitError *error) override
	{
		void *found = Lookup(version);
		if (error)
			*error = found ? VRInitError_None : VRInitError_Init_InterfaceNotFound;
		if (!found)
			Log("UNEXPECTED\tGetGenericInterface " + Str(version));
		return found;
	}

	bool BIsHmdPresent() override { return false; }
	const char *GetEnglishStringForHmdError(EVRInitError) override { return "fake OpenVR runtime error"; }
	const char *GetIDForVRInitError(EVRInitError) override { return "VRInitError_Fake"; }
};

FakeClientCore g_core;

} // namespace

extern "C" __declspec(dllexport) void *VRClientCoreFactory(const char *interfaceName, int *returnCode)
{
	Log("VRClientCoreFactory\t" + Str(interfaceName));
	const bool known = interfaceName && strcmp(interfaceName, "IVRClientCore_003") == 0;
	if (returnCode)
		*returnCode = known ? VRInitError_None : VRInitError_Init_InterfaceNotFound;
	if (!known)
		Log("UNEXPECTED\tclient core version " + Str(interfaceName));
	return known ? static_cast<IVRClientCore *>(&g_core) : nullptr;
}
