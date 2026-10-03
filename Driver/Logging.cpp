#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <chrono>
#include <string>

FILE *LogFile;

// The driver runs inside vrserver.exe, whose working directory is not ours to
// rely on. Anchor the log next to the driver DLL itself so it is discoverable
// (...\01questcalibrator\bin\win64\quest_calibrator_driver.log).
static std::string LogFilePath()
{
	HMODULE module = nullptr;
	if (!GetModuleHandleExA(
		GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
		reinterpret_cast<LPCSTR>(&LogFilePath), &module))
	{
		return "";
	}

	char path[MAX_PATH];
	DWORD len = GetModuleFileNameA(module, path, MAX_PATH);
	if (len == 0 || len >= MAX_PATH)
		return "";

	std::string dir(path, len);
	size_t slash = dir.find_last_of("\\/");
	if (slash == std::string::npos)
		return "";

	return dir.substr(0, slash + 1) + "quest_calibrator_driver.log";
}

// A log past this size is set aside as quest_calibrator_driver.old.log when
// the driver next loads, replacing the previous one, so the two stay bounded.
static constexpr unsigned long long MaxLogBytes = 4ull * 1024 * 1024;

static FILE *OpenRotatedLog(const std::string &path)
{
	WIN32_FILE_ATTRIBUTE_DATA attributes;
	if (GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &attributes) &&
		((static_cast<unsigned long long>(attributes.nFileSizeHigh) << 32) |
			attributes.nFileSizeLow) > MaxLogBytes)
	{
		const std::string previous = path.substr(0, path.size() - 4) + ".old.log";
		MoveFileExA(path.c_str(), previous.c_str(), MOVEFILE_REPLACE_EXISTING);
	}
	return fopen(path.c_str(), "a");
}

void OpenLogFile()
{
	LogFile = nullptr;

	std::string path = LogFilePath();
	if (!path.empty())
		LogFile = OpenRotatedLog(path);

	if (LogFile == nullptr)
	{
		char tempDir[MAX_PATH];
		DWORD len = GetTempPathA(MAX_PATH, tempDir);
		if (len > 0 && len < MAX_PATH)
			LogFile = OpenRotatedLog(std::string(tempDir) + "quest_calibrator_driver.log");
	}

	if (LogFile == nullptr)
		LogFile = stderr;
}

tm TimeForLog()
{
	auto now = std::chrono::system_clock::now();
	auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value;
	localtime_s(&value, &nowTime);
	return value;
}

void LogFlush()
{
	fflush(LogFile);
}
