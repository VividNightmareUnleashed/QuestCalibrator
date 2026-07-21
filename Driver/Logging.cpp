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

void OpenLogFile()
{
	LogFile = nullptr;

	std::string path = LogFilePath();
	if (!path.empty())
		LogFile = fopen(path.c_str(), "a");

	if (LogFile == nullptr)
	{
		char tempDir[MAX_PATH];
		DWORD len = GetTempPathA(MAX_PATH, tempDir);
		if (len > 0 && len < MAX_PATH)
			LogFile = fopen((std::string(tempDir) + "quest_calibrator_driver.log").c_str(), "a");
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
