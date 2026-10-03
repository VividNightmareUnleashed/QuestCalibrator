#pragma once

#include <cstdio>
#include <ctime>

extern FILE *LogFile;

void OpenLogFile();
tm TimeForLog();
void LogFlush();

// The format string travels inside __VA_ARGS__ so LOG("literal") compiles under
// the conforming preprocessor. A line takes three stdio calls, so concurrent
// lines can interleave; each is flushed so the log survives a vrserver crash.
// Each line carries its date, since one SteamVR session can run past midnight
// and the log spans many sessions.
#ifndef LOG
#define LOG(...) do { \
	tm logNow = TimeForLog(); \
	fprintf(LogFile, "[%04d-%02d-%02d %02d:%02d:%02d] ", logNow.tm_year + 1900, logNow.tm_mon + 1, \
		logNow.tm_mday, logNow.tm_hour, logNow.tm_min, logNow.tm_sec); \
	fprintf(LogFile, __VA_ARGS__); \
	fputc('\n', LogFile); \
	LogFlush(); \
} while (0)
#endif

#define TRACE(...) {}
