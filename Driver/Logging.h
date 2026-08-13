#pragma once

#include <cstdio>
#include <ctime>

extern FILE *LogFile;

void OpenLogFile();
tm TimeForLog();
void LogFlush();

// The format string is the first variadic argument and the timestamp prefix is
// printed separately, so a call with no format arguments - LOG("IPC client
// connected") - needs no trailing comma to swallow. Passing __VA_ARGS__ after a
// comma only compiles under MSVC's traditional preprocessor; /Zc:preprocessor
// (the conforming mode, and the default for newer toolsets) and the clang-tidy
// pass both reject the empty case. The cost is that a line now reaches stdio in
// three calls instead of one, so two threads logging at once can interleave
// within a line; the timestamp prefix and the flush-per-line that makes the log
// survive a vrserver crash are unchanged.
#ifndef LOG
#define LOG(...) do { \
	tm logNow = TimeForLog(); \
	fprintf(LogFile, "[%02d:%02d:%02d] ", logNow.tm_hour, logNow.tm_min, logNow.tm_sec); \
	fprintf(LogFile, __VA_ARGS__); \
	fputc('\n', LogFile); \
	LogFlush(); \
} while (0)
#endif

#define TRACE(...) {}
