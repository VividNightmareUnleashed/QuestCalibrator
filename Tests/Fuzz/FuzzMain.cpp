// libFuzzer entry point for one of the targets in FuzzTargets.h, chosen with
// /DQUESTCAL_FUZZ_TARGET=name. Built and run by tools/fuzz.ps1 with
// /fsanitize=fuzzer,address; not part of the solution.
#include "FuzzTargets.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>

#ifndef QUESTCAL_FUZZ_TARGET
#error define QUESTCAL_FUZZ_TARGET as a target name from FuzzTargets.h
#endif
#define QUESTCAL_FUZZ_STRING2(x) #x
#define QUESTCAL_FUZZ_STRING(x) QUESTCAL_FUZZ_STRING2(x)

namespace
{
const char *const Name = QUESTCAL_FUZZ_STRING(QUESTCAL_FUZZ_TARGET);

const questcalfuzz::Target &Chosen()
{
	for (const auto &t : questcalfuzz::Targets())
		if (std::strcmp(t.name, Name) == 0)
			return t;
	std::fprintf(stderr, "no fuzz target named %s\n", Name);
	std::abort();
}
}

// With QUESTCAL_FUZZ_SEED_DIR set, writes the target's seeds there first, so
// the corpus starts from inputs that get past the parser.
extern "C" int LLVMFuzzerInitialize(int *, char ***)
{
	char *dir = nullptr;
	size_t length = 0;
	if (_dupenv_s(&dir, &length, "QUESTCAL_FUZZ_SEED_DIR") == 0 && dir)
	{
		std::filesystem::create_directories(dir);
		int n = 0;
		for (const std::string &seed : Chosen().seeds())
		{
			std::ofstream out(std::filesystem::path(dir) / ("seed-" + std::to_string(n++)), std::ios::binary);
			out.write(seed.data(), static_cast<std::streamsize>(seed.size()));
		}
		free(dir);
	}
	return 0;
}

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
	const std::string why = Chosen().check(data, size);
	if (!why.empty())
	{
		std::fprintf(stderr, "property failed (%s): %s\n", Name, why.c_str());
		std::abort();
	}
	return 0;
}
