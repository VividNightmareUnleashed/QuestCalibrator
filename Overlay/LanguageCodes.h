#pragma once

#include <string>

// The language codes a saved setting may hold, in one place for both of their
// readers: the settings codec keeps a code only if it is listed here, and
// Localization maps each to the language at the same position in
// i18n::Language.
namespace questcal::i18n
{
inline constexpr const char *LanguageCodes[] = { "en", "ja", "it" };
inline constexpr int LanguageCodeCount = static_cast<int>(sizeof LanguageCodes / sizeof LanguageCodes[0]);

// The position of a saved code in LanguageCodes, or -1 for one this build does
// not know.
inline int LanguageCodeIndex(const std::string &code)
{
	for (int i = 0; i < LanguageCodeCount; ++i)
		if (code == LanguageCodes[i])
			return i;
	return -1;
}
}
