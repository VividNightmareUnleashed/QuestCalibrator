#pragma once

// The translation tables behind Localization.cpp. Keys are the English text
// exactly as the UI draws it; a key holding printf conversions is a pattern
// whose translation names the values as {0}, {1}... (see Localization.h).

#include <cstddef>

namespace questcal::i18n
{
	struct Entry
	{
		const char *english;
		const char *translation;
	};

	extern const Entry kJapanese[];
	extern const size_t kJapaneseCount;

	// The language's own name, for the selector: shown as is in every language.
	extern const char *const kJapaneseNativeName;
}
