#pragma once

// The translation tables behind Localization.cpp: one UTF-8 file per language
// (Overlay/lang/*.txt, whose header describes the format), built into the
// executable by Overlay/Translations.rc. Keys are the English text exactly as
// the UI draws it; a key holding printf conversions is a pattern whose
// translation names the values as {0}, {1}... (see Localization.h).

#include <string>
#include <string_view>
#include <vector>

namespace questcal::i18n
{
	struct Entry
	{
		std::string english;
		std::string translation;
	};

	struct TableFile
	{
		// The language's own name, for the selector: shown as is in every language.
		std::string nativeName;
		std::vector<Entry> entries;
		// Lines the parser could not use, as "line N: why".
		std::vector<std::string> problems;
	};

	// Reads a table file whose translation lines start with `languageCode`.
	TableFile ParseTableFile(std::string_view text, std::string_view languageCode);

	// The tables built into this executable. A missing resource reads as an
	// empty table with a problem.
	const TableFile &JapaneseTable();
	const TableFile &ItalianTable();
}
