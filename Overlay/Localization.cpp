// Display-time translation; see Localization.h for the lookup order.
#include "Localization.h"
#include "LocalizationTables.h"

#include <windows.h>

#include <algorithm>
#include <fstream>
#include <optional>
#include <regex>
#include <set>
#include <unordered_map>
#include <vector>

namespace questcal::i18n
{
namespace
{

Language g_language = Language::English;
Language g_pending = Language::English;
// Latin-script languages draw with the bundled face; Japanese waits for
// the shell to find a font.
bool g_fontAvailable[kLanguageCount] = { true, false, true };

// Tr hands out pointers into this map, so it is only ever cleared between
// frames. Formatted text (ages, counts) adds entries as the numbers change;
// the bound keeps that from growing without limit.
std::unordered_map<std::string, std::string> g_cache;
constexpr size_t kCacheMax = 4096;

std::set<std::string> g_missing;
constexpr size_t kMissingMax = 2000;

struct Pattern
{
	std::regex re;
	std::string translation;
	size_t literalChars = 0;   // how specific the key is; see BuildTable
};

struct Table
{
	std::unordered_map<std::string, std::string> exact;
	std::vector<Pattern> patterns;
	// How the language joins what the English joined with ". " and ": ".
	std::string sentenceGap;
	std::string colon;
};

bool IsConversion(char c)
{
	return c == 'd' || c == 'i' || c == 'u' || c == 'f' || c == 'g' ||
		c == 's' || c == 'x' || c == 'X' || c == 'c';
}

// A printf-style key as a whole-string regex, one capture per conversion.
// Returns nothing when the key has no conversions: it is an exact key.
std::optional<std::string> KeyToRegex(const std::string &key)
{
	static const std::string special = "\\^$.|?*+()[]{}";
	std::string out;
	bool any = false;
	for (size_t i = 0; i < key.size(); ++i)
	{
		const char c = key[i];
		if (c == '%' && i + 1 < key.size() && key[i + 1] == '%')
		{
			out += '%';
			++i;
			continue;
		}
		if (c == '%')
		{
			size_t j = i + 1;
			while (j < key.size() && !IsConversion(key[j]))
				++j;
			if (j >= key.size())
				return std::nullopt;
			switch (key[j])
			{
			case 'd': case 'i': case 'u':
				out += "([-+]?\\d+)";
				break;
			case 'f': case 'g':
				out += "([-+]?(?:\\d+(?:\\.\\d*)?|inf|nan)[^ ,;:)]*)";
				break;
			case 'x': case 'X':
				out += "([0-9A-Fa-f]+)";
				break;
			default:
				out += "([\\s\\S]*?)";
				break;
			}
			any = true;
			i = j;
			continue;
		}
		if (special.find(c) != std::string::npos)
			out += '\\';
		out += c;
	}
	if (!any)
		return std::nullopt;
	return out;
}

Table BuildTable(const Entry *entries, size_t count, const char *sentenceGap, const char *colon)
{
	Table table;
	table.sentenceGap = sentenceGap;
	table.colon = colon;
	for (size_t i = 0; i < count; ++i)
	{
		const std::string key = entries[i].english;
		if (auto re = KeyToRegex(key))
		{
			size_t literal = 0;
			for (size_t j = 0; j < key.size(); ++j)
			{
				if (key[j] != '%')
				{
					++literal;
					continue;
				}
				while (j < key.size() && !IsConversion(key[j]))
					++j;
			}
			table.patterns.push_back({ std::regex(*re, std::regex::ECMAScript | std::regex::optimize),
				entries[i].translation, literal });
		}
		else
			table.exact.emplace(key, entries[i].translation);
	}
	// The most specific key wins: "Drift: %s; %u slips..." has to be tried
	// before "Drift: %s", whose %s would otherwise swallow the rest.
	std::stable_sort(table.patterns.begin(), table.patterns.end(),
		[](const Pattern &a, const Pattern &b) { return a.literalChars > b.literalChars; });
	return table;
}

const Table *TableFor(Language language)
{
	switch (language)
	{
	case Language::Japanese:
	{
		// Japanese runs sentences together and uses the full-width colon.
		static const Table japanese = BuildTable(kJapanese, kJapaneseCount, "", "\xEF\xBC\x9A");
		return &japanese;
	}
	case Language::Italian:
	{
		static const Table italian = BuildTable(kItalian, kItalianCount, " ", ": ");
		return &italian;
	}
	default:
		return nullptr;
	}
}

bool HasLetters(const std::string &s)
{
	for (unsigned char c : s)
		if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
			return true;
	return false;
}

std::optional<std::string> Translate(const Table &table, const std::string &text, int depth);

// A captured value: numbers and names stay as they are, text that has its
// own entry ("3 min ago") is translated.
std::string TranslateCapture(const Table &table, const std::string &value, int depth)
{
	if (!HasLetters(value) || depth > 3)
		return value;
	auto translated = Translate(table, value, depth + 1);
	return translated ? *translated : value;
}

std::optional<std::string> MatchPattern(const Table &table, const std::string &text, int depth)
{
	std::smatch m;
	for (const auto &pattern : table.patterns)
	{
		if (!std::regex_match(text, m, pattern.re))
			continue;
		std::string out;
		const std::string &t = pattern.translation;
		for (size_t i = 0; i < t.size(); ++i)
		{
			if (t[i] == '{' && i + 2 < t.size() && t[i + 1] >= '0' && t[i + 1] <= '9' && t[i + 2] == '}')
			{
				const size_t group = static_cast<size_t>(t[i + 1] - '0') + 1;
				if (group < m.size())
					out += TranslateCapture(table, m[group].str(), depth);
				i += 2;
				continue;
			}
			out += t[i];
		}
		return out;
	}
	return std::nullopt;
}

// Splits after ". ", "? ", "! " and at newlines, keeping each sentence's own
// punctuation; the separators come back as the pieces between.
std::vector<std::string> SplitSentences(const std::string &text)
{
	std::vector<std::string> pieces;
	size_t start = 0;
	for (size_t i = 0; i < text.size(); ++i)
	{
		const char c = text[i];
		if (c == '\n')
		{
			pieces.push_back(text.substr(start, i - start));
			pieces.push_back("\n");
			start = i + 1;
		}
		else if ((c == '.' || c == '?' || c == '!') && i + 1 < text.size() && text[i + 1] == ' ')
		{
			pieces.push_back(text.substr(start, i + 1 - start));
			pieces.push_back(" ");
			start = i + 2;
			++i;
		}
	}
	pieces.push_back(text.substr(start));
	return pieces;
}

std::optional<std::string> TranslateSentence(const Table &table, const std::string &text, int depth)
{
	auto exact = table.exact.find(text);
	if (exact != table.exact.end())
		return exact->second;
	if (auto matched = MatchPattern(table, text, depth))
		return matched;
	// "Headline: body", as the activity feed and some errors join them.
	const size_t colon = text.find(": ");
	if (colon != std::string::npos && depth < 3)
	{
		auto head = Translate(table, text.substr(0, colon), depth + 1);
		auto tail = Translate(table, text.substr(colon + 2), depth + 1);
		if (head || tail)
			return (head ? *head : text.substr(0, colon)) + table.colon +
				(tail ? *tail : text.substr(colon + 2));
	}
	return std::nullopt;
}

std::optional<std::string> Translate(const Table &table, const std::string &text, int depth)
{
	// Trailing newlines and spaces are formatting, not wording.
	size_t end = text.size();
	while (end > 0 && (text[end - 1] == '\n' || text[end - 1] == '\r' || text[end - 1] == ' '))
		--end;
	const std::string core = text.substr(0, end);
	const std::string tail = text.substr(end);
	if (core.empty())
		return std::nullopt;

	if (auto whole = TranslateSentence(table, core, depth))
		return *whole + tail;

	const std::vector<std::string> pieces = SplitSentences(core);
	if (pieces.size() < 2)
		return std::nullopt;
	std::string out;
	bool any = false;
	bool spacePending = false;     // a sentence gap waiting to see its neighbours
	bool previousEnglish = false;
	for (const auto &piece : pieces)
	{
		if (piece == " ")
		{
			spacePending = true;
			continue;
		}
		if (piece == "\n" || piece.empty())
		{
			out += piece;
			spacePending = false;
			previousEnglish = false;
			continue;
		}
		auto translated = TranslateSentence(table, piece, depth);
		// The language's own sentence gap; English left untranslated keeps
		// the space it had.
		if (spacePending)
			out += previousEnglish || !translated ? std::string(" ") : table.sentenceGap;
		spacePending = false;
		previousEnglish = !translated;
		if (translated)
		{
			out += *translated;
			any = true;
		}
		else
			out += piece;
	}
	if (!any)
		return std::nullopt;
	return out + tail;
}

Language WindowsUiLanguage()
{
	switch (PRIMARYLANGID(GetUserDefaultUILanguage()))
	{
	case LANG_JAPANESE: return Language::Japanese;
	case LANG_ITALIAN:  return Language::Italian;
	default:            return Language::English;
	}
}

} // namespace

Language LanguageFromCode(const std::string &code)
{
	if (code == "ja")
		return Language::Japanese;
	if (code == "it")
		return Language::Italian;
	if (code == "en")
		return Language::English;
	return SystemLanguage();
}

const char *LanguageCode(Language language)
{
	switch (language)
	{
	case Language::Japanese: return "ja";
	case Language::Italian:  return "it";
	default:                 return "en";
	}
}

Language SystemLanguage()
{
	const Language windows = WindowsUiLanguage();
	return FontAvailable(windows) ? windows : Language::English;
}

void SetLanguage(Language language)
{
	g_pending = language;
}

Language CurrentLanguage()
{
	return g_language;
}

void BeginFrame()
{
	if (g_pending != g_language)
	{
		g_language = g_pending;
		g_cache.clear();
	}
	if (g_cache.size() > kCacheMax)
		g_cache.clear();
}

void SetFontAvailable(Language language, bool available)
{
	g_fontAvailable[static_cast<int>(language)] = available;
}

bool FontAvailable(Language language)
{
	return g_fontAvailable[static_cast<int>(language)];
}

const char *Tr(const char *english)
{
	const Table *table = TableFor(g_language);
	if (!table || !english || !*english)
		return english;
	auto cached = g_cache.find(english);
	if (cached != g_cache.end())
		return cached->second.c_str();

	std::string key(english);
	auto translated = Translate(*table, key, 0);
	if (!translated && HasLetters(key) && g_missing.size() < kMissingMax)
		g_missing.insert(key);
	auto inserted = g_cache.emplace(std::move(key), translated ? *translated : std::string(english));
	return inserted.first->second.c_str();
}

std::string Tr(const std::string &english)
{
	return std::string(Tr(english.c_str()));
}

bool WriteMissing(const std::wstring &path)
{
	std::ofstream out(path, std::ios::binary);
	for (const auto &line : g_missing)
		out << line << "\n";
	return static_cast<bool>(out);
}

} // namespace questcal::i18n
