// Display-time translation; see Localization.h for the lookup order.
#include "Localization.h"
#include "LanguageCodes.h"
#include "LocalizationTables.h"

#include <windows.h>

#include <algorithm>
#include <fstream>
#include <optional>
#include <regex>
#include <set>
#include <string_view>
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

// Which of a translation's plural forms ({0|one|other}) a count takes.
using PluralRule = size_t (*)(unsigned long long count);

// Japanese has a single form; Italian, like English, one for 1 and another
// for every other count.
size_t SingleForm(unsigned long long)
{
	return 0;
}

size_t OneOrOther(unsigned long long count)
{
	return count == 1 ? 0 : 1;
}

struct Table
{
	std::unordered_map<std::string, std::string> exact;
	std::vector<Pattern> patterns;
	// How the language joins what the English joined with ". " and ": ".
	std::string sentenceGap;
	std::string colon;
	PluralRule plural = SingleForm;
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

// "\n" is a line break and "\\" a backslash; any other backslash is text.
std::string Unescape(std::string_view text)
{
	std::string out;
	out.reserve(text.size());
	for (size_t i = 0; i < text.size(); ++i)
	{
		if (text[i] == '\\' && i + 1 < text.size() && (text[i + 1] == 'n' || text[i + 1] == '\\'))
		{
			out += text[i + 1] == 'n' ? '\n' : '\\';
			++i;
			continue;
		}
		out += text[i];
	}
	return out;
}

bool StartsWith(std::string_view text, std::string_view prefix)
{
	return text.substr(0, prefix.size()) == prefix;
}

// RT_RCDATA resources are file bytes; the tables are UTF-8 text.
std::string_view BuiltInFile(const wchar_t *name)
{
	const HMODULE module = GetModuleHandleW(nullptr);
	const HRSRC resource = FindResourceW(module, name, MAKEINTRESOURCEW(10));
	if (!resource)
		return {};
	const HGLOBAL loaded = LoadResource(module, resource);
	const DWORD size = SizeofResource(module, resource);
	const void *bytes = loaded ? LockResource(loaded) : nullptr;
	if (!bytes || size == 0)
		return {};
	return std::string_view(static_cast<const char *>(bytes), size);
}

TableFile LoadBuiltInTable(const wchar_t *resource, std::string_view languageCode)
{
	const std::string_view text = BuiltInFile(resource);
	if (text.empty())
	{
		TableFile missing;
		missing.problems.push_back("the built-in table is missing");
		return missing;
	}
	return ParseTableFile(text, languageCode);
}

Table BuildTable(const std::vector<Entry> &entries, const char *sentenceGap, const char *colon,
	PluralRule plural)
{
	Table table;
	table.sentenceGap = sentenceGap;
	table.colon = colon;
	table.plural = plural;
	for (size_t i = 0; i < entries.size(); ++i)
	{
		const std::string &key = entries[i].english;
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
		static const Table japanese = BuildTable(JapaneseTable().entries, "", "\xEF\xBC\x9A", SingleForm);
		return &japanese;
	}
	case Language::Italian:
	{
		static const Table italian = BuildTable(ItalianTable().entries, " ", ": ", OneOrOther);
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

// `complete` turns false when any part of the text stays English: Translate
// still answers with what it could translate, as the UI wants, and
// HasTranslation asks for all of it.
std::optional<std::string> Translate(const Table &table, const std::string &text, int depth,
	bool &complete);

// A captured value: numbers and names stay as they are, text that has its
// own entry ("3 min ago") is translated.
std::string TranslateCapture(const Table &table, const std::string &value, int depth)
{
	if (!HasLetters(value) || depth > 3)
		return value;
	bool complete = true;
	auto translated = Translate(table, value, depth + 1, complete);
	return translated ? *translated : value;
}

// The form `{N|one|other...}` picks for the count captured as value N. A
// value that is not a count takes the last form, as "other" does.
std::string PluralForm(const Table &table, std::string_view forms, const std::string &value)
{
	std::vector<std::string_view> options;
	for (size_t start = 0;;)
	{
		const size_t bar = forms.find('|', start);
		options.push_back(forms.substr(start, bar == std::string_view::npos ? std::string_view::npos : bar - start));
		if (bar == std::string_view::npos)
			break;
		start = bar + 1;
	}
	size_t digits = 0;
	while (digits < value.size() && value[digits] >= '0' && value[digits] <= '9')
		++digits;
	const size_t form = digits == 0 || digits != value.size() || digits > 18
		? options.size() - 1
		: (std::min)(table.plural(std::stoull(value)), options.size() - 1);
	return std::string(options[form]);
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
			if (t[i] == '{' && i + 2 < t.size() && t[i + 1] >= '0' && t[i + 1] <= '9')
			{
				const size_t group = static_cast<size_t>(t[i + 1] - '0') + 1;
				if (t[i + 2] == '}')
				{
					out += TranslateCapture(table, m[group].str(), depth);
					i += 2;
					continue;
				}
				const size_t close = t.find('}', i);
				if (t[i + 2] == '|' && close != std::string::npos)
				{
					out += PluralForm(table, std::string_view(t).substr(i + 3, close - i - 3), m[group].str());
					i = close;
					continue;
				}
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

std::optional<std::string> TranslateSentence(const Table &table, const std::string &text, int depth,
	bool &complete)
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
		auto head = Translate(table, text.substr(0, colon), depth + 1, complete);
		auto tail = Translate(table, text.substr(colon + 2), depth + 1, complete);
		if (head || tail)
		{
			if (!head || !tail)
				complete = false;
			return (head ? *head : text.substr(0, colon)) + table.colon +
				(tail ? *tail : text.substr(colon + 2));
		}
	}
	return std::nullopt;
}

std::optional<std::string> Translate(const Table &table, const std::string &text, int depth,
	bool &complete)
{
	// Trailing newlines and spaces are formatting, not wording.
	size_t end = text.size();
	while (end > 0 && (text[end - 1] == '\n' || text[end - 1] == '\r' || text[end - 1] == ' '))
		--end;
	const std::string core = text.substr(0, end);
	const std::string tail = text.substr(end);
	if (core.empty())
		return std::nullopt;

	if (auto whole = TranslateSentence(table, core, depth, complete))
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
		auto translated = TranslateSentence(table, piece, depth, complete);
		if (!translated)
			complete = false;
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

TableFile ParseTableFile(std::string_view text, std::string_view languageCode)
{
	TableFile file;
	const std::string translationPrefix = std::string(languageCode) + ": ";
	auto problem = [&file](size_t line, const char *why)
	{
		file.problems.push_back("line " + std::to_string(line) + ": " + why);
	};
	if (StartsWith(text, "\xEF\xBB\xBF"))
		text.remove_prefix(3);   // a byte order mark is not part of the first line

	std::string key;
	size_t keyLine = 0;   // 0: no key waiting for its translation
	size_t lineNumber = 0;
	size_t start = 0;
	while (start <= text.size())
	{
		size_t end = text.find('\n', start);
		if (end == std::string_view::npos)
			end = text.size();
		std::string_view line = text.substr(start, end - start);
		start = end + 1;
		++lineNumber;
		if (!line.empty() && line.back() == '\r')
			line.remove_suffix(1);
		if (line.empty() || line.front() == '#')
			continue;
		if (StartsWith(line, "name: "))
			file.nativeName = Unescape(line.substr(6));
		else if (StartsWith(line, "en: "))
		{
			if (keyLine != 0)
				problem(keyLine, "a key with no translation");
			key = Unescape(line.substr(4));
			keyLine = lineNumber;
		}
		else if (StartsWith(line, translationPrefix))
		{
			if (keyLine == 0)
			{
				problem(lineNumber, "a translation with no key before it");
				continue;
			}
			file.entries.push_back({ std::move(key), Unescape(line.substr(translationPrefix.size())) });
			key.clear();
			keyLine = 0;
		}
		else
			problem(lineNumber, "not a key, a translation, a name or a comment");
	}
	if (keyLine != 0)
		problem(keyLine, "a key with no translation");
	return file;
}

const TableFile &JapaneseTable()
{
	static const TableFile table = LoadBuiltInTable(L"TRANSLATIONS_JA", "ja");
	return table;
}

const TableFile &ItalianTable()
{
	static const TableFile table = LoadBuiltInTable(L"TRANSLATIONS_IT", "it");
	return table;
}

#ifdef QUESTCAL_LOCALIZATION_TEST_SEAM
std::optional<std::string> TranslateWithEntriesForTest(const std::vector<Entry> &entries,
	Language language, const std::string &english)
{
	const Table table = language == Language::Japanese
		? BuildTable(entries, "", "\xEF\xBC\x9A", SingleForm)
		: BuildTable(entries, " ", ": ", OneOrOther);
	bool complete = true;
	return Translate(table, english, 0, complete);
}
#endif

bool HasTranslation(Language language, const std::string &english)
{
	const Table *table = TableFor(language);
	if (!table)
		return true;   // English is the source
	bool complete = true;
	return Translate(*table, english, 0, complete).has_value() && complete;
}

// LanguageCodes lists one code per Language, at its position.
static_assert(LanguageCodeCount == kLanguageCount, "one saved code for each language");

Language LanguageFromCode(const std::string &code)
{
	const int index = LanguageCodeIndex(code);
	return index < 0 ? SystemLanguage() : static_cast<Language>(index);
}

const char *LanguageCode(Language language)
{
	const int index = static_cast<int>(language);
	return LanguageCodes[index >= 0 && index < LanguageCodeCount ? index : 0];
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
	if (!table || !*english)
		return english;
	auto cached = g_cache.find(english);
	if (cached != g_cache.end())
		return cached->second.c_str();

	std::string key(english);
	bool complete = true;
	auto translated = Translate(*table, key, 0, complete);
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
