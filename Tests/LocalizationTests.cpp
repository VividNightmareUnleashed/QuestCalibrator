// Display-time translation: the table's own consistency, and the lookup
// order Localization.h promises (exact, pattern, sentences) on the shapes the
// overlay actually builds. Expected text is assembled from the table itself,
// so these hold for any wording a translator picks.
#include "../Overlay/Localization.h"
#include "../Overlay/LocalizationTables.h"

#include <windows.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace
{
using Check = void (*)(const char *, bool, const char *);
using namespace questcal::i18n;

const char *Lookup(const TableFile &table, const char *english)
{
	for (const Entry &entry : table.entries)
		if (entry.english == english)
			return entry.translation.c_str();
	return nullptr;
}

const char *Japanese(const char *english)
{
	return Lookup(JapaneseTable(), english);
}

const char *Italian(const char *english)
{
	return Lookup(ItalianTable(), english);
}

// Conversions in a key, "%%" not counted: the number of values a pattern
// captures.
int Conversions(const std::string &key)
{
	int n = 0;
	for (size_t i = 0; i < key.size(); ++i)
	{
		if (key[i] != '%')
			continue;
		if (i + 1 < key.size() && key[i + 1] == '%')
		{
			++i;
			continue;
		}
		++n;
	}
	return n;
}

std::string Fill(const char *translation, const std::string &a, const std::string &b = "")
{
	std::string out = translation;
	for (const auto &[slot, value] : { std::pair<std::string, std::string>{ "{0}", a }, { "{1}", b } })
	{
		const size_t at = out.find(slot);
		if (at != std::string::npos)
			out.replace(at, slot.size(), value);
	}
	return out;
}

std::set<std::string> Keys(const TableFile &table)
{
	std::set<std::string> keys;
	for (const Entry &entry : table.entries)
		keys.insert(entry.english);
	return keys;
}

void TableIsConsistent(Check check, const char *language, const TableFile &table)
{
	std::set<std::string> keys;
	bool unique = true, placeholders = true, nonEmpty = true;
	for (const Entry &e : table.entries)
	{
		unique = keys.insert(e.english).second && unique;
		nonEmpty = nonEmpty && !e.english.empty() && !e.translation.empty();
		const int captured = Conversions(e.english);
		const std::string &t = e.translation;
		for (size_t i = 0; i + 2 < t.size(); ++i)
			if (t[i] == '{' && t[i + 1] >= '0' && t[i + 1] <= '9' &&
				(t[i + 2] == '}' || t[i + 2] == '|') && t[i + 1] - '0' >= captured)
				placeholders = false;
	}
	const std::string prefix = std::string("i18n ") + language;
	check((prefix + " file reads cleanly").c_str(),
		table.problems.empty() && !table.entries.empty() && !table.nativeName.empty(),
		table.problems.empty() ? "every line is a key, a translation, a name or a comment"
			: table.problems.front().c_str());
	check((prefix + " keys unique").c_str(), unique, "every English key appears once");
	check((prefix + " non-empty").c_str(), nonEmpty, "no blank key or translation");
	check((prefix + " placeholders").c_str(), placeholders, "every {n} names a value its key captures");
}

// Every language translates the same strings: a key added to one table and
// forgotten in another would show that language's player English.
void TablesCoverTheSameText(Check check)
{
	check("i18n tables match", Keys(JapaneseTable()) == Keys(ItalianTable()),
		"Japanese and Italian have the same keys");
}

// The file format, on a sample with every kind of line and the mistakes a
// hand edit makes.
void TableFileFormat(Check check)
{
	const TableFile good = ParseTableFile(
		"\xEF\xBB\xBF# A comment\r\n"
		"name: Sample\r\n"
		"\r\n"
		"en: Two\\nlines and a \\\\ backslash\r\n"
		"xx: Due\\nrighe\r\n"
		"en: Read from %s\n"
		"xx: Letto da {0}\n", "xx");
	const bool read = good.problems.empty() && good.nativeName == "Sample" &&
		good.entries.size() == 2 &&
		good.entries[0].english == "Two\nlines and a \\ backslash" &&
		good.entries[0].translation == "Due\nrighe" &&
		good.entries[1].english == "Read from %s" && good.entries[1].translation == "Letto da {0}";
	const TableFile bad = ParseTableFile(
		"en: No translation\n"
		"en: Second key\n"
		"yy: Wrong language\n"
		"xx: Seconda\n"
		"xx: Orphan\n"
		"en: Last key\n", "xx");
	const bool reported = bad.entries.size() == 1 && bad.entries[0].english == "Second key" &&
		bad.problems.size() == 4;
	check("i18n file format", read && reported,
		read ? (reported ? "" : "a mistake went unreported") : "a well-formed sample was misread");
}

// A translation picks its plural form by the captured count and the
// language's rule; the English keeps its own suffix, which the translation
// need not use.
void PluralForms(Check check)
{
	const std::vector<Entry> italian = { { "Dropped out %u time%s", "Interrotto {0} {0|volta|volte}" } };
	const std::vector<Entry> japanese = { { "Dropped out %u time%s", "{0} {0|回}中断" } };
	auto in = [](const std::vector<Entry> &entries, Language language, const char *english)
	{
		const auto translated = TranslateWithEntriesForTest(entries, language, english);
		return translated ? *translated : std::string();
	};
	const bool italianForms =
		in(italian, Language::Italian, "Dropped out 1 time") == "Interrotto 1 volta" &&
		in(italian, Language::Italian, "Dropped out 3 times") == "Interrotto 3 volte" &&
		in(italian, Language::Italian, "Dropped out 0 times") == "Interrotto 0 volte";
	const bool japaneseForm = in(japanese, Language::Japanese, "Dropped out 3 times") == "3 回中断";
	check("i18n plural forms", italianForms && japaneseForm,
		italianForms ? (japaneseForm ? "" : "the single Japanese form was not used") :
			"Italian picked the wrong form");
}

// ---- Every literal the overlay shows has a translation ----

// What reaches the player, by call and the arguments that are player text:
// Tr itself, the context's messages and toasts, and the shared widgets that
// translate their labels (UiInternal.h). Log and detail lines stay English.
struct ShownCall
{
	const char *name;
	bool member;   // reached through an object, as ctx.Tell
	std::vector<size_t> positions;
};

const std::vector<ShownCall> &ShownCalls()
{
	static const std::vector<ShownCall> calls = {
		{ "Tr", false, { 0 } },
		{ "Tell", true, { 0 } },
		{ "ReportError", true, { 0 } },
		{ "Instruct", true, { 0 } },
		{ "Note", true, { 0 } },
		{ "Outcome", true, { 0, 1, 2 } },
		{ "NotifyOnce", false, { 2, 4 } },
		{ "LinkText", false, { 0 } },
		{ "SectionLabel", false, { 0 } },
		{ "IconButton", false, { 1 } },
		{ "ButtonWidthFor", false, { 0 } },
		{ "RowIconLabel", false, { 2 } },
		{ "RowSubLine", false, { 1 } },
		{ "ToggleRow", false, { 2, 4 } },
		{ "ShowTip", false, { 0 } },
		{ "NestedToggle", false, { 3, 5 } },
	};
	return calls;
}

// Comments out, string and character literals untouched.
std::string StripComments(const std::string &text)
{
	std::string out;
	out.reserve(text.size());
	for (size_t i = 0; i < text.size();)
	{
		const char c = text[i];
		if (c == '"' || c == '\'')
		{
			size_t j = i + 1;
			while (j < text.size() && text[j] != c)
				j += text[j] == '\\' ? 2 : 1;
			out.append(text, i, (std::min)(j + 1, text.size()) - i);
			i = j + 1;
		}
		else if (text.compare(i, 2, "//") == 0)
			i = (std::min)(text.find('\n', i), text.size());
		else if (text.compare(i, 2, "/*") == 0)
		{
			const size_t end = text.find("*/", i + 2);
			i = end == std::string::npos ? text.size() : end + 2;
		}
		else
			out += text[i++];
	}
	return out;
}

// The top-level arguments of the call whose '(' is at `open`.
std::vector<std::string> ArgumentsOf(const std::string &text, size_t open)
{
	std::vector<std::string> args;
	int depth = 0;
	size_t start = open + 1;
	for (size_t i = open; i < text.size(); ++i)
	{
		const char c = text[i];
		if (c == '"' || c == '\'')
		{
			size_t j = i + 1;
			while (j < text.size() && text[j] != c)
				j += text[j] == '\\' ? 2 : 1;
			i = j;
		}
		else if (c == '(' || c == '[' || c == '{')
			++depth;
		else if (c == ')' || c == ']' || c == '}')
		{
			if (--depth == 0)
			{
				args.push_back(text.substr(start, i - start));
				return args;
			}
		}
		else if (c == ',' && depth == 1)
		{
			args.push_back(text.substr(start, i - start));
			start = i + 1;
		}
	}
	return args;
}

// An argument that is nothing but string literals, joined and unescaped.
bool LiteralValue(const std::string &arg, std::string &value)
{
	value.clear();
	size_t i = 0;
	bool any = false;
	while (true)
	{
		while (i < arg.size() && isspace(static_cast<unsigned char>(arg[i])))
			++i;
		if (i == arg.size())
			return any;
		if (arg[i] != '"')
			return false;
		for (++i; i < arg.size() && arg[i] != '"'; ++i)
		{
			if (arg[i] != '\\' || i + 1 >= arg.size())
			{
				value += arg[i];
				continue;
			}
			const char e = arg[++i];
			if (e == 'n') value += '\n';
			else if (e == 't') value += '\t';
			else if (e == 'x' && i + 2 < arg.size())
			{
				value += static_cast<char>(std::stoi(arg.substr(i + 1, 2), nullptr, 16));
				i += 2;
			}
			else value += e;   // \" \\ \'
		}
		++i;
		any = true;
	}
}

bool IsIdentifierChar(char c)
{
	return isalnum(static_cast<unsigned char>(c)) || c == '_';
}

// The Overlay sources, found from the harness binary (x64/<config>/).
std::filesystem::path OverlaySources()
{
	wchar_t exe[MAX_PATH] = {};
	GetModuleFileNameW(nullptr, exe, MAX_PATH);
	std::filesystem::path dir = std::filesystem::path(exe).parent_path();
	for (int up = 0; up < 5 && !dir.empty(); ++up, dir = dir.parent_path())
		if (std::filesystem::exists(dir / "Overlay" / "Localization.h"))
			return dir / "Overlay";
	return {};
}

// A literal a reworded or new string left without an entry would show its
// English in every other language; this finds it from the sources, by the
// lookup Tr uses. A literal followed by more text ("Saved to " + path) is
// only the start of a message, and its pattern is checked where it is built.
void EveryShownLiteralIsTranslated(Check check)
{
	const std::filesystem::path overlay = OverlaySources();
	if (overlay.empty())
	{
		check("i18n every shown literal is translated", false, "the Overlay sources were not found");
		return;
	}
	size_t literals = 0;
	std::vector<std::string> missing;
	for (const auto &item : std::filesystem::directory_iterator(overlay))
	{
		const std::filesystem::path &path = item.path();
		const std::string name = path.filename().string();
		if ((path.extension() != ".cpp" && path.extension() != ".h") || name.rfind("Localization", 0) == 0)
			continue;
		std::ifstream file(path, std::ios::binary);
		std::stringstream buffer;
		buffer << file.rdbuf();
		const std::string text = StripComments(buffer.str());
		for (const ShownCall &call : ShownCalls())
		{
			const std::string token = std::string(call.name) + "(";
			for (size_t at = text.find(token); at != std::string::npos; at = text.find(token, at + 1))
			{
				const char before = at > 0 ? text[at - 1] : ' ';
				if (call.member ? (before != '.' && before != '>') : IsIdentifierChar(before))
					continue;
				const std::vector<std::string> args = ArgumentsOf(text, at + token.size() - 1);
				for (size_t position : call.positions)
				{
					std::string value;
					if (position >= args.size() || !LiteralValue(args[position], value))
						continue;
					bool letters = false;
					for (unsigned char c : value)
						letters = letters || isalpha(c);
					if (!letters)
						continue;
					++literals;
					if (!HasTranslation(Language::Japanese, value) || !HasTranslation(Language::Italian, value))
						missing.push_back(name + ": " + value);
				}
			}
		}
	}
	std::string detail = std::to_string(literals) + " literals";
	if (!missing.empty())
		detail += ", untranslated: " + missing.front() +
			(missing.size() > 1 ? " (+" + std::to_string(missing.size() - 1) + " more)" : "");
	check("i18n every shown literal is translated", literals > 100 && missing.empty(), detail.c_str());
}

void EnglishPassesThrough(Check check)
{
	SetLanguage(Language::English);
	BeginFrame();
	const char *text = "Start calibration";
	check("i18n english identity", Tr(text) == text, "English returns the argument itself");
	check("i18n english string", Tr(std::string("Recalibrate.\n")) == "Recalibrate.\n",
		"English leaves formatting alone");
}

void JapaneseLookups(Check check)
{
	SetLanguage(Language::Japanese);
	BeginFrame();

	check("i18n exact", std::string(Tr("Start calibration")) == Japanese("Start calibration"),
		"an exact key");

	// A pattern, with a device name that has no entry of its own.
	check("i18n pattern capture",
		Tr(std::string("VIVE Tracker 3.0 isn't tracking.")) ==
			Fill(Japanese("%s isn't tracking."), "VIVE Tracker 3.0"),
		"a formatted message keeps its value");

	// A captured value that is itself text gets translated.
	check("i18n nested capture",
		Tr(std::string("last adjusted 3 min ago")) ==
			Fill(Japanese("last adjusted %s"), Fill(Japanese("%d min ago"), "3")),
		"an age inside a sentence is translated too");

	// The activity feed joins headline and body with ": ".
	check("i18n headline and body",
		Tr(std::string("Calibration failed: The devices didn't rotate far enough.")) ==
			std::string(Japanese("Calibration failed")) + "\xEF\xBC\x9A" +
				Japanese("The devices didn't rotate far enough."),
		"each half of an activity line is translated");

	// An outcome body built from several sentences, with the trailing
	// newline the log kept.
	check("i18n sentences",
		Tr(std::string("Check that the tracker positions line up in VR. Headset tracker set up. "
			"Turn on continuous calibration in Settings to use it.\n")) ==
			std::string(Japanese("Check that the tracker positions line up in VR.")) +
				Japanese("Headset tracker set up.") +
				Japanese("Turn on continuous calibration in Settings to use it.") + "\n",
		"joined sentences are translated one by one");

	// The most specific pattern wins over one whose %s would swallow the rest.
	check("i18n specific pattern first",
		Tr(std::string("Drift: stale; 2 slips up to 1.5 cm while standing still")).find("1.5") != std::string::npos &&
			Tr(std::string("Drift: stale; 2 slips up to 1.5 cm while standing still")).find("slips") == std::string::npos,
		"the drift line with evidence matches its own pattern");

	// A sentence with no entry stays English and keeps its space.
	check("i18n partial",
		Tr(std::string("Chaperone restored. Nothing else to say.")) ==
			std::string(Japanese("Chaperone restored.")) + " Nothing else to say.",
		"an untranslated sentence is left readable");

	check("i18n unknown", std::string(Tr("No entry for this at all")) == "No entry for this at all",
		"text with no entry falls back to English");

	SetLanguage(Language::English);
	BeginFrame();
}

// Italian joins what Japanese runs together: sentences keep their space
// and "headline: body" keeps a plain colon.
void ItalianLookups(Check check)
{
	SetLanguage(Language::Italian);
	BeginFrame();

	check("i18n it exact", std::string(Tr("Start calibration")) == Italian("Start calibration"),
		"an exact key");
	check("i18n it headline and body",
		Tr(std::string("Calibration failed: The devices didn't rotate far enough.")) ==
			std::string(Italian("Calibration failed")) + ": " +
				Italian("The devices didn't rotate far enough."),
		"an activity line keeps a plain colon");
	check("i18n it sentences",
		Tr(std::string("Check that the tracker positions line up in VR. Headset tracker set up.")) ==
			std::string(Italian("Check that the tracker positions line up in VR.")) + " " +
				Italian("Headset tracker set up."),
		"joined sentences keep their space");
	check("i18n it code", LanguageFromCode("it") == Language::Italian &&
		std::string(LanguageCode(Language::Italian)) == "it", "the saved code round-trips");

	SetLanguage(Language::English);
	BeginFrame();
}

} // namespace

void RunLocalizationScenarios(Check check)
{
	TableIsConsistent(check, "ja", JapaneseTable());
	TableIsConsistent(check, "it", ItalianTable());
	TablesCoverTheSameText(check);
	TableFileFormat(check);
	PluralForms(check);
	EveryShownLiteralIsTranslated(check);
	EnglishPassesThrough(check);
	JapaneseLookups(check);
	ItalianLookups(check);
}
