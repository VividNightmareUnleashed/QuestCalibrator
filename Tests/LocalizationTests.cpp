// Display-time translation: the table's own consistency, and the lookup
// order Localization.h promises (exact, pattern, sentences) on the shapes the
// overlay actually builds. Expected text is assembled from the table itself,
// so these hold for any wording a translator picks.
#include "../Overlay/Localization.h"
#include "../Overlay/LocalizationTables.h"

#include <cstring>
#include <set>
#include <string>

namespace
{
using Check = void (*)(const char *, bool, const char *);
using namespace questcal::i18n;

const char *Lookup(const Entry *table, size_t count, const char *english)
{
	for (size_t i = 0; i < count; ++i)
		if (std::strcmp(table[i].english, english) == 0)
			return table[i].translation;
	return nullptr;
}

const char *Japanese(const char *english)
{
	return Lookup(kJapanese, kJapaneseCount, english);
}

const char *Italian(const char *english)
{
	return Lookup(kItalian, kItalianCount, english);
}

// Conversions in a key, "%%" not counted: the number of values a pattern
// captures.
int Conversions(const char *key)
{
	int n = 0;
	for (const char *p = key; *p; ++p)
	{
		if (*p != '%')
			continue;
		if (p[1] == '%')
		{
			++p;
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

std::set<std::string> Keys(const Entry *table, size_t count)
{
	std::set<std::string> keys;
	for (size_t i = 0; i < count; ++i)
		keys.insert(table[i].english);
	return keys;
}

void TableIsConsistent(Check check, const char *language, const Entry *table, size_t count)
{
	std::set<std::string> keys;
	bool unique = true, placeholders = true, nonEmpty = true;
	for (size_t i = 0; i < count; ++i)
	{
		const Entry &e = table[i];
		unique = keys.insert(e.english).second && unique;
		nonEmpty = nonEmpty && *e.english && *e.translation;
		const int captured = Conversions(e.english);
		for (const char *p = e.translation; *p; ++p)
			if (p[0] == '{' && p[1] >= '0' && p[1] <= '9' && p[2] == '}' && p[1] - '0' >= captured)
				placeholders = false;
	}
	const std::string prefix = std::string("i18n ") + language;
	check((prefix + " keys unique").c_str(), unique, "every English key appears once");
	check((prefix + " non-empty").c_str(), nonEmpty, "no blank key or translation");
	check((prefix + " placeholders").c_str(), placeholders, "every {n} names a value its key captures");
}

// Every language translates the same strings: a key added to one table and
// forgotten in another would show that language's player English.
void TablesCoverTheSameText(Check check)
{
	check("i18n tables match",
		Keys(kJapanese, kJapaneseCount) == Keys(kItalian, kItalianCount),
		"Japanese and Italian have the same keys");
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
	TableIsConsistent(check, "ja", kJapanese, kJapaneseCount);
	TableIsConsistent(check, "it", kItalian, kItalianCount);
	TablesCoverTheSameText(check);
	EnglishPassesThrough(check);
	JapaneseLookups(check);
	ItalianLookups(check);
}
