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

const char *Japanese(const char *english)
{
	for (size_t i = 0; i < kJapaneseCount; ++i)
		if (std::strcmp(kJapanese[i].english, english) == 0)
			return kJapanese[i].translation;
	return nullptr;
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

void TableIsConsistent(Check check)
{
	std::set<std::string> keys;
	bool unique = true, placeholders = true, nonEmpty = true;
	for (size_t i = 0; i < kJapaneseCount; ++i)
	{
		const Entry &e = kJapanese[i];
		unique = keys.insert(e.english).second && unique;
		nonEmpty = nonEmpty && *e.english && *e.translation;
		const int captured = Conversions(e.english);
		for (const char *p = e.translation; *p; ++p)
			if (p[0] == '{' && p[1] >= '0' && p[1] <= '9' && p[2] == '}' && p[1] - '0' >= captured)
				placeholders = false;
	}
	check("i18n keys unique", unique, "every English key appears once");
	check("i18n entries non-empty", nonEmpty, "no blank key or translation");
	check("i18n placeholders captured", placeholders, "every {n} names a value its key captures");
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

} // namespace

void RunLocalizationScenarios(Check check)
{
	TableIsConsistent(check);
	EnglishPassesThrough(check);
	JapaneseLookups(check);
}
