#pragma once

// Display-time translation of the overlay's player-facing text.
//
// English is the source language and the key: every message is built, stored
// and logged in English, and only the UI turns it into the chosen language as
// it draws it. That keeps the session log and diagnostics readable for bug
// reports, and a language switch retranslates everything already on screen,
// messages included.
//
// A lookup tries, in order:
//  1. the exact English string;
//  2. a pattern whose key holds printf conversions ("%s isn't tracking."),
//     so text formatted at run time still finds its entry. The translation
//     refers to the captured values as {0}, {1}... in any order, and a
//     captured value that is itself translatable text is translated too;
//  3. the string split into sentences (and "A: B" halves), each translated
//     on its own, for messages that join several sentences.
// Anything still unmatched is shown in English.

#include <string>

namespace questcal::i18n
{
	enum class Language { English, Japanese };

	// Persisted codes: "en", "ja". Empty or unknown means "follow Windows".
	Language LanguageFromCode(const std::string &code);
	const char *LanguageCode(Language language);

	// The language Windows' display language asks for, when a translation
	// and a font to draw it exist; English otherwise.
	Language SystemLanguage();

	// Takes effect at the next BeginFrame: pointers handed out this frame
	// stay valid until then.
	void SetLanguage(Language language);
	Language CurrentLanguage();

	// Once per UI frame, before any Tr call. Applies a pending language
	// change and bounds the cache behind Tr's pointers.
	void BeginFrame();

	// Whether a font able to draw the language was found at startup.
	void SetFontAvailable(Language language, bool available);
	bool FontAvailable(Language language);

	// The text in the current language. The pointer stays valid until the
	// next BeginFrame; in English it is the argument itself.
	const char *Tr(const char *english);
	std::string Tr(const std::string &english);

	// Debug aid for translators: English strings that reached Tr without a
	// translation this session, one per line.
	bool WriteMissing(const std::wstring &path);
}
