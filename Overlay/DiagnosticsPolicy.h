#pragma once
#include <algorithm>
#include <cctype>
#include <string>

// Explicit byte-wise substitutions. Unicode case equivalence and discovery
// of arbitrary secrets are outside this policy. Callers bound report inputs.
namespace questcal { namespace diagnostics {
// Case-insensitive replace of every occurrence: Windows paths arrive in
// whatever case the writer used. `needle` must not be empty.
inline void ReplaceAllNoCase(std::string &text, const std::string &needle, const std::string &with)
{
	if (needle.empty()) return;
	std::string lowerText = text, lowerNeedle = needle;
	std::transform(lowerText.begin(), lowerText.end(), lowerText.begin(),
		[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	std::transform(lowerNeedle.begin(), lowerNeedle.end(), lowerNeedle.begin(),
		[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	std::string out;
	size_t pos = 0;
	for (;;)
	{
		size_t hit = lowerText.find(lowerNeedle, pos);
		if (hit == std::string::npos)
		{
			out.append(text, pos, std::string::npos);
			break;
		}
		out.append(text, pos, hit - pos);
		out += with;
		pos = hit + needle.size();
	}
	text.swap(out);
}

inline std::string AnonymiseDiagnosticsText(const std::string &text,
	const std::string &userProfileDir, const std::string &userName, const std::string &computerName)
{
	std::string out = text;
	// Longest first, so the directory goes before the bare name inside it.
	if (!userProfileDir.empty())
		ReplaceAllNoCase(out, userProfileDir, "<user>");
	if (userName.size() >= 2)
		ReplaceAllNoCase(out, userName, "<user>");
	if (computerName.size() >= 2)
		ReplaceAllNoCase(out, computerName, "<pc>");
	return out;
}

inline std::string ShortenUserPath(const std::string &path,
	const std::string &localAppData, const std::string &userProfileDir)
{
	auto startsWithFolder = [&](const std::string &prefix)
	{
		if (prefix.size() < 3 || path.size() < prefix.size())
			return false;
		for (size_t i = 0; i < prefix.size(); ++i)
			if (std::tolower(static_cast<unsigned char>(path[i])) !=
				std::tolower(static_cast<unsigned char>(prefix[i])))
				return false;
		// A whole folder only: C:\Users\jo is not the start of C:\Users\joanna.
		return path.size() == prefix.size() || path[prefix.size()] == '\\' || path[prefix.size()] == '/';
	};
	// %LOCALAPPDATA% first: it sits inside the profile directory.
	if (startsWithFolder(localAppData))
		return "%LOCALAPPDATA%" + path.substr(localAppData.size());
	if (startsWithFolder(userProfileDir))
		return "%USERPROFILE%" + path.substr(userProfileDir.size());
	return path;
}

} }
