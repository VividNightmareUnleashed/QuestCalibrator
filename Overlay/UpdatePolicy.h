#pragma once

#include "../common/Version.h"
#include "../lib/picojson.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace questcal
{
namespace update
{

// An empty prerelease label means a final release. The label and ordinal are
// the shape QuestCalibrator actually publishes ("alpha", 3), not the whole of
// SemVer's dot-separated identifier list.
struct Version
{
	uint32_t major = 0;
	uint32_t minor = 0;
	uint32_t patch = 0;
	std::string prereleaseLabel;
	uint32_t prereleaseOrdinal = 0;
};

inline bool IsPrerelease(const Version &version)
{
	return !version.prereleaseLabel.empty();
}

// The build's own identity, assembled in exactly one place. Filling a Version
// field by field at the call site is what let 1.2.0-alpha.3 report itself as
// 1.2.0: a field nobody remembered to set defaults to "final", and the tester
// then compares equal to the release they are waiting for.
inline Version CurrentVersion()
{
	Version version;
	version.major = QUESTCAL_VERSION_MAJOR;
	version.minor = QUESTCAL_VERSION_MINOR;
	version.patch = QUESTCAL_VERSION_PATCH;
	version.prereleaseLabel = QUESTCAL_VERSION_PRERELEASE_LABEL;
	version.prereleaseOrdinal = QUESTCAL_VERSION_PRERELEASE_ORDINAL;
	return version;
}

// SemVer 2.0.0 section 11: when the three numbers agree, a final release
// outranks every prerelease of it, and two prereleases of it are ordered by
// label then ordinal. Labels compare as text, which puts alpha below beta
// below rc.
//
// The update path never reaches these last four lines, because it decides the
// lane before it compares anything and a feed candidate is always final. They
// are here because a comparison that quietly ignored half the fields of the
// thing it compares is what made 1.2.0-alpha.3 equal to 1.2.0 in the first
// place, and whoever builds the prerelease lane will need the order to hold.
inline int CompareVersions(const Version &a, const Version &b)
{
	if (a.major != b.major) return a.major < b.major ? -1 : 1;
	if (a.minor != b.minor) return a.minor < b.minor ? -1 : 1;
	if (a.patch != b.patch) return a.patch < b.patch ? -1 : 1;
	if (IsPrerelease(a) != IsPrerelease(b)) return IsPrerelease(a) ? -1 : 1;
	if (!IsPrerelease(a)) return 0;
	if (a.prereleaseLabel != b.prereleaseLabel)
		return a.prereleaseLabel < b.prereleaseLabel ? -1 : 1;
	if (a.prereleaseOrdinal != b.prereleaseOrdinal)
		return a.prereleaseOrdinal < b.prereleaseOrdinal ? -1 : 1;
	return 0;
}

inline std::string VersionString(const Version &version)
{
	char text[48];
	snprintf(text, sizeof text, "%u.%u.%u",
		version.major, version.minor, version.patch);
	std::string rendered = text;
	if (IsPrerelease(version))
	{
		snprintf(text, sizeof text, ".%u", version.prereleaseOrdinal);
		rendered += "-" + version.prereleaseLabel + text;
	}
	return rendered;
}

inline bool ParseVersionComponent(const std::string &text, size_t &cursor,
	uint32_t &component, bool final)
{
	if (cursor >= text.size() || text[cursor] < '0' || text[cursor] > '9')
		return false;

	uint32_t value = 0;
	const size_t first = cursor;
	while (cursor < text.size() && text[cursor] >= '0' && text[cursor] <= '9')
	{
		const uint32_t digit = static_cast<uint32_t>(text[cursor] - '0');
		if (value > (std::numeric_limits<uint32_t>::max() - digit) / 10)
			return false;
		value = value * 10 + digit;
		++cursor;
	}
	if (cursor - first > 1 && text[first] == '0')
		return false;
	if (final)
	{
		if (cursor != text.size())
			return false;
	}
	else if (cursor >= text.size() || text[cursor++] != '.')
	{
		return false;
	}
	component = value;
	return true;
}

// Only the fork's stable release namespace is eligible. This deliberately
// rejects inherited v* tags and prerelease suffixes; alpha/beta delivery can be
// added later as a separate, explicit channel without changing stable users.
// Every version it yields is therefore a final release, which is what lets a
// prerelease running locally compare below the stable release it matches.
inline bool ParseReleaseTag(const std::string &tag, Version &version)
{
	static const std::string prefix = "questcalibrator-v";
	if (tag.compare(0, prefix.size(), prefix) != 0)
		return false;

	Version parsed;
	size_t cursor = prefix.size();
	if (!ParseVersionComponent(tag, cursor, parsed.major, false) ||
		!ParseVersionComponent(tag, cursor, parsed.minor, false) ||
		!ParseVersionComponent(tag, cursor, parsed.patch, true))
		return false;
	version = parsed;
	return true;
}

// Only ever called for a candidate out of the feed, which ParseReleaseTag
// guarantees is a final release, so this never has to name a prerelease asset.
inline std::string CanonicalPackageName(const Version &version)
{
	return "QuestCalibrator-" + VersionString(version) + ".zip";
}

inline int HexNibble(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

inline bool ParseSha256Digest(const std::string &digest,
	std::array<unsigned char, 32> &bytes)
{
	static const std::string prefix = "sha256:";
	if (digest.size() != prefix.size() + bytes.size() * 2 ||
		digest.compare(0, prefix.size(), prefix) != 0)
		return false;
	for (size_t i = 0; i < bytes.size(); ++i)
	{
		const int high = HexNibble(digest[prefix.size() + i * 2]);
		const int low = HexNibble(digest[prefix.size() + i * 2 + 1]);
		if (high < 0 || low < 0)
			return false;
		bytes[i] = static_cast<unsigned char>((high << 4) | low);
	}
	return true;
}

struct ReleaseCandidate
{
	Version version;
	std::string tag;
	std::string releaseUrl;
	std::string packageName;
	std::string downloadUrl;
	std::string digest;
	uint64_t size = 0;
};

template<typename T>
inline bool JsonField(const picojson::object &object, const char *key, T &out)
{
	auto it = object.find(key);
	if (it == object.end() || !it->second.is<T>())
		return false;
	out = it->second.get<T>();
	return true;
}

// The stable lane, and the only lane there is: the overlay reads published
// stable releases and nothing else.
//
// The feed is untrusted input. Select the newest published stable release first,
// then require its one canonical package to be complete and internally
// consistent. Never fall back to an older package when the newest release is
// malformed: that would hide a broken or partially published release.
inline bool SelectReleaseCandidate(const std::string &json,
	const Version &current, ReleaseCandidate &candidate, bool &updateAvailable,
	std::string &error)
{
	updateAvailable = false;
	error.clear();
	// A prerelease is on its own lane. It is installed by hand and leaves by
	// hand, so the stable feed never has anything to say to it, whatever the
	// feed holds. The updater stops before it gets here; this is the second
	// lock, so no future caller can hand a tester a stable package by accident.
	if (IsPrerelease(current))
		return true;
	picojson::value root;
	std::string parseError;
	try
	{
		parseError = picojson::parse(root, json);
	}
	catch (const std::overflow_error &)
	{
		// picojson throws, rather than reports, a number past a double's range.
		parseError = "number out of range";
	}
	if (!parseError.empty() || !root.is<picojson::array>())
	{
		error = "GitHub returned an invalid release list.";
		return false;
	}

	const picojson::object *best = nullptr;
	Version bestVersion;
	std::string bestTag;
	for (const auto &entry : root.get<picojson::array>())
	{
		if (!entry.is<picojson::object>())
			continue;
		const auto &release = entry.get<picojson::object>();
		bool draft = true, prerelease = true;
		std::string tag;
		Version version;
		if (!JsonField(release, "draft", draft) ||
			!JsonField(release, "prerelease", prerelease) ||
			!JsonField(release, "tag_name", tag) || draft || prerelease ||
			!ParseReleaseTag(tag, version))
			continue;
		if (!best || CompareVersions(version, bestVersion) > 0)
		{
			best = &release;
			bestVersion = version;
			bestTag = tag;
		}
	}

	if (!best || CompareVersions(bestVersion, current) <= 0)
		return true;
	updateAvailable = true;

	ReleaseCandidate parsed;
	parsed.version = bestVersion;
	parsed.tag = bestTag;
	if (!JsonField(*best, "html_url", parsed.releaseUrl))
	{
		error = "The newest release has no trusted release page.";
		return false;
	}
	const std::string expectedReleaseUrl =
		"https://github.com/VividNightmareUnleashed/QuestCalibrator/releases/tag/" + bestTag;
	if (parsed.releaseUrl != expectedReleaseUrl)
	{
		error = "The newest release points outside the QuestCalibrator repository.";
		return false;
	}

	picojson::array assets;
	if (!JsonField(*best, "assets", assets))
	{
		error = "The newest release has no install package.";
		return false;
	}
	parsed.packageName = CanonicalPackageName(bestVersion);
	const picojson::object *package = nullptr;
	for (const auto &assetValue : assets)
	{
		if (!assetValue.is<picojson::object>())
			continue;
		const auto &asset = assetValue.get<picojson::object>();
		std::string name;
		if (JsonField(asset, "name", name) && name == parsed.packageName)
		{
			if (package)
			{
				error = "The newest release contains duplicate install packages.";
				return false;
			}
			package = &asset;
		}
	}
	if (!package)
	{
		error = "The newest release does not contain " + parsed.packageName + ".";
		return false;
	}

	double size = 0.0;
	if (!JsonField(*package, "browser_download_url", parsed.downloadUrl) ||
		!JsonField(*package, "digest", parsed.digest) ||
		!JsonField(*package, "size", size) || !std::isfinite(size) ||
		std::floor(size) != size || size < 128.0 * 1024.0 ||
		size > 64.0 * 1024.0 * 1024.0)
	{
		error = "The newest release package has invalid metadata.";
		return false;
	}
	std::array<unsigned char, 32> digestBytes;
	if (!ParseSha256Digest(parsed.digest, digestBytes))
	{
		error = "The newest release package has no valid SHA-256 digest.";
		return false;
	}
	const std::string expectedDownloadUrl =
		"https://github.com/VividNightmareUnleashed/QuestCalibrator/releases/download/" +
		bestTag + "/" + parsed.packageName;
	if (parsed.downloadUrl != expectedDownloadUrl)
	{
		error = "The newest release package points outside the QuestCalibrator repository.";
		return false;
	}
	parsed.size = static_cast<uint64_t>(size);
	candidate = std::move(parsed);
	return true;
}

} // namespace update
} // namespace questcal
