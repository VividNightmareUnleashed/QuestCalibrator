// PCH-free on purpose: SolverTests compiles this file standalone.
#include "LighthouseLog.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace lighthouselog
{

namespace
{

// "S-16" -> 16; anything else -> -1.
int ParseChannel(const std::string &token)
{
	if (token.size() < 3 || token[0] != 'S' || token[1] != '-')
		return -1;
	for (size_t i = 2; i < token.size(); ++i)
		if (!std::isdigit(static_cast<unsigned char>(token[i])))
			return -1;
	return std::atoi(token.c_str() + 2);
}

// "D3D4E73B" or " 4D47FB4" (a leading zero printed as a space) -> the id;
// 0 when the text is not hexadecimal.
uint32_t ParseStationId(std::string text)
{
	text.erase(std::remove(text.begin(), text.end(), ' '), text.end());
	if (text.empty() || text.size() > 8)
		return 0;
	for (char c : text)
		if (!std::isxdigit(static_cast<unsigned char>(c)))
			return 0;
	return static_cast<uint32_t>(std::strtoul(text.c_str(), nullptr, 16));
}

// Splits "S-5 (D3D4E73B) S-8 S-16 ( 4D47FB4)" into channel/id pairs. A
// parenthesised token that is not an id is skipped.
void ParseStationList(const std::string &text, std::vector<int> &channels,
	std::vector<uint32_t> &ids)
{
	size_t i = 0;
	while (i < text.size())
	{
		while (i < text.size() && text[i] == ' ')
			++i;
		if (i >= text.size())
			break;
		if (text[i] == '(')
		{
			size_t close = text.find(')', i);
			if (close == std::string::npos)
				break;
			uint32_t id = ParseStationId(text.substr(i + 1, close - i - 1));
			if (id != 0 && !ids.empty())
				ids.back() = id;
			i = close + 1;
			continue;
		}
		size_t end = text.find(' ', i);
		if (end == std::string::npos)
			end = text.size();
		int channel = ParseChannel(text.substr(i, end - i));
		if (channel >= 0)
		{
			channels.push_back(channel);
			ids.push_back(0);
		}
		i = end;
	}
}

bool StartsWith(const std::string &text, const char *prefix)
{
	size_t n = std::strlen(prefix);
	return text.size() >= n && text.compare(0, n, prefix) == 0;
}

} // namespace

bool ParseTimestamp(const std::string &line, double &unixTime)
{
	// "Fri Sep 11 2026 22:08:55.819 [Info] - ...": five space-separated
	// fields, the last one a clock with a fractional second.
	std::string field[5];
	size_t pos = 0;
	for (auto &f : field)
	{
		while (pos < line.size() && line[pos] == ' ')
			++pos;
		size_t end = pos;
		while (end < line.size() && line[end] != ' ')
			++end;
		if (end == pos)
			return false;
		f = line.substr(pos, end - pos);
		pos = end;
	}
	const std::string &month = field[1];
	char *stop = nullptr;
	const long day = std::strtol(field[2].c_str(), &stop, 10);
	if (*stop != '\0')
		return false;
	const long year = std::strtol(field[3].c_str(), &stop, 10);
	if (*stop != '\0')
		return false;
	const std::string &clock = field[4];
	if (clock.size() < 8 || clock[2] != ':' || clock[5] != ':')
		return false;
	// Named, not temporaries: `stop` points into the text strtol read, and a
	// temporary's buffer is gone by the time it is tested.
	const std::string hourText = clock.substr(0, 2);
	const std::string minuteText = clock.substr(3, 2);
	const long hour = std::strtol(hourText.c_str(), &stop, 10);
	if (*stop != '\0')
		return false;
	const long minute = std::strtol(minuteText.c_str(), &stop, 10);
	if (*stop != '\0')
		return false;
	const double second = std::strtod(clock.c_str() + 6, &stop);
	if (*stop != '\0')
		return false;
	static const char *const months[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
		"Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
	int monthIndex = -1;
	for (int i = 0; i < 12; ++i)
		if (month == months[i])
			monthIndex = i;
	// isfinite first: strtod reads "nan", which fails both range comparisons.
	if (monthIndex < 0 || day < 1 || day > 31 || year < 2000 || hour < 0 || hour > 23 ||
		minute < 0 || minute > 59 || !std::isfinite(second) || second < 0.0 || second >= 61.0)
		return false;
	std::tm tm{};
	tm.tm_year = static_cast<int>(year) - 1900;
	tm.tm_mon = monthIndex;
	tm.tm_mday = static_cast<int>(day);
	tm.tm_hour = static_cast<int>(hour);
	tm.tm_min = static_cast<int>(minute);
	tm.tm_sec = 0;
	tm.tm_isdst = -1;
	std::time_t whole = std::mktime(&tm);   // the log is written in local time
	if (whole == static_cast<std::time_t>(-1))
		return false;
	unixTime = static_cast<double>(whole) + second;
	return true;
}

bool ParseLine(const std::string &line, Event &out)
{
	out = Event{};
	static const char marker[] = "lighthouse: LHR-";
	size_t at = line.find(marker);
	if (at == std::string::npos)
		return false;
	size_t serialStart = at + std::strlen("lighthouse: ");
	size_t serialEnd = serialStart;
	while (serialEnd < line.size() && line[serialEnd] != ' ' && line[serialEnd] != ':')
		++serialEnd;
	out.serial = line.substr(serialStart, serialEnd - serialStart);

	// "LHR-x C: body" (C controller or tracker, H headset) or "LHR-x: body".
	size_t bodyStart = serialEnd;
	if (bodyStart + 2 < line.size() && line[bodyStart] == ' ' &&
		std::isalpha(static_cast<unsigned char>(line[bodyStart + 1])) && line[bodyStart + 2] == ':')
		bodyStart += 3;
	else if (bodyStart < line.size() && line[bodyStart] == ':')
		bodyStart += 1;
	else
		return false;
	while (bodyStart < line.size() && line[bodyStart] == ' ')
		++bodyStart;
	std::string body = line.substr(bodyStart);
	while (!body.empty() && (body.back() == '\r' || body.back() == '\n' || body.back() == ' '))
		body.pop_back();

	out.timeKnown = ParseTimestamp(line, out.unixTime);

	if (StartsWith(body, "SOB: add ") || StartsWith(body, "SOB: drop "))
	{
		bool add = body[5] == 'a';
		out.kind = add ? Event::Kind::StationAdded : Event::Kind::StationDropped;
		std::string rest = body.substr(add ? 9 : 10);
		size_t end = rest.find(' ');
		out.channel = ParseChannel(rest.substr(0, end));
		if (out.channel < 0)
			return false;
		rest = end == std::string::npos ? std::string() : rest.substr(end + 1);

		// Optional "(ID)" or "(generation changed)" about the station itself.
		while (!rest.empty() && rest[0] == '(')
		{
			size_t close = rest.find(')');
			if (close == std::string::npos)
				break;
			std::string inside = rest.substr(1, close - 1);
			if (inside == "generation changed")
				out.generationChanged = true;
			else if (uint32_t id = ParseStationId(inside))
				out.stationId = id;
			rest = rest.substr(close + 1);
			while (!rest.empty() && rest[0] == ' ')
				rest.erase(rest.begin());
		}

		std::vector<int> others;
		std::vector<uint32_t> otherIds;
		if (StartsWith(rest, "also seeing "))
			ParseStationList(rest.substr(12), others, otherIds);
		else if (StartsWith(rest, "seeing "))
			ParseStationList(rest.substr(7), others, otherIds);

		out.visibleKnown = true;
		out.visibleChannels = others;
		out.visibleIds = otherIds;
		if (add)
		{
			out.visibleChannels.push_back(out.channel);
			out.visibleIds.push_back(out.stationId);
		}
		return true;
	}
	if (StartsWith(body, "No base stations seen"))
	{
		out.kind = Event::Kind::NoneSeen;
		out.visibleKnown = true;
		return true;
	}
	size_t boot = body.find("BOOTSTRAPPED base ");
	if (boot != std::string::npos)
	{
		out.kind = Event::Kind::Bootstrapped;
		std::string rest = body.substr(boot + std::strlen("BOOTSTRAPPED base "));
		out.stationId = ParseStationId(rest.substr(0, rest.find(' ')));
		return true;
	}
	if (StartsWith(body, "Trying to start tracking from base "))
	{
		out.kind = Event::Kind::BootstrapFailed;
		std::string rest = body.substr(std::strlen("Trying to start tracking from base "));
		out.stationId = ParseStationId(rest.substr(0, rest.find_first_of(": ")));
		return true;
	}
	return false;
}

std::string DefaultLogPath()
{
	std::string steam;
#ifdef _WIN32
	char buf[MAX_PATH] = {};
	DWORD len = sizeof buf;
	if (RegGetValueA(HKEY_CURRENT_USER, "Software\\Valve\\Steam", "SteamPath",
		RRF_RT_REG_SZ, nullptr, buf, &len) == ERROR_SUCCESS && buf[0] != '\0')
		steam = buf;
#endif
	if (steam.empty())
		steam = "C:\\Program Files (x86)\\Steam";
	for (char &c : steam)
		if (c == '/')
			c = '\\';
	if (steam.back() == '\\')
		steam.pop_back();
	return steam + "\\logs\\vrserver.txt";
}

Tailer::Tailer(std::string p) : path(std::move(p)) { }

void Tailer::Poll(std::vector<Event> &out)
{
	std::ifstream in(path, std::ios::binary);
	if (!in)
	{
		available = false;
		return;
	}
	available = true;
	in.seekg(0, std::ios::end);
	const std::streamoff endPos = in.tellg();
	if (endPos < 0)
		return;
	const uint64_t size = static_cast<uint64_t>(endPos);

	if (!primed)
	{
		primed = true;
		offset = size > ReplayBytes ? size - ReplayBytes : 0;
		skipToNextLine = offset > 0;
		replayEnd = size;
		partial.clear();
	}
	else if (size < offset)
	{
		// Rotated (SteamVR renamed the old file and started a new one) or
		// truncated: whatever is there now is new.
		offset = 0;
		replayEnd = 0;
		skipToNextLine = false;
		partial.clear();
		++rotations;
	}
	if (size == offset)
		return;

	const uint64_t want = std::min<uint64_t>(size - offset, MaxBytesPerPoll);
	std::string buf(static_cast<size_t>(want), '\0');
	in.seekg(static_cast<std::streamoff>(offset));
	in.read(&buf[0], static_cast<std::streamsize>(want));
	buf.resize(static_cast<size_t>(in.gcount()));
	if (buf.empty())
		return;
	uint64_t consumed = offset;   // file position of partial's first byte, once set
	offset += buf.size();

	// A replay that starts mid-file begins inside some line; drop it.
	size_t from = 0;
	if (skipToNextLine)
	{
		size_t nl = buf.find('\n');
		if (nl == std::string::npos)
			return;
		from = nl + 1;
		skipToNextLine = false;
	}
	consumed += from;
	consumed -= partial.size();

	partial.append(buf, from, std::string::npos);
	size_t lineStart = 0;
	for (;;)
	{
		size_t nl = partial.find('\n', lineStart);
		if (nl == std::string::npos)
			break;
		std::string line = partial.substr(lineStart, nl - lineStart);
		++linesSeen;
		Event e;
		if (ParseLine(line, e))
		{
			e.historical = consumed + lineStart < replayEnd;
			out.push_back(std::move(e));
		}
		lineStart = nl + 1;
	}
	partial.erase(0, lineStart);
}

} // namespace lighthouselog
