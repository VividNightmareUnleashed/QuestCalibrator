// Base station visibility: the lighthouse driver's log lines, the tailer
// that follows them, the per-device state folded from them, and the drift
// monitor's behaviour on a simulated tracker whose stations come and go.
#include "VirtualLighthouse.h"
#include "../Overlay/DriftMonitor.h"
#include "../Overlay/LighthouseLog.h"
#include "../Overlay/LighthouseVisibility.h"
#include "../Overlay/RingPoseMath.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{
using lighthouselog::Event;
using Check = void (*)(const char *, bool, const char *);

const char *const Prefix = "Fri Sep 11 2026 22:09:41.018 [Info] - lighthouse: ";

std::string Channels(const std::vector<int> &v)
{
	std::string s;
	for (int c : v)
		s += (s.empty() ? "" : ",") + std::to_string(c);
	return s.empty() ? "-" : s;
}

bool Same(const std::vector<int> &a, std::vector<int> b)
{
	std::vector<int> c = a;
	std::sort(c.begin(), c.end());
	std::sort(b.begin(), b.end());
	return c == b;
}

void ParserScenarios(Check check)
{
	char detail[256];
	Event e;

	bool ok = lighthouselog::ParseLine(std::string(Prefix) +
		"LHR-A3C36EA5 C: SOB: add S-5 also seeing S-8 S-9 S-16", e);
	std::time_t whole = static_cast<std::time_t>(std::floor(e.unixTime));
	std::tm local{};
	localtime_s(&local, &whole);
	double frac = e.unixTime - static_cast<double>(whole);
	snprintf(detail, sizeof detail, "serial %s channel %d visible %s stamp %02d:%02d:%02d.%03d",
		e.serial.c_str(), e.channel, Channels(e.visibleChannels).c_str(),
		local.tm_hour, local.tm_min, local.tm_sec, static_cast<int>(frac * 1000.0 + 0.5));
	check("lighthouse log: add line with plain channels",
		ok && e.kind == Event::Kind::StationAdded && e.serial == "LHR-A3C36EA5" &&
		e.channel == 5 && e.stationId == 0 && e.visibleKnown &&
		Same(e.visibleChannels, { 5, 8, 9, 16 }) && e.timeKnown &&
		local.tm_hour == 22 && local.tm_min == 9 && local.tm_sec == 41 &&
		std::abs(frac - 0.018) < 0.0015 && local.tm_year == 126 && local.tm_mon == 8 &&
		local.tm_mday == 11, detail);

	ok = lighthouselog::ParseLine(std::string(Prefix) +
		"LHR-A3C36EA5 C: SOB: drop S-16 ( 4D47FB4) seeing S-5 (D3D4E73B) S-8 (170EE067) S-9 (F210FBA6)", e);
	snprintf(detail, sizeof detail, "channel %d id %08X visible %s ids %08X %08X %08X",
		e.channel, e.stationId, Channels(e.visibleChannels).c_str(),
		e.visibleIds.size() > 0 ? e.visibleIds[0] : 0u,
		e.visibleIds.size() > 1 ? e.visibleIds[1] : 0u,
		e.visibleIds.size() > 2 ? e.visibleIds[2] : 0u);
	check("lighthouse log: drop line with ids and a leading-zero id",
		ok && e.kind == Event::Kind::StationDropped && e.channel == 16 &&
		e.stationId == 0x04D47FB4u && Same(e.visibleChannels, { 5, 8, 9 }) &&
		e.visibleIds.size() == 3 && e.visibleIds[0] == 0xD3D4E73Bu &&
		e.visibleIds[1] == 0x170EE067u && e.visibleIds[2] == 0xF210FBA6u, detail);

	ok = lighthouselog::ParseLine(std::string(Prefix) +
		"LHR-A3C36EA5 C: SOB: add S-9 (generation changed) also seeing S-5 (D3D4E73B) S-16 ( 4D47FB4)", e);
	snprintf(detail, sizeof detail, "channel %d generation %d visible %s",
		e.channel, e.generationChanged, Channels(e.visibleChannels).c_str());
	check("lighthouse log: generation change is not an id",
		ok && e.kind == Event::Kind::StationAdded && e.channel == 9 && e.generationChanged &&
		e.stationId == 0 && Same(e.visibleChannels, { 5, 16, 9 }), detail);

	ok = lighthouselog::ParseLine(std::string(Prefix) + "LHR-3E61E6B7 C: SOB: add S-8", e);
	check("lighthouse log: first station of a solution",
		ok && e.kind == Event::Kind::StationAdded && e.visibleKnown &&
		Same(e.visibleChannels, { 8 }), Channels(e.visibleChannels).c_str());

	ok = lighthouselog::ParseLine(std::string(Prefix) + "LHR-3E61E6B7 C: SOB: drop S-8", e);
	check("lighthouse log: last station dropped leaves an empty set",
		ok && e.kind == Event::Kind::StationDropped && e.visibleKnown &&
		e.visibleChannels.empty(), "");

	ok = lighthouselog::ParseLine(std::string(Prefix) + "LHR-3E61E6B7 C: No base stations seen...", e);
	check("lighthouse log: no base stations seen",
		ok && e.kind == Event::Kind::NoneSeen && e.visibleKnown && e.visibleChannels.empty(), "");

	ok = lighthouselog::ParseLine(std::string(Prefix) +
		"LHR-3E61E6B7 C: ----- BOOTSTRAPPED base F210FBA6 (best) distance 2.14m velocity 0.31m/s base pitch ~30 deg roll ~-1 deg -----", e);
	snprintf(detail, sizeof detail, "id %08X", e.stationId);
	check("lighthouse log: bootstrapped from a station",
		ok && e.kind == Event::Kind::Bootstrapped && e.stationId == 0xF210FBA6u && !e.visibleKnown, detail);

	ok = lighthouselog::ParseLine(std::string(Prefix) +
		"LHR-3E61E6B7 C: Trying to start tracking from base D3D4E73B: Not enough contiguous samples for a bootstrap pose", e);
	snprintf(detail, sizeof detail, "id %08X", e.stationId);
	check("lighthouse log: failed bootstrap names the station",
		ok && e.kind == Event::Kind::BootstrapFailed && e.stationId == 0xD3D4E73Bu, detail);

	// A VIVE Tracker 3.0 printed no SOB line in six hours (live 2026-09-25):
	// the stations joining each new solution came only as SECONDARY lines,
	// padded to align, with a leading zero printed as a space as elsewhere.
	ok = lighthouselog::ParseLine(std::string(Prefix) +
		"LHR-D520226E C: ----- SECONDARY base 4921060B distance 2.01m  -----", e);
	Event padded, attempt;
	const bool paddedOk = lighthouselog::ParseLine(std::string(Prefix) +
		"LHR-D520226E C: ----- SECONDARY base  4D47FB4 distance 3.03m   -----", padded);
	const bool attemptIgnored = !lighthouselog::ParseLine(std::string(Prefix) +
		"LHR-D520226E C: Trying to add a secondary base FD626122: Not enough contiguous samples for a bootstrap pose", attempt);
	snprintf(detail, sizeof detail, "id %08X channel %d, padded %d id %08X, attempt ignored %d",
		e.stationId, e.channel, paddedOk, padded.stationId, attemptIgnored);
	check("lighthouse log: a secondary station names its id",
		ok && e.kind == Event::Kind::SecondaryAdded && e.stationId == 0x4921060Bu && e.channel < 0 &&
		!e.visibleKnown && paddedOk && padded.kind == Event::Kind::SecondaryAdded &&
		padded.stationId == 0x04D47FB4u && attemptIgnored, detail);

	bool noise =
		!lighthouselog::ParseLine(std::string(Prefix) + "LHR-A3C36EA5 C: Assertion failed:  Explicit right sync does not match pending frame", e) &&
		!lighthouselog::ParseLine(std::string(Prefix) + "LHR-A3C36EA5 C: Unexpected centroid ordering error 1.0ms S-5", e) &&
		!lighthouselog::ParseLine(std::string(Prefix) + "LHR-A3C36EA5: Updated IMU calibration: Accel bias change 0.01m/s/s", e) &&
		!lighthouselog::ParseLine("Fri Sep 11 2026 22:08:58.279 [Info] - lighthouse: 0C9FF12B16: Triggered keepalive (succeeded)", e) &&
		!lighthouselog::ParseLine("Fri Sep 11 2026 22:08:45.150 [Info] - Loaded server driver 01questcalibrator", e) &&
		!lighthouselog::ParseLine("", e) &&
		!lighthouselog::ParseLine("lighthouse: LHR-", e);
	check("lighthouse log: other driver lines are ignored", noise, "");

	ok = lighthouselog::ParseLine("lighthouse: LHR-A3C36EA5 C: SOB: add S-5", e);
	check("lighthouse log: a line without a stamp still parses",
		ok && !e.timeKnown && e.channel == 5, "");

	// The radio link, as the headset tracker's went on 2026-09-26: SteamVR
	// switched it off after it sat still, the receiver let it go, and the
	// player turned it back on two minutes later.
	{
		const std::string at = "Sat Sep 26 2026 23:47:20.560 [Info] - lighthouse: ";
		Event off, gone, back, plainOff, receiver;
		const bool offOk = lighthouselog::ParseLine(at + "Device LHR-D520226E powering off upon entering standby.", off);
		const bool goneOk = lighthouselog::ParseLine(at + "LHR-D520226E: Disconnected from receiver 4BF089B604", gone);
		const bool backOk = lighthouselog::ParseLine(at + "LHR-D520226E: Connected to receiver 4BF089B604", back);
		const bool plainOk = lighthouselog::ParseLine(at + "Device LHR-D520226E powering off", plainOff);
		const bool ignored =
			!lighthouselog::ParseLine(at + "4BF089B604: Wireless controller LHR-D520226E disconnected", receiver) &&
			!lighthouselog::ParseLine("Sat Sep 26 2026 23:47:20.560 [Info] - 9 - entering standby", receiver) &&
			!lighthouselog::ParseLine(at + "Device LHR-D520226E is ready", receiver) &&
			!lighthouselog::ParseLine(at + "LHR-D520226E C: Dropped 5 rejected updates, 122213 back-facing hits", receiver);
		snprintf(detail, sizeof detail, "off %d (%s standby %d stamp %d) gone %d back %d plain %d (standby %d) ignored %d",
			offOk, off.serial.c_str(), off.standby, off.timeKnown, goneOk, backOk, plainOk, plainOff.standby, ignored);
		check("lighthouse log: power-off and receiver lines name the device",
			offOk && off.kind == Event::Kind::PoweredOff && off.serial == "LHR-D520226E" && off.standby &&
			off.timeKnown && !off.visibleKnown && off.channel < 0 &&
			goneOk && gone.kind == Event::Kind::Disconnected && gone.serial == "LHR-D520226E" && !gone.standby &&
			backOk && back.kind == Event::Kind::Connected && back.serial == "LHR-D520226E" &&
			plainOk && plainOff.kind == Event::Kind::PoweredOff && !plainOff.standby && ignored, detail);
	}

	// The universe, which no device owns, and the server's first line, as
	// SteamVR 2.17.10 wrote them on 2026-09-27. The driver's other universe
	// lines are not these.
	{
		const std::string at = "Sun Sep 27 2026 21:43:52.456 [Info] - lighthouse: ";
		Event chosen, created, stopped, server, other;
		const bool chosenOk = lighthouselog::ParseLine(at +
			"Selected existing universe 1744988537 (170EE067 is primary)", chosen);
		const bool createdOk = lighthouselog::ParseLine(at +
			"Creating new universe 1744988537 because there were no existing universes", created);
		const bool stoppedOk = lighthouselog::ParseLine(
			"Sun Sep 27 2026 22:30:01.060 [Info] - lighthouse: Stopped tracking with universe 1744988537", stopped);
		const bool serverOk = lighthouselog::ParseLine(
			"Sun Sep 27 2026 21:43:33.604 [Info] - vrserver 2.17.10 startup with PID=5976, "
			"config=C:\\Program Files (x86)\\Steam\\config, arch=win64", server);
		const bool othersIgnored =
			!lighthouselog::ParseLine(at + "BootstrapFinished setting tilt base to 170EE067", other) &&
			!lighthouselog::ParseLine(at + "Setting universe tilt from 170EE067 via transform to global: pitch -26.26 deg roll 1.89 deg", other) &&
			!lighthouselog::ParseLine(at + "Moving base F210FBA6 1205mm and 42.7 deg because of relationship with 170EE067, which is closer to the origin", other) &&
			!lighthouselog::ParseLine(at + "Selected existing universe (170EE067 is primary)", other) &&
			!lighthouselog::ParseLine("Sun Sep 27 2026 21:43:34.881 [Info] - Found universe 1 in chaperone file", other) &&
			!lighthouselog::ParseLine("Sun Sep 27 2026 21:43:33.604 [Info] - vrserver 2.17.10 watchdogs enabled", other);
		snprintf(detail, sizeof detail, "chosen %d (%llu, created %d, '%s') created %d (%d) stopped %d (%llu) server %d (%d) ignored %d",
			chosenOk, static_cast<unsigned long long>(chosen.universeId), chosen.universeCreated, chosen.serial.c_str(),
			createdOk, created.universeCreated, stoppedOk, static_cast<unsigned long long>(stopped.universeId),
			serverOk, server.timeKnown, othersIgnored);
		check("lighthouse log: the universe and the server's start name no device",
			chosenOk && chosen.kind == Event::Kind::UniverseChosen && chosen.universeId == 1744988537ull &&
			!chosen.universeCreated && chosen.serial.empty() && chosen.timeKnown && chosen.channel < 0 &&
			createdOk && created.kind == Event::Kind::UniverseChosen && created.universeCreated &&
			stoppedOk && stopped.kind == Event::Kind::UniverseStopped && stopped.universeId == 1744988537ull &&
			serverOk && server.kind == Event::Kind::ServerStarted && server.serial.empty() && server.timeKnown &&
			othersIgnored, detail);
	}
}

void TailerScenarios(Check check)
{
	namespace fs = std::filesystem;
	const auto dir = fs::temp_directory_path();
	const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
	const fs::path path = dir / ("QuestCalLighthouseTail-" + std::to_string(stamp) + ".txt");
	const std::string a = std::string(Prefix) + "LHR-A3C36EA5 C: SOB: add S-5\n";
	const std::string b = std::string(Prefix) + "LHR-A3C36EA5 C: SOB: add S-8 also seeing S-5\n";
	const std::string noise = "Fri Sep 11 2026 22:08:45.150 [Info] - Loaded server driver lighthouse\n";
	const std::string c = std::string(Prefix) + "LHR-A3C36EA5 C: SOB: drop S-5 seeing S-8\n";
	{
		std::ofstream out(path, std::ios::binary);
		out << a << noise << b << c.substr(0, 40);   // the last line is still being written
	}
	lighthouselog::Tailer tail(path.string());
	std::vector<Event> events;
	tail.Poll(events);
	char detail[160];
	snprintf(detail, sizeof detail, "events %zu lines %llu available %d",
		events.size(), static_cast<unsigned long long>(tail.LinesSeen()), tail.Available());
	check("lighthouse tail: the first poll replays what is there as history",
		tail.Available() && events.size() == 2 && events[0].historical && events[1].historical &&
		events[0].channel == 5 && events[1].channel == 8 && tail.LinesSeen() == 3, detail);

	events.clear();
	tail.Poll(events);
	check("lighthouse tail: nothing new, nothing returned", events.empty(), "");

	{
		std::ofstream out(path, std::ios::binary | std::ios::app);
		out << c.substr(40) << a;
	}
	tail.Poll(events);
	snprintf(detail, sizeof detail, "events %zu first kind %d historical %d/%d", events.size(),
		events.empty() ? -1 : static_cast<int>(events[0].kind),
		events.size() > 0 ? events[0].historical : -1, events.size() > 1 ? events[1].historical : -1);
	check("lighthouse tail: a line completed later arrives once, new lines are live",
		events.size() == 2 && events[0].kind == Event::Kind::StationDropped &&
		events[0].historical && !events[1].historical && events[1].channel == 5, detail);

	events.clear();
	{
		std::ofstream out(path, std::ios::binary | std::ios::trunc);
		out << b;
	}
	tail.Poll(events);
	snprintf(detail, sizeof detail, "events %zu rotations %llu", events.size(),
		static_cast<unsigned long long>(tail.Rotations()));
	check("lighthouse tail: a rotated file is read from its start",
		events.size() == 1 && !events[0].historical && events[0].channel == 8 &&
		tail.Rotations() == 1, detail);

	std::error_code ec;
	fs::remove(path, ec);

	lighthouselog::Tailer missing((dir / "QuestCalLighthouseTail-missing.txt").string());
	events.clear();
	missing.Poll(events);
	check("lighthouse tail: a missing log is no information, not an error",
		!missing.Available() && events.empty(), "");

	const std::string logPath = lighthouselog::DefaultLogPath();
	const std::string suffix = "\\logs\\vrserver.txt";
	check("lighthouse tail: the default path is SteamVR's server log",
		logPath.size() > suffix.size() + 2 &&
		logPath.compare(logPath.size() - suffix.size(), suffix.size(), suffix) == 0, logPath.c_str());
}

Event Made(Event::Kind kind, const char *serial, int channel, std::vector<int> visible,
	bool historical = false)
{
	Event e;
	e.kind = kind;
	e.serial = serial;
	e.channel = channel;
	e.stationId = channel >= 0 ? 0xA000u + channel : 0;
	e.visibleKnown = kind == Event::Kind::StationAdded || kind == Event::Kind::StationDropped ||
		kind == Event::Kind::NoneSeen;
	e.visibleChannels = std::move(visible);
	e.visibleIds.assign(e.visibleChannels.size(), 0);
	e.historical = historical;
	return e;
}

void VisibilityScenarios(Check check)
{
	using K = Event::Kind;
	LighthouseVisibility vis;
	char detail[256];

	// Startup replay: the sets are learned, nothing is a disturbance.
	vis.Apply(Made(K::StationAdded, "LHR-1", 5, { 5 }, true), 0.0);
	vis.Apply(Made(K::StationAdded, "LHR-1", 8, { 5, 8 }, true), 0.0);
	vis.Apply(Made(K::StationAdded, "LHR-1", 9, { 5, 8, 9 }, true), 0.0);
	vis.Apply(Made(K::StationAdded, "LHR-2", 5, { 5 }, true), 0.0);
	vis.Apply(Made(K::NoneSeen, "LHR-2", -1, {}, true), 0.0);
	const LighthouseVisibility::Device *one = vis.Find("LHR-1");
	const LighthouseVisibility::Device *two = vis.Find("LHR-2");
	snprintf(detail, sizeof detail, "LHR-1 sees %s, LHR-2 sees %s, stations %d",
		one ? Channels(one->visible).c_str() : "?", two ? Channels(two->visible).c_str() : "?",
		vis.StationCount());
	// LHR-2 is degraded right now (it sees nothing), which is state and
	// holds; but the replay opened no window for it.
	check("lighthouse state: replayed lines set the sets and open no window",
		one && two && Same(one->visible, { 5, 8, 9 }) && two->visible.empty() && two->visibleKnown &&
		vis.StationCount() == 3 && !vis.Disturbed("LHR-1", 5.0) && !one->degraded &&
		vis.Disturbed("LHR-2", 5.0) && two->degraded && two->lastDisturbance <= -1e8 &&
		one->events == 3 && two->losses == 1, detail);

	// Routine handoffs with two or more stations left are counted, not
	// disturbances.
	std::string note1 = vis.Apply(Made(K::StationDropped, "LHR-1", 9, { 5, 8 }), 100.0);
	std::string note2 = vis.Apply(Made(K::StationAdded, "LHR-1", 9, { 5, 8, 9 }), 103.0);
	one = vis.Find("LHR-1");
	check("lighthouse state: a handoff that keeps two stations is only counted",
		note1.empty() && note2.empty() && one->drops == 1 && !vis.Disturbed("LHR-1", 104.0) &&
		vis.Stations().front().channel == 9 && vis.Stations().front().drops == 1, note1.c_str());

	// Down to one station, then back: both disturb.
	std::string down1 = vis.Apply(Made(K::StationDropped, "LHR-1", 9, { 5, 8 }), 200.0);
	std::string down2 = vis.Apply(Made(K::StationDropped, "LHR-1", 8, { 5 }), 200.5);
	snprintf(detail, sizeof detail, "note '%s'", down2.c_str());
	check("lighthouse state: a drop to a single station disturbs for as long as it lasts",
		down1.empty() && down2.find("down to one station") == 0 &&
		vis.Disturbed("LHR-1", 205.0) && vis.Disturbed("LHR-1", 229.0) &&
		vis.Find("LHR-1")->degraded, detail);
	std::string returned = vis.Apply(Made(K::StationAdded, "LHR-1", 8, { 5, 8 }), 230.0);
	check("lighthouse state: the return to two stations disturbs for the window only",
		returned.find("back to 2 stations") == 0 && !vis.Find("LHR-1")->degraded &&
		vis.Disturbed("LHR-1", 235.0) && vis.Disturbed("LHR-1", 240.0) &&
		!vis.Disturbed("LHR-1", 241.0), returned.c_str());

	// Full loss and bootstrap.
	std::string lost = vis.Apply(Made(K::StationDropped, "LHR-1", 8, { 5 }), 300.0);
	std::string last = vis.Apply(Made(K::StationDropped, "LHR-1", 5, {}), 300.2);
	std::string none = vis.Apply(Made(K::NoneSeen, "LHR-1", -1, {}), 300.3);
	std::string booted = vis.Apply(Made(K::Bootstrapped, "LHR-1", 8, {}), 309.0);
	std::string rejoined = vis.Apply(Made(K::StationAdded, "LHR-1", 8, { 8 }), 309.1);
	const bool singleAfterBoot = vis.Find("LHR-1")->degraded && vis.Disturbed("LHR-1", 311.0);
	std::string second = vis.Apply(Made(K::StationAdded, "LHR-1", 5, { 5, 8 }), 312.0);
	one = vis.Find("LHR-1");
	snprintf(detail, sizeof detail, "losses %u bootstraps %u drops %u notes '%s' / '%s' / '%s' / '%s'",
		one->losses, one->bootstraps, one->drops, last.c_str(), booted.c_str(), rejoined.c_str(),
		second.c_str());
	check("lighthouse state: a full loss counts once and every recovery line disturbs",
		last.find("lost its last station") == 0 && !none.empty() && one->losses == 1 &&
		booted.find("started a new solution") == 0 && rejoined.find("tracking again") == 0 &&
		singleAfterBoot && second.find("back to 2 stations") == 0 &&
		one->bootstraps == 1 && one->drops == 5 && Same(one->visible, { 5, 8 }) &&
		!one->degraded && vis.Disturbed("LHR-1", 321.0) && !vis.Disturbed("LHR-1", 323.0), detail);

	// Station ranking: most dropped first; names carry the id once known.
	vis.Apply(Made(K::StationDropped, "LHR-2", 5, {}), 400.0);
	auto ranked = vis.Stations();
	snprintf(detail, sizeof detail, "first %s drops %u, name %s", ranked.empty() ? "?" :
		std::to_string(ranked[0].channel).c_str(), ranked.empty() ? 0u : ranked[0].drops,
		vis.StationName(5).c_str());
	check("lighthouse state: stations rank by drops and are named by id",
		ranked.size() == 3 && ranked[0].channel == 5 && ranked[0].drops == 2 &&
		vis.StationName(5) == "S-5 (0000A005)", detail);

	// Settling, for the continuous loop: a device the log never named is
	// settled, a replayed bootstrap leaves its device on the one station it
	// started from (state) without counting a restart, a live restart is
	// counted and holds for the window (a bootstrap counts as one of its own
	// as well, "tracking again" does not), and fewer than two known stations
	// hold for as long as they last.
	{
		const bool unknownSettled = !vis.Settling("LHR-3", 500.0);
		vis.Apply(Made(K::Bootstrapped, "LHR-4", 5, {}, true), 0.0);
		const LighthouseVisibility::Device *replayed = vis.Find("LHR-4");
		const bool replayedSettled = vis.Settling("LHR-4", 500.0) && vis.Disturbed("LHR-4", 500.0) &&
			replayed->InView() == 1 && replayed->liveRestarts == 0 && replayed->liveDisturbances == 0 &&
			replayed->liveBootstraps == 0;

		vis.Apply(Made(K::Bootstrapped, "LHR-3", 5, {}), 500.0);
		const LighthouseVisibility::Device *three = vis.Find("LHR-3");
		const bool bootCounted = three->liveRestarts == 1 && three->liveDisturbances == 1 &&
			three->liveBootstraps == 1 &&
			vis.RestartedWithin("LHR-3", 503.0, 5.0) && !vis.RestartedWithin("LHR-3", 506.0, 5.0) &&
			vis.Settling("LHR-3", 505.0);
		// "tracking again": a restart too, but no solution from scratch.
		vis.Apply(Made(K::StationAdded, "LHR-3", 5, { 5 }), 500.1);
		vis.Apply(Made(K::StationAdded, "LHR-3", 8, { 5, 8 }), 501.0); // "back to 2": disturbance only
		three = vis.Find("LHR-3");
		const bool windowOnly = three->liveRestarts == 2 && three->liveDisturbances == 3 &&
			three->liveBootstraps == 1 &&
			vis.Settling("LHR-3", 510.0) && !vis.Settling("LHR-3", 512.0);
		vis.Apply(Made(K::StationDropped, "LHR-3", 8, { 5 }), 520.0);
		const bool singleHolds = vis.Settling("LHR-3", 535.0) && vis.Find("LHR-3")->liveRestarts == 2;

		snprintf(detail, sizeof detail, "unknown %d replayed %d boot %d window %d single %d",
			unknownSettled, replayedSettled, bootCounted, windowOnly, singleHolds);
		check("lighthouse state: settling and restart counts for the continuous loop",
			unknownSettled && replayedSettled && bootCounted && windowOnly && singleHolds, detail);
	}

	// A device that reports the stations joining a new solution only in
	// SECONDARY lines (the headset tracker of 2026-09-25): the bootstrap
	// leaves it on its one station, which holds past the restart's window,
	// and each SECONDARY line adds one without a session-log line. A station
	// no line has named yet still counts. An SOB line is the device's own
	// account and replaces the rebuilt set.
	{
		LighthouseVisibility rebuild;
		Event map = Made(K::StationAdded, "LHR-9", 13, { 2, 3, 11, 13 }, true);
		map.visibleIds = { 0xA002u, 0xA003u, 0xA00Bu, 0xA00Du };
		rebuild.Apply(map, 0.0);
		auto boot = [&](double t)
		{
			Event b = Made(K::Bootstrapped, "LHR-T", -1, {});
			b.stationId = 0xA00Du;   // the real line names the id, not the channel
			return rebuild.Apply(b, t);
		};
		auto secondary = [&](uint32_t id, double t)
		{
			Event s2 = Made(K::SecondaryAdded, "LHR-T", -1, {});
			s2.stationId = id;
			return rebuild.Apply(s2, t);
		};
		const std::string started = boot(100.0);
		const LighthouseVisibility::Device *t = rebuild.Find("LHR-T");
		const bool oneHolds = t->visibleKnown && t->InView() == 1 && Same(t->visible, { 13 }) &&
			t->degraded && rebuild.Settling("LHR-T", 115.0);
		const std::string joined = secondary(0xA00Bu, 116.0);
		const bool twoClear = joined.empty() && t->InView() == 2 && !t->degraded &&
			!rebuild.Settling("LHR-T", 117.0);
		secondary(0xA002u, 116.1);
		secondary(0xB0B0B0B0u, 116.2);
		secondary(0xA002u, 116.3);   // named twice, counted once
		const bool full = Same(t->visible, { 2, 11, 13 }) && t->unmappedIds.size() == 1 &&
			t->unmappedIds[0] == 0xB0B0B0B0u && t->InView() == 4;
		boot(200.0);
		const bool over = t->InView() == 1 && t->unmappedIds.empty() && rebuild.Settling("LHR-T", 215.0);
		Event own = Made(K::StationAdded, "LHR-T", 13, { 13 });
		own.visibleIds = { 0xA00Du };
		const std::string again = rebuild.Apply(own, 200.1);
		secondary(0xA002u, 200.2);   // after the SOB line, not news
		const bool sobRules = again.find("tracking again") == 0 && !t->rebuilding && t->InView() == 1 &&
			t->liveRestarts == 3;
		snprintf(detail, sizeof detail, "boot '%s' one %d two %d full %d (in view %d) over %d sob %d",
			started.c_str(), oneHolds, twoClear, full, t->InView(), over, sobRules);
		check("lighthouse state: a solution rebuilt from SECONDARY lines",
			started.find("started a new solution from S-13") == 0 && oneHolds && twoClear && full &&
			over && sobRules, detail);
	}

	// The radio link: a standby power-off is state and a note, never a
	// disturbance or a restart (the device reports no pose to disturb), its
	// reason survives the receiver's disconnect line, and the link coming
	// back or any tracking line clears it. A replayed power-off sets the
	// state but counts nothing.
	{
		LighthouseVisibility radio;
		auto link = [&](K kind, bool standby, double t, bool historical = false)
		{
			Event e = Made(kind, "LHR-D520226E", -1, {}, historical);
			e.standby = standby;
			return radio.Apply(e, t);
		};
		radio.Apply(Made(K::StationAdded, "LHR-D520226E", 13, { 2, 13 }), 10.0);
		const LighthouseVisibility::Device *t = radio.Find("LHR-D520226E");
		const uint32_t disturbancesBefore = t->liveDisturbances;
		const std::string off = link(K::PoweredOff, true, 100.0);
		const std::string gone = link(K::Disconnected, false, 100.5);
		const bool offState = t->off && t->standbyOff && t->standbyPowerOffs == 1 &&
			t->liveDisturbances == disturbancesBefore && t->liveRestarts == 0 &&
			t->lastPowerOff == 100.0 && gone.empty();
		const std::string back = link(K::Connected, false, 243.0);
		const bool backState = !t->off && !t->standbyOff && t->standbyPowerOffs == 1;

		const std::string dropped = link(K::Disconnected, false, 300.0);
		const bool droppedState = t->off && !t->standbyOff;
		radio.Apply(Made(K::Bootstrapped, "LHR-D520226E", 13, {}), 305.0);   // no Connected line seen
		const bool trackingClears = !t->off;

		link(K::PoweredOff, true, 0.0, /*historical=*/true);
		const bool replayed = t->off && t->standbyOff && t->standbyPowerOffs == 1;

		snprintf(detail, sizeof detail, "off '%s' gone '%s' back '%s' dropped '%s', state %d %d %d %d %d",
			off.c_str(), gone.c_str(), back.c_str(), dropped.c_str(), offState, backState, droppedState,
			trackingClears, replayed);
		check("lighthouse state: a standby power-off is the device's state, not a disturbance",
			off == "switched off by SteamVR after sitting still" && back == "connected again" &&
			dropped == "disconnected from its receiver" &&
			offState && backState && droppedState && trackingClears && replayed, detail);
	}

	// A calibration waits for the lighthouse side to settle. The headset
	// tracker's return on 2026-09-26, from 23:50:19 (0 s): tracking again,
	// back to two stations, new solutions at 3 s and 7 s, three secondaries
	// at 9 s. It was measured from 23:50:32 (13 s) and came out 1.2 deg off
	// its mount; 15 s after the last new solution starts it at 22 s. A device
	// down to one station waits however long ago it restarted.
	{
		LighthouseVisibility wait;
		const char *tracker = "LHR-D520226E";
		wait.Apply(Made(K::NoneSeen, tracker, 13, {}), -150.0);
		wait.Apply(Made(K::StationAdded, tracker, 13, { 13 }), 0.0);
		wait.Apply(Made(K::StationAdded, tracker, 2, { 2, 13 }), 2.0);
		wait.Apply(Made(K::Bootstrapped, tracker, 11, {}), 3.0);
		wait.Apply(Made(K::Bootstrapped, tracker, 13, {}), 7.0);
		for (int station : { 3, 2, 11 })
			wait.Apply(Made(K::SecondaryAdded, tracker, station, {}), 9.0);
		const bool atSampling = wait.SettledFor(tracker, 13.0, 15.0);
		const bool justBefore = wait.SettledFor(tracker, 21.9, 15.0);
		const bool after = wait.SettledFor(tracker, 22.1, 15.0);
		const bool unnamed = wait.SettledFor("LHR-00000000", 13.0, 15.0);
		wait.Apply(Made(K::StationDropped, tracker, 3, { 13 }), 30.0);
		const bool single = wait.SettledFor(tracker, 60.0, 15.0);
		snprintf(detail, sizeof detail, "at 13 s %d, 21.9 s %d, 22.1 s %d; unnamed %d; one station %d",
			atSampling, justBefore, after, unnamed, single);
		check("lighthouse state: a calibration waits 15 s after the last new solution, and for two stations",
			!atSampling && !justBefore && after && unnamed && !single, detail);
	}

	// SteamVR's startup on 2026-09-27 through the parser, as the overlay read
	// it: when it started the file held an earlier session and this one's
	// first line, and the rest came live. Seconds from the server's start.
	// The census followed the station placed 26 ms after the universe was
	// chosen; a station SteamVR moved 20 s later is one to follow.
	{
		using U = LighthouseVisibility::UniverseSetup;
		LighthouseVisibility setup;
		double start = 0.0;
		lighthouselog::ParseTimestamp("Sun Sep 27 2026 21:43:33.604 [Info] - x", start);
		auto feed = [&](const char *text, bool historical)
		{
			Event e;
			if (!lighthouselog::ParseLine(text, e))
				return -1e9;
			e.historical = historical;
			setup.Apply(e, e.unixTime - start);
			return e.unixTime - start;
		};
		const U before = setup.Universe(0.0);
		feed("Sun Sep 27 2026 18:17:55.130 [Info] - vrserver 2.17.10 startup with PID=416, arch=win64", true);
		// Made up: a device an earlier session named.
		feed("Sun Sep 27 2026 18:17:56.000 [Info] - lighthouse: LHR-A3C36EA5 C: SOB: add S-5 also seeing S-8", true);
		const bool earlierNamed = setup.Find("LHR-A3C36EA5") != nullptr;
		feed("Sun Sep 27 2026 21:43:33.604 [Info] - vrserver 2.17.10 startup with PID=5976, arch=win64", true);
		const bool forgotten = setup.Find("LHR-A3C36EA5") == nullptr && setup.StationCount() == 0;
		const U waiting = setup.Universe(5.0);
		const double sob = feed("Sun Sep 27 2026 21:43:43.887 [Info] - lighthouse: LHR-3E61E6B7 C: SOB: add S-8", false);
		setup.Apply(Made(K::StationAdded, "LHR-C48117DF", 5, { 5, 8 }), sob);   // two stations, no restart
		const double boot = feed("Sun Sep 27 2026 21:43:47.830 [Info] - lighthouse: LHR-3E61E6B7 C: ----- BOOTSTRAPPED "
			"base F210FBA6 (best) distance 1.23m velocity 0.85m/s base pitch ~21.1 deg roll ~-1.7 deg -----", false);
		const U tracking = setup.Universe(boot + 0.1);
		const double chosen = feed("Sun Sep 27 2026 21:43:52.456 [Info] - lighthouse: "
			"Selected existing universe 1744988537 (170EE067 is primary)", false);
		const U placing = setup.Universe(chosen + 0.026);       // Moving base F210FBA6 1205mm and 42.7 deg
		const U stampedBefore = setup.Universe(chosen - 0.2);
		const U lastPlacing = setup.Universe(chosen + 9.9);
		const U settled = setup.Universe(chosen + 10.1);
		const U later = setup.Universe(chosen + 19.821);        // Moving base D3D4E73B 94mm ... for tracking too
		const bool heldForUniverse = !setup.SettledFor("LHR-C48117DF", chosen + 5.0, 15.0) &&
			setup.SettledFor("LHR-C48117DF", chosen + 10.5, 15.0) &&
			setup.SettledFor("LHR-00000000", chosen + 5.0, 15.0);
		const uint64_t id = setup.UniverseId();
		const double stop = feed("Sun Sep 27 2026 22:30:01.060 [Info] - lighthouse: Stopped tracking with universe 1744988537", false);
		const U down = setup.Universe(stop + 1.0);
		// Tracking again and no universe chosen a minute on: the log stopped
		// saying so, and says nothing either way.
		setup.Apply(Made(K::StationAdded, "LHR-3E61E6B7", 8, { 5, 8 }), stop + 10.0);
		const U stillWaiting = setup.Universe(stop + 69.0);
		const U silent = setup.Universe(stop + 71.0);

		// Replayed at the overlay's start, the choice counts from its stamp:
		// an overlay started 4 s after it still leaves the placing alone.
		LighthouseVisibility replayed;
		Event pick;
		lighthouselog::ParseLine("Sun Sep 27 2026 21:43:52.456 [Info] - lighthouse: "
			"Selected existing universe 1744988537 (170EE067 is primary)", pick);
		pick.historical = true;
		replayed.Apply(pick, 100.0);
		const U replayedRecent = replayed.Universe(104.0);
		const U replayedOld = replayed.Universe(200.0);

		snprintf(detail, sizeof detail, "before %d, earlier %d forgotten %d, waiting %d tracking %d, chosen at %.3f s: "
			"placing %d before %d last %d settled %d later %d, held %d, id %llu, down %d, still %d silent %d, replayed %d %d",
			static_cast<int>(before), earlierNamed, forgotten, static_cast<int>(waiting), static_cast<int>(tracking), chosen,
			static_cast<int>(placing), static_cast<int>(stampedBefore), static_cast<int>(lastPlacing),
			static_cast<int>(settled), static_cast<int>(later), heldForUniverse, static_cast<unsigned long long>(id),
			static_cast<int>(down), static_cast<int>(stillWaiting), static_cast<int>(silent),
			static_cast<int>(replayedRecent), static_cast<int>(replayedOld));
		check("lighthouse state: frame moves and calibrations wait while SteamVR sets up its universe",
			before == U::Unknown && earlierNamed && forgotten && waiting == U::SettingUp && tracking == U::SettingUp &&
			std::abs(chosen - 18.852) < 0.002 && placing == U::SettingUp && stampedBefore == U::SettingUp &&
			lastPlacing == U::SettingUp && settled == U::Settled && later == U::Settled && heldForUniverse &&
			id == 1744988537ull && down == U::SettingUp && stillWaiting == U::SettingUp && silent == U::Unknown &&
			replayedRecent == U::SettingUp && replayedOld == U::Settled, detail);
	}

	// A tracker whose SOB lines carry no ids (LHR-BFF71BF1 on 2026-09-27)
	// rebuilds its set from bootstrap and SECONDARY lines by id; once another
	// device's line gives a station's channel, the set holds it by channel.
	{
		LighthouseVisibility ids;
		Event boot = Made(K::Bootstrapped, "LHR-BFF71BF1", -1, {});
		boot.stationId = 0xF210FBA6u;
		ids.Apply(boot, 48.5);
		Event secondary = Made(K::SecondaryAdded, "LHR-BFF71BF1", -1, {});
		secondary.stationId = 0x170EE067u;
		ids.Apply(secondary, 50.7);
		const LighthouseVisibility::Device *t = ids.Find("LHR-BFF71BF1");
		const bool byId = t->visible.empty() && t->unmappedIds.size() == 2 && t->InView() == 2 && !t->degraded;
		Event named = Made(K::StationAdded, "LHR-841C98C3", 9, { 5, 9 });
		named.stationId = 0xF210FBA6u;
		named.visibleIds = { 0, 0xF210FBA6u };
		ids.Apply(named, 60.0);
		const bool byChannel = Same(t->visible, { 9 }) && t->unmappedIds.size() == 1 &&
			t->unmappedIds[0] == 0x170EE067u && t->InView() == 2 && !t->degraded;
		snprintf(detail, sizeof detail, "by id %d, by channel %d (visible %s, unmapped %zu)",
			byId, byChannel, Channels(t->visible).c_str(), t->unmappedIds.size());
		check("lighthouse state: a station named by id alone joins by channel once a line gives it",
			byId && byChannel, detail);
	}

	vis.Reset();
	check("lighthouse state: reset forgets everything",
		vis.Devices().empty() && vis.StationCount() == 0 &&
		vis.Universe(0.0) == LighthouseVisibility::UniverseSetup::Unknown, "");
}

// Runs the simulated tracker through the drift monitor with the log lines
// applied at their times, the way the overlay does it.
struct Verdict
{
	int slides = 0, losses = 0;
	int attributed = 0;      // events the visibility state explained
	double firstEventTime = -1.0;
	double lastEventMagnitude = 0.0;
};

Verdict Judge(const vlighthouse::Output &run, const vlighthouse::Config &cfg,
	bool applyLines, bool asHistory = false)
{
	Verdict v;
	DriftMonitor monitor(cfg.qpcToSeconds);
	LighthouseVisibility vis;
	size_t next = 0;
	for (const auto &f : run.frames)
	{
		while (next < run.events.size() && run.events[next].time <= f.time)
		{
			if (applyLines)
			{
				Event e = run.events[next].event;
				if (asHistory)
					e.historical = true;
				vis.Apply(e, run.events[next].time);
			}
			++next;
		}
		// The overlay feeds the monitor trusted samples only.
		if (IsTrustedRingSample(f.sample, cfg.qpcToSeconds))
			monitor.Push(f.sample);
		DriftMonitor::Event ev;
		while (monitor.PollEvent(ev))
		{
			if (ev.type == DriftMonitor::Event::StationarySlide)
				v.slides++;
			else
				v.losses++;
			if (v.firstEventTime < 0.0)
				v.firstEventTime = ev.time;
			v.lastEventMagnitude = ev.magnitude;
			if (vis.Disturbed(cfg.serial, ev.time))
				v.attributed++;
		}
	}
	return v;
}

// The detailed log gets one line a minute for station counts, not one per
// change: the devices that changed, their range and how often.
void DigestScenarios(Check check)
{
	VisibilityDigest digest;
	bool quietAtStart = digest.Flush(100.0).empty();
	digest.Note("LHR-A", 4);      // first sight is not a change
	digest.Note("LHR-B", 3);
	digest.Note("LHR-A", 3);
	digest.Note("LHR-A", 3);      // the same count again is not a change
	digest.Note("LHR-A", 2);
	digest.Note("LHR-A", 4);
	bool notDue = digest.Flush(159.0).empty();
	std::string line = digest.Flush(160.0);
	bool summarised = line == "base stations seen over the last minute: LHR-A 2 to 4 (3 changes)";
	digest.Note("LHR-B", 4);
	std::string next = digest.Flush(220.0);
	bool restarted = next == "base stations seen over the last minute: LHR-B 3 to 4 (1 change)";
	bool quiet = digest.Flush(280.0).empty();
	check("lighthouse log: station counts are summarised once a minute",
		quietAtStart && notDue && summarised && restarted && quiet, (line + " | " + next).c_str());
}

void ModelScenarios(Check check)
{
	using namespace vlighthouse;
	char detail[256];
	Config cfg;
	const Eigen::Vector3d home(0.4, 1.1, -0.3);
	const std::vector<int> all = { 5, 8, 9, 16 };

	// A stationary tracker with every station in view that starts to slide
	// (the universes drifting apart) is drift evidence and stays so.
	{
		Output run = Run(cfg, DriftFrom(home, Eigen::Vector3d(0, 0, 0.002), 12.0), { { 0.0, all } }, 40.0);
		Verdict v = Judge(run, cfg, true);
		snprintf(detail, sizeof detail, "slides %d at %.1f s (%.1f cm), attributed %d, log lines %zu",
			v.slides, v.firstEventTime, v.lastEventMagnitude * 100.0, v.attributed, run.events.size());
		check("lighthouse model: a slide with four stations in view stays drift evidence",
			v.slides >= 1 && v.losses == 0 && v.attributed == 0 && run.events.size() == 5, detail);
	}

	// The same slide while the tracker is down to one station is the
	// single-baseline swim: the monitor still sees a slide, the visibility
	// state explains it.
	{
		Output run = Run(cfg, StillAt(home), { { 0.0, all }, { 20.0, { 9 } }, { 45.0, all } }, 60.0);
		Verdict withLines = Judge(run, cfg, true);
		Verdict without = Judge(run, cfg, false);
		snprintf(detail, sizeof detail,
			"slides %d at %.1f s (%.1f cm), attributed %d; without the log %d slide(s) counted",
			withLines.slides, withLines.firstEventTime, withLines.lastEventMagnitude * 100.0,
			withLines.attributed, without.slides - without.attributed);
		// The swim keeps the monitor raising slides for as long as the single
		// station lasts; every one of them is the station's, not drift.
		check("lighthouse model: a single-station swim is attributed to the station",
			withLines.slides >= 2 && withLines.attributed == withLines.slides + withLines.losses &&
			withLines.firstEventTime > 20.0 && withLines.firstEventTime < 30.0 &&
			without.slides == withLines.slides && without.attributed == 0, detail);
	}

	// A short blackout during a quick reposition: the pose reappears far from
	// the straight-line prediction, which the monitor scores as a
	// re-localization; the lines say the tracker had no station.
	{
		std::vector<VisibilityChange> schedule = { { 0.0, all }, { 30.0, {} }, { 30.6, all } };
		Output run = Run(cfg, MoveBetween(home, home + Eigen::Vector3d(0.4, 0, 0), 30.2, 0.3), schedule, 45.0);
		Verdict v = Judge(run, cfg, true);
		int invalid = 0;
		for (const auto &f : run.frames)
			if (!f.sample.poseIsValid)
				invalid++;
		snprintf(detail, sizeof detail, "losses %d at %.2f s (%.0f cm off), attributed %d, invalid frames %d",
			v.losses, v.firstEventTime, v.lastEventMagnitude * 100.0, v.attributed, invalid);
		check("lighthouse model: a blackout recovery is attributed to the lost stations",
			v.losses == 1 && v.slides == 0 && v.attributed == 1 && v.lastEventMagnitude > 0.25 &&
			invalid > 0, detail);
	}

	// Lines replayed at startup describe state, not timing. A tracker that
	// swam on one station, then came back to four shortly before a genuine
	// slide: the swim's slide is attributed either way (the degraded state
	// is real whenever it was learned), but only the live return opens the
	// window that also attributes the genuine slide right after it.
	{
		std::vector<VisibilityChange> schedule = { { 0.0, all }, { 20.0, { 9 } }, { 25.0, all } };
		Truth truth = DriftFrom(home, Eigen::Vector3d(0, 0, 0.003), 26.0);
		Verdict live = Judge(Run(cfg, truth, schedule, 45.0), cfg, true);
		Verdict replayed = Judge(Run(cfg, truth, schedule, 45.0), cfg, true, true);
		snprintf(detail, sizeof detail, "live: %d slide(s) from %.1f s, %d attributed; replayed: %d slide(s), %d attributed",
			live.slides, live.firstEventTime, live.attributed, replayed.slides, replayed.attributed);
		check("lighthouse model: replayed history sets state but never a window",
			live.slides >= 2 && replayed.slides == live.slides &&
			replayed.attributed >= 1 && live.attributed == replayed.attributed + 1, detail);
	}

	// The log lines the model writes are the ones the parser produces from
	// the real driver, so the two halves cannot drift apart unnoticed.
	{
		Output run = Run(cfg, StillAt(home), { { 0.0, { 5, 8 } }, { 10.0, {} }, { 20.0, { 9 } } }, 25.0);
		std::vector<Event::Kind> kinds;
		kinds.reserve(run.events.size());
		for (const auto &te : run.events)
			kinds.push_back(te.event.kind);
		using K = Event::Kind;
		std::vector<K> expected = { K::Bootstrapped, K::StationAdded, K::StationAdded,
			K::StationDropped, K::StationDropped, K::NoneSeen, K::Bootstrapped, K::StationAdded };
		bool historicalPrefix = run.events.size() >= 3 && run.events[0].event.historical &&
			run.events[2].event.historical && !run.events[3].event.historical;
		snprintf(detail, sizeof detail, "%zu lines", run.events.size());
		check("lighthouse model: a visibility change writes the driver's line sequence",
			kinds == expected && historicalPrefix && run.events[4].event.visibleChannels.empty() &&
			Same(run.events[7].event.visibleChannels, { 9 }), detail);
	}
}

} // namespace

void RunLighthouseScenarios(void (*check)(const char *, bool, const char *))
{
	ParserScenarios(check);
	TailerScenarios(check);
	VisibilityScenarios(check);
	DigestScenarios(check);
	ModelScenarios(check);
}
