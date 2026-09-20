// Base station visibility: the lighthouse driver's log lines, the tailer
// that follows them, the per-device state folded from them, and the drift
// monitor's behaviour on a simulated tracker whose stations come and go.
#include "VirtualLighthouse.h"
#include "../Overlay/DriftMonitor.h"
#include "../Overlay/LighthouseLog.h"
#include "../Overlay/LighthouseVisibility.h"

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
#ifdef _WIN32
	localtime_s(&local, &whole);
#else
	localtime_r(&whole, &local);
#endif
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
	e.visibleKnown = kind != Event::Kind::Bootstrapped && kind != Event::Kind::BootstrapFailed;
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
	std::string back = vis.Apply(Made(K::StationAdded, "LHR-1", 8, { 5, 8 }), 230.0);
	check("lighthouse state: the return to two stations disturbs for the window only",
		back.find("back to 2 stations") == 0 && !vis.Find("LHR-1")->degraded &&
		vis.Disturbed("LHR-1", 235.0) && vis.Disturbed("LHR-1", 240.0) &&
		!vis.Disturbed("LHR-1", 241.0), back.c_str());

	// Full loss and bootstrap.
	std::string lost = vis.Apply(Made(K::StationDropped, "LHR-1", 8, { 5 }), 300.0);
	std::string last = vis.Apply(Made(K::StationDropped, "LHR-1", 5, {}), 300.2);
	std::string none = vis.Apply(Made(K::NoneSeen, "LHR-1", -1, {}), 300.3);
	std::string boot = vis.Apply(Made(K::Bootstrapped, "LHR-1", 8, {}), 309.0);
	std::string again = vis.Apply(Made(K::StationAdded, "LHR-1", 8, { 8 }), 309.1);
	const bool singleAfterBoot = vis.Find("LHR-1")->degraded && vis.Disturbed("LHR-1", 311.0);
	std::string second = vis.Apply(Made(K::StationAdded, "LHR-1", 5, { 5, 8 }), 312.0);
	one = vis.Find("LHR-1");
	snprintf(detail, sizeof detail, "losses %u bootstraps %u drops %u notes '%s' / '%s' / '%s' / '%s'",
		one->losses, one->bootstraps, one->drops, last.c_str(), boot.c_str(), again.c_str(),
		second.c_str());
	check("lighthouse state: a full loss counts once and every recovery line disturbs",
		last.find("lost its last station") == 0 && !none.empty() && one->losses == 1 &&
		boot.find("started a new solution") == 0 && again.find("tracking again") == 0 &&
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

	vis.Reset();
	check("lighthouse state: reset forgets everything",
		vis.Devices().empty() && vis.StationCount() == 0, "");
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
