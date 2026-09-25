// PCH-free on purpose: SolverTests compiles this file standalone.
#include "LighthouseVisibility.h"

#include <algorithm>
#include <cstdio>

using lighthouselog::Event;

namespace
{

std::vector<int> Sorted(std::vector<int> channels)
{
	std::sort(channels.begin(), channels.end());
	channels.erase(std::unique(channels.begin(), channels.end()), channels.end());
	return channels;
}

// Within [-1 s, window] of a live event at `at` (a log line can be stamped
// just after the pose it explains); -1e9 marks no event yet.
bool Within(double at, double ringTime, double window)
{
	if (at <= -1e8)
		return false;
	const double since = ringTime - at;
	return since >= -1.0 && since <= window;
}

} // namespace

std::string LighthouseVisibility::IdName(uint32_t id)
{
	char buf[16];
	std::snprintf(buf, sizeof buf, "%08X", id);
	return buf;
}

std::string LighthouseVisibility::StationName(int channel) const
{
	char buf[32];
	auto it = stations.find(channel);
	if (it != stations.end() && it->second.id != 0)
		std::snprintf(buf, sizeof buf, "S-%d (%08X)", channel, it->second.id);
	else
		std::snprintf(buf, sizeof buf, "S-%d", channel);
	return buf;
}

std::string LighthouseVisibility::Apply(const Event &e, double ringTime)
{
	Device &d = devices[e.serial];
	d.serial = e.serial;
	d.events++;

	auto note = [&](int channel, uint32_t id)
	{
		if (channel < 0)
			return;
		Station &s = stations[channel];
		s.channel = channel;
		if (id != 0)
			s.id = id;
	};
	note(e.channel, e.stationId);
	for (size_t i = 0; i < e.visibleChannels.size(); ++i)
		note(e.visibleChannels[i], e.visibleIds[i]);

	// A station named only by id (bootstrap lines) still deserves a name.
	auto nameById = [&](uint32_t id)
	{
		for (const auto &kv : stations)
			if (kv.second.id == id && id != 0)
				return StationName(kv.first);
		return IdName(id);
	};

	const int before = d.visibleKnown ? static_cast<int>(d.visible.size()) : -1;
	std::string what;
	bool restart = false;
	switch (e.kind)
	{
	case Event::Kind::StationAdded:
		d.visible = Sorted(e.visibleChannels);
		d.visibleKnown = true;
		if (before == 0 || (before < 0 && d.visible.size() == 1))
		{
			what = "tracking again from " + StationName(e.channel);
			restart = true;
		}
		else if (before > 0 && before < config.cleanStations &&
			static_cast<int>(d.visible.size()) >= config.cleanStations)
			what = "back to " + std::to_string(d.visible.size()) + " stations with " +
				StationName(e.channel);
		break;

	case Event::Kind::StationDropped:
		stations[e.channel].drops++;
		d.drops++;
		d.visible = Sorted(e.visibleChannels);
		d.visibleKnown = true;
		if (d.visible.empty())
		{
			d.losses++;
			stations[e.channel].losses++;
			what = "lost its last station " + StationName(e.channel);
		}
		else if (static_cast<int>(d.visible.size()) < config.cleanStations)
			what = "down to one station " + StationName(d.visible.front()) +
				" after losing " + StationName(e.channel);
		break;

	case Event::Kind::NoneSeen:
		if (before != 0)
			d.losses++;
		d.visible.clear();
		d.visibleKnown = true;
		what = "sees no base station";
		break;

	case Event::Kind::Bootstrapped:
		d.bootstraps++;
		// The add lines that follow rebuild the set; until then it is
		// whatever the new solution starts from.
		d.visible.clear();
		d.visibleKnown = false;
		what = "started a new solution from " + nameById(e.stationId);
		restart = true;
		break;

	case Event::Kind::BootstrapFailed:
		what = "could not start tracking from " + nameById(e.stationId);
		break;
	}

	d.degraded = !d.visibleKnown ||
		static_cast<int>(d.visible.size()) < config.cleanStations;

	if (what.empty())
		return std::string();
	d.lastDisturbanceText = what;
	if (e.historical)
		return std::string();
	d.lastDisturbance = ringTime;
	d.liveDisturbances++;
	if (restart)
	{
		d.liveRestarts++;
		d.lastRestart = ringTime;
	}
	return what;
}

bool LighthouseVisibility::Settling(const std::string &serial, double ringTime) const
{
	const Device *d = Find(serial);
	if (!d)
		return false;
	if (d->visibleKnown && static_cast<int>(d->visible.size()) < config.cleanStations)
		return true;
	return Within(d->lastDisturbance, ringTime, config.disturbedSeconds);
}

bool LighthouseVisibility::RestartedWithin(const std::string &serial, double ringTime,
	double seconds) const
{
	const Device *d = Find(serial);
	return d && Within(d->lastRestart, ringTime, seconds);
}

bool LighthouseVisibility::Disturbed(const std::string &serial, double ringTime) const
{
	const Device *d = Find(serial);
	return d && (d->degraded || Within(d->lastDisturbance, ringTime, config.disturbedSeconds));
}

const LighthouseVisibility::Device *LighthouseVisibility::Find(const std::string &serial) const
{
	auto it = devices.find(serial);
	return it == devices.end() ? nullptr : &it->second;
}

std::vector<LighthouseVisibility::Station> LighthouseVisibility::Stations() const
{
	std::vector<Station> out;
	out.reserve(stations.size());
	for (const auto &kv : stations)
		out.push_back(kv.second);
	std::stable_sort(out.begin(), out.end(), [](const Station &a, const Station &b)
	{
		if (a.drops != b.drops)
			return a.drops > b.drops;
		return a.channel < b.channel;
	});
	return out;
}

void LighthouseVisibility::Reset()
{
	devices.clear();
	stations.clear();
}
