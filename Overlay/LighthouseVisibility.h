#pragma once

// What each lighthouse-tracked device currently sees, folded from the
// driver's log lines (LighthouseLog.h), and which of those changes make a
// device's pose unfit as drift evidence for a while.
//
// A station leaving or joining a solution that keeps two or more stations
// barely moves the pose; the optical fit still has depth from two baselines.
// The changes that do move it are the ones that change the kind of solution:
// down to a single station (the fit loses its second baseline and the pose
// can swim along the remaining line of sight), no station at all (the pose
// coasts on the IMU, then goes out of range), and every recovery from those
// (the optical solution snaps back). The drift monitor scores a stationary
// device that slides or a device that reappears away from where it vanished
// as evidence that the two universes moved apart; a lighthouse device doing
// either of those right after such a change is reporting its own tracking,
// not the universes. Disturbed() answers that question for the monitor.
//
// Times are ring seconds (the overlay's QPC clock), converted by the caller
// from the wall-clock stamp on each log line. Lines replayed from the log at
// startup carry their state but no usable time; they set the visible sets
// and the counters and never count as a disturbance.

#include "LighthouseLog.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

class LighthouseVisibility
{
public:
	struct Config
	{
		// A drift-monitor event this long after a disturbance is attributed
		// to it. Covers the monitor's 8 s rest window plus the time the log
		// line can lag the pose it explains.
		double disturbedSeconds = 10.0;
		// Fewer visible stations than this is a single-baseline solution.
		int cleanStations = 2;
	};

	struct Station
	{
		uint32_t id = 0;        // 0 until a line names it
		int channel = -1;
		uint32_t drops = 0;     // times any device dropped it
		uint32_t losses = 0;    // times it was the last station a device lost
	};

	struct Device
	{
		std::string serial;
		bool visibleKnown = false;
		std::vector<int> visible;   // channels, ascending
		uint32_t events = 0;
		uint32_t drops = 0;         // stations dropped, any count remaining
		uint32_t losses = 0;        // times it saw no station at all
		uint32_t bootstraps = 0;    // solutions started from scratch
		// Fewer than cleanStations in view, or no solution at all: the pose
		// is a single-baseline fit or an IMU coast for as long as this holds.
		// State, not timing, so replayed lines set it too.
		bool degraded = false;
		double lastEvent = -1e9;    // ring seconds; live lines only
		double lastDisturbance = -1e9;
		std::string lastDisturbanceText;
	};

	LighthouseVisibility() = default;
	explicit LighthouseVisibility(const Config &c) : config(c) { }

	// Folds one line in. Returns a short description when the line is a
	// disturbance worth a session-log line ("lost its last station
	// (S-5 D3D4E73B)"), empty for routine handoffs and for replayed lines.
	std::string Apply(const lighthouselog::Event &e, double ringTime);

	// True while the device is degraded (fewer than cleanStations in view,
	// or no solution), and for config.disturbedSeconds after any live
	// disturbance, or up to a second before one (a log line can be stamped
	// just after the pose it explains).
	bool Disturbed(const std::string &serial, double ringTime) const;

	const Device *Find(const std::string &serial) const;
	const std::map<std::string, Device> &Devices() const { return devices; }
	std::vector<Station> Stations() const;   // most dropped first
	int StationCount() const { return static_cast<int>(stations.size()); }
	// "S-5 (D3D4E73B)", or "S-5" while the id is unknown.
	std::string StationName(int channel) const;
	static std::string IdName(uint32_t id);
	const Config &Settings() const { return config; }
	void Reset();

private:
	Config config;
	std::map<std::string, Device> devices;
	std::map<int, Station> stations;
};
