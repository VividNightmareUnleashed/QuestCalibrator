#pragma once

// SteamVR's lighthouse driver writes one line to vrserver.txt every time a
// tracked device's set of visible base stations changes, and a few more when
// a device loses every station or starts tracking again. Nothing in the
// OpenVR API exposes this, so the log is the only root-free source of "which
// base station sees this tracker". This header parses those lines and follows
// the file as SteamVR appends to it.
//
// The lines are an undocumented format of a closed driver, observed on
// SteamVR 2.x with base station 2.0 in its sync-on-beam mode. The parser is
// tolerant (unknown lines are ignored, ids are optional) and every consumer
// treats a missing or unreadable log as "no information", never as an
// error. The shapes it understands, with the fixed prefix removed:
//
//   LHR-A3C36EA5 C: SOB: add S-16 also seeing S-5 (D3D4E73B) S-8 (170EE067)
//   LHR-A3C36EA5 C: SOB: add S-5
//   LHR-A3C36EA5 C: SOB: add S-9 (generation changed) also seeing S-5 (D3D4E73B)
//   LHR-A3C36EA5 C: SOB: drop S-5 (D3D4E73B) seeing S-8 (170EE067) S-16 ( 4D47FB4)
//   LHR-A3C36EA5 C: SOB: drop S-8 seeing S-5 S-9 S-16
//   LHR-A3C36EA5 C: No base stations seen...
//   LHR-A3C36EA5 C: ----- BOOTSTRAPPED base F210FBA6 (best) distance 2.1m ... -----
//   LHR-A3C36EA5 C: ----- SECONDARY base 4921060B distance 2.01m  -----
//   LHR-A3C36EA5 C: Trying to start tracking from base D3D4E73B: Not enough ...
//   Device LHR-A3C36EA5 powering off upon entering standby.
//   LHR-A3C36EA5: Disconnected from receiver 4BF089B604
//   LHR-A3C36EA5: Connected to receiver 4BF089B604
//   Selected existing universe 1744988537 (170EE067 is primary)
//   Creating new universe 1744988537 because there were no existing universes
//   Stopped tracking with universe 1744988537
//
// and, outside the lighthouse driver, the server's own first line
// ("Sun Sep 27 2026 21:43:33.604 [Info] - vrserver 2.17.10 startup with
// PID=5976, ..."). The universe lines name no device: the universe is the set
// of base station poses every lighthouse pose is reported in, which the
// driver chooses once the first devices have started tracking after SteamVR
// starts, and again after it stopped tracking with one because no device
// was left tracking. vrserver.txt is not always rotated at a start: one file
// held five days of sessions on 2026-09-27.
//
// Not every device prints the SOB lines. A VIVE Tracker 3.0 in a four-station
// room (live 2026-09-25) printed none in six hours; after each bootstrap it
// named the stations joining its new solution only in SECONDARY lines.
//
// The last three are the device's radio link. SteamVR switches a controller
// or tracker off once it has not moved for its "Turn off controllers after"
// time, 5 minutes unless changed; a headset tracker sits still whenever the
// headset is off, and stays off when it goes back on (live 2026-09-26: 305 s
// after the headset was set down).
//
// S-N is the station's channel; the hex in parentheses is the station id,
// which equals the serial SteamVR stores for it (a leading zero is printed
// as a space). The device is its LHR serial, the same string OpenVR returns
// as the serial-number property, so lines join onto device indices.

#include <cstdint>
#include <string>
#include <vector>

namespace lighthouselog
{

struct Event
{
	enum class Kind
	{
		StationAdded,      // a station joined the device's solution
		StationDropped,    // a station left it
		NoneSeen,          // the device sees no station at all
		Bootstrapped,      // a fresh solution started from one station
		BootstrapFailed,   // it tried to start one and could not
		SecondaryAdded,    // a station joined a solution after its bootstrap (id only)
		PoweredOff,        // SteamVR switched the device off (standby says why)
		Disconnected,      // its radio link to the receiver dropped
		Connected,         // its radio link came up
		// No device (serial empty):
		ServerStarted,     // vrserver started: no universe yet
		UniverseChosen,    // the driver selected or created its universe
		UniverseStopped,   // it stopped tracking with it
	};
	Kind kind = Kind::StationAdded;
	std::string serial;              // "LHR-A3C36EA5"; empty for the no-device kinds
	int channel = -1;                // S-N of the station the line is about; -1 when absent
	uint32_t stationId = 0;          // its id when the line carried one, else 0
	// The device's visible channels after this line, when the line lists
	// them (add and drop lines do; NoneSeen means empty).
	bool visibleKnown = false;
	std::vector<int> visibleChannels;
	std::vector<uint32_t> visibleIds;   // parallel to visibleChannels; 0 when unknown
	bool generationChanged = false;
	// PoweredOff: on entering standby, i.e. after sitting still, rather than
	// for a reason the line does not give.
	bool standby = false;
	// UniverseChosen and UniverseStopped: the universe's id; UniverseChosen
	// also says whether the driver created it rather than selected a known one.
	uint64_t universeId = 0;
	bool universeCreated = false;
	// Local wall-clock time printed on the line, as Unix seconds with the
	// millisecond part; timeKnown is false when the prefix did not parse.
	bool timeKnown = false;
	double unixTime = 0.0;
	// Set by the tailer for lines that were already in the file when it
	// started: they describe the current state but happened at an unknown
	// distance in the past.
	bool historical = false;
};

// True when the line is one of the shapes above; `out` is fully rewritten.
bool ParseLine(const std::string &line, Event &out);

// Parses the "Fri Sep 11 2026 22:08:55.819" prefix SteamVR puts on every
// line; returns false when it is not there.
bool ParseTimestamp(const std::string &line, double &unixTime);

// <Steam>\logs\vrserver.txt, with the Steam folder from the registry and
// the default install location as the fallback.
std::string DefaultLogPath();
// The same for a given Steam folder, which may use either slash and end in
// one; empty means the default install location.
std::string LogPathUnder(std::string steamFolder);

// Follows a growing log file by re-opening it on every poll, so no handle
// is held between polls and SteamVR can rename the file at its next start.
// The first poll replays the tail of the existing file (so the current
// visibility is known at once) with `historical` set; later polls return
// only what was appended. A file that shrank was rotated and is read from
// its start.
class Tailer
{
public:
	explicit Tailer(std::string path);

	void Poll(std::vector<Event> &out);

	const std::string &Path() const { return path; }
	bool Available() const { return available; }   // the file opened on the last poll
	uint64_t LinesSeen() const { return linesSeen; }
	uint64_t Rotations() const { return rotations; }

	// How much of an existing file the first poll replays, and the most one
	// poll reads, so a burst never stalls the caller's frame.
	static constexpr uint64_t ReplayBytes = 4u << 20;
	static constexpr uint64_t MaxBytesPerPoll = 1u << 20;

private:
	std::string path;
	std::string partial;         // an incomplete last line, kept for the next poll
	uint64_t offset = 0;         // next byte to read
	uint64_t replayEnd = 0;      // bytes below this were present at the first poll
	bool primed = false;
	bool skipToNextLine = false; // the replay started inside a line
	bool available = false;
	uint64_t linesSeen = 0;
	uint64_t rotations = 0;
};

} // namespace lighthouselog
