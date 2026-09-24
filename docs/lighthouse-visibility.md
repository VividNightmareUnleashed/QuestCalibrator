# Base station visibility

Which base station sees a tracker, and when it stops seeing it, is not in the
OpenVR API. SteamVR's lighthouse driver does write it to its own log, one line
per change, per device, as it happens. QuestCalibrator follows that log and
uses it in three places: a figure on every lighthouse device row, a
per-station drop count, and a rule that keeps a tracker's own tracking trouble
out of the drift evidence.

Everything here is read from SteamVR's log file. Nothing is read from the
headset, and nothing changes when the log is missing or unreadable.

## Where it comes from

`<Steam>\logs\vrserver.txt`, the Steam folder from the registry
(`HKEY_CURRENT_USER\Software\Valve\Steam\SteamPath`) with the default install
location as the fallback. The lines that matter, with the fixed prefix
`Fri Sep 11 2026 22:08:55.819 [Info] - lighthouse: ` removed:

```
LHR-A3C36EA5 C: SOB: add S-16 also seeing S-5 (D3D4E73B) S-8 (170EE067) S-9 (F210FBA6)
LHR-A3C36EA5 C: SOB: add S-5
LHR-A3C36EA5 C: SOB: add S-9 (generation changed) also seeing S-5 (D3D4E73B) S-16 ( 4D47FB4)
LHR-A3C36EA5 C: SOB: drop S-5 (D3D4E73B) seeing S-8 (170EE067) S-9 (F210FBA6) S-16 ( 4D47FB4)
LHR-A3C36EA5 C: SOB: drop S-8 seeing S-5 S-9 S-16
LHR-A3C36EA5 C: No base stations seen...
LHR-A3C36EA5 C: ----- BOOTSTRAPPED base F210FBA6 (best) distance 2.1m velocity 0.3m/s ... -----
LHR-A3C36EA5 C: Trying to start tracking from base D3D4E73B: Not enough contiguous samples for a bootstrap pose
```

- `LHR-…` is the device's serial, the same string OpenVR returns as its
  serial-number property. That is how a line is joined to a device index.
- `S-N` is a base station's channel. The hex in parentheses is the station's
  id, which is the serial SteamVR keeps for it in its lighthouse database
  (`D3D4E73B` is `3553945403` there). A leading zero is printed as a space.
- `SOB` is sync on beam, the base station 2.0 mode the driver selects at
  startup. Base station 1.0 setups have not been checked; if their lines
  differ they are simply not parsed.
- `add` and `drop` list the stations the device still sees after the change,
  so every line carries the full set. `No base stations seen` and
  `BOOTSTRAPPED` mark a full loss and a solution started from scratch.

The file is re-opened on every poll (four times a second) and never held, so
SteamVR can rename it at its next start. The first poll replays up to the
last 4 MB of the existing file to learn the current sets; those lines are
marked historical and only ever set the sets and the counters. A file that
shrank was rotated: SteamVR restarted, and the sets and counters start over.

## What is shown

- **On the Lighthouse tab:** one grid, lighthouse devices down the side and
  stations across the top in SteamVR's own art (the `NamedIconPath`
  properties). A cell is a filled dot while the device has the station in
  view, a red ring while it has lost it, and a faint ring when it is only
  out of view. A station counts as lost only after the device had it (the
  `lost` set on each device in `LighthouseVisibility`, fed by drop lines and
  by a set that empties); one it never had from where it stands is only out
  of view. A ring lost in the last four seconds glows behind it. Each row
  ends with the in-view figure (`3 of 4`, red below two), and a row with
  something to say names it under the device: `Down to one station`,
  `Lost S-16`, `Lost S-8 just now`, `Lost a station 7 times`, and
  `Off or out of range` for a device SteamVR has lost, which then shows no
  figure. The copy says "lost" for every loss, never "dropped". Rows with
  a problem come first, devices that are off last. Under each column is how
  many of the reporting devices see that station, red when none does: that
  is the one to look at. Hovering a row lists what it sees, its drops and
  the last change; hovering a station lists its id and drops. The
  Calibration tab's device rows carry none of this. `-uipreview-lighthouse`
  opens the tab in the preview.
- **Channels (same tab):** two stations whose `Prop_ModeLabel_String` is the
  same channel get in each other's way. The card names the channel, the
  station to move (the one the log is not naming on that channel) and the
  lowest channel from 1 to 16 that no listed or logged station uses, with
  where SteamVR changes it. The "Switch to channel" button is drawn but
  not built: no OpenVR call for it is known, and setting the channel over
  Bluetooth, as some third-party tools do, is still to come. A station
  that drops often is not a channel problem, so drops never lead to a
  channel suggestion. A neighbour's station on one of the player's channels
  is not detected yet.
- **3D View (same tab):** a card for the optional module that will show the
  room in 3D in a browser. Its button is drawn but not built. The station
  and device poses and the boundary that `LoadVRState` reads are there for
  that module; the tab itself no longer draws the room.
- **In the session log:** one line per disturbance (`LHR-A3C36EA5 down to one
  station S-9 (F210FBA6) after losing S-16 (04D47FB4)`), and, with detailed
  logging on, one line per routine handoff.
- **In a diagnostics export:** a `[base stations]` section with all of the
  above.

## Where the stations stand

A log channel is joined to an OpenVR base station by its id: the station's
serial is `LHB-` and the same eight hex digits (`LHB-D3D4E73B` for S-5
above, as vrserver's "finished adding tracked device" lines show). Until the
log has printed an id, the station's `Prop_ModeLabel_String` is compared with
the channel number instead.

Stations are lighthouse devices, so the driver moves them with the
calibration like any tracker, and their standing-universe poses land in the
headset's space next to the boundary. Until a calibration is applied they do
not, and anything drawing them with the boundary has to say so. The optical
axis is taken as the pose's -Z, the OpenVR convention; this has not yet been
checked against a physical station.

## What it changes

The drift monitor scores two things as evidence that the two universes have
moved apart: a stationary device that slides more than 12 mm across an 8 s
rest window, and a device that reappears after a short absence more than
25 cm from where it vanished. A lighthouse device does both on its own when
it loses stations: with a single station the optical solution has one
baseline and can swim along that station's line of sight; with none it coasts
on the IMU and then reports itself out of range; when a station returns the
solution snaps back.

So a drift-monitor event on a device is attributed to its base stations, and
not counted, while the device is **degraded** (fewer than two stations in
view, or no solution at all; a state, so lines replayed at startup set it
too) and for the ten seconds after a **disturbance** of that device (or up
to one second before one, since a log line can be stamped just after the pose
it explains). A disturbance is any of:

- the device fell to a single station, or to none;
- the device saw no station at all (`No base stations seen`);
- a solution was bootstrapped, or a bootstrap failed;
- the device came back from none or one station to two or more.

A station leaving or joining a solution that keeps two or more is counted but
is not a disturbance: the fit keeps a second baseline and the pose barely
moves. Attributed events are logged with the reason (`Device 7 slid 1.4 cm
after it down to one station S-9 (F210FBA6) ... -- its base stations, not
drift`) and summed in the diagnostics.

Cost: a genuine slide that happens to fall inside such a window is missed
once. In the session below, disturbances covered a few percent of any one
device's time.

Lighthouse devices are target-side devices, so none of this touches the
universe-jump detector, which only reads the reference system's stream.

## One session, in numbers

`vrserver.txt` from 2026-09-11, eight lighthouse devices, about eighty
minutes:

| Measure | Value |
| --- | --- |
| Drop and re-add pairs | 141 |
| Median time a station stayed lost | 9.4 s |
| Longest ten percent of losses | over 85 s |
| Drops on channel 16 (04D47FB4) | 59 |
| Drops on channel 5 (D3D4E73B) | 48 |
| Drops on channel 8 (170EE067) | 23 |
| Drops on channel 9 (F210FBA6) | 11 |

Most drops left three stations in view, which is why they never appeared as
tracking loss. `No base stations seen` appeared once.

## The simulated tracker

`Tests/VirtualLighthouse.h` generates the pose stream and the log lines of one
tracker whose stations come and go on a schedule, so the drift monitor and
the visibility state can be run together exactly as the overlay runs them.
The line sequence it writes for a change is the one the real driver writes
(drops one at a time listing what remains, `No base stations seen` when the
set empties, a bootstrap when it refills, adds one at a time), and the
scenarios check that the parser and the model agree.

Assumed, not measured, and named as such in its configuration: the
single-station swim (3 mm/s along the station's line of sight, capped at
3 cm), how long a coasting pose stays valid (0.2 s) and its bias
(0.3 m/s²). The scenarios only need those failures to exist, not their exact
size.

Scenarios (`RunLighthouseScenarios`):

- a slide with four stations in view stays drift evidence;
- the same slide while down to one station is raised by the monitor, every
  time it recurs, and attributed to the station; it would have counted
  without the log;
- a 0.6 s blackout during a 40 cm reposition is raised as a re-localization
  and attributed to the lost stations;
- lines replayed at startup set the state but open no window: a genuine
  slide just after a replayed return to four stations counts, the same slide
  after a live return is attributed;
- the parser accepts every line shape above and rejects the driver's other
  lines; the tailer replays history, follows appends, completes a half-written
  line once and survives a rotation.

## Limits

- An undocumented log format of a closed driver. The parser is tolerant and a
  changed format degrades to "no information", but it would go unnoticed until
  the figure stops appearing.
- Log lines are not synchronized to the pose stream. The ten-second window and
  the one-second tolerance cover the lag seen so far; per-frame gating is out
  of reach this way.
- The headset is not a lighthouse device and never appears.
- Whether SteamVR flushes the file promptly on every machine is not something
  the parser can check; the session log's first lighthouse line says whether
  anything is being read at all.
