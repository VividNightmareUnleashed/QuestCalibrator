#include "../Overlay/StreamEvents.h"

#include <cstdio>

// The keep/drop rules in StreamEvents.h, each checked over every event, so a
// new event that breaks one fails here rather than in a session.
namespace
{
using Check = void (*)(const char *, bool, const char *);
using questcal::StreamEvent;
using Reason = questcal::ContinuousAlignment::ResetReason;

const StreamEvent AllEvents[] = {
	StreamEvent::MonitorDrop,
	StreamEvent::MonitorHole,
	StreamEvent::SessionBoundary,
	StreamEvent::TrackerFramesReset,
	StreamEvent::MonitorsParked,
	StreamEvent::Calibrating,
	StreamEvent::MonitorsResumed,
	StreamEvent::UniverseJump,
	StreamEvent::FrameMoved,
	StreamEvent::Recalibrated,
};

void RunFrameWatchRule(Check check)
{
	// Only a new driver session, or frame corrections starting over, forgets
	// each device's last lighthouse frame; a pause, a hole or a calibration
	// keeps it (a station re-solved meanwhile must still reach its trackers).
	// How far the watch has examined each device goes with the frames.
	int broken = 0;
	for (StreamEvent event : AllEvents)
	{
		const questcal::StreamResets r = questcal::ResetsFor(event);
		const bool forgets = event == StreamEvent::SessionBoundary ||
			event == StreamEvent::TrackerFramesReset;
		broken += (r.frameWatch != forgets || r.frameProgress != forgets) ? 1 : 0;
	}
	char detail[64];
	snprintf(detail, sizeof detail, "%d event(s) break the rule", broken);
	check("stream events: only a new session or new frame corrections forget the frames",
		broken == 0, detail);
}

void RunRideThroughRule(Check check)
{
	// A ridden-through drop only notes the hole for the jump detector. Any
	// event that drops the jump detector's windows drops the drift monitor's
	// and the watermarks with them: they read the same stream.
	const questcal::StreamResets drop = questcal::ResetsFor(StreamEvent::MonitorDrop);
	const bool dropKeeps = drop.universeHoleNote && !drop.universeObservations && !drop.drift &&
		!drop.monitorObservations && !drop.frameWatch && !drop.frameProgress && !drop.continuous;
	int apart = 0;
	for (StreamEvent event : AllEvents)
	{
		const questcal::StreamResets r = questcal::ResetsFor(event);
		apart += (r.universeObservations && !(r.drift && r.monitorObservations)) ? 1 : 0;
	}
	char detail[64];
	snprintf(detail, sizeof detail, "drop keeps %d, events apart %d", dropKeeps, apart);
	check("stream events: a ridden-through drop drops nothing else", dropKeeps && apart == 0, detail);
}

void RunContinuousRule(Check check)
{
	// The continuous loop judges holes and sessions from its own drain. The
	// monitors' events reach it only when what it measures moved: frame
	// corrections starting over, and a compensated jump, each with its reason.
	int broken = 0;
	for (StreamEvent event : AllEvents)
	{
		const bool reaches = event == StreamEvent::TrackerFramesReset ||
			event == StreamEvent::UniverseJump;
		broken += questcal::ResetsFor(event).continuous != reaches ? 1 : 0;
	}
	const bool reasons =
		questcal::ResetsFor(StreamEvent::TrackerFramesReset).continuousReason == Reason::Suspended &&
		questcal::ResetsFor(StreamEvent::UniverseJump).continuousReason == Reason::UniverseJump;
	char detail[64];
	snprintf(detail, sizeof detail, "%d event(s) break the rule, reasons %d", broken, reasons);
	check("stream events: the continuous loop hears only what moved its measurements",
		broken == 0 && reasons, detail);
}
}

void RunStreamEventScenarios(Check check)
{
	RunFrameWatchRule(check);
	RunRideThroughRule(check);
	RunContinuousRule(check);
}
