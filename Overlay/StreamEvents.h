#pragma once

// What happened to the pose stream or to the calibration that the windows
// built on the stream must hear about, and which of them each event drops.
// The runtime monitors and the continuous loop keep everything an event does
// not list, so a new event is one row here, decided once.

#include "ContinuousAlignment.h"

namespace questcal
{

enum class StreamEvent
{
	// One or two poses lost from the monitors' drain (the driver's contended
	// publishes): ridden through, since every window pairs by sample time.
	MonitorDrop,
	// A hole too long to ride through, with no session boundary in it.
	MonitorHole,
	// The monitors' drain crossed a driver session boundary.
	SessionBoundary,
	// The per-tracker frame corrections started over (trackerFrameEpoch).
	TrackerFramesReset,
	// The monitors stopped (profile off, ring closed, frames recovering) or a
	// manual calibration is collecting.
	MonitorsParked,
	Calibrating,
	// The monitors start again after being parked.
	MonitorsResumed,
	// A headset re-center was compensated.
	UniverseJump,
	// A lighthouse frame move was applied to a tracker.
	FrameMoved,
	// A calibration finished and was applied.
	Recalibrated,
};

struct StreamResets
{
	bool universeObservations = false;   // the jump detector's windows
	bool universeHoleNote = false;       // only a note that a few poses went missing
	bool drift = false;                  // the drift monitor's windows
	bool monitorObservations = false;    // per-device watermarks and the HMD's raw position
	bool frameWatch = false;             // each device's last lighthouse frame
	bool frameProgress = false;          // how far the frame watch has examined each device
	bool continuous = false;             // the continuous loop's windows and correction gate
	ContinuousAlignment::ResetReason continuousReason = ContinuousAlignment::ResetReason::Suspended;
};

// The keep/drop table.
//  - A drop or a hole costs the monitors' windows at most: the frame watch
//    judges each device across it by its own samples, as it does a pause, and
//    the continuous loop judges its own drain.
//  - Only a new driver session, or frame corrections starting over, forgets
//    each device's last lighthouse frame: a station SteamVR re-solved while
//    the monitors were parked is still found from the samples on either side.
//  - A compensated jump moved the raw stream under the drift and continuous
//    windows; it was corrected, so they refill instead of reading it as
//    drift. A frame move or a new calibration does the same for drift only.
inline StreamResets ResetsFor(StreamEvent event)
{
	StreamResets r;
	switch (event)
	{
	case StreamEvent::MonitorDrop:
		r.universeHoleNote = true;
		break;
	case StreamEvent::MonitorHole:
	case StreamEvent::MonitorsParked:
	case StreamEvent::Calibrating:
		r.universeObservations = true;
		r.drift = true;
		r.monitorObservations = true;
		break;
	case StreamEvent::SessionBoundary:
		r.universeObservations = true;
		r.drift = true;
		r.monitorObservations = true;
		r.frameWatch = true;
		r.frameProgress = true;
		break;
	case StreamEvent::TrackerFramesReset:
		r.frameWatch = true;
		r.frameProgress = true;
		r.continuous = true;
		r.continuousReason = ContinuousAlignment::ResetReason::Suspended;
		break;
	case StreamEvent::MonitorsResumed:
		r.monitorObservations = true;
		break;
	case StreamEvent::UniverseJump:
		r.drift = true;
		r.continuous = true;
		r.continuousReason = ContinuousAlignment::ResetReason::UniverseJump;
		break;
	case StreamEvent::FrameMoved:
	case StreamEvent::Recalibrated:
		r.drift = true;
		break;
	}
	return r;
}

} // namespace questcal
