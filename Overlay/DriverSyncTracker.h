#pragma once

#include <cstdint>

namespace questcal
{

// Overlay-side bookkeeping for the asynchronous driver sync: which submission
// is current, and whether the driver's last verdict still stands.
//
// The identities the runtime monitors steer by are derived on the submitting
// thread from the same devices the worker ships (DeriveDriverSlotState), so a
// sync in flight is not a reason to disable anything: a submitted state is
// applied optimistically and only a completed refusal withdraws it. That
// refusal then holds across resubmissions of the identical state -- the scan
// resubmits the same state every second, and re-deriving "enabled" from the
// gates alone flipped a refused profile back on for one round trip each time.
// A changed state lifts the hold: it is a different question, and the driver
// has not answered it yet.
struct DriverSyncTracker
{
	uint64_t latestSequence = 0;
	uint64_t latestStateChangeSequence = 0;
	uint64_t lastAcceptedVerdictSequence = 0;
	bool lastVerdictRefused = false;
	bool stateChangedSinceVerdict = false;

	// Records a submission. Returns true when the profile must stay disabled
	// because the driver refused this same state and has not been asked
	// anything different since.
	bool NoteSubmission(uint64_t sequence, bool stateChanged)
	{
		latestSequence = sequence;
		if (stateChanged)
		{
			latestStateChangeSequence = sequence;
			stateChangedSinceVerdict = true;
		}
		return HoldsRefusal();
	}

	bool IsLatest(uint64_t sequence) const
	{
		return sequence == latestSequence;
	}

	// Records the driver's verdict. Periodic retries of the same desired state
	// do not make an older completion stale: a slow or timing-out transport can
	// take longer than the retry interval, and rejecting every such completion
	// would prevent the caller from ever observing the driver's verdict. Only a
	// completion older than the latest actual state change, a duplicate, or a
	// future sequence is ignored.
	bool NoteVerdict(uint64_t sequence, bool synchronized)
	{
		if (sequence > latestSequence ||
			sequence < latestStateChangeSequence ||
			sequence <= lastAcceptedVerdictSequence)
			return false;
		lastAcceptedVerdictSequence = sequence;
		lastVerdictRefused = !synchronized;
		stateChangedSinceVerdict = false;
		return true;
	}

	bool HoldsRefusal() const
	{
		return lastVerdictRefused && !stateChangedSinceVerdict;
	}
};

}
