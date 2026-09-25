#pragma once

#include <cstdint>

namespace questcal
{

// Overlay-side bookkeeping for the asynchronous driver sync: which submission
// is current, and whether the driver's last verdict still stands. A submitted
// state is applied optimistically; a completed refusal withdraws it and holds
// across resubmissions of the identical state until a changed state is sent.
// Submissions and verdicts are noted on one thread, and the single worker
// completes submissions in order.
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

	// Records the driver's verdict. Only a completion older than the latest
	// state change is ignored: a retry of the same state does not make an older
	// completion stale, or a transport slower than the retry interval would
	// never deliver a verdict.
	bool NoteVerdict(uint64_t sequence, bool synchronized)
	{
		if (sequence < latestStateChangeSequence)
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
