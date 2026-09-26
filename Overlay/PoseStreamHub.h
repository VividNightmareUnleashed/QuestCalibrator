#pragma once

#include "../common/PoseChannel.h"

#include <atomic>
#include <cstdint>
#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
#include <functional>
#endif
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Continuously drains the driver's shmem pose ring on a dedicated thread and
// fans samples out to any number of consumers, each with its own cursor into
// the buffered history.
//
// The shmem queue itself only holds a few seconds of slack before producers
// safely discard old poses, and the UI thread (which used to own the reader)
// can stall longer than that on a minimized window, blocking IPC, or a
// registry save. The hub's job is to keep it drained on a guaranteed cadence,
// propagate source-drop counts to every consumer, and hold a longer window
// locally so consumers do not bridge observation gaps across a UI stall.
class PoseStreamHub
{
public:
	// Power of two. At a typical aggregate pose rate (a few hundred Hz per
	// device, a handful of devices) this is tens of seconds of history.
	static const uint64_t HistoryCapacity = 1 << 15;

	~PoseStreamHub();

	void Start(const char *shmemName);
	void Stop();

	// Whether the underlying shmem ring is currently open (driver present).
	bool RingOpen() const { return ringOpen.load(std::memory_order_acquire); }

	// A consumer id is a private cursor. Samples buffered before the consumer
	// was created are not delivered to it.
	int CreateConsumer();

	// Fills `out` (cleared first) with every sample since this consumer's
	// cursor, in publish order. Returns how many samples this consumer lost
	// to history overflow since its last drain (0 = kept up); a non-zero
	// return means there is a gap immediately before out.front(), or a terminal
	// gap after the previously returned prefix when `out` is empty.
	//
	// One exception to the count: a driver session boundary (writer death or a
	// new session epoch) discards each consumer's whole un-drained backlog and
	// reports it as a single-count gap, not as the number of samples dropped.
	// Consumers must treat a non-zero return as "there is a hole here", not as
	// a loss rate.
	uint64_t Drain(int consumer, std::vector<protocol::DevicePoseSample> &out);

	// Skip this consumer to now, discarding its backlog.
	void DiscardBacklog(int consumer);

#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
	void AppendSampleForTest(const protocol::DevicePoseSample &sample);
	void AppendGapForTest(uint64_t count);
	void SetDrainChunkHookForTest(std::function<void()> hook);
	uint64_t ResetDeferralsForTest() const
	{
		return resetDeferralsForTest.load(std::memory_order_acquire);
	}
#endif

private:
	struct HistoryEntry
	{
		protocol::DevicePoseSample sample;
		uint64_t sourceDropCountBefore = 0;
		uint64_t sampleCountBefore = 0;
		bool hasSample = false;
	};
	struct ConsumerCursor
	{
		uint64_t historyPosition = 0;
		uint64_t samplePosition = 0;
		uint64_t sourceDropPosition = 0;
	};

	void AccountForHistoryOverflowLocked(int consumer, uint64_t &dropped);
	void AppendSampleLocked(const protocol::DevicePoseSample &sample);
	void AppendGapLocked(uint64_t count);
	// Publishes an observation hole: discards every consumer's backlog (it may
	// predate a universe rebase) and marks the position so the next drain
	// reports a gap rather than bridging it.
	void AppendSessionBoundaryLocked();
	// Thread entry: catches, so an allocation failure stops the drain instead of
	// terminating the process without unwinding.
	void DrainLoop(const std::string &shmemName);
	void DrainRing(const std::string &shmemName);

	std::thread drainThread;
	std::atomic<bool> stopRequested{ false };
	std::atomic<bool> ringOpen{ false };

	std::mutex mutex;                                   // guards all fields below
	std::vector<HistoryEntry> history;                  // ring, HistoryCapacity entries
	uint64_t head = 0;                                  // absolute index of next write
	uint64_t sampleCount = 0;                           // actual samples, excluding gap markers
	uint64_t sourceDropCount = 0;                       // cumulative source-gap sequence
	std::vector<ConsumerCursor> consumers;              // private per-consumer positions
#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
	std::function<void()> drainChunkHookForTest;
	std::atomic<uint64_t> resetDeferralsForTest{ 0 };
#endif
};
