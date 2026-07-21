#pragma once

#include "../common/PoseChannel.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Continuously drains the driver's shmem pose ring on a dedicated thread and
// fans samples out to any number of consumers, each with its own cursor into
// the buffered history.
//
// The shmem ring itself only holds a second or two of slack before the
// reader's skip-ahead drops history, and the UI thread (which used to own the
// reader) can stall longer than that on a minimized window, blocking IPC, or
// a registry save. The hub's job is to keep the ring drained on a guaranteed
// cadence and hold a longer window locally so consumers never miss events
// (e.g. a universe jump) across a UI stall.
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
	// return means there is a gap immediately before out.front().
	uint64_t Drain(int consumer, std::vector<protocol::DevicePoseSample> &out);

	// Skip this consumer to now, discarding its backlog.
	void DiscardBacklog(int consumer);

private:
	void DrainLoop(std::string shmemName);

	std::thread drainThread;
	std::atomic<bool> stopRequested{ false };
	std::atomic<bool> ringOpen{ false };

	std::mutex mutex;                                   // guards history/head/cursors
	std::vector<protocol::DevicePoseSample> history;    // ring, HistoryCapacity entries
	uint64_t head = 0;                                  // absolute index of next write
	std::vector<uint64_t> cursors;                      // absolute per-consumer positions
};
