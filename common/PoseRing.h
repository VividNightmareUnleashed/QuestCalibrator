#pragma once

// The pose ring both processes map: its layout, the checks that a mapping
// has it, and the writer liveness proof. The driver writes it
// (PoseRingWriter.h) and the overlay drains it (PoseRingReader.h); neither
// side compiles the other's half.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <atomic>
#include <cstdint>

#include "Protocol.h"
#include "PoseRingCounters.h"

namespace protocol
{
	inline bool QueryProcessCreationTime(HANDLE process, uint64_t &creationTime)
	{
		FILETIME creation{};
		FILETIME exit{};
		FILETIME kernel{};
		FILETIME user{};
		if (!GetProcessTimes(process, &creation, &exit, &kernel, &user))
			return false;
		ULARGE_INTEGER value{};
		value.LowPart = creation.dwLowDateTime;
		value.HighPart = creation.dwHighDateTime;
		creationTime = value.QuadPart;
		return creationTime != 0;
	}

	// Lock-free pose ring in shared memory. The driver publishes every raw
	// driver-space pose it intercepts (pre-transform, QPC-stamped); the overlay
	// drains it to feed the calibration solver. Slots use the bounded MPMC
	// sequence protocol: a producer owns a slot until it publishes, and a
	// consumer owns it until it marks the slot reusable. That ownership is what
	// makes the ordinary DevicePoseSample copy race-free; sequence checks around
	// a concurrently-mutated payload are not sufficient under the C++ memory
	// model.
	//
	// vrserver invokes TrackedDevicePoseUpdated from each device driver's own
	// thread, so the writer side must be multi-producer safe. When the overlay
	// falls behind, producers safely discard the oldest completed slot before
	// retrying. If the oldest producer is itself still in flight, the new sample
	// is dropped instead of blocking a pose thread.
	struct PoseRing
	{
		// Capacity is a sample count, so the history it buys depends on the
		// aggregate publish rate. That rate is unmeasured; it is assumed here and
		// the margin is checked against it below.
		static const uint64_t AssumedDeviceCount = 5;
		static const uint64_t AssumedPerDevicePoseHz = 300;
		static const uint64_t AssumedAggregatePoseHz =
			AssumedDeviceCount * AssumedPerDevicePoseHz;
		// What the ring exists to survive: one full idle wait of the overlay's
		// drain loop without the readable prefix being overwritten.
		static const uint64_t DrainStallBudgetMs = 1000;

		static const uint64_t Capacity = 4096;   // power of two
		static const uint32_t Magic = 0x51435052; // "QCPR"
		// 4 gave pendingFailedDrops its harvest generation: same size, but a
		// layout-3 reader would read the generation as loss.
		static const uint32_t LayoutVersion = 4;

		struct Slot
		{
			std::atomic<uint64_t> seq;
			// Failed-tail samples immediately before this queue position. Producer
			// claims and pending-gap exchange share `claimLock`, making the marker
			// linearizable even with concurrent driver threads.
			uint64_t failedDropsBefore;
			DevicePoseSample sample;
		};

		uint32_t magic;
		uint32_t layoutVersion;
		uint64_t layoutBytes;
		std::atomic<uint32_t> initialized;
		// A new vrserver process resets an overlay-held mapping before reuse. The
		// reader gate makes that reset safe even if its drain thread is active.
		std::atomic<uint32_t> resetting;
		std::atomic<uint32_t> activeReaders;
		std::atomic<uint64_t> sessionEpoch;
		std::atomic<uint64_t> enqueuePos;
		std::atomic<uint64_t> dequeuePos;
		// Total positional loss removed from the readable prefix. This includes
		// both each overwritten sample and any failed-publish marker carried by
		// that sample. `discardSequence` makes the counter and dequeue position a
		// coherent snapshot for the reader.
		std::atomic<uint64_t> discardSequence;
		std::atomic<uint64_t> discardedLossCount;
		std::atomic<uint32_t> claimLock;
		// Failed publishes no slot carries yet (see PendingDropCount).
		std::atomic<uint64_t> pendingFailedDrops;
		// A heartbeat would incorrectly declare a writer dead while SteamVR is in
		// standby. The reader instead holds a process handle for this advertised
		// owner and can distinguish a quiet writer from a terminated vrserver.
		std::atomic<uint32_t> writerActive;
		std::atomic<uint32_t> writerProcessId;
		std::atomic<uint64_t> writerProcessCreationTime;
		Slot slots[Capacity];
	};

	static_assert((PoseRing::Capacity & (PoseRing::Capacity - 1)) == 0, "capacity must be a power of two");
	// 4096 samples is ~2.7 s at the assumed 1500 Hz but only ~0.68 s at 6 devices
	// x 1000 Hz. A larger Capacity changes sizeof(PoseRing), so it needs a
	// LayoutVersion bump and a new mapping name; measure the real rate first.
	static_assert(PoseRing::Capacity * 1000 >=
		PoseRing::DrainStallBudgetMs * PoseRing::AssumedAggregatePoseHz,
		"PoseRing::Capacity no longer covers DrainStallBudgetMs at the assumed aggregate pose rate");
	static_assert(ATOMIC_LLONG_LOCK_FREE == 2,
		"the shared pose queue requires lock-free 64-bit atomics");

	// True once the mapping is initialized with the layout this build expects.
	inline bool RingHasExpectedLayout(const PoseRing *ring)
	{
		return ring->initialized.load(std::memory_order_acquire) == 1 &&
			ring->magic == PoseRing::Magic &&
			ring->layoutVersion == PoseRing::LayoutVersion &&
			ring->layoutBytes == sizeof(PoseRing);
	}

	// The mapping name literal ends in ".layout4".
	static_assert(PoseRing::LayoutVersion == 4,
		"PoseRing::LayoutVersion changed - QUESTCALIBRATOR_SHMEM_NAME must change with it");

	// pendingFailedDrops holds (generation << 32) | count. A harvest replaces it
	// with the next generation and no count, so a word a reader loaded is still
	// current only if nothing was harvested since, even when later failures bring
	// the count back to the same value. The count cannot reach 2^32 between
	// harvests. At the conservative envelope of 64 devices * 1000 Hz this
	// requires a harvest within 18.6 hours of continuous publish failures.
	// Generation wrap (2^32 harvests) must not occur while a reader holds a
	// word for CAS. These are bounded-stall assumptions, not uptime proofs.
	inline uint64_t PendingDropCount(uint64_t word)
	{
		return questcal_pose_pending_drop_count(word);
	}

	inline uint64_t PendingDropsAfterHarvest(uint64_t word)
	{
		return questcal_pose_after_drop_harvest(word);
	}
}
