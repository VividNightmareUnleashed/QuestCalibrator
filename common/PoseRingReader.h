#pragma once

// The overlay's side of the pose ring (PoseRing.h): the pose hub drains it.

#include "PoseRing.h"

#include <utility>

namespace protocol
{
	class PoseRingReader
	{
	public:
		enum class DrainStatus
		{
			Drained,
			ResetInProgress,
			WriterDead
		};

		~PoseRingReader() { Close(); }

		bool Open(const char *name)
		{
			return OpenImpl(name, []() { }, []() { });
		}

#ifdef QUESTCAL_POSE_CHANNEL_TEST_SEAM
		template<typename B, typename A>
		bool OpenWithGateHooksForTest(const char *name, B beforeRelease, A afterRelease)
		{
			return OpenImpl(name, beforeRelease, afterRelease);
		}

		bool ResetInProgressForTest() const
		{
			return ring && ring->resetting.load(std::memory_order_seq_cst) != 0;
		}

		void SetResetInProgressForTest(bool inProgress)
		{
			if (ring)
				ring->resetting.store(inProgress ? 1u : 0u, std::memory_order_seq_cst);
		}

		// Runs `beforeCas` inside the terminal-gap check, after both empty-queue
		// observations and immediately before the marker compare-exchange.
		template<typename F, typename G, typename H>
		DrainStatus DrainWithTerminalGapHookForTest(F &&fn, G &&gapFn, H beforeCas)
		{
			return DrainImpl(fn, gapFn, beforeCas);
		}
#endif

		void Close()
		{
			if (writerProcess) { CloseHandle(writerProcess); writerProcess = nullptr; }
			observedWriterProcessId = 0;
			observedWriterProcessCreationTime = 0;
			if (ring) { UnmapViewOfFile(ring); ring = nullptr; }
			if (hMap) { CloseHandle(hMap); hMap = nullptr; }
		}

		bool IsOpen() const { return ring != nullptr; }

	private:
		bool WriterAlive()
		{
			if (ring->writerActive.load(std::memory_order_acquire) == 0)
				return false;
			DWORD processId = ring->writerProcessId.load(std::memory_order_acquire);
			uint64_t processCreationTime =
				ring->writerProcessCreationTime.load(std::memory_order_acquire);
			if (processId == 0 || processCreationTime == 0)
				return false;

			if (writerProcess == nullptr || observedWriterProcessId != processId ||
				observedWriterProcessCreationTime != processCreationTime)
			{
				if (writerProcess)
					CloseHandle(writerProcess);
				writerProcess = OpenProcess(
					SYNCHRONIZE | PROCESS_QUERY_LIMITED_INFORMATION, FALSE, processId);
				observedWriterProcessId = processId;
				observedWriterProcessCreationTime = processCreationTime;
				if (writerProcess == nullptr)
					return false;
				uint64_t actualCreationTime = 0;
				if (!QueryProcessCreationTime(writerProcess, actualCreationTime) ||
					actualCreationTime != processCreationTime)
				{
					CloseHandle(writerProcess);
					writerProcess = nullptr;
					return false;
				}
			}

			DWORD waitResult = WaitForSingleObject(writerProcess, 0);
			return waitResult == WAIT_TIMEOUT &&
				ring->writerActive.load(std::memory_order_acquire) != 0 &&
				ring->writerProcessId.load(std::memory_order_acquire) == processId &&
				ring->writerProcessCreationTime.load(std::memory_order_acquire) ==
					processCreationTime;
		}

	public:
		uint64_t SessionEpoch() const
		{
			return ring ? ring->sessionEpoch.load(std::memory_order_acquire) : 0;
		}

		// Deliver every completed sample in queue order. A producer may safely
		// discard the oldest item only before this reader claims it.
		template<typename F>
		DrainStatus Drain(F &&fn)
		{
			return Drain(std::forward<F>(fn), [](uint64_t) { });
		}

		// `gapFn` runs at the exact position of source loss, before the first
		// surviving sample after it. This matters when a full queue is blocked
		// by an in-flight head: failed tail publishes occur while an older prefix
		// is still readable and must not reset consumers before that prefix.
		template<typename F, typename G>
		DrainStatus Drain(F &&fn, G &&gapFn)
		{
			return DrainImpl(fn, gapFn, []() { });
		}

	private:
		template<typename F, typename G, typename H>
		DrainStatus DrainImpl(F &fn, G &gapFn, H beforeTerminalCas)
		{
			if (ring == nullptr || !WriterAlive())
				return DrainStatus::WriterDead;
			if (!BeginRead())
				return DrainStatus::ResetInProgress;
			struct ReadGuard
			{
				std::atomic<uint32_t> &count;
				~ReadGuard() { count.fetch_sub(1, std::memory_order_seq_cst); }
			} readGuard{ ring->activeReaders };
			uint64_t currentEpoch = ring->sessionEpoch.load(std::memory_order_acquire);
			if (currentEpoch != observedSessionEpoch)
			{
				uint64_t dequeuePosition = 0;
				uint64_t discardedLosses = 0;
				if (!TrySnapshotDiscardState(dequeuePosition, discardedLosses))
					return DrainStatus::Drained;
				(void)dequeuePosition;
				observedSessionEpoch = currentEpoch;
				observedDiscardedLossCount = discardedLosses;
			}

			for (;;)
			{
				uint64_t pos = 0;
				uint64_t discardedLosses = 0;
				if (!TrySnapshotDiscardState(pos, discardedLosses))
					return DrainStatus::Drained;
				if (discardedLosses != observedDiscardedLossCount)
				{
					gapFn(discardedLosses - observedDiscardedLossCount);
					observedDiscardedLossCount = discardedLosses;
				}
				auto &slot = ring->slots[pos & (PoseRing::Capacity - 1)];
				uint64_t seq = slot.seq.load(std::memory_order_acquire);
				int difference = questcal_pose_sequence_relation(seq, pos + 1);
				if (difference < 0)
				{
					EmitTerminalGapIfEmpty(gapFn, beforeTerminalCas);
					break;   // empty, or the head producer has not published yet
				}
				if (difference > 0)
					continue;
				if (!ring->dequeuePos.compare_exchange_weak(pos, pos + 1,
					std::memory_order_relaxed, std::memory_order_relaxed))
					continue;

				DevicePoseSample copy = slot.sample;
				uint64_t failedBefore = slot.failedDropsBefore;
				slot.seq.store(pos + PoseRing::Capacity, std::memory_order_release);
				if (failedBefore != 0)
					gapFn(failedBefore);
				fn(copy);
			}
			return DrainStatus::Drained;
		}

		template<typename B, typename A>
		bool OpenImpl(const char *name, B beforeRelease, A afterRelease)
		{
			hMap = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, name);
			if (hMap == nullptr)
				return false;

			ring = static_cast<PoseRing *>(MapViewOfFile(
				hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(PoseRing)));
			if (ring == nullptr)
			{
				CloseHandle(hMap);
				hMap = nullptr;
				return false;
			}
			if (!RingHasExpectedLayout(ring))
			{
				Close();
				return false;
			}
			if (!WriterAlive() || !BeginRead())
			{
				Close();
				return false;
			}
			uint64_t dequeuePosition = 0;
			uint64_t discardedLosses = 0;
			bool snapshotRead = TrySnapshotDiscardState(dequeuePosition, discardedLosses);
			uint64_t sessionEpoch = ring->sessionEpoch.load(std::memory_order_acquire);
			beforeRelease();
			ring->activeReaders.fetch_sub(1, std::memory_order_seq_cst);
			afterRelease();
			if (!snapshotRead)
			{
				Close();
				return false;
			}
			(void)dequeuePosition; // reading it is part of the coherent seqlock snapshot
			observedSessionEpoch = sessionEpoch;
			observedDiscardedLossCount = discardedLosses;
			return true;
		}

		bool TrySnapshotDiscardState(uint64_t &dequeuePosition,
			uint64_t &discardedLosses) const
		{
			for (int attempt = 0; attempt < 64; ++attempt)
			{
				uint64_t before = ring->discardSequence.load(std::memory_order_acquire);
				if ((before & 1) != 0)
				{
					YieldProcessor();
					continue;
				}
				dequeuePosition = ring->dequeuePos.load(std::memory_order_acquire);
				discardedLosses = ring->discardedLossCount.load(std::memory_order_acquire);
				uint64_t after = ring->discardSequence.load(std::memory_order_acquire);
				if (before == after)
					return true;
				YieldProcessor();
			}
			return false;
		}

		// Reports failed publishes left on an empty queue without taking the
		// producer claim lock: a descheduled GUI-process reader holding it would
		// make every vrserver pose thread drop. The same empty queue on both sides
		// of the marker load proves no claim completed in between; a producer that
		// claims after that takes the markers into its slot, and the harvest
		// generation (see PendingDropCount) makes the exchange below fail.
		template<typename G, typename H>
		void EmitTerminalGapIfEmpty(G &gapFn, H &beforeCas)
		{
			uint64_t emptyAt = ring->dequeuePos.load(std::memory_order_acquire);
			if (ring->enqueuePos.load(std::memory_order_acquire) != emptyAt)
				return;
			uint64_t word = ring->pendingFailedDrops.load(std::memory_order_acquire);
			uint64_t drops = PendingDropCount(word);
			if (drops == 0)
				return;
			if (ring->dequeuePos.load(std::memory_order_acquire) != emptyAt ||
				ring->enqueuePos.load(std::memory_order_acquire) != emptyAt)
				return;   // a sample was published; its slot carries the markers
			beforeCas();
			if (!ring->pendingFailedDrops.compare_exchange_strong(word,
				PendingDropsAfterHarvest(word),
				std::memory_order_acq_rel, std::memory_order_relaxed))
				return;   // a producer harvested them, or more arrived; retry next drain
			gapFn(drops);
		}

		bool BeginRead()
		{
			if (ring->resetting.load(std::memory_order_seq_cst) != 0)
				return false;
			ring->activeReaders.fetch_add(1, std::memory_order_seq_cst);
			if (ring->resetting.load(std::memory_order_seq_cst) == 0)
				return true;
			ring->activeReaders.fetch_sub(1, std::memory_order_seq_cst);
			return false;
		}

		HANDLE hMap = nullptr;
		HANDLE writerProcess = nullptr;
		PoseRing *ring = nullptr;
		uint32_t observedWriterProcessId = 0;
		uint64_t observedWriterProcessCreationTime = 0;
		uint64_t observedSessionEpoch = 0;
		uint64_t observedDiscardedLossCount = 0;
	};
}
