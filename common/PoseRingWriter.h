#pragma once

// The driver's side of the pose ring (PoseRing.h): vrserver's pose threads
// publish into it.

#include "PoseRing.h"

#include <new>
#include <string>

namespace protocol
{
	class PoseRingWriter
	{
	private:
		// One Create attempt blocks for at most this long on the reset mutex and,
		// separately, on the reader-drain gate. Init can afford the full budget;
		// vrserver's driver frame loop passes zero rather than stalling every
		// other driver's RunFrame behind a peer that holds either gate.
		static constexpr DWORD WriterResetMutexWaitMs = 2000;
		static constexpr DWORD ReaderDrainWaitMs = 1000;
		// A departing writer only needs the mutex long enough to retire its own
		// identity, and this runs on vrserver's shutdown path.
		static constexpr DWORD WriterRetireWaitMs = 200;

	public:
		~PoseRingWriter() { Close(); }

		bool Create(const char *name, DWORD waitBudgetMs = WriterResetMutexWaitMs)
		{
			if (ring != nullptr)
				return true;
			uint64_t processCreationTime = 0;
			if (!QueryProcessCreationTime(GetCurrentProcess(), processCreationTime))
				return false;

			// A named mutex, not the in-mapping `resetting` bit, owns creation and
			// reset: Windows abandons it when its owner dies, so the next writer can
			// take over, whereas a stale `resetting` bit cannot be proven stale.
			HANDLE resetMutex = AcquireWriterResetMutex(name, waitBudgetMs);
			if (resetMutex == nullptr)
				return false;
			struct ResetMutexGuard
			{
				HANDLE handle;
				~ResetMutexGuard()
				{
					// Preserve the operation's diagnostic across Release/Close.
					DWORD error = GetLastError();
					ReleaseMutex(handle);
					CloseHandle(handle);
					SetLastError(error);
				}
			} resetMutexGuard{ resetMutex };

			hMap = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, sizeof(PoseRing), name);
			if (hMap == nullptr)
				return false;
			const bool freshMapping = GetLastError() != ERROR_ALREADY_EXISTS;

			ring = static_cast<PoseRing *>(MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(PoseRing)));
			if (ring == nullptr)
			{
				CloseHandle(hMap);
				hMap = nullptr;
				return false;
			}

			if (freshMapping)
			{
				// A bounded sequence queue needs slot i to start with sequence i.
				// Placement construction also starts the C++ object lifetimes in the
				// freshly-created shared mapping; the OS already zeroed its storage.
				ring->magic = PoseRing::Magic;
				ring->layoutVersion = PoseRing::LayoutVersion;
				ring->layoutBytes = sizeof(PoseRing);
				new (&ring->initialized) std::atomic<uint32_t>(0);
				new (&ring->resetting) std::atomic<uint32_t>(1);
				new (&ring->activeReaders) std::atomic<uint32_t>(0);
				new (&ring->sessionEpoch) std::atomic<uint64_t>(1);
				new (&ring->enqueuePos) std::atomic<uint64_t>(0);
				new (&ring->dequeuePos) std::atomic<uint64_t>(0);
				new (&ring->discardSequence) std::atomic<uint64_t>(0);
				new (&ring->discardedLossCount) std::atomic<uint64_t>(0);
				new (&ring->claimLock) std::atomic<uint32_t>(0);
				new (&ring->pendingFailedDrops) std::atomic<uint64_t>(0);
				new (&ring->writerActive) std::atomic<uint32_t>(0);
				new (&ring->writerProcessId) std::atomic<uint32_t>(0);
				new (&ring->writerProcessCreationTime) std::atomic<uint64_t>(0);
				for (uint64_t i = 0; i < PoseRing::Capacity; ++i)
				{
					new (&ring->slots[i].seq) std::atomic<uint64_t>(i);
					ring->slots[i].failedDropsBefore = 0;
					new (&ring->slots[i].sample) DevicePoseSample();
				}
				ring->initialized.store(1, std::memory_order_release);
			}
			// Each code below is a different user action in the driver log, and
			// Close() may change the last error, so the code is set after it.
			else if (!HasExpectedLayout())
			{
				Close();
				SetLastError(ERROR_REVISION_MISMATCH);
				return false;
			}
			else if (ExistingWriterAlive())
			{
				Close();
				SetLastError(ERROR_ALREADY_EXISTS);
				return false;
			}
			else if (!ResetForNewWriterSession(waitBudgetMs < ReaderDrainWaitMs
				? waitBudgetMs : static_cast<DWORD>(ReaderDrainWaitMs)))
			{
				DWORD error = GetLastError();   // ERROR_BUSY: readers would not drain
				Close();
				SetLastError(error);
				return false;
			}

			ownedSessionEpoch = ring->sessionEpoch.load(std::memory_order_acquire);
			ring->writerProcessId.store(GetCurrentProcessId(), std::memory_order_relaxed);
			ring->writerProcessCreationTime.store(processCreationTime,
				std::memory_order_relaxed);
			ring->writerActive.store(1, std::memory_order_release);
			// Keep the reset gate held until the complete owner identity is visible.
			// This prevents a second writer from slipping between reset completion
			// and publication of writerActive.
			ring->resetting.store(0, std::memory_order_seq_cst);
			// Close retires this identity under the same mutex that published it
			// and has no mapping name of its own, so hold a second handle open for
			// the session. If it cannot be opened, Close skips the clear and the
			// next writer falls back to the PID + creation-time liveness proof.
			writerResetMutex = CreateWriterResetMutex(name);
			return true;
		}

		void Close()
		{
			// Create's failure paths get here with no owned session or retained
			// handle (both are set only on success), so they skip the retire step.
			if (ownedSessionEpoch != 0 && writerResetMutex != nullptr)
				RetireWriterIdentity();
			ownedSessionEpoch = 0;
			if (writerResetMutex) { CloseHandle(writerResetMutex); writerResetMutex = nullptr; }
			if (ring) { UnmapViewOfFile(ring); ring = nullptr; }
			if (hMap) { CloseHandle(hMap); hMap = nullptr; }
		}

		bool Publish(const DevicePoseSample &sample)
		{
			return PublishImpl(sample, []() { });
		}

#ifdef QUESTCAL_POSE_CHANNEL_TEST_SEAM
		// Called just before a writer blocks on another writer's reset mutex, so a
		// test can see a contender wait instead of sleeping and hoping.
		static inline void (*BeforeResetMutexWaitForTest)() = nullptr;

		// Returns an owned named-mutex handle after marking a retained mapping as
		// mid-reset. A test worker deliberately closes this handle without calling
		// ReleaseMutex and then exits while a contender is waiting, reproducing the
		// abandoned-owner state that follows a vrserver crash.
		static HANDLE AcquireResetOwnershipForTest(const char *name)
		{
			HANDLE mutex = AcquireWriterResetMutex(name, WriterResetMutexWaitMs);
			if (mutex == nullptr)
				return nullptr;

			HANDLE mapping = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, name);
			if (mapping == nullptr)
			{
				DWORD error = GetLastError();
				ReleaseMutex(mutex);
				CloseHandle(mutex);
				SetLastError(error);
				return nullptr;
			}
			auto shared = static_cast<PoseRing *>(MapViewOfFile(
				mapping, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(PoseRing)));
			if (shared == nullptr)
			{
				DWORD error = GetLastError();
				CloseHandle(mapping);
				ReleaseMutex(mutex);
				CloseHandle(mutex);
				SetLastError(error);
				return nullptr;
			}

			bool valid = RingHasExpectedLayout(shared);
			if (valid)
			{
				shared->resetting.store(1, std::memory_order_seq_cst);
				shared->writerActive.store(0, std::memory_order_release);
				shared->writerProcessId.store(0, std::memory_order_relaxed);
				shared->writerProcessCreationTime.store(0, std::memory_order_relaxed);
			}
			UnmapViewOfFile(shared);
			CloseHandle(mapping);
			if (!valid)
			{
				ReleaseMutex(mutex);
				CloseHandle(mutex);
				SetLastError(ERROR_REVISION_MISMATCH);
				return nullptr;
			}
			return mutex;
		}

		template<typename F>
		bool PublishAfterClaimForTest(const DevicePoseSample &sample, F afterClaim)
		{
			return PublishImpl(sample, afterClaim);
		}

		// What a pose thread does when it cannot take the claim lock.
		void RecordFailedPublishForTest()
		{
			if (ring)
				RecordFailedPublish();
		}

		// Simulate a process crash: leave the shared writer-active flag set while
		// abandoning this process's mapping handle under a known-dead PID.
		void AbandonForTest(uint32_t staleProcessId, uint64_t staleCreationTime)
		{
			if (ring)
			{
				ring->writerProcessId.store(staleProcessId, std::memory_order_release);
				ring->writerProcessCreationTime.store(staleCreationTime,
					std::memory_order_release);
			}
			ownedSessionEpoch = 0;   // so Close skips retiring the identity
			Close();
		}
#endif

	private:
		// Under the reset mutex, like publishing: unlocked, a preempted departing
		// writer could zero the identity a replacement has just published. Skipping
		// the clear when the mutex is contended is safe, because the PID +
		// creation-time liveness proof already covers a writer that never retires.
		void RetireWriterIdentity()
		{
			DWORD waitResult = WaitForSingleObject(writerResetMutex, WriterRetireWaitMs);
			if (waitResult != WAIT_OBJECT_0 && waitResult != WAIT_ABANDONED)
				return;
			struct RetireMutexGuard
			{
				HANDLE handle;
				~RetireMutexGuard()
				{
					// Preserve the operation's diagnostic across the release.
					DWORD error = GetLastError();
					ReleaseMutex(handle);
					SetLastError(error);
				}
			} retireGuard{ writerResetMutex };

			if (ring->sessionEpoch.load(std::memory_order_acquire) != ownedSessionEpoch ||
				ring->writerProcessId.load(std::memory_order_acquire) != GetCurrentProcessId())
				return;
			ring->writerActive.store(0, std::memory_order_release);
			ring->writerProcessId.store(0, std::memory_order_relaxed);
			ring->writerProcessCreationTime.store(0, std::memory_order_relaxed);
		}

		static HANDLE CreateWriterResetMutex(const char *mappingName)
		{
			try
			{
				std::string mutexName(mappingName);
				mutexName += ".WriterResetMutex";
				return CreateMutexA(nullptr, FALSE, mutexName.c_str());
			}
			catch (...)
			{
				SetLastError(ERROR_NOT_ENOUGH_MEMORY);
				return nullptr;
			}
		}

		// Returns the owned reset mutex, or null with the last error set. An
		// abandoned mutex is acquired immediately, so a zero budget costs nothing
		// on the crash-recovery path.
		static HANDLE AcquireWriterResetMutex(const char *mappingName, DWORD waitMs)
		{
			HANDLE mutex = CreateWriterResetMutex(mappingName);
			if (mutex == nullptr)
				return nullptr;
#ifdef QUESTCAL_POSE_CHANNEL_TEST_SEAM
			if (waitMs != 0 && BeforeResetMutexWaitForTest)
				BeforeResetMutexWaitForTest();
#endif
			DWORD waitResult = WaitForSingleObject(mutex, waitMs);
			if (waitResult == WAIT_OBJECT_0 || waitResult == WAIT_ABANDONED)
				return mutex;
			DWORD error = waitResult == WAIT_TIMEOUT ? ERROR_BUSY : GetLastError();
			CloseHandle(mutex);
			SetLastError(error);
			return nullptr;
		}

		bool ExistingWriterAlive() const
		{
			if (ring->writerActive.load(std::memory_order_acquire) == 0)
				return false;
			DWORD processId = ring->writerProcessId.load(std::memory_order_acquire);
			uint64_t expectedCreationTime =
				ring->writerProcessCreationTime.load(std::memory_order_acquire);
			if (processId == 0 || expectedCreationTime == 0)
				return false;

			HANDLE process = OpenProcess(SYNCHRONIZE | PROCESS_QUERY_LIMITED_INFORMATION,
				FALSE, processId);
			if (process == nullptr)
			{
				// Only a definitively absent PID permits takeover. Access failures are
				// fail-closed so two live vrserver writers can never share the ring.
				return GetLastError() != ERROR_INVALID_PARAMETER;
			}
			uint64_t actualCreationTime = 0;
			bool queried = QueryProcessCreationTime(process, actualCreationTime);
			DWORD waitResult = WaitForSingleObject(process, 0);
			CloseHandle(process);
			if (!queried)
				return true;
			return actualCreationTime == expectedCreationTime &&
				(waitResult == WAIT_TIMEOUT || waitResult == WAIT_FAILED);
		}

		bool HasExpectedLayout() const { return RingHasExpectedLayout(ring); }

		bool ResetForNewWriterSession(DWORD drainWaitMs)
		{
			// Create holds the crash-recoverable named mutex, so an inherited 1 is
			// known to belong to an abandoned reset and can be reasserted safely.
			// Readers use a seq_cst enter/recheck gate: after this store they are
			// either counted below or observe the reset before touching payloads.
			ring->resetting.store(1, std::memory_order_seq_cst);
			ring->writerActive.store(0, std::memory_order_release);
			ring->writerProcessId.store(0, std::memory_order_relaxed);
			ring->writerProcessCreationTime.store(0, std::memory_order_relaxed);
			ULONGLONG started = GetTickCount64();
			while (ring->activeReaders.load(std::memory_order_seq_cst) != 0)
			{
				if (GetTickCount64() - started >= drainWaitMs)
				{
					ring->resetting.store(0, std::memory_order_seq_cst);
					SetLastError(ERROR_BUSY);
					return false;
				}
				Sleep(0);
			}

			ring->enqueuePos.store(0, std::memory_order_relaxed);
			ring->dequeuePos.store(0, std::memory_order_relaxed);
			ring->discardSequence.store(0, std::memory_order_relaxed);
			ring->discardedLossCount.store(0, std::memory_order_relaxed);
			ring->claimLock.store(0, std::memory_order_relaxed);
			ring->pendingFailedDrops.store(0, std::memory_order_relaxed);
			for (uint64_t i = 0; i < PoseRing::Capacity; ++i)
				ring->slots[i].seq.store(i, std::memory_order_relaxed);
			ring->sessionEpoch.fetch_add(1, std::memory_order_relaxed);
			// Create publishes the new owner before releasing this gate.
			return true;
		}

		template<typename F>
		bool PublishImpl(const DevicePoseSample &sample, F afterClaim)
		{
			// Serialize only queue-position claims/gap publication, never the pose
			// copy. Contention fails fast and is itself recorded as a dropped pose;
			// vrserver pose threads are never blocked behind another device driver.
			if (!TryAcquireClaimLock(16))
			{
				RecordFailedPublish();
				return false;
			}

			uint64_t pos = ring->enqueuePos.load(std::memory_order_relaxed);
			for (;;)
			{
				auto &slot = ring->slots[pos & (PoseRing::Capacity - 1)];
				uint64_t seq = slot.seq.load(std::memory_order_acquire);
				int difference = questcal_pose_sequence_relation(seq, pos);

				if (difference == 0)
				{
					uint64_t failedBefore = HarvestPendingDrops();
					ring->enqueuePos.store(pos + 1, std::memory_order_relaxed);
					ReleaseClaimLock();
					afterClaim();
					slot.failedDropsBefore = failedBefore;
					slot.sample = sample;
					slot.seq.store(pos + 1, std::memory_order_release);
					return true;
				}

				if (difference < 0)
				{
					// Full queue. Reclaim the oldest completed slot without ever
					// touching a payload another consumer owns. If its producer is
					// still in flight, fail fast rather than blocking this pose thread.
					if (!TryDiscardOldest())
					{
						RecordFailedPublish();
						ReleaseClaimLock();
						return false;
					}
				}

				pos = ring->enqueuePos.load(std::memory_order_relaxed);
			}
		}

		bool TryAcquireClaimLock(int attempts)
		{
			for (int attempt = 0; attempt < attempts; ++attempt)
			{
				uint32_t expected = 0;
				if (ring->claimLock.compare_exchange_weak(expected, 1,
					std::memory_order_acquire, std::memory_order_relaxed))
					return true;
				YieldProcessor();
			}
			return false;
		}

		void ReleaseClaimLock()
		{
			ring->claimLock.store(0, std::memory_order_release);
		}

		void RecordFailedPublish()
		{
			ring->pendingFailedDrops.fetch_add(1, std::memory_order_acq_rel);
		}

		// Takes the pending markers for the slot being claimed. Only a concurrent
		// RecordFailedPublish or the reader's terminal harvest can make the
		// exchange retry, so the loop is as short as the contention it absorbs.
		uint64_t HarvestPendingDrops()
		{
			uint64_t word = ring->pendingFailedDrops.load(std::memory_order_acquire);
			while (!ring->pendingFailedDrops.compare_exchange_weak(word,
				PendingDropsAfterHarvest(word),
				std::memory_order_acq_rel, std::memory_order_acquire))
			{
			}
			return PendingDropCount(word);
		}

		bool TryDiscardOldest()
		{
			uint64_t pos = ring->dequeuePos.load(std::memory_order_relaxed);
			for (;;)
			{
				auto &slot = ring->slots[pos & (PoseRing::Capacity - 1)];
				uint64_t seq = slot.seq.load(std::memory_order_acquire);
				int difference = questcal_pose_sequence_relation(seq, pos + 1);
				if (difference < 0)
					return false;
				if (difference > 0)
				{
					pos = ring->dequeuePos.load(std::memory_order_relaxed);
					continue;
				}

				// Publish dequeue advancement and its complete positional loss as
				// one seqlock generation. The failed-drop marker belongs before this
				// discarded sample and would otherwise disappear with its slot.
				ring->discardSequence.fetch_add(1, std::memory_order_acq_rel);
				if (ring->dequeuePos.compare_exchange_weak(pos, pos + 1,
					std::memory_order_relaxed, std::memory_order_relaxed))
				{
					uint64_t positionalLoss = 1 + slot.failedDropsBefore;
					ring->discardedLossCount.fetch_add(positionalLoss,
						std::memory_order_relaxed);
					slot.seq.store(pos + PoseRing::Capacity, std::memory_order_release);
					ring->discardSequence.fetch_add(1, std::memory_order_release);
					return true;
				}
				ring->discardSequence.fetch_add(1, std::memory_order_release);
			}
		}

		HANDLE hMap = nullptr;
		// Held for the whole writer session so Close can retire this writer's
		// published identity under the mutex that published it.
		HANDLE writerResetMutex = nullptr;
		PoseRing *ring = nullptr;
		uint64_t ownedSessionEpoch = 0;
	};
}
