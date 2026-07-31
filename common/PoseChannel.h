#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <atomic>
#include <cstdint>
#include <new>
#include <string>
#include <utility>

#include "Protocol.h"

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
		static const uint64_t Capacity = 4096;   // power of two; seconds of history at full pose rate
		static const uint32_t Magic = 0x51435052; // "QCPR"
		static const uint32_t LayoutVersion = 3;

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
	static_assert(ATOMIC_LLONG_LOCK_FREE == 2,
		"the shared pose queue requires lock-free 64-bit atomics");

	class PoseRingWriter
	{
	public:
		~PoseRingWriter() { Close(); }

		bool Create(const char *name)
		{
			if (ring != nullptr)
				return true;
			if (name == nullptr || *name == '\0')
			{
				SetLastError(ERROR_INVALID_NAME);
				return false;
			}
			uint64_t processCreationTime = 0;
			if (!QueryProcessCreationTime(GetCurrentProcess(), processCreationTime))
				return false;

			// A named mutex is the crash-recoverable owner of mapping creation and
			// reset. The in-mapping `resetting` bit cannot fill that role by itself:
			// if a writer dies after setting it, no surviving process can prove that
			// it is stale. Windows abandons an owned mutex when its thread/process
			// exits, allowing the next writer to take over safely.
			HANDLE resetMutex = CreateWriterResetMutex(name);
			if (resetMutex == nullptr)
				return false;
			DWORD waitResult = WaitForSingleObject(resetMutex, WriterResetMutexWaitMs);
			if (waitResult != WAIT_OBJECT_0 && waitResult != WAIT_ABANDONED)
			{
				DWORD error = waitResult == WAIT_TIMEOUT ? ERROR_BUSY : GetLastError();
				CloseHandle(resetMutex);
				SetLastError(error);
				return false;
			}
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
			else if (!HasExpectedLayout())
			{
				SetLastError(ERROR_REVISION_MISMATCH);
				Close();
				return false;
			}
			else if (ExistingWriterAlive())
			{
				SetLastError(ERROR_ALREADY_EXISTS);
				Close();
				return false;
			}
			else if (!ResetForNewWriterSession())
			{
				Close();
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
			return true;
		}

		void Close()
		{
			if (ring && ownedSessionEpoch != 0 &&
				ring->sessionEpoch.load(std::memory_order_acquire) == ownedSessionEpoch &&
				ring->writerProcessId.load(std::memory_order_acquire) == GetCurrentProcessId())
			{
				ring->writerActive.store(0, std::memory_order_release);
				ring->writerProcessId.store(0, std::memory_order_relaxed);
				ring->writerProcessCreationTime.store(0, std::memory_order_relaxed);
			}
			ownedSessionEpoch = 0;
			if (ring) { UnmapViewOfFile(ring); ring = nullptr; }
			if (hMap) { CloseHandle(hMap); hMap = nullptr; }
		}

		bool Publish(const DevicePoseSample &sample)
		{
			return PublishImpl(sample, []() { });
		}

#ifdef QUESTCAL_POSE_CHANNEL_TEST_SEAM
		// Returns an owned named-mutex handle after marking a retained mapping as
		// mid-reset. A test worker deliberately closes this handle without calling
		// ReleaseMutex and then exits while a contender is waiting, reproducing the
		// abandoned-owner state that follows a vrserver crash.
		static HANDLE AcquireResetOwnershipForTest(const char *name)
		{
			HANDLE mutex = CreateWriterResetMutex(name);
			if (mutex == nullptr)
				return nullptr;
			DWORD waitResult = WaitForSingleObject(mutex, WriterResetMutexWaitMs);
			if (waitResult != WAIT_OBJECT_0 && waitResult != WAIT_ABANDONED)
			{
				DWORD error = waitResult == WAIT_TIMEOUT ? ERROR_BUSY : GetLastError();
				CloseHandle(mutex);
				SetLastError(error);
				return nullptr;
			}

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

			bool valid = shared->initialized.load(std::memory_order_acquire) == 1 &&
				shared->magic == PoseRing::Magic &&
				shared->layoutVersion == PoseRing::LayoutVersion &&
				shared->layoutBytes == sizeof(PoseRing);
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
			ownedSessionEpoch = 0;
			if (ring) { UnmapViewOfFile(ring); ring = nullptr; }
			if (hMap) { CloseHandle(hMap); hMap = nullptr; }
		}
#endif

	private:
		static constexpr DWORD WriterResetMutexWaitMs = 2000;

		static HANDLE CreateWriterResetMutex(const char *mappingName)
		{
			if (mappingName == nullptr || *mappingName == '\0')
			{
				SetLastError(ERROR_INVALID_NAME);
				return nullptr;
			}
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

		bool HasExpectedLayout() const
		{
			return ring->initialized.load(std::memory_order_acquire) == 1 &&
				ring->magic == PoseRing::Magic &&
				ring->layoutVersion == PoseRing::LayoutVersion &&
				ring->layoutBytes == sizeof(PoseRing);
		}

		bool ResetForNewWriterSession()
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
				if (GetTickCount64() - started >= 1000)
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
			if (ring == nullptr)
				return false;

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
				intptr_t difference = static_cast<intptr_t>(seq - pos);

				if (difference == 0)
				{
					uint64_t failedBefore = ring->pendingFailedDrops.exchange(
						0, std::memory_order_acq_rel);
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
		bool TryDiscardOldest()
		{
			uint64_t pos = ring->dequeuePos.load(std::memory_order_relaxed);
			for (;;)
			{
				auto &slot = ring->slots[pos & (PoseRing::Capacity - 1)];
				uint64_t seq = slot.seq.load(std::memory_order_acquire);
				intptr_t difference = static_cast<intptr_t>(seq - (pos + 1));
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
		PoseRing *ring = nullptr;
		uint64_t ownedSessionEpoch = 0;
	};

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
			if (ring == nullptr || ring->writerActive.load(std::memory_order_acquire) == 0)
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
				if (discardedLosses > observedDiscardedLossCount)
				{
					gapFn(discardedLosses - observedDiscardedLossCount);
					observedDiscardedLossCount = discardedLosses;
				}
				auto &slot = ring->slots[pos & (PoseRing::Capacity - 1)];
				uint64_t seq = slot.seq.load(std::memory_order_acquire);
				intptr_t difference = static_cast<intptr_t>(seq - (pos + 1));
				if (difference < 0)
				{
					EmitTerminalGapIfEmpty(gapFn);
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

	private:
		template<typename B, typename A>
		bool OpenImpl(const char *name, B beforeRelease, A afterRelease)
		{
			if (ring != nullptr)
				return true;

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
			if (ring->initialized.load(std::memory_order_acquire) != 1 ||
				ring->magic != PoseRing::Magic ||
				ring->layoutVersion != PoseRing::LayoutVersion ||
				ring->layoutBytes != sizeof(PoseRing))
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

		template<typename G>
		void EmitTerminalGapIfEmpty(G &gapFn)
		{
			uint32_t expected = 0;
			if (!ring->claimLock.compare_exchange_strong(expected, 1,
				std::memory_order_acquire, std::memory_order_relaxed))
				return;
			uint64_t drops = 0;
			if (ring->dequeuePos.load(std::memory_order_acquire) ==
				ring->enqueuePos.load(std::memory_order_acquire))
			{
				drops = ring->pendingFailedDrops.exchange(0, std::memory_order_acq_rel);
			}
			ring->claimLock.store(0, std::memory_order_release);
			if (drops != 0)
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
