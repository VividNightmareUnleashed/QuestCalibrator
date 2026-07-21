#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <atomic>
#include <cstdint>

#include "Protocol.h"

namespace protocol
{
	// Lock-free pose ring in shared memory. The driver publishes every raw
	// driver-space pose it intercepts (pre-transform, QPC-stamped); the overlay
	// drains it to feed the calibration solver.
	//
	// vrserver invokes TrackedDevicePoseUpdated from each device driver's own
	// thread, so the writer side must be multi-producer safe: slots carry a
	// Vyukov-style sequence number (completed slot for absolute index i holds
	// seq == i + 1; 0 marks empty/in-progress).
	struct PoseRing
	{
		static const uint64_t Capacity = 4096;   // power of two; seconds of history at full pose rate

		struct Slot
		{
			std::atomic<uint64_t> seq;
			DevicePoseSample sample;
		};

		std::atomic<uint64_t> claimCount;        // total slots ever claimed by writers
		Slot slots[Capacity];
	};

	static_assert((PoseRing::Capacity & (PoseRing::Capacity - 1)) == 0, "capacity must be a power of two");

	class PoseRingWriter
	{
	public:
		~PoseRingWriter() { Close(); }

		bool Create(const char *name)
		{
			hMap = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, sizeof(PoseRing), name);
			if (hMap == nullptr)
				return false;

			// A fresh mapping is zero-filled, which is exactly the empty-ring state.
			// If the overlay outlived a previous vrserver, the named object (and its
			// counters) survive and publishing simply continues.
			ring = static_cast<PoseRing *>(MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(PoseRing)));
			if (ring == nullptr)
			{
				CloseHandle(hMap);
				hMap = nullptr;
				return false;
			}
			return true;
		}

		void Close()
		{
			if (ring) { UnmapViewOfFile(ring); ring = nullptr; }
			if (hMap) { CloseHandle(hMap); hMap = nullptr; }
		}

		bool IsOpen() const { return ring != nullptr; }

		void Publish(const DevicePoseSample &sample)
		{
			if (ring == nullptr)
				return;

			uint64_t idx = ring->claimCount.fetch_add(1, std::memory_order_relaxed);
			auto &slot = ring->slots[idx % PoseRing::Capacity];

			slot.seq.store(0, std::memory_order_release);   // invalidate old content for readers
			slot.sample = sample;
			slot.seq.store(idx + 1, std::memory_order_release);
		}

	private:
		HANDLE hMap = nullptr;
		PoseRing *ring = nullptr;
	};

	class PoseRingReader
	{
	public:
		~PoseRingReader() { Close(); }

		bool Open(const char *name)
		{
			hMap = OpenFileMappingA(FILE_MAP_READ, FALSE, name);
			if (hMap == nullptr)
				return false;

			ring = static_cast<const PoseRing *>(MapViewOfFile(hMap, FILE_MAP_READ, 0, 0, sizeof(PoseRing)));
			if (ring == nullptr)
			{
				CloseHandle(hMap);
				hMap = nullptr;
				return false;
			}
			cursor = 0;
			return true;
		}

		void Close()
		{
			if (ring) { UnmapViewOfFile(const_cast<PoseRing *>(ring)); ring = nullptr; }
			if (hMap) { CloseHandle(hMap); hMap = nullptr; }
		}

		bool IsOpen() const { return ring != nullptr; }

		// Deliver every completed sample since the last drain, in claim order.
		// Stops at the first still-in-flight slot and resumes there next call.
		template<typename F>
		void Drain(F &&fn)
		{
			if (ring == nullptr)
				return;

			uint64_t published = ring->claimCount.load(std::memory_order_acquire);

			if (published < cursor)
				cursor = 0;   // ring was recreated from scratch

			// Fell behind the writers: skip to the oldest slot that cannot be
			// overwritten while we read it.
			if (published - cursor > PoseRing::Capacity / 2)
				cursor = published - PoseRing::Capacity / 2;

			while (cursor < published)
			{
				const auto &slot = ring->slots[cursor % PoseRing::Capacity];

				if (slot.seq.load(std::memory_order_acquire) != cursor + 1)
					break;   // in-flight (or lost to overwrite; the fallback above unwedges us)

				DevicePoseSample copy = slot.sample;

				std::atomic_thread_fence(std::memory_order_acquire);
				if (slot.seq.load(std::memory_order_relaxed) != cursor + 1)
					break;   // overwritten mid-copy

				fn(copy);
				cursor++;
			}
		}

	private:
		HANDLE hMap = nullptr;
		const PoseRing *ring = nullptr;
		uint64_t cursor = 0;
	};
}
