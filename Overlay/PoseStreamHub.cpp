#include "stdafx.h"
#include "PoseStreamHub.h"

PoseStreamHub::~PoseStreamHub()
{
	Stop();
}

void PoseStreamHub::Start(const char *shmemName)
{
	{
		std::lock_guard<std::mutex> lock(mutex);   // like every other history access
		history.resize(static_cast<size_t>(capacity));
	}
	drainThread = std::thread(&PoseStreamHub::DrainLoop, this, std::string(shmemName));
}

void PoseStreamHub::Stop()
{
	stopRequested.store(true, std::memory_order_release);
	if (drainThread.joinable())
		drainThread.join();
	ringOpen.store(false, std::memory_order_release);
}

int PoseStreamHub::CreateConsumer()
{
	std::lock_guard<std::mutex> lock(mutex);
	consumers.push_back({ head, sampleCount, sourceDropCount });
	return static_cast<int>(consumers.size()) - 1;
}

PoseStreamHub::Diagnostics PoseStreamHub::ReadDiagnostics()
{
	std::lock_guard<std::mutex> lock(mutex);
	Diagnostics snapshot = diagnostics;
	snapshot.reportedLoss = sourceDropCount;
	snapshot.open = RingOpen();
	return snapshot;
}

void PoseStreamHub::AccountForHistoryOverflowLocked(int consumer, uint64_t &dropped)
{
	auto &cursor = consumers[consumer];
	uint64_t oldest = head > capacity ? head - capacity : 0;
	if (cursor.historyPosition >= oldest)
		return;

	// History also contains standalone gap markers. Count only actual samples
	// overwritten here; their source-drop payload is accounted independently by
	// sourceDropCountBefore on the first retained entry.
	uint64_t oldestSampleCount = history[oldest % capacity].sampleCountBefore;
	dropped += oldestSampleCount - cursor.samplePosition;
	cursor.holeSize += oldestSampleCount - cursor.samplePosition;
	cursor.samplePosition = oldestSampleCount;
	cursor.historyPosition = oldest;
}

uint64_t PoseStreamHub::Drain(int consumer, std::vector<protocol::DevicePoseSample> &out, Hole *hole)
{
	out.clear();
	Hole found;
	const uint64_t dropped = DrainAppend(consumer, out, found);
	if (hole)
		*hole = found;
	return dropped;
}

PoseStreamHub::DrainSummary PoseStreamHub::DrainThroughGaps(int consumer,
	std::vector<protocol::DevicePoseSample> &out)
{
	out.clear();
	DrainSummary summary;
	// Each pass ends at a gap or at the head as it stood when the pass began.
	// The bound only guards against a producer that outruns the copy; in
	// practice a pass or two reaches the head.
	for (int pass = 0; pass < 64; ++pass)
	{
		const size_t before = out.size();
		Hole hole;
		const uint64_t dropped = DrainAppend(consumer, out, hole);
		if (dropped > 0)
		{
			summary.loss += dropped;
			++summary.gaps;
		}
		// The whole hole, which an earlier pass or call may have begun.
		summary.largestGap = (std::max)(summary.largestGap, hole.size);
		if (dropped == 0 && out.size() == before)
			break;
	}
	return summary;
}

uint64_t PoseStreamHub::StreamBoundaries()
{
	std::lock_guard<std::mutex> lock(mutex);
	return diagnostics.streamBoundaries;
}

uint64_t PoseStreamHub::DrainAppend(int consumer, std::vector<protocol::DevicePoseSample> &out, Hole &hole)
{
	const size_t start = out.size();
	uint64_t dropped = 0;
	uint64_t snapshotHead = 0;
	size_t reserveCount = 0;
	{
		std::lock_guard<std::mutex> lock(mutex);
		AccountForHistoryOverflowLocked(consumer, dropped);
		uint64_t &cursor = consumers[consumer].historyPosition;
		snapshotHead = head;
		reserveCount = static_cast<size_t>(snapshotHead - cursor);
	}
	// One allocation for the whole backlog, taken OUTSIDE the mutex. Reserving
	// only a chunk would move the growth into the copy loop below, which runs
	// under the producer mutex - the opposite of what the chunking is for.
	out.reserve(start + reserveCount);

	// Do not hold the producer mutex through an arbitrarily large backlog copy.
	// A fixed snapshot makes this loop finite; chunking lets the dedicated
	// ring-drain thread publish between batches.
	for (;;)
	{
		bool copyComplete = false;
#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
		std::function<void()> chunkHook;
#endif
		{
			std::lock_guard<std::mutex> lock(mutex);
			auto &consumerCursor = consumers[consumer];
			uint64_t &cursor = consumerCursor.historyPosition;
			uint64_t &dropCursor = consumerCursor.sourceDropPosition;
			const uint64_t end = snapshotHead;   // head only grows
			uint64_t copiedThisChunk = 0;
			for (;;)
			{
				// Positional truth, decided once for both kinds of loss: the
				// cursor having fallen behind the retained window, and the entry
				// about to be copied carrying a source-drop marker, are the same
				// contract - never acknowledge a gap after copying an older
				// prefix. Reporting it before the NEXT batch's front is what
				// makes `dropped` always describe out.front().
				uint64_t oldest = head > capacity ? head - capacity : 0;
				bool overflowPending = cursor < oldest;
				bool sourcePending = !overflowPending && cursor < end &&
					history[cursor % capacity].sourceDropCountBefore > dropCursor;
				if ((overflowPending || sourcePending) && out.size() > start)
					return dropped;

				if (overflowPending)
				{
					// The surviving position may itself carry a source gap.
					AccountForHistoryOverflowLocked(consumer, dropped);
					continue;
				}
				if (sourcePending)
				{
					const auto &gapEntry = history[cursor % capacity];
					dropped += gapEntry.sourceDropCountBefore - dropCursor;
					consumerCursor.holeSize += gapEntry.sourceDropCountBefore - dropCursor;
					dropCursor = gapEntry.sourceDropCountBefore;
				}
				if (cursor >= end || copiedThisChunk >= copyChunk)
					break;

				const auto &entry = history[cursor % capacity];
				++cursor;
				++copiedThisChunk;
				consumerCursor.samplePosition = entry.sampleCountBefore +
					(entry.hasSample ? 1 : 0);
				if (entry.hasSample)
				{
					// The batch's first sample closes the hole in front of it.
					if (out.size() == start)
						hole = { consumerCursor.holeSize, consumerCursor.holeHasBoundary };
					consumerCursor.holeSize = 0;
					consumerCursor.holeHasBoundary = false;
					out.push_back(entry.sample);
				}
			}
			copyComplete = cursor >= end;
			// Nothing copied: report the hole still open, under this lock.
			if (copyComplete && out.size() == start)
				hole = { consumerCursor.holeSize, consumerCursor.holeHasBoundary };
#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
			chunkHook = drainChunkHookForTest;
#endif
		}
		if (copyComplete)
			break;
#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
		if (chunkHook)
			chunkHook();
#endif
	}

	return dropped;
}

void PoseStreamHub::DiscardBacklog(int consumer)
{
	std::lock_guard<std::mutex> lock(mutex);
	consumers[consumer] = { head, sampleCount, sourceDropCount };
}

void PoseStreamHub::AppendSampleLocked(const protocol::DevicePoseSample &sample)
{
	if (sample.deviceId < diagnostics.devices.size())
	{
		auto &device = diagnostics.devices[sample.deviceId];
		++device.received;
		device.streamBoundary = diagnostics.streamBoundaries;
		device.latest = sample;
	}
	auto &entry = history[head % capacity];
	entry = HistoryEntry{};
	entry.sample = sample;
	entry.sourceDropCountBefore = sourceDropCount;
	entry.sampleCountBefore = sampleCount;
	entry.hasSample = true;
	++sampleCount;
	++head;
}

void PoseStreamHub::AppendGapLocked(uint64_t count)
{
	++diagnostics.gapMarkers;
	auto &entry = history[head % capacity];
	entry = HistoryEntry{};
	entry.sourceDropCountBefore = (sourceDropCount += count);
	entry.sampleCountBefore = sampleCount;
	++head;
}

void PoseStreamHub::AppendSessionBoundaryLocked()
{
	++diagnostics.streamBoundaries;
	// The single-count marker says "there is a hole here" (see Drain). The open
	// hole keeps its size and now holds a boundary, which the drain reports
	// with the first sample after it.
	for (auto &consumer : consumers)
		consumer = { head, sampleCount, sourceDropCount, consumer.holeSize, true };
	AppendGapLocked(1);
}

#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
void PoseStreamHub::AppendSampleForTest(const protocol::DevicePoseSample &sample)
{
	std::lock_guard<std::mutex> lock(mutex);
	if (history.empty())
		history.resize(static_cast<size_t>(capacity));
	AppendSampleLocked(sample);
}

void PoseStreamHub::AppendGapForTest(uint64_t count)
{
	std::lock_guard<std::mutex> lock(mutex);
	if (history.empty())
		history.resize(static_cast<size_t>(capacity));
	AppendGapLocked(count);
}

void PoseStreamHub::AppendSessionBoundaryForTest()
{
	std::lock_guard<std::mutex> lock(mutex);
	if (history.empty())
		history.resize(static_cast<size_t>(capacity));
	AppendSessionBoundaryLocked();
}

void PoseStreamHub::SetDrainChunkHookForTest(std::function<void()> hook)
{
	std::lock_guard<std::mutex> lock(mutex);
	drainChunkHookForTest = std::move(hook);
}

void PoseStreamHub::SetGeometryForTest(uint64_t historyCapacity, uint64_t chunk)
{
	std::lock_guard<std::mutex> lock(mutex);
	if (head != 0 || historyCapacity == 0 || chunk == 0)
		return;
	capacity = historyCapacity;
	copyChunk = chunk;
	history.assign(static_cast<size_t>(capacity), HistoryEntry{});
}
#endif

void PoseStreamHub::DrainLoop(const std::string &shmemName)
{
	// An exception escaping a thread entry calls std::terminate with no
	// unwind, so ShutdownCalibrator would never flush the debounced profile
	// and settings writes. The scratch buffer grows to a full ring drain, so
	// bad_alloc is reachable: stop draining instead, and RingOpen() reads false.
	try
	{
		DrainRing(shmemName);
	}
	catch (...)
	{
	}
	ringOpen.store(false, std::memory_order_release);
}

void PoseStreamHub::DrainRing(const std::string &shmemName)
{
	protocol::PoseRingReader reader;
	// A sample or a gap, as the ring reader's two callbacks delivered it.
	struct PendingEntry
	{
		bool isGap = false;
		protocol::DevicePoseSample sample;
		uint64_t dropped = 0;
	};
	std::vector<PendingEntry> scratch;
	ULONGLONG lastOpenAttempt = 0;
	uint64_t lastSessionEpoch = 0;
	auto appendSessionGap = [&]()
	{
		std::lock_guard<std::mutex> lock(mutex);
		AppendSessionBoundaryLocked();
	};

	while (!stopRequested.load(std::memory_order_acquire))
	{
		if (!reader.IsOpen())
		{
			ULONGLONG now = GetTickCount64();
			if (now - lastOpenAttempt >= 1000)
			{
				lastOpenAttempt = now;
				reader.Open(shmemName.c_str());
				ringOpen.store(reader.IsOpen(), std::memory_order_release);
				if (reader.IsOpen())
				{
					lastSessionEpoch = reader.SessionEpoch();
				}
			}
			if (!reader.IsOpen())
			{
				Sleep(50);
				continue;
			}
		}
		uint64_t epochBeforeDrain = reader.SessionEpoch();
		bool sessionChanged = epochBeforeDrain != lastSessionEpoch;
		scratch.clear();
		auto drainStatus = reader.Drain(
			[&](const protocol::DevicePoseSample &s)
			{
				scratch.push_back({ false, s, 0 });
			},
			[&](uint64_t count)
			{
				scratch.push_back({ true, protocol::DevicePoseSample{}, count });
			});
		if (drainStatus == protocol::PoseRingReader::DrainStatus::WriterDead)
		{
			reader.Close();
			ringOpen.store(false, std::memory_order_release);
			lastSessionEpoch = 0;
			appendSessionGap();
			continue;
		}
		if (drainStatus == protocol::PoseRingReader::DrainStatus::ResetInProgress)
		{
			// Keep the mapping while its crash-safe reset gate is active. The next
			// successful drain observes the new epoch and publishes one boundary.
#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
			resetDeferralsForTest.fetch_add(1, std::memory_order_relaxed);
#endif
			Sleep(2);
			continue;
		}
		if (sessionChanged)
			appendSessionGap();
		uint64_t epochAfterDrain = reader.SessionEpoch();
		if (epochAfterDrain != epochBeforeDrain)
		{
			// Never publish a batch that may straddle the reset. Record the new
			// session as a positional gap before its first future sample.
			scratch.clear();
			appendSessionGap();
		}
		lastSessionEpoch = epochAfterDrain;

		if (!scratch.empty())
		{
			std::lock_guard<std::mutex> lock(mutex);
			for (const auto &pending : scratch)
			{
				if (pending.isGap)
					AppendGapLocked(pending.dropped);
				else
					AppendSampleLocked(pending.sample);
			}
		}

		Sleep(2);
	}
}
