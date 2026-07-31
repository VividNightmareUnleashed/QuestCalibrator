#include "stdafx.h"
#include "PoseStreamHub.h"

PoseStreamHub::~PoseStreamHub()
{
	Stop();
}

void PoseStreamHub::Start(const char *shmemName)
{
	if (drainThread.joinable())
		return;

	history.resize(static_cast<size_t>(HistoryCapacity));
#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
	resetDeferralsForTest.store(0, std::memory_order_relaxed);
#endif
	stopRequested.store(false, std::memory_order_release);
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

void PoseStreamHub::AccountForHistoryOverflowLocked(int consumer, uint64_t &dropped)
{
	auto &cursor = consumers[consumer];
	uint64_t oldest = head > HistoryCapacity ? head - HistoryCapacity : 0;
	if (cursor.historyPosition >= oldest)
		return;

	// History also contains standalone gap markers. Count only actual samples
	// overwritten here; their source-drop payload is accounted independently by
	// sourceDropCountBefore on the first retained entry.
	uint64_t oldestSampleCount = history[oldest % HistoryCapacity].sampleCountBefore;
	dropped += oldestSampleCount - cursor.samplePosition;
	cursor.samplePosition = oldestSampleCount;
	cursor.historyPosition = oldest;
}

uint64_t PoseStreamHub::Drain(int consumer, std::vector<protocol::DevicePoseSample> &out)
{
	out.clear();

	uint64_t dropped = 0;
	uint64_t snapshotHead = 0;
	size_t reserveCount = 0;
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (consumer < 0 || consumer >= static_cast<int>(consumers.size()))
			return 0;

		AccountForHistoryOverflowLocked(consumer, dropped);
		uint64_t &cursor = consumers[consumer].historyPosition;
		snapshotHead = head;
		reserveCount = static_cast<size_t>(snapshotHead - cursor);
	}
	out.reserve(reserveCount);

	// Do not hold the producer mutex through an arbitrarily large allocation and
	// backlog copy. A fixed snapshot makes this loop finite; chunking lets the
	// dedicated ring-drain thread publish between batches.
	constexpr uint64_t CopyChunk = 512;
	for (;;)
	{
		bool copyComplete = false;
#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
		std::function<void()> chunkHook;
#endif
		{
			std::lock_guard<std::mutex> lock(mutex);
			uint64_t oldest = head > HistoryCapacity ? head - HistoryCapacity : 0;
			auto &consumerCursor = consumers[consumer];
			if (consumerCursor.historyPosition < oldest && !out.empty())
			{
				// Do not acknowledge a newly-overwritten middle span after copying
				// an older prefix. The next Drain accounts it before returning the
				// surviving suffix, so `dropped` always describes out.front().
				return dropped;
			}
			AccountForHistoryOverflowLocked(consumer, dropped);
			uint64_t &cursor = consumerCursor.historyPosition;
			uint64_t end = std::min(snapshotHead, head);
			uint64_t chunkEnd = std::min(end, cursor + CopyChunk);
			for (; cursor < chunkEnd; )
			{
				const auto &entry = history[cursor % HistoryCapacity];
				uint64_t &dropCursor = consumerCursor.sourceDropPosition;
				if (entry.sourceDropCountBefore > dropCursor)
				{
					// Preserve positional truth: return an older prefix first. On the
					// next call the gap is reported immediately before this sample.
					if (!out.empty())
						return dropped;
					dropped += entry.sourceDropCountBefore - dropCursor;
					dropCursor = entry.sourceDropCountBefore;
				}
				++cursor;
				consumerCursor.samplePosition = entry.sampleCountBefore +
					(entry.hasSample ? 1 : 0);
				if (entry.hasSample)
					out.push_back(entry.sample);
			}
			copyComplete = cursor >= end;
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
	if (consumer >= 0 && consumer < static_cast<int>(consumers.size()))
		consumers[consumer] = { head, sampleCount, sourceDropCount };
}

void PoseStreamHub::AppendSampleLocked(const protocol::DevicePoseSample &sample)
{
	auto &entry = history[head % HistoryCapacity];
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
	if (count == 0)
		return;
	auto &entry = history[head % HistoryCapacity];
	entry = HistoryEntry{};
	entry.sourceDropCountBefore = (sourceDropCount += count);
	entry.sampleCountBefore = sampleCount;
	++head;
}

#ifdef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
void PoseStreamHub::AppendSampleForTest(const protocol::DevicePoseSample &sample)
{
	std::lock_guard<std::mutex> lock(mutex);
	if (history.empty())
		history.resize(static_cast<size_t>(HistoryCapacity));
	AppendSampleLocked(sample);
}

void PoseStreamHub::AppendGapForTest(uint64_t count)
{
	std::lock_guard<std::mutex> lock(mutex);
	if (history.empty())
		history.resize(static_cast<size_t>(HistoryCapacity));
	AppendGapLocked(count);
}

void PoseStreamHub::SetDrainChunkHookForTest(std::function<void()> hook)
{
	std::lock_guard<std::mutex> lock(mutex);
	drainChunkHookForTest = std::move(hook);
}
#endif

void PoseStreamHub::DrainLoop(const std::string &shmemName)
{
	protocol::PoseRingReader reader;
	struct PendingEntry
	{
		protocol::DevicePoseSample sample;
		uint64_t dropped = 0;
	};
	std::vector<PendingEntry> scratch;
	ULONGLONG lastOpenAttempt = 0;
	uint64_t lastSessionEpoch = 0;
	auto appendSessionGap = [&]()
	{
		std::lock_guard<std::mutex> lock(mutex);
		for (auto &consumer : consumers)
			consumer = { head, sampleCount, sourceDropCount };
		AppendGapLocked(1);
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
				scratch.push_back({ s, 0 });
			},
			[&](uint64_t count)
			{
				scratch.push_back({ protocol::DevicePoseSample{}, count });
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
				if (pending.dropped != 0)
					AppendGapLocked(pending.dropped);
				else
					AppendSampleLocked(pending.sample);
			}
		}

		Sleep(2);
	}
	ringOpen.store(false, std::memory_order_release);
}
