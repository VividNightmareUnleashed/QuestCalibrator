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

	{
		// Under the mutex like every other history access, even though no drain
		// thread is running yet: the consumers touched below are readable from
		// any thread that already holds a consumer id.
		std::lock_guard<std::mutex> lock(mutex);
		history.resize(static_cast<size_t>(HistoryCapacity));

		// A restart is an observation hole of unknown length. Without a boundary
		// here, post-restart samples would land directly adjacent to
		// pre-restart ones and consumers would bridge a gap they were never
		// told about - exactly the condition the monitors Reset() on.
		if (head != 0)
			AppendSessionBoundaryLocked();
	}
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
	// One allocation for the whole backlog, taken OUTSIDE the mutex. Reserving
	// only a chunk would move the growth into the copy loop below, which runs
	// under the producer mutex - the opposite of what the chunking is for.
	out.reserve(reserveCount);

	// Do not hold the producer mutex through an arbitrarily large backlog copy.
	// A fixed snapshot makes this loop finite; chunking lets the dedicated
	// ring-drain thread publish between batches.
	constexpr uint64_t CopyChunk = 512;
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
			uint64_t end = std::min(snapshotHead, head);
			uint64_t copiedThisChunk = 0;
			for (;;)
			{
				// Positional truth, decided once for both kinds of loss: the
				// cursor having fallen behind the retained window, and the entry
				// about to be copied carrying a source-drop marker, are the same
				// contract - never acknowledge a gap after copying an older
				// prefix. Reporting it before the NEXT batch's front is what
				// makes `dropped` always describe out.front().
				uint64_t oldest = head > HistoryCapacity ? head - HistoryCapacity : 0;
				bool overflowPending = cursor < oldest;
				bool sourcePending = !overflowPending && cursor < end &&
					history[cursor % HistoryCapacity].sourceDropCountBefore > dropCursor;
				if ((overflowPending || sourcePending) && !out.empty())
					return dropped;

				if (overflowPending)
				{
					// The surviving position may itself carry a source gap.
					AccountForHistoryOverflowLocked(consumer, dropped);
					continue;
				}
				if (sourcePending)
				{
					const auto &gapEntry = history[cursor % HistoryCapacity];
					dropped += gapEntry.sourceDropCountBefore - dropCursor;
					dropCursor = gapEntry.sourceDropCountBefore;
				}
				if (cursor >= end || copiedThisChunk >= CopyChunk)
					break;

				const auto &entry = history[cursor % HistoryCapacity];
				++cursor;
				++copiedThisChunk;
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

void PoseStreamHub::AppendSessionBoundaryLocked()
{
	// Everything buffered may predate a universe rebase, so discard it rather
	// than hand a consumer positionally incoherent history. The marker is a
	// single count deliberately: it says "there is a hole here", not how many
	// samples were behind it (see Drain's contract in the header).
	for (auto &consumer : consumers)
		consumer = { head, sampleCount, sourceDropCount };
	AppendGapLocked(1);
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
	// An exception escaping a thread entry calls std::terminate, with no
	// unwind: wWinMain's catch blocks would never run, so ShutdownCalibrator
	// would never flush the debounced profile and settings writes. The scratch
	// buffer can hold a full ring drain and the history append allocates, so
	// bad_alloc is reachable here. Give up draining instead of killing the
	// process - RingOpen() then reads false and the UI reports no driver.
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
	// A pending entry is a sample OR a gap; the tag is explicit because the
	// ring reader delivers the two through separate callbacks and the history
	// stores them with their own flag. Inferring it from a non-zero count would
	// publish a zero-count gap as a default-constructed pose.
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
