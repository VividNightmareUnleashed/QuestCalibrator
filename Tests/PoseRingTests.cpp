// The pose ring (common/PoseRing*.h) and the pose stream hub over real named
// shared memory: concurrent publishers, overflow, stalled and abandoned
// writers, reset races, and the hub's draining, markers and failures. Moved
// from main.cpp as they were; they reach the harness through Check, which
// RunPoseRingScenarios is handed.

#ifndef QUESTCAL_POSE_CHANNEL_TEST_SEAM
#define QUESTCAL_POSE_CHANNEL_TEST_SEAM
#endif
#ifndef QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
#define QUESTCAL_POSE_STREAM_HUB_TEST_SEAM
#endif

#include "../Overlay/PoseStreamHub.h"
#include "../Overlay/RingPoseMath.h"
#include "../common/PoseRingReader.h"
#include "../common/PoseRingWriter.h"

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{

// The harness's result sink, set by RunPoseRingScenarios.
void (*Check)(const char *, bool, const char *) = nullptr;

std::string PoseRingMappingName(const char *scenario)
{
	char name[160];
	snprintf(name, sizeof name,
		"Local\\QuestCalibratorSolverTests_%s_%lu_%llu", scenario,
		static_cast<unsigned long>(GetCurrentProcessId()),
		static_cast<unsigned long long>(GetTickCount64()));
	return name;
}

struct PoseRingFixture
{
	explicit PoseRingFixture(const char *scenario) : mappingName(PoseRingMappingName(scenario)) { }

	bool Open()
	{
		return writer.Create(mappingName.c_str()) && reader.Open(mappingName.c_str());
	}

	std::string mappingName;
	protocol::PoseRingWriter writer;
	protocol::PoseRingReader reader;
};

// A minimal ring payload with an identity rotation, identified by its timestamp.
protocol::DevicePoseSample TokenSample(int64_t qpc, uint32_t deviceId = 0, double x = 0.0)
{
	protocol::DevicePoseSample sample{};
	sample.sampleTimeQpc = qpc;
	sample.deviceId = deviceId;
	sample.rotation.w = 1.0;
	sample.position[0] = x;
	return sample;
}

// Polls `done` for up to five seconds, sleeping `sleepMs` between polls, and
// returns its final value.
template <typename Done>
bool WaitFor(Done done, DWORD sleepMs)
{
	const ULONGLONG deadline = GetTickCount64() + 5000;
	while (!done() && GetTickCount64() < deadline)
		Sleep(sleepMs);
	return done();
}

// Publishes `head` on its own thread and parks that producer just after its
// claim, so the head slot stays owned until Release().
class HeldClaim
{
public:
	HeldClaim(protocol::PoseRingWriter &writer, const protocol::DevicePoseSample &head)
		: thread([this, &writer, head]()
		{
			writer.PublishAfterClaimForTest(head, [this]()
			{
				claimed.store(true, std::memory_order_release);
				while (!released.load(std::memory_order_acquire))
					Sleep(0);
			});
		})
	{
	}
	~HeldClaim() { Release(); }

	// Whether the producer reached its claim within five seconds.
	bool Wait()
	{
		return WaitFor([this] { return claimed.load(std::memory_order_acquire); }, 0);
	}
	void Release()
	{
		released.store(true, std::memory_order_release);
		if (thread.joinable())
			thread.join();
	}

private:
	std::atomic<bool> claimed{ false };
	std::atomic<bool> released{ false };
	std::thread thread;   // last: its lambda uses the flags above
};

void RunPoseRingConcurrentScenario()
{
	// Exercise the actual named shared-memory implementation with concurrent
	// publishers. Every payload carries redundant token fields so a torn or
	// mismatched sample is distinguishable from a merely missing one.
	const int producerCount = 4;
	const int samplesPerProducer = 200;
	const int total = producerCount * samplesPerProducer;

	PoseRingFixture ring("Concurrent");
	auto &writer = ring.writer;
	auto &reader = ring.reader;
	bool opened = ring.Open();
	if (!opened)
	{
		Check("pose ring: concurrent integrity", false, "could not create/open mapping");
		return;
	}

	std::unique_ptr<std::atomic<int>[]> seen(new std::atomic<int>[total]);
	for (int i = 0; i < total; ++i)
		seen[i].store(0, std::memory_order_relaxed);
	std::atomic<int> producersDone{ 0 };
	std::atomic<int> received{ 0 };
	std::atomic<int> corrupt{ 0 };
	std::atomic<int> publishRetries{ 0 };
	std::atomic<int> abandoned{ 0 };

	std::thread consumer([&]()
	{
		auto consume = [&](const protocol::DevicePoseSample &sample)
		{
			int token = static_cast<int>(sample.sampleTimeQpc - 1);
			bool valid = token >= 0 && token < total &&
				sample.deviceId == static_cast<uint32_t>(token / samplesPerProducer) &&
				sample.position[0] == static_cast<double>(token) &&
				sample.position[1] == static_cast<double>(-token) &&
				sample.rotation.w == 1.0;
			if (!valid)
				corrupt.fetch_add(1, std::memory_order_relaxed);
			else
				seen[token].fetch_add(1, std::memory_order_relaxed);
			received.fetch_add(1, std::memory_order_relaxed);
		};
		ULONGLONG started = GetTickCount64();
		while (producersDone.load(std::memory_order_acquire) < producerCount ||
		       received.load(std::memory_order_relaxed) < total)
		{
			reader.Drain(consume);
			if (received.load(std::memory_order_relaxed) >= total)
				break;
			if (producersDone.load(std::memory_order_acquire) == producerCount &&
			    GetTickCount64() - started > 5000)
				break;
			Sleep(0);
		}
		// The timeout break can race a producer's final publishes; one more
		// drain keeps a starved run from miscounting that tail as missing.
		reader.Drain(consume);
	});

	std::vector<std::thread> producers;
	producers.reserve(producerCount);
	for (int producer = 0; producer < producerCount; ++producer)
	{
		producers.emplace_back([&, producer]()
		{
			for (int i = 0; i < samplesPerProducer; ++i)
			{
				int token = producer * samplesPerProducer + i;
				protocol::DevicePoseSample sample = TokenSample(token + 1,
					static_cast<uint32_t>(producer), static_cast<double>(token));
				sample.poseIsValid = true;
				sample.deviceIsConnected = true;
				sample.position[1] = static_cast<double>(-token);
				// Integrity, not the fail-fast policy tested elsewhere: retry a
				// contention drop so a preempted producer cannot make this flaky,
				// but not forever, or a ring that never takes a sample would hang
				// the harness instead of failing this scenario.
				const ULONGLONG giveUp = GetTickCount64() + 5000;
				bool published = writer.Publish(sample);
				while (!published && GetTickCount64() < giveUp)
				{
					publishRetries.fetch_add(1, std::memory_order_relaxed);
					Sleep(0);
					published = writer.Publish(sample);
				}
				if (!published)
				{
					abandoned.fetch_add(1, std::memory_order_relaxed);
					break;
				}
			}
			producersDone.fetch_add(1, std::memory_order_release);
		});
	}

	for (auto &producer : producers)
		producer.join();
	consumer.join();

	int missing = 0;
	int duplicate = 0;
	for (int i = 0; i < total; ++i)
	{
		int count = seen[i].load(std::memory_order_relaxed);
		if (count == 0) ++missing;
		if (count > 1) duplicate += count - 1;
	}

	char detail[192];
	snprintf(detail, sizeof detail,
		"received %d/%d missing %d duplicate %d corrupt %d retries %d, producers that gave up %d",
		received.load(), total, missing, duplicate, corrupt.load(), publishRetries.load(),
		abandoned.load());
	Check("pose ring: concurrent integrity",
		received.load() == total && missing == 0 && duplicate == 0 && corrupt.load() == 0 &&
			abandoned.load() == 0,
		detail);
}

void RunPoseRingDrainStatusScenario()
{
	PoseRingFixture ring("DrainStatus");
	bool opened = ring.Open();
	protocol::PoseRingReader::DrainStatus resetStatus =
		protocol::PoseRingReader::DrainStatus::WriterDead;
	protocol::PoseRingReader::DrainStatus deadStatus =
		protocol::PoseRingReader::DrainStatus::Drained;
	if (opened)
	{
		ring.reader.SetResetInProgressForTest(true);
		resetStatus = ring.reader.Drain([](const protocol::DevicePoseSample &) { });
		ring.reader.SetResetInProgressForTest(false);
		ring.writer.Close();
		deadStatus = ring.reader.Drain([](const protocol::DevicePoseSample &) { });
	}
	Check("pose ring: drain status",
		opened && resetStatus == protocol::PoseRingReader::DrainStatus::ResetInProgress &&
		deadStatus == protocol::PoseRingReader::DrainStatus::WriterDead,
		"reset gate and writer death distinguished");
}

void RunPoseRingTerminalGapInPlaceScenario()
{
	// A failed-publish marker the reader sees on an empty queue can be taken by
	// a producer's claim and replaced by an equal count of later failures in
	// the moment between the reader's second empty-queue check and its
	// compare-exchange. Those later failures began after the claimed pose was
	// published, so they belong after it; a compare-exchange on the count
	// alone reports them ahead of it instead.
	PoseRingFixture ring("TerminalGapInPlace");
	bool opened = ring.Open();
	const protocol::DevicePoseSample pose = TokenSample(15000000, 3);
	std::string events;
	auto onSample = [&](const protocol::DevicePoseSample &sample)
	{
		events += sample.sampleTimeQpc == pose.sampleTimeQpc ? "S" : "?";
	};
	auto onGap = [&](uint64_t count) { events += "g" + std::to_string(count); };
	bool interleaved = false;
	bool published = false;
	if (opened)
	{
		ring.writer.RecordFailedPublishForTest();   // lost before the pose
		ring.reader.DrainWithTerminalGapHookForTest(onSample, onGap, [&]()
		{
			if (interleaved)
				return;
			interleaved = true;
			published = ring.writer.Publish(pose);   // carries the earlier loss
			ring.writer.RecordFailedPublishForTest();   // lost after the pose
		});
		ring.reader.Drain(onSample, onGap);
	}
	char detail[160];
	snprintf(detail, sizeof detail, "opened %d interleaved %d published %d events %s",
		opened, interleaved, published, events.c_str());
	Check("pose ring: terminal gap stays behind an earlier pose",
		opened && interleaved && published && events == "g1Sg1", detail);
}

void RunPoseRingOverflowScenario()
{
	// Let the bounded queue overflow without a reader. Producers safely reclaim
	// completed oldest slots, so the newest full ring remains available without
	// any reader/writer payload race.
	PoseRingFixture ring("Overflow");
	auto &writer = ring.writer;
	auto &reader = ring.reader;
	if (!ring.Open())
	{
		Check("pose ring: overwrite recovery", false, "could not create/open mapping");
		return;
	}

	const uint64_t overflowExtra = 257;
	for (uint64_t i = 0; i < protocol::PoseRing::Capacity + overflowExtra; ++i)
		writer.Publish(TokenSample(static_cast<int64_t>(1000000 + i), 7, static_cast<double>(i)));
	int overflowCount = 0;
	int overflowCorrupt = 0;
	uint64_t overflowGapDrops = 0;
	bool overflowGapBeforeFirst = false;
	int64_t firstToken = -1;
	int64_t lastToken = -1;
	reader.Drain(
		[&](const protocol::DevicePoseSample &sample)
		{
			int64_t token = sample.sampleTimeQpc - 1000000;
			if (overflowCount == 0)
				firstToken = token;
			lastToken = token;
			if (sample.deviceId != 7 || sample.rotation.w != 1.0 ||
				sample.position[0] != static_cast<double>(token))
				++overflowCorrupt;
			++overflowCount;
		},
		[&](uint64_t count)
		{
			overflowGapBeforeFirst = overflowCount == 0;
			overflowGapDrops += count;
		});
	int expectedRetained = static_cast<int>(protocol::PoseRing::Capacity);
	char detail[192];
	snprintf(detail, sizeof detail,
		"retained %d/%d first %lld last %lld corrupt %d positional %llu/%d",
		overflowCount, expectedRetained,
		static_cast<long long>(firstToken), static_cast<long long>(lastToken),
		overflowCorrupt,
		static_cast<unsigned long long>(overflowGapDrops), overflowGapBeforeFirst);
	Check("pose ring: overwrite recovery",
		overflowCount == expectedRetained &&
		firstToken == static_cast<int64_t>(overflowExtra) &&
		lastToken == static_cast<int64_t>(protocol::PoseRing::Capacity + overflowExtra - 1) &&
		overflowCorrupt == 0 &&
		overflowGapDrops == overflowExtra && overflowGapBeforeFirst, detail);
}

void RunPoseRingStalledProducerScenario()
{
	// Pause one producer after it owns the head slot, then fill the rest of the
	// queue. No later producer may lap and overwrite that owned payload; once
	// full, new poses must fail fast until the owner resumes.
	PoseRingFixture ring("Stalled");
	auto &stalledWriter = ring.writer;
	auto &stalledReader = ring.reader;
	if (!ring.Open())
	{
		Check("pose ring: stalled producer ownership", false, "could not create/open mapping");
		return;
	}
	char detail[192];

	HeldClaim claim(stalledWriter, TokenSample(2000000, 9));
	const bool held = claim.Wait();
	int fillSucceeded = 0;
	for (uint64_t i = 1; held && i < protocol::PoseRing::Capacity; ++i)
	{
		if (stalledWriter.Publish(TokenSample(static_cast<int64_t>(2000000 + i), 9, static_cast<double>(i))))
			++fillSucceeded;
	}

	const int rejectedAttempts = 128;
	int rejected = 0;
	for (int i = 0; held && i < rejectedAttempts; ++i)
	{
		if (!stalledWriter.Publish(TokenSample(3000000 + i)))
			++rejected;
	}
	int consumedWhileHeld = 0;
	stalledReader.Drain([&](const protocol::DevicePoseSample &) { ++consumedWhileHeld; });
	claim.Release();

	int stalledConsumed = 0;
	int stalledCorrupt = 0;
	uint64_t gapAfterPrefix = 0;
	bool terminalGapPosition = false;
	stalledReader.Drain(
		[&](const protocol::DevicePoseSample &sample)
		{
			int64_t token = sample.sampleTimeQpc - 2000000;
			if (token != stalledConsumed || sample.deviceId != 9 || sample.rotation.w != 1.0 ||
				sample.position[0] != static_cast<double>(token))
				++stalledCorrupt;
			++stalledConsumed;
		},
		[&](uint64_t count)
		{
			terminalGapPosition = stalledConsumed == static_cast<int>(protocol::PoseRing::Capacity);
			gapAfterPrefix += count;
		});

	const protocol::DevicePoseSample afterGap = TokenSample(5000000, 9, 500.0);
	bool afterGapPublished = stalledWriter.Publish(afterGap);
	uint64_t positionalTailDrops = 0;
	bool gapBeforeTailSample = false;
	int tailSamples = 0;
	stalledReader.Drain(
		[&](const protocol::DevicePoseSample &sample)
		{
			++tailSamples;
			if (sample.sampleTimeQpc != afterGap.sampleTimeQpc)
				++stalledCorrupt;
		},
		[&](uint64_t count)
		{
			gapBeforeTailSample = tailSamples == 0;
			positionalTailDrops += count;
		});
	snprintf(detail, sizeof detail,
		"held %d fill %d/%llu rejected %d/%d pre-read %d drained %d corrupt %d terminalGap %llu/%d tail %d/%llu/%d",
		held, fillSucceeded,
		static_cast<unsigned long long>(protocol::PoseRing::Capacity - 1),
		rejected, rejectedAttempts, consumedWhileHeld, stalledConsumed, stalledCorrupt,
		static_cast<unsigned long long>(gapAfterPrefix), terminalGapPosition, tailSamples,
		static_cast<unsigned long long>(positionalTailDrops), gapBeforeTailSample);
	Check("pose ring: stalled producer ownership",
		held && fillSucceeded == static_cast<int>(protocol::PoseRing::Capacity - 1) &&
		rejected == rejectedAttempts && consumedWhileHeld == 0 &&
		stalledConsumed == static_cast<int>(protocol::PoseRing::Capacity) &&
		stalledCorrupt == 0 &&
		gapAfterPrefix == rejectedAttempts && terminalGapPosition &&
		afterGapPublished && tailSamples == 1 && positionalTailDrops == 0 &&
		!gapBeforeTailSample,
		detail);
}

void RunPoseRingOverwrittenMarkerScenario()
{
	// A failed-publish marker can be attached to a later sample and then have
	// that carrier sample overwritten before the reader reaches it. The prefix
	// loss ledger must preserve both the carrier and its marker.
	PoseRingFixture ring("MarkerOverwrite");
	auto &markerWriter = ring.writer;
	auto &markerReader = ring.reader;
	if (!ring.Open())
	{
		Check("pose ring: overwritten marker accounting", false, "could not create/open mapping");
		return;
	}
	char detail[192];
	HeldClaim claim(markerWriter, TokenSample(10000000));
	const bool markerHeld = claim.Wait();
	int markerFill = 0;
	for (uint64_t i = 1; markerHeld && i < protocol::PoseRing::Capacity; ++i)
	{
		if (markerWriter.Publish(TokenSample(10000000 + static_cast<int64_t>(i))))
			++markerFill;
	}
	const int markerFailures = 11;
	int markerRejected = 0;
	for (int i = 0; markerHeld && i < markerFailures; ++i)
	{
		if (!markerWriter.Publish(TokenSample(10100000 + i)))
			++markerRejected;
	}
	claim.Release();

	const int64_t markerTailBase = 11000000;
	int markerTailPublished = 0;
	for (uint64_t i = 0; markerHeld && i <= protocol::PoseRing::Capacity; ++i)
	{
		if (markerWriter.Publish(TokenSample(markerTailBase + static_cast<int64_t>(i), 14)))
			++markerTailPublished;
	}
	int markerRetained = 0;
	bool markerOrdered = true;
	bool markerGapBeforeFirst = false;
	uint64_t markerGapTotal = 0;
	markerReader.Drain(
		[&](const protocol::DevicePoseSample &sample)
		{
			int64_t expected = markerTailBase + markerRetained + 1;
			if (sample.sampleTimeQpc != expected || sample.deviceId != 14 ||
				sample.rotation.w != 1.0)
				markerOrdered = false;
			++markerRetained;
		},
		[&](uint64_t count)
		{
			markerGapBeforeFirst = markerRetained == 0;
			markerGapTotal += count;
		});
	uint64_t markerExpectedLoss = protocol::PoseRing::Capacity + 1 + markerFailures;
	snprintf(detail, sizeof detail,
		"held %d fill %d rejected %d tail %d retained %d ordered %d gap %llu/%llu positional %d",
		markerHeld, markerFill, markerRejected, markerTailPublished,
		markerRetained, markerOrdered, static_cast<unsigned long long>(markerGapTotal),
		static_cast<unsigned long long>(markerExpectedLoss), markerGapBeforeFirst);
	Check("pose ring: overwritten marker accounting",
		markerHeld &&
		markerFill == static_cast<int>(protocol::PoseRing::Capacity - 1) &&
		markerRejected == markerFailures &&
		markerTailPublished == static_cast<int>(protocol::PoseRing::Capacity + 1) &&
		markerRetained == static_cast<int>(protocol::PoseRing::Capacity) && markerOrdered &&
		markerGapBeforeFirst && markerGapTotal == markerExpectedLoss,
		detail);
}

void RunPoseRingAbandonedWriterScenario()
{
	// Simulate vrserver dying after claiming (but before publishing) the head
	// slot while the overlay keeps the named mapping alive. A replacement writer
	// must start a clean session instead of inheriting an unreclaimable head.
	std::string reopenMappingName = PoseRingMappingName("Reopen");
	protocol::PoseRingWriter abandonedWriter;
	protocol::PoseRingReader survivingReader;
	bool reopenOpened = abandonedWriter.Create(reopenMappingName.c_str()) &&
		survivingReader.Open(reopenMappingName.c_str());
	char detail[192];
	uint64_t abandonedEpoch = survivingReader.SessionEpoch();
	bool abandoned = false;
	if (reopenOpened)
	{
		protocol::DevicePoseSample abandonedSample{};
		try
		{
			abandonedWriter.PublishAfterClaimForTest(abandonedSample, []()
			{
				throw std::runtime_error("simulated producer death");
			});
		}
		catch (const std::runtime_error &)
		{
			abandoned = true;
		}
	}
	abandonedWriter.Close();

	protocol::PoseRingWriter replacementWriter;
	bool replacementOpened = reopenOpened && replacementWriter.Create(reopenMappingName.c_str());
	uint64_t replacementEpoch = survivingReader.SessionEpoch();
	const protocol::DevicePoseSample replacementSample = TokenSample(4000000, 11, 42.0);
	bool replacementPublished = replacementOpened && replacementWriter.Publish(replacementSample);
	int replacementReceived = 0;
	bool replacementValid = false;
	survivingReader.Drain([&](const protocol::DevicePoseSample &sample)
	{
		++replacementReceived;
		replacementValid = sample.sampleTimeQpc == replacementSample.sampleTimeQpc &&
			sample.deviceId == replacementSample.deviceId &&
			sample.position[0] == replacementSample.position[0];
	});
	snprintf(detail, sizeof detail,
		"opened %d abandoned %d replacement %d epoch %llu->%llu published %d received %d valid %d",
		reopenOpened, abandoned, replacementOpened,
		static_cast<unsigned long long>(abandonedEpoch),
		static_cast<unsigned long long>(replacementEpoch), replacementPublished,
		replacementReceived, replacementValid);
	Check("pose ring: abandoned writer restart",
		reopenOpened && abandoned && replacementOpened && replacementPublished &&
		replacementEpoch == abandonedEpoch + 1 && replacementReceived == 1 && replacementValid,
		detail);
}

std::atomic<bool> ResetContenderWaiting{ false };

void NoteResetContenderWaiting()
{
	ResetContenderWaiting.store(true);
}

void RunPoseRingAbandonedResetOwnerScenario()
{
	// A retained mapping can also outlive a writer that dies midway through the
	// reset itself. The named writer/reset mutex must serialize a live owner,
	// become abandoned with its worker thread, and let exactly one replacement
	// clear the inherited resetting=1 gate. The replacement's live-writer
	// metadata must then reject another contender.
	std::string resetCrashMappingName = PoseRingMappingName("ResetCrash");
	protocol::PoseRingWriter resetCrashSeedWriter;
	protocol::PoseRingReader resetCrashReader;
	bool resetCrashCreated = resetCrashSeedWriter.Create(resetCrashMappingName.c_str()) &&
		resetCrashReader.Open(resetCrashMappingName.c_str());
	char detail[192];
	uint64_t resetCrashOldEpoch = resetCrashReader.SessionEpoch();
	resetCrashSeedWriter.Close();

	std::atomic<HANDLE> abandonedResetMutex{ nullptr };
	std::atomic<bool> resetOwnerReady{ false };
	std::atomic<bool> releaseResetOwner{ false };
	std::thread resetOwner;
	if (resetCrashCreated)
	{
		resetOwner = std::thread([&]()
		{
			HANDLE mutex = protocol::PoseRingWriter::AcquireResetOwnershipForTest(
				resetCrashMappingName.c_str());
			abandonedResetMutex.store(mutex, std::memory_order_release);
			resetOwnerReady.store(true, std::memory_order_release);
			while (mutex != nullptr &&
				!releaseResetOwner.load(std::memory_order_acquire))
				Sleep(0);
			if (mutex != nullptr)
			{
				// Intentionally do not call ReleaseMutex. Another handle is already
				// waiting below, so thread exit marks the persistent object abandoned.
				CloseHandle(mutex);
			}
		});
		WaitFor([&] { return resetOwnerReady.load(std::memory_order_acquire); }, 0);
	}
	HANDLE liveResetMutex = abandonedResetMutex.load(std::memory_order_acquire);
	bool liveResetOwnerObserved = liveResetMutex != nullptr &&
		WaitForSingleObject(liveResetMutex, 0) == WAIT_TIMEOUT;

	protocol::PoseRingWriter resetCrashReplacement;
	std::atomic<bool> resetContenderStarted{ false };
	std::atomic<bool> resetContenderDone{ false };
	bool resetCrashReplacementOpened = false;
	std::thread resetContender;
	ResetContenderWaiting.store(false);
	protocol::PoseRingWriter::BeforeResetMutexWaitForTest = &NoteResetContenderWaiting;
	if (liveResetOwnerObserved)
	{
		resetContender = std::thread([&]()
		{
			resetContenderStarted.store(true, std::memory_order_release);
			resetCrashReplacementOpened = resetCrashReplacement.Create(
				resetCrashMappingName.c_str());
			resetContenderDone.store(true, std::memory_order_release);
		});
		WaitFor([&] { return resetContenderStarted.load(std::memory_order_acquire); }, 0);
	}
	// The contender reached the live owner's reset mutex and is still behind it.
	const bool contenderWaiting = liveResetOwnerObserved &&
		WaitFor([] { return ResetContenderWaiting.load(); }, 0);
	bool resetContenderSerialized = contenderWaiting &&
		!resetContenderDone.load(std::memory_order_acquire);
	releaseResetOwner.store(true, std::memory_order_release);
	if (resetOwner.joinable())
		resetOwner.join();
	if (resetContender.joinable())
		resetContender.join();
	protocol::PoseRingWriter::BeforeResetMutexWaitForTest = nullptr;

	uint64_t resetCrashNewEpoch = resetCrashReader.SessionEpoch();
	protocol::PoseRingWriter resetCrashLiveContender;
	bool resetCrashLiveContenderRejected = resetCrashReplacementOpened &&
		!resetCrashLiveContender.Create(resetCrashMappingName.c_str());
	const protocol::DevicePoseSample resetCrashSample = TokenSample(4100000, 12, 43.0);
	bool resetCrashPublished = resetCrashReplacementOpened &&
		resetCrashReplacement.Publish(resetCrashSample);
	int resetCrashReceived = 0;
	resetCrashReader.Drain([&](const protocol::DevicePoseSample &sample)
	{
		if (sample.sampleTimeQpc == resetCrashSample.sampleTimeQpc &&
			sample.deviceId == resetCrashSample.deviceId &&
			sample.position[0] == resetCrashSample.position[0])
			++resetCrashReceived;
	});
	snprintf(detail, sizeof detail,
		"created %d owner %d serialized %d replacement %d epoch %llu->%llu liveRejected %d published %d received %d",
		resetCrashCreated, liveResetOwnerObserved, resetContenderSerialized,
		resetCrashReplacementOpened,
		static_cast<unsigned long long>(resetCrashOldEpoch),
		static_cast<unsigned long long>(resetCrashNewEpoch),
		resetCrashLiveContenderRejected, resetCrashPublished, resetCrashReceived);
	Check("pose ring: abandoned reset-owner recovery",
		resetCrashCreated && liveResetOwnerObserved && resetContenderSerialized &&
		resetCrashReplacementOpened &&
		resetCrashNewEpoch == resetCrashOldEpoch + 1 &&
		resetCrashLiveContenderRejected && resetCrashPublished &&
		resetCrashReceived == 1,
		detail);
}

void RunPoseRingOpenResetRaceScenario()
{
	// Reset exactly across Reader::Open's read-gate release. Open must retain
	// the epoch paired with its old discarded-loss snapshot; loading the epoch
	// after releasing activeReaders can pair old loss=5 with the reset epoch and
	// suppress the first five real losses in the replacement session.
	std::string openRaceMappingName = PoseRingMappingName("OpenReset");
	protocol::PoseRingWriter openRaceOldWriter;
	char detail[192];
	bool openRaceCreated = openRaceOldWriter.Create(openRaceMappingName.c_str());
	int oldRacePublished = 0;
	for (uint64_t i = 0; openRaceCreated &&
		i < protocol::PoseRing::Capacity + 5; ++i)
	{
		if (openRaceOldWriter.Publish(TokenSample(14000000 + static_cast<int64_t>(i))))
			++oldRacePublished;
	}

	protocol::PoseRingReader openRaceReader;
	protocol::PoseRingWriter openRaceReplacement;
	protocol::PoseRingWriter competingReplacement;
	std::atomic<bool> openInsideGate{ false };
	std::atomic<bool> releaseOpenGate{ false };
	std::atomic<bool> replacementDone{ false };
	std::atomic<bool> competingAttemptStarted{ false };
	bool openRaceOpened = false;
	bool openRaceReplacementOpened = false;
	bool competingReplacementOpened = false;
	std::thread openingReader;
	std::thread resettingWriter;
	std::thread competingWriter;
	if (openRaceCreated)
	{
		openingReader = std::thread([&]()
		{
			openRaceOpened = openRaceReader.OpenWithGateHooksForTest(
				openRaceMappingName.c_str(),
				[&]()
				{
					openInsideGate.store(true, std::memory_order_release);
					while (!releaseOpenGate.load(std::memory_order_acquire))
						Sleep(0);
				},
				[&]() { WaitFor([&] { return replacementDone.load(std::memory_order_acquire); }, 0); });
		});
	}
	const bool openGateReached = openRaceCreated &&
		WaitFor([&] { return openInsideGate.load(std::memory_order_acquire); }, 0);
	if (openGateReached)
	{
		openRaceOldWriter.Close();
		resettingWriter = std::thread([&]()
		{
			openRaceReplacementOpened = openRaceReplacement.Create(openRaceMappingName.c_str());
			replacementDone.store(true, std::memory_order_release);
		});
		WaitFor([&] { return openRaceReader.ResetInProgressForTest(); }, 0);
	}
	bool resetWaitObserved = openGateReached && openRaceReader.ResetInProgressForTest();
	if (resetWaitObserved)
	{
		// Keep the active reader gate held while a second replacement races the
		// first one. Exactly one writer may own and reset a retained mapping.
		competingWriter = std::thread([&]()
		{
			competingAttemptStarted.store(true, std::memory_order_release);
			competingReplacementOpened = competingReplacement.Create(openRaceMappingName.c_str());
		});
		WaitFor([&] { return competingAttemptStarted.load(std::memory_order_acquire); }, 0);
		// Let the contender reach the already-held reset gate (its failed claim
		// is immediate) before releasing the reader.
		Sleep(20);
	}
	releaseOpenGate.store(true, std::memory_order_release);
	if (openingReader.joinable())
		openingReader.join();
	if (resettingWriter.joinable())
		resettingWriter.join();
	if (competingWriter.joinable())
		competingWriter.join();

	const protocol::DevicePoseSample openRaceSeed = TokenSample(15000000, 17);
	bool openRaceSeedPublished = openRaceReplacementOpened &&
		openRaceReplacement.Publish(openRaceSeed);
	int openRaceSeedReceived = 0;
	uint64_t openRaceSeedDrops = 0;
	openRaceReader.Drain(
		[&](const protocol::DevicePoseSample &sample)
		{
			if (sample.sampleTimeQpc == openRaceSeed.sampleTimeQpc)
				++openRaceSeedReceived;
		},
		[&](uint64_t count) { openRaceSeedDrops += count; });
	int openRaceTailPublished = 0;
	for (uint64_t i = 0; openRaceSeedReceived == 1 &&
		i < protocol::PoseRing::Capacity + 1; ++i)
	{
		if (openRaceReplacement.Publish(TokenSample(15100000 + static_cast<int64_t>(i), 17)))
			++openRaceTailPublished;
	}
	int openRaceTailReceived = 0;
	uint64_t openRaceTailDrops = 0;
	openRaceReader.Drain(
		[&](const protocol::DevicePoseSample &) { ++openRaceTailReceived; },
		[&](uint64_t count) { openRaceTailDrops += count; });
	snprintf(detail, sizeof detail,
		"created %d old %d gate %d reset %d open %d replacement %d contender %d seed %d/%llu tail %d/%d gaps %llu",
		openRaceCreated, oldRacePublished, openGateReached, resetWaitObserved,
		openRaceOpened, openRaceReplacementOpened, competingReplacementOpened,
		openRaceSeedReceived,
		static_cast<unsigned long long>(openRaceSeedDrops), openRaceTailPublished,
		openRaceTailReceived, static_cast<unsigned long long>(openRaceTailDrops));
	Check("pose ring: reset during reader open",
		openRaceCreated &&
		oldRacePublished == static_cast<int>(protocol::PoseRing::Capacity + 5) &&
		openGateReached && resetWaitObserved && openRaceOpened && openRaceReplacementOpened &&
		!competingReplacementOpened &&
		openRaceSeedPublished && openRaceSeedReceived == 1 && openRaceSeedDrops == 0 &&
		openRaceTailPublished == static_cast<int>(protocol::PoseRing::Capacity + 1) &&
		openRaceTailReceived == static_cast<int>(protocol::PoseRing::Capacity) &&
		openRaceTailDrops == 1,
		detail);
}

void RunPoseHubTerminalGapScenario()
{
	// Exercise the complete producer -> shared-memory reader -> hub drain thread
	// -> consumer path. A full queue blocked behind an in-flight head records
	// failed tail publishes after the older prefix. The hub must retain the
	// standalone terminal marker even though no later sample exists yet.
	std::string hubMappingName = PoseRingMappingName("Hub");
	protocol::PoseRingWriter hubWriter;
	char detail[192];
	if (!hubWriter.Create(hubMappingName.c_str()))
	{
		Check("pose hub: positional terminal gap", false, "could not create mapping");
		return;
	}
	const int64_t hubPrefixBase = 6000000;
	int hubFillSucceeded = 0;
	int hubRejected = 0;
	const int hubRejectedAttempts = 7;
	HeldClaim claim(hubWriter, TokenSample(hubPrefixBase, 12));
	const bool hubHeld = claim.Wait();
	for (uint64_t i = 1; hubHeld && i < protocol::PoseRing::Capacity; ++i)
	{
		if (hubWriter.Publish(TokenSample(hubPrefixBase + static_cast<int64_t>(i), 12)))
			++hubFillSucceeded;
	}
	for (int i = 0; hubHeld && i < hubRejectedAttempts; ++i)
	{
		if (!hubWriter.Publish(TokenSample(7000000 + i, 12)))
			++hubRejected;
	}
	claim.Release();

	PoseStreamHub hub;
	int hubConsumer = hub.CreateConsumer();
	if (hubHeld)
		hub.Start(hubMappingName.c_str());
	ULONGLONG hubDeadline = GetTickCount64() + 5000;
	while (hubHeld && !hub.RingOpen() && GetTickCount64() < hubDeadline)
		Sleep(1);

	std::vector<protocol::DevicePoseSample> hubOut;
	int hubPrefixCount = 0;
	bool hubPrefixValid = true;
	bool hubTerminal = false;
	uint64_t hubTerminalDrops = 0;
	while (hubHeld && GetTickCount64() < hubDeadline && !hubTerminal)
	{
		uint64_t dropped = hub.Drain(hubConsumer, hubOut);
		for (const auto &sample : hubOut)
		{
			int64_t token = sample.sampleTimeQpc - hubPrefixBase;
			if (token != hubPrefixCount || sample.deviceId != 12 || sample.rotation.w != 1.0)
				hubPrefixValid = false;
			++hubPrefixCount;
		}
		if (dropped != 0)
		{
			hubTerminalDrops += dropped;
			hubTerminal = hubOut.empty() &&
				hubPrefixCount == static_cast<int>(protocol::PoseRing::Capacity);
		}
		if (!hubTerminal)
			Sleep(1);
	}

	const protocol::DevicePoseSample hubAfterGap = TokenSample(8000000, 12);
	bool hubAfterPublished = hubTerminal && hubWriter.Publish(hubAfterGap);
	bool hubResumed = false;
	uint64_t hubAfterDrops = 0;
	hubDeadline = GetTickCount64() + 5000;
	while (hubAfterPublished && GetTickCount64() < hubDeadline && !hubResumed)
	{
		hubAfterDrops += hub.Drain(hubConsumer, hubOut);
		hubResumed = hubOut.size() == 1 &&
			hubOut[0].sampleTimeQpc == hubAfterGap.sampleTimeQpc;
		if (!hubResumed)
			Sleep(1);
	}
	bool hubWasOpen = hub.RingOpen();
	hub.Stop();

	snprintf(detail, sizeof detail,
		"held %d fill %d/%llu rejected %d/%d prefix %d valid %d terminal %llu/%d resumed %d postDrops %llu",
		hubHeld, hubFillSucceeded,
		static_cast<unsigned long long>(protocol::PoseRing::Capacity - 1),
		hubRejected, hubRejectedAttempts, hubPrefixCount, hubPrefixValid,
		static_cast<unsigned long long>(hubTerminalDrops), hubTerminal,
		hubResumed, static_cast<unsigned long long>(hubAfterDrops));
	Check("pose hub: positional terminal gap",
		hubHeld && hubWasOpen &&
		hubFillSucceeded == static_cast<int>(protocol::PoseRing::Capacity - 1) &&
		hubRejected == hubRejectedAttempts && hubPrefixValid &&
		hubPrefixCount == static_cast<int>(protocol::PoseRing::Capacity) &&
		hubTerminal && hubTerminalDrops == hubRejectedAttempts &&
		hubAfterPublished && hubResumed && hubAfterDrops == 0,
		detail);
}

void RunPoseHubMarkerOverflowScenario()
{
	// Gap markers occupy history slots but are not themselves pose samples.
	// When a lagging consumer loses two samples plus an intervening marker, the
	// result is exactly two history losses plus the marker's source-drop count.
	PoseStreamHub overflowHub;
	char detail[192];
	std::vector<protocol::DevicePoseSample> hubOut;
	int overflowConsumer = overflowHub.CreateConsumer();
	protocol::DevicePoseSample overflowSample = TokenSample(9000000, 13);
	overflowHub.AppendSampleForTest(overflowSample);
	overflowSample.sampleTimeQpc++;
	overflowHub.AppendSampleForTest(overflowSample);
	overflowHub.AppendGapForTest(5);
	const int64_t retainedBase = 9100000;
	for (uint64_t i = 0; i < PoseStreamHub::HistoryCapacity; ++i)
	{
		overflowSample.sampleTimeQpc = retainedBase + static_cast<int64_t>(i);
		overflowHub.AppendSampleForTest(overflowSample);
	}
	uint64_t overflowHubDrops = overflowHub.Drain(overflowConsumer, hubOut);
	bool overflowHubSamples = hubOut.size() == PoseStreamHub::HistoryCapacity &&
		hubOut.front().sampleTimeQpc == retainedBase &&
		hubOut.back().sampleTimeQpc ==
			retainedBase + static_cast<int64_t>(PoseStreamHub::HistoryCapacity - 1);
	snprintf(detail, sizeof detail, "drops %llu/7 retained %zu/%llu ordered %d",
		static_cast<unsigned long long>(overflowHubDrops), hubOut.size(),
		static_cast<unsigned long long>(PoseStreamHub::HistoryCapacity), overflowHubSamples);
	Check("pose hub: marker overflow accounting",
		overflowHubDrops == 7 && overflowHubSamples, detail);
}

void RunPoseHubConsumerIndependenceScenario()
{
	PoseStreamHub hub;
	int firstConsumer = hub.CreateConsumer();
	int secondConsumer = hub.CreateConsumer();
	auto append = [&](int64_t token) { hub.AppendSampleForTest(TokenSample(token, 18)); };

	append(1);
	append(2);
	std::vector<protocol::DevicePoseSample> out;
	uint64_t firstInitialDrops = hub.Drain(firstConsumer, out);
	bool firstInitial = firstInitialDrops == 0 && out.size() == 2 &&
		out[0].sampleTimeQpc == 1 && out[1].sampleTimeQpc == 2;

	// The second consumer skips only its own backlog. Both consumers must still
	// observe the next source gap immediately before the next sample.
	hub.DiscardBacklog(secondConsumer);
	hub.AppendGapForTest(3);
	append(3);
	uint64_t firstGapDrops = hub.Drain(firstConsumer, out);
	bool firstAfterDiscard = firstGapDrops == 3 && out.size() == 1 &&
		out[0].sampleTimeQpc == 3;
	uint64_t secondGapDrops = hub.Drain(secondConsumer, out);
	bool secondAfterDiscard = secondGapDrops == 3 && out.size() == 1 &&
		out[0].sampleTimeQpc == 3;

	append(4);
	append(5);
	uint64_t firstPrefixDrops = hub.Drain(firstConsumer, out);
	bool firstPrefix = firstPrefixDrops == 0 && out.size() == 2 &&
		out[0].sampleTimeQpc == 4 && out[1].sampleTimeQpc == 5;
	hub.AppendGapForTest(2);
	append(6);
	uint64_t secondPrefixDrops = hub.Drain(secondConsumer, out);
	bool secondPrefix = secondPrefixDrops == 0 && out.size() == 2 &&
		out[0].sampleTimeQpc == 4 && out[1].sampleTimeQpc == 5;
	uint64_t firstTailDrops = hub.Drain(firstConsumer, out);
	bool firstTail = firstTailDrops == 2 && out.size() == 1 &&
		out[0].sampleTimeQpc == 6;
	uint64_t secondTailDrops = hub.Drain(secondConsumer, out);
	bool secondTail = secondTailDrops == 2 && out.size() == 1 &&
		out[0].sampleTimeQpc == 6;

	char detail[192];
	snprintf(detail, sizeof detail,
		"initial %d first/second gap %d/%d prefix %d/%d tail %d/%d",
		firstInitial, firstAfterDiscard, secondAfterDiscard, firstPrefix,
		secondPrefix, firstTail, secondTail);
	Check("pose hub: consumers + discard",
		firstInitial && firstAfterDiscard && secondAfterDiscard && firstPrefix &&
		secondPrefix && firstTail && secondTail,
		detail);
}

void RunPoseHubDrainThroughGapsScenario()
{
	// An idle consumer's backlog with driver drops in it: a single Drain stops
	// at the first gap and hands back only the stale prefix, which is how the
	// calibration preflight kept missing fresh traffic. Draining through gaps
	// must reach the newest sample and report every hole it crossed.
	PoseStreamHub hub;
	const int consumer = hub.CreateConsumer();
	auto append = [&](int64_t token) { hub.AppendSampleForTest(TokenSample(token, 4)); };
	append(1);
	append(2);
	hub.AppendGapForTest(1);
	append(3);
	hub.AppendGapForTest(2);
	append(4);
	append(5);
	hub.AppendGapForTest(1);   // terminal: nothing after it yet

	std::vector<protocol::DevicePoseSample> single;
	const uint64_t singleDrops = hub.Drain(consumer, single);
	bool singleStale = singleDrops == 0 && single.size() == 2 && single.back().sampleTimeQpc == 2;

	std::vector<protocol::DevicePoseSample> all;
	const auto summary = hub.DrainThroughGaps(consumer, all);
	bool reachedHead = all.size() == 3 && all.front().sampleTimeQpc == 3 &&
		all.back().sampleTimeQpc == 5;
	bool counted = summary.loss == 4 && summary.gaps == 3 && summary.largestGap == 2;
	const auto idle = hub.DrainThroughGaps(consumer, all);
	bool caughtUp = all.empty() && idle.loss == 0 && idle.gaps == 0;

	// Boundaries are counted separately: in a drain they look like a one-sample gap.
	const uint64_t boundaries = hub.StreamBoundaries();

	// The collection policy rides through contended-publish drops and stops
	// on a stall-sized hole or a session boundary.
	bool policy = ringpose::CollectionGapTolerable(0, false) &&
		ringpose::CollectionGapTolerable(summary.largestGap, false) &&
		ringpose::CollectionGapTolerable(ringpose::MaxToleratedCollectionGap, false) &&
		!ringpose::CollectionGapTolerable(ringpose::MaxToleratedCollectionGap + 1, false) &&
		!ringpose::CollectionGapTolerable(1, true);

	char detail[192];
	snprintf(detail, sizeof detail,
		"single stale %d, through %zu (head %d) loss %llu gaps %llu largest %llu, idle %d, boundaries %llu, policy %d",
		singleStale, all.size(), reachedHead, static_cast<unsigned long long>(summary.loss),
		static_cast<unsigned long long>(summary.gaps), static_cast<unsigned long long>(summary.largestGap),
		caughtUp, static_cast<unsigned long long>(boundaries), policy);
	Check("pose hub: drain through gaps to the head",
		singleStale && reachedHead && counted && caughtUp && boundaries == 0 && policy, detail);
}

void RunPoseHubMidDrainOverflowScenario()
{
	// Drain copies in bounded chunks so the producer thread can keep appending.
	// If those appends overwrite the consumer between chunks, return the already
	// copied prefix alone and leave the hole unacknowledged. The next call must
	// then report that hole immediately before the surviving suffix.
	PoseStreamHub midDrainHub;
	char detail[256];
	std::vector<protocol::DevicePoseSample> hubOut;
	int midDrainConsumer = midDrainHub.CreateConsumer();
	const int64_t midDrainBase = 12000000;
	for (uint64_t i = 0; i < PoseStreamHub::HistoryCapacity; ++i)
		midDrainHub.AppendSampleForTest(TokenSample(midDrainBase + static_cast<int64_t>(i), 15));
	// The overwrite is sized from what the first chunk copied (twice it, so the
	// cursor falls behind the retained window) and every position is relative to
	// that prefix, so no chunk size is named here. It runs on a separate thread
	// released from inside the chunk hook: if Drain held the producer mutex across
	// chunks, the injector would block, the bounded wait expire, and the second
	// Check fail rather than hang the suite.
	std::mutex injectMutex;
	std::condition_variable injectSignal;
	uint64_t injectedSamples = 0;
	bool injectRequested = false;
	bool injectFinished = false;
	std::thread injector([&]()
	{
		std::unique_lock<std::mutex> lock(injectMutex);
		injectSignal.wait(lock, [&] { return injectRequested; });
		uint64_t count = injectedSamples;
		lock.unlock();

		for (uint64_t i = 0; i < count; ++i)
		{
			midDrainHub.AppendSampleForTest(TokenSample(midDrainBase +
				static_cast<int64_t>(PoseStreamHub::HistoryCapacity + i), 15));
		}

		lock.lock();
		injectFinished = true;
		lock.unlock();
		injectSignal.notify_all();
	});

	bool overflowInjected = false;
	bool producerRanBetweenChunks = false;
	midDrainHub.SetDrainChunkHookForTest([&]()
	{
		if (overflowInjected)
			return;
		overflowInjected = true;
		std::unique_lock<std::mutex> lock(injectMutex);
		injectedSamples = static_cast<uint64_t>(hubOut.size()) * 2;
		injectRequested = true;
		injectSignal.notify_all();
		producerRanBetweenChunks = injectSignal.wait_for(lock,
			std::chrono::seconds(5), [&] { return injectFinished; });
	});
	uint64_t prefixDrops = midDrainHub.Drain(midDrainConsumer, hubOut);
	{
		// Unpark an injector whose hook never fired, so the join cannot hang.
		std::lock_guard<std::mutex> lock(injectMutex);
		injectRequested = true;
	}
	injectSignal.notify_all();
	injector.join();
	midDrainHub.SetDrainChunkHookForTest({});

	size_t prefixCount = hubOut.size();
	bool prefixExact = prefixCount > 0 &&
		hubOut.front().sampleTimeQpc == midDrainBase &&
		hubOut.back().sampleTimeQpc ==
			midDrainBase + static_cast<int64_t>(prefixCount) - 1;
	uint64_t suffixDrops = midDrainHub.Drain(midDrainConsumer, hubOut);
	bool suffixExact = hubOut.size() == PoseStreamHub::HistoryCapacity &&
		hubOut.front().sampleTimeQpc == midDrainBase + static_cast<int64_t>(injectedSamples) &&
		hubOut.back().sampleTimeQpc == midDrainBase +
			static_cast<int64_t>(PoseStreamHub::HistoryCapacity + injectedSamples - 1);
	snprintf(detail, sizeof detail,
		"injected %d/%llu prefix %d/%zu drops %llu suffix %d/%zu drops %llu",
		overflowInjected, static_cast<unsigned long long>(injectedSamples),
		prefixExact, prefixCount,
		static_cast<unsigned long long>(prefixDrops), suffixExact, hubOut.size(),
		static_cast<unsigned long long>(suffixDrops));
	Check("pose hub: mid-drain overflow position",
		overflowInjected && prefixExact && prefixDrops == 0 && suffixExact &&
		suffixDrops == injectedSamples - prefixCount,
		detail);
	Check("pose hub: producer runs between chunks",
		producerRanBetweenChunks,
		producerRanBetweenChunks
			? "another thread appended while a drain was mid-backlog"
			: "the producer mutex was not released between copy chunks");
}

// Drains until `sample` (by time and device) arrives or five seconds pass,
// adding every reported loss to `drops`.
bool DrainUntilReceived(PoseStreamHub &hub, int consumer,
	const protocol::DevicePoseSample &sample, uint64_t &drops)
{
	std::vector<protocol::DevicePoseSample> out;
	const ULONGLONG deadline = GetTickCount64() + 5000;
	while (GetTickCount64() < deadline)
	{
		drops += hub.Drain(consumer, out);
		for (const auto &candidate : out)
		{
			if (candidate.sampleTimeQpc == sample.sampleTimeQpc &&
				candidate.deviceId == sample.deviceId)
				return true;
		}
		Sleep(1);
	}
	return false;
}

void RunPoseHubWriterLivenessScenario()
{
	// A quiet writer is alive even during SteamVR standby, but a terminated one
	// must make RingOpen fall promptly. The hardest stale-owner case: the PID was
	// reused by a live process with a different creation time. The hub must
	// disconnect, reopen a replacement writer, and resume after one boundary.
	std::string livenessMappingName = PoseRingMappingName("Liveness");
	DWORD staleProcessId = GetCurrentProcessId();
	uint64_t staleCreationTime = std::numeric_limits<uint64_t>::max();
	protocol::PoseRingWriter livenessWriter;
	char detail[192];
	bool livenessCreated = livenessWriter.Create(livenessMappingName.c_str());
	PoseStreamHub livenessHub;
	int livenessConsumer = livenessHub.CreateConsumer();
	auto ringOpen = [&] { return livenessHub.RingOpen(); };
	if (livenessCreated)
	{
		livenessHub.Start(livenessMappingName.c_str());
		WaitFor(ringOpen, 1);
	}
	bool initiallyAlive = livenessHub.RingOpen();
	if (initiallyAlive)
	{
		livenessWriter.AbandonForTest(staleProcessId, staleCreationTime);
		WaitFor([&] { return !livenessHub.RingOpen(); }, 1);
	}
	bool disappearanceDetected = initiallyAlive && !livenessHub.RingOpen();

	protocol::PoseRingWriter replacementLivenessWriter;
	bool livenessRecreated = disappearanceDetected &&
		replacementLivenessWriter.Create(livenessMappingName.c_str());
	if (livenessRecreated)
		WaitFor(ringOpen, 1);
	bool reopenedAlive = livenessHub.RingOpen();
	const protocol::DevicePoseSample livenessSample = TokenSample(13000000, 16);
	bool livenessPublished = reopenedAlive &&
		replacementLivenessWriter.Publish(livenessSample);
	uint64_t livenessDrops = 0;
	bool livenessReceived = livenessPublished &&
		DrainUntilReceived(livenessHub, livenessConsumer, livenessSample, livenessDrops);
	livenessHub.Stop();
	snprintf(detail, sizeof detail,
		"reusedPid %lu created %d initial %d disappeared %d recreated %d reopened %d published %d received %d gaps %llu",
		static_cast<unsigned long>(staleProcessId), livenessCreated, initiallyAlive,
		disappearanceDetected, livenessRecreated, reopenedAlive, livenessPublished,
		livenessReceived, static_cast<unsigned long long>(livenessDrops));
	Check("pose hub: writer liveness restart",
		livenessCreated && initiallyAlive && disappearanceDetected &&
		livenessRecreated && reopenedAlive && livenessPublished && livenessReceived &&
		livenessDrops == 1,
		detail);
}

void RunPoseHubLiveResetGateScenario()
{
	std::string mappingName = PoseRingMappingName("HubLiveReset");
	protocol::PoseRingWriter writer;
	protocol::PoseRingReader resetController;
	bool created = writer.Create(mappingName.c_str());
	bool controllerOpened = created && resetController.Open(mappingName.c_str());
	PoseStreamHub hub;
	int consumer = hub.CreateConsumer();
	if (controllerOpened)
	{
		hub.Start(mappingName.c_str());
		WaitFor([&] { return hub.RingOpen(); }, 1);
	}
	bool initiallyOpen = hub.RingOpen();
	if (initiallyOpen)
	{
		resetController.SetResetInProgressForTest(true);
		WaitFor([&] { return hub.ResetDeferralsForTest() != 0; }, 1);
	}
	bool resetObserved = hub.ResetDeferralsForTest() != 0;
	bool mappingRetained = resetObserved && hub.RingOpen();
	if (initiallyOpen)
		resetController.SetResetInProgressForTest(false);

	const protocol::DevicePoseSample sample = TokenSample(16000000, 19);
	bool published = mappingRetained && writer.Publish(sample);
	uint64_t drops = 0;
	bool received = published && DrainUntilReceived(hub, consumer, sample, drops);
	hub.Stop();

	char detail[192];
	snprintf(detail, sizeof detail,
		"created/controller/open %d/%d/%d reset %d retained %d published %d received %d gaps %llu",
		created, controllerOpened, initiallyOpen, resetObserved, mappingRetained,
		published, received, static_cast<unsigned long long>(drops));
	Check("pose hub: live reset gate",
		created && controllerOpened && initiallyOpen && resetObserved &&
		mappingRetained && published && received && drops == 0,
		detail);
}

void RunPoseHubDrainFailureScenario()
{
	// A ring drain that throws (bad_alloc growing the scratch buffer, in
	// practice) must neither stop the hub for good nor let a consumer bridge
	// the outage: the hub records the failure, publishes a boundary, and
	// resumes by itself. Stop must not wait out the restart pause.
	std::string mappingName = PoseRingMappingName("HubFailure");
	protocol::PoseRingWriter writer;
	bool created = writer.Create(mappingName.c_str());
	std::atomic<bool> failNextDrain{ false };   // outlives the hub's thread
	PoseStreamHub hub;
	int consumer = hub.CreateConsumer();
	hub.SetRingDrainHookForTest([&]
	{
		if (failNextDrain.exchange(false))
			throw std::runtime_error("injected drain failure");
	});
	if (created)
	{
		hub.Start(mappingName.c_str());
		WaitFor([&] { return hub.RingOpen(); }, 1);
	}
	const protocol::DevicePoseSample before = TokenSample(17000000, 20);
	bool published = hub.RingOpen() && writer.Publish(before);
	uint64_t drops = 0;
	bool receivedBefore = published && DrainUntilReceived(hub, consumer, before, drops);
	const uint64_t boundariesBefore = hub.StreamBoundaries();
	if (receivedBefore)
	{
		failNextDrain.store(true);
		WaitFor([&] { return hub.Failures() != 0; }, 1);
	}
	const bool failed = hub.Failures() == 1;
	const bool reasonKept = hub.LastFailure() == "injected drain failure";

	// Published during the restart pause: the queue holds it for the reopened
	// reader, behind the boundary.
	const protocol::DevicePoseSample after = TokenSample(17000001, 20);
	bool publishedAfter = failed && writer.Publish(after);
	bool receivedAfter = false;
	bool boundaryInHole = false;
	uint64_t dropsAfter = 0;
	std::vector<protocol::DevicePoseSample> out;
	const ULONGLONG deadline = GetTickCount64() + 5000;
	while (publishedAfter && !receivedAfter && GetTickCount64() < deadline)
	{
		PoseStreamHub::Hole hole;
		dropsAfter += hub.Drain(consumer, out, &hole);
		receivedAfter = out.size() == 1 && out[0].sampleTimeQpc == after.sampleTimeQpc;
		boundaryInHole = hole.sessionBoundary;
		if (!receivedAfter)
			Sleep(1);
	}
	const bool reopened = hub.RingOpen();
	const uint64_t boundariesAdded = hub.StreamBoundaries() - boundariesBefore;

	// A second failure starts a two-second pause; Stop cuts it short.
	if (receivedAfter)
	{
		failNextDrain.store(true);
		WaitFor([&] { return hub.Failures() == 2; }, 1);
	}
	const bool failedAgain = hub.Failures() == 2;
	const ULONGLONG stopStarted = GetTickCount64();
	hub.Stop();
	const ULONGLONG stopMs = GetTickCount64() - stopStarted;

	char detail[224];
	snprintf(detail, sizeof detail,
		"created %d before %d/%llu failed %d reason %d after %d/%d boundary %d/%llu gaps %llu reopened %d again %d stop %llums",
		created, receivedBefore, static_cast<unsigned long long>(drops), failed, reasonKept,
		publishedAfter, receivedAfter, boundaryInHole,
		static_cast<unsigned long long>(boundariesAdded),
		static_cast<unsigned long long>(dropsAfter), reopened, failedAgain,
		static_cast<unsigned long long>(stopMs));
	Check("pose hub: a failed drain restarts behind a boundary",
		created && receivedBefore && drops == 0 && failed && reasonKept &&
		publishedAfter && receivedAfter && boundaryInHole && boundariesAdded == 1 &&
		dropsAfter == 1 && reopened && failedAgain && stopMs < 1000,
		detail);
}

}

void RunPoseRingScenarios(void (*check)(const char *, bool, const char *))
{
	Check = check;
	RunPoseRingConcurrentScenario();
	RunPoseRingDrainStatusScenario();
	RunPoseRingOverflowScenario();
	RunPoseRingStalledProducerScenario();
	RunPoseRingOverwrittenMarkerScenario();
	RunPoseRingAbandonedWriterScenario();
	RunPoseRingAbandonedResetOwnerScenario();
	RunPoseRingOpenResetRaceScenario();
	RunPoseRingTerminalGapInPlaceScenario();
	RunPoseHubTerminalGapScenario();
	RunPoseHubMarkerOverflowScenario();
	RunPoseHubConsumerIndependenceScenario();
	RunPoseHubDrainThroughGapsScenario();
	RunPoseHubMidDrainOverflowScenario();
	RunPoseHubWriterLivenessScenario();
	RunPoseHubLiveResetGateScenario();
	RunPoseHubDrainFailureScenario();
}
