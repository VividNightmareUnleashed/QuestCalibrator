#include "../Overlay/PoseStreamHub.h"
#include "../Overlay/RingPoseMath.h"

#include <cstdio>
#include <vector>

// The hole a drain reports in front of its batch. The monitors and the
// collector judge a hole against a leash and reset at a session boundary;
// both must come from the drain itself, positionally. A hole can reach a
// consumer over several drains, and a boundary can land between any read of
// StreamBoundaries and the drain.
namespace
{
using Check = void (*)(const char *, bool, const char *);
using Samples = std::vector<protocol::DevicePoseSample>;

struct Feed
{
	PoseStreamHub hub;
	int64_t next = 1;

	void Sample()
	{
		protocol::DevicePoseSample sample{};
		sample.deviceId = 0;
		sample.rotation.w = 1.0;
		sample.sampleTimeQpc = next++;
		hub.AppendSampleForTest(sample);
	}
};

// RuntimeMonitorTick read StreamBoundaries, then drained. A boundary landing
// in between reached the drain as a one-count gap, which the monitors took
// for a tolerable hole and bridged with the pre-boundary observations.
void BoundaryBetweenReadAndDrain(Check check)
{
	Feed feed;
	const int consumer = feed.hub.CreateConsumer();
	Samples out;
	feed.Sample();
	feed.hub.Drain(consumer, out);

	const uint64_t boundariesBefore = feed.hub.StreamBoundaries();
	feed.hub.AppendSessionBoundaryForTest();
	feed.Sample();
	PoseStreamHub::Hole hole;
	const uint64_t dropped = feed.hub.Drain(consumer, out, &hole);

	const bool looksSmall = dropped == 1 && ringpose::MonitorGapTolerable(dropped, false);
	char detail[160];
	snprintf(detail, sizeof detail, "batch %zu, dropped %llu, hole %llu boundary %d, count read before %llu",
		out.size(), static_cast<unsigned long long>(dropped), static_cast<unsigned long long>(hole.size),
		hole.sessionBoundary, static_cast<unsigned long long>(boundariesBefore));
	check("pose hub hole: a boundary after the count read is reported with the batch",
		out.size() == 1 && looksSmall && hole.sessionBoundary &&
		!ringpose::MonitorGapTolerable(hole.size, hole.sessionBoundary), detail);
}

// A drain that ends on a gap reports it with an empty batch; loss before the
// next sample arrives with the next drain. Judged drain by drain, each share
// passes the leash while the hole the monitors bridge does not.
void HoleSplitAcrossDrains(Check check)
{
	Feed feed;
	const int consumer = feed.hub.CreateConsumer();
	Samples out;
	feed.Sample();
	feed.hub.Drain(consumer, out);

	const uint64_t share = ringpose::MaxToleratedMonitorGap;
	feed.hub.AppendGapForTest(share);
	PoseStreamHub::Hole first;
	const uint64_t firstDropped = feed.hub.Drain(consumer, out, &first);
	const bool terminal = out.empty() && firstDropped == share && first.size == share;

	feed.hub.AppendGapForTest(share);
	feed.Sample();
	PoseStreamHub::Hole second;
	const uint64_t secondDropped = feed.hub.Drain(consumer, out, &second);

	// Once a sample closes it, the next hole starts from nothing.
	feed.hub.AppendGapForTest(1);
	feed.Sample();
	PoseStreamHub::Hole third;
	feed.hub.Drain(consumer, out, &third);

	char detail[192];
	snprintf(detail, sizeof detail, "terminal %d, then dropped %llu hole %llu (tolerable %d), next hole %llu",
		terminal, static_cast<unsigned long long>(secondDropped), static_cast<unsigned long long>(second.size),
		ringpose::MonitorGapTolerable(second.size, second.sessionBoundary),
		static_cast<unsigned long long>(third.size));
	check("pose hub hole: a hole reported over two drains is judged whole",
		terminal && secondDropped == share && second.size == 2 * share && !second.sessionBoundary &&
		!ringpose::MonitorGapTolerable(second.size, false) && third.size == 1, detail);
}

// The collector's largest gap, split the same way across two collection ticks.
void CollectionHoleSplitAcrossTicks(Check check)
{
	Feed feed;
	const int consumer = feed.hub.CreateConsumer();
	Samples out;
	feed.Sample();
	feed.hub.DrainThroughGaps(consumer, out);

	const uint64_t share = ringpose::MaxToleratedCollectionGap;
	feed.hub.AppendGapForTest(share);
	const auto firstTick = feed.hub.DrainThroughGaps(consumer, out);
	feed.hub.AppendGapForTest(share);
	feed.Sample();
	const auto secondTick = feed.hub.DrainThroughGaps(consumer, out);

	char detail[160];
	snprintf(detail, sizeof detail, "first tick largest %llu, second tick largest %llu loss %llu, batch %zu",
		static_cast<unsigned long long>(firstTick.largestGap),
		static_cast<unsigned long long>(secondTick.largestGap),
		static_cast<unsigned long long>(secondTick.loss), out.size());
	check("pose hub hole: the collector sizes a hole split across ticks",
		firstTick.largestGap == share && secondTick.loss == share && out.size() == 1 &&
		secondTick.largestGap == 2 * share &&
		!ringpose::CollectionGapTolerable(secondTick.largestGap, false), detail);
}

// A boundary between two chunks of one drain: the prefix copied before it is
// returned clean, and the boundary comes with the first sample after it.
void BoundaryBetweenChunks(Check check)
{
	Feed feed;
	const int consumer = feed.hub.CreateConsumer();
	const size_t backlog = 600;   // more than one CopyChunk
	for (size_t i = 0; i < backlog; ++i)
		feed.Sample();
	bool fired = false;
	const int64_t firstAfter = feed.next;
	feed.hub.SetDrainChunkHookForTest([&]()
	{
		if (fired)
			return;
		fired = true;
		feed.hub.AppendSessionBoundaryForTest();
		feed.Sample();
	});
	Samples out;
	PoseStreamHub::Hole prefixHole;
	const uint64_t prefixDropped = feed.hub.Drain(consumer, out, &prefixHole);
	feed.hub.SetDrainChunkHookForTest({});
	const size_t prefix = out.size();
	const bool prefixClean = fired && prefix > 0 && prefix < backlog && prefixDropped == 0 &&
		prefixHole.size == 0 && !prefixHole.sessionBoundary;

	PoseStreamHub::Hole hole;
	const uint64_t dropped = feed.hub.Drain(consumer, out, &hole);
	const bool after = out.size() == 1 && out.front().sampleTimeQpc == firstAfter;

	char detail[160];
	snprintf(detail, sizeof detail, "hook %d prefix %zu clean %d, then %zu (post-boundary %d) dropped %llu boundary %d",
		fired, prefix, prefixClean, out.size(), after, static_cast<unsigned long long>(dropped),
		hole.sessionBoundary);
	check("pose hub hole: a boundary between chunks is reported after the prefix",
		prefixClean && after && dropped == 1 && hole.sessionBoundary, detail);
}

// DiscardBacklog starts a new stream: nothing open carries over.
void DiscardClosesHole(Check check)
{
	Feed feed;
	const int consumer = feed.hub.CreateConsumer();
	Samples out;
	feed.hub.AppendGapForTest(3);
	feed.hub.Drain(consumer, out);
	feed.hub.AppendSessionBoundaryForTest();
	feed.hub.DiscardBacklog(consumer);
	feed.Sample();
	PoseStreamHub::Hole hole;
	const uint64_t dropped = feed.hub.Drain(consumer, out, &hole);

	char detail[128];
	snprintf(detail, sizeof detail, "batch %zu dropped %llu hole %llu boundary %d", out.size(),
		static_cast<unsigned long long>(dropped), static_cast<unsigned long long>(hole.size),
		hole.sessionBoundary);
	check("pose hub hole: discarding the backlog closes the hole",
		out.size() == 1 && dropped == 0 && hole.size == 0 && !hole.sessionBoundary, detail);
}
} // namespace

void RunPoseHubHoleScenarios(Check check)
{
	BoundaryBetweenReadAndDrain(check);
	HoleSplitAcrossDrains(check);
	CollectionHoleSplitAcrossTicks(check);
	BoundaryBetweenChunks(check);
	DiscardClosesHole(check);
}
