#include "../Overlay/ContinuousAlignment.h"
#include "../Overlay/JumpDetector.h"
#include "../Overlay/TrackingStreamDigest.h"
#include "../Overlay/LighthouseFrameWatch.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr double QpcSeconds = 1e-6;

protocol::DevicePoseSample Sample(uint32_t id, double time, double shift)
{
	protocol::DevicePoseSample s{};
	s.sampleTimeQpc = static_cast<int64_t>(time / QpcSeconds + 0.5);
	s.deviceId = id;
	s.deviceIsConnected = true;
	s.poseIsValid = true;
	s.trackingResult = vr::TrackingResult_Running_OK;
	s.rotation = s.worldFromDriverRotation = { 1, 0, 0, 0 };
	s.position[0] = shift + 0.3 * id;
	s.position[1] = 1.5;
	return s;
}

struct Deltas
{
	int count = 0;
	double total = 0.0;   // summed x shift of every accepted delta
	JumpDetector::UniverseDelta last;
};

// Pushes make(frame, id) for every id on each 10 ms frame and collects the
// accepted deltas.
Deltas Drive(const std::vector<uint32_t> &ids, int frames,
             const std::function<protocol::DevicePoseSample(int, uint32_t)> &make)
{
	JumpDetector detector(QpcSeconds);
	Deltas r;
	for (int frame = 0; frame <= frames; ++frame)
	{
		for (auto id : ids)
			detector.Push(make(frame, id));
		JumpDetector::UniverseDelta delta;
		while (detector.PollDelta(delta))
		{
			++r.count;
			r.total += delta.translation.x();
			r.last = delta;
		}
	}
	return r;
}

// A device whose position repeats bit for bit is, on the engine, a
// controller the static prior has locked still; `moving` adds the
// sub-millimetre motion a tracking controller always carries.
Deltas Replay(const std::function<double(int, uint32_t)> &shift, const std::vector<uint32_t> &ids,
              int frames = 300, bool moving = false)
{
	return Drive(ids, frames, [&](int frame, uint32_t id)
	{
		auto s = Sample(id, 1.0 + frame * 0.01, shift(frame, id));
		if (moving)
			s.position[2] = 1e-5 * std::sin(700.0 * (1.0 + frame * 0.01) + id);
		return s;
	});
}

// Headset-only replay with a living headset: a slow turn so the heading
// changes every frame, and (unless `heldPosition`) sub-millimetre SLAM noise
// so the position never repeats bit-for-bit. Frames in [skipFrom, skipTo)
// are not pushed, which the detector reads as a stream gap. Every
// `holeEvery`th frame is a driver queue drop instead: not pushed, and
// reported as a hole the way the runtime monitor reports one. Notes are
// drained into `log` when given.
int ReplaySolo(int frames, const std::function<double(int)> &shift,
               const std::function<double(int)> &yawStep, bool heldPosition,
               int skipFrom = -1, int skipTo = -1,
               std::vector<std::string> *log = nullptr,
               JumpDetector::UniverseDelta *last = nullptr,
               int holeEvery = 0, bool driftFollowed = false)
{
	JumpDetector detector(QpcSeconds);
	detector.SetDriftFollowed(driftFollowed);
	int count = 0;
	for (int frame = 0; frame <= frames; ++frame)
	{
		if (frame >= skipFrom && frame < skipTo)
			continue;
		if (holeEvery > 0 && frame > 0 && frame % holeEvery == 0)
		{
			detector.NoteStreamHole();
			continue;
		}
		double time = 1.0 + frame * 0.01;
		auto s = Sample(0, time, shift(frame));
		double yaw = 0.1 * time + yawStep(frame);
		s.rotation = { std::cos(yaw / 2), 0, std::sin(yaw / 2), 0 };
		if (!heldPosition)
			s.position[2] = 1e-5 * std::sin(time * 700.0);
		detector.Push(s);
		JumpDetector::UniverseDelta delta;
		while (detector.PollDelta(delta))
		{
			++count;
			if (last) *last = delta;
		}
		std::string note;
		while (detector.PollNote(note))
			if (log) log->push_back(note);
	}
	return count;
}

// The last line containing `text`, or "" when none does.
std::string LastWith(const std::vector<std::string> &lines, const char *text)
{
	std::string found;
	for (const auto &l : lines)
		if (l.find(text) != std::string::npos)
			found = l;
	return found;
}

bool Has(const std::vector<std::string> &lines, const char *text)
{
	return !LastWith(lines, text).empty();
}

double YawDegrees(const Eigen::Quaterniond &q)
{
	return 2.0 * std::atan2(q.y(), q.w()) * 180.0 / EIGEN_PI;
}

questcal::ContinuousAlignment Aligner()
{
	questcal::ContinuousAlignment result;
	questcal::MountExtrinsic mount;
	mount.valid = true;
	result.SetExtrinsic(mount);
	return result;
}

void Pair(questcal::ContinuousAlignment &aligner, double time, double offset)
{
	questcal::PoseSample h, t;
	h.time = t.time = time;
	h.pos.x() = offset;
	aligner.PushReference(h);
	aligner.PushTarget(t);
	aligner.Update(time, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 1.0, 0.0);
}

void Warm(questcal::ContinuousAlignment &aligner, double offset)
{
	for (int i = 0; i <= 1000; ++i)
		Pair(aligner, 1.0 + i * 0.01, offset);
}
}

void RunTrackingRecoveryScenarios(void (*check)(const char *, bool, const char *))
{
	using CA = questcal::ContinuousAlignment;
	Deltas r = Replay([](int f, uint32_t) { return f >= 150 ? .15 : 0.; }, {0, 1});
	check("recovery: 15 cm HMD/controller reset, no mounted tracker",
		r.count == 1 && std::abs(r.last.translation.x() - .15) < 1e-8, "matching persistent pose steps, unchanged WFD");

	check("recovery: isolated moderate HMD error waits",
		Replay([](int f, uint32_t id) { return f >= 150 && id == 0 ? .15 : 0.; }, {0, 1}).count == 0,
		"no corroborating controller");
	check("recovery: moderate reset never uses solo fallback",
		Replay([](int f, uint32_t) { return f >= 150 ? .15 : 0.; }, {0}).count == 0, "HMD only");
	check("recovery: moderate reset needs tight agreement",
		Replay([](int f, uint32_t id) { return f >= 150 ? (id == 0 ? .15 : .21) : 0.; }, {0, 1}).count == 0,
		"6 cm disagreement");
	check("recovery: controllers cannot move the HMD universe",
		Replay([](int f, uint32_t id) { return f >= 150 && id != 0 ? .4 : 0.; }, {0, 1, 2}).count == 0,
		"two controllers reset, HMD remains continuous");
	check("recovery: shared one-frame glitch is not a reset",
		Replay([](int f, uint32_t) { return f == 150 ? .15 : 0.; }, {0, 1}).count == 0, "both streams immediately return");
	r = Replay([](int f, uint32_t) { return (f >= 100 ? .15 : 0.) + (f >= 230 ? -.12 : 0.); }, {0, 1});
	check("recovery: distinct moderate resets each apply once",
		r.count == 2 && std::abs(r.last.translation.x() + .12) < 1e-8, "no duplicate compensation");
	check("recovery: distinct resets inside one second",
		Replay([](int f, uint32_t) { return (f >= 100 ? .15 : 0.) + (f >= 160 ? -.12 : 0.); }, {0, 1}).count == 2,
		"each has a clean window after the preceding reset");
	for (bool reset : {false, true})
	{
		r = Drive({0, 1}, 300, [&](int i, uint32_t id)
		{
			double captureTime = 1 + i * .01;
			double horizon = i >= 150 ? .06 - .01 * id : 0;
			double speed = 1.2 - .1 * id;
			auto s = Sample(id, captureTime, reset && i >= 150 ? .15 : 0);
			s.poseTimeOffset = horizon;
			s.position[0] += speed * (captureTime + horizon);
			s.velocity[0] = speed;
			return s;
		});
		check(reset ? "recovery: reset survives a prediction horizon change"
		            : "recovery: prediction horizon change is not a reset",
			reset ? r.count == 1 && std::abs(r.last.translation.x() - .15) < 1e-8 : r.count == 0,
			"different device speeds and horizons; pose validity time includes driver offset");
	}
	r = Drive({0, 1}, 300, [](int i, uint32_t id)
	{
		double time = 1 + i * .01;
		auto s = Sample(id, time, i >= 150 ? .15 : 0.);
		s.position[0] += .1 * std::sin(3 * time + id) + .001 * std::sin(77 * time + id);
		s.velocity[0] = .3 * std::cos(3 * time + id);
		return s;
	});
	check("recovery: moderate reset during independent device motion",
		r.count == 1 && std::abs(r.last.translation.x() - .15) < .005,
		"different trajectories plus millimetre tracking noise");
	r = Drive({0, 1}, 300, [](int i, uint32_t id)
	{
		double angle = i >= 150 ? 3 * EIGEN_PI / 180 : 0;
		auto s = Sample(id, 1 + i * .01, 0);
		s.rotation = {std::cos(angle / 2), 0, std::sin(angle / 2), 0};
		s.position[2] = -std::sin(angle) * s.position[0];
		s.position[0] *= std::cos(angle);
		return s;
	});
	check("recovery: corroborated small yaw reset", r.count == 1, "3 degrees, shared rigid reference change");

	// Quest Pro controllers are separate tracking frontends on the shared map
	// and may keep the previous frame for up to 30 s after a headset map
	// switch. An HMD step nobody confirmed at once is held for that follow-up,
	// stamped with the HMD's jump time, and never applied alone while
	// controllers stay continuous.
	r = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 1350) ? .4 : 0.; }, {0, 1}, 3500);
	check("recovery: controller follows a headset map switch 12 s later",
		r.count == 1 && std::abs(r.last.translation.x() - .4) < 1e-8 && std::abs(r.last.time - 2.5) < 1e-6 &&
		std::abs(r.last.confirmationLagSeconds - 12.0) < 1e-6 && r.last.devicesAgreeing == 2,
		"held HMD candidate confirmed by the late controller step; delta keeps the HMD jump time");
	check("recovery: headset step no controller follows expires",
		Replay([](int f, uint32_t id) { return f >= 150 && id == 0 ? .4 : 0.; }, {0, 1}, 3500, true).count == 0,
		"controller tracking and continuous for 33 s");
	check("recovery: late controller step must match the headset step",
		Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 1350) ? (id == 0 ? .4 : .25) : 0.; }, {0, 1}, 3500).count == 0,
		"15 cm disagreement 12 s later");
	check("recovery: controller step after the follow-up window does not confirm",
		Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 3300) ? .4 : 0.; }, {0, 1}, 3600, true).count == 0,
		"tracking controller, 31.5 s later");

	// A controller lying still is locked by the engine's static prior and
	// cannot step until the hand moves. The follow-up clock pauses while every
	// other device is locked, so the same 31.5 s step confirms, and a locked
	// controller that never moves keeps the candidate waiting.
	r = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 3300) ? .4 : 0.; }, {0, 1}, 3600);
	check("recovery: a locked controller confirms the headset step when it moves",
		r.count == 1 && std::abs(r.last.translation.x() - .4) < 1e-8 &&
		std::abs(r.last.confirmationLagSeconds - 31.5) < 1e-6, "position bit-identical for 33 s, then the step");
	check("recovery: a locked controller's later step must still match",
		Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 3300) ? (id == 0 ? .4 : .25) : 0.; }, {0, 1}, 3600).count == 0,
		"15 cm disagreement after the lock");

	// The controller frontend's anchor follower has no grace, so inside the
	// headset smoother's 5 s grace the controllers snap first. A matching
	// controller step up to 5 s ahead confirms the headset's, with a negative
	// lag; further ahead it does not.
	r = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 450 : 150) ? .4 : 0.; }, {0, 1}, 900, true);
	check("recovery: controller step 3 s ahead confirms the headset step",
		r.count == 1 && std::abs(r.last.translation.x() - .4) < 1e-8 &&
		std::abs(r.last.time - 5.5) < 1e-6 && std::abs(r.last.confirmationLagSeconds + 3.0) < 1e-6,
		"delta keeps the HMD jump time; lag reported as -3.0 s");
	check("recovery: controller step 6 s ahead does not confirm",
		Replay([](int f, uint32_t id) { return f >= (id == 0 ? 750 : 150) ? .4 : 0.; }, {0, 1}, 1200, true).count == 0,
		"beyond the 5 s grace");
	check("recovery: leading controller step must match the headset step",
		Replay([](int f, uint32_t id) { return f >= (id == 0 ? 450 : 150) ? (id == 0 ? .4 : .25) : 0.; }, {0, 1}, 900, true).count == 0,
		"15 cm disagreement 3 s ahead");
	r = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 1350) ? .15 : 0.; }, {0, 1}, 3500);
	check("recovery: moderate headset step waits for the follow-up too",
		r.count == 1 && std::abs(r.last.translation.x() - .15) < 1e-8, "tight agreement, 12 s apart");
	check("recovery: large solo step still applies at once",
		Replay([](int f, uint32_t) { return f >= 150 ? .4 : 0.; }, {0}, 3500).count == 1, "no controller to wait for");
	// Two headset steps 3 s apart, each followed by the controller 12 s later:
	// accepting the first must not discard the second, still waiting.
	{
		r = Replay([](int f, uint32_t id)
		{
			return (f >= (id == 0 ? 150 : 1350) ? .4 : 0.) + (f >= (id == 0 ? 450 : 1650) ? .4 : 0.);
		}, {0, 1}, 3500, true);
		char detail[96];
		snprintf(detail, sizeof detail, "%d deltas, %.3f m of 0.800 m compensated", r.count, r.total);
		check("recovery: a second held headset step survives the first's confirmation",
			r.count == 2 && std::abs(r.total - .8) < 1e-6, detail);
	}

	// Headset-only: below the solo floor a persistent HMD step is applied on
	// the fit alone once the stream has been steady for a minute and the
	// pre-window shows no held position (the engine's 3DoF fallback holds the
	// last tracked position bit-for-bit under a moving IMU heading).
	{
		auto none = [](int) { return 0.; };
		auto lateStep = [](int f) { return f >= 9000 ? .08 : 0.; };
		auto twoSteps = [](int f) { return (f >= 500 ? .08 : 0.) + (f >= 1500 ? .08 : 0.); };
		JumpDetector::UniverseDelta delta;
		std::vector<std::string> log;

		int count = ReplaySolo(9200, lateStep, none, false, -1, -1, &log, &delta);
		check("headset-only: 8 cm step after 90 s of steady tracking applies alone",
			count == 1 && std::abs(delta.translation.x() - .08) < 1e-6 && delta.devicesAgreeing == 1 &&
			Has(log, "accepted alone"), "clean fit, stream steady, position never repeated");

		count = ReplaySolo(2300, [](int f) { return f >= 2000 ? .08 : 0.; }, none, false, -1, -1, &log);
		check("headset-only: 8 cm step 20 s after the stream started is refused",
			count == 0 && Has(log, "stream steady under 60 s"),
			"a wake or a stream restart re-zeroes inside the first minute");

		count = ReplaySolo(11600, [](int f) { return f >= 11300 ? .08 : 0.; }, none, false, 9000, 9300);
		check("headset-only: 8 cm step 20 s after a stream gap is refused", count == 0, "3 s gap, then 20 s of stream");

		log.clear();
		count = ReplaySolo(9200, lateStep, none, true, -1, -1, &log);
		check("headset-only: step after a held position is refused",
			count == 0 && Has(log, "position was held before the step (3DoF)"),
			"position bit-constant while the heading moved: the 3DoF-to-6DoF snap");

		// A frame change turns about the map's points, so the head moves
		// with the heading: 3 deg about a point 57 cm away moves it 3 cm.
		auto lateTurn = [](int f) { return f >= 9000 ? 3.0 * EIGEN_PI / 180.0 : 0.; };
		count = ReplaySolo(9200, [](int f) { return f >= 9000 ? .03 : 0.; }, lateTurn, false, -1, -1, nullptr, &delta);
		check("headset-only: 3 deg yaw step alone applies after steady tracking",
			count == 1 && std::abs(YawDegrees(delta.rotation) - 3.0) < 0.05, "3 deg with the head moved 3 cm, above the 2 deg floor");

		// The heading alone stepping, the head still, while the user turns
		// under the smoother's yaw dead zone: the drift catch-up. Refused,
		// unless continuous alignment has followed the drift, in which case
		// the calibration carries it and the step is compensated.
		log.clear();
		count = ReplaySolo(9200, none, lateTurn, false, -1, -1, &log);
		check("headset-only: a 3 deg turn about the head is refused as a drift catch-up",
			count == 0 && Has(log, "a turn about the head"),
			"heading stepped, head still: the smoother cancelling heading drift");

		count = ReplaySolo(9200, none, lateTurn, false, -1, -1, nullptr, &delta, 0, true);
		check("headset-only: the same turn is compensated while continuous alignment follows drift",
			count == 1 && std::abs(YawDegrees(delta.rotation) - 3.0) < 0.05, "the calibration already carries the drift");

		count = ReplaySolo(9200, [](int f) { return f >= 9000 ? .03 : 0.; }, none, false);
		check("headset-only: 3 cm step stays below the floor", count == 0, "under 5 cm and 2 deg nothing is applied");

		// Refused steps are summed in the log, to compare with the drift a
		// recalibration removes.
		log.clear();
		count = ReplaySolo(1800, twoSteps, none, false, -1, -1, &log);
		std::string total = LastWith(log, "ignored");
		check("headset-only: ignored steps are totalled in the log",
			count == 0 && total.find("2 ignored this session") != std::string::npos &&
			total.find("shift 0.160 m in total") != std::string::npos,
			total.c_str());

		// With a dozen devices on the ring the driver drops an isolated pose
		// every few seconds (186 holes in one 57 minute session). A hole breaks
		// continuity and nothing else: it must not restart the steady clock.
		log.clear();
		count = ReplaySolo(9300, [](int f) { return f >= 9050 ? .08 : 0.; }, none, false, -1, -1, &log, &delta, 1800);
		check("stream holes: 8 cm step still applies alone with a pose lost every 18 s",
			count == 1 && std::abs(delta.translation.x() - .08) < 1e-6 && Has(log, "accepted alone"),
			"five holes in 90 s, the last 0.5 s before the step");

		count = ReplaySolo(9200, lateStep, none, false, -1, -1, nullptr, nullptr, 1800);
		check("stream holes: a step across a hole is never measured", count == 0,
			"the lost pose is the one at the step");

		log.clear();
		count = ReplaySolo(1800, twoSteps, none, false, -1, -1, &log, nullptr, 700);
		total = LastWith(log, "ignored");
		check("stream holes: ignored steps keep totalling across holes",
			count == 0 && total.find("2 ignored this session") != std::string::npos &&
			total.find("shift 0.160 m in total") != std::string::npos,
			total.c_str());

		count = ReplaySolo(11600, [](int f) { return f >= 11300 ? .08 : 0.; }, none, false, 9000, 9300, nullptr, nullptr, 1800);
		check("stream holes: a real stream gap still restarts the steady clock", count == 0,
			"3 s gap, then 20 s of stream with holes");

		check("stream holes: the monitors ride through a few poses only",
			ringpose::MonitorGapTolerable(0, false) && ringpose::MonitorGapTolerable(2, false) &&
			ringpose::MonitorGapTolerable(ringpose::MaxToleratedMonitorGap, false) &&
			!ringpose::MonitorGapTolerable(ringpose::MaxToleratedMonitorGap + 1, false) &&
			!ringpose::MonitorGapTolerable(1, true),
			"a larger hole or a session boundary resets them");
	}
	// A held headset step has both fit windows behind it, so a hole during the
	// wait for the controller says nothing about it; an invalid headset frame
	// still revokes it.
	for (bool hole : { true, false })
	{
		JumpDetector detector(QpcSeconds);
		for (int i = 0; i <= 1400; ++i)
		{
			if (hole && i == 800)
			{
				detector.NoteStreamHole();
				continue;
			}
			auto hmd = Sample(0, 1 + i * .01, i >= 150 ? .4 : 0);
			hmd.poseIsValid = hole || i != 800;
			detector.Push(hmd);
			detector.Push(Sample(1, 1 + i * .01, i >= 1350 ? .4 : 0));
		}
		JumpDetector::UniverseDelta delta;
		const bool accepted = detector.PollDelta(delta);
		if (hole)
			check("stream holes: a held headset step survives a hole", accepted &&
				std::abs(delta.translation.x() - .4) < 1e-8 && delta.devicesAgreeing == 2,
				"hole 6.5 s into the hold, controller follows at 12 s");
		else
			check("recovery: tracking loss during the hold revokes it", !accepted,
				"one invalid HMD frame 6.5 s in, controller follows at 12 s");
	}

	// The HMD candidate has finished fitting, then loses tracking before a
	// delayed controller reaches agreement. Ready evidence must expire too.
	{
		JumpDetector detector(QpcSeconds);
		for (int i = 0; i <= 124; ++i)
		{
			detector.Push(Sample(0, 1 + i * .01, i >= 100 ? .4 : 0));
			detector.Push(Sample(1, 1 + i * .01, i >= 120 ? .4 : 0));
		}
		auto invalid = Sample(0, 2.25, .4);
		invalid.poseIsValid = false;
		detector.Push(invalid);
		for (int i = 125; i < 160; ++i)
			detector.Push(Sample(1, 1 + i * .01, .4));
		JumpDetector::UniverseDelta delta;
		check("recovery: tracking loss revokes ready jump evidence", !detector.PollDelta(delta), "delayed agreement after invalid HMD frame");
	}
	{
		auto aligner = Aligner();
		Warm(aligner, .015);
		bool before = aligner.CorrectionEligible();
		Pair(aligner, 11.01, .165);
		CA::Correction correction;
		check("recovery: first suspected jump revokes queued correction",
			before && !aligner.CorrectionEligible() && !aligner.PollCorrection(correction), "do not wait for the second jump observation");
	}
	{
		auto aligner = Aligner();
		Warm(aligner, .015);
		Pair(aligner, 11.3, .03);
		CA::Correction correction;
		check("recovery: 300 ms tracking gap drops the old estimate",
			aligner.ObservationCount() < 2 && !aligner.CorrectionEligible() && !aligner.PollCorrection(correction), "post-gap samples must refill a fresh window");
	}
	{
		auto aligner = Aligner();
		Warm(aligner, .15);
		for (int i = 1101; i <= 1400; ++i)
			Pair(aligner, i * .01, .15);
		bool frozen = aligner.GetState() == CA::State::Frozen;
		aligner.Reset(CA::ResetReason::StreamGap);
		check("recovery: tracking loss does not clear a mount fault",
			frozen && aligner.GetState() == CA::State::Frozen, "fresh sustained recovery evidence remains necessary");
	}

	// Detailed logging: the detector's detail lines carry each candidate's
	// evidence and every drop reason, and only while asked for.
	{
		// A living headset (slow turn, sub-millimetre noise) that steps by
		// `step` m at frame 9000; `glitch` makes that a one-frame spike instead;
		// `holeAt` reports a driver queue drop at that frame.
		auto run = [](bool detailed, double step, bool glitch, int holeAt)
		{
			JumpDetector detector(QpcSeconds);
			detector.SetDetailed(detailed);
			std::vector<std::string> lines;
			for (int frame = 0; frame <= 9200; ++frame)
			{
				if (frame == holeAt)
				{
					detector.NoteStreamHole();
					continue;
				}
				double time = 1.0 + frame * 0.01;
				double shift = glitch ? (frame == 9000 ? step : 0.) : (frame >= 9000 ? step : 0.);
				auto s = Sample(0, time, shift);
				double yaw = 0.1 * time;
				s.rotation = { std::cos(yaw / 2), 0, std::sin(yaw / 2), 0 };
				s.position[2] = 1e-5 * std::sin(time * 700.0);
				detector.Push(s);
				JumpDetector::UniverseDelta delta;
				while (detector.PollDelta(delta)) {}
				std::string line;
				while (detector.PollNote(line)) {}
				while (detector.PollDetail(line))
					lines.push_back(line);
			}
			return lines;
		};

		auto quiet = run(false, .08, false, -1);
		check("detailed log: nothing gathered while off", quiet.empty(), "detail is opt-in");

		auto step = run(true, .08, false, -1);
		check("detailed log: a step's evidence is written",
			Has(step, "candidate on device 0 at") && Has(step, "frame jump 0.080 m") &&
			Has(step, "fitted: before (") && Has(step, "fit rms before") && Has(step, "samples around the step on device 0"),
			step.empty() ? "no lines" : step.back().substr(0, 120).c_str());

		auto spike = run(true, .08, true, -1);
		check("detailed log: a glitch's candidate says why it was dropped",
			Has(spike, "dropped: fit not clean"), spike.empty() ? "no lines" : spike.back().substr(0, 120).c_str());

		auto hole = run(true, .08, false, 9010);
		check("detailed log: a candidate lost to a driver queue drop says so",
			Has(hole, "dropped: a driver queue drop"), hole.empty() ? "no lines" : hole.back().substr(0, 120).c_str());
	}

	// The stream digest: one line per device a minute with what the
	// transport did. A 90 Hz headset for 61 s with every 13th frame
	// re-predicted (velocities repeated, position moved on), 20 frames of
	// held position under a moving heading, one 150 ms gap, two invalid
	// frames and one driver queue drop.
	{
		TrackingStreamDigest digest;
		digest.Flush(0.0);
		double time = 1.0;
		Eigen::Vector3d pos(0.3, 1.6, 0.4);
		protocol::DevicePoseSample previous{};
		int repredicted = 0;
		for (int frame = 0; frame < 61 * 90; ++frame)
		{
			time += frame == 3000 ? 0.15 : 1.0 / 90.0;
			auto s = Sample(0, time, 0.0);
			const double yaw = 0.2 * time;
			s.rotation = { std::cos(yaw / 2), 0, std::sin(yaw / 2), 0 };
			const bool held = frame >= 1000 && frame < 1020;
			if (!held)
				pos.x() += 0.001;
			for (int k = 0; k < 3; ++k)
				s.position[k] = pos[k];
			s.velocity[0] = 0.09 + 1e-4 * std::sin(frame * 1.3);
			s.angularVelocity[1] = 0.2 + 1e-4 * std::cos(frame * 0.7);
			if (frame % 13 == 0 && frame > 0 && !held && frame != 3000)
			{
				for (int k = 0; k < 3; ++k)
				{
					s.velocity[k] = previous.velocity[k];
					s.angularVelocity[k] = previous.angularVelocity[k];
				}
				++repredicted;
			}
			if (frame == 4000 || frame == 4001)
				s.poseIsValid = false;
			digest.Note(s, QpcSeconds);
			if (s.poseIsValid)
				previous = s;
		}
		digest.NoteDrops(1);
		auto lines = digest.Flush(61.0);
		const std::string line = lines.empty() ? std::string() : lines.front();
		char expect[64];
		snprintf(expect, sizeof expect, "re-predicted %d (", repredicted);
		const bool counts = line.find(expect) != std::string::npos &&
			line.find("position repeated 20 (20 under a moving heading)") != std::string::npos &&
			line.find("1 over 100 ms") != std::string::npos && line.find("2 invalid") != std::string::npos &&
			line.find("angular velocity alone repeated 0") != std::string::npos;
		check("detailed log: the stream digest counts what the transport did",
			lines.size() == 2 && counts && lines[1] == "driver queue drops over the last minute: 1 (1 pose(s))",
			line.substr(0, 200).c_str());
		check("detailed log: the digest waits a minute between lines", digest.Flush(90.0).empty(), "next line at 121 s");
	}

	// The lighthouse frame watch. Three base stations publish every 0.5 s
	// (identity local pose, worldFromDriver = their pose); the headset
	// tracker (9) walks and a second tracker (10) stands still, both in
	// station 1's frame, at 250 Hz for nine seconds. A universe move applies
	// one world delta (62 deg of yaw over 78.6 deg of tilt, as on 2026-09-24)
	// to every worldFromDriver and leaves the local poses on their paths.
	{
		using Pose = std::pair<Eigen::Quaterniond, Eigen::Vector3d>;
		const double deg = 3.14159265358979323846 / 180.0;
		const Pose identity(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
		auto compose = [](const Pose &a, const Pose &b)
		{
			return Pose((a.first * b.first).normalized(), a.first * b.second + a.second);
		};
		auto inverse = [](const Pose &a)
		{
			Eigen::Quaterniond r = a.first.conjugate();
			return Pose(r, -(r * a.second));
		};
		auto rot = [&](double yawDeg, double pitchDeg)
		{
			return Eigen::Quaterniond(Eigen::AngleAxisd(yawDeg * deg, Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(pitchDeg * deg, Eigen::Vector3d::UnitX()));
		};
		auto sample = [](uint32_t id, double time, const Pose &wfd, const Pose &local, const Eigen::Vector3d &vel)
		{
			auto s = Sample(id, time, 0.0);
			s.worldFromDriverRotation = { wfd.first.w(), wfd.first.x(), wfd.first.y(), wfd.first.z() };
			s.rotation = { local.first.w(), local.first.x(), local.first.y(), local.first.z() };
			for (int k = 0; k < 3; ++k)
			{
				s.worldFromDriverTranslation[k] = wfd.second[k];
				s.position[k] = local.second[k];
				s.velocity[k] = vel[k];
			}
			return s;
		};
		const Pose stations[3] = {
			Pose(rot(30, -30), Eigen::Vector3d(-2.6, 2.2, -0.3)),
			Pose(rot(150, -35), Eigen::Vector3d(1.4, 2.3, -3.9)),
			Pose(rot(-115, -25), Eigen::Vector3d(0.2, 2.1, 2.5)),
		};
		const Pose delta(rot(62, 78.6), Eigen::Vector3d(0.4, -1.1, 2.0));

		struct Result
		{
			std::vector<LighthouseFrameWatch::Report> reports;
			std::string line;
		};
		// `moves`: universe moves; `reexpressAt`: tracker 10 changes to station
		// 2's frame without moving; `jumpAt`: tracker 9's local pose jumps 30 cm.
		auto run = [&](const std::vector<double> &moves, double reexpressAt, double jumpAt)
		{
			LighthouseFrameWatch watch;
			Result out;
			const Eigen::Vector3d walk(0.2, 0.0, 0.1);
			double nextStation = 1.0;
			for (int i = 0; i <= 9 * 250; ++i)
			{
				const double time = 1.0 + i / 250.0;
				Pose world = identity;
				for (double at : moves)
					if (time >= at)
						world = compose(delta, world);
				Pose s[3];
				for (int k = 0; k < 3; ++k)
					s[k] = compose(world, stations[k]);

				Pose local9(rot(20, 0), Eigen::Vector3d(0.5, -1.0, 2.5) + walk * (time - 1.0));
				if (jumpAt >= 0.0 && time >= jumpAt)
					local9.second.x() += 0.3;
				watch.Note(sample(9, time, s[0], local9, walk), QpcSeconds, false);

				const Pose local10(Eigen::Quaterniond::Identity(), Eigen::Vector3d(-0.4, -1.8, 2.2));
				if (reexpressAt >= 0.0 && time >= reexpressAt)
					watch.Note(sample(10, time, s[1], compose(inverse(s[1]), compose(s[0], local10)),
						Eigen::Vector3d::Zero()), QpcSeconds, false);
				else
					watch.Note(sample(10, time, s[0], local10, Eigen::Vector3d::Zero()), QpcSeconds, false);

				if (time >= nextStation)
				{
					for (int k = 0; k < 3; ++k)
						watch.Note(sample(1 + k, time, s[k], identity, Eigen::Vector3d::Zero()), QpcSeconds, true);
					nextStation += 0.5;
				}
				for (const auto &r : watch.Flush())
				{
					out.reports.push_back(r);
					out.line = LighthouseFrameWatch::Describe(r, 9);
				}
			}
			return out;
		};

		auto still = run({}, -1.0, -1.0);
		check("lighthouse frame: nothing reported while no frame changes",
			still.reports.empty(), still.line.c_str());

		auto moved = run({ 5.0 }, -1.0, -1.0);
		bool whole = moved.reports.size() == 1;
		if (whole)
		{
			const auto &r = moved.reports[0];
			whole = r.moved == 5 && r.movedBaseStations == 3 && r.reExpressed == 0 && r.unclear == 0 &&
				std::abs(r.yawDeg - 62.0) < 0.01 && std::abs(r.tiltDeg - 78.6) < 0.01 &&
				r.spreadDeg < 1e-6 && r.spreadM < 1e-6 && r.largestShiftM > 1.0 &&
				moved.line.find("5 device(s) (3 base station(s), the headset tracker among them)") != std::string::npos;
		}
		check("lighthouse frame: a universe move is one report, every device, one delta",
			whole, moved.line.c_str());

		auto twice = run({ 3.0, 7.0 }, -1.0, -1.0);
		check("lighthouse frame: moves seconds apart are reported apart",
			twice.reports.size() == 2 && twice.reports[0].moved == 5 && twice.reports[1].moved == 5,
			twice.line.c_str());

		auto reexpressed = run({}, 5.0, -1.0);
		check("lighthouse frame: a pose put in another station's frame is not a move",
			reexpressed.reports.size() == 1 && reexpressed.reports[0].moved == 0 &&
			reexpressed.reports[0].reExpressed == 1 &&
			reexpressed.line.find("re-expressed their pose in another frame without moving") != std::string::npos,
			reexpressed.line.c_str());

		auto jumped = run({}, -1.0, 5.0);
		check("lighthouse frame: a tracker's own jump is not a frame change",
			jumped.reports.empty(), jumped.line.c_str());

		auto both = run({ 5.0 }, -1.0, 5.0);
		check("lighthouse frame: a tracker that jumps with the move is set apart",
			both.reports.size() == 1 && both.reports[0].moved == 4 && both.reports[0].unclear == 1 &&
			both.line.find("headset tracker") == std::string::npos &&
			both.line.find("1 whose own pose jumped as well") != std::string::npos,
			both.line.c_str());

		LighthouseFrameWatch::Report refined;
		refined.moved = refined.movedBaseStations = 1;
		refined.largestShiftM = 0.004;
		refined.yawDeg = 0.02;
		refined.tiltDeg = 0.01;
		check("lighthouse frame: a station refined by millimetres stays in the detailed log",
			!LighthouseFrameWatch::Notable(refined) &&
			!moved.reports.empty() && LighthouseFrameWatch::Notable(moved.reports[0]), "");
	}
}
