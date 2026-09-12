#include "../Overlay/ContinuousAlignment.h"
#include "../Overlay/JumpDetector.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
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

// A device whose position repeats bit for bit is, on the engine, a
// controller the static prior has locked still; `moving` adds the
// sub-millimetre motion a tracked 6DoF position always carries, for the
// cases that mean a tracking controller.
int Replay(const std::function<double(int, uint32_t)> &shift,
           const std::vector<uint32_t> &ids, double *translation = nullptr,
           int frames = 300, JumpDetector::UniverseDelta *last = nullptr,
           bool moving = false)
{
	JumpDetector detector(QpcSeconds);
	int count = 0;
	for (int frame = 0; frame <= frames; ++frame)
	{
		for (auto id : ids)
		{
			auto s = Sample(id, 1.0 + frame * 0.01, shift(frame, id));
			if (moving)
				s.position[2] = 1e-5 * std::sin(700.0 * (1.0 + frame * 0.01) + id);
			detector.Push(s);
		}
		JumpDetector::UniverseDelta delta;
		while (detector.PollDelta(delta))
		{
			++count;
			if (translation) *translation = delta.translation.x();
			if (last) *last = delta;
		}
	}
	return count;
}

// Headset-only replay with a living headset: a slow turn so the heading
// changes every frame, and (unless `heldPosition`) sub-millimetre SLAM noise
// so the position never repeats bit-for-bit. Frames in [skipFrom, skipTo)
// are not pushed, which the detector reads as a stream gap. Notes are
// drained into `log` when given.
int ReplaySolo(int frames, const std::function<double(int)> &shift,
               const std::function<double(int)> &yawStep, bool heldPosition,
               int skipFrom = -1, int skipTo = -1,
               std::vector<std::string> *log = nullptr,
               JumpDetector::UniverseDelta *last = nullptr)
{
	JumpDetector detector(QpcSeconds);
	int count = 0;
	for (int frame = 0; frame <= frames; ++frame)
	{
		if (frame >= skipFrom && frame < skipTo)
			continue;
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
	double recovered = 0;
	int count = Replay([](int f, uint32_t) { return f >= 150 ? .15 : 0.; }, {0, 1}, &recovered);
	check("recovery: 15 cm HMD/controller reset, no mounted tracker",
		count == 1 && std::abs(recovered - .15) < 1e-8, "matching persistent pose steps, unchanged WFD");

	count = Replay([](int f, uint32_t id) { return f >= 150 && id == 0 ? .15 : 0.; }, {0, 1});
	check("recovery: isolated moderate HMD error waits", count == 0, "no corroborating controller");
	count = Replay([](int f, uint32_t) { return f >= 150 ? .15 : 0.; }, {0});
	check("recovery: moderate reset never uses solo fallback", count == 0, "HMD only");
	count = Replay([](int f, uint32_t id) { return f >= 150 ? (id == 0 ? .15 : .21) : 0.; }, {0, 1});
	check("recovery: moderate reset needs tight agreement", count == 0, "6 cm disagreement");
	count = Replay([](int f, uint32_t id) { return f >= 150 && id != 0 ? .4 : 0.; }, {0, 1, 2});
	check("recovery: controllers cannot move the HMD universe", count == 0, "two controllers reset, HMD remains continuous");
	count = Replay([](int f, uint32_t) { return f == 150 ? .15 : 0.; }, {0, 1});
	check("recovery: shared one-frame glitch is not a reset", count == 0, "both streams immediately return");
	count = Replay([](int f, uint32_t) { return (f >= 100 ? .15 : 0.) + (f >= 230 ? -.12 : 0.); }, {0, 1}, &recovered);
	check("recovery: distinct moderate resets each apply once", count == 2 && std::abs(recovered + .12) < 1e-8, "no duplicate compensation");
	count = Replay([](int f, uint32_t) { return (f >= 100 ? .15 : 0.) + (f >= 160 ? -.12 : 0.); }, {0, 1});
	check("recovery: distinct resets inside one second", count == 2, "each has a clean window after the preceding reset");
	for (bool reset : {false, true})
	{
		JumpDetector detector(QpcSeconds);
		int accepted = 0;
		double translation = 0;
		for (int i = 0; i <= 300; ++i)
		{
			for (uint32_t id : {0u, 1u})
			{
				double captureTime = 1 + i * .01;
				double horizon = i >= 150 ? .06 - .01 * id : 0;
				double speed = 1.2 - .1 * id;
				auto s = Sample(id, captureTime, reset && i >= 150 ? .15 : 0);
				s.poseTimeOffset = horizon;
				s.position[0] += speed * (captureTime + horizon);
				s.velocity[0] = speed;
				detector.Push(s);
			}
			JumpDetector::UniverseDelta delta;
			while (detector.PollDelta(delta))
			{
				++accepted;
				translation = delta.translation.x();
			}
		}
		check(reset ? "recovery: reset survives a prediction horizon change"
		            : "recovery: prediction horizon change is not a reset",
			reset ? accepted == 1 && std::abs(translation - .15) < 1e-8 : accepted == 0,
			"different device speeds and horizons; pose validity time includes driver offset");
	}
	{
		JumpDetector detector(QpcSeconds);
		int accepted = 0;
		double error = 0;
		for (int i = 0; i <= 300; ++i)
		{
			double time = 1 + i * .01;
			for (uint32_t id : {0u, 1u})
			{
				auto s = Sample(id, time, i >= 150 ? .15 : 0.);
				s.position[0] += .1 * std::sin(3 * time + id) + .001 * std::sin(77 * time + id);
				s.velocity[0] = .3 * std::cos(3 * time + id);
				detector.Push(s);
			}
			JumpDetector::UniverseDelta delta;
			while (detector.PollDelta(delta))
			{
				++accepted;
				error = std::abs(delta.translation.x() - .15);
			}
		}
		check("recovery: moderate reset during independent device motion",
			accepted == 1 && error < .005, "different trajectories plus millimetre tracking noise");
	}
	{
		JumpDetector detector(QpcSeconds);
		int accepted = 0;
		for (int i = 0; i <= 300; ++i)
		{
			double angle = i >= 150 ? 3 * EIGEN_PI / 180 : 0;
			for (uint32_t id : {0u, 1u})
			{
				auto s = Sample(id, 1 + i * .01, 0);
				s.rotation = {std::cos(angle / 2), 0, std::sin(angle / 2), 0};
				s.position[2] = -std::sin(angle) * s.position[0];
				s.position[0] *= std::cos(angle);
				detector.Push(s);
			}
			JumpDetector::UniverseDelta delta;
			while (detector.PollDelta(delta)) ++accepted;
		}
		check("recovery: corroborated small yaw reset", accepted == 1, "3 degrees, shared rigid reference change");
	}

	// Quest Pro controllers are separate tracking frontends on the shared map;
	// after a headset map switch the engine lets them keep the previous frame
	// for up to 30 s. An HMD step nobody
	// confirmed at once is held for that follow-up, stamped with the HMD's
	// jump time when it arrives, and never applied alone while controllers
	// stay continuous.
	{
		JumpDetector::UniverseDelta delta;
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 1350) ? .4 : 0.; }, {0, 1}, &recovered, 3500, &delta);
		check("recovery: controller follows a headset map switch 12 s later",
			count == 1 && std::abs(recovered - .4) < 1e-8 && std::abs(delta.time - 2.5) < 1e-6 &&
			std::abs(delta.confirmationLagSeconds - 12.0) < 1e-6 && delta.devicesAgreeing == 2,
			"held HMD candidate confirmed by the late controller step; delta keeps the HMD jump time");
		count = Replay([](int f, uint32_t id) { return f >= 150 && id == 0 ? .4 : 0.; }, {0, 1}, nullptr, 3500, nullptr, true);
		check("recovery: headset step no controller follows expires", count == 0, "controller tracking and continuous for 33 s");
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 1350) ? (id == 0 ? .4 : .25) : 0.; }, {0, 1}, nullptr, 3500);
		check("recovery: late controller step must match the headset step", count == 0, "15 cm disagreement 12 s later");
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 3300) ? .4 : 0.; }, {0, 1}, nullptr, 3600, nullptr, true);
		check("recovery: controller step after the follow-up window does not confirm", count == 0, "tracking controller, 31.5 s later");

		// The engine's static prior locks a controller that lies still: its
		// position repeats bit for bit and it cannot step until the hand
		// moves. The follow-up clock pauses while every other device is
		// locked, so the same 31.5 s step confirms when the controller was
		// still, and a locked controller that never moves keeps the
		// candidate waiting rather than expiring it.
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 3300) ? .4 : 0.; }, {0, 1}, &recovered, 3600, &delta);
		check("recovery: a locked controller confirms the headset step when it moves", count == 1 && std::abs(recovered - .4) < 1e-8 &&
			std::abs(delta.confirmationLagSeconds - 31.5) < 1e-6, "position bit-identical for 33 s, then the step");
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 3300) ? (id == 0 ? .4 : .25) : 0.; }, {0, 1}, nullptr, 3600);
		check("recovery: a locked controller's later step must still match", count == 0, "15 cm disagreement after the lock");

		// The controller frontend's anchor follower has no grace: inside the
		// headset smoother's 5 s grace the controllers snap first and the
		// headset resets up to 5 s later. A matching controller step up to
		// 5 s ahead confirms the headset's, with a negative lag; further
		// ahead it does not.
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 450 : 150) ? .4 : 0.; }, {0, 1}, &recovered, 900, &delta, true);
		check("recovery: controller step 3 s ahead confirms the headset step", count == 1 && std::abs(recovered - .4) < 1e-8 &&
			std::abs(delta.time - 5.5) < 1e-6 && std::abs(delta.confirmationLagSeconds + 3.0) < 1e-6,
			"delta keeps the HMD jump time; lag reported as -3.0 s");
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 750 : 150) ? .4 : 0.; }, {0, 1}, nullptr, 1200, nullptr, true);
		check("recovery: controller step 6 s ahead does not confirm", count == 0, "beyond the 5 s grace");
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 450 : 150) ? (id == 0 ? .4 : .25) : 0.; }, {0, 1}, nullptr, 900, nullptr, true);
		check("recovery: leading controller step must match the headset step", count == 0, "15 cm disagreement 3 s ahead");
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 1350) ? .15 : 0.; }, {0, 1}, &recovered, 3500);
		check("recovery: moderate headset step waits for the follow-up too", count == 1 && std::abs(recovered - .15) < 1e-8, "tight agreement, 12 s apart");
		count = Replay([](int f, uint32_t) { return f >= 150 ? .4 : 0.; }, {0}, nullptr, 3500);
		check("recovery: large solo step still applies at once", count == 1, "no controller to wait for");
	}

	// Headset-only: below the solo floor a persistent HMD step is applied on
	// the fit alone once the stream has been steady for a minute and the
	// pre-window shows no held position (the engine's 3DoF fallback holds the
	// last tracked position bit-for-bit under a moving IMU heading).
	{
		auto none = [](int) { return 0.; };
		auto lateStep = [](int f) { return f >= 9000 ? .08 : 0.; };
		JumpDetector::UniverseDelta delta;
		std::vector<std::string> log;

		count = ReplaySolo(9200, lateStep, none, false, -1, -1, &log, &delta);
		bool noted = false;
		for (const auto &n : log)
			noted = noted || n.find("accepted alone") != std::string::npos;
		check("headset-only: 8 cm step after 90 s of steady tracking applies alone",
			count == 1 && std::abs(delta.translation.x() - .08) < 1e-6 && delta.devicesAgreeing == 1 && noted,
			"clean fit, stream steady, position never repeated");

		count = ReplaySolo(2300, [](int f) { return f >= 2000 ? .08 : 0.; }, none, false, -1, -1, &log);
		bool gated = false;
		for (const auto &n : log)
			gated = gated || n.find("stream steady under 60 s") != std::string::npos;
		check("headset-only: 8 cm step 20 s after the stream started is refused", count == 0 && gated,
			"a wake or a stream restart re-zeroes inside the first minute");

		count = ReplaySolo(11600, [](int f) { return f >= 11300 ? .08 : 0.; }, none, false, 9000, 9300);
		check("headset-only: 8 cm step 20 s after a stream gap is refused", count == 0, "3 s gap, then 20 s of stream");

		log.clear();
		count = ReplaySolo(9200, lateStep, none, true, -1, -1, &log);
		bool held = false;
		for (const auto &n : log)
			held = held || n.find("position was held before the step (3DoF)") != std::string::npos;
		check("headset-only: step after a held position is refused", count == 0 && held,
			"position bit-constant while the heading moved: the 3DoF-to-6DoF snap");

		count = ReplaySolo(9200, none, [](int f) { return f >= 9000 ? 3.0 * EIGEN_PI / 180.0 : 0.; }, false, -1, -1, nullptr, &delta);
		check("headset-only: 3 deg yaw step alone applies after steady tracking",
			count == 1 && std::abs(YawDegrees(delta.rotation) - 3.0) < 0.05, "yaw-only correction above the 2 deg floor");

		count = ReplaySolo(9200, [](int f) { return f >= 9000 ? .03 : 0.; }, none, false);
		check("headset-only: 3 cm step stays below the floor", count == 0, "under 5 cm and 2 deg nothing is applied");

		// What was refused is summed in the log, so a session's discarded
		// steps can be compared with the drift a recalibration removes.
		log.clear();
		count = ReplaySolo(1800, [](int f) { return (f >= 500 ? .08 : 0.) + (f >= 1500 ? .08 : 0.); }, none, false, -1, -1, &log);
		std::string total;
		for (const auto &n : log)
			if (n.find("ignored") != std::string::npos)
				total = n;
		check("headset-only: ignored steps are totalled in the log",
			count == 0 && total.find("2 ignored this session") != std::string::npos &&
			total.find("shift 0.160 m in total") != std::string::npos,
			total.c_str());
	}
	{
		JumpDetector detector(QpcSeconds);
		for (int i = 0; i <= 1400; ++i)
		{
			auto hmd = Sample(0, 1 + i * .01, i >= 150 ? .4 : 0);
			if (i == 800) hmd.poseIsValid = false;
			detector.Push(hmd);
			detector.Push(Sample(1, 1 + i * .01, i >= 1350 ? .4 : 0));
		}
		JumpDetector::UniverseDelta delta;
		check("recovery: tracking loss during the hold revokes it", !detector.PollDelta(delta), "one invalid HMD frame 6.5 s in, controller follows at 12 s");
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
}

#ifdef QUESTCAL_TRACKING_RECOVERY_STANDALONE
int main()
{
	static int failures = 0;
	RunTrackingRecoveryScenarios([](const char *name, bool pass, const char *detail)
	{
		std::printf("%s %s: %s\n", pass ? "PASS" : "FAIL", name, detail);
		failures += !pass;
	});
	return failures;
}
#endif
