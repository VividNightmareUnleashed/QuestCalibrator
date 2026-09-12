#include "../Overlay/ContinuousAlignment.h"
#include "../Overlay/JumpDetector.h"

#include <cstdio>
#include <functional>

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

int Replay(const std::function<double(int, uint32_t)> &shift,
           const std::vector<uint32_t> &ids, double *translation = nullptr,
           int frames = 300, JumpDetector::UniverseDelta *last = nullptr)
{
	JumpDetector detector(QpcSeconds);
	int count = 0;
	for (int frame = 0; frame <= frames; ++frame)
	{
		for (auto id : ids)
			detector.Push(Sample(id, 1.0 + frame * 0.01, shift(frame, id)));
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
		count = Replay([](int f, uint32_t id) { return f >= 150 && id == 0 ? .4 : 0.; }, {0, 1}, nullptr, 3500);
		check("recovery: headset step no controller follows expires", count == 0, "controller continuous for 33 s");
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 1350) ? (id == 0 ? .4 : .25) : 0.; }, {0, 1}, nullptr, 3500);
		check("recovery: late controller step must match the headset step", count == 0, "15 cm disagreement 12 s later");
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 3300) ? .4 : 0.; }, {0, 1}, nullptr, 3600);
		check("recovery: controller step after the follow-up window does not confirm", count == 0, "31.5 s later");
		count = Replay([](int f, uint32_t id) { return f >= (id == 0 ? 150 : 1350) ? .15 : 0.; }, {0, 1}, &recovered, 3500);
		check("recovery: moderate headset step waits for the follow-up too", count == 1 && std::abs(recovered - .15) < 1e-8, "tight agreement, 12 s apart");
		count = Replay([](int f, uint32_t) { return f >= 150 ? .4 : 0.; }, {0}, nullptr, 3500);
		check("recovery: large solo step still applies at once", count == 1, "no controller to wait for");
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
