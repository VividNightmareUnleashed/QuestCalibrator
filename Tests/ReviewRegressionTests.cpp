#include "../Driver/ServerTrackedDeviceProvider.h"
#include "../Overlay/ContinuousCorrectionGate.h"
#include "../Overlay/FieldMath.h"
#include "../Overlay/LegacyContinuous.h"
#include "../Overlay/PersistenceState.h"
#include "../Overlay/RingPoseMath.h"
#include "../Overlay/Updater.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

// Tests call the real provider without installing hooks into a VR runtime.
bool InjectHooks(ServerTrackedDeviceProvider *, vr::IVRDriverContext *) { return false; }
bool IsPoseUpdateHookInstalled() { return false; }
uint32_t PoseUpdateHookMask() { return 0; }
bool DisableHooks() { return true; }

namespace
{
using Check = void (*)(const char *, bool, const char *);

void RuntimeTransactionScenario(Check check)
{
	auto provider = std::make_unique<ServerTrackedDeviceProvider>();
	protocol::SetRuntimeState a, b;
	a.enabledMask = b.enabledMask = 1;
	a.field.enabled = b.field.enabled = 1;
	a.field.anchorCount = b.field.anchorCount = 1;
	a.field.anchors[0].translationDelta[0] = 0.1;
	b.transform.translation.v[0] = 0.2;
	b.field.anchors[0].position[0] = 0.2;
	b.field.anchors[0].translationDelta[0] = -0.1;
	a.transform.generation = a.field.generation = 1;
	b.transform.generation = b.field.generation = 2;
	auto read = [&]() {
		vr::DriverPose_t pose{};
		pose.poseIsValid = pose.deviceIsConnected = true;
		pose.result = vr::TrackingResult_Running_OK;
		pose.qRotation.w = pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = 1;
		provider->HandleDevicePoseUpdated(0, pose);
		return pose.vecWorldFromDriverTranslation[0];
	};
	bool accepted = provider->TrySetRuntimeState(a);
	double expectedA = read();
	accepted &= provider->TrySetRuntimeState(b);
	double expectedB = read();
	std::atomic<bool> done{ false };
	std::atomic<unsigned> mixed{ 0 }, samples{ 0 };
	auto observe = [&]() {
		do
		{
			double x = read();
			if (std::abs(x - expectedA) > 1e-10 && std::abs(x - expectedB) > 1e-10)
				++mixed;
			++samples;
		} while (!done.load());
	};
	std::thread reader(observe);
	std::thread writer([&]() {
		for (unsigned i = 0; i < 20000; ++i)
			accepted &= provider->TrySetRuntimeState((i & 1) ? a : b);
		done.store(true);
	});
	observe();
	writer.join();
	reader.join();
	char detail[128];
	snprintf(detail, sizeof detail, "%u mixed snapshots in %u callbacks", mixed.load(), samples.load());
	check("driver: base and field publish together", accepted && mixed == 0 && samples > 0, detail);

	protocol::SetDeviceTransform disabled;
	disabled.openVRID = 0;
	disabled.hidden = 1;
	check("driver: individual slot update survives runtime publication",
		provider->TrySetDeviceTransform(disabled) && std::abs(read()) < 1e-12, "");
}

void CorrectionWithdrawalScenarios(Check check)
{
	using namespace questcal;
	for (bool freeze : { false, true })
	{
		ContinuousAlignment engine;
		MountExtrinsic mount;
		mount.valid = true;
		engine.SetExtrinsic(mount);
		ContinuousCorrectionGate gate;
		bool offered = false, inspected = false, retainedBetweenDecisions = false;
		bool safe = false;
		for (int i = 1; i <= 3000; ++i)
		{
			double t = i * 0.02;
			double offset = freeze ? std::min(0.08, 0.02 + std::max(0.0, t - 8) * 0.002)
				: std::max(0.0, 0.02 - std::max(0.0, t - 8) * 0.000625);
			PoseSample a, b;
			a.time = b.time = 1000 + t;
			a.pos = Eigen::Vector3d(offset, 1.6, 0);
			b.pos = Eigen::Vector3d(0, 1.6, 0);
			engine.PushReference(a);
			engine.PushTarget(b);
			engine.Update(a.time, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 1, 0);
			ContinuousAlignment::Correction correction;
			bool received = engine.PollCorrection(correction);
			if (!engine.CorrectionEligible())
				gate.Clear();
			else if (received)
			{
				gate.Offer(correction, false);
				offered = true;
			}
			else if (gate.HasPending())
				retainedBetweenDecisions = true;
			auto deviation = engine.CurrentDeviation();
			if (freeze ? deviation.valid && deviation.posM >= 0.05 : t >= 60)
			{
				inspected = true;
				safe = engine.GetState() == ContinuousAlignment::State::Tracking &&
					!engine.CorrectionEligible() && !gate.Take(true, true, correction);
				break;
			}
		}
		check(freeze ? "continuous: withdraw while confirming freeze" : "continuous: withdraw settled correction",
			offered && retainedBetweenDecisions && inspected && safe, "");
	}
}

bool ObserveCorrection(const Eigen::Vector3d &head, double yawDegrees,
	const Eigen::Vector3d &displacement, questcal::ContinuousAlignment::Correction &correction)
{
	questcal::ContinuousAlignment engine;
	questcal::MountExtrinsic mount;
	mount.valid = true;
	engine.SetExtrinsic(mount);
	const Eigen::Quaterniond desired(Eigen::AngleAxisd(yawDegrees * EIGEN_PI / 180.0,
		Eigen::Vector3d::UnitY()));
	for (int i = 0; i < 1000; ++i)
	{
		questcal::PoseSample h, t;
		h.time = t.time = 1000 + i * 0.02;
		h.pos = head;
		t.pos = head - desired.conjugate() * displacement;
		t.rot = desired.conjugate();
		engine.PushReference(h);
		engine.PushTarget(t);
		engine.Update(h.time, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 1, 0);
		if (engine.PollCorrection(correction))
			return true;
	}
	return false;
}

Eigen::Vector3d MappedPose(const vr::DriverPose_t &pose)
{
	const auto &q = pose.qWorldFromDriverRotation;
	return Eigen::Quaterniond(q.w, q.x, q.y, q.z) * Eigen::Map<const Eigen::Vector3d>(pose.vecPosition)
		+ Eigen::Map<const Eigen::Vector3d>(pose.vecWorldFromDriverTranslation);
}

void ContinuousDriverSlewScenarios(Check check)
{
	for (bool moving : { false, true })
	{
		bool valid = true;
		double excursion = 0, largestStep = 0, finalError = 0, angularStep = 0;
		for (double distance : { 1.0, 3.0, 5.0, 10.0 })
		for (double scale : { 0.8, 1.0, 1.2 })
		{
			const Eigen::Vector3d head(distance, 1.6, 0);
			questcal::ContinuousAlignment::Correction correction;
			valid &= ObserveCorrection(head, 0.5, Eigen::Vector3d::Zero(), correction);
			auto provider = std::make_unique<ServerTrackedDeviceProvider>();
			protocol::SetRuntimeState runtime;
			runtime.enabledMask = 1;
			runtime.transform.generation = 1;
			runtime.transform.scale = scale;
			valid &= provider->TrySetRuntimeState(runtime);
			const Eigen::Quaterniond originR(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()));
			const Eigen::Vector3d originT(2, 0.3, -1);
			auto rawPose = [&](const Eigen::Vector3d &point) {
				vr::DriverPose_t pose{};
				pose.poseIsValid = pose.deviceIsConnected = true;
				pose.result = vr::TrackingResult_Running_OK;
				pose.qRotation.w = pose.qDriverFromHeadRotation.w = 1;
				pose.qWorldFromDriverRotation = { originR.w(), originR.x(), originR.y(), originR.z() };
				const Eigen::Vector3d local = originR.conjugate() * (point / scale - originT);
				for (int k = 0; k < 3; ++k)
				{
					pose.vecPosition[k] = local[k];
					pose.vecWorldFromDriverTranslation[k] = originT[k];
				}
				return pose;
			};
			provider->SetPoseTimeForTest(0);
			auto firstBad = rawPose(head);
			firstBad.vecPosition[0] = std::numeric_limits<double>::quiet_NaN();
			provider->HandleDevicePoseUpdated(0, firstBad);
			auto pose = rawPose(head);
			provider->HandleDevicePoseUpdated(0, pose);
			const auto &r = correction.rotation;
			runtime.transform.rotation = { r.w(), r.x(), r.y(), r.z() };
			for (int k = 0; k < 3; ++k)
				runtime.transform.translation.v[k] = correction.translation[k];
			valid &= provider->TrySetRuntimeState(runtime);
			Eigen::Quaterniond previousR = Eigen::Quaterniond::Identity();
			Eigen::Vector3d previousT = Eigen::Vector3d::Zero();
			constexpr double dt = 1.0 / 90.0;
			for (int frame = 1; frame <= 270; ++frame)
			{
				Eigen::Vector3d point = head;
				if (moving)
					point.z() += 0.2 * std::sin(frame * dt);
				const Eigen::Vector3d before = previousR * point + previousT;
				pose = rawPose(point);
				provider->SetPoseTimeForTest(frame * dt);
				provider->HandleDevicePoseUpdated(0, pose);
				const Eigen::Vector3d mapped = MappedPose(pose);
				const auto &q = pose.qWorldFromDriverRotation;
				const Eigen::Quaterniond applied = Eigen::Quaterniond(q.w, q.x, q.y, q.z) * originR.conjugate();
				largestStep = std::max(largestStep, (mapped - before).norm());
				angularStep = std::max(angularStep, applied.angularDistance(previousR));
				previousR = applied;
				previousT = mapped - applied * point;
				if (!moving)
					excursion = std::max(excursion, (mapped - head).norm());
				if (frame >= 100)
					finalError = std::max(finalError, (mapped - (r * point + correction.translation)).norm()
						+ applied.angularDistance(r));
			}
			// A bad pose must not poison persistent slew state, including on first use.
			for (int badFrame : { 1, 2, 3 })
			{
				pose = rawPose(head);
				pose.poseIsValid = badFrame != 1;
				pose.vecPosition[0] = badFrame == 3 ? 1e300 : std::numeric_limits<double>::quiet_NaN();
				provider->SetPoseTimeForTest(3 + badFrame * dt);
				provider->HandleDevicePoseUpdated(0, pose);
			}
			pose = rawPose(head);
			provider->SetPoseTimeForTest(3 + 4 * dt);
			provider->HandleDevicePoseUpdated(0, pose);
			valid &= MappedPose(pose).allFinite();
		}
		char detail[192];
		snprintf(detail, sizeof detail, "excursion %.6f mm, step %.6f mm, settled error %.3e",
			excursion * 1000, largestStep * 1000, finalError);
		check(moving ? "continuous driver: moving point respects slew limits" : "continuous driver: yaw keeps head position fixed",
			valid && excursion < 1e-9 && finalError < 1e-8 &&
			largestStep <= alignfield::BaseSlewLimits.maxTranslationPerSec / 90 + 1e-10 &&
			angularStep <= alignfield::BaseSlewLimits.maxRotationPerSec / 90 + 1e-9, detail);
	}
}

void ContinuousHeadClampScenario(Check check)
{
	bool valid = true;
	double worst = 0;
	for (double distance : { 3.0, 10.0, 1000.0 })
	{
		const Eigen::Vector3d head(distance, 1.6, 0), displacement(0, 0, 0.02);
		questcal::ContinuousAlignment::Correction correction;
		valid &= ObserveCorrection(head, 1.2, displacement, correction);
		const Eigen::Vector3d actual = correction.rotation * head + correction.translation - head;
		worst = std::max(worst, (actual - (0.5 / 1.2) * displacement).norm());
		valid &= actual.norm() <= 0.01 + 1e-10;
	}
	char detail[128];
	snprintf(detail, sizeof detail, "worst displacement error %.6f mm", worst * 1000);
	check("continuous: fractional yaw respects head displacement", valid && worst < 1e-10, detail);
}

void FieldPivotSlewScenario(Check check)
{
	double worst = 0;
	for (double distance : { 1.0, 3.0, 10.0, 1000.0 })
	{
		const Eigen::Vector3d point(distance, 1.6, 0);
		const Eigen::Quaterniond q(Eigen::AngleAxisd(3 * EIGEN_PI / 180.0, Eigen::Vector3d::UnitY()));
		const Eigen::Vector3d translation = point - q * point;
		protocol::SetAlignmentField field;
		field.enabled = 1;
		field.generation = 1;
		field.anchorCount = 1;
		auto &anchor = field.anchors[0];
		for (int k = 0; k < 3; ++k)
			anchor.position[k] = point[k];
		double p[3] = { point.x(), point.y(), point.z() };
		alignfield::EvalState state;
		alignfield::Evaluate(field, p, 0, state);
		anchor.rotationDelta = { q.w(), q.x(), q.y(), q.z() };
		for (int k = 0; k < 3; ++k)
			anchor.translationDelta[k] = translation[k];
		for (int frame = 1; frame <= 90; ++frame)
		{
			alignfield::Evaluate(field, p, frame / 90.0, state);
			const auto &r = state.rot;
			const Eigen::Vector3d mapped = Eigen::Quaterniond(r.w, r.x, r.y, r.z) * point
				+ Eigen::Map<const Eigen::Vector3d>(state.trans);
			worst = std::max(worst, (mapped - point).norm());
		}
	}
	char detail[128];
	snprintf(detail, sizeof detail, "worst pivot excursion %.6f mm", worst * 1000);
	check("field: slew preserves pivot position", worst < 1e-9, detail);
}

struct Anchor
{
	Eigen::Vector3d position;
	Eigen::Quaterniond rotation;
	Eigen::Vector3d translationMeters;
};

void FieldRebaseScenario(Check check)
{
	using namespace questcal;
	double worst = 0;
	for (double degrees : { 1.0, 5.0, 8.0 })
	for (double scale : { 0.8, 1.0, 1.2 })
	for (double yaw : { 0.0, 1.7 })
	{
		const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
		const Eigen::Quaterniond dR(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()));
		const Eigen::Vector3d dT(2, 0.4, -1), zero = Eigen::Vector3d::Zero();
		std::vector<Anchor> anchors;
		for (int sign : { -1, 1 })
			anchors.push_back({ Eigen::Vector3d(sign * 0.6, 0, 0),
				Eigen::Quaterniond(Eigen::AngleAxisd(sign * degrees * EIGEN_PI / 180.0,
					Eigen::Vector3d::UnitY())), zero });
		for (const Eigen::Vector3d &raw : { zero, Eigen::Vector3d(0.2, 1.6, -0.4), Eigen::Vector3d(8, 1, 9) })
		{
			Eigen::Vector3d query = scale * raw;
			Eigen::Quaterniond oldR, newR;
			Eigen::Vector3d oldT, newT;
			BlendedFieldCalibration(anchors, identity, zero, query, oldR, oldT);
			auto rebased = anchors;
			protocol::SetAlignmentField field;
			field.enabled = 1;
			field.anchorCount = static_cast<uint32_t>(rebased.size());
			for (size_t i = 0; i < rebased.size(); ++i)
			{
				auto &a = rebased[i];
				a.position = dR * a.position + dT;
				a.rotation = dR * a.rotation;
				a.translationMeters = dR * a.translationMeters + dT;
				Eigen::Quaterniond r;
				Eigen::Vector3d t;
				AnchorDelta(a.rotation, a.translationMeters, dR.conjugate(), dT, r, t);
				field.anchors[i].rotationDelta = { r.w(), r.x(), r.y(), r.z() };
				for (int k = 0; k < 3; ++k)
				{
					field.anchors[i].position[k] = a.position[k];
					field.anchors[i].translationDelta[k] = t[k];
				}
			}
			Eigen::Vector3d newQuery = dR * query + dT;
			BlendedFieldCalibration(rebased, dR, dT, newQuery, newR, newT);
			Eigen::Vector3d expected = dR * (oldR * query + oldT) + dT;
			worst = std::max(worst, (newR * query + newT - expected).norm());
			worst = std::max(worst, newR.angularDistance(dR * oldR));
			double p[3] = { newQuery.x(), newQuery.y(), newQuery.z() }, t[3];
			vr::HmdQuaternion_t r;
			alignfield::BlendAt(field, p, r, t);
			Eigen::Quaterniond driverR(r.w, r.x, r.y, r.z);
			Eigen::Vector3d driverPoint = driverR * newQuery + Eigen::Vector3d(t[0], t[1], t[2]);
			worst = std::max(worst, (driverPoint - expected).norm());
		}
	}
	char detail[128];
	snprintf(detail, sizeof detail, "worst rebase error %.3e (meters/radians)", worst);
	check("field: rebase preserves physical alignment", worst < 1e-12, detail);
}

void FractionalLatencyGridScenario(Check check)
{
	using namespace questcal;
	double worst = 0;
	bool valid = true;
	for (double truth : { 0.0168, -0.0168, 0.0008 })
	{
		std::vector<PoseSample> ref, target;
		auto angle = [](double t) {
			return 0.4 * std::sin(2.1 * t) + 0.25 * std::sin(4.7 * t) + 0.15 * std::sin(8.3 * t);
		};
		for (int i = 0; i <= 8000; ++i)
		{
			PoseSample a, b;
			a.time = b.time = i * 0.001;
			a.rot = Eigen::Quaterniond(Eigen::AngleAxisd(angle(a.time), Eigen::Vector3d::UnitY()));
			b.rot = Eigen::Quaterniond(Eigen::AngleAxisd(angle(b.time - truth), Eigen::Vector3d::UnitY()));
			ref.push_back(a);
			target.push_back(b);
		}
		for (double range : { 0.060, 0.061, 0.062, 0.063 })
		{
			EngineConfig config;
			config.timeOffsetStep = 0.004;
			config.timeOffsetRange = range;
			double offset = 0;
			valid &= CalibrationEngine::EstimateTimeOffset(ref, target, config, offset);
			worst = std::max(worst, std::abs(offset - truth));
		}
	}
	char detail[128];
	snprintf(detail, sizeof detail, "worst latency error %.6f ms", worst * 1000);
	check("latency: refine actual grid peak", valid && worst < 1e-5, detail);
}

void MixedVelocityLatencyScenario(Check check)
{
	using namespace questcal;
	bool valid = true;
	double worst = 0;
	for (double refRate : { 30.0, 72.0, 90.0 })
	for (double targetRate : { 90.0, 120.0 })
	for (int reportedMask : { 0, 1, 2, 3 })
	for (double latency : { -0.018, 0.0, 0.018 })
	{
		auto stream = [](double rate, bool reported, double offset) {
			std::vector<PoseSample> result;
			for (int i = 0; i <= static_cast<int>(rate * 12); ++i)
			{
				PoseSample sample;
				sample.time = i / rate + 0.15 / rate * std::sin(0.7 * i);
				double t = sample.time - offset;
				double angle = 0.4 * std::sin(2.1 * t) + 0.25 * std::sin(4.7 * t) + 0.15 * std::sin(8.3 * t);
				sample.rot = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));
				if (reported)
					sample.angVel.y() = 0.84 * std::cos(2.1 * t) + 1.175 * std::cos(4.7 * t) + 1.245 * std::cos(8.3 * t);
				result.push_back(sample);
			}
			return result;
		};
		const auto ref = stream(refRate, (reportedMask & 1) != 0, 0);
		const auto target = stream(targetRate, (reportedMask & 2) != 0, latency);
		EngineConfig config;
		double solved = 0;
		valid &= CalibrationEngine::EstimateTimeOffset(ref, target, config, solved);
		worst = std::max(worst, std::abs(solved - latency));
	}
	char detail[128];
	snprintf(detail, sizeof detail, "72 mixed-rate/source cases, worst timing bias %.6f ms", worst * 1000);
	check("latency: derived speeds use interval timestamps", valid && worst < 0.0005, detail);
}

void SettingsPartialCommitScenario(Check check)
{
	bool valid = true;
	for (bool coupled : { false, true })
	for (bool profileSucceeds : { false, true })
	for (bool settingsSucceeds : { false, true })
	{
		questcal::PersistenceState persistence;
		persistence.MarkProfileAndSettings(1);
		persistence.coupled = coupled;
		bool settingsWritten = false;
		std::string order;
		const auto result = persistence.SaveSettings([&]() {
			order += 'P';
			if (profileSucceeds)
				persistence.profileDirty = false;
			return profileSucceeds;
		}, [&]() {
			order += 'S';
			if (settingsSucceeds)
			{
				settingsWritten = true;
				persistence.settingsDirty = false;
			}
			return settingsSucceeds;
		});
		const bool blocked = coupled && !profileSucceeds;
		valid &= result.profileSaved == profileSucceeds &&
			result.settingsSaved == settingsWritten &&
			result.settingsSaved == (!blocked && settingsSucceeds) &&
			result.AllSaved() == (profileSucceeds && settingsSucceeds) &&
			persistence.profileDirty == !profileSucceeds &&
			persistence.settingsDirty == !settingsWritten &&
			order == (blocked ? "P" : "PS");
	}
	check("settings: partial commit retains its own result", valid, "8 independent/coupled write outcomes");
}

void ConnectedPoseTrustScenario(Check check)
{
	protocol::DevicePoseSample sample;
	sample.sampleTimeQpc = 1000;
	sample.rotation.w = sample.worldFromDriverRotation.w = 1;
	bool valid = true;
	for (bool connected : { false, true })
	for (bool tracked : { false, true })
	for (bool poseValid : { false, true })
	{
		sample.deviceIsConnected = connected;
		sample.poseIsValid = poseValid;
		sample.trackingResult = tracked ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
		valid &= IsTrustedRingSample(sample, 0.001) == (connected && tracked && poseValid);
	}
	check("pose trust: connection required with tracking flags", valid, "8 connected/valid/tracking combinations");
}

void FrozenRecoveryBoundaryScenario(Check check)
{
	using namespace questcal;
	ContinuousAlignment engine;
	MountExtrinsic mount;
	mount.valid = true;
	engine.SetExtrinsic(mount);
	bool frozen = false, stayedFrozen = true, resumed = false;
	for (int i = 1; i <= 4000; ++i)
	{
		double time = i * 0.02;
		PoseSample h, t;
		h.time = t.time = 1000 + time;
		t.pos = Eigen::Vector3d(0, 1.6, 0);
		h.pos = t.pos + Eigen::Vector3d(time < 20 ? 0.08 : time < 55 ? 0.03 : 0.01, 0, 0);
		engine.PushReference(h);
		engine.PushTarget(t);
		engine.Update(h.time, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 1, 0);
		ContinuousAlignment::Correction correction;
		bool emitted = engine.PollCorrection(correction);
		if (time > 15 && time < 20)
			frozen |= engine.GetState() == ContinuousAlignment::State::Frozen;
		if (time > 35 && time < 55)
			stayedFrozen &= engine.GetState() == ContinuousAlignment::State::Frozen && !emitted;
		if (time > 70)
			resumed |= engine.GetState() == ContinuousAlignment::State::Tracking;
	}
	check("continuous: clean 3 cm disagreement stays frozen", frozen && stayedFrozen && resumed,
		"after 8 cm freeze: clean 3 cm holds; 1 cm permits confirmed resume");
}

void LegacyTranslationScenario(Check check)
{
	using namespace questcal::legacy;
	double worst = 0;
	bool valid = true;
	for (int count : { 24, 60 })
	for (double noise : { 0.0, 0.001 })
	{
		CalibrationCalc calc;
		calc.Clear();
		std::vector<Sample> samples;
		Eigen::Quaterniond world(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
		for (int i = 0; i < count; ++i)
		{
			double t = i * 0.3;
			Eigen::Quaterniond r(Eigen::AngleAxisd(0.7 * std::sin(t), Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(0.6 * std::cos(1.3 * t), Eigen::Vector3d::UnitX()));
			Eigen::Vector3d p(0.1 * std::sin(t), 1.5, 0.2 * std::cos(t));
			Eigen::Vector3d target = world.conjugate() * (p + r * Eigen::Vector3d(0, 0.15, 0)
				- Eigen::Vector3d(0.2, 0.05, -0.1));
			target += noise * Eigen::Vector3d(std::sin(7 * t), std::cos(9 * t), std::sin(11 * t));
			samples.emplace_back(Pose(r, p), Pose(world.conjugate() * r, target), t);
			calc.PushSample(samples.back());
		}
		valid &= calc.ComputeOneshot(false);
		const Eigen::Matrix3d rotation = calc.Transformation().rotation();
		// Independent all-pairs oracle, retaining the original objective.
		Eigen::MatrixXd coefficients(3 * count * (count - 1), 3);
		Eigen::VectorXd constants(coefficients.rows());
		int row = 0;
		for (int i = 0; i < count; ++i)
		for (int j = 0; j < i; ++j)
		for (int family = 0; family < 2; ++family)
		{
			Eigen::Matrix3d qi = samples[i].ref.rot.transpose();
			Eigen::Matrix3d qj = samples[j].ref.rot.transpose();
			if (family == 1)
			{
				qi = (rotation * samples[i].target.rot).transpose();
				qj = (rotation * samples[j].target.rot).transpose();
			}
			coefficients.block<3, 3>(row, 0) = qj - qi;
			constants.segment<3>(row) = qj * (samples[j].ref.trans - rotation * samples[j].target.trans)
				- qi * (samples[i].ref.trans - rotation * samples[i].target.trans);
			row += 3;
		}
		Eigen::Vector3d expected = coefficients.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
		worst = std::max(worst, (expected - calc.Transformation().translation()).norm());
	}
	char detail[128];
	snprintf(detail, sizeof detail, "worst difference from all-pairs solve %.3e m", worst);
	check("legacy: centered solve preserves all-pairs objective", valid && worst < 1e-10, detail);
}

void UpdaterRestartScenario(Check check)
{
	using namespace questcal::update;
	for (bool reenable : { false, true })
	{
		std::mutex mutex;
		std::condition_variable changed;
		unsigned calls = 0;
		bool release = false, completed = false;
		Updater updater([&]() {
			std::unique_lock<std::mutex> lock(mutex);
			++calls;
			changed.notify_all();
			if (calls == 1)
				changed.wait(lock, [&]() { return release; });
			return std::string("[]");
		});
		updater.SetLogSink([&](const std::string &message) {
			if (message.find("check completed:") == 0)
			{
				std::lock_guard<std::mutex> lock(mutex);
				completed = true;
				changed.notify_all();
			}
		});
		updater.SetEnabled(true);
		bool started;
		{
			std::unique_lock<std::mutex> lock(mutex);
			started = changed.wait_for(lock, std::chrono::seconds(5), [&]() { return calls > 0; });
		}
		updater.SetEnabled(false);
		if (reenable)
		{
			// Several toggles coalesce into one fresh request after cancellation.
			for (int i = 0; i < 3; ++i)
			{
				updater.SetEnabled(true);
				updater.SetEnabled(false);
			}
			updater.SetEnabled(true);
		}
		{
			std::lock_guard<std::mutex> lock(mutex);
			release = true;
			changed.notify_all();
		}
		bool refreshed = false;
		if (reenable)
		{
			std::unique_lock<std::mutex> lock(mutex);
			refreshed = changed.wait_for(lock, std::chrono::seconds(5), [&]() { return completed; });
		}
		const auto state = updater.GetSnapshot().state;
		updater.Shutdown();
		check(reenable ? "updates: re-enable queues a fresh check" : "updates: disabled check cannot publish",
			started && (reenable ? refreshed && calls == 2 && state == State::UpToDate
				: !completed && calls == 1 && state == State::Disabled), "");
	}
}
} // namespace

void RunReviewRegressionScenarios(Check check)
{
	RuntimeTransactionScenario(check);
	CorrectionWithdrawalScenarios(check);
	ContinuousDriverSlewScenarios(check);
	ContinuousHeadClampScenario(check);
	FieldPivotSlewScenario(check);
	FieldRebaseScenario(check);
	FractionalLatencyGridScenario(check);
	MixedVelocityLatencyScenario(check);
	SettingsPartialCommitScenario(check);
	ConnectedPoseTrustScenario(check);
	FrozenRecoveryBoundaryScenario(check);
	LegacyTranslationScenario(check);
	UpdaterRestartScenario(check);
}
