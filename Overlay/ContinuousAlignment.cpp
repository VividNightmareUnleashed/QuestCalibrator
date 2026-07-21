#include "ContinuousAlignment.h"

#include <algorithm>
#include <cmath>

namespace questcal
{

namespace
{

constexpr double RadToDeg = 180.0 / EIGEN_PI;

// Hemisphere-safe quaternion mean (Markley eigenvector method): the maximal
// eigenvector of M = sum q q^T. The outer product is invariant under q -> -q,
// so the double cover needs no bookkeeping.
Eigen::Quaterniond EigenvectorMean(const std::vector<Eigen::Quaterniond> &quats,
                                   const std::vector<char> &keep)
{
	Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
	for (size_t i = 0; i < quats.size(); ++i)
	{
		if (!keep[i])
			continue;
		Eigen::Vector4d v = quats[i].coeffs();   // (x, y, z, w)
		M += v * v.transpose();
	}
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(M);
	Eigen::Vector4d ev = es.eigenvectors().col(3);   // eigenvalues ascend
	Eigen::Quaterniond mean(ev(3), ev(0), ev(1), ev(2));
	mean.normalize();
	return mean;
}

double Median(std::vector<double> values)
{
	if (values.empty())
		return 0.0;
	size_t mid = values.size() / 2;
	std::nth_element(values.begin(), values.begin() + mid, values.end());
	return values[mid];
}

struct RobustStats
{
	Eigen::Quaterniond rot{ 1, 0, 0, 0 };
	Eigen::Vector3d trans{ 0, 0, 0 };
	double rotRmsDeg = 0.0;
	double posRmsM = 0.0;
	size_t kept = 0;
};

// Eigenvector rotation mean + arithmetic translation mean, with one trim pass:
// samples beyond max(3 x median residual, floor) on either component are
// dropped and the means recomputed. The RMS is over the kept samples.
bool RobustAverage(const std::vector<Eigen::Quaterniond> &quats,
                   const std::vector<Eigen::Vector3d> &vecs,
                   double rotFloorDeg, double posFloorM, RobustStats &out)
{
	const size_t n = quats.size();
	if (n < 3)
		return false;

	std::vector<char> keep(n, 1);
	Eigen::Quaterniond qMean;
	Eigen::Vector3d tMean;
	std::vector<double> rotRes(n), posRes(n);

	for (int pass = 0; pass < 2; ++pass)
	{
		qMean = EigenvectorMean(quats, keep);
		tMean = Eigen::Vector3d::Zero();
		size_t kept = 0;
		for (size_t i = 0; i < n; ++i)
		{
			if (!keep[i])
				continue;
			tMean += vecs[i];
			++kept;
		}
		if (kept < 3)
			return false;
		tMean /= static_cast<double>(kept);

		for (size_t i = 0; i < n; ++i)
		{
			rotRes[i] = quats[i].angularDistance(qMean) * RadToDeg;
			posRes[i] = (vecs[i] - tMean).norm();
		}

		if (pass == 0)
		{
			std::vector<double> keptRot, keptPos;
			keptRot.reserve(n);
			keptPos.reserve(n);
			for (size_t i = 0; i < n; ++i)
			{
				keptRot.push_back(rotRes[i]);
				keptPos.push_back(posRes[i]);
			}
			double rotGate = std::max(3.0 * Median(keptRot), rotFloorDeg);
			double posGate = std::max(3.0 * Median(keptPos), posFloorM);
			for (size_t i = 0; i < n; ++i)
				keep[i] = rotRes[i] <= rotGate && posRes[i] <= posGate;
		}
	}

	double rotSq = 0.0, posSq = 0.0;
	size_t kept = 0;
	for (size_t i = 0; i < n; ++i)
	{
		if (!keep[i])
			continue;
		rotSq += rotRes[i] * rotRes[i];
		posSq += posRes[i] * posRes[i];
		++kept;
	}
	if (kept < 3)
		return false;

	out.rot = qMean;
	out.trans = tMean;
	out.rotRmsDeg = std::sqrt(rotSq / static_cast<double>(kept));
	out.posRmsM = std::sqrt(posSq / static_cast<double>(kept));
	out.kept = kept;
	return true;
}

} // namespace

void ContinuousAlignment::PushReference(const PoseSample &s)
{
	// Per-device ring order is monotonic; drop the odd inversion rather than
	// hand InterpolateAt an unsorted stream.
	if (!refWindow.empty() && s.time <= refWindow.back().time)
		return;
	refWindow.push_back(s);
}

void ContinuousAlignment::PushTarget(const PoseSample &s)
{
	if (!targetWindow.empty() && s.time <= targetWindow.back().time)
		return;
	targetWindow.push_back(s);
}

void ContinuousAlignment::FormObservations(double calScale, double calTimeOffset)
{
	for (; targetProcessed < targetWindow.size(); ++targetProcessed)
	{
		const PoseSample &t = targetWindow[targetProcessed];

		// Speed gates on both sides: residual timing error converts speed
		// directly into observation error through the mount lever arm.
		if (t.angVel.norm() > config.maxAngularSpeed || t.vel.norm() > config.maxLinearSpeed)
			continue;

		// Same time-alignment convention as the engine: the reference stream
		// is interpolated at the target's timestamp minus the solved offset.
		PoseSample h;
		if (!CalibrationEngine::InterpolateAt(refWindow, t.time - calTimeOffset,
			config.maxInterpolationGap, h))
			continue;
		if (h.angVel.norm() > config.maxAngularSpeed || h.vel.norm() > config.maxLinearSpeed)
			continue;

		// C_obs = H o E o T_s^-1 (see header). Independent of the currently
		// applied calibration: both poses are raw-universe.
		Eigen::Quaterniond qObs = (h.rot * extrinsic.rot * t.rot.conjugate()).normalized();
		Eigen::Vector3d tObs = (h.rot * extrinsic.pos + h.pos) - qObs * (calScale * t.pos);

		lastObsTime = t.time;   // fresh valid pair (pre-thinning) for coast detection

		Observation obs{ t.time, qObs, tObs };

		// Discontinuity guard: a step this large this fast is a universe jump.
		// JumpDetector owns jumps; a window straddling one must never be
		// averaged into a "correction". A real jump shifts every subsequent
		// observation, while a single-sample tracking glitch is one-off — so a
		// discontinuous observation is held as a candidate and only a second
		// one agreeing with it drops the window; otherwise the candidate is
		// discarded as an outlier.
		if (pendingDiscontinuity)
		{
			// Wait for an observation from an independent reference bracket: a
			// glitched ref sample corrupts every obs interpolated across it
			// with correlated errors, which would otherwise self-confirm.
			if (obs.time - pendingObs.time < config.jumpConfirmSpacing)
				continue;
			pendingDiscontinuity = false;
			double dRot = qObs.angularDistance(pendingObs.rot) * RadToDeg;
			double dPos = (tObs - pendingObs.trans).norm();
			if (dRot <= config.jumpGuardRotDeg && dPos <= config.jumpGuardPosM)
			{
				observations.clear();
				refWindow.clear();
				targetWindow.clear();
				targetProcessed = 0;
				return;
			}
			// Candidate was a glitch; fall through to handle this obs normally.
		}
		if (!observations.empty())
		{
			const Observation &prev = observations.back();
			if (obs.time - prev.time < config.jumpGuardWindow)
			{
				double dRot = qObs.angularDistance(prev.rot) * RadToDeg;
				double dPos = (tObs - prev.trans).norm();
				if (dRot > config.jumpGuardRotDeg || dPos > config.jumpGuardPosM)
				{
					pendingDiscontinuity = true;
					pendingObs = obs;
					continue;
				}
			}
		}

		if (obs.time - lastKeptObsTime < config.obsMinSpacing)
			continue;
		lastKeptObsTime = obs.time;
		observations.push_back(obs);
	}
}

void ContinuousAlignment::TrimWindows(double now)
{
	// Streams keep a little margin past the obs window for interpolation; the
	// latency correlator needs a longer window when opted in.
	double keepSeconds = config.windowSeconds + 2.0;
	if (config.latencyReestimation)
		keepSeconds = std::max(keepSeconds, config.latencyWindowSeconds);
	double streamCutoff = now - keepSeconds;
	size_t drop = 0;
	while (drop < refWindow.size() && refWindow[drop].time < streamCutoff)
		++drop;
	if (drop > 0)
		refWindow.erase(refWindow.begin(), refWindow.begin() + drop);

	drop = 0;
	while (drop < targetWindow.size() && targetWindow[drop].time < streamCutoff)
		++drop;
	if (drop > 0)
	{
		targetWindow.erase(targetWindow.begin(), targetWindow.begin() + drop);
		targetProcessed = targetProcessed > drop ? targetProcessed - drop : 0;
	}

	while (!observations.empty() && observations.front().time < now - config.windowSeconds)
		observations.pop_front();
}

bool ContinuousAlignment::EstimateWindow(Eigen::Quaterniond &rotOut, Eigen::Vector3d &transOut)
{
	std::vector<Eigen::Quaterniond> quats;
	std::vector<Eigen::Vector3d> vecs;
	quats.reserve(observations.size());
	vecs.reserve(observations.size());
	for (const auto &o : observations)
	{
		quats.push_back(o.rot);
		vecs.push_back(o.trans);
	}

	RobustStats stats;
	if (!RobustAverage(quats, vecs, 0.3, 0.01, stats))
		return false;

	scatterRotRmsDeg = stats.rotRmsDeg;
	scatterPosRmsM = stats.posRmsM;
	rotOut = stats.rot;
	transOut = stats.trans;
	return true;
}

void ContinuousAlignment::Decide(double now, const Eigen::Quaterniond &calRotation,
                                 const Eigen::Vector3d &calTranslationMeters)
{
	Eigen::Quaterniond rBar;
	Eigen::Vector3d tBar;
	if (!EstimateWindow(rBar, tBar))
	{
		deviation.valid = false;
		return;
	}

	// Scatter sanity: a noisy window means the pair itself is untrustworthy.
	// Hold rather than correct — but a rigid pair never scatters for long, so
	// a sustained exceed is itself a fault (a slipped mount's error rotates
	// with the head and lands here, not in a stable deviation) and freezes on
	// a longer confirm than a stable deviation would.
	if (scatterRotRmsDeg > config.maxScatterRotDeg || scatterPosRmsM > config.maxScatterPosM)
	{
		deviation.valid = false;
		if (state == State::Frozen)
		{
			resumeBelowSince = -1.0;
			return;
		}
		if (freezeExceededSince < 0.0)
		{
			freezeExceededSince = now;
		}
		else if (now - freezeExceededSince >= config.scatterFreezeConfirmSeconds)
		{
			state = State::Frozen;
			resumeBelowSince = -1.0;
			Event e;
			e.type = Event::FrozenLargeDeviation;
			e.deviation = deviation;
			events.push_back(e);
		}
		return;
	}

	// Deviation of the windowed estimate from the current calibration, as a
	// left delta D: newCal = D o oldCal.
	Eigen::Quaterniond rD = (rBar * calRotation.conjugate()).normalized();
	if (rD.w() < 0.0)
		rD.coeffs() = -rD.coeffs();
	double yawAngle = 2.0 * std::atan2(rD.y(), rD.w());
	Eigen::Quaterniond rYaw(Eigen::AngleAxisd(yawAngle, Eigen::Vector3d::UnitY()));
	double tiltDeg = rYaw.angularDistance(rD) * RadToDeg;
	Eigen::Vector3d tD = tBar - rD * calTranslationMeters;

	// Effective displacement at the user's head: the honest magnitude of the
	// deviation (a yaw delta far from the origin has a huge raw translation).
	Eigen::Vector3d headPos = refWindow.empty() ? Eigen::Vector3d::Zero() : refWindow.back().pos;
	double dEff = (rD * headPos + tD - headPos).norm();

	deviation.valid = true;
	deviation.yawDeg = std::abs(yawAngle) * RadToDeg;
	deviation.tiltDeg = tiltDeg;
	deviation.posM = dEff;

	bool exceed = deviation.yawDeg >= config.freezeYawDeg
		|| deviation.tiltDeg >= config.freezeTiltDeg
		|| deviation.posM >= config.freezePosM;

	if (state == State::Frozen)
	{
		// Resume hysteresis: only a deviation that fell well inside the freeze
		// band and stayed there (a cleared tracking fault, or a recalibration
		// that resets us anyway) unfreezes. A genuine mount slip never does.
		bool below = deviation.yawDeg < config.freezeYawDeg * config.resumeFactor
			&& deviation.tiltDeg < config.freezeTiltDeg * config.resumeFactor
			&& deviation.posM < config.freezePosM * config.resumeFactor;
		if (below)
		{
			if (resumeBelowSince < 0.0)
				resumeBelowSince = now;
			else if (now - resumeBelowSince >= config.resumeConfirmSeconds)
			{
				state = State::Tracking;
				resumeBelowSince = -1.0;
				freezeExceededSince = -1.0;
				Event e;
				e.type = Event::Resumed;
				events.push_back(e);
			}
		}
		else
		{
			resumeBelowSince = -1.0;
		}
		return;
	}

	if (exceed)
	{
		if (freezeExceededSince < 0.0)
		{
			freezeExceededSince = now;
		}
		else if (now - freezeExceededSince >= config.freezeConfirmSeconds)
		{
			state = State::Frozen;
			resumeBelowSince = -1.0;
			Event e;
			e.type = Event::FrozenLargeDeviation;
			e.deviation = deviation;
			events.push_back(e);
		}
		// Suspicious either way: never correct while confirming.
		return;
	}
	freezeExceededSince = -1.0;

	state = State::Tracking;

	if (deviation.yawDeg < config.deadbandYawDeg && deviation.posM < config.deadbandPosM)
		return;

	// Yaw + translation correction only (see header). The translation is
	// chosen so a full step lands the calibration's translation exactly on the
	// windowed estimate; a clamped fractional step is second-order accurate
	// and the next cycle corrects the remainder.
	Eigen::Vector3d tPrime = tBar - rYaw * calTranslationMeters;
	double f = 1.0;
	if (deviation.yawDeg > config.maxStepYawDeg)
		f = std::min(f, config.maxStepYawDeg / deviation.yawDeg);
	if (deviation.posM > config.maxStepPosM)
		f = std::min(f, config.maxStepPosM / deviation.posM);

	pendingCorrection.rotation = Eigen::Quaterniond(
		Eigen::AngleAxisd(f * yawAngle, Eigen::Vector3d::UnitY()));
	pendingCorrection.translation = f * tPrime;
	hasPendingCorrection = true;
}

void ContinuousAlignment::Update(double now, const Eigen::Quaterniond &calRotation,
                                 const Eigen::Vector3d &calTranslationMeters,
                                 double calScale, double calTimeOffset)
{
	if (!extrinsic.valid)
	{
		state = State::Inactive;
		return;
	}

	FormObservations(calScale, calTimeOffset);
	TrimWindows(now);

	bool obsFresh = lastObsTime > 0.0 && (now - lastObsTime) <= config.coastGapSeconds;
	if (!obsFresh)
	{
		// Occluded / powered off / face away from the base stations: hold the
		// calibration, resume cleanly. Frozen stays frozen through occlusion.
		if (state == State::Tracking)
		{
			state = State::Coasting;
			Event e;
			e.type = Event::TrackerLost;
			events.push_back(e);
		}
		deviation.valid = false;
		return;
	}
	if (state == State::Coasting)
	{
		// Back to gathering; Tracking resumes once a sane estimate lands.
		state = State::Inactive;
		Event e;
		e.type = Event::TrackerRecovered;
		events.push_back(e);
	}

	if (now - lastEvaluateTime < config.evaluateInterval)
		return;
	lastEvaluateTime = now;

	if (observations.size() < config.minObsForEstimate)
	{
		deviation.valid = false;
		return;
	}

	Decide(now, calRotation, calTranslationMeters);

	// Opt-in online latency measurement, only while actively Tracking (a
	// frozen or coasting pair proves nothing). Runs on the raw stream windows,
	// so it measures the true current latency independent of calTimeOffset;
	// the engine's correlation floor rejects motionless windows.
	if (config.latencyReestimation && state == State::Tracking &&
		now - lastLatencyEstimateTime >= config.latencyIntervalSeconds)
	{
		lastLatencyEstimateTime = now;
		EngineConfig engineCfg;
		double measured = 0.0;
		if (CalibrationEngine::EstimateTimeOffset(refWindow, targetWindow, engineCfg, measured))
		{
			pendingTimeOffset = measured;
			hasPendingTimeOffset = true;
		}
	}
}

bool ContinuousAlignment::PollTimeOffset(double &out)
{
	if (!hasPendingTimeOffset)
		return false;
	out = pendingTimeOffset;
	hasPendingTimeOffset = false;
	return true;
}

bool ContinuousAlignment::PollCorrection(Correction &out)
{
	if (!hasPendingCorrection)
		return false;
	out = pendingCorrection;
	hasPendingCorrection = false;
	return true;
}

bool ContinuousAlignment::PollEvent(Event &out)
{
	if (events.empty())
		return false;
	out = events.front();
	events.pop_front();
	return true;
}

void ContinuousAlignment::Reset()
{
	refWindow.clear();
	targetWindow.clear();
	observations.clear();
	targetProcessed = 0;
	lastObsTime = 0.0;
	lastKeptObsTime = 0.0;
	deviation = Deviation();
	scatterRotRmsDeg = 0.0;
	scatterPosRmsM = 0.0;
	freezeExceededSince = -1.0;
	resumeBelowSince = -1.0;
	pendingDiscontinuity = false;
	hasPendingCorrection = false;
	hasPendingTimeOffset = false;
	events.clear();
	state = State::Inactive;
}

bool ContinuousAlignment::DeriveMountExtrinsic(const std::vector<PoseSample> &refStream,
                                               const std::vector<PoseSample> &targetStream,
                                               const EngineResult &calibration,
                                               const Config &config,
                                               MountExtrinsic &out)
{
	out = MountExtrinsic();
	if (!calibration.valid || targetStream.empty())
		return false;

	// E = H^-1 o (C o T_s): the tracker's pose in the HMD body frame, one
	// estimate per time-aligned pair. Stride-subsampled: consecutive frames
	// are redundant for a constant.
	std::vector<Eigen::Quaterniond> quats;
	std::vector<Eigen::Vector3d> vecs;
	size_t stride = std::max<size_t>(1, targetStream.size() / 600);
	for (size_t i = 0; i < targetStream.size(); i += stride)
	{
		const PoseSample &t = targetStream[i];
		if (t.angVel.norm() > config.maxAngularSpeed || t.vel.norm() > config.maxLinearSpeed)
			continue;

		PoseSample h;
		if (!CalibrationEngine::InterpolateAt(refStream, t.time - calibration.timeOffset,
			config.maxInterpolationGap, h))
			continue;
		if (h.angVel.norm() > config.maxAngularSpeed || h.vel.norm() > config.maxLinearSpeed)
			continue;

		Eigen::Quaterniond qE = (h.rot.conjugate() * (calibration.rotation * t.rot)).normalized();
		Eigen::Vector3d pE = h.rot.conjugate()
			* (calibration.rotation * (calibration.scale * t.pos) + calibration.translation - h.pos);
		quats.push_back(qE);
		vecs.push_back(pE);
	}

	RobustStats stats;
	if (!RobustAverage(quats, vecs, 0.1, 0.005, stats))
		return false;

	out.rot = stats.rot;
	out.pos = stats.trans;
	out.rotRmsDeg = stats.rotRmsDeg;
	out.posRmsM = stats.posRmsM;
	out.pairs = stats.kept;

	// Rigidity gate: a hand-held or wobbly-mounted tracker produces per-pair
	// extrinsics that disagree; refuse to arm continuous mode from them.
	out.valid = stats.kept >= config.extrinsicMinPairs
		&& stats.rotRmsDeg <= config.extrinsicMaxRotRmsDeg
		&& stats.posRmsM <= config.extrinsicMaxPosRmsM;
	return out.valid;
}

} // namespace questcal
