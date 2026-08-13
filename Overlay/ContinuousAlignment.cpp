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

	// Means over the currently kept samples, plus EVERY sample's residual
	// against them (the trim gate below is a median over all residuals, not
	// only the survivors).
	auto estimate = [&]() -> bool
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
		return true;
	};

	if (!estimate())
		return false;

	double rotGate = std::max(3.0 * Median(rotRes), rotFloorDeg);
	double posGate = std::max(3.0 * Median(posRes), posFloorM);
	for (size_t i = 0; i < n; ++i)
		keep[i] = rotRes[i] <= rotGate && posRes[i] <= posGate;

	if (!estimate())
		return false;

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
		// A negative offset asks for a reference pose in the target sample's
		// future. During live streaming that bracket may simply not have arrived
		// yet: keep this target at the processing cursor so the next Update can
		// retry it. Since both streams are monotonic, every later target would
		// also need a future reference and can wait behind it. Conversely, a
		// requested time before the retained reference window is irrecoverably
		// old, and a failed interpolation inside the completed window is a hard
		// tracking gap that future samples cannot repair.
		double refTime = t.time - calTimeOffset;
		const size_t refLive = refWindow.size() - refHead;
		if (refLive == 0 || refTime > refWindow.back().time ||
			(refLive < 2 && refTime >= refWindow[refHead].time))
			break;
		if (refTime < refWindow[refHead].time)
			continue;

		PoseSample h;
		if (!CalibrationEngine::InterpolateAt(refWindow, refTime,
			config.maxInterpolationGap, h))
			continue;
		if (h.angVel.norm() > config.maxAngularSpeed || h.vel.norm() > config.maxLinearSpeed)
			continue;

		// C_obs = H o E o T_s^-1 (see header). Independent of the currently
		// applied calibration: both poses are raw-universe.
		Eigen::Quaterniond qObs = (h.rot * extrinsic.rot * t.rot.conjugate()).normalized();
		Eigen::Vector3d tObs = (h.rot * extrinsic.pos + h.pos) - qObs * (calScale * t.pos);

		Observation obs{ t.time, qObs, tObs, t.pos };

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
				refHead = 0;
				targetHead = 0;
				targetProcessed = 0;
				// Observation continuity is broken: every sustained-evidence
				// mark was accumulated against a stream that no longer exists.
				ClearConfirmMarks();
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

		// A pair is fresh only after it survives the discontinuity guard. Keep
		// this before normal thinning so a healthy high-rate stream does not coast,
		// but rejected jump candidates cannot leave stale state marked Tracking.
		lastObsTime = obs.time;

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

	// Retire expired samples by advancing a head cursor. Both windows hold
	// thousands of samples and retire a handful per tick, so erasing from the
	// front relocated every survivor up to 50 times a second for an
	// O(dropped) job; the prefix is compacted away only once it is a quarter
	// of the buffer, i.e. once the single move it costs is worth making.
	while (refHead < refWindow.size() && refWindow[refHead].time < streamCutoff)
		++refHead;
	while (targetHead < targetWindow.size() && targetWindow[targetHead].time < streamCutoff)
		++targetHead;
	// A target that expired before it was processed stays unprocessed —
	// exactly what the erase that used to rebase this cursor left behind.
	if (targetProcessed < targetHead)
		targetProcessed = targetHead;
	if (4 * refHead > refWindow.size() || 4 * targetHead > targetWindow.size())
		CompactWindows();

	while (!observations.empty() && observations.front().time < now - config.windowSeconds)
		observations.pop_front();
}

// Drop the retired prefixes for real, rebasing the processing cursor with
// them. Separate from TrimWindows because the latency correlator consumes the
// whole vectors and must see exactly the retained window.
void ContinuousAlignment::CompactWindows()
{
	if (refHead > 0)
	{
		refWindow.erase(refWindow.begin(), refWindow.begin() + refHead);
		refHead = 0;
	}
	if (targetHead > 0)
	{
		targetWindow.erase(targetWindow.begin(), targetWindow.begin() + targetHead);
		targetProcessed -= targetHead;   // >= targetHead by the clamp above
		targetHead = 0;
	}
}

bool ContinuousAlignment::EstimateWindow(const Eigen::Quaterniond &calRotation,
                                         const Eigen::Vector3d &calTranslationMeters,
                                         const ExpectedCalibrationAt &expectedAt,
                                         WindowEstimate &out) const
{
	std::vector<Eigen::Quaterniond> quats;
	std::vector<Eigen::Vector3d> vecs;
	quats.reserve(observations.size());
	vecs.reserve(observations.size());
	for (const auto &o : observations)
	{
		if (!expectedAt)
		{
			quats.push_back(o.rot);
			vecs.push_back(o.trans);
			continue;
		}

		Eigen::Quaterniond expectedRot;
		Eigen::Vector3d expectedTrans;
		expectedAt(o.targetRawPos, expectedRot, expectedTrans);

		// Remove the position-specific field transform, then compose the residual
		// over the canonical base calibration. A healthy field therefore reduces
		// to one constant base transform before robust averaging, while a genuine
		// universe delta remains unchanged at every position.
		Eigen::Quaterniond dRot = (o.rot * expectedRot.conjugate()).normalized();
		Eigen::Vector3d dTrans = o.trans - dRot * expectedTrans;
		quats.push_back((dRot * calRotation).normalized());
		vecs.push_back(dRot * calTranslationMeters + dTrans);
	}

	RobustStats stats;
	if (!RobustAverage(quats, vecs, 0.3, 0.01, stats))
		return false;   // nothing published; `out` is untouched

	out.rot = stats.rot;
	out.trans = stats.trans;
	out.scatterRotDeg = stats.rotRmsDeg;
	out.scatterPosM = stats.posRmsM;

	// Short-term noise: robust scatter-equivalent from obs pairs ~0.2 s apart.
	// A slipped mount's head-orientation-locked error barely moves over that
	// span while tracking noise — white, or the mid-frequency warble of
	// grazing lighthouse geometry — decorrelates, so the ratio of window
	// scatter to this estimate is what separates the two. ~0.2 s rather than
	// one spacing because warble is still partially correlated over the
	// shorter span and would read as structure. The partner is chosen by
	// ELAPSED TIME, not by a fixed index lag: obsMinSpacing is only a lower
	// bound (both speed gates, interpolation failures and the jump guard all
	// skip observations), so a fixed lag-2 can span up to ~0.5 s — over which
	// a slipped mount's error HAS moved, inflating the estimate and
	// misclassifying the slip as ordinary noise. At the nominal ~0.1 s
	// spacing the nearest partner is still exactly lag 2. Median-based so
	// glitch pairs don't inflate it (they would push a real slip toward the
	// harmless "noise" verdict).
	// The deltas are norms of 3D differences — Maxwell-distributed for
	// Gaussian noise, median 1.5382 * (sigma * sqrt(2)) per axis — while the
	// window scatter under pure noise is sigma * sqrt(3), so the median is
	// scaled by sqrt(3) / (1.5382 * sqrt(2)) to make noiseX directly
	// comparable to scatterX: unstructured windows sit at a ratio of ~1 and
	// structuredScatterFactor compares like with like. (The scalar-Gaussian
	// MAD constant used before overstated the estimate by ~32%, silently
	// raising the effective structured threshold to ~2.1x the designed 1.6x.)
	// RobustAverage refuses fewer than 3 samples, so at least one pair exists
	// here — the re-test that used to guard this block could never be false.
	const double noiseLag = 2.0 * config.obsMinSpacing;
	std::vector<double> dRot, dPos;
	dRot.reserve(quats.size());
	dPos.reserve(quats.size());
	size_t j = 0;
	for (size_t i = 2; i < quats.size(); ++i)
	{
		// Observation times ascend, so |span - noiseLag| is V-shaped in j and
		// the best partner never moves backward: walk j forward while it
		// improves.
		while (j + 1 < i &&
			std::abs(observations[i].time - observations[j + 1].time - noiseLag) <=
			std::abs(observations[i].time - observations[j].time - noiseLag))
			++j;
		dRot.push_back(quats[i].angularDistance(quats[j]) * RadToDeg);
		dPos.push_back((vecs[i] - vecs[j]).norm());
	}
	constexpr double k = 1.7320508075688772 / (1.5382 * 1.4142135623730951);
	out.noiseRotDeg = k * Median(dRot);
	out.noisePosM = k * Median(dPos);
	return true;
}

void ContinuousAlignment::Decide(double now, const Eigen::Quaterniond &calRotation,
                                 const Eigen::Vector3d &calTranslationMeters,
                                 const ExpectedCalibrationAt &expectedAt)
{
	WindowEstimate est;
	if (!EstimateWindow(calRotation, calTranslationMeters, expectedAt, est))
	{
		deviation.valid = false;
		return;
	}

	// Publish the estimate's figures as a unit, and only on success: the
	// accessors and the structured-scatter test below then always describe the
	// same window.
	scatterRotRmsDeg = est.scatterRotDeg;
	scatterPosRmsM = est.scatterPosM;
	noiseRotDeg = est.noiseRotDeg;
	noisePosM = est.noisePosM;
	const Eigen::Quaterniond &rBar = est.rot;
	const Eigen::Vector3d &tBar = est.trans;

	// Scatter sanity: a noisy window means the pair itself is untrustworthy —
	// never correct from it. Whether it is also a FAULT depends on structure:
	// window scatter far above the short-term noise estimate means the error
	// is locked to head orientation (slipped mount) and freezes after a
	// confirm; scatter explained by per-sample noise is degraded tracking
	// (grazing lighthouse angles while lying down) and only holds until it
	// settles. The vote counters make a flickering classification converge to
	// its majority instead of resetting the episode.
	bool rotExceed = scatterRotRmsDeg > config.maxScatterRotDeg;
	bool posExceed = scatterPosRmsM > config.maxScatterPosM;
	if (rotExceed || posExceed)
	{
		deviation.valid = false;
		freezeExceededSince = -1.0;   // no deviation can confirm through noise
		if (state == State::Frozen)
		{
			resumeBelowSince = -1.0;
			return;
		}

		bool structured =
			(rotExceed && scatterRotRmsDeg > config.structuredScatterFactor * noiseRotDeg) ||
			(posExceed && scatterPosRmsM > config.structuredScatterFactor * noisePosM);

		if (scatterSince < 0.0)
		{
			scatterSince = now;
			scatterStructuredVotes.clear();
		}
		if (scatterEpisodeSince < 0.0)
			scatterEpisodeSince = now;
		// Sliding vote window: a mount slip that starts deep into a long
		// degraded-tracking episode must only outvote the window, not the
		// episode's whole history, so the freeze delay stays bounded by
		// ~scatterVoteWindow evaluations.
		scatterStructuredVotes.push_back(structured ? 1 : 0);
		if (scatterStructuredVotes.size() > static_cast<size_t>(config.scatterVoteWindow))
			scatterStructuredVotes.pop_front();

		int structuredVotes = 0;
		for (char v : scatterStructuredVotes)
			structuredVotes += v;
		bool structuredMajority =
			2 * structuredVotes > static_cast<int>(scatterStructuredVotes.size());
		// Both scatter-path events carry the window scatter that classified
		// them; the deviation stays invalid because no deviation was measured
		// through this noise.
		auto scatterEvent = [&](Event::Type type)
		{
			Event e;
			e.type = type;
			e.scatterRotDeg = scatterRotRmsDeg;
			e.scatterPosM = scatterPosRmsM;
			return e;
		};

		if (now - scatterSince >= config.scatterFreezeConfirmSeconds && structuredMajority)
		{
			EnterState(State::Frozen);
			events.push_back(scatterEvent(Event::FrozenMountScatter));
			return;
		}

		EnterState(State::Holding);
		if (!unstableNotified && !structuredMajority &&
			now - scatterEpisodeSince >= config.scatterNotifySeconds)
		{
			unstableNotified = true;
			events.push_back(scatterEvent(Event::ObservationsUnstable));
		}
		return;
	}
	scatterSince = -1.0;
	scatterEpisodeSince = -1.0;
	unstableNotified = false;   // next sustained episode logs again
	if (state == State::Holding)
		EnterState(State::Tracking);   // settled; resumes silently below

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
	Eigen::Vector3d headPos =
		refHead < refWindow.size() ? refWindow.back().pos : Eigen::Vector3d::Zero();
	double dEff = (rD * headPos + tD - headPos).norm();

	// The correction below is yaw-only, so the share of that displacement the
	// tilt contributes cannot be reduced by it. Measure the reachable part
	// separately: gating and clamping on the full delta gives any real mount
	// tilt a floor no correction can get under, and the loop then emits a
	// near-zero correction every cycle forever - each one re-stamping the
	// drift counters and pinning alignment health at Fresh. deviation.posM
	// keeps the full delta, because tilt is the mount-fault signal the freeze
	// and resume tests exist to catch.
	Eigen::Vector3d tPrime = tBar - rYaw * calTranslationMeters;
	double dEffYaw = (rYaw * headPos + tPrime - headPos).norm();

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
				EnterState(State::Tracking);
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
			EnterState(State::Frozen);
			Event e;
			e.type = Event::FrozenLargeDeviation;
			e.deviation = deviation;
			events.push_back(e);
		}
		// Suspicious either way: never correct while confirming.
		return;
	}

	EnterState(State::Tracking);

	if (deviation.yawDeg < config.deadbandYawDeg && dEffYaw < config.deadbandPosM)
		return;

	// Yaw + translation correction only (see header). The translation is
	// chosen so a full step lands the calibration's translation exactly on the
	// windowed estimate; a clamped fractional step is second-order accurate
	// and the next cycle corrects the remainder.
	double f = 1.0;
	if (deviation.yawDeg > config.maxStepYawDeg)
		f = std::min(f, config.maxStepYawDeg / deviation.yawDeg);
	if (dEffYaw > config.maxStepPosM)
		f = std::min(f, config.maxStepPosM / dEffYaw);

	pendingCorrection.rotation = Eigen::Quaterniond(
		Eigen::AngleAxisd(f * yawAngle, Eigen::Vector3d::UnitY()));
	pendingCorrection.translation = f * tPrime;
	hasPendingCorrection = true;
}

// One owner for the freeze hysteresis' paired sentinels: entering either end
// abandons the confirm that was running toward the other, so no transition
// site has to remember which mark its own transition invalidates.
void ContinuousAlignment::EnterState(State s)
{
	if (s == State::Frozen || s == State::Tracking)
	{
		freezeExceededSince = -1.0;
		resumeBelowSince = -1.0;
	}
	state = s;
}

// Observation continuity broke (occlusion, a dropped window). Every sustained-
// evidence mark was accumulated against a stream that no longer exists, so the
// next evaluation must re-earn its verdict from scratch instead of completing
// a confirm on one post-recovery sample. Clearing the marks and NOT the State
// is what keeps a Frozen mount frozen through an occlusion while still forcing
// a full resumeConfirmSeconds of fresh evidence before it unfreezes.
// unstableNotified deliberately survives: it is a notification latch, not
// evidence, and re-arming it would spam one toast per occlusion.
void ContinuousAlignment::ClearConfirmMarks()
{
	freezeExceededSince = -1.0;
	resumeBelowSince = -1.0;
	scatterSince = -1.0;
	scatterStructuredVotes.clear();
}

void ContinuousAlignment::Update(double now, const Eigen::Quaterniond &calRotation,
                                 const Eigen::Vector3d &calTranslationMeters,
                                 double calScale, double calTimeOffset,
                                 const ExpectedCalibrationAt &expectedAt)
{
	if (!extrinsic.valid)
	{
		EnterState(State::Inactive);
		return;
	}

	FormObservations(calScale, calTimeOffset);
	TrimWindows(now);

	bool obsFresh = lastObsTime > 0.0 && (now - lastObsTime) <= config.coastGapSeconds;
	if (!obsFresh)
	{
		// Occluded / powered off / face away from the base stations: hold the
		// calibration, resume cleanly. Frozen stays frozen through occlusion —
		// but the confirms do not, including while Frozen (which changes no
		// state here and so used to keep its marks).
		ClearConfirmMarks();
		if (state == State::Tracking || state == State::Holding)
		{
			EnterState(State::Coasting);
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
		EnterState(State::Inactive);
		Event e;
		e.type = Event::TrackerRecovered;
		events.push_back(e);
	}

	if (now - lastEvaluateTime < config.evaluateInterval)
		return;
	lastEvaluateTime = now;

	if (observations.size() < config.minObsForEstimate)
	{
		if (state == State::Tracking || state == State::Holding)
			EnterState(State::Inactive);
		deviation.valid = false;
		return;
	}

	Decide(now, calRotation, calTranslationMeters, expectedAt);

	// Opt-in online latency measurement, only while actively Tracking (a
	// frozen or coasting pair proves nothing). Runs on the raw stream windows,
	// so it measures the true current latency independent of calTimeOffset;
	// the engine's correlation floor rejects motionless windows.
	if (config.latencyReestimation && state == State::Tracking &&
		now - lastLatencyEstimateTime >= config.latencyIntervalSeconds)
	{
		lastLatencyEstimateTime = now;
		// The correlator consumes the whole vectors, so hand it exactly the
		// retained window rather than one still carrying an expired prefix.
		CompactWindows();
		EngineConfig engineCfg;
		double measured = 0.0;
		// No input validation: the two push methods are the only way into
		// these windows and already enforce the monotonicity IsValidStream
		// re-scans, while the caller gates usability at ingestion. Re-scanning
		// ~20 s of both streams here costs a frame on the render thread.
		if (CalibrationEngine::EstimateTimeOffset(refWindow, targetWindow, engineCfg,
			measured, false))
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
	refHead = 0;
	targetHead = 0;
	observations.clear();
	targetProcessed = 0;
	lastObsTime = 0.0;
	lastKeptObsTime = 0.0;
	deviation = Deviation();
	scatterRotRmsDeg = 0.0;
	scatterPosRmsM = 0.0;
	noiseRotDeg = 0.0;
	noisePosM = 0.0;
	ClearConfirmMarks();
	scatterEpisodeSince = -1.0;
	unstableNotified = false;
	pendingDiscontinuity = false;
	hasPendingCorrection = false;
	hasPendingTimeOffset = false;
	events.clear();
	EnterState(State::Inactive);
}

bool ContinuousAlignment::DeriveMountExtrinsic(const std::vector<PoseSample> &refStream,
                                               const std::vector<PoseSample> &targetStream,
                                               const EngineResult &calibration,
                                               MountExtrinsic &out)
{
	if (!calibration.valid || targetStream.empty())
		return false;

	// Fixed policy, not caller knobs: the speed/interpolation gates that decide
	// which pairs are usable, and the rigidity gate that decides whether
	// continuous calibration arms at all.
	const Config config;

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

	MountExtrinsic derived;
	derived.rot = stats.rot;
	derived.pos = stats.trans;
	derived.rotRmsDeg = stats.rotRmsDeg;
	derived.posRmsM = stats.posRmsM;
	derived.pairs = stats.kept;

	// Rigidity gate: a hand-held or wobbly-mounted tracker produces per-pair
	// extrinsics that disagree; refuse to arm continuous mode from them.
	derived.valid = stats.kept >= config.extrinsicMinPairs
		&& stats.rotRmsDeg <= config.extrinsicMaxRotRmsDeg
		&& stats.posRmsM <= config.extrinsicMaxPosRmsM;
	if (!derived.valid)
		return false;   // `out` untouched: the caller keeps any previous extrinsic

	out = derived;
	return true;
}

} // namespace questcal
