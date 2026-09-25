#include "ContinuousAlignment.h"

#include <algorithm>
#include <cmath>

namespace questcal
{

namespace
{

constexpr double RadToDeg = 180.0 / EIGEN_PI;

bool HasPoseStep(const std::vector<PoseSample> &samples, double begin, double end,
                const ContinuousAlignment::Config &config)
{
	auto next = std::upper_bound(samples.begin(), samples.end(), begin,
		[](double time, const PoseSample &s) { return time < s.time; });
	auto stop = std::lower_bound(samples.begin(), samples.end(), end,
		[](const PoseSample &s, double time) { return s.time < time; });
	// The reference pose may interpolate against the first sample after end.
	if (stop != samples.end())
		++stop;
	if (next == samples.begin() && next != samples.end())
		++next;
	for (; next < stop; ++next)
	{
		const auto &previous = *(next - 1);
		double dt = next->time - previous.time;
		if (dt > config.maxInterpolationGap)
			continue;
		Eigen::Vector3d positionError = next->pos - previous.pos -
			0.5 * (previous.vel + next->vel) * dt;
		Eigen::Vector3d angularStep = 0.5 * (previous.angVel + next->angVel) * dt;
		Eigen::Quaterniond predicted = previous.rot;
		if (angularStep.norm() > 1e-12)
			predicted = Eigen::Quaterniond(Eigen::AngleAxisd(angularStep.norm(), angularStep.normalized())) * predicted;
		if (positionError.norm() > config.jumpGuardPosM ||
			next->rot.angularDistance(predicted) * RadToDeg > config.jumpGuardRotDeg)
			return true;
	}
	return false;
}

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

	// keep[] is what estimate() just accepted, so kept >= 3.
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
	{
		++diagnostics.referenceOutOfOrder;
		if (s.time == refWindow.back().time)
			++diagnostics.referenceSameTime;
		return;
	}
	if (!refWindow.empty() && s.time - refWindow.back().time >= config.maxStreamGapSeconds)
		Reset(ResetReason::StreamGap);
	refWindow.push_back(s);
}

void ContinuousAlignment::PushTarget(const PoseSample &s)
{
	if (!targetWindow.empty() && s.time <= targetWindow.back().time)
	{
		++diagnostics.targetOutOfOrder;
		if (s.time == targetWindow.back().time)
			++diagnostics.targetSameTime;
		return;
	}
	if (!targetWindow.empty() && s.time - targetWindow.back().time >= config.maxStreamGapSeconds)
		Reset(ResetReason::StreamGap);
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
		{
			++diagnostics.targetSpeedRejected;
			continue;
		}

		// Same time-alignment convention as the engine: the reference stream
		// is interpolated at the target's timestamp minus the solved offset.
		// A bracket that has not arrived yet (a negative offset asks for the
		// reference's future) keeps this target, and every later one, at the
		// cursor for the next Update. A time before the retained reference
		// window is irrecoverably old, and a failed interpolation inside the
		// completed window is a hard tracking gap.
		double refTime = t.time - calTimeOffset;
		const size_t refLive = refWindow.size() - refHead;
		if (refLive == 0 || refTime > refWindow.back().time ||
			(refLive < 2 && refTime >= refWindow[refHead].time))
		{
			++diagnostics.referenceWaitUpdates;
			break;
		}
		if (refTime < refWindow[refHead].time)
		{
			++diagnostics.referenceTooOld;
			continue;
		}

		PoseSample h;
		if (!CalibrationEngine::InterpolateAt(refWindow, refTime,
			config.maxInterpolationGap, h))
		{
			++diagnostics.interpolationRejected;
			continue;
		}
		if (h.angVel.norm() > config.maxAngularSpeed || h.vel.norm() > config.maxLinearSpeed)
		{
			++diagnostics.referenceSpeedRejected;
			continue;
		}

		// C_obs = H o E o T_s^-1 (see header). Independent of the currently
		// applied calibration: both poses are raw-universe.
		Eigen::Quaterniond qObs = (h.rot * extrinsic.rot * t.rot.conjugate()).normalized();
		Eigen::Vector3d tObs = (h.rot * extrinsic.pos + h.pos) - qObs * (calScale * t.pos);

		Observation obs{ t.time, qObs, tObs, t.pos };

		// Discontinuity guard: a fast step may indicate a universe jump.
		// JumpDetector owns jumps; a window straddling one must never be
		// averaged into a "correction". A real jump shifts every subsequent
		// observation, while a single-sample tracking glitch is one-off — so a
		// discontinuous observation is held as a candidate and only a second
		// one agreeing with it drops the window; otherwise the candidate is
		// discarded as an outlier.
		if (pendingObs)
		{
			// Wait for an observation from an independent reference bracket: a
			// glitched ref sample corrupts every obs interpolated across it
			// with correlated errors, which would otherwise self-confirm.
			if (obs.time - pendingObs->time < config.jumpConfirmSpacing)
			{
				++diagnostics.jumpGuardRejected;
				continue;
			}
			double dRot = qObs.angularDistance(pendingObs->rot) * RadToDeg;
			double dPos = (tObs - pendingObs->trans).norm();
			// Consumed either way: confirmed below, or discarded as a glitch.
			pendingObs.reset();
			if (dRot <= config.jumpGuardRotDeg && dPos <= config.jumpGuardPosM)
			{
				++diagnostics.jumpGuardResets;
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
				// Sparse, speed-gated observations can turn continuous tracking
				// noise into an apparent step. Require a discontinuity in the raw
				// adjacent poses before discarding the estimation window.
				if ((dRot > config.jumpGuardRotDeg || dPos > config.jumpGuardPosM) &&
					(HasPoseStep(refWindow, prev.time - calTimeOffset, obs.time - calTimeOffset, config) ||
					 HasPoseStep(targetWindow, prev.time, obs.time, config)))
				{
					++diagnostics.jumpGuardRejected;
					pendingObs = obs;
					correctionEligible = false;
					pendingCorrection.reset();
					pendingTimeOffset.reset();
					continue;
				}
			}
		}

		// A pair is fresh only after it survives the discontinuity guard. Keep
		// this before normal thinning so a healthy high-rate stream does not coast,
		// but rejected jump candidates cannot leave stale state marked Tracking.
		lastObsTime = obs.time;
		++diagnostics.observationsFormed;

		if (obs.time - lastKeptObsTime < config.obsMinSpacing)
			continue;
		lastKeptObsTime = obs.time;
		observations.push_back(obs);
		++diagnostics.observationsKept;
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

	// Retire expired samples by advancing the head cursors (see refHead);
	// compact once the dead prefix is a quarter of the buffer.
	while (refHead < refWindow.size() && refWindow[refHead].time < streamCutoff)
		++refHead;
	while (targetHead < targetWindow.size() && targetWindow[targetHead].time < streamCutoff)
		++targetHead;
	// A target that expired before it was processed is never processed.
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
		targetProcessed -= targetHead;   // >= targetHead by TrimWindows' clamp
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
	return true;
}

void ContinuousAlignment::Decide(double now, const Eigen::Quaterniond &calRotation,
                                 const Eigen::Vector3d &calTranslationMeters,
                                 const ExpectedCalibrationAt &expectedAt)
{
	correctionEligible = false;
	pendingCorrection.reset();
	WindowEstimate est;
	if (!EstimateWindow(calRotation, calTranslationMeters, expectedAt, est))
	{
		deviation.valid = false;
		return;
	}

	// Published only on success, so the accessors describe the window tested
	// below.
	scatterRotRmsDeg = est.scatterRotDeg;
	scatterPosRmsM = est.scatterPosM;

	bool rotExceed = scatterRotRmsDeg > config.maxScatterRotDeg;
	bool posExceed = scatterPosRmsM > config.maxScatterPosM;

	double yawAngle = 0.0;
	Eigen::Vector3d headStep, headPos;

	// The target's own tracking says its pose is unsettled: a single-baseline
	// lighthouse fit swims along the remaining line of sight, and a restarted
	// solution can land centimeters away before it converges (live 2026-09-25:
	// a headset tracker restarted from one base station read 4.4 deg / 33 cm
	// off for ten minutes and returned on its next restart). Nothing measured
	// through that is evidence about the universes, so every
	// verdict waits: the deviation is still published for the log and pane.
	if (targetSettling)
	{
		freezeExceededSince = -1.0;
		resumeBelowSince = -1.0;
		resumeInBandSince = -1.0;
		reanchorSince = -1.0;
		undoSince = -1.0;
		if (rotExceed || posExceed)
			deviation.valid = false;
		else
			deviation = MeasureDeviation(est, calRotation, calTranslationMeters,
				yawAngle, headStep, headPos);
		if (state != State::Frozen)
			EnterState(State::Holding);
		return;
	}

	// Scatter sanity: a noisy window means the pair is untrustworthy right
	// now (grazing lighthouse angles while lying down, partial occlusion):
	// never correct from it and never freeze on it, only hold until it
	// settles. A long episode is reported once.
	if (rotExceed || posExceed)
	{
		deviation.valid = false;
		freezeExceededSince = -1.0;   // no deviation can confirm through noise
		reanchorSince = -1.0;         // nor can a re-anchor, or its undoing
		undoSince = -1.0;
		if (state == State::Frozen)
		{
			resumeBelowSince = -1.0;
			resumeInBandSince = -1.0;
			return;
		}

		if (scatterEpisodeSince < 0.0)
			scatterEpisodeSince = now;
		EnterState(State::Holding);
		if (!unstableNotified && now - scatterEpisodeSince >= config.scatterNotifySeconds)
		{
			unstableNotified = true;
			// The event carries the scatter it was raised on; the deviation
			// stays invalid because none was measured through this noise.
			Event e;
			e.type = Event::ObservationsUnstable;
			e.scatterRotDeg = scatterRotRmsDeg;
			e.scatterPosM = scatterPosRmsM;
			events.push_back(e);
		}
		return;
	}
	scatterEpisodeSince = -1.0;
	unstableNotified = false;   // next sustained episode logs again
	if (state == State::Holding)
		EnterState(State::Tracking);   // settled; resumes silently below

	deviation = MeasureDeviation(est, calRotation, calTranslationMeters,
		yawAngle, headStep, headPos);

	bool exceed = deviation.yawDeg >= config.freezeYawDeg
		|| deviation.posM >= config.freezePosM;
	bool tilted = deviation.tiltDeg >= config.holdTiltDeg;

	if (state == State::Frozen)
	{
		// Resume hysteresis: a deviation that fell well inside the freeze band
		// unfreezes after a short confirm (a cleared tracking fault, or a
		// recalibration that resets us anyway); one anywhere inside the band
		// after a long one, since Tracking would correct it without freezing.
		// A persisting fault stays outside the band and never does. A tilted
		// window holds, so it confirms neither.
		bool below = deviation.yawDeg < config.freezeYawDeg * config.resumeFactor
			&& deviation.posM < config.freezePosM * config.resumeFactor
			&& !tilted;
		if (below)
		{
			if (resumeBelowSince < 0.0)
				resumeBelowSince = now;
		}
		else
		{
			resumeBelowSince = -1.0;
		}
		if (!exceed && !tilted)
		{
			if (resumeInBandSince < 0.0)
				resumeInBandSince = now;
		}
		else
		{
			resumeInBandSince = -1.0;
		}
		if ((resumeBelowSince >= 0.0 && now - resumeBelowSince >= config.resumeConfirmSeconds) ||
			(resumeInBandSince >= 0.0 && now - resumeInBandSince >= config.resumeInBandSeconds))
		{
			EnterState(State::Tracking);
			episodeSince = -1.0;
			reanchorSince = -1.0;
			undoSince = -1.0;
			Event e;
			e.type = Event::Resumed;
			events.push_back(e);
			return;
		}
		// Still off by the freeze thresholds or tilted: the deviation that
		// stays put is followed. Anything inside the band waits for the resume.
		if (exceed || tilted)
			TryReanchor(now, est, calRotation, calTranslationMeters, yawAngle, headStep, headPos);
		else
			reanchorSince = undoSince = -1.0;
		return;
	}

	if (exceed)
	{
		// A tilt hold that grew into a deviation is a new episode, which the
		// freeze starts.
		episodeSince = -1.0;
		reanchorSince = -1.0;
		undoSince = -1.0;
		if (freezeExceededSince < 0.0)
		{
			freezeExceededSince = now;
		}
		else if (now - freezeExceededSince >= config.freezeConfirmSeconds)
		{
			EnterState(State::Frozen);
			episodeSince = now;
			Event e;
			e.type = Event::FrozenLargeDeviation;
			e.deviation = deviation;
			e.afterTargetResolve = now - lastTargetResolveTime <= config.resolveAttributionSeconds;
			events.push_back(e);
		}
		// Suspicious either way: never correct while confirming.
		return;
	}

	// Tilted: hold like scatter, with the deviation still published. Only
	// sustained yaw or position at the head freezes; a tilt that stays put
	// re-anchors like a freeze does.
	if (tilted)
	{
		freezeExceededSince = -1.0;
		if (episodeSince < 0.0)
			episodeSince = now;
		EnterState(State::Holding);
		TryReanchor(now, est, calRotation, calTranslationMeters, yawAngle, headStep, headPos);
		return;
	}

	episodeSince = -1.0;
	reanchorSince = -1.0;
	undoSince = -1.0;
	EnterState(State::Tracking);

	if (deviation.yawDeg < config.deadbandYawDeg && deviation.posM < config.deadbandPosM)
		return;

	// Clamp displacement at the head together with yaw. Scaling translation
	// coefficients alone breaks the pivot cancellation of a fractional yaw.
	double f = 1.0;
	if (deviation.yawDeg > config.maxStepYawDeg)
		f = std::min(f, config.maxStepYawDeg / deviation.yawDeg);
	if (deviation.posM > config.maxStepPosM)
		f = std::min(f, config.maxStepPosM / deviation.posM);

	// Yaw about the head, then the head moved by the full displacement: the
	// tilt stays out of the calibration and nothing of it is left at the head.
	Correction correction;
	correction.rotation = Eigen::Quaterniond(
		Eigen::AngleAxisd(f * yawAngle, Eigen::Vector3d::UnitY()));
	correction.translation = headPos + f * headStep - correction.rotation * headPos;
	pendingCorrection = correction;
	correctionEligible = true;
}

void ContinuousAlignment::TryReanchor(double now, const WindowEstimate &est,
                                      const Eigen::Quaterniond &calRotation,
                                      const Eigen::Vector3d &calTranslationMeters,
                                      double yawAngle, const Eigen::Vector3d &headStep,
                                      const Eigen::Vector3d &headPos)
{
	// The readings are back on the calibration the last re-anchor replaced:
	// what it followed has cleared. Nothing new is followed meanwhile, and a
	// restart does not hold this back, since a restart is what usually
	// clears a fault of the target.
	if (replaced)
	{
		double undoYaw = 0.0;
		Eigen::Vector3d undoStep, undoHead;
		const Deviation back = MeasureDeviation(est, replaced->rot, replaced->trans,
			undoYaw, undoStep, undoHead);
		if (back.yawDeg < config.freezeYawDeg * config.resumeFactor &&
			back.posM < config.freezePosM * config.resumeFactor &&
			back.tiltDeg < config.holdTiltDeg)
		{
			reanchorSince = -1.0;
			if (undoSince < 0.0)
				undoSince = now;
			if (now - undoSince < config.resumeConfirmSeconds || pendingObs)
				return;
			Correction undo;
			undo.rotation = (replaced->rot * calRotation.conjugate()).normalized();
			undo.translation = replaced->trans - undo.rotation * calTranslationMeters;
			pendingReanchor = undo;
			Event e;
			e.type = Event::ReanchorUndone;
			e.deviation = deviation;
			events.push_back(e);
			replaced.reset();
			episodeSince = -1.0;
			undoSince = -1.0;
			EnterState(State::Tracking);
			return;
		}
		undoSince = -1.0;
	}

	// A restart of the target shortly before the episode, or during it, says
	// the target moved rather than the universes: freeze on it (live
	// 2026-09-25, every remaining freeze came within two minutes of one).
	// Follow mode takes the target's word regardless, as
	// OpenVR-SpaceCalibrator does.
	const bool attributed = episodeSince >= 0.0 &&
		lastTargetResolveTime >= episodeSince - config.resolveAttributionSeconds;
	if (attributed && !followMode)
	{
		reanchorSince = -1.0;
		return;
	}

	// The estimate has to hold still, not merely stay off: compared with the
	// one the run began on, at the same head position, so the head moving
	// through a large rotation does not read as the estimate moving.
	if (reanchorSince >= 0.0)
	{
		double driftYaw = 0.0;
		Eigen::Vector3d driftStep, driftHead;
		const Deviation drift = MeasureDeviation(est, reanchorRef.rot, reanchorRef.trans,
			driftYaw, driftStep, driftHead);
		if (drift.yawDeg >= config.reanchorSteadyDeg || drift.tiltDeg >= config.reanchorSteadyDeg ||
			drift.posM >= config.reanchorSteadyPosM)
			reanchorSince = -1.0;
	}
	if (reanchorSince < 0.0)
	{
		reanchorSince = now;
		reanchorRef = est;
		return;
	}
	if (now - reanchorSince < (followMode ? config.followConfirmSeconds : config.reanchorConfirmSeconds))
		return;
	// A window in front of an unresolved jump-guard step authorizes nothing,
	// this included; the confirm carries on and completes on the next one.
	if (pendingObs)
		return;

	// The estimate becomes the calibration. A tilt this large is the lighthouse
	// frame's own and goes in whole; below it, what tilt there is reads as the
	// tracker's orientation bias, and the re-anchor turns about the head
	// exactly as a correction would, without the step cap.
	Correction reanchor;
	if (deviation.tiltDeg >= config.holdTiltDeg)
	{
		Eigen::Quaterniond rD = (est.rot * calRotation.conjugate()).normalized();
		reanchor.rotation = rD;
		reanchor.translation = est.trans - rD * calTranslationMeters;
	}
	else
	{
		reanchor.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(yawAngle, Eigen::Vector3d::UnitY()));
		reanchor.translation = headPos + headStep - reanchor.rotation * headPos;
	}
	pendingReanchor = reanchor;
	replaced = Replaced{ calRotation, calTranslationMeters };
	Event e;
	e.type = Event::Reanchored;
	e.deviation = deviation;
	e.afterTargetResolve = attributed;
	events.push_back(e);
	episodeSince = -1.0;
	reanchorSince = -1.0;
	EnterState(State::Tracking);
}

ContinuousAlignment::Deviation ContinuousAlignment::MeasureDeviation(
	const WindowEstimate &est, const Eigen::Quaterniond &calRotation,
	const Eigen::Vector3d &calTranslationMeters,
	double &yawAngleOut, Eigen::Vector3d &headStepOut, Eigen::Vector3d &headPosOut) const
{
	// Deviation of the windowed estimate from the current calibration, as a
	// left delta D: newCal = D o oldCal.
	Eigen::Quaterniond rD = (est.rot * calRotation.conjugate()).normalized();
	if (rD.w() < 0.0)
		rD.coeffs() = -rD.coeffs();
	yawAngleOut = 2.0 * std::atan2(rD.y(), rD.w());
	Eigen::Quaterniond rYaw(Eigen::AngleAxisd(yawAngleOut, Eigen::Vector3d::UnitY()));
	Eigen::Vector3d tD = est.trans - rD * calTranslationMeters;

	// Effective displacement at the user's head: the honest magnitude of the
	// deviation (a yaw delta far from the origin has a huge raw translation).
	// Every observation maps the tracker exactly onto the head, so the
	// estimate is right there whatever its tilt. Dropping the tilt about any
	// other pivot leaves tilt times the lever arm at the head (live 2026-09-25:
	// 1.1 deg of tracker tilt about an origin 3.2 m away walked the head 3.7 cm
	// off). Decide runs only on a fresh observation (< coastGapSeconds old,
	// |time offset| <= 1 s), so the newest reference sample is still retained.
	headPosOut = refWindow.back().pos;
	headStepOut = rD * headPosOut + tD - headPosOut;

	Deviation d;
	d.valid = true;
	d.yawDeg = std::abs(yawAngleOut) * RadToDeg;
	d.tiltDeg = rYaw.angularDistance(rD) * RadToDeg;
	d.posM = headStepOut.norm();
	return d;
}

// One owner for the freeze hysteresis' paired sentinels: entering either end
// abandons the confirm that was running toward the other, so no transition
// site has to remember which mark its own transition invalidates.
void ContinuousAlignment::EnterState(State s)
{
	if (s != State::Tracking)
	{
		correctionEligible = false;
		pendingCorrection.reset();
	}
	if (s == State::Frozen || s == State::Tracking)
	{
		freezeExceededSince = -1.0;
		resumeBelowSince = -1.0;
		resumeInBandSince = -1.0;
	}
	// Without an estimate there is no stuck episode to follow; a freeze keeps
	// its own through a gap by not passing through here.
	if (s == State::Inactive || s == State::Coasting)
	{
		episodeSince = -1.0;
		reanchorSince = -1.0;
		undoSince = -1.0;
	}
	state = s;
}

// Observation continuity broke (occlusion, a dropped window): every sustained-
// evidence mark was accumulated against a stream that no longer exists, so the
// next verdict is re-earned from scratch. The State stays, so a freeze holds
// through an occlusion yet needs a full resumeConfirmSeconds of fresh evidence
// to lift. unstableNotified survives: it is a notification latch, not evidence,
// and re-arming it would raise one toast per occlusion.
void ContinuousAlignment::ClearConfirmMarks()
{
	correctionEligible = false;
	pendingCorrection.reset();
	pendingTimeOffset.reset();
	freezeExceededSince = -1.0;
	resumeBelowSince = -1.0;
	resumeInBandSince = -1.0;
	reanchorSince = -1.0;
	undoSince = -1.0;
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
	// A pending jump candidate already cleared these when it was raised.
	if (observations.size() < config.minObsForEstimate)
	{
		correctionEligible = false;
		pendingCorrection.reset();
		pendingTimeOffset.reset();
	}

	bool obsFresh = lastObsTime > 0.0 && (now - lastObsTime) <= config.coastGapSeconds;
	if (!obsFresh)
	{
		// Occluded / powered off / face away from the base stations: hold the
		// calibration, resume cleanly. Frozen stays frozen through occlusion,
		// but its confirms do not.
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
	if (pendingObs)
	{
		// Continue evaluating tracking faults, but a window preceding
		// an unresolved step cannot authorize a correction.
		correctionEligible = false;
		pendingCorrection.reset();
	}

	// Opt-in online latency measurement, only while actively Tracking (a
	// frozen or coasting pair proves nothing). Runs on the raw stream windows,
	// so it measures the true current latency independent of calTimeOffset;
	// the engine's correlation floor rejects motionless windows.
	if (config.latencyReestimation && state == State::Tracking && !pendingObs &&
		now - lastLatencyEstimateTime >= config.latencyIntervalSeconds)
	{
		lastLatencyEstimateTime = now;
		// The correlator consumes the whole vectors, so hand it exactly the
		// retained window rather than one still carrying an expired prefix.
		CompactWindows();
		EngineConfig engineCfg;
		double measured = 0.0;
		if (CalibrationEngine::EstimateTimeOffset(refWindow, targetWindow, engineCfg,
			measured))
			pendingTimeOffset = measured;
	}
}

bool ContinuousAlignment::PollTimeOffset(double &out)
{
	if (!pendingTimeOffset)
		return false;
	out = *pendingTimeOffset;
	pendingTimeOffset.reset();
	return true;
}

bool ContinuousAlignment::PollCorrection(Correction &out)
{
	if (!pendingCorrection)
		return false;
	out = *pendingCorrection;
	pendingCorrection.reset();
	return true;
}

bool ContinuousAlignment::PollReanchor(Correction &out)
{
	if (!pendingReanchor)
		return false;
	out = *pendingReanchor;
	pendingReanchor.reset();
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

ContinuousAlignment::Diagnostics ContinuousAlignment::GetDiagnostics() const
{
	Diagnostics result = diagnostics;
	result.referenceSamples = refWindow.size() - refHead;
	result.targetSamples = targetWindow.size() - targetHead;
	result.pendingTargets = targetWindow.size() - targetProcessed;
	result.observations = observations.size();
	result.requiredObservations = config.minObsForEstimate;
	result.lastObservationTime = lastObsTime;
	return result;
}

void ContinuousAlignment::Reset(ResetReason reason)
{
	// A break in the target's own tracking is a gap too: the fault a freeze
	// recorded is not cleared by it, only by fresh evidence afterwards.
	const bool gap = reason == ResetReason::StreamGap || reason == ResetReason::TargetResolved;
	const bool keepFrozen = gap && state == State::Frozen;
	const bool keepCoasting = gap && state == State::Coasting;
	++diagnostics.resets[static_cast<size_t>(reason)];
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
	ClearConfirmMarks();
	scatterEpisodeSince = -1.0;
	unstableNotified = false;
	pendingObs.reset();
	pendingReanchor.reset();
	if (!gap)
		replaced.reset();
	events.clear();
	EnterState(keepFrozen ? State::Frozen : keepCoasting ? State::Coasting : State::Inactive);
}

void ContinuousAlignment::NoteTargetResolved(double time)
{
	Reset(ResetReason::TargetResolved);
	lastTargetResolveTime = time;
}

bool ContinuousAlignment::DeriveMountExtrinsic(const std::vector<PoseSample> &refStream,
                                               const std::vector<PoseSample> &targetStream,
                                               const EngineResult &calibration,
                                               MountExtrinsic &out)
{
	// Fixed policy: the speed/interpolation gates that decide which pairs are
	// usable, and the rigidity gate that decides whether continuous
	// calibration arms at all.
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
