#include "CalibrationEngine.h"

#include <algorithm>
#include <cmath>

namespace questcal
{

namespace
{

const Eigen::Vector3d kUp(0.0, 1.0, 0.0);

// Shortest-arc axis/angle of a delta rotation, via quaternions.
// Quaternion extraction keeps the axis well-conditioned even near 180 degrees,
// where the matrix off-diagonal method degrades.
bool DeltaAxis(const Eigen::Quaterniond &from, const Eigen::Quaterniond &to,
               double minAngle, Eigen::Vector3d &axisOut, double &angleOut)
{
	Eigen::Quaterniond dq = to * from.conjugate();
	dq.normalize();
	if (dq.w() < 0.0)
		dq.coeffs() = -dq.coeffs();   // shortest arc

	double sinHalf = dq.vec().norm();
	double angle = 2.0 * std::atan2(sinHalf, dq.w());
	if (angle < minAngle || sinHalf < 1e-9)
		return false;

	axisOut = dq.vec() / sinHalf;
	angleOut = angle;
	return true;
}

struct AxisPair
{
	Eigen::Vector3d ref;
	Eigen::Vector3d target;
	double weight = 1.0;
};

// Weighted Kabsch solving ref ~= R * target over unit axis pairs,
// with proper reflection handling.
Eigen::Matrix3d WeightedKabsch(const std::vector<AxisPair> &pairs, double gravityWeight)
{
	Eigen::Matrix3d cross = Eigen::Matrix3d::Zero();
	for (const auto &p : pairs)
		cross += p.weight * (p.target * p.ref.transpose());

	// Gravity prior: a virtual pair mapping target-up onto reference-up. A prior,
	// not a constraint — enough real off-vertical axis content outvotes it.
	cross += gravityWeight * (kUp * kUp.transpose());

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(cross, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
	d(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
	return svd.matrixV() * d * svd.matrixU().transpose();
}

double AngularSpeedAt(const std::vector<PoseSample> &stream, size_t i)
{
	double reported = stream[i].angVel.norm();
	if (reported > 1e-6)
		return reported;

	// Fallback: finite difference against the next sample.
	if (i + 1 >= stream.size())
		return 0.0;
	double dt = stream[i + 1].time - stream[i].time;
	if (dt <= 1e-6)
		return 0.0;
	Eigen::Quaterniond dq = stream[i + 1].rot * stream[i].rot.conjugate();
	dq.normalize();
	if (dq.w() < 0.0)
		dq.coeffs() = -dq.coeffs();
	double angle = 2.0 * std::atan2(dq.vec().norm(), dq.w());
	return angle / dt;
}

// Piecewise-linear resample of an angular-speed profile onto a uniform grid.
std::vector<double> ResampleSpeed(const std::vector<PoseSample> &stream,
                                  double t0, double dt, size_t count)
{
	std::vector<double> out(count, 0.0);
	if (stream.size() < 2)
		return out;

	size_t j = 0;
	for (size_t i = 0; i < count; ++i)
	{
		double t = t0 + dt * static_cast<double>(i);
		while (j + 1 < stream.size() && stream[j + 1].time < t)
			++j;
		if (j + 1 >= stream.size())
			break;
		double ta = stream[j].time, tb = stream[j + 1].time;
		double f = (tb - ta > 1e-9) ? (t - ta) / (tb - ta) : 0.0;
		f = std::min(1.0, std::max(0.0, f));
		double sa = AngularSpeedAt(stream, j);
		double sb = AngularSpeedAt(stream, j + 1);
		out[i] = sa + f * (sb - sa);
	}
	return out;
}

double Mean(const std::vector<double> &v)
{
	if (v.empty())
		return 0.0;
	double s = 0.0;
	for (double x : v) s += x;
	return s / static_cast<double>(v.size());
}

} // namespace

bool CalibrationEngine::InterpolateAt(const std::vector<PoseSample> &stream, double t,
                                      double maxGap, PoseSample &out)
{
	if (stream.size() < 2 || t < stream.front().time || t > stream.back().time)
		return false;

	// Binary search for the first sample with time >= t.
	size_t lo = 0, hi = stream.size() - 1;
	while (lo + 1 < hi)
	{
		size_t mid = (lo + hi) / 2;
		if (stream[mid].time < t) lo = mid; else hi = mid;
	}

	const PoseSample &a = stream[lo];
	const PoseSample &b = stream[hi];
	double gap = b.time - a.time;
	if (gap > maxGap)
		return false;   // tracking dropout; do not bridge it

	double f = (gap > 1e-9) ? (t - a.time) / gap : 0.0;
	f = std::min(1.0, std::max(0.0, f));

	out.time = t;
	out.rot = a.rot.slerp(f, b.rot);
	out.pos = a.pos + f * (b.pos - a.pos);
	out.vel = a.vel + f * (b.vel - a.vel);
	out.angVel = a.angVel + f * (b.angVel - a.angVel);
	return true;
}

bool CalibrationEngine::EstimateTimeOffset(const std::vector<PoseSample> &refStream,
                                           const std::vector<PoseSample> &targetStream,
                                           const EngineConfig &config,
                                           double &offsetOut)
{
	offsetOut = 0.0;
	if (refStream.size() < 8 || targetStream.size() < 8)
		return false;

	double start = std::max(refStream.front().time, targetStream.front().time) + config.timeOffsetRange;
	double end = std::min(refStream.back().time, targetStream.back().time) - config.timeOffsetRange;
	if (end - start < 0.5)
		return false;   // not enough overlap to correlate

	double dt = std::max(1e-3, config.timeOffsetStep * 0.5);
	size_t count = static_cast<size_t>((end - start) / dt);
	if (count < 64)
		return false;

	std::vector<double> targetSpeed = ResampleSpeed(targetStream, start, dt, count);
	double targetMean = Mean(targetSpeed);
	for (double &x : targetSpeed) x -= targetMean;

	double targetNorm = 0.0;
	for (double x : targetSpeed) targetNorm += x * x;
	if (targetNorm < 1e-8)
		return false;   // no rotation happening; nothing to correlate

	// Physical event at time T shows up in the reference stream at T and in the
	// target stream at T + offset; so targetSpeed(t) matches refSpeed(t - offset).
	double bestOffset = 0.0, bestScore = -2.0;
	std::vector<double> scores;
	int steps = static_cast<int>(config.timeOffsetRange / config.timeOffsetStep);
	for (int k = -steps; k <= steps; ++k)
	{
		double lag = static_cast<double>(k) * config.timeOffsetStep;
		std::vector<double> refSpeed = ResampleSpeed(refStream, start - lag, dt, count);
		double refMean = Mean(refSpeed);
		double dot = 0.0, refNorm = 0.0;
		for (size_t i = 0; i < count; ++i)
		{
			double r = refSpeed[i] - refMean;
			dot += r * targetSpeed[i];
			refNorm += r * r;
		}
		double score = (refNorm > 1e-8) ? dot / std::sqrt(refNorm * targetNorm) : -2.0;
		scores.push_back(score);
		if (score > bestScore)
		{
			bestScore = score;
			bestOffset = lag;
		}
	}

	if (bestScore < 0.25)
		return false;   // no meaningful correlation peak

	// Parabolic refinement around the discrete peak.
	int bestIdx = static_cast<int>((bestOffset + config.timeOffsetRange) / config.timeOffsetStep + 0.5);
	if (bestIdx > 0 && bestIdx + 1 < static_cast<int>(scores.size()))
	{
		double y0 = scores[bestIdx - 1], y1 = scores[bestIdx], y2 = scores[bestIdx + 1];
		double denom = y0 - 2.0 * y1 + y2;
		if (std::abs(denom) > 1e-12)
		{
			double shift = 0.5 * (y0 - y2) / denom;
			if (std::abs(shift) <= 1.0)
				bestOffset += shift * config.timeOffsetStep;
		}
	}

	offsetOut = bestOffset;
	return true;
}

EngineResult CalibrationEngine::SolveAligned(const std::vector<AlignedSample> &samples,
                                             const EngineConfig &config)
{
	EngineResult result;
	result.samplesUsed = samples.size();

	if (samples.size() < 8)
	{
		result.message = "Not enough samples collected.";
		return result;
	}

	// ---- delta-rotation axis pairs over multiple baselines ----------------
	// Multi-lag pairing gives diverse baselines with O(n * lags) cost and less
	// inter-pair correlation than the full O(n^2) set.
	static const size_t kLags[] = { 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233 };

	std::vector<AxisPair> pairs;
	for (size_t lag : kLags)
	{
		if (lag >= samples.size())
			break;
		for (size_t i = 0; i + lag < samples.size(); ++i)
		{
			if (pairs.size() >= config.maxPairs)
				break;
			const AlignedSample &a = samples[i];
			const AlignedSample &b = samples[i + lag];

			Eigen::Vector3d refAxis, targetAxis;
			double refAngle = 0.0, targetAngle = 0.0;
			if (!DeltaAxis(a.ref.rot, b.ref.rot, config.minPairAngle, refAxis, refAngle) ||
			    !DeltaAxis(a.target.rot, b.target.rot, config.minPairAngle, targetAxis, targetAngle))
			{
				result.pairsRejected++;
				continue;
			}

			// The devices are rigid: both deltas must rotate by the same angle.
			// A large mismatch means jitter or a timing glitch on this pair.
			if (std::abs(refAngle - targetAngle) > 0.35)
			{
				result.pairsRejected++;
				continue;
			}

			pairs.push_back({ refAxis, targetAxis, 1.0 });
		}
	}
	result.pairsUsed = pairs.size();

	if (pairs.size() < config.minPairs)
	{
		result.message = "Not enough rotation — rotate the devices together, at least a quarter turn at a time.";
		return result;
	}

	// ---- axis diversity (conditioning) ------------------------------------
	// Outer-product accumulation is sign-invariant, so the per-pair axis sign
	// ambiguity cannot corrupt it (unlike averaging the axes directly).
	Eigen::Matrix3d axisCov = Eigen::Matrix3d::Zero();
	for (const auto &p : pairs)
		axisCov += p.ref * p.ref.transpose();
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> axisEig(axisCov);
	// Eigenvalues ascend: [2] is largest.
	result.axisSpread = axisEig.eigenvalues()(1) / std::max(1e-12, axisEig.eigenvalues()(2));

	// ---- rotation: weighted Kabsch + gravity prior + IRLS ------------------
	// The prior is scaled by how well the data itself constrains tilt: with rich
	// two-axis motion it fades to a nudge (no bias on genuinely tilted setups);
	// near the single-axis regime it dominates, keeping the ill-constrained tilt
	// DOF gravity-aligned instead of noise-driven.
	double priorScale = 1.0;
	double richSpread = config.minAxisSpread * 4.0;
	if (result.axisSpread > richSpread)
		priorScale = richSpread / result.axisSpread;
	double gravityRatio = config.gravityPriorRatio * priorScale;

	double dataWeight = static_cast<double>(pairs.size());
	Eigen::Matrix3d rot = WeightedKabsch(pairs, gravityRatio * dataWeight);

	for (int iter = 0; iter < config.irlsIterations; ++iter)
	{
		for (auto &p : pairs)
		{
			double c = (rot * p.target).dot(p.ref);
			double residual = std::acos(std::min(1.0, std::max(-1.0, c)));
			p.weight = (residual <= config.huberRotation) ? 1.0 : config.huberRotation / residual;
		}
		double wsum = 0.0;
		for (const auto &p : pairs) wsum += p.weight;
		rot = WeightedKabsch(pairs, gravityRatio * wsum);
	}

	double rotResidualSq = 0.0, rotWeight = 0.0;
	for (const auto &p : pairs)
	{
		double c = (rot * p.target).dot(p.ref);
		double residual = std::acos(std::min(1.0, std::max(-1.0, c)));
		rotResidualSq += p.weight * residual * residual;
		rotWeight += p.weight;
	}
	result.rotationRmsDeg = std::sqrt(rotResidualSq / std::max(1e-12, rotWeight)) * 180.0 / EIGEN_PI;
	result.rotation = Eigen::Quaterniond(rot);
	result.tiltDeg = std::acos(std::min(1.0, std::max(-1.0, (rot * kUp).dot(kUp)))) * 180.0 / EIGEN_PI;

	// ---- translation (+ optional scale): weighted linear least squares -----
	// math.pdf eq. 8 over sample pairs, with the target universe pre-rotated by
	// the solved rotation. Row blocks are linear in the scale, so scale search
	// reuses the same decomposition inputs.
	struct TransRow
	{
		Eigen::Matrix3d dQ;
		Eigen::Vector3d base;     // constant part of the RHS
		Eigen::Vector3d scalePart; // part multiplied by scale
		double weight = 1.0;
	};
	std::vector<TransRow> rows;

	static const size_t kTransLags[] = { 1, 3, 8, 21, 55, 144 };
	for (size_t lag : kTransLags)
	{
		if (lag >= samples.size())
			break;
		for (size_t i = 0; i + lag < samples.size(); ++i)
		{
			const AlignedSample &si = samples[i];
			const AlignedSample &sj = samples[i + lag];

			Eigen::Matrix3d QAi = si.ref.rot.toRotationMatrix().transpose();
			Eigen::Matrix3d QAj = sj.ref.rot.toRotationMatrix().transpose();
			Eigen::Matrix3d QBi = (rot * si.target.rot.toRotationMatrix()).transpose();
			Eigen::Matrix3d QBj = (rot * sj.target.rot.toRotationMatrix()).transpose();

			Eigen::Vector3d refI = si.ref.pos, refJ = sj.ref.pos;
			Eigen::Vector3d tgtI = rot * si.target.pos, tgtJ = rot * sj.target.pos;

			TransRow ra;
			ra.dQ = QAj - QAi;
			ra.base = QAj * refJ - QAi * refI;
			ra.scalePart = -(QAj * tgtJ - QAi * tgtI);
			rows.push_back(ra);

			TransRow rb;
			rb.dQ = QBj - QBi;
			rb.base = QBj * refJ - QBi * refI;
			rb.scalePart = -(QBj * tgtJ - QBi * tgtI);
			rows.push_back(rb);
		}
	}

	auto solveTranslation = [&rows, &config](double scale, Eigen::Vector3d &tOut) -> double
	{
		for (int iter = 0; iter <= config.irlsIterations; ++iter)
		{
			Eigen::Matrix3d ata = Eigen::Matrix3d::Zero();
			Eigen::Vector3d atb = Eigen::Vector3d::Zero();
			for (const auto &r : rows)
			{
				Eigen::Vector3d rhs = r.base + scale * r.scalePart;
				ata += r.weight * (r.dQ.transpose() * r.dQ);
				atb += r.weight * (r.dQ.transpose() * rhs);
			}
			tOut = ata.ldlt().solve(atb);

			if (iter == config.irlsIterations)
				break;
			for (auto &r : rows)
			{
				double residual = (r.dQ * tOut - (r.base + scale * r.scalePart)).norm();
				r.weight = (residual <= config.huberTranslation) ? 1.0 : config.huberTranslation / residual;
			}
		}

		double sq = 0.0, wsum = 0.0;
		for (const auto &r : rows)
		{
			double residual = (r.dQ * tOut - (r.base + scale * r.scalePart)).norm();
			sq += r.weight * residual * residual;
			wsum += r.weight;
		}
		return std::sqrt(sq / std::max(1e-12, wsum));
	};

	double scale = 1.0;
	Eigen::Vector3d translation;
	double transRms = solveTranslation(1.0, translation);

	if (config.solveScale)
	{
		// Golden-section search over the residual; the function is smooth and
		// near-quadratic around the optimum.
		const double phi = 0.6180339887498949;
		double lo = 1.0 - config.scaleSearchRange, hi = 1.0 + config.scaleSearchRange;
		double x1 = hi - phi * (hi - lo), x2 = lo + phi * (hi - lo);
		Eigen::Vector3d tTmp;
		double f1 = solveTranslation(x1, tTmp), f2 = solveTranslation(x2, tTmp);
		for (int it = 0; it < 24; ++it)
		{
			if (f1 < f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - phi * (hi - lo); f1 = solveTranslation(x1, tTmp); }
			else         { lo = x1; x1 = x2; f1 = f2; x2 = lo + phi * (hi - lo); f2 = solveTranslation(x2, tTmp); }
		}
		scale = (lo + hi) * 0.5;
		transRms = solveTranslation(scale, translation);
	}

	// The solved translation maps pre-rotated, pre-scaled target space; what the
	// driver applies is world-from-driver, which matches this frame directly.
	result.translation = translation;
	result.scale = scale;
	result.translationRmsMeters = transRms;

	// ---- validation --------------------------------------------------------
	if (result.axisSpread < config.minAxisSpread)
	{
		result.message = "Rotation happened around only one axis — tilt and roll are unconstrained. "
		                 "Rotate the devices together around two different axes and recalibrate.";
		return result;
	}
	if (result.rotationRmsDeg > config.maxRotationRms)
	{
		result.message = "Rotation residual too high (" + std::to_string(result.rotationRmsDeg).substr(0, 4) +
		                 " deg) — tracking is jittery or the devices are not rigidly attached.";
		return result;
	}
	if (result.translationRmsMeters > config.maxTranslationRms)
	{
		result.message = "Position residual too high (" +
		                 std::to_string(result.translationRmsMeters * 100.0).substr(0, 4) +
		                 " cm) — tracking is jittery, or motion was too fast for the sample rate.";
		return result;
	}

	result.valid = true;
	result.message = "Calibration OK.";
	return result;
}

EngineResult CalibrationEngine::Solve(const std::vector<PoseSample> &refStream,
                                      const std::vector<PoseSample> &targetStream,
                                      const EngineConfig &config)
{
	EngineResult failure;

	if (refStream.size() < 8 || targetStream.size() < 8)
	{
		failure.message = "Not enough samples collected.";
		return failure;
	}

	// ---- inter-system time alignment --------------------------------------
	double offset = 0.0;
	bool offsetKnown = false;
	if (config.estimateTimeOffset)
		offsetKnown = EstimateTimeOffset(refStream, targetStream, config, offset);

	// ---- pair up samples at target timestamps ------------------------------
	std::vector<AlignedSample> aligned;
	size_t gated = 0;
	for (const auto &t : targetStream)
	{
		PoseSample refAt;
		if (!InterpolateAt(refStream, t.time - offset, config.maxInterpolationGap, refAt))
			continue;

		// Velocity gate: timing error converts speed directly into position
		// error, and any residual latency estimate error does the same.
		if (t.vel.norm() > config.maxLinearSpeed || refAt.vel.norm() > config.maxLinearSpeed ||
		    t.angVel.norm() > config.maxAngularSpeed || refAt.angVel.norm() > config.maxAngularSpeed)
		{
			gated++;
			continue;
		}

		aligned.push_back({ t.time, refAt, t });
	}

	// Thin evenly to the configured budget.
	if (aligned.size() > config.maxAlignedSamples)
	{
		std::vector<AlignedSample> thinned;
		thinned.reserve(config.maxAlignedSamples);
		double stride = static_cast<double>(aligned.size()) / static_cast<double>(config.maxAlignedSamples);
		for (size_t i = 0; i < config.maxAlignedSamples; ++i)
			thinned.push_back(aligned[static_cast<size_t>(i * stride)]);
		aligned.swap(thinned);
	}

	EngineResult result = SolveAligned(aligned, config);
	result.timeOffset = offset;
	result.samplesGated = gated;
	if (config.estimateTimeOffset && !offsetKnown)
		result.message += " (time offset could not be estimated; assuming zero)";
	return result;
}

double ComputeAppliedTimeOffset(double solvedTimeOffset,
                                double maxDelaySeconds,
                                double maxAdvanceSeconds)
{
	double shift = -solvedTimeOffset;
	if (shift > maxDelaySeconds)
		shift = maxDelaySeconds;
	if (shift < -maxAdvanceSeconds)
		shift = -maxAdvanceSeconds;
	return shift;
}

} // namespace questcal
