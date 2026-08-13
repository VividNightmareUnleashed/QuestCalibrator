#include "CalibrationEngine.h"
#include "../common/TransformLimits.h"

#include <algorithm>
#include <cmath>

namespace questcal
{

namespace
{

const Eigen::Vector3d kUp(0.0, 1.0, 0.0);
// Eigen's slerp/toRotationMatrix paths require unit quaternions. A 1e-3
// squared-norm tolerance admits ordinary floating-point drift (~0.05% in the
// norm) while rejecting scaled representations that would corrupt the solve.
constexpr double MaxQuaternionNormSquaredError = 1e-3;
constexpr size_t MaxSolverSampleBudget = 1000000;
constexpr size_t MaxSolverPairBudget = 1000000;
constexpr double MaxTimeOffsetSearchSteps = 100000.0;
constexpr double MaxResampledPointCount = 1000000.0;
constexpr double MaxCorrelationWork = 100000000.0;

// Returns nullptr when the config is usable, else the field or relation that
// failed. The alternative on this path is a calibration that silently does not
// run, so the refusal has to name the knob: only two of these vary in
// production today, which means the first real failure will belong to whoever
// adds the next one. Grouped by what each check defends.
const char *ConfigError(const EngineConfig &config)
{
	auto finiteNonnegative = [](double value)
	{
		return std::isfinite(value) && value >= 0.0;
	};
	auto finitePositive = [](double value)
	{
		return std::isfinite(value) && value > 0.0;
	};

	// --- per-field bounds ---
	if (!finiteNonnegative(config.timeOffsetRange) ||
		config.timeOffsetRange > protocol::limits::MaxAbsTimeOffsetSeconds)
		return "timeOffsetRange";
	if (!finitePositive(config.timeOffsetStep))
		return "timeOffsetStep";
	if (!finiteNonnegative(config.maxLinearSpeed) ||
		config.maxLinearSpeed > protocol::limits::MaxAbsLinearVelocityMetersPerSecond)
		return "maxLinearSpeed";
	if (!finiteNonnegative(config.maxAngularSpeed) ||
		config.maxAngularSpeed > protocol::limits::MaxAbsAngularVelocityRadiansPerSecond)
		return "maxAngularSpeed";
	if (!finiteNonnegative(config.maxInterpolationGap) ||
		config.maxInterpolationGap > protocol::limits::MaxAbsTimeOffsetSeconds)
		return "maxInterpolationGap";
	if (config.maxAlignedSamples < 8)
		return "maxAlignedSamples";
	if (!finiteNonnegative(config.minPairAngle))
		return "minPairAngle";
	if (!std::isfinite(config.maxPairAngle) || config.maxPairAngle > EIGEN_PI)
		return "maxPairAngle";
	if (config.minPairs < 3)
		return "minPairs";
	if (!finiteNonnegative(config.gravityPriorRatio) ||
		config.gravityPriorRatio > 1000000.0)
		return "gravityPriorRatio";
	if (config.irlsIterations < 0 || config.irlsIterations > 100)
		return "irlsIterations";
	if (!finitePositive(config.huberRotation))
		return "huberRotation";
	if (!finitePositive(config.huberTranslation))
		return "huberTranslation";
	if (config.refineIterations < 0 || config.refineIterations > 100)
		return "refineIterations";
	if (!finitePositive(config.gainSplitSeconds) || config.gainSplitSeconds > 3600.0)
		return "gainSplitSeconds";
	if (!finiteNonnegative(config.gainSmoothingMargin))
		return "gainSmoothingMargin";
	if (!finiteNonnegative(config.maxCleanGrossDeviation))
		return "maxCleanGrossDeviation";
	if (!finiteNonnegative(config.maxRotationRms))
		return "maxRotationRms";
	if (!finiteNonnegative(config.maxTranslationRms))
		return "maxTranslationRms";
	if (!finiteNonnegative(config.minAxisSpread) || config.minAxisSpread > 1.0)
		return "minAxisSpread";
	if (!finiteNonnegative(config.minTransEigRatio) || config.minTransEigRatio > 1.0)
		return "minTransEigRatio";

	// --- ordering relations between fields ---
	if (!(config.maxPairAngle > config.minPairAngle))
		return "maxPairAngle must exceed minPairAngle";
	if (!(config.maxPairs >= config.minPairs))
		return "maxPairs must be at least minPairs";
	// A scale search that can reach below the protocol's own floor would solve
	// a scale the driver is not allowed to apply.
	if (!finiteNonnegative(config.scaleSearchRange) ||
		config.scaleSearchRange > 1.0 - protocol::limits::MinScale)
		return "scaleSearchRange exceeds the protocol minimum scale";

	// --- work budgets (bounded above by the field checks, so divisions here
	//     are already safe) ---
	if (config.timeOffsetRange / config.timeOffsetStep > MaxTimeOffsetSearchSteps)
		return "timeOffsetRange / timeOffsetStep exceeds the search-step budget";
	if (config.maxAlignedSamples > MaxSolverSampleBudget)
		return "maxAlignedSamples exceeds the solver sample budget";
	if (config.maxPairs > MaxSolverPairBudget)
		return "maxPairs exceeds the solver pair budget";

	return nullptr;
}

bool IsFinitePose(const PoseSample &sample)
{
	double rotationNormSquared = sample.rot.squaredNorm();
	return std::isfinite(sample.time) &&
		std::abs(sample.time) <= protocol::limits::MaxAbsPoseTimestampSeconds &&
		sample.rot.coeffs().allFinite() &&
		std::isfinite(rotationNormSquared) &&
		std::abs(rotationNormSquared - 1.0) <= MaxQuaternionNormSquaredError &&
		sample.pos.allFinite() &&
		sample.pos.cwiseAbs().maxCoeff() <=
			protocol::limits::MaxAbsPosePositionMeters &&
		sample.vel.allFinite() &&
		sample.vel.cwiseAbs().maxCoeff() <=
			protocol::limits::MaxAbsLinearVelocityMetersPerSecond &&
		sample.angVel.allFinite() &&
		sample.angVel.cwiseAbs().maxCoeff() <=
			protocol::limits::MaxAbsAngularVelocityRadiansPerSecond;
}

bool IsValidStream(const std::vector<PoseSample> &stream)
{
	for (size_t i = 0; i < stream.size(); ++i)
	{
		if (!IsFinitePose(stream[i]))
			return false;
		if (i > 0 && !(stream[i].time > stream[i - 1].time))
			return false;
	}
	return true;
}

bool IsValidAlignedSamples(const std::vector<AlignedSample> &samples)
{
	for (size_t i = 0; i < samples.size(); ++i)
	{
		if (!std::isfinite(samples[i].time) ||
		    std::abs(samples[i].time) > protocol::limits::MaxAbsPoseTimestampSeconds ||
		    !IsFinitePose(samples[i].ref) ||
		    !IsFinitePose(samples[i].target))
			return false;
		if (i > 0 && !(samples[i].time > samples[i - 1].time))
			return false;
	}
	return true;
}

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
	// Inverse-variance base weight: the axis direction of a noisy delta
	// rotation carries noise ~ sigma/theta, so a near-threshold pair's axis is
	// several times noisier than a wide sweep's yet would otherwise vote
	// equally (Park & Martin's insight, applied to the axis-pair form). The
	// Huber IRLS factor multiplies on top of this rather than replacing it.
	double base = 1.0;
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

// Frequency-split amplitude gain of the reference stream's position track
// relative to the target's, over a time-aligned uniform resampling. The
// comparison track is the SOLVED MODEL's prediction of the reference device's
// position (R(s p_B) + T - Q_A d, with the mount arm d estimated from the
// data): comparing against the raw target track instead would read the
// rotation-driven mount-lever motion — which the target has and the reference
// lacks — as a phantom frequency tilt. Gains are reported in ref-vs-target
// terms (the solved scale times the measured ratio), so under a genuine
// metric difference both bands sit at the true scale. See
// EngineConfig::gainSplitSeconds for what the two bands mean.
bool EstimateMotionGain(const std::vector<PoseSample> &refStream,
                        const std::vector<PoseSample> &targetStream,
                        double offset, const EngineConfig &config,
                        const EngineResult &solved,
                        double &lowOut, double &highOut)
{
	if (refStream.size() < 2 || targetStream.size() < 2)
		return false;

	const double dt = 0.02;
	double start = std::max(refStream.front().time + offset, targetStream.front().time);
	double end = std::min(refStream.back().time + offset, targetStream.back().time);
	if (end - start < 5.0)
		return false;
	double resampledCount = (end - start) / dt + 1.0;
	if (!std::isfinite(resampledCount) || resampledCount > MaxResampledPointCount)
		return false;

	std::vector<PoseSample> refAt, tgtAt;
	refAt.reserve(static_cast<size_t>(resampledCount));
	tgtAt.reserve(refAt.capacity());
	for (double t = start; t <= end; t += dt)
	{
		PoseSample a, b;
		if (!CalibrationEngine::InterpolateAt(targetStream, t, config.maxInterpolationGap, b) ||
		    !CalibrationEngine::InterpolateAt(refStream, t - offset, config.maxInterpolationGap, a))
			continue;
		tgtAt.push_back(b);
		refAt.push_back(a);
	}

	const size_t n = refAt.size();
	const size_t W = std::max<size_t>(3, static_cast<size_t>(config.gainSplitSeconds / dt));
	if (n < 250 || n < 4 * W)
		return false;

	// Mount arm from the reference device to the target, in the reference body
	// frame; rough is fine (the possibly contaminated solved scale moves it by
	// millimeters, second-order here).
	Eigen::Vector3d arm = Eigen::Vector3d::Zero();
	for (size_t i = 0; i < n; ++i)
		arm += refAt[i].rot.conjugate() *
			(solved.rotation * (solved.scale * tgtAt[i].pos) + solved.translation - refAt[i].pos);
	arm /= static_cast<double>(n);

	// Predicted reference-device track: the model maps the target to the
	// target's own position, so the reference sits one lever arm back from it.
	std::vector<Eigen::Vector3d> rp(n), tp(n);
	for (size_t i = 0; i < n; ++i)
	{
		rp[i] = refAt[i].pos;
		tp[i] = solved.rotation * (solved.scale * tgtAt[i].pos) + solved.translation
			- refAt[i].rot * arm;
	}

	auto bandRms = [n, W](const std::vector<Eigen::Vector3d> &p, double &lowRms, double &highRms)
	{
		std::vector<Eigen::Vector3d> prefix(n + 1, Eigen::Vector3d::Zero());
		for (size_t i = 0; i < n; ++i)
			prefix[i + 1] = prefix[i] + p[i];
		Eigen::Vector3d meanAll = prefix[n] / static_cast<double>(n);

		double sqLow = 0.0, sqHigh = 0.0;
		size_t m = 0;
		for (size_t i = W; i + W < n; ++i)
		{
			Eigen::Vector3d ma = (prefix[i + W + 1] - prefix[i - W])
				/ static_cast<double>(2 * W + 1);
			sqHigh += (p[i] - ma).squaredNorm();
			sqLow += (ma - meanAll).squaredNorm();
			++m;
		}
		lowRms = m ? std::sqrt(sqLow / static_cast<double>(m)) : 0.0;
		highRms = m ? std::sqrt(sqHigh / static_cast<double>(m)) : 0.0;
	};

	double rLow = 0.0, rHigh = 0.0, tLow = 0.0, tHigh = 0.0;
	bandRms(rp, rLow, rHigh);
	bandRms(tp, tLow, tHigh);

	// A band with too little motion cannot support a ratio (the slow-cautious
	// or occluded cases); refuse rather than divide noise by noise.
	if (tLow < 0.03 || tHigh < 0.008)
		return false;

	// The predicted track already carries the solved scale, so fold it back in
	// to report gains in ref-vs-target terms: under a genuine metric
	// difference both bands read the true scale, under smoothing the fine
	// band reads below the gross band.
	lowOut = solved.scale * (rLow / tLow);
	highOut = solved.scale * (rHigh / tHigh);
	return true;
}

// Rotation vector of a delta near the identity (shortest arc), zero-safe.
Eigen::Vector3d LogVec(const Eigen::Quaterniond &q)
{
	Eigen::Quaterniond dq = q;
	if (dq.w() < 0.0)
		dq.coeffs() = -dq.coeffs();
	double sinHalf = dq.vec().norm();
	if (sinHalf < 1e-12)
		return Eigen::Vector3d::Zero();
	return dq.vec() * (2.0 * std::atan2(sinHalf, dq.w()) / sinHalf);
}

// Joint Gauss-Newton polish over rotation, translation, the body-frame mount
// transform (offset d, rotation C — both nuisance parameters the pairwise
// eq. 6/8 formulations eliminate), and optional scale, on BOTH residual sets:
//   position:    e_p = pA_i - R (s pB_i) - t - QA_i d
//   orientation: e_r = kRot * Log(QA_i^T R QB_i C)
// plus the same gravity prior the Kabsch stage uses (as a virtual residual),
// so the polish fuses the two information sources instead of trading one for
// the other. The sequential pipeline treats the Kabsch rotation as exact in
// the translation solve, so its residual error otherwise leaks into t as a
// bias whose lever arm is the play-space size; positions alone, though, carry
// far less rotation information than the orientations (short mount lever) and
// a position-only polish would chase position noise away from the
// orientation-optimal rotation. kRot converts radians to meters so one Huber
// knee (huberTranslation) governs both sets. Writes back and returns true
// only when the combined robust cost improved.
bool JointRefine(const std::vector<AlignedSample> &samples, const EngineConfig &config,
                 double gravityRatio,
                 Eigen::Matrix3d &rotInOut, Eigen::Vector3d &transInOut, double &scaleInOut)
{
	const Eigen::Vector3d up(0.0, 1.0, 0.0);
	const double n = static_cast<double>(samples.size());
	const double kRot = config.huberTranslation / config.huberRotation;   // m per rad
	const double gravityWeight = std::max(0.0, gravityRatio) * n;

	Eigen::Matrix3d R = rotInOut;
	Eigen::Vector3d t = transInOut;
	double s = scaleInOut;
	const bool solveS = config.solveScale;
	const int dim = solveS ? 13 : 12;   // [omega, t, d, gamma, (s)]

	// Initial mount transform: means of the per-sample values at the seed.
	Eigen::Vector3d d = Eigen::Vector3d::Zero();
	Eigen::Matrix4d cAccum = Eigen::Matrix4d::Zero();   // eigenvector quaternion mean
	for (const auto &a : samples)
	{
		d += a.ref.rot.conjugate() * (a.ref.pos - R * (s * a.target.pos) - t);
		Eigen::Quaterniond ci = (Eigen::Quaterniond(R) * a.target.rot).conjugate() * a.ref.rot;
		Eigen::Vector4d v = ci.coeffs();
		cAccum += v * v.transpose();
	}
	d /= n;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> cEig(cAccum);
	Eigen::Vector4d cv = cEig.eigenvectors().col(3);
	Eigen::Quaterniond C(cv(3), cv(0), cv(1), cv(2));
	C.normalize();

	auto robustCost = [&](const Eigen::Matrix3d &Rc, const Eigen::Vector3d &tc,
	                      const Eigen::Vector3d &dc, const Eigen::Quaterniond &Cc,
	                      double sc) -> double
	{
		Eigen::Quaterniond Rq(Rc);
		double sq = 0.0;
		for (const auto &a : samples)
		{
			double resP = (a.ref.pos - Rc * (sc * a.target.pos) - tc - a.ref.rot * dc).norm();
			double wP = (resP <= config.huberTranslation) ? 1.0 : config.huberTranslation / resP;
			double resR = kRot * LogVec(a.ref.rot.conjugate() * (Rq * a.target.rot * Cc)).norm();
			double wR = (resR <= config.huberTranslation) ? 1.0 : config.huberTranslation / resR;
			sq += wP * resP * resP + wR * resR * resR;
		}
		sq += gravityWeight * (kRot * kRot) * (Rc * up - up).squaredNorm();
		return sq;
	};

	const double before = robustCost(R, t, d, C, s);
	if (!std::isfinite(before))
		return false;

	for (int iter = 0; iter < config.refineIterations; ++iter)
	{
		Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
		Eigen::VectorXd g = Eigen::VectorXd::Zero(dim);
		Eigen::Quaterniond Rq(R);

		auto addResidual = [&](const Eigen::Matrix<double, 3, 13> &J, const Eigen::Vector3d &e)
		{
			double res = e.norm();
			double w = (res <= config.huberTranslation) ? 1.0 : config.huberTranslation / res;
			H += w * (J.leftCols(dim).transpose() * J.leftCols(dim));
			g += w * (J.leftCols(dim).transpose() * e);
		};

		for (const auto &a : samples)
		{
			// Position residual.
			Eigen::Vector3d rv = R * (s * a.target.pos);
			Eigen::Vector3d eP = a.ref.pos - rv - t - a.ref.rot * d;

			Eigen::Matrix<double, 3, 13> Jp = Eigen::Matrix<double, 3, 13>::Zero();
			// Left perturbation R <- exp(omega) R: d eP / d omega = [R s pB]x.
			Jp.block<3, 3>(0, 0) <<
			        0.0, -rv.z(),  rv.y(),
			     rv.z(),     0.0, -rv.x(),
			    -rv.y(),  rv.x(),     0.0;
			Jp.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
			Jp.block<3, 3>(0, 6) = -a.ref.rot.toRotationMatrix();
			if (solveS)
				Jp.col(12) = -(R * a.target.pos);
			addResidual(Jp, eP);

			// Orientation residual (scaled to meters by kRot).
			Eigen::Vector3d eR = kRot * LogVec(a.ref.rot.conjugate() * (Rq * a.target.rot * C));

			Eigen::Matrix<double, 3, 13> Jr = Eigen::Matrix<double, 3, 13>::Zero();
			// d eR / d omega = kRot * QA^T; right perturbation C <- C exp(gamma):
			// d eR / d gamma = kRot * I (both to first order near small residuals).
			Jr.block<3, 3>(0, 0) = kRot * a.ref.rot.conjugate().toRotationMatrix();
			Jr.block<3, 3>(0, 9) = kRot * Eigen::Matrix3d::Identity();
			addResidual(Jr, eR);
		}

		// Gravity prior as a virtual residual, mirroring the Kabsch stage's.
		if (gravityWeight > 0.0)
		{
			Eigen::Vector3d ru = R * up;
			Eigen::Vector3d eG = std::sqrt(gravityWeight) * kRot * (ru - up);
			Eigen::Matrix<double, 3, 13> Jg = Eigen::Matrix<double, 3, 13>::Zero();
			Jg.block<3, 3>(0, 0) <<
			        0.0,  ru.z(), -ru.y(),
			    -ru.z(),     0.0,  ru.x(),
			     ru.y(), -ru.x(),     0.0;
			Jg.block<3, 3>(0, 0) *= std::sqrt(gravityWeight) * kRot;
			// Bypass addResidual's Huber: a prior is never an outlier.
			H += Jg.leftCols(dim).transpose() * Jg.leftCols(dim);
			g += Jg.leftCols(dim).transpose() * eG;
		}

		// Light damping keeps a weakly observed direction from blowing up the
		// step (the conditioning gate has already rejected true degeneracy).
		H.diagonal().array() += 1e-6 * H.diagonal().maxCoeff() + 1e-12;

		Eigen::VectorXd delta = -H.ldlt().solve(g);
		if (!delta.allFinite())
			return false;

		// The polish corrects sub-degree Kabsch error; a large rotation ask
		// means an outlier regime this linearization should not chase.
		Eigen::Vector3d omega = delta.segment<3>(0);
		if (omega.norm() > 0.02)
			omega *= 0.02 / omega.norm();
		if (omega.norm() > 1e-14)
			R = Eigen::AngleAxisd(omega.norm(), omega.normalized()).toRotationMatrix() * R;
		t += delta.segment<3>(3);
		d += delta.segment<3>(6);
		Eigen::Vector3d gamma = delta.segment<3>(9);
		if (gamma.norm() > 1e-14)
			C = C * Eigen::Quaterniond(Eigen::AngleAxisd(gamma.norm(), gamma.normalized()));
		if (solveS)
		{
			s += delta(12);
			s = std::min(1.0 + config.scaleSearchRange, std::max(1.0 - config.scaleSearchRange, s));
		}
	}

	R = Eigen::Quaterniond(R).normalized().toRotationMatrix();

	double after = robustCost(R, t, d, C, s);
	if (!std::isfinite(after) || after >= before || !R.allFinite() ||
		!t.allFinite() || !d.allFinite() || !C.coeffs().allFinite() || !std::isfinite(s))
		return false;
	rotInOut = R;
	transInOut = t;
	scaleInOut = s;
	return true;
}

} // namespace

bool CalibrationEngine::InterpolateAt(const std::vector<PoseSample> &stream, double t,
                                      double maxGap, PoseSample &out)
{
	if (stream.size() < 2 || !std::isfinite(t) || !std::isfinite(maxGap) || maxGap < 0.0 ||
	    t < stream.front().time || t > stream.back().time)
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
                                           double &offsetOut,
                                           bool validateInputs)
{
	offsetOut = 0.0;
	if (refStream.size() < 8 || targetStream.size() < 8)
		return false;
	if (ConfigError(config))
		return false;
	if (validateInputs && (!IsValidStream(refStream) || !IsValidStream(targetStream)))
		return false;

	double start = std::max(refStream.front().time, targetStream.front().time) + config.timeOffsetRange;
	double end = std::min(refStream.back().time, targetStream.back().time) - config.timeOffsetRange;
	if (end - start < 0.5)
		return false;   // not enough overlap to correlate

	double dt = std::max(1e-3, config.timeOffsetStep * 0.5);
	double resampledCount = (end - start) / dt;
	if (!std::isfinite(resampledCount) || resampledCount < 64.0 ||
		resampledCount > MaxResampledPointCount)
		return false;
	double searchSteps = config.timeOffsetRange / config.timeOffsetStep;
	if ((2.0 * searchSteps + 1.0) * resampledCount > MaxCorrelationWork)
		return false;
	size_t count = static_cast<size_t>(resampledCount);

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
                                             const EngineConfig &config,
                                             bool validateInputs)
{
	EngineResult result;
	result.samplesUsed = samples.size();
	if (const char *configError = ConfigError(config))
	{
		result.message = "Invalid calibration engine configuration: " +
			std::string(configError) + ".";
		return result;
	}

	if (samples.size() < 8)
	{
		result.message = "Not enough samples collected.";
		return result;
	}
	if (validateInputs && !IsValidAlignedSamples(samples))
	{
		result.message = "Pose samples contain an invalid or out-of-range value, or non-increasing timestamps.";
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

			// Near a half turn the two streams' independent shortest-arc choices
			// can land on opposite hemispheres, feeding Kabsch an anti-aligned
			// pair the angle-mismatch gate below cannot catch (both angles ~pi).
			if (refAngle > config.maxPairAngle || targetAngle > config.maxPairAngle)
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

			pairs.push_back({ refAxis, targetAxis, 1.0, refAngle * targetAngle });
		}
	}
	result.pairsUsed = pairs.size();

	if (pairs.size() < config.minPairs)
	{
		result.message = "Not enough rotation — rotate the devices together, at least a quarter turn at a time.";
		return result;
	}

	// Normalize the angle weights to mean 1 so the gravity-prior ratio and the
	// Huber knee keep their configured meaning regardless of motion scale.
	{
		double sum = 0.0;
		for (const auto &p : pairs)
			sum += p.base;
		double inv = static_cast<double>(pairs.size()) / std::max(1e-12, sum);
		for (auto &p : pairs)
		{
			p.base *= inv;
			p.weight = p.base;
		}
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
			p.weight = p.base *
				((residual <= config.huberRotation) ? 1.0 : config.huberRotation / residual);
		}
		double wsum = 0.0;
		for (const auto &p : pairs) wsum += p.weight;
		rot = WeightedKabsch(pairs, gravityRatio * wsum);
	}

	// Weighted axis-pair residual, reused to guard the joint refinement below.
	auto axisRmsDeg = [&pairs](const Eigen::Matrix3d &rotM) -> double
	{
		double rotResidualSq = 0.0, rotWeight = 0.0;
		for (const auto &p : pairs)
		{
			double c = (rotM * p.target).dot(p.ref);
			double residual = std::acos(std::min(1.0, std::max(-1.0, c)));
			rotResidualSq += p.weight * residual * residual;
			rotWeight += p.weight;
		}
		return std::sqrt(rotResidualSq / std::max(1e-12, rotWeight)) * 180.0 / EIGEN_PI;
	};

	// ---- translation (+ optional scale): weighted linear least squares -----
	// math.pdf eq. 8 over sample pairs, with the target universe pre-rotated by
	// the solved rotation. Row blocks are linear in the scale, so scale search
	// reuses the same decomposition inputs.
	struct TransRow
	{
		Eigen::Matrix3d dQ;
		Eigen::Matrix3d dQtdQ;     // dQ^T dQ; fixed for the row's lifetime
		Eigen::Vector3d base;     // constant part of the RHS
		Eigen::Vector3d scalePart; // part multiplied by scale
		double weight = 1.0;
	};
	std::vector<TransRow> rows;

	auto buildRows = [&samples, &rows](const Eigen::Matrix3d &rotM)
	{
		rows.clear();
		// Each of these is a property of ONE sample, so derive it once per
		// sample rather than once per pair: the multi-lag pairing below visits
		// most samples several times and would otherwise rebuild the same
		// rotation matrices on every visit.
		const size_t n = samples.size();
		std::vector<Eigen::Matrix3d> qA(n), qB(n);
		std::vector<Eigen::Vector3d> tgt(n);
		for (size_t i = 0; i < n; ++i)
		{
			qA[i] = samples[i].ref.rot.toRotationMatrix().transpose();
			qB[i] = (rotM * samples[i].target.rot.toRotationMatrix()).transpose();
			tgt[i] = rotM * samples[i].target.pos;
		}

		static const size_t kTransLags[] = { 1, 3, 8, 21, 55, 144 };
		for (size_t lag : kTransLags)
		{
			if (lag >= n)
				break;
			for (size_t i = 0; i + lag < n; ++i)
			{
				const size_t j = i + lag;
				const Eigen::Vector3d &refI = samples[i].ref.pos;
				const Eigen::Vector3d &refJ = samples[j].ref.pos;

				TransRow ra;
				ra.dQ = qA[j] - qA[i];
				ra.dQtdQ = ra.dQ.transpose() * ra.dQ;
				ra.base = qA[j] * refJ - qA[i] * refI;
				ra.scalePart = -(qA[j] * tgt[j] - qA[i] * tgt[i]);
				rows.push_back(ra);

				TransRow rb;
				rb.dQ = qB[j] - qB[i];
				rb.dQtdQ = rb.dQ.transpose() * rb.dQ;
				rb.base = qB[j] * refJ - qB[i] * refI;
				rb.scalePart = -(qB[j] * tgt[j] - qB[i] * tgt[i]);
				rows.push_back(rb);
			}
		}
	};
	buildRows(rot);

	// Conditioning of the system the translation is actually solved from: see
	// minTransEigRatio. Measured on the IRLS-WEIGHTED normal matrix, because a
	// row the robust pass drives toward zero weight does not constrain t - an
	// unweighted ratio can pass this gate on the strength of glitched rows
	// whose diverse dQ never survives the reweighting, which is exactly the
	// silent failure the gate exists to prevent. Called after every solve or
	// rebuild of the rows, so it always describes the transform that ships.
	auto weightedTransEigRatio = [&rows]() -> double
	{
		Eigen::Matrix3d ata = Eigen::Matrix3d::Zero();
		for (const auto &r : rows)
			ata += r.weight * r.dQtdQ;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(ata);
		return eig.eigenvalues()(0) / std::max(1e-12, eig.eigenvalues()(2));
	};

	auto solveTranslation = [&rows, &config](double scale, Eigen::Vector3d &tOut) -> double
	{
		// Each scale candidate is a separate robust objective evaluation. Do
		// not inherit IRLS weights from whichever candidate the golden-section
		// search happened to visit previously: that makes scores path-dependent
		// and therefore incomparable.
		for (auto &r : rows)
			r.weight = 1.0;

		for (int iter = 0; iter <= config.irlsIterations; ++iter)
		{
			Eigen::Matrix3d ata = Eigen::Matrix3d::Zero();
			Eigen::Vector3d atb = Eigen::Vector3d::Zero();
			for (const auto &r : rows)
			{
				Eigen::Vector3d rhs = r.base + scale * r.scalePart;
				ata += r.weight * r.dQtdQ;
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

		// Each iteration costs a full IRLS translation solve, so the loop bound
		// is derived from the scale precision that actually means something
		// downstream rather than picked: the driver applies scale as a plain
		// multiplier and the motion-gain guard compares it against a 3%
		// threshold, so resolving below this is spending solves on digits no
		// consumer can distinguish. Each iteration shrinks the interval by phi.
		constexpr double kScaleTolerance = 3e-6;
		int iterations = 0;
		if (hi - lo > kScaleTolerance)
			iterations = static_cast<int>(
				std::ceil(std::log(kScaleTolerance / (hi - lo)) / std::log(phi)));

		double x1 = hi - phi * (hi - lo), x2 = lo + phi * (hi - lo);
		Eigen::Vector3d tTmp;
		double f1 = solveTranslation(x1, tTmp), f2 = solveTranslation(x2, tTmp);
		for (int it = 0; it < iterations; ++it)
		{
			if (f1 < f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - phi * (hi - lo); f1 = solveTranslation(x1, tTmp); }
			else         { lo = x1; x1 = x2; f1 = f2; x2 = lo + phi * (hi - lo); f2 = solveTranslation(x2, tTmp); }
		}
		scale = (lo + hi) * 0.5;
		transRms = solveTranslation(scale, translation);
	}
	result.transEigRatio = weightedTransEigRatio();

	// ---- joint refinement (see EngineConfig::refineIterations) -------------
	// Pareto guard: the polish only optimizes position residuals, which carry
	// far less rotation information than the axis pairs (short mount lever),
	// so it is accepted only when it does not measurably worsen the axis-pair
	// fit — otherwise it would chase position noise away from the
	// orientation-optimal rotation. Rejected polishes leave the sequential
	// result untouched.
	if (config.refineIterations > 0)
	{
		Eigen::Matrix3d rotJ = rot;
		Eigen::Vector3d transJ = translation;
		double scaleJ = scale;
		if (JointRefine(samples, config, gravityRatio, rotJ, transJ, scaleJ) &&
		    axisRmsDeg(rotJ) <= axisRmsDeg(rot) * 1.02)
		{
			result.refinementApplied = true;
			rot = rotJ;
			translation = transJ;
			scale = scaleJ;

			// Rebuild the eq. 8 system at the refined rotation so the reported
			// residual (and the gates below) judge the transform that ships.
			buildRows(rot);
			for (auto &r : rows)
			{
				double residual = (r.dQ * translation - (r.base + scale * r.scalePart)).norm();
				r.weight = (residual <= config.huberTranslation) ? 1.0 : config.huberTranslation / residual;
			}
			double sq = 0.0, wsum = 0.0;
			for (const auto &r : rows)
			{
				double residual = (r.dQ * translation - (r.base + scale * r.scalePart)).norm();
				sq += r.weight * residual * residual;
				wsum += r.weight;
			}
			transRms = std::sqrt(sq / std::max(1e-12, wsum));
			result.transEigRatio = weightedTransEigRatio();
		}
	}

	// Rotation metrics must be computed here, after the joint refinement above
	// may have replaced `rot` - they describe the rotation that ships.
	result.rotationRmsDeg = axisRmsDeg(rot);
	result.rotation = Eigen::Quaterniond(rot);
	result.tiltDeg = std::acos(std::min(1.0, std::max(-1.0, (rot * kUp).dot(kUp)))) * 180.0 / EIGEN_PI;

	// The solved translation maps pre-rotated, pre-scaled target space; what the
	// driver applies is world-from-driver, which matches this frame directly.
	result.translation = translation;
	result.scale = scale;
	result.translationRmsMeters = transRms;

	// ---- validation --------------------------------------------------------
	if (!result.rotation.coeffs().allFinite() ||
	    !result.translation.allFinite() ||
	    !std::isfinite(result.scale) ||
	    !std::isfinite(result.rotationRmsDeg) ||
	    !std::isfinite(result.translationRmsMeters) ||
	    !std::isfinite(result.axisSpread) ||
	    !std::isfinite(result.transEigRatio))
	{
		result.message = "Calibration solve produced non-finite values.";
		return result;
	}
	if (result.axisSpread < config.minAxisSpread)
	{
		result.message = "Rotation happened around only one axis — tilt and roll are unconstrained. "
		                 "Rotate the devices together around two different axes and recalibrate.";
		return result;
	}
	if (result.transEigRatio < config.minTransEigRatio)
	{
		result.message = "Not enough varied rotation to pin the position along every direction — "
		                 "rotate the devices together around at least two different axes and recalibrate.";
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
	if (const char *configError = ConfigError(config))
	{
		failure.message = "Invalid calibration engine configuration: " +
			std::string(configError) + ".";
		return failure;
	}

	if (refStream.size() < 8 || targetStream.size() < 8)
	{
		failure.message = "Not enough samples collected.";
		return failure;
	}
	if (!IsValidStream(refStream) || !IsValidStream(targetStream))
	{
		failure.message = "Pose streams contain an invalid or out-of-range value, or non-increasing timestamps.";
		return failure;
	}

	// ---- inter-system time alignment --------------------------------------
	double offset = 0.0;
	bool offsetKnown = false;
	if (config.estimateTimeOffset)
		offsetKnown = EstimateTimeOffset(refStream, targetStream, config, offset, false);

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

	// Streams were validated above and alignment preserves order/finiteness.
	EngineResult result = SolveAligned(aligned, config, false);

	// ---- motion-amplitude gain diagnostic + scale guard --------------------
	// (see EngineConfig::gainSplitSeconds). When the fine band's gain sits
	// below the gross band's, the reference stream is low-passing motion and
	// the least-squares scale is dragged toward the fine-band gain; the
	// gross-band gain is usable only when it remains near unity. If gross is
	// attenuated too, neither measured band identifies physical metric scale,
	// so re-solve at neutral scale rather than applying contaminated motion.
	double gainLow = 0.0, gainHigh = 0.0;
	bool gainValid = result.valid &&
		EstimateMotionGain(refStream, targetStream, offset, config, result, gainLow, gainHigh);
	bool smoothingDetected = gainValid &&
		gainHigh < gainLow - config.gainSmoothingMargin;

	// A guard that runs only when its own detector succeeded cannot honour "a
	// contaminated free-scale fit is never applied". EstimateMotionGain
	// abstains on six conditions that have nothing to do with whether
	// smoothing is present, and two of them fire precisely in the
	// low-translation regime the calibration instructions produce (vary
	// rotation about two axes, which a user can do largely in place). An
	// abstention means the scale is not identifiable from this motion, not
	// that it is clean - so take the same guarded path at neutral scale
	// instead of committing and persisting the free fit with no log line.
	bool scaleNotIdentifiable = !gainValid;

	if (result.valid && config.solveScale && config.pinScaleOnSmoothing &&
	    (smoothingDetected || scaleNotIdentifiable))
	{
		// Without a valid diagnostic there is no gross band to trust.
		bool grossClean = gainValid &&
			std::abs(gainLow - 1.0) <= config.maxCleanGrossDeviation;
		double guardedScale = grossClean
			? std::min(1.0 + config.scaleSearchRange,
				std::max(1.0 - config.scaleSearchRange, gainLow))
			: 1.0;

		// Pre-scaling the target and solving with scale fixed composes exactly
		// with the driver's scale-then-transform application (same trick as
		// the anchor path).
		EngineConfig pinnedConfig = config;
		pinnedConfig.solveScale = false;
		std::vector<AlignedSample> scaled = aligned;
		for (auto &a : scaled)
		{
			a.target.pos *= guardedScale;
			a.target.vel *= guardedScale;
		}
		EngineResult r2 = SolveAligned(scaled, pinnedConfig, false);
		if (r2.valid)
		{
			r2.scale = guardedScale;
			r2.scaleGuard = grossClean
				? ScaleGuard::FromGrossMotion
				: ScaleGuard::NeutralizedForSmoothing;
			r2.message += scaleNotIdentifiable
				? " Motion did not identify playspace scale (too little translation); scale held at neutral 1.0."
				: grossClean
					? " Fine-motion attenuation detected (streamed-pose smoothing); scale taken from clean gross motion."
					: " Gross and fine motion are attenuated (streamed-pose smoothing); scale held at neutral 1.0.";
			result = r2;
		}
		else
		{
			// Never silently fall back to the contaminated free-scale fit.
			result.valid = false;
			result.message = std::string(scaleNotIdentifiable
					? "Playspace scale could not be identified from this motion"
					: "Streamed-pose smoothing was detected") +
				", but the guarded fixed-scale re-solve failed: " + r2.message;
		}
	}

	result.motionGainValid = gainValid;
	result.motionGainLow = gainLow;
	result.motionGainHigh = gainHigh;
	result.motionSmoothingDetected = smoothingDetected;
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
