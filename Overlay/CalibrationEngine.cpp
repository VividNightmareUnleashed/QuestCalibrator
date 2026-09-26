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
	if (!finiteNonnegative(config.minScaleCondition) || config.minScaleCondition > 1.0)
		return "minScaleCondition";
	if (!finiteNonnegative(config.maxScaleStdDev))
		return "maxScaleStdDev";

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

// Angular speed at every sample of a stream, derived once.
//
// The driver's reported angular velocity is used where it exists. Where it does
// not, the speed comes from a forward finite difference — which is a property of
// the INTERVAL [i, i+1], not of sample i — so the last sample has no interval of
// its own and holds the previous speed instead of reading zero.
//
// That tail rule is a deliberate decision, not a detail. Zero at the end makes
// the profile ramp linearly to zero across the stream's final inter-sample
// interval, and EstimateTimeOffset slides a window over this profile once per
// candidate lag: a fake decay sitting at a fixed absolute time lands against
// different target content at every lag, which is a lag-dependent artifact
// inside the one function whose entire job is to compare lags. Holding the last
// measured speed states what the data actually supports (the device did not
// stop; the stream did) and leaves nothing that moves with the lag.
std::vector<double> BuildSpeedProfile(const std::vector<PoseSample> &stream)
{
	std::vector<double> speed(stream.size(), 0.0);
	for (size_t i = 0; i < stream.size(); ++i)
	{
		double reported = stream[i].angVel.norm();
		if (reported > 1e-6)
		{
			speed[i] = reported;
			continue;
		}
		if (i + 1 >= stream.size())
		{
			speed[i] = (i > 0) ? speed[i - 1] : 0.0;   // hold; see above
			continue;
		}
		double dt = stream[i + 1].time - stream[i].time;
		if (dt <= 1e-6)
			continue;
		Eigen::Quaterniond dq = stream[i + 1].rot * stream[i].rot.conjugate();
		dq.normalize();
		if (dq.w() < 0.0)
			dq.coeffs() = -dq.coeffs();
		speed[i] = 2.0 * std::atan2(dq.vec().norm(), dq.w()) / dt;
	}
	return speed;
}

struct ResampledSpeed
{
	std::vector<double> values;
	std::vector<uint8_t> valid;
};

// Piecewise-linear resample of a per-sample speed profile onto a uniform grid.
// `speed` is the stream's own BuildSpeedProfile output, so the two are indexed
// together. It is passed in rather than derived here because it is a property of
// the stream alone: the correlator below resamples one shared grid that every
// candidate lag slices, instead of rebuilding the profile per lag.
ResampledSpeed ResampleSpeed(const std::vector<PoseSample> &stream,
	const std::vector<double> &speed, double t0, double dt, size_t count,
	double maxGap)
{
	ResampledSpeed out;
	out.values.resize(count, 0.0);
	out.valid.resize(count, 0);
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
		if (std::abs(t - ta) <= 1e-9)
		{
			out.values[i] = speed[j];
			out.valid[i] = 1;
			continue;
		}
		if (std::abs(t - tb) <= 1e-9)
		{
			out.values[i] = speed[j + 1];
			out.valid[i] = 1;
			continue;
		}
		if (tb - ta > maxGap)
			continue;
		double f = (tb - ta > 1e-9) ? (t - ta) / (tb - ta) : 0.0;
		f = std::min(1.0, std::max(0.0, f));
		out.values[i] = speed[j] + f * (speed[j + 1] - speed[j]);
		out.valid[i] = 1;
	}
	return out;
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
	if (std::abs(t - a.time) <= 1e-9)
	{
		out = a;
		out.time = t;
		return true;
	}
	if (std::abs(t - b.time) <= 1e-9)
	{
		out = b;
		out.time = t;
		return true;
	}
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
                                            double *scoreOut,
                                            double *peakMarginOut,
                                            bool validateInputs)
{
	offsetOut = 0.0;
	if (scoreOut) *scoreOut = 0.0;
	if (peakMarginOut) *peakMarginOut = 0.0;
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

	// The reference profile is resampled ONCE below, onto a grid every lag's
	// window is a whole-slot slice of. That requires the lag step to be an exact
	// integer multiple of the grid spacing, so the spacing is DERIVED from the
	// step instead of being clamped independently of it: both divisions here are
	// exact in IEEE arithmetic (a power-of-two divisor, or the identity), which
	// makes the precondition structural rather than an assumption a config could
	// quietly break. At or above a 2 ms step this is the previous
	// max(1e-3, step * 0.5) exactly, so the shipped default (2 ms step, 1 ms
	// grid) is unchanged; below it the grid follows the step itself rather than
	// the old 1 ms clamp, and the work budgets below still bound the result.
	const int subdiv = (config.timeOffsetStep >= 2e-3) ? 2 : 1;
	const double dt = config.timeOffsetStep / subdiv;
	double resampledCount = (end - start) / dt;
	if (!std::isfinite(resampledCount) || resampledCount < 64.0 ||
		resampledCount > MaxResampledPointCount)
		return false;
	double searchSteps = config.timeOffsetRange / config.timeOffsetStep;
	if ((2.0 * searchSteps + 1.0) * resampledCount > MaxCorrelationWork)
		return false;
	size_t count = static_cast<size_t>(resampledCount);

	int steps = static_cast<int>(config.timeOffsetRange / config.timeOffsetStep);
	// The shared grid spans the union of every lag's window: `steps` whole steps
	// of lead-in and the same of run-out. It is what actually gets allocated, so
	// it carries the same cap the per-lag window count does.
	double gridPointCount = static_cast<double>(count) +
		2.0 * static_cast<double>(steps) * static_cast<double>(subdiv);
	if (gridPointCount > MaxResampledPointCount)
		return false;
	size_t gridCount = static_cast<size_t>(gridPointCount);

	ResampledSpeed targetSpeed = ResampleSpeed(targetStream,
		BuildSpeedProfile(targetStream), start, dt, count,
		config.maxInterpolationGap);

	// The reference profile over that shared grid. Lag k's window starts at
	// start - k*step == gridStart + (steps - k)*step, and step == subdiv*dt
	// exactly, so the window is the slice beginning at whole slot
	// (steps - k)*subdiv — the lag-dependent part of this is an array index.
	//
	// The grid also stays inside both streams: gridStart >= max(front times)
	// because steps*step <= timeOffsetRange, and the last grid point sits at
	// start + steps*step + dt*(count - 1) <= min(back times) - dt. So every
	// point interpolates between two real samples, the past-the-end zeros
	// ResampleSpeed would otherwise emit are unreachable, and the only stream
	// boundary the grid can touch is the final inter-sample interval — which is
	// exactly where BuildSpeedProfile's hold rule applies.
	const double gridStart = start - static_cast<double>(steps) * config.timeOffsetStep;
	const ResampledSpeed refGrid = ResampleSpeed(refStream,
		BuildSpeedProfile(refStream), gridStart, dt, gridCount,
		config.maxInterpolationGap);

	// Physical event at time T shows up in the reference stream at T and in the
	// target stream at T + offset; so targetSpeed(t) matches refSpeed(t - offset).
	double bestOffset = 0.0, bestScore = -2.0;
	std::vector<double> scores;
	constexpr size_t minSupport = 64;
	for (int k = -steps; k <= steps; ++k)
	{
		double lag = static_cast<double>(k) * config.timeOffsetStep;
		const size_t base = static_cast<size_t>(steps - k) * static_cast<size_t>(subdiv);
		double refSum = 0.0, targetSum = 0.0;
		size_t support = 0;
		for (size_t i = 0; i < count; ++i)
		{
			if (!targetSpeed.valid[i] || !refGrid.valid[base + i])
				continue;
			refSum += refGrid.values[base + i];
			targetSum += targetSpeed.values[i];
			++support;
		}
		if (support < minSupport)
		{
			scores.push_back(-2.0);
			continue;
		}
		double refMean = refSum / static_cast<double>(support);
		double targetMean = targetSum / static_cast<double>(support);
		double dot = 0.0, refNorm = 0.0;
		double targetNorm = 0.0;
		for (size_t i = 0; i < count; ++i)
		{
			if (!targetSpeed.valid[i] || !refGrid.valid[base + i])
				continue;
			double r = refGrid.values[base + i] - refMean;
			double target = targetSpeed.values[i] - targetMean;
			dot += r * target;
			refNorm += r * r;
			targetNorm += target * target;
		}
		double score = (refNorm > 1e-8 && targetNorm > 1e-8)
			? dot / std::sqrt(refNorm * targetNorm) : -2.0;
		scores.push_back(score);
		if (score > bestScore)
		{
			bestScore = score;
			bestOffset = lag;
		}
	}

	if (bestScore < 0.25)
		return false;   // no meaningful correlation peak

	int bestGridIndex = static_cast<int>(std::distance(scores.begin(),
		std::max_element(scores.begin(), scores.end())));
	const int peakExclusion = std::max(1,
		static_cast<int>(std::ceil(0.01 / config.timeOffsetStep)));
	double secondScore = -2.0;
	for (int i = 0; i < static_cast<int>(scores.size()); ++i)
	{
		if (std::abs(i - bestGridIndex) <= peakExclusion)
			continue;
		secondScore = std::max(secondScore, scores[i]);
	}
	double peakMargin = secondScore > -1.0 ? bestScore - secondScore : bestScore;
	if (peakMargin <= 1e-6)
		return false;

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
	if (scoreOut) *scoreOut = bestScore;
	if (peakMarginOut) *peakMarginOut = peakMargin;
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
		result.failure = EngineFailure::Config;
		result.message = "Invalid calibration engine configuration: " +
			std::string(configError) + ".";
		return result;
	}

	if (samples.size() < 8)
	{
		result.failure = EngineFailure::NotEnoughSamples;
		result.message = "Not enough samples collected.";
		return result;
	}
	if (validateInputs && !IsValidAlignedSamples(samples))
	{
		result.failure = EngineFailure::InvalidSamples;
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
		result.failure = EngineFailure::NotEnoughRotation;
		result.message = "Not enough rotation -- rotate the devices together, at least a quarter turn at a time.";
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

	// The Huber weighting rule and the weighted score, each written once. Both
	// the IRLS loop below and the joint-refinement block need them, and
	// transRms is what the maxTranslationRms gate judges - so with the rule
	// copied, a change to the knee, the weight form or the 1e-12 floor applied
	// to one leaves the other (the path that ships whenever refineIterations >
	// 0, the default) gated on a residual computed under the old rule, both
	// numbers finite and plausible.
	//
	// They stay SEPARATE on purpose. solveTranslation scores with the weights
	// its last reweight produced - its loop solves and then breaks without
	// reweighting - while the refinement block reweights at the refined
	// transform first. Fusing them into one reweight-and-score would quietly
	// change the solver's own scores.
	auto reweightRows = [&rows, &config](const Eigen::Vector3d &t, double scale)
	{
		for (auto &r : rows)
		{
			double residual = (r.dQ * t - (r.base + scale * r.scalePart)).norm();
			r.weight = (residual <= config.huberTranslation) ? 1.0 : config.huberTranslation / residual;
		}
	};
	auto weightedRms = [&rows](const Eigen::Vector3d &t, double scale) -> double
	{
		double sq = 0.0, wsum = 0.0;
		for (const auto &r : rows)
		{
			double residual = (r.dQ * t - (r.base + scale * r.scalePart)).norm();
			sq += r.weight * residual * residual;
			wsum += r.weight;
		}
		return std::sqrt(sq / std::max(1e-12, wsum));
	};

	auto solveTranslation = [&rows, &config, &reweightRows, &weightedRms](
		double scale, Eigen::Vector3d &tOut) -> double
	{
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
			reweightRows(tOut, scale);
		}

		return weightedRms(tOut, scale);
	};

	double scale = 1.0;
	Eigen::Vector3d translation;
	double transRms = solveTranslation(1.0, translation);

	if (config.solveScale)
	{
		// Translation and scale are one linear system. Solving them jointly avoids
		// a nested 1-D search whose objective inherited a different IRLS history at
		// each candidate and exposes the conditional scale information directly.
		// Seed the joint system with the robust fixed-scale translation solve
		// above. Starting scale in an unweighted errors-in-variables system lets a
		// handful of corrupted target positions drag both the scale column and RHS
		// to a search bound before IRLS has identified them.
		auto reweightJointRows = [&rows, &config](
			const Eigen::Vector3d &t, double candidateScale)
		{
			for (auto &r : rows)
			{
				double residual = (r.dQ * t -
					(r.base + candidateScale * r.scalePart)).norm();
				double ratio = residual <= config.huberTranslation
					? 1.0 : config.huberTranslation / residual;
				// Scale is an errors-in-variables column: a corrupted target
				// position increases both its residual and its leverage. Squaring the
				// Huber ratio bounds that leverage instead of letting one bad point
				// pull the metric scale through every pair containing it.
				r.weight = ratio * ratio;
			}
		};
		reweightJointRows(translation, scale);
		for (int iter = 0; iter <= 2 * config.irlsIterations; ++iter)
		{
			Eigen::Matrix4d ata = Eigen::Matrix4d::Zero();
			Eigen::Vector4d atb = Eigen::Vector4d::Zero();
			for (const auto &r : rows)
			{
				Eigen::Matrix<double, 3, 4> a;
				a.leftCols<3>() = r.dQ;
				a.col(3) = -r.scalePart;
				ata += r.weight * a.transpose() * a;
				atb += r.weight * a.transpose() * r.base;
			}
			Eigen::Vector4d solved = ata.ldlt().solve(atb);
			translation = solved.head<3>();
			scale = std::min(1.0 + config.scaleSearchRange,
				std::max(1.0 - config.scaleSearchRange, solved(3)));
			if (scale != solved(3))
			{
				transRms = solveTranslation(scale, translation);
				break;
			}
			if (iter == 2 * config.irlsIterations)
			{
				transRms = weightedRms(translation, scale);
				break;
			}
			reweightJointRows(translation, scale);
		}
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
			reweightRows(translation, scale);
			transRms = weightedRms(translation, scale);
			result.transEigRatio = weightedTransEigRatio();
		}
	}

	if (config.solveScale)
	{
		Eigen::Matrix3d translationInformation = Eigen::Matrix3d::Zero();
		Eigen::Vector3d coupling = Eigen::Vector3d::Zero();
		double scaleInformation = 0.0;
		double weightedSquaredError = 0.0;
		double weightSum = 0.0;
		for (const auto &r : rows)
		{
			translationInformation += r.weight * r.dQtdQ;
			coupling -= r.weight * r.dQ.transpose() * r.scalePart;
			scaleInformation += r.weight * r.scalePart.squaredNorm();
			double residual =
				(r.dQ * translation - (r.base + scale * r.scalePart)).norm();
			weightedSquaredError += r.weight * residual * residual;
			weightSum += r.weight;
		}
		double conditionalInformation = scaleInformation - coupling.dot(
			translationInformation.ldlt().solve(coupling));
		conditionalInformation = std::max(0.0, conditionalInformation);
		result.scaleCondition = conditionalInformation /
			std::max(1e-12, scaleInformation);
		double variance = weightedSquaredError /
			std::max(1.0, 3.0 * weightSum - 4.0);
		result.scaleStdDev = std::sqrt(variance /
			std::max(1e-12, conditionalInformation));
		result.scaleIdentifiable = std::isfinite(result.scaleCondition) &&
			std::isfinite(result.scaleStdDev) &&
			result.scaleCondition >= config.minScaleCondition &&
			result.scaleStdDev <= config.maxScaleStdDev;
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
		result.failure = EngineFailure::NonFinite;
		result.message = "Calibration solve produced non-finite values.";
		return result;
	}
	if (result.translation.cwiseAbs().maxCoeff() >
		protocol::limits::MaxAbsTranslationMeters ||
		result.scale < protocol::limits::MinScale ||
		result.scale > protocol::limits::MaxScale)
	{
		result.failure = EngineFailure::OutOfRange;
		result.message = "Calibration solve produced a transform outside the supported range.";
		return result;
	}
	if (result.axisSpread < config.minAxisSpread)
	{
		result.failure = EngineFailure::SingleAxis;
		result.message = "Rotation happened around only one axis -- tilt and roll are unconstrained. "
		                 "Rotate the devices together around two different axes and recalibrate.";
		return result;
	}
	if (result.transEigRatio < config.minTransEigRatio)
	{
		result.failure = EngineFailure::TranslationUnobservable;
		result.message = "Not enough varied rotation to pin the position along every direction -- "
		                 "rotate the devices together around at least two different axes and recalibrate.";
		return result;
	}
	if (result.rotationRmsDeg > config.maxRotationRms)
	{
		result.failure = EngineFailure::RotationResidual;
		result.message = "Rotation residual too high (" + std::to_string(result.rotationRmsDeg).substr(0, 4) +
		                 " deg) -- tracking is jittery or the devices are not rigidly attached.";
		return result;
	}
	if (result.translationRmsMeters > config.maxTranslationRms)
	{
		result.failure = EngineFailure::PositionResidual;
		result.message = "Position residual too high (" +
		                 std::to_string(result.translationRmsMeters * 100.0).substr(0, 4) +
		                 " cm) -- tracking is jittery, or motion was too fast for the sample rate.";
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
		failure.failure = EngineFailure::Config;
		failure.message = "Invalid calibration engine configuration: " +
			std::string(configError) + ".";
		return failure;
	}

	if (refStream.size() < 8 || targetStream.size() < 8)
	{
		failure.failure = EngineFailure::NotEnoughSamples;
		failure.message = "Not enough samples collected.";
		return failure;
	}
	if (!IsValidStream(refStream) || !IsValidStream(targetStream))
	{
		failure.failure = EngineFailure::InvalidSamples;
		failure.message = "Pose streams contain an invalid or out-of-range value, or non-increasing timestamps.";
		return failure;
	}

	// ---- inter-system time alignment --------------------------------------
	double offset = 0.0;
	double offsetScore = 0.0;
	double offsetPeakMargin = 0.0;
	bool offsetKnown = false;
	if (config.estimateTimeOffset)
	{
		offsetKnown = EstimateTimeOffset(refStream, targetStream, config, offset,
			&offsetScore, &offsetPeakMargin, false);
		if (!offsetKnown)
		{
			failure.failure = EngineFailure::TimeOffset;
			failure.message = "Time offset could not be measured reliably. Keep both devices visible and rotate them together with varied motion.";
			return failure;
		}
	}

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
	bool fineAttenuated = gainValid &&
		gainHigh < gainLow - config.gainSmoothingMargin;
	bool bandMismatch = gainValid &&
		std::abs(gainHigh - gainLow) > config.gainSmoothingMargin;

	// A guard that runs only when its own detector succeeded cannot honour "a
	// contaminated free-scale fit is never applied". EstimateMotionGain
	// abstains on six conditions that have nothing to do with whether
	// smoothing is present, and two of them fire precisely in the
	// low-translation regime the calibration instructions produce (vary
	// rotation about two axes, which a user can do largely in place). An
	// abstention means the scale is not identifiable from this motion, not
	// that it is clean - so take the same guarded path at neutral scale
	// instead of committing and persisting the free fit with no log line.
	bool scaleNotIdentifiable = !gainValid || !result.scaleIdentifiable;

	if (result.valid && config.solveScale && config.pinScaleOnSmoothing &&
	    (bandMismatch || scaleNotIdentifiable))
	{
		// Without a valid diagnostic there is no gross band to trust.
		bool grossClean = fineAttenuated &&
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
			r2.scaleCondition = result.scaleCondition;
			r2.scaleStdDev = result.scaleStdDev;
			r2.scaleIdentifiable = result.scaleIdentifiable;
			r2.scale = guardedScale;
			r2.scaleGuard = grossClean
				? ScaleGuard::FromGrossMotion
				: ScaleGuard::NeutralizedForSmoothing;
			r2.message += scaleNotIdentifiable
				? " Motion did not identify playspace scale with enough confidence; scale held at neutral 1.0."
				: grossClean
					? " Fine-motion attenuation detected (streamed-pose smoothing); scale taken from clean gross motion."
					: " Frequency-dependent motion gain is physically inconsistent; scale held at neutral 1.0.";
			result = r2;
		}
		else
		{
			// Never silently fall back to the contaminated free-scale fit.
			result.valid = false;
			result.failure = r2.failure != EngineFailure::None
				? r2.failure : EngineFailure::ScaleNotIdentifiable;
			result.message = std::string(scaleNotIdentifiable
					? "Playspace scale could not be identified from this motion"
					: "Frequency-dependent motion gain was detected") +
				", but the guarded fixed-scale re-solve failed: " + r2.message;
		}
	}
	else if (result.valid && config.solveScale && !result.scaleIdentifiable)
	{
		result.valid = false;
		result.failure = EngineFailure::ScaleNotIdentifiable;
		result.message = "Playspace scale is not identifiable from this motion; move both devices across a larger volume and recalibrate.";
	}

	result.motionGainValid = gainValid;
	result.motionGainLow = gainLow;
	result.motionGainHigh = gainHigh;
	result.motionGainInconsistent = bandMismatch;
	result.motionSmoothingDetected = fineAttenuated;
	result.timeOffset = offset;
	result.timeOffsetValid = offsetKnown;
	result.timeOffsetScore = offsetScore;
	result.timeOffsetPeakMargin = offsetPeakMargin;
	result.samplesGated = gated;
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
