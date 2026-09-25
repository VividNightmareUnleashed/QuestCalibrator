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
// Caps the correlation's resampling grid: stream timestamps come from the
// driver, so a malformed stream can span any time range.
constexpr double MaxResampledPointCount = 1000000.0;

inline double HuberWeight(double residual, double knee)
{
	return residual <= knee ? 1.0 : knee / residual;
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
	if (angle < minAngle)
		return false;

	axisOut = dq.vec() / sinHalf;   // sinHalf >= sin(minAngle / 2) > 0
	angleOut = angle;
	return true;
}

struct AxisPair
{
	Eigen::Vector3d ref;
	Eigen::Vector3d target;
	double weight = 1.0;
	// Inverse-variance base weight: a delta rotation's axis carries noise
	// ~ sigma/theta (Park & Martin, applied to the axis-pair form). The Huber
	// IRLS factor multiplies on top of this.
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

	Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(cross);
	Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
	d(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
	return svd.matrixV() * d * svd.matrixU().transpose();
}

struct SpeedSample
{
	double time = 0.0;
	double value = 0.0;
	bool valid = false;
};

// OpenVR has no angular-velocity availability flag. Preserve reported nonzero
// speeds; otherwise derive an interval average and timestamp it at its midpoint
// (the left endpoint would bias lag by half a sample period).
//
// A gyro reading never repeats bit for bit, so a repeat is a transport holding
// the previous value (Virtual Desktop re-predicts about 8 % of headset frames).
// A held value is the profile one frame late and moved the measured offset by
// about the held share of a frame (-4.5 ms at 30 %, simulated), so a stream
// with more than one repeat in twenty has its speed derived from its rotations.
std::vector<SpeedSample> BuildSpeedProfile(const std::vector<PoseSample> &stream, double maxGap)
{
	size_t reportedCount = 0, repeatedCount = 0;
	for (size_t i = 1; i < stream.size(); ++i)
	{
		if (stream[i].angVel.norm() <= 1e-6)
			continue;
		++reportedCount;
		if (stream[i].angVel == stream[i - 1].angVel)
			++repeatedCount;
	}
	const bool heldReports = repeatedCount * 20 > reportedCount;

	std::vector<SpeedSample> speed(stream.size());
	for (size_t i = 0; i < stream.size(); ++i)
	{
		auto &sample = speed[i];
		sample.time = stream[i].time;
		double reported = stream[i].angVel.norm();
		if (!heldReports && reported > 1e-6)
		{
			sample.value = reported;
			sample.valid = true;
			continue;
		}
		if (i + 1 >= stream.size())
		{
			// No final interval: hold the last measured speed, not a fake zero.
			if (i > 0)
			{
				sample.value = speed[i - 1].value;
				sample.valid = speed[i - 1].valid;
			}
			continue;
		}
		double dt = stream[i + 1].time - stream[i].time;
		if (dt <= 1e-6 || dt > maxGap)
			continue;
		Eigen::Quaterniond dq = stream[i + 1].rot * stream[i].rot.conjugate();
		dq.normalize();
		if (dq.w() < 0.0)
			dq.coeffs() = -dq.coeffs();
		sample.time += 0.5 * dt;
		sample.value = 2.0 * std::atan2(dq.vec().norm(), dq.w()) / dt;
		sample.valid = true;
	}
	return speed;
}

struct ResampledSpeed
{
	std::vector<double> values;
	std::vector<uint8_t> valid;
};

// Resample once per stream; every candidate lag slices the same uniform grid.
ResampledSpeed ResampleSpeed(const std::vector<SpeedSample> &speed,
	double t0, double dt, size_t count, double maxGap)
{
	ResampledSpeed out;
	out.values.resize(count, 0.0);
	out.valid.resize(count, 0);

	size_t j = 0;
	for (size_t i = 0; i < count; ++i)
	{
		double t = t0 + dt * static_cast<double>(i);
		while (j + 1 < speed.size() && speed[j + 1].time < t)
			++j;
		if (j + 1 >= speed.size())
			break;
		const auto &a = speed[j];
		const auto &b = speed[j + 1];
		if (std::abs(t - a.time) <= 1e-9)
		{
			out.values[i] = a.value;
			out.valid[i] = a.valid;
			continue;
		}
		if (std::abs(t - b.time) <= 1e-9)
		{
			out.values[i] = b.value;
			out.valid[i] = b.valid;
			continue;
		}
		if (!a.valid || !b.valid || t < a.time || b.time - a.time > maxGap)
			continue;
		double f = (t - a.time) / (b.time - a.time);
		out.values[i] = a.value + f * (b.value - a.value);
		out.valid[i] = 1;
	}
	return out;
}

// Frequency-split amplitude gain of the reference stream's position track
// relative to the target's, over a time-aligned uniform resampling (see
// EngineConfig::gainSplitSeconds). The comparison track is the solved model's
// prediction of the reference device's position (R(s p_B) + T - Q_A d, with
// the mount arm d estimated from the data): the raw target track would read
// the rotation-driven mount-lever motion as a phantom frequency tilt. Gains
// are the solved scale times the measured ratio, so under a genuine metric
// difference both bands sit at the true scale.
//
// With the time offset estimated (always, in production), its resampling cap
// at a 1 ms grid over the same span also bounds this 20 ms one.
bool EstimateMotionGain(const std::vector<PoseSample> &refStream,
                        const std::vector<PoseSample> &targetStream,
                        double offset, const EngineConfig &config,
                        const EngineResult &solved,
                        double &lowOut, double &highOut)
{
	const double dt = 0.02;
	double start = std::max(refStream.front().time + offset, targetStream.front().time);
	double end = std::min(refStream.back().time + offset, targetStream.back().time);
	if (end - start < 5.0)
		return false;

	std::vector<PoseSample> refAt, tgtAt;
	refAt.reserve(static_cast<size_t>((end - start) / dt + 1.0));
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
	// frame; rough is fine (a contaminated scale moves it by millimeters).
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
		lowRms = std::sqrt(sqLow / static_cast<double>(m));   // m = n - 2W >= 2W
		highRms = std::sqrt(sqHigh / static_cast<double>(m));
	};

	double rLow = 0.0, rHigh = 0.0, tLow = 0.0, tHigh = 0.0;
	bandRms(rp, rLow, rHigh);
	bandRms(tp, tLow, tHigh);

	// A band with too little motion cannot support a ratio (the slow-cautious
	// or occluded cases); refuse rather than divide noise by noise.
	if (tLow < 0.03 || tHigh < 0.008)
		return false;

	// The predicted track already carries the solved scale; fold it back in.
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
// plus the Kabsch stage's gravity prior as a virtual residual. Positions alone
// carry far less rotation information than the orientations (short mount
// lever), so a position-only polish would chase position noise. kRot converts
// radians to meters so one Huber knee (huberTranslation) governs both sets.
// Writes back and returns true only when the combined robust cost improved.
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
			double resR = kRot * LogVec(a.ref.rot.conjugate() * (Rq * a.target.rot * Cc)).norm();
			sq += HuberWeight(resP, config.huberTranslation) * resP * resP +
				HuberWeight(resR, config.huberTranslation) * resR * resR;
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
			double w = HuberWeight(e.norm(), config.huberTranslation);
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
	if (!std::isfinite(after) || after >= before)
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
	if (stream.size() < 2 || !std::isfinite(t) ||
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

	// a.time < t <= b.time, neither within 1e-9 of t, so f lies in (0, 1).
	double f = (t - a.time) / gap;

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
                                            double *peakMarginOut)
{
	offsetOut = 0.0;
	if (scoreOut) *scoreOut = 0.0;
	if (peakMarginOut) *peakMarginOut = 0.0;
	if (refStream.size() < 8 || targetStream.size() < 8)
		return false;

	double start = std::max(refStream.front().time, targetStream.front().time) + config.timeOffsetRange;
	double end = std::min(refStream.back().time, targetStream.back().time) - config.timeOffsetRange;
	if (end - start < 0.5)
		return false;   // not enough overlap to correlate

	// The reference profile is resampled once, onto a grid every lag's window
	// is a whole-slot slice of, so the grid spacing is derived from the lag step
	// by an exact IEEE division (a power-of-two divisor, or the identity).
	const int subdiv = (config.timeOffsetStep >= 2e-3) ? 2 : 1;
	const double dt = config.timeOffsetStep / subdiv;
	size_t count = static_cast<size_t>((end - start) / dt);

	int steps = static_cast<int>(config.timeOffsetRange / config.timeOffsetStep);
	// The shared grid spans the union of every lag's window: `steps` whole steps
	// of lead-in and the same of run-out.
	double gridPointCount = static_cast<double>(count) +
		2.0 * static_cast<double>(steps) * static_cast<double>(subdiv);
	if (gridPointCount > MaxResampledPointCount)
		return false;
	size_t gridCount = static_cast<size_t>(gridPointCount);

	ResampledSpeed targetSpeed = ResampleSpeed(
		BuildSpeedProfile(targetStream, config.maxInterpolationGap), start, dt, count,
		config.maxInterpolationGap);

	// The reference profile over that shared grid. Lag k's window starts at
	// start - k*step == gridStart + (steps - k)*step, and step == subdiv*dt
	// exactly, so it is the slice beginning at whole slot (steps - k)*subdiv.
	//
	// The grid stays inside both streams: gridStart >= max(front times) because
	// steps*step <= timeOffsetRange, and the last point sits at
	// start + steps*step + dt*(count - 1) <= min(back times) - dt. Derived speeds
	// begin half an interval later; ResampleSpeed marks that leading edge
	// invalid and holds the final speed through the last pose timestamp.
	const double gridStart = start - static_cast<double>(steps) * config.timeOffsetStep;
	const ResampledSpeed refGrid = ResampleSpeed(
		BuildSpeedProfile(refStream, config.maxInterpolationGap), gridStart, dt, gridCount,
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
	if (bestGridIndex > 0 && bestGridIndex + 1 < static_cast<int>(scores.size()))
	{
		double y0 = scores[bestGridIndex - 1], y1 = scores[bestGridIndex], y2 = scores[bestGridIndex + 1];
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
                                             const EngineConfig &config)
{
	EngineResult result;
	result.samplesUsed = samples.size();
	if (samples.size() < 8)
	{
		result.failure = EngineFailure::NotEnoughSamples;
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

			// Ambiguous shortest-arc hemisphere (see EngineConfig::maxPairAngle).
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
	// Each base is at least minPairAngle^2, so the sum is positive.
	{
		double sum = 0.0;
		for (const auto &p : pairs)
			sum += p.base;
		double inv = static_cast<double>(pairs.size()) / sum;
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
	// Eigenvalues ascend; the largest is at least trace/3 = pairs/3.
	result.axisSpread = axisEig.eigenvalues()(1) / axisEig.eigenvalues()(2);

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
			p.weight = p.base * HuberWeight(residual, config.huberRotation);
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
		return std::sqrt(rotResidualSq / rotWeight) * 180.0 / EIGEN_PI;
	};

	// ---- translation (+ optional scale): weighted linear least squares -----
	// math.pdf eq. 8 over sample pairs, with the target universe pre-rotated by
	// the solved rotation. Rows are linear in the scale, so the joint (t, s)
	// solve reuses them.
	struct TransRow
	{
		Eigen::Matrix3d dQ;
		Eigen::Matrix3d dQtdQ;     // dQ^T dQ; fixed for the row's lifetime
		Eigen::Vector3d base;     // constant part of the RHS
		Eigen::Vector3d scalePart; // part multiplied by scale
		double weight = 1.0;
		size_t first = 0, second = 0;   // the pair's sample indices, for the block jackknife
	};
	std::vector<TransRow> rows;

	auto buildRows = [&samples, &rows](const Eigen::Matrix3d &rotM)
	{
		rows.clear();
		// Per-sample terms, derived once: the multi-lag pairing visits most
		// samples several times.
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
				ra.first = i;
				ra.second = j;
				rows.push_back(ra);

				TransRow rb;
				rb.dQ = qB[j] - qB[i];
				rb.dQtdQ = rb.dQ.transpose() * rb.dQ;
				rb.base = qB[j] * refJ - qB[i] * refI;
				rb.scalePart = -(qB[j] * tgt[j] - qB[i] * tgt[i]);
				rb.first = i;
				rb.second = j;
				rows.push_back(rb);
			}
		}
	};
	buildRows(rot);

	// Conditioning of the system the translation is solved from (see
	// minTransEigRatio), on the IRLS-weighted normal matrix: a row the robust
	// pass drives toward zero weight does not constrain t, so glitched rows with
	// diverse dQ must not pass the gate. Re-measured after every solve or
	// rebuild of the rows.
	auto weightedTransEigRatio = [&rows]() -> double
	{
		Eigen::Matrix3d ata = Eigen::Matrix3d::Zero();
		for (const auto &r : rows)
			ata += r.weight * r.dQtdQ;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(ata);
		return eig.eigenvalues()(0) / std::max(1e-12, eig.eigenvalues()(2));
	};

	// Shared by the IRLS loop and the refinement block. Kept separate:
	// solveTranslation scores with the weights of its last reweight, while the
	// refinement block reweights at the refined transform before scoring.
	auto reweightRows = [&rows, &config](const Eigen::Vector3d &t, double scale)
	{
		for (auto &r : rows)
			r.weight = HuberWeight((r.dQ * t - (r.base + scale * r.scalePart)).norm(),
				config.huberTranslation);
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
		return std::sqrt(sq / wsum);   // every Huber weight is positive
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

	// Normal equations of the joint (t, s) system, leaving out every row that
	// touches a sample in [skipBegin, skipEnd) (the jackknife's blocks).
	auto jointNormalEquations = [&rows](size_t skipBegin, size_t skipEnd,
		Eigen::Matrix4d &ata, Eigen::Vector4d &atb)
	{
		ata.setZero();
		atb.setZero();
		for (const auto &r : rows)
		{
			if ((r.first >= skipBegin && r.first < skipEnd) ||
				(r.second >= skipBegin && r.second < skipEnd))
				continue;
			Eigen::Matrix<double, 3, 4> a;
			a.leftCols<3>() = r.dQ;
			a.col(3) = -r.scalePart;
			ata += r.weight * a.transpose() * a;
			atb += r.weight * a.transpose() * r.base;
		}
	};

	if (config.solveScale)
	{
		// Translation and scale are one linear system, seeded with the robust
		// fixed-scale solve above: an unweighted start would let a handful of
		// corrupted target positions drag the scale to a bound before IRLS has
		// identified them.
		auto reweightJointRows = [&rows, &config](
			const Eigen::Vector3d &t, double candidateScale)
		{
			for (auto &r : rows)
			{
				double ratio = HuberWeight((r.dQ * t -
					(r.base + candidateScale * r.scalePart)).norm(), config.huberTranslation);
				// Scale is an errors-in-variables column: a corrupted target
				// position raises both its residual and its leverage, and the
				// squared ratio bounds that leverage.
				r.weight = ratio * ratio;
			}
		};
		reweightJointRows(translation, scale);
		for (int iter = 0; iter <= 2 * config.irlsIterations; ++iter)
		{
			Eigen::Matrix4d ata;
			Eigen::Vector4d atb;
			jointNormalEquations(0, 0, ata, atb);
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
	// Pareto guard: the polish is accepted only when it does not measurably
	// worsen the axis-pair fit; otherwise the sequential result stands.
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

		// That textbook figure treats every row as independent, but rows are
		// overlapping sample pairs and a headset's position error wanders over a
		// second or more: simulated, it understated the scale's real scatter
		// 2.2x on white noise and 18x with 2 mm of slow wander, and one live
		// session reported 0.0007 for a scale the next calibration contradicted
		// by 0.028. The delete-one-block jackknife assumes neither; the larger
		// of the two is reported.
		const size_t sampleCount = samples.size();
		const size_t blocks = config.scaleJackknifeBlocks;
		if (blocks >= 2 && sampleCount >= 4 * blocks)
		{
			std::vector<double> leftOut;
			for (size_t b = 0; b < blocks; ++b)
			{
				Eigen::Matrix4d ata;
				Eigen::Vector4d atb;
				jointNormalEquations(b * sampleCount / blocks,
					(b + 1) * sampleCount / blocks, ata, atb);
				Eigen::LDLT<Eigen::Matrix4d> factor(ata);
				if (factor.info() != Eigen::Success || !factor.isPositive())
					continue;
				double blockScale = factor.solve(atb)(3);
				if (std::isfinite(blockScale))
					leftOut.push_back(blockScale);
			}
			if (leftOut.size() == blocks)
			{
				double mean = 0.0;
				for (double v : leftOut)
					mean += v;
				mean /= static_cast<double>(blocks);
				double spread = 0.0;
				for (double v : leftOut)
					spread += (v - mean) * (v - mean);
				double jackknife = std::sqrt(spread *
					static_cast<double>(blocks - 1) / static_cast<double>(blocks));
				result.scaleStdDev = std::max(result.scaleStdDev, jackknife);
			}
			else
			{
				// A stretch the scale cannot be solved without is not a
				// measured scale. Finite, because the figure is persisted
				// with the profile.
				result.scaleStdDev = std::max(result.scaleStdDev, 1.0);
			}
		}
		result.scaleIdentifiable =
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
		protocol::limits::MaxAbsTranslationMeters)
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
			&offsetScore, &offsetPeakMargin);
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

	EngineResult result = SolveAligned(aligned, config);

	// ---- motion-amplitude gain diagnostic + scale guard --------------------
	// (see EngineConfig::gainSplitSeconds). A contaminated scale is re-solved
	// at the clean gross-band gain, or at neutral scale.
	double gainLow = 0.0, gainHigh = 0.0;
	bool gainValid = result.valid &&
		EstimateMotionGain(refStream, targetStream, offset, config, result, gainLow, gainHigh);
	bool fineAttenuated = gainValid &&
		gainHigh < gainLow - config.gainSmoothingMargin;
	bool bandMismatch = gainValid &&
		std::abs(gainHigh - gainLow) > config.gainSmoothingMargin;

	// EstimateMotionGain abstains in the low-translation regime the calibration
	// instructions produce (rotation largely in place). An abstention means the
	// scale is not identifiable from this motion, not that it is clean, so it
	// takes the same guarded path at neutral scale.
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
		EngineResult r2 = SolveAligned(scaled, pinnedConfig);
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
			result.failure = r2.failure;
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
