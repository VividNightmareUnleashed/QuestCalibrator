#pragma once

// Calibration solver for fusing two VR tracking universes.
//
// Pure Eigen + std; no OpenVR or protocol dependencies, so the synthetic test
// harness compiles exactly the code the overlay ships.
//
// Pipeline:
//   1. Inter-system time alignment: estimate the constant latency between the
//      two pose streams by cross-correlating angular-speed profiles, then
//      interpolate the reference stream onto the (shifted) target timestamps.
//   2. Velocity gating: samples taken during fast motion carry the largest
//      residual timing error and are dropped (they can be re-enabled by config).
//   3. Rotation: paired delta-rotation axes -> weighted Kabsch with
//      reflection handling, IRLS/Huber reweighting, and a gravity prior in the
//      form of weighted virtual up-axis pairs (prior, not constraint: rich
//      motion outvotes it, so genuinely tilted universes are still recovered).
//   4. Translation: linear least squares over sample pairs (math.pdf eq. 8)
//      with the same IRLS weights; optional playspace scale via a 1-D search.
//   5. Validation: rotation and translation residuals, axis-cloud conditioning,
//      and a plain-language verdict. `valid` is only set when all gates pass.

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace questcal
{

// One device pose in its own tracking universe, on a common clock.
struct PoseSample
{
	double time = 0.0;                 // seconds
	Eigen::Quaterniond rot{ 1, 0, 0, 0 };
	Eigen::Vector3d pos{ 0, 0, 0 };
	Eigen::Vector3d vel{ 0, 0, 0 };    // m/s (zero if the driver does not report it)
	Eigen::Vector3d angVel{ 0, 0, 0 }; // rad/s (zero if the driver does not report it)
};

// A time-aligned reference/target pose pair fed to the solve.
struct AlignedSample
{
	double time = 0.0;
	PoseSample ref;
	PoseSample target;
};

struct EngineConfig
{
	// --- time alignment ---
	bool   estimateTimeOffset = true;
	double timeOffsetRange = 0.06;     // seconds searched on each side of zero
	double timeOffsetStep = 0.002;     // coarse grid; refined parabolically

	// --- sample gating ---
	double maxLinearSpeed = 1.6;       // m/s; samples above either bound are dropped
	double maxAngularSpeed = 8.0;      // rad/s
	double maxInterpolationGap = 0.06; // seconds; do not interpolate across dropouts
	size_t maxAlignedSamples = 240;    // evenly thinned above this

	// --- rotation solve ---
	double minPairAngle = 0.4;         // rad; both deltas must rotate at least this much
	size_t maxPairs = 20000;           // all-pairs count is thinned above this
	double gravityPriorRatio = 0.5;    // virtual up-pair weight as a fraction of data weight; 0 disables
	int    irlsIterations = 4;
	double huberRotation = 0.06;       // rad; residuals above this are downweighted
	double huberTranslation = 0.03;    // m

	// --- scale ---
	bool   solveScale = false;
	double scaleSearchRange = 0.15;    // searched as [1-r, 1+r]

	// --- validation gates ---
	double maxRotationRms = 3.0;       // degrees
	double maxTranslationRms = 0.05;   // meters
	double minAxisSpread = 0.010;      // second/first eigenvalue ratio of the axis cloud
	size_t minPairs = 30;
};

struct EngineResult
{
	bool valid = false;

	Eigen::Quaterniond rotation{ 1, 0, 0, 0 };  // maps target universe into reference universe
	Eigen::Vector3d translation{ 0, 0, 0 };     // meters
	double scale = 1.0;
	double timeOffset = 0.0;                    // seconds; positive = target stream lags reference

	// Quality metrics (populated even when invalid, when computable).
	double rotationRmsDeg = 0.0;       // axis-pair residual after the solve
	double translationRmsMeters = 0.0; // translation LS residual
	double axisSpread = 0.0;           // rotation-axis diversity; ~0 = single-axis motion
	double tiltDeg = 0.0;              // pitch+roll magnitude of the solution (diagnostic)
	size_t samplesUsed = 0;
	size_t samplesGated = 0;
	size_t pairsUsed = 0;
	size_t pairsRejected = 0;

	std::string message;               // plain-language verdict for the UI
};

class CalibrationEngine
{
public:
	// Full pipeline over two raw streams (each sorted by time).
	static EngineResult Solve(const std::vector<PoseSample> &refStream,
	                          const std::vector<PoseSample> &targetStream,
	                          const EngineConfig &config);

	// Exposed for tests and diagnostics.
	static bool EstimateTimeOffset(const std::vector<PoseSample> &refStream,
	                               const std::vector<PoseSample> &targetStream,
	                               const EngineConfig &config,
	                               double &offsetOut);

	static bool InterpolateAt(const std::vector<PoseSample> &stream, double t,
	                          double maxGap, PoseSample &out);

	// Solve over pre-aligned pairs (skips alignment; used internally and by tests).
	static EngineResult SolveAligned(const std::vector<AlignedSample> &samples,
	                                 const EngineConfig &config);
};

// Runtime application of the solved inter-system time offset.
//
// The engine's solved offset is positive when the *target* stream lags the
// reference. At runtime the target devices' DriverPose_t::poseTimeOffset is
// shifted by the returned value: a positive shift declares the pose "newer",
// shortening vrserver's forward prediction — i.e. presenting the device
// slightly in the past. With a laggy wireless reference (Quest HMD) and fresh
// lighthouse targets, the solved offset is negative and the shift = -solved
// delays the targets to match the reference timeline (consistency over
// freshness; the relative wobble during motion is what users perceive, not
// the absolute latency).
//
// The clamp is asymmetric: delaying (positive shift) only backdates along
// already-measured motion, while advancing (negative shift) extends velocity
// extrapolation, which amplifies jitter during fast motion.
double ComputeAppliedTimeOffset(double solvedTimeOffset,
                                double maxDelaySeconds = 0.050,
                                double maxAdvanceSeconds = 0.015);

} // namespace questcal
