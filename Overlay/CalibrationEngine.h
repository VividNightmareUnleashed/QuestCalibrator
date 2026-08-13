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
//      reflection handling, inverse-variance angle weighting (a small delta's
//      axis is noise-amplified by 1/theta), IRLS/Huber reweighting, and a
//      gravity prior in the form of weighted virtual up-axis pairs (prior,
//      not constraint: rich motion outvotes it, so genuinely tilted universes
//      are still recovered).
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
	// Within noise of a half turn the shortest-arc hemisphere choice
	// decorrelates between the two streams, so a pair can enter Kabsch with
	// anti-aligned axes that no downstream check can detect (both angles are
	// ~pi, so the angle-mismatch gate passes). Reject the ambiguous band.
	double maxPairAngle = 2.9;         // rad
	size_t maxPairs = 20000;           // all-pairs count is thinned above this
	double gravityPriorRatio = 0.5;    // virtual up-pair weight as a fraction of data weight; 0 disables
	int    irlsIterations = 4;
	double huberRotation = 0.06;       // rad; residuals above this are downweighted
	double huberTranslation = 0.03;    // m

	// --- joint refinement ---
	// Gauss-Newton polish over (R, t, mount offset, s) on the full pose-pair
	// position residuals after the sequential pipeline: the eq. 8 solve treats
	// the Kabsch rotation as exact, so its residual error otherwise leaks into
	// the translation as a bias that grows with distance from the sampled
	// cloud. 0 disables.
	int    refineIterations = 3;

	// --- scale ---
	bool   solveScale = false;
	double scaleSearchRange = 0.15;    // searched as [1-r, 1+r]

	// --- motion-amplitude gain diagnostic + scale guard ---
	// The two position tracks' amplitude ratio, split into a gross-motion band
	// and a fine-motion band by a moving average of gainSplitSeconds. A
	// genuine metric scale difference is frequency-flat (both bands ~= s);
	// streamed-pose smoothing is a low-pass, so its fingerprint is the fine
	// band's gain sitting below the gross band's — and it drags the solved
	// scale down with it (the least-squares scale reflects the motion the
	// calibration wiggling actually contains). When that fingerprint is
	// detected, the gross-band gain is used only if it is demonstrably clean
	// (near unity). If both bands are attenuated, unity is the only defensible
	// scale: the observed motion cannot identify a physical metric difference.
	double gainSplitSeconds = 1.2;
	double gainSmoothingMargin = 0.03; // fine below gross by this = smoothing detected
	double maxCleanGrossDeviation = 0.03;
	bool   pinScaleOnSmoothing = true; // replace the contaminated solved scale with guarded fixed scale

	// --- validation gates ---
	double maxRotationRms = 3.0;       // degrees
	double maxTranslationRms = 0.05;   // meters
	double minAxisSpread = 0.010;      // second/first eigenvalue ratio of the axis cloud
	// Observability of the translation itself (math.pdf eq. 8): each pair row
	// constrains Fp only perpendicular to its relative-rotation axis, so a
	// near-common axis leaves that direction (the vertical, under yaw-dominant
	// motion) resting on noise. The axis-spread gate is a poor proxy — axis
	// outer products ignore the rotation magnitudes that weight the rows, so
	// a brief nod burst can pass it while every LARGE delta is still pure yaw
	// — hence the translation normal matrix is gated on its own eigenvalue
	// ratio. 0.008 ~= 11x noise amplification along the weak direction.
	double minTransEigRatio = 0.008;   // smallest/largest eigenvalue of sum dQ^T dQ
	size_t minPairs = 30;
};

// What the scale guard did (see EngineConfig::gainSplitSeconds). Three
// mutually-exclusive outcomes, so a consumer reads one value instead of
// reconstructing them from flags that can disagree. `NotApplied` also covers
// "attempted, but the guarded re-solve failed" - that case is already
// distinguished by `valid == false` plus the message.
enum class ScaleGuard
{
	NotApplied,
	FromGrossMotion,          // guard replaced the solved scale with motionGainLow
	NeutralizedForSmoothing,  // no trustworthy band; guard used unity
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
	double transEigRatio = 0.0;        // translation-system conditioning; ~0 = a direction is unobservable

	// Motion-amplitude gain diagnostic (see EngineConfig::gainSplitSeconds).
	bool   motionGainValid = false;
	double motionGainLow = 0.0;        // gross-motion band; ~true scale under either hypothesis
	double motionGainHigh = 0.0;       // fine-motion band; sits below gross under smoothing
	// Independent of the guard below: this legitimately fires with solveScale off.
	bool   motionSmoothingDetected = false;
	ScaleGuard scaleGuard = ScaleGuard::NotApplied;
	bool   refinementApplied = false;      // joint Gauss-Newton result passed its Pareto guard
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

	// Exposed for tests and diagnostics. validateInputs=false skips the
	// stream-integrity scan ONLY; internal callers pass it after validating
	// once. The config is revalidated on every entry point regardless of this
	// flag - it is cheap, and a caller must never be able to skip the refusal
	// that names a bad knob.
	static bool EstimateTimeOffset(const std::vector<PoseSample> &refStream,
	                               const std::vector<PoseSample> &targetStream,
	                               const EngineConfig &config,
	                               double &offsetOut,
	                               bool validateInputs = true);

	static bool InterpolateAt(const std::vector<PoseSample> &stream, double t,
	                          double maxGap, PoseSample &out);

	// Solve over pre-aligned pairs (skips alignment; used internally and by tests).
	static EngineResult SolveAligned(const std::vector<AlignedSample> &samples,
	                                 const EngineConfig &config,
	                                 bool validateInputs = true);
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
