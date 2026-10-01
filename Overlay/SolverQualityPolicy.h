#pragma once
#include <cmath>

namespace questcal {
enum class SolverQuality { Pass, NonFinite, SingleAxis, TranslationUnobservable,
	RotationResidual, PositionResidual };
// Called on the metrics of the final transform, after optional refinement.
// Thresholds are finite/nonnegative under IsValidEngineConfig. This boundary
// makes no assumption that Eigen produced a good solution.
inline SolverQuality JudgeSolverQuality(double axisSpread, double transEigRatio,
	double rotationRms, double translationRms, double minAxis, double minTrans,
	double maxRotation, double maxTranslation)
{
	if (!std::isfinite(axisSpread) || !std::isfinite(transEigRatio) ||
		!std::isfinite(rotationRms) || !std::isfinite(translationRms))
		return SolverQuality::NonFinite;
	if (axisSpread < minAxis) return SolverQuality::SingleAxis;
	if (transEigRatio < minTrans) return SolverQuality::TranslationUnobservable;
	if (rotationRms > maxRotation) return SolverQuality::RotationResidual;
	if (translationRms > maxTranslation) return SolverQuality::PositionResidual;
	return SolverQuality::Pass;
}
}
