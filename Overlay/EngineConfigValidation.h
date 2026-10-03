#pragma once
#include "CalibrationEngine.h"
#include "SolverResourcePolicy.h"
#include "../common/MathConstants.h"
#include "../common/TransformLimits.h"
#include <cmath>

namespace questcal {
inline bool IsValidEngineConfig(const EngineConfig &c)
{
	const double nonnegative[] = {c.timeOffsetRange, c.maxLinearSpeed,
		c.maxAngularSpeed, c.gravityPriorRatio, c.scaleSearchRange, c.minScaleCondition,
		c.maxScaleStdDev, c.gainSmoothingMargin, c.maxCleanGrossDeviation,
		c.maxRotationRms, c.maxTranslationRms, c.minAxisSpread, c.minTransEigRatio};
	for (double value : nonnegative)
		if (!std::isfinite(value) || value < 0 || value > 1e6) return false;
	const double positive[] = {c.timeOffsetStep, c.maxInterpolationGap,
		c.minPairAngle, c.maxPairAngle, c.maxPairAngleMismatch, c.huberRotation,
		c.huberTranslation, c.gainSplitSeconds};
	for (double value : positive)
		if (!std::isfinite(value) || value < 1e-12 || value > 1e6) return false;
	return std::isfinite(c.fallbackTimeOffset) &&
		std::isfinite(c.minTimeOffsetCorrelation) &&
		c.minTimeOffsetCorrelation >= -1.0 && c.minTimeOffsetCorrelation <= 1.0 &&
		std::isfinite(c.maxRefinementAxisRmsRatio) &&
		c.maxRefinementAxisRmsRatio >= 1.0 && c.maxRefinementAxisRmsRatio <= 1e6 &&
		std::abs(c.fallbackTimeOffset) <= protocol::limits::MaxAbsTimeOffsetSeconds &&
		c.timeOffsetRange <= protocol::limits::MaxAbsTimeOffsetSeconds &&
		c.timeOffsetRange / c.timeOffsetStep <= solverresource::MaxLagSteps &&
		c.maxAlignedSamples >= 8 && c.maxAlignedSamples <= solverresource::MaxAlignedSamples &&
		c.minPairs >= 1 && c.minPairs <= c.maxPairs && c.maxPairs <= solverresource::MaxPairs &&
		c.irlsIterations >= 0 && c.irlsIterations <= 64 &&
		c.refineIterations >= 0 && c.refineIterations <= 64 &&
		c.scaleJackknifeBlocks <= 32 && c.scaleJackknifeBlocks != 1 &&
		// This numeric envelope keeps the angle-weight normalization finite:
		// every base is at least 1e-12 and there are at most MaxPairs bases.
		c.minPairAngle >= 1e-6 && c.minPairAngle < c.maxPairAngle && c.maxPairAngle < questcal::Pi &&
		c.scaleSearchRange <= 1.0 - protocol::limits::MinScale &&
		c.minAxisSpread <= 1 && c.minTransEigRatio <= 1 &&
		c.minScaleCondition <= 1;
}
} // namespace questcal
