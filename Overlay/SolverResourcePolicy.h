#pragma once
#include <cstddef>
#include <cstdint>
#include <cmath>

namespace questcal {
namespace solverresource {
constexpr size_t MaxRawSamples = 100000;
constexpr size_t MaxAlignedSamples = 4096;
constexpr size_t MaxPairs = 100000;
constexpr int MaxLagSteps = 2000;
constexpr double MaxGridPoints = 1000000;
constexpr double MaxCorrelationWork = 100000000;

// Preconditions: 0 < budget <= count <= MaxRawSamples, index < budget.
// These bounds make the integer product safe on the supported 64-bit target.
inline size_t ThinIndex(size_t index, size_t count, size_t budget)
{
	return (index * count) / budget;
}

inline bool GridFits(double points, int lagSteps)
{
	return std::isfinite(points) && points >= 0 && points <= MaxGridPoints &&
		lagSteps >= 0 && lagSteps <= MaxLagSteps &&
		points * (2.0 * lagSteps + 1.0) <= MaxCorrelationWork;
}
} // namespace solverresource
} // namespace questcal
