#pragma once
#include <cmath>

namespace questcal { namespace robust {
// Preconditions: finite residual >= 0 and finite knee > 0. IRLS uses this
// weight; weight*r*r is the scoring convention, not the textbook Huber loss.
inline double HuberWeight(double residual, double knee)
{
	return residual <= knee ? 1.0 : knee / residual;
}
inline bool CostImproved(double before, double after)
{
	return std::isfinite(before) && std::isfinite(after) && after < before;
}
} }
