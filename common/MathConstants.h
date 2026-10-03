#pragma once

namespace questcal
{

// Pi as a double. Eigen's EIGEN_PI is a long double literal: on MSVC, where
// long double is double, every run-time expression built from it narrows back
// to double, which /W4 reports as C4244. On MSVC the results are bit-identical.
constexpr double Pi = 3.14159265358979323846;

}
