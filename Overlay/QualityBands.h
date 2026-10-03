#pragma once

// The quality bands that the calibration result, the main screen's rating
// and the alignment health share, so the words after a calibration and the
// rating cannot disagree.

namespace questcal
{

// How good a solve's residuals are. A valid solve is already within the
// solver's refusals (EngineConfig::maxRotationRms, 3 deg, and
// maxTranslationRms, 5 cm), so beyond Good's rotation bound only the position
// residual separates the bands.
enum class SolveQuality
{
	Good,
	Decent,
	Poor,
};

constexpr double GoodSolveRotationRmsDeg = 3.0;
constexpr double GoodSolveTranslationRmsMeters = 0.015;
constexpr double DecentSolveTranslationRmsMeters = 0.03;

inline SolveQuality JudgeSolveQuality(double rotationRmsDeg, double translationRmsMeters)
{
	if (rotationRmsDeg <= GoodSolveRotationRmsDeg &&
		translationRmsMeters <= GoodSolveTranslationRmsMeters)
		return SolveQuality::Good;
	return translationRmsMeters <= DecentSolveTranslationRmsMeters
		? SolveQuality::Decent : SolveQuality::Poor;
}

// Bands of CalibrationContext::driftScore (0 to 1): the alignment health the
// core assigns, and where the main screen calls the alignment very poor.
constexpr double DriftAgingScore = 0.30;
constexpr double DriftStaleScore = 0.65;
constexpr double DriftVeryPoorScore = 0.85;

} // namespace questcal
