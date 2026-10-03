#include "../Overlay/CalibrationEngine.h"
#include "../Overlay/EngineConfigValidation.h"
#include "../Overlay/QualityBands.h"

#include <cstdio>

// The bands the result message and the main screen's rating share, at their
// edges, and the tuning values that moved into the solver's configuration.
namespace
{
using Check = void (*)(const char *, bool, const char *);

void RunSolveQualityScenario(Check check)
{
	using questcal::SolveQuality;
	struct Case
	{
		double rotationDeg;
		double translationM;
		SolveQuality expected;
	};
	const Case cases[] = {
		{ 0.5, 0.015, SolveQuality::Good },
		{ 3.0, 0.010, SolveQuality::Good },
		{ 0.5, 0.016, SolveQuality::Decent },
		{ 0.5, 0.030, SolveQuality::Decent },
		{ 0.5, 0.031, SolveQuality::Poor },
		// Past Good's rotation bound only the position separates the rest.
		{ 3.1, 0.010, SolveQuality::Decent },
		{ 3.1, 0.040, SolveQuality::Poor },
	};
	int wrong = 0;
	for (const auto &c : cases)
		wrong += questcal::JudgeSolveQuality(c.rotationDeg, c.translationM) != c.expected ? 1 : 0;
	// Every valid solve is within the solver's refusals: Good's rotation bound
	// is the solver's, and Poor is still reachable below its position refusal.
	const questcal::EngineConfig engine;
	const bool withinSolver = questcal::GoodSolveRotationRmsDeg == engine.maxRotationRms &&
		questcal::DecentSolveTranslationRmsMeters < engine.maxTranslationRms;
	char detail[96];
	snprintf(detail, sizeof detail, "%d of %zu misjudged, within the solver %d",
		wrong, sizeof cases / sizeof cases[0], withinSolver);
	check("quality bands: the result and the rating share one judgement",
		wrong == 0 && withinSolver, detail);
}

void RunEngineTuningScenario(Check check)
{
	// The thresholds that were literals in the solver are validated with the
	// rest of its configuration.
	const questcal::EngineConfig defaults;
	questcal::EngineConfig noMismatch = defaults;
	noMismatch.maxPairAngleMismatch = 0.0;
	questcal::EngineConfig loosePolish = defaults;
	loosePolish.maxRefinementAxisRmsRatio = 0.99;
	questcal::EngineConfig impossibleCorrelation = defaults;
	impossibleCorrelation.minTimeOffsetCorrelation = 1.5;
	const bool defaultValid = questcal::IsValidEngineConfig(defaults);
	const bool refused = !questcal::IsValidEngineConfig(noMismatch) &&
		!questcal::IsValidEngineConfig(loosePolish) &&
		!questcal::IsValidEngineConfig(impossibleCorrelation);
	char detail[96];
	snprintf(detail, sizeof detail, "defaults valid %d, invalid values refused %d", defaultValid, refused);
	check("solver: its tuning thresholds are validated configuration", defaultValid && refused, detail);
}
}

void RunQualityBandsScenarios(Check check)
{
	RunSolveQualityScenario(check);
	RunEngineTuningScenario(check);
}
