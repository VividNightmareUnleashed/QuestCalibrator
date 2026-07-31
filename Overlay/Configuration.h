#pragma once

#include "Calibration.h"

void LoadProfile(CalibrationContext &ctx);
// Reports success for the Config commit and leaves ordinary pending Settings
// work independent (the one-time legacy migration may materialize Settings).
bool SaveProfile(CalibrationContext &ctx);
// Transactional UI operations: persist a narrow profile candidate first, then
// apply only the requested profile mutation to the live runtime context.
bool ClearSavedProfile(CalibrationContext &ctx);
bool SaveProfileTransformEdit(CalibrationContext &ctx,
	const Eigen::Quaterniond &rotation,
	const Eigen::Vector3d &translationMeters, double scale,
	bool rotationEdited);
// Persists global UI/chaperone preferences. It normally leaves Config alone,
// but first flushes an already-dirty coupled Config revision so Settings can
// never overtake it. A malformed-but-recoverable profile is never overwritten.
bool SaveSettings(CalibrationContext &ctx);
