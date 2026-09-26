#pragma once

#include "Calibration.h"

#include <functional>

void LoadProfile(CalibrationContext &ctx);
// Commits Config only; pending Settings work stays independent (the one-time
// legacy migration may materialize Settings first).
bool SaveProfile(CalibrationContext &ctx);
// Transactional UI operations: persist a narrow profile candidate first, then
// apply only the requested mutation to the live context, so a refused write
// leaves nothing to roll back.
bool ClearSavedProfile(CalibrationContext &ctx);
bool SaveProfileTransformEdit(CalibrationContext &ctx,
	const Eigen::Quaterniond &rotation,
	const Eigen::Vector3d &translationMeters, double scale,
	bool rotationEdited);
// The same transaction for every profile-backed preference edit; `mutate`
// edits the candidate record and never sees live state. `bumpFieldGeneration`
// makes the driver snap, so set it only for edits its field blend can see
// (fieldEnabled / fieldAnchors).
bool SaveProfileFieldEdit(CalibrationContext &ctx,
	const std::function<void(questcal::ProfileRecord &)> &mutate,
	bool bumpFieldGeneration = false);
// ProfileRecord's anchor type mirrors CalibrationContext's (see
// ProfileValidation.h); this is the one converter between them.
questcal::PersistedFieldAnchor PersistedAnchor(
	const CalibrationContext::FieldAnchor &anchor);
// Persists global UI/chaperone preferences. It normally leaves Config alone,
// but first flushes an already-dirty coupled Config revision so Settings can
// never overtake it. A malformed-but-recoverable profile is never overwritten.
bool SaveSettings(CalibrationContext &ctx);
// UI rollback depends only on settingsSaved; a pending independent Config
// failure remains an error/retry without undoing a committed preference.
questcal::SettingsSaveResult SaveSettingsWithResult(CalibrationContext &ctx);

// Flushes whichever persisted records are dirty while preserving the coupled
// Config-before-Settings ordering owned by SaveSettings.
bool SavePendingChanges(CalibrationContext &ctx);
