#pragma once

#include "Calibration.h"

#include <functional>

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
// The same transaction for every profile-backed preference edit — the spatial
// field, the continuous-calibration pick and mount, the game-visibility and
// latency toggles. `mutate` states the edit on a candidate record and nothing
// else: it never sees live state, so no call site owns a rollback, and the
// partial hand-written snapshots that used to sit at each toggle (and could not
// undo the persistence-layer state a failed write leaves behind) are gone.
// `bumpFieldGeneration` is for edits the driver's field blend can see
// (fieldEnabled / fieldAnchors) — it makes the driver snap, so a tracker-pick
// or hide-in-games toggle must not set it.
bool SaveProfileFieldEdit(CalibrationContext &ctx,
	const std::function<void(questcal::ProfileRecord &)> &mutate,
	bool bumpFieldGeneration = false);
// ProfileRecord's anchor type is deliberately not CalibrationContext's (see
// ProfileValidation.h). One converter, so a caller building a candidate anchor
// set does not have to restate that the two are field-identical.
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

// A pending Config write with no valid profile to write is the same fault
// wherever it is noticed, so both flush paths say the same thing about it.
inline constexpr const char *PendingProfileWithoutValidProfileMessage =
	"Could not flush the pending calibration profile: there is no valid profile to write\n";
