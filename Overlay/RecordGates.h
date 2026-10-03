#pragma once

// What may be used and written once the records are read: each record's
// load state, and the gates every write passes.

namespace questcal
{

enum class RecordLoadState
{
	Missing,
	Loaded,
	Unreadable,
};

// A separately parsed Settings record is already authoritative and remains
// usable when Config is damaged. Falling back to embedded/default settings is
// safe only when Config was absent or parsed successfully.
inline bool CanUseRecoveredSettings(
	RecordLoadState settings, RecordLoadState config)
{
	return settings == RecordLoadState::Loaded ||
		config != RecordLoadState::Unreadable;
}

// Creating or automatically rewriting Settings can discard the only legacy
// settings copy hidden inside Config, so it requires Config to be absent or to
// have parsed successfully even when a separate Settings record was readable.
inline bool CanMaterializeSettings(RecordLoadState config)
{
	return config != RecordLoadState::Unreadable;
}

inline bool CanPersistSettings(
	RecordLoadState config, RecordLoadState settings)
{
	return config != RecordLoadState::Unreadable ||
		settings == RecordLoadState::Loaded;
}

// ---------------------------------------------------------------------------
// Write gates
// ---------------------------------------------------------------------------

enum class PersistenceWriteGate
{
	Allowed,
	// -uipreview runs the UI on fabricated state and must never touch HKCU.
	// The only outcome that reports success without writing, so a preview flag
	// set on a normal launch would silently lose every save.
	SkippedPreview,
	RefusedConfigUnreadable,
	RefusedSettingsUnreadable,
};

inline PersistenceWriteGate GateProfileWrite(bool previewMode,
	RecordLoadState config, RecordLoadState settings,
	bool legacySettingsMigrationPending)
{
	if (previewMode)
		return PersistenceWriteGate::SkippedPreview;
	if (config == RecordLoadState::Unreadable)
		return PersistenceWriteGate::RefusedConfigUnreadable;
	// The one-time migration materializes Settings as part of the profile
	// write, so an unreadable Settings record blocks the profile write too:
	// completing it would strip the legacy copy out of Config while the only
	// other copy is unreadable.
	if (legacySettingsMigrationPending && settings == RecordLoadState::Unreadable)
		return PersistenceWriteGate::RefusedSettingsUnreadable;
	return PersistenceWriteGate::Allowed;
}

inline PersistenceWriteGate GateSettingsWrite(
	bool previewMode, RecordLoadState config, RecordLoadState settings)
{
	if (previewMode)
		return PersistenceWriteGate::SkippedPreview;
	if (!CanPersistSettings(config, settings))
		return PersistenceWriteGate::RefusedConfigUnreadable;
	if (settings == RecordLoadState::Unreadable)
		return PersistenceWriteGate::RefusedSettingsUnreadable;
	return PersistenceWriteGate::Allowed;
}

} // namespace questcal
