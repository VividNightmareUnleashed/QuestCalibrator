#include "stdafx.h"
#include "Configuration.h"
#include "ChaperoneMath.h"
#include "ProfileValidation.h"
#include "RecordBounds.h"
#include "SettingsRecordJson.h"
#include "UserInterface.h"
#include "../common/Protocol.h"

#include <picojson.h>

#include <string>
#include <fstream>
#include <iomanip>
#include <limits>
#include <cmath>

// PersistedCalibrationSpeed mirrors this enum (see ProfileValidation.h); pin
// the two together where both are visible.
static_assert(static_cast<int>(CalibrationContext::FAST) ==
	static_cast<int>(questcal::PersistedCalibrationSpeed::Fast) &&
	static_cast<int>(CalibrationContext::SLOW) ==
	static_cast<int>(questcal::PersistedCalibrationSpeed::Slow) &&
	static_cast<int>(CalibrationContext::VERY_SLOW) ==
	static_cast<int>(questcal::PersistedCalibrationSpeed::VerySlow),
	"CalibrationContext::Speed and PersistedCalibrationSpeed must agree");

using questcal::MountExtrinsicRecord;
using questcal::PersistedFieldAnchor;
using questcal::PersistedRevision;
using questcal::ProfileParseResult;
using questcal::ProfileRecord;
using questcal::ChaperoneRecord;
using questcal::SettingsRecord;
using questcal::ParseChaperone;
using questcal::WriteChaperone;
using questcal::ValidateChaperoneRecord;
using questcal::WriteSettings;
using questcal::ParseSettings;

// Registry I/O captures and applies the portable records. The actual profile,
// settings and chaperone codecs are shared with the implementation contracts.
static ChaperoneRecord CaptureChaperoneRecord(
	const CalibrationContext::Chaperone &source)
{
	ChaperoneRecord record;
	if (!source.valid)
		return record;
	record.valid = source.valid;
	record.autoApply = source.autoApply;
	record.ownerTrackingSystem = source.ownerTrackingSystem;
	record.ownerHmdSerial = source.ownerHmdSerial;
	record.worldFromDriverValid = source.worldFromDriverValid;
	record.worldFromDriverRotation = source.worldFromDriverRotation;
	record.worldFromDriverTranslation = source.worldFromDriverTranslation;
	record.geometry = source.geometry;
	record.standingCenter = source.standingCenter;
	record.playSpaceSize = source.playSpaceSize;
	record.copyUnixTime = source.copyUnixTime;
	return record;
}

static SettingsRecord CaptureSettingsRecord(const CalibrationContext &ctx)
{
	SettingsRecord record;
#define QUESTCAL_CAPTURE_SWITCH(member, key, value) record.member = ctx.member;
	QUESTCAL_SETTINGS_SWITCHES(QUESTCAL_CAPTURE_SWITCH)
#undef QUESTCAL_CAPTURE_SWITCH
	record.calibrationSpeed = static_cast<questcal::PersistedCalibrationSpeed>(ctx.calibrationSpeed);
	record.language = ctx.language;
	record.deviceNames = ctx.deviceNames;
	record.chaperone = CaptureChaperoneRecord(ctx.chaperone);
	return record;
}

questcal::PersistedFieldAnchor PersistedAnchor(
	const CalibrationContext::FieldAnchor &anchor)
{
	PersistedFieldAnchor persisted;
	persisted.position = anchor.position;
	persisted.rotation = anchor.rotation;
	persisted.translationMeters = anchor.translationMeters;
	return persisted;
}

static ProfileRecord CaptureProfileRecord(const CalibrationContext &ctx)
{
	ProfileRecord record;
	record.valid = ctx.validProfile;
	record.referenceTrackingSystem = ctx.referenceTrackingSystem;
	record.targetTrackingSystem = ctx.targetTrackingSystem;
	record.rotation = ctx.transform.rotation;
	record.translationMeters = ctx.transform.translationMeters;
	record.scale = ctx.transform.scale;
	record.timeOffset = ctx.transform.timeOffset;
	record.calibrationUnixTime = ctx.calibrationUnixTime;
	record.universeUnsafe = ctx.profileUniverseUnsafe;
	record.universeValid = ctx.profileUniverseValid;
	record.universeHmdSerial = ctx.profileHmdSerial;
	record.universeRotation = ctx.profileWorldFromDriverRotation;
	record.universeTranslation = ctx.profileWorldFromDriverTranslation;
#define QUESTCAL_CAPTURE_SWITCH(member, key, value) record.member = ctx.member;
	QUESTCAL_PROFILE_SWITCHES(QUESTCAL_CAPTURE_SWITCH)
#undef QUESTCAL_CAPTURE_SWITCH
	record.fieldAnchors.reserve(ctx.fieldAnchors.size());
	for (const auto &anchor : ctx.fieldAnchors)
		record.fieldAnchors.push_back(PersistedAnchor(anchor));
	record.continuousTrackerSerial = ctx.continuousTrackerSerial;
	record.continuousNoPause = ctx.continuousNoPause;
	record.mountExtrinsic.valid = ctx.mountExtrinsic.valid;
	record.mountExtrinsic.rotation = ctx.mountExtrinsic.rot;
	record.mountExtrinsic.translationMeters = ctx.mountExtrinsic.pos;
	record.mountExtrinsic.rotationRmsDeg = ctx.mountExtrinsic.rotRmsDeg;
	record.mountExtrinsic.translationRmsM = ctx.mountExtrinsic.posRmsM;
	return record;
}

static void ApplyChaperoneRecord(CalibrationContext &ctx, ChaperoneRecord record)
{
	CalibrationContext::Chaperone applied;
	applied.valid = record.valid;
	applied.autoApply = record.autoApply;
	applied.ownerTrackingSystem = std::move(record.ownerTrackingSystem);
	applied.ownerHmdSerial = std::move(record.ownerHmdSerial);
	applied.worldFromDriverValid = record.worldFromDriverValid;
	applied.worldFromDriverRotation = record.worldFromDriverRotation;
	applied.worldFromDriverTranslation = record.worldFromDriverTranslation;
	applied.geometry = std::move(record.geometry);
	applied.standingCenter = record.standingCenter;
	applied.playSpaceSize = record.playSpaceSize;
	applied.copyUnixTime = record.copyUnixTime;
	ctx.chaperone = std::move(applied);
}

static void ApplySettingsRecord(CalibrationContext &ctx, SettingsRecord record)
{
#define QUESTCAL_APPLY_SWITCH(member, key, value) ctx.member = record.member;
	QUESTCAL_SETTINGS_SWITCHES(QUESTCAL_APPLY_SWITCH)
#undef QUESTCAL_APPLY_SWITCH
	ctx.calibrationSpeed = static_cast<CalibrationContext::Speed>(record.calibrationSpeed);
	ctx.language = record.language;
	ctx.deviceNames = record.deviceNames;
	ApplyChaperoneRecord(ctx, std::move(record.chaperone));
}

// The preference half of a profile record (spatial field, continuous pick and
// mount), shared by the load path and SaveProfileFieldEdit. The edit path must
// not re-apply the base transform, which would bump baseGeneration per toggle.
static void ApplyProfilePreferences(
	CalibrationContext &ctx, const ProfileRecord &record)
{
#define QUESTCAL_APPLY_SWITCH(member, key, value) ctx.member = record.member;
	QUESTCAL_PROFILE_SWITCHES(QUESTCAL_APPLY_SWITCH)
#undef QUESTCAL_APPLY_SWITCH
	ctx.fieldAnchors.clear();
	ctx.fieldAnchors.reserve(record.fieldAnchors.size());
	for (const auto &persisted : record.fieldAnchors)
	{
		CalibrationContext::FieldAnchor anchor;
		anchor.position = persisted.position;
		anchor.rotation = persisted.rotation;
		anchor.translationMeters = persisted.translationMeters;
		ctx.fieldAnchors.push_back(anchor);
	}
	ctx.continuousTrackerSerial = record.continuousTrackerSerial;
	ctx.continuousNoPause = record.continuousNoPause;
	// Only the persisted members: MountExtrinsic::pairs is a runtime statistic
	// that the load path resets and a preference edit must keep.
	ctx.mountExtrinsic.valid = record.mountExtrinsic.valid;
	ctx.mountExtrinsic.rot = record.mountExtrinsic.rotation;
	ctx.mountExtrinsic.pos = record.mountExtrinsic.translationMeters;
	ctx.mountExtrinsic.rotRmsDeg = record.mountExtrinsic.rotationRmsDeg;
	ctx.mountExtrinsic.posRmsM = record.mountExtrinsic.translationRmsM;
}

static void ApplyProfileRecord(CalibrationContext &ctx, ProfileRecord record)
{
	ctx.ResetTrackerFrames();
	ctx.referenceTrackingSystem = std::move(record.referenceTrackingSystem);
	ctx.targetTrackingSystem = std::move(record.targetTrackingSystem);
	ctx.SetCalibration(record.rotation, record.translationMeters, record.scale);
	ctx.transform.timeOffset = record.timeOffset;
	ctx.calibrationUnixTime = record.calibrationUnixTime;
	ctx.profileUniverseUnsafe = record.universeUnsafe;
	ctx.frameMovesLost = false;   // not saved; see its declaration
	ctx.profileUniverseValid = record.universeValid;
	ctx.profileHmdSerial = std::move(record.universeHmdSerial);
	ctx.profileWorldFromDriverRotation = record.universeRotation;
	ctx.profileWorldFromDriverTranslation = record.universeTranslation;
	ctx.mountExtrinsic = questcal::MountExtrinsic();
	ApplyProfilePreferences(ctx, record);
	ctx.validProfile = record.valid;
}

using questcal::FloatArray;
using questcal::GetDouble;
using questcal::HasTypedValue;
using questcal::LoadFloatArray;
using questcal::ReadPersistenceRevision;
using questcal::RejectExcessiveJsonNesting;

// One definition of a well-formed chaperone snapshot for parser and writer.
// Only the writer requires a complete owner: the load path disarms an ownerless
// snapshot instead of failing the whole record (and a good calibration with it).
// The Settings-owned half of a legacy Config record: older releases embedded
// the global settings alongside the profile. An absent key leaves the caller's
// already-loaded value alone rather than resetting it to a default.
static void ParseLegacyEmbeddedSettings(const questcal::LegacyProfileSettings &legacy,
	const picojson::object &obj, SettingsRecord &settings)
{
	if (legacy.hasApplyTimeOffset)
		settings.applyTimeOffset = legacy.applyTimeOffset;
	if (legacy.hasSolveScale)
		settings.solveScale = legacy.solveScale;
	if (legacy.hasUiAdvanced)
		settings.uiAdvanced = legacy.uiAdvanced;
	if (legacy.hasChaperoneWarningAck)
		settings.chaperoneWarningAck = legacy.chaperoneWarningAck;
	if (legacy.hasCalibrationSpeed)
		settings.calibrationSpeed =
			static_cast<questcal::PersistedCalibrationSpeed>(legacy.calibrationSpeed);

	ParseChaperone(settings, obj);
}

// The shared codec reads the profile; this adds what it cannot see: the legacy
// embedded settings and the chaperone snapshot.
static ProfileParseResult ParseProfile(ProfileRecord &profile,
	SettingsRecord &legacySettings, std::istream &stream)
{
	picojson::value profileValue = questcal::ParseProfileEnvelope(stream);
	const auto &obj = profileValue.get<picojson::object>();

	questcal::LegacyProfileSettings legacy;
	ProfileParseResult result = questcal::ParseProfileObject(
		profile, legacy, obj, protocol::SetAlignmentField::MaxAnchors);
	ParseLegacyEmbeddedSettings(legacy, obj, legacySettings);
	return result;
}

static std::string RegistryError(LSTATUS result)
{
	char *message = nullptr;
	DWORD chars = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, result, LANG_USER_DEFAULT,
		reinterpret_cast<LPSTR>(&message), 0, nullptr);
	std::string text = chars != 0 && message ? message : "Windows error " + std::to_string(result);
	if (message)
		LocalFree(message);
	while (!text.empty() && (text.back() == '\r' || text.back() == '\n' || text.back() == ' '))
		text.pop_back();
	return text;
}

static const char *RegistryKey = "Software\\QuestCalibrator";

// HKEY_CURRENT_USER_LOCAL_SETTINGS resolves under HKCU\Software\Classes\Local
// Settings, which regedit does not label. An unreadable record is never
// overwritten, so the error text must say where to delete it by hand.
static const char *RegistryKeyDisplayPath =
	"HKEY_CURRENT_USER\\Software\\Classes\\Local Settings\\Software\\QuestCalibrator";

enum class RegistryReadStatus
{
	Missing,
	Present,
	Error,
};

struct RegistryReadResult
{
	RegistryReadStatus status = RegistryReadStatus::Missing;
	std::string value;
	std::string error;
};

static RegistryReadResult ReadRegistryValue(const char *valueName)
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, valueName, RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS)
	{
		if (result == ERROR_FILE_NOT_FOUND)
			return {};
		return { RegistryReadStatus::Error, {},
			std::string("reading ") + valueName + ": " + RegistryError(result) };
	}

	// size counts the trailing NUL; zero would underflow the resize below.
	if (size == 0)
		return { RegistryReadStatus::Present, {}, {} };
	if (size > questcal::MaxRecordBytes)
	{
		return { RegistryReadStatus::Error, {},
			std::string(valueName) + " exceeds the 16 MiB safety limit" };
	}

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, valueName, RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS)
	{
		return { RegistryReadStatus::Error, {},
			std::string("reading ") + valueName + ": " + RegistryError(result) };
	}

	if (size == 0)
		return { RegistryReadStatus::Present, {}, {} };

	str.resize(size - 1);
	return { RegistryReadStatus::Present, std::move(str), {} };
}

static bool WriteRegistryValue(const char *valueName, const std::string &str, std::string &error)
{
	if (str.size() >= questcal::MaxRecordBytes)
	{
		error = std::string(valueName) + " exceeds the 16 MiB safety limit";
		return false;
	}

	HKEY hkey = nullptr;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0,
		KEY_SET_VALUE, nullptr, &hkey, nullptr);
	if (result != ERROR_SUCCESS)
	{
		error = "opening the registry key: " + RegistryError(result);
		return false;
	}

	DWORD size = static_cast<DWORD>(str.size() + 1);

	result = RegSetValueExA(hkey, valueName, 0, REG_SZ,
		reinterpret_cast<const BYTE *>(str.c_str()), size);
	RegCloseKey(hkey);
	if (result != ERROR_SUCCESS)
	{
		error = std::string("writing ") + valueName + ": " + RegistryError(result);
		return false;
	}
	return true;
}

// Runs once, at startup, on a context whose load states are still Missing.
void LoadProfile(CalibrationContext &ctx)
{
	PersistedRevision profileRevision;
	PersistedRevision settingsRevision;
	bool settingsRewriteNeeded = false;

	auto profileRead = ReadRegistryValue("Config");
	if (profileRead.status == RegistryReadStatus::Error)
	{
		ctx.profileLoadState = questcal::RecordLoadState::Unreadable;
		ctx.Log("Calibration profile read failed: " + profileRead.error + "\n");
		ctx.ReportError("Couldn't read the saved calibration, so it was left as it is. "
			"Restart QuestCalibrator. If this repeats, save a diagnostics file in Settings and report it.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
	}
	else if (profileRead.status == RegistryReadStatus::Missing || profileRead.value.empty())
	{
		AppendSessionLog("no saved calibration profile");
		ctx.Clear();
	}
	else
	{
		try
		{
			RejectExcessiveJsonNesting(profileRead.value);
			std::stringstream io(profileRead.value);
			// Parse transactionally. A malformed late field must not leave an
			// earlier transform active after the overall profile load failed.
			ProfileRecord profile = CaptureProfileRecord(ctx);
			SettingsRecord legacySettings = CaptureSettingsRecord(ctx);
			profile.valid = false;
			ProfileParseResult parsed = ParseProfile(profile, legacySettings, io);
			profileRevision = parsed.revision;
			ApplyProfileRecord(ctx, std::move(profile));
			ApplySettingsRecord(ctx, std::move(legacySettings));
			ctx.profileLoadState = questcal::RecordLoadState::Loaded;
			if (parsed.migratedScaleSetting)
			{
				ctx.Log("Playspace scale solving is now opt-in and has been turned off for this profile (re-enable it in settings if you need it)\n");
				if (parsed.suspiciousLegacyScale)
				{
					ctx.Log("The stored playspace scale (" +
						std::to_string(ctx.transform.scale).substr(0, 5) +
						"x) likely came from streamed-pose smoothing -- recalibrate to clear it\n");
				}
			}
			ctx.ClearError(CalibrationContext::ErrorSource::ProfilePersistence);
			AppendSessionLog("calibration profile loaded");
		}
		catch (const std::exception &e)
		{
			ctx.profileLoadState = questcal::RecordLoadState::Unreadable;
			ctx.ReportError(std::string("Error loading calibration profile: ") + e.what() +
					"\nThe profile is preserved, not overwritten. To start over, delete the"
					" Config value under " + RegistryKeyDisplayPath + "\n",
				CalibrationContext::ErrorSource::ProfilePersistence);
		}
	}

	// Settings keeps chaperone protection and global preferences alive even
	// with no calibration profile.
	auto settingsRead = ReadRegistryValue("Settings");
	if (settingsRead.status == RegistryReadStatus::Error)
	{
		ctx.settingsLoadState = questcal::RecordLoadState::Unreadable;
		ctx.Log("Settings read failed: " + settingsRead.error + "\n");
		ctx.ReportError("Couldn't read QuestCalibrator's settings, so they were left as they are. "
			"Restart QuestCalibrator. If this repeats, save a diagnostics file in Settings and report it.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
	}
	// Missing or empty keeps whatever the (possibly legacy) Config load produced;
	// PlanPersistenceLoad decides whether to materialize Settings from it.
	else if (settingsRead.status == RegistryReadStatus::Present && !settingsRead.value.empty())
	{
		try
		{
			SettingsRecord parsed = CaptureSettingsRecord(ctx);
			RejectExcessiveJsonNesting(settingsRead.value);
			std::stringstream io(settingsRead.value);
			settingsRevision = ParseSettings(parsed, io);
			ApplySettingsRecord(ctx, std::move(parsed));
			ctx.settingsLoadState = questcal::RecordLoadState::Loaded;
			ctx.ClearError(CalibrationContext::ErrorSource::SettingsPersistence);
		}
		catch (const std::exception &e)
		{
			ctx.settingsLoadState = questcal::RecordLoadState::Unreadable;
			// Until this record is cleared every settings write is refused, so
			// the toggles in the UI will silently roll back. Say where it is.
			ctx.ReportError(std::string("Error loading application settings: ") + e.what() +
					"\nSettings changes will not save until this is fixed. To start over,"
					" delete the Settings value under " + RegistryKeyDisplayPath + "\n",
				CalibrationContext::ErrorSource::SettingsPersistence);
		}
	}
	// PlanPersistenceLoad decides everything from these facts; this block only
	// gathers them and applies the verdict.
	questcal::PersistenceLoadFacts facts;
	facts.profile = ctx.profileLoadState;
	facts.settings = ctx.settingsLoadState;
	facts.profileRevision = profileRevision;
	facts.settingsRevision = settingsRevision;
	facts.chaperoneArmed = ctx.chaperone.valid;
	facts.chaperoneOwnerComplete = questcal::IsCompleteChaperoneOwner(
		ctx.chaperone.ownerTrackingSystem, ctx.chaperone.ownerHmdSerial,
		ctx.chaperone.worldFromDriverValid);
	facts.chaperoneOwnerMatchesReference =
		ctx.chaperone.ownerTrackingSystem == ctx.referenceTrackingSystem;
	facts.profileValid = ctx.validProfile;

	questcal::PersistenceLoadPlan plan = questcal::PlanPersistenceLoad(facts);
	settingsRewriteNeeded = plan.settingsRewriteNeeded;
	ctx.persistence.SetRevision(plan.persistenceRevision);
	if (plan.legacySettingsMigrationPending)
		ctx.persistence.legacySettingsMigrationPending = true;
	if (plan.disarmChaperone)
		ctx.DisarmChaperone();
	if (plan.reportRevisionMismatch)
	{
		ctx.ReportError(
			"The calibration profile and protected chaperone were saved at different revisions. "
			"Chaperone auto-restore is disabled until you capture it again.\n",
			CalibrationContext::ErrorSource::Chaperone);
	}

	switch (plan.gate)
	{
	case questcal::ChaperoneLoadGate::Armed:
		break;
	case questcal::ChaperoneLoadGate::ProfileUnreadable:
		ctx.Log("Protected chaperone left disarmed because the calibration profile could not be read\n");
		break;
	case questcal::ChaperoneLoadGate::SettingsUnreadable:
		ctx.Log("Protected chaperone left disarmed because application settings could not be read\n");
		break;
	case questcal::ChaperoneLoadGate::IncompleteOwner:
		ctx.ReportError(
			"The protected chaperone has no complete headset/universe baseline. "
			"It has been disarmed; capture it again before enabling auto-restore.\n",
			CalibrationContext::ErrorSource::Chaperone);
		break;
	case questcal::ChaperoneLoadGate::ForeignTrackingSystem:
		ctx.ReportError(
			"The protected chaperone belongs to a different reference tracking system. "
			"It has been disarmed; capture it again for this profile.\n",
			CalibrationContext::ErrorSource::Chaperone);
		break;
	}

	// A new coupled revision, not the stranded one: if only Config lands, the
	// two records still differ and the next load still fails closed.
	if (plan.profileRewriteNeeded)
	{
		ctx.persistence.AdvanceRevision();
		ctx.persistence.MarkProfile(ctx.timeLastTick);
	}

	if (settingsRewriteNeeded)
	{
		ctx.persistence.MarkSettings(ctx.timeLastTick);
		if (!SaveSettings(ctx))
			ctx.persistence.legacySettingsMigrationPending =
				plan.legacySettingsMigrationPendingIfRewriteFails;
	}

	ctx.pendingReferenceTrackingSystem = ctx.referenceTrackingSystem;
	ctx.pendingTargetTrackingSystem = ctx.targetTrackingSystem;
}

// SaveProfile/SaveSettings coordinate the two records: either may commit the
// other first to keep the Config-before-Settings order and shared revision.
// Write*Record writes exactly one record and never calls a coordinator, which
// is what terminates SaveSettings -> SaveProfile -> WriteConfigRecord ->
// WriteSettingsRecord.
static bool WriteSettingsRecord(CalibrationContext &ctx);

// Logged once per record per session, so a preview never silently "saves".
static void NotePreviewWriteSkipped(CalibrationContext &ctx, const char *what,
	bool &announced)
{
	if (announced)
		return;
	announced = true;
	ctx.Log(std::string("UI preview mode: ") + what +
		" was not written to the registry\n");
}

static bool WriteConfigRecord(CalibrationContext &ctx, const ProfileRecord &record)
{
	questcal::PersistenceWriteGate gate = questcal::GateProfileWrite(
		g_uiPreviewMode, ctx.profileLoadState, ctx.settingsLoadState,
		ctx.persistence.legacySettingsMigrationPending);
	switch (gate)
	{
	case questcal::PersistenceWriteGate::Allowed:
		break;
	case questcal::PersistenceWriteGate::SkippedPreview:
		{
			static bool announced = false;
			NotePreviewWriteSkipped(ctx, "the calibration profile", announced);
		}
		return true;
	case questcal::PersistenceWriteGate::RefusedConfigUnreadable:
		ctx.ReportError(
			"Couldn't save the calibration because the saved one couldn't be read. "
			"It was left untouched so it can be recovered.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	case questcal::PersistenceWriteGate::RefusedSettingsUnreadable:
		ctx.ReportError(
			"Couldn't save the calibration because QuestCalibrator's settings couldn't be read. "
			"They were left untouched so they can be recovered.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	if (ctx.persistence.legacySettingsMigrationPending && !WriteSettingsRecord(ctx))
	{
		ctx.ReportError(
			"Couldn't save the calibration because settings from an older version haven't been "
			"moved over safely yet. Restart QuestCalibrator and try again.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	// The same definition the parser enforces, so a record that saves is a
	// record that will load again.
	std::string why;
	if (!questcal::ValidateProfileRecord(
		record, protocol::SetAlignmentField::MaxAnchors, why))
	{
		ctx.Log("Calibration profile rejected: " + why + "\n");
		ctx.ReportError("Couldn't save the calibration because it failed a safety check. Recalibrate.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}

	std::stringstream profile;
	questcal::WriteProfile(record, ctx.persistence.revision, profile);
	std::string error;
	if (!WriteRegistryValue("Config", profile.str(), error))
	{
		ctx.Log("Calibration profile write failed: " + error + "\n");
		ctx.ReportError("Couldn't save the calibration. It will be lost when QuestCalibrator closes.\n",
			CalibrationContext::ErrorSource::ProfilePersistence);
		return false;
	}
	ctx.persistence.profileDirty = false;
	ctx.profileLoadState = record.valid
		? questcal::RecordLoadState::Loaded
		: questcal::RecordLoadState::Missing;
	ctx.ClearError(CalibrationContext::ErrorSource::ProfilePersistence);
	return true;
}

bool SaveProfile(CalibrationContext &ctx)
{
	return WriteConfigRecord(ctx, CaptureProfileRecord(ctx));
}

bool ClearSavedProfile(CalibrationContext &ctx)
{
	// An empty Config carries no revision. Flush newer Settings first so a
	// crash after clearing Config cannot hide a coupled-write mismatch and make
	// the previous chaperone snapshot look authoritative on the next launch.
	// SaveSettings also commits a dirty Config first when both halves are pending.
	if (ctx.persistence.settingsDirty && !SaveSettings(ctx))
		return false;

	ProfileRecord cleared;
	if (!WriteConfigRecord(ctx, cleared))
		return false;
	ctx.Clear();
	return true;
}

bool SaveProfileTransformEdit(CalibrationContext &ctx,
	const Eigen::Quaterniond &rotation,
	const Eigen::Vector3d &translationMeters, double scale,
	bool rotationEdited)
{
	ProfileRecord candidate = CaptureProfileRecord(ctx);
	if (rotationEdited)
		candidate.rotation = rotation.normalized();
	candidate.translationMeters = translationMeters;
	candidate.scale = scale;

	if (!WriteConfigRecord(ctx, candidate))
		return false;

	if (rotationEdited)
		ctx.SetCalibration(candidate.rotation, candidate.translationMeters, candidate.scale);
	else
	{
		// Do not normalize or otherwise rewrite the quaternion when only the
		// translation/scale changed: its persisted bits remain the source truth.
		ctx.transform.translationMeters = candidate.translationMeters;
		ctx.transform.scale = candidate.scale;
		ctx.baseGeneration++;
	}
	if (!ctx.fieldAnchors.empty())
		ctx.fieldGeneration++;
	ctx.state = CalibrationState::None;
	ctx.timeLastScan = -1e9;
	return true;
}

bool SaveProfileFieldEdit(CalibrationContext &ctx,
	const std::function<void(questcal::ProfileRecord &)> &mutate,
	bool bumpFieldGeneration)
{
	ProfileRecord candidate = CaptureProfileRecord(ctx);
	mutate(candidate);
	if (!WriteConfigRecord(ctx, candidate))
		return false;

	ApplyProfilePreferences(ctx, candidate);
	if (bumpFieldGeneration)
		ctx.fieldGeneration++;
	return true;
}

static bool WriteSettingsRecord(CalibrationContext &ctx)
{
	questcal::PersistenceWriteGate gate = questcal::GateSettingsWrite(
		g_uiPreviewMode, ctx.profileLoadState, ctx.settingsLoadState);
	switch (gate)
	{
	case questcal::PersistenceWriteGate::Allowed:
		break;
	case questcal::PersistenceWriteGate::SkippedPreview:
		{
			static bool announced = false;
			NotePreviewWriteSkipped(ctx, "the application settings", announced);
		}
		return true;
	case questcal::PersistenceWriteGate::RefusedConfigUnreadable:
		ctx.ReportError(
			"Couldn't save QuestCalibrator's settings because the saved calibration couldn't be read. "
			"Nothing was changed, so both can still be recovered.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	case questcal::PersistenceWriteGate::RefusedSettingsUnreadable:
		ctx.ReportError(
			"Couldn't save QuestCalibrator's settings because the saved ones couldn't be read. "
			"They were left untouched so they can be recovered.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}
	SettingsRecord record = CaptureSettingsRecord(ctx);
	// An unowned room cannot be restored, so refuse to write one.
	std::string why;
	if (!ValidateChaperoneRecord(record.chaperone, true, why))
	{
		ctx.Log("Settings rejected: " + why + "\n");
		ctx.ReportError(
			"Couldn't save QuestCalibrator's settings because the protected chaperone failed a safety check. "
			"Protect it again.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}

	std::stringstream settings;
	WriteSettings(record, ctx.persistence.revision, settings);
	std::string error;
	if (!WriteRegistryValue("Settings", settings.str(), error))
	{
		ctx.Log("Settings write failed: " + error + "\n");
		ctx.ReportError("Couldn't save QuestCalibrator's settings. Changes will be lost when it closes.\n",
			CalibrationContext::ErrorSource::SettingsPersistence);
		return false;
	}
	ctx.persistence.legacySettingsMigrationPending = false;
	ctx.settingsLoadState = questcal::RecordLoadState::Loaded;
	ctx.persistence.settingsDirty = false;
	ctx.ClearError(CalibrationContext::ErrorSource::SettingsPersistence);
	return true;
}

bool SaveSettings(CalibrationContext &ctx)
{
	return SaveSettingsWithResult(ctx).AllSaved();
}

// profileDirty implies validProfile: every MarkProfile site requires a valid
// profile, and Clear() (the only way to lose one) drops the pending write.
questcal::SettingsSaveResult SaveSettingsWithResult(CalibrationContext &ctx)
{
	return ctx.persistence.SaveSettings(
		[&]() { return SaveProfile(ctx); },
		[&]() { return WriteSettingsRecord(ctx); });
}

bool SavePendingChanges(CalibrationContext &ctx)
{
	if (ctx.persistence.settingsDirty)
		return SaveSettings(ctx);
	if (ctx.persistence.profileDirty && !SaveProfile(ctx))
		return false;
	ctx.persistence.coupled = false;
	return true;
}
