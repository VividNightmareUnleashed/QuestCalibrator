#pragma once

// The load-time decision over both records: what is applied, what is
// rewritten, and whether the protected chaperone stays armed.

#include "ProfileRecord.h"
#include "RecordGates.h"

#include <cstdint>
#include <string>

namespace questcal
{

// ---------------------------------------------------------------------------
// Load decision matrix
// ---------------------------------------------------------------------------

// Why the protected chaperone ended the load disarmed. Each carries its own
// message because the user's next action differs: two of them are recoverable
// by fixing a registry value, two require capturing the room again.
enum class ChaperoneLoadGate
{
	Armed,
	// Config could not be read and there is no separate Settings record, so
	// Config may hold the only copy of the room. Disarm in memory, persist
	// nothing.
	ProfileUnreadable,
	// A present-but-unreadable Settings record is authoritative over any legacy
	// snapshot embedded in a Config that happened to parse.
	SettingsUnreadable,
	IncompleteOwner,
	ForeignTrackingSystem,
};

struct PersistenceLoadFacts
{
	RecordLoadState profile = RecordLoadState::Missing;
	RecordLoadState settings = RecordLoadState::Missing;
	PersistedRevision profileRevision;
	PersistedRevision settingsRevision;
	// State of the chaperone snapshot after both records have been parsed.
	bool chaperoneArmed = false;
	bool chaperoneOwnerComplete = false;
	bool chaperoneOwnerMatchesReference = true;
	bool profileValid = false;
};

struct PersistenceLoadPlan
{
	uint32_t persistenceRevision = 1;
	// The two records were written at different revisions (or the Settings half
	// never landed). A chaperone captured before a universe rebase must not be
	// restored onto the rebased playspace, so this fails closed.
	bool revisionMismatch = false;
	// ... and there was actually a snapshot to lose. The mismatch is only worth
	// telling the user about when it cost them something.
	bool reportRevisionMismatch = false;
	bool disarmChaperone = false;
	ChaperoneLoadGate gate = ChaperoneLoadGate::Armed;
	bool settingsRewriteNeeded = false;
	bool legacySettingsMigrationPending = false;
	// If that rewrite fails, this is the value the migration latch must fall
	// back to. Getting it wrong lets one later profile save destroy a legacy
	// user's only copy of their global settings and protected room.
	bool legacySettingsMigrationPendingIfRewriteFails = false;
	// Config must be rewritten as a new coupled revision (AdvanceRevision,
	// MarkProfile) before the Settings rewrite, so that it carries a revision
	// again and a partial repair still reads as a mismatch.
	bool profileRewriteNeeded = false;
};

// The load-time state machine over {profile} x {settings} x {revision}, pure so
// every cell is testable. VirtualQuest's FormalConformanceTests compares it
// cell by cell with the Lean model (VirtualQuest/formal/lean/LoadPlan.lean).
inline PersistenceLoadPlan PlanPersistenceLoad(const PersistenceLoadFacts &facts)
{
	PersistenceLoadPlan plan;
	const bool profileLoaded = facts.profile == RecordLoadState::Loaded;
	const bool settingsLoaded = facts.settings == RecordLoadState::Loaded;
	const bool settingsMissing = facts.settings == RecordLoadState::Missing;
	// An unreadable record is preserved, never migrated over: rewriting
	// Settings is allowed only when Settings is absent or parsed AND Config was
	// not the unreadable one holding the only legacy copy.
	const bool settingsCanRewrite = (settingsMissing || settingsLoaded) &&
		CanMaterializeSettings(facts.profile);
	bool armed = facts.chaperoneArmed;

	plan.legacySettingsMigrationPendingIfRewriteFails =
		profileLoaded && !facts.profileRevision.present;

	// Config is written before Settings for a coupled universe rebase. If the
	// process dies between those writes, their revisions differ (or Settings is
	// absent). Never restore a raw-space boundary from that mixed state.
	if (profileLoaded && facts.profileRevision.present)
	{
		plan.persistenceRevision = facts.profileRevision.value;
		if (!settingsLoaded || !facts.settingsRevision.present ||
			facts.settingsRevision.value != facts.profileRevision.value)
		{
			plan.revisionMismatch = true;
			plan.reportRevisionMismatch = armed;
			if (armed)
			{
				plan.disarmChaperone = true;
				armed = false;
			}
			plan.settingsRewriteNeeded = settingsCanRewrite;
		}
	}
	else if (profileLoaded)
	{
		// Config-only releases kept the sole copy of global settings and the
		// chaperone inside Config. Materialize Settings before any new-format
		// profile save is allowed to strip those embedded fields.
		plan.persistenceRevision =
			facts.settingsRevision.present ? facts.settingsRevision.value : 1;
		if (!settingsLoaded || !facts.settingsRevision.present)
		{
			plan.legacySettingsMigrationPending = true;
			plan.settingsRewriteNeeded = settingsCanRewrite;
		}
		else if (facts.settingsRevision.value > 1)
		{
			// The migration materializes Settings at revision 1, and every later
			// revision is coupled, written Config first. But while the migration
			// is pending, WriteConfigRecord writes Settings before Config, so a
			// coupled write can leave its Settings half here with a Config that
			// never got its half, and no Config revision to compare against. Treat
			// it as the mismatch it is, and have Config rewritten as the next
			// revision so it carries one again.
			plan.revisionMismatch = true;
			plan.reportRevisionMismatch = armed;
			if (armed)
			{
				plan.disarmChaperone = true;
				armed = false;
			}
			plan.settingsRewriteNeeded = settingsCanRewrite;
			// Only a valid profile can be saved; a coupled save of an invalid
			// one would hold every Settings write back for the session.
			plan.profileRewriteNeeded = facts.profileValid;
		}
	}
	else
	{
		plan.persistenceRevision =
			facts.settingsRevision.present ? facts.settingsRevision.value : 1;
		if (settingsMissing || (settingsLoaded && !facts.settingsRevision.present))
			plan.settingsRewriteNeeded = settingsCanRewrite;
	}

	// Assignment, not |=, in the two gates below: they are the same question
	// the revision branch already answered, and a gate that fires while
	// rewriting is forbidden must not leave an earlier `true` standing.
	if (!CanUseRecoveredSettings(facts.settings, facts.profile))
	{
		plan.gate = ChaperoneLoadGate::ProfileUnreadable;
		if (armed)
			plan.disarmChaperone = true;
	}
	else if (facts.settings == RecordLoadState::Unreadable && armed)
	{
		plan.gate = ChaperoneLoadGate::SettingsUnreadable;
		plan.disarmChaperone = true;
	}
	else if (armed && !facts.chaperoneOwnerComplete)
	{
		plan.gate = ChaperoneLoadGate::IncompleteOwner;
		plan.disarmChaperone = true;
		plan.settingsRewriteNeeded = settingsCanRewrite;
	}
	else if (armed && facts.profileValid && !facts.chaperoneOwnerMatchesReference)
	{
		plan.gate = ChaperoneLoadGate::ForeignTrackingSystem;
		plan.disarmChaperone = true;
		plan.settingsRewriteNeeded = settingsCanRewrite;
	}

	return plan;
}

} // namespace questcal
