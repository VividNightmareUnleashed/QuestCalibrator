#pragma once

#include <cstdint>

namespace questcal
{
	// When the two registry records get written, and which of them. This was
	// eight loose fields plus their methods on CalibrationContext, reachable by
	// every file that touched the context; it is one closed state machine
	// touched only by the persistence path, so it is a member instead. Nothing
	// here knows about OpenVR or the UI, which is also what lets the harness
	// compile it — as loose fields on the context these rules had no direct
	// coverage at all, because Calibration.h cannot be compiled by the tests.
	//
	// What the callers own, and this type does not: performing the writes,
	// their ordering (Settings first, see SaveDirtyPersistence), and the
	// decision that a failed write should Retry.
	struct PersistenceState
	{
		// A registry write per continuous correction would be one write every
		// few seconds forever, so a dirty record waits for the stream to go
		// quiet. But a steadily drifting universe never goes quiet, and the
		// debounce alone would defer every record to shutdown and lose the
		// session to a crash or a SteamVR kill — hence the ceiling.
		static constexpr double QuietPeriodSeconds = 5.0;
		static constexpr double MaxDirtyAgeSeconds = 60.0;

		// Independent, and deliberately so: a partial write must leave exactly
		// the record that failed still dirty, so it can be retried alone.
		bool profileDirty = false;
		bool settingsDirty = false;
		// One shared clock for both records: any persistent mutation restarts
		// the quiet period, while the dirty bits above retain partial-write
		// state across it.
		double dirtyTime = 0.0;
		// When the current dirty streak began, for the ceiling above.
		double firstDirtyTime = 0.0;
		uint32_t revision = 0;
		// A universe rebase writes the calibration and the protected standing
		// center as one revision: the Settings half must not land without the
		// Config half. Independent dirty bits carry no such ordering rule.
		bool coupled = false;
		// Older releases embedded global settings in Config. Until their first
		// Settings write succeeds, SaveProfile must not replace that only copy.
		bool legacySettingsMigrationPending = false;

		// "A persisted revision is never zero" lives here and nowhere else. The
		// reader rejects anything below 1, so 0 means "absent" on the way in; a
		// record written with 0 would read back revisionless, which is exactly
		// the partial-write mismatch the shared revision exists to detect. Both
		// record writers re-apply this rule to the live value immediately
		// before serializing, and the wrap case goes through it too.
		void SetRevision(uint32_t value)
		{
			revision = value == 0 ? 1u : value;
		}

		void AdvanceRevision()
		{
			SetRevision(revision + 1);
			coupled = true;
		}

		bool HasDirty() const
		{
			return profileDirty || settingsDirty;
		}

		// Starts the maximum-age clock only on the clean -> dirty transition, so
		// a stream of corrections cannot push the forced flush out indefinitely.
		void StartDebounce(double now)
		{
			if (!HasDirty())
				firstDirtyTime = now;
			dirtyTime = now;
		}

		void MarkProfile(double now)
		{
			StartDebounce(now);
			profileDirty = true;
		}

		void MarkSettings(double now)
		{
			StartDebounce(now);
			settingsDirty = true;
		}

		void MarkProfileAndSettings(double now)
		{
			StartDebounce(now);
			profileDirty = true;
			settingsDirty = true;
		}

		// Quiet long enough, or dirty too long.
		bool Due(double now) const
		{
			if (!HasDirty())
				return false;
			return !(now - dirtyTime <= QuietPeriodSeconds &&
				now - firstDirtyTime <= MaxDirtyAgeSeconds);
		}

		// Restart both clocks: a record that keeps refusing the write must retry
		// on the quiet-period cadence, not once per tick because the ceiling has
		// passed.
		void Retry(double now)
		{
			if (!HasDirty())
				return;
			dirtyTime = now;
			firstDirtyTime = now;
		}

		// Clearing a calibration discards the profile, so its pending write is
		// moot. Everything else survives on purpose: the Settings record is not
		// part of the calibration being discarded, and dropping the revision or
		// the migration flag here would lose partial-write and legacy-copy state
		// that outlives any single calibration.
		void OnProfileDiscarded()
		{
			profileDirty = false;
		}
	};
}
