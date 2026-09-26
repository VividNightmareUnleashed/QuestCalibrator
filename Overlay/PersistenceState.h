#pragma once

#include <cstdint>

namespace questcal
{
	struct SettingsSaveResult
	{
		bool profileSaved;
		bool settingsSaved;
		bool AllSaved() const { return profileSaved && settingsSaved; }
	};

	// When the two registry records get written, and which of them. Free of
	// OpenVR and the UI so the harness can compile it. Callers own the ordered
	// writes (see SavePendingChanges) and deciding that a failure should Retry.
	struct PersistenceState
	{
		// A registry write per continuous correction would be one write every
		// few seconds forever, so a dirty record waits for the stream to go
		// quiet. But a steadily drifting universe never goes quiet, and the
		// debounce alone would defer every record to shutdown and lose the
		// session to a crash or a SteamVR kill — hence the ceiling.
		static constexpr double QuietPeriodSeconds = 5.0;
		static constexpr double MaxDirtyAgeSeconds = 60.0;

		// Independent: a partial write leaves only the failed record dirty.
		bool profileDirty = false;
		bool settingsDirty = false;
		// One shared clock for both records: any persistent mutation restarts
		// the quiet period, while the dirty bits above retain partial-write
		// state across it.
		double dirtyTime = 0.0;
		// When the current dirty streak began, for the ceiling above.
		double firstDirtyTime = 0.0;
		uint32_t revision = 0;   // LoadProfile sets it before any write
		// A universe rebase writes the calibration and the protected standing
		// center as one revision: the Settings half must not land without the
		// Config half. Independent dirty bits carry no such ordering rule.
		bool coupled = false;
		// Older releases embedded global settings in Config. Until their first
		// Settings write succeeds, SaveProfile must not replace that only copy.
		bool legacySettingsMigrationPending = false;

		// A persisted revision is never zero: the reader treats 0 as absent, so
		// a record written with 0 would read back revisionless. Every write of
		// `revision`, the wrap in AdvanceRevision included, goes through here.
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

		// An independent Settings commit can succeed while Config still needs
		// a retry. Its caller must retain the committed preference in that case.
		template<typename P, typename S>
		SettingsSaveResult SaveSettings(P saveProfile, S saveSettings)
		{
			bool profileSaved = !profileDirty || saveProfile();
			if (!profileSaved && coupled)
				return { false, false };
			bool settingsSaved = saveSettings();
			if (!HasDirty())
				coupled = false;
			return { profileSaved, settingsSaved };
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
			dirtyTime = now;
			firstDirtyTime = now;
		}

		// Clearing a calibration makes only the profile's pending write moot;
		// Settings, the revision and the migration flag outlive it.
		void OnProfileDiscarded()
		{
			profileDirty = false;
		}
	};
}
