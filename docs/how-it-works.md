# How QuestCalibrator works

The technical detail behind the [README](../README.md): what changed from
OpenVR-SpaceCalibrator, how alignment is kept during play, what the logs record, and
where the math comes from.

## What changed from OpenVR-SpaceCalibrator

Solver (new `CalibrationEngine`, covered by synthetic tests in `Tests/`):

- **Inter-system time alignment.** The driver timestamps every raw pose at capture
  (QueryPerformanceCounter) and publishes it over shared memory; the solver estimates the
  constant latency between the two systems by cross-correlating angular-speed profiles and
  interpolates the reference stream accordingly. Latency between tracking systems converts
  hand speed directly into calibration error and was previously unmodeled.
- **Velocity gating.** Samples taken during fast motion are dropped using driver-reported
  velocities.
- **Gravity prior, not constraint.** The rotation solve is full 3-DOF Kabsch with a
  weighted virtual up-axis pair whose weight fades as real two-axis motion accumulates:
  single-axis (ill-conditioned) sessions stay gravity-aligned, genuinely tilted universes
  are still recovered.
- **Robust numerics.** Reflection-checked Kabsch, quaternion-based axis extraction (stable
  near 180 degrees), IRLS/Huber reweighting against jitter and glitches, sign-invariant
  axis conditioning check, and per-pair rigid-angle consistency rejection.
- **Fit validation.** Rotation RMS, translation RMS, and axis-diversity gates; the
  solver refuses (with a plain-language reason) rather than save a bad calibration.
- **Playspace scale** is an experimental, opt-in solve. A gross/fine motion-gain
  diagnostic distinguishes a frequency-flat metric difference from streamed-pose
  smoothing. When smoothing is detected, scale is fixed from a clean gross band or
  held at neutral 1.0 if gross motion is attenuated too; a contaminated free-scale
  fit is never applied.
- Calibration collects both streams for a fixed duration, solves once, and publishes
  the complete transform.

Driver and IPC:

- Base transforms and their field share one seqlock publication, with
  identity-quaternion / scale = 1 defaults instead of a zeroing `memset`.
- Device ids arriving over the pipe are bounds-checked; short pipe messages are rejected;
  the wire protocol (v9) requires a same-version per-connection handshake and only ever
  carries complete, transactionally validated transforms/fields.
- Race-free bounded multi-producer pose ring in shared memory (vrserver invokes pose
  updates from each device driver's own thread), with fail-fast contention handling,
  exact positional loss markers, and clean recovery across vrserver restarts. Its named
  mapping is layout-versioned separately from the pipe protocol so an older overlay
  cannot pin an incompatible mapping across an upgrade.
- The driver log lands next to the driver DLL instead of vrserver's working directory.

Profiles:

- The calibrated rotation is stored as a **quaternion** (plus translation in meters);
  Euler angles exist only in the profile editor UI. Stored in the per-user local-settings
  hive, which regedit shows as
  `HKEY_CURRENT_USER\Software\Classes\Local Settings\Software\QuestCalibrator` — profiles from
  upstream are not migrated. If the overlay reports a profile or settings record it cannot
  read, it preserves the record rather than overwriting it; deleting the `Config` or
  `Settings` value there is how you start over.

## Runtime alignment maintenance

Runtime alignment maintenance uses the timestamped pose ring and calibration solver:

- **Runtime latency re-prediction** — the driver shifts the lighthouse devices'
  prediction time by the solved inter-system offset, so vrserver's own predictor
  aligns the two timelines during live motion, not just at calibration time.
- **Universe-jump compensation** — pose discontinuities inconsistent with the
  device's reported velocity (headset recenter / SLAM re-localization) are detected
  and the detected universe delta is applied to the calibration. Active alignment monitoring
  runs every 50 ms even with the dashboard closed.
- **Drift detection** — alignment staleness is scored from calibration age plus
  stationary-slide and tracking-loss evidence, shown in the overlay, and raised as
  a one-shot notification instead of letting the alignment degrade silently.
- **Base station visibility** — SteamVR's lighthouse driver logs which base
  stations each tracker sees and when one drops out. QuestCalibrator follows that
  log, lays the stations out per device on the Lighthouse tab (which ones each
  tracker has in view, and which station drops out most), and stops counting a
  tracker's slide or re-appearance as drift evidence while it sees fewer than
  two stations and for ten seconds after it lost or regained one.
  Nothing changes when the log is unavailable. See
  [docs/lighthouse-visibility.md](https://github.com/VividNightmareUnleashed/QuestCalibrator/blob/alpha/docs/lighthouse-visibility.md).
- **Field anchors (spatial correction field)** — multi-point calibration interpolated by each
  device's own position (Gaussian RBF blending in the driver), correcting SLAM map
  deformation that a single rigid transform cannot represent.
- **Continuous calibration** — with a spare lighthouse tracker mounted firmly on the
  headset, a background loop keeps the alignment maintained during play. A
  head-referenced calibration learns the mount offset (with a rigidity gate), after
  which every time-aligned HMD+tracker pose pair directly measures the universe
  transform with no motion required. Small yaw+translation corrections are
  auto-applied and smoothed by the driver. A large deviation (a tracking fault,
  the lighthouse side moving) pauses auto-apply, and one that then holds still
  becomes the new calibration when SteamVR's log shows no restart of the
  headset tracker to explain it. The **Legacy** method in Settings never
  pauses: it follows every such deviation at once, the headset tracker's own
  faults included, as OpenVR-SpaceCalibrator does. The
  mounted tracker can be hidden from games so full-body setups never mistake it for
  a body tracker. Optional (off by default): online re-estimation of the
  inter-system time offset from the same rigid pair.
- **Languages** — English, Italian and Japanese, picked in Settings (Windows' display
  language is the default). The translations may not be accurate;
  corrections are welcome as GitHub issues. Japanese text is drawn with a font
  Windows already has (Yu Gothic, Meiryo or MS Gothic), and the session log and
  diagnostics stay in English.

Quest continuous mode smooths rotation and translation together at each device's
position, so a yaw correction about that device does not temporarily push it
sideways. Derived angular speeds are timestamped at their interval midpoints to
avoid introducing latency when the two devices report at different rates.

A paused Quest loop does not correct its way out of a large disagreement. With the
default configuration it resumes after five seconds of readings below 1° yaw and
2.5 cm at the head, or thirty seconds below the 2° / 5 cm that paused it. Readings
that stay off but hold still for thirty seconds become the calibration instead,
unless the headset tracker restarted its lighthouse tracking in the two minutes
before they moved, or since: that tracker's own fault waits for the resume. When
SteamVR's log cannot be read, or does not name the headset tracker, no restart
could show, so nothing becomes the calibration this way. If the
readings later return to the calibration it replaced, that one comes back. Tilt of
1.5° or more waits without pausing, and becomes the calibration the same way if it
holds still. The Legacy method skips all of this: readings past those limits
become the calibration at the next two-second evaluation, tilt of 1.5° or more
included. If tracking is clean but alignment stays wrong, use **Recalibrate
with the headset tracker**. Waiting in a particular posture is
not a calibration step. Manual profile editing changes the base transform; it does
not relearn the mounted tracker relationship.

Ordinary FBT users do not need a headset-mounted tracker for jump compensation
or drift warnings. Continuous drift correction without that rigid pair remains
an open research problem: independently moving body trackers do not uniquely
measure the headset-to-lighthouse alignment.

## What the logs record

For continuous-calibration failures, turn on **Detailed logging** in Settings,
then use **Save diagnostics file** once while alignment looks correct and again
after the problem appears, before recalibrating or restarting. Keep the devices
still while exporting. The report includes per-device input counts and freshness,
stream gaps, window resets, Quest observation gates, re-anchors, lighthouse frame
moves, and a snapshot of connected devices relative to SteamVR's floor. Counters
survive recalibration for the session. Exports also include raw driver
poses for every device, scale and timing confidence, mount and field transforms,
driver synchronization status, and executable hashes to identify the build. The
raw stream is sampled independently of continuous mode. Detailed-log snapshots are
written every ten seconds. Describe whether the problem is visible in SteamVR
tracker positions or only in the game, and identify which devices look misplaced.

A jump compensated within a minute of the reference stream resuming is logged
with that age. In an observed Quest wake sequence, streaming poses resumed
before full 6DoF tracking returned. A jump around that transition needs more
evidence before its effect on alignment can be judged.

Quest Pro controllers track on their own and can lag a headset map switch by
up to half a minute. A headset step that no controller matched at once is
held rather than dropped, and applied when a controller follows within 30 s;
the log line then says how much later it was confirmed. A step nothing
follows is discarded. Two engine habits are allowed for: a controller lying
still is frozen by the headset and cannot step until the hand moves, so the
30 s wait pauses while every controller is frozen and the step is confirmed
by their first movement; and a controller corrects up to 5 s ahead of the
headset after a recent reset, so a matching controller step that arrived
first also confirms. None of this applies without Quest controllers.

With no Quest controllers in use there is nothing to confirm a headset step,
so the headset's small tracking corrections used to be detected and then
discarded, and the alignment drifted by their sum. A clean headset step of at
least 5 cm or 2 degrees is now applied on its own once the headset's stream
has been continuous for a minute (a wake or a stream restart re-zeroes inside
that minute) and its position was not being held before the step (the
headset's 3DoF fallback). A step of any size that follows a held position is
never applied on its own: it is the headset catching up onto resumed
tracking, not a change of frame. Steps that are still discarded are totalled
in the log, so a session's ignored corrections can be compared with what the
next calibration removes.

One more refusal covers the headset's own drift correction. Under continuous
tracking the headset removes odometry drift by sliding while you move and,
once the remaining error exceeds its reset threshold (10 cm or 10 degrees),
snapping the rest in one step. That snap restores the alignment, so applying
it would put the drift back. Drift crosses the threshold a hair at a time,
so the snap is a step of almost exactly the threshold: a headset-only step
within half a centimetre of 10 cm or a third of a degree of 10 degrees is
refused as a drift catch-up and counted separately in the log. A change of
frame that happens to be that size is left for the next correction.

## The test harness

The deterministic harness includes fixed regression scenarios plus randomized
property trials over general rotations, translations, rigid mount transforms,
both latency signs, sample rates, irregular sample timing, scale, noise, and
outliers.
It also tests interpolation/gating/rejection contracts and compares the exact
driver pose-transform path against an independent Eigen oracle, then stresses
the actual named shared-memory pose ring with concurrent publishers.
For a longer deterministic campaign, pass `--property-trials N` and optionally
`--property-seed N` to `SolverTests.exe`; the default validation uses 64 trials.

## How the math works

The two-stage hand-eye solve is inherited from upstream — see
[math.pdf](https://github.com/pushrax/OpenVR-SpaceCalibrator/blob/master/math.pdf) for the
derivation, **together with [math errata](math-errata.md)**: two steps of the pdf's
algebra are wrong as written (the mount offset composes on the wrong side, and eq. 6 is
really a conjugation), and the errata also states the observability, latency, and scale
properties the implementation depends on. Rotation comes from paired delta-rotation axes
(the rigid mount offset cancels under conjugation) via Kabsch; translation is a linear
least-squares over sample pairs. This fork keeps that core and wraps it in the time
alignment, weighting, and validation described above.
