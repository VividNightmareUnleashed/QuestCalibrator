# QuestCalibrator

A personal fork of [OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator)
that fuses two VR tracking systems (e.g. a Quest headset plus lighthouse trackers) into a
single SteamVR playspace, rebuilt around a hardened driver and a new calibration solver.

> **Important:** uninstall or disable the original OpenVR-SpaceCalibrator (and any fork of
> it) before installing this. Both register a SteamVR driver that rewrites device poses;
> two of them active at once will apply two transforms and mangle tracking.

## What changed vs. upstream

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
- **Honest validation.** Rotation RMS, translation RMS, and axis-diversity gates; the
  solver refuses (with a plain-language reason) rather than save a bad calibration.
- **Playspace scale** is an experimental, opt-in solve. A gross/fine motion-gain
  diagnostic distinguishes a frequency-flat metric difference from streamed-pose
  smoothing. When smoothing is detected, scale is fixed from a clean gross band or
  held at neutral 1.0 if gross motion is attenuated too; a contaminated free-scale
  fit is never applied.
- Calibration collects both streams for a fixed duration and solves once — the two-stage
  rotation-then-translation dance (and its partial-transform IPC updates) is gone.

Driver and IPC:

- Transform slots are seqlock-protected against the IPC-thread/pose-thread race, with
  identity-quaternion / scale = 1 defaults instead of a zeroing `memset`.
- Device ids arriving over the pipe are bounds-checked; short pipe messages are rejected;
  the wire protocol (v6) requires a same-version per-connection handshake and only ever
  carries complete, transactionally validated transforms/fields.
- Race-free bounded multi-producer pose ring in shared memory (vrserver invokes pose
  updates from each device driver's own thread), with fail-fast contention handling,
  exact positional loss markers, and clean recovery across vrserver restarts. Its named
  mapping is layout-versioned separately from the v6 pipe protocol so an older overlay
  cannot pin an incompatible mapping across an upgrade.
- The driver log lands next to the driver DLL instead of vrserver's working directory.

Profiles:

- The calibrated rotation is stored as a **quaternion** (plus translation in meters);
  Euler angles exist only in the profile editor UI. Stored under
  `HKCU\Software\QuestCalibrator` — profiles from upstream are not migrated.

## Runtime alignment maintenance

Calibrating once is the easy part; these keep the alignment true during play. All of
them build on the timestamped pose ring and the solver above:

- **Runtime latency re-prediction** — the driver shifts the lighthouse devices'
  prediction time by the solved inter-system offset, so vrserver's own predictor
  aligns the two timelines during live motion, not just at calibration time.
- **Universe-jump compensation** — pose discontinuities inconsistent with the
  device's reported velocity (headset recenter / SLAM re-localization) are detected
  and the inverse delta is folded into the calibration instantly, so a recenter no
  longer breaks the alignment.
- **Drift detection** — alignment staleness is scored from calibration age plus
  stationary-slide and tracking-loss evidence, shown in the overlay, and raised as
  a one-shot notification instead of letting the alignment degrade silently.
- **Spatial correction field** — multi-point calibration interpolated by each
  device's own position (Gaussian RBF blending in the driver), correcting SLAM map
  deformation across the room instead of pretending one rigid transform fits
  everywhere.
- **Continuous calibration** — with a spare lighthouse tracker mounted firmly on the
  headset, a background loop keeps the alignment maintained during play. A
  head-referenced calibration learns the mount offset (with a rigidity gate), after
  which every time-aligned HMD+tracker pose pair directly measures the universe
  transform with no motion required. Small yaw+translation corrections are
  auto-applied and slewed sub-perceptually by the driver; large or tilted deviations
  (a bumped mount, a tracking fault) freeze auto-apply and notify instead. The
  mounted tracker can be hidden from games so full-body setups never mistake it for
  a body tracker. Optional (off by default): online re-estimation of the
  inter-system time offset from the same rigid pair.

Not planned: trackerless continuous alignment (without a rigid cross-universe pair
there is nothing sound to measure during play).

## Reporting problems

Every calibration, correction, freeze, and jump compensation is logged to
`%LOCALAPPDATA%\QuestCalibrator\QuestCalibrator.log` (the previous session is kept
as `QuestCalibrator.prev.log`; nothing older accumulates). Attach both files to a
bug report — they carry the timeline and the numbers behind whatever the overlay
decided to do.

## Building

Visual Studio 2022 build tools (v143, Windows 10 SDK). Build dependencies are
vendored, so no package restore is required. The checked-in components and their
available notice files are recorded in [vendored dependencies](docs/vendored-dependencies.md).

```
MSBuild QuestCalibrator.sln /p:Configuration=Release /p:Platform=x64
```

Solver tests (build and run; exit code = failed scenarios):

```
MSBuild Tests\SolverTests.vcxproj /p:Configuration=Release /p:Platform=x64
Tests\x64\Release\SolverTests.exe
```

The deterministic harness includes fixed regression scenarios plus randomized
property trials over general rotations, translations, rigid mount transforms,
both latency signs, sample rates, irregular sample timing, scale, noise, and
outliers.
It also tests interpolation/gating/rejection contracts and compares the exact
driver pose-transform path against an independent Eigen oracle, then stresses
the actual named shared-memory pose ring with concurrent publishers.
For a longer deterministic campaign, pass `--property-trials N` and optionally
`--property-seed N` to `SolverTests.exe`; the default validation uses 64 trials.

Repository-aware validation is configured through `cpp-validation.json`:

```powershell
# Fast: build and scan for substantial copied C/C++ blocks
tools\validate-cpp.ps1 -Mode Build
tools\validate-cpp.ps1 -Mode Duplicates

# Deep: rebuild every translation unit under Clang-Tidy
tools\validate-cpp.ps1 -Mode Analyze -All
```

Fast mode runs the evaluated MSVC solution build, executes the deterministic
solver/property harness, and provides conservative C++ clone detection. Deep
mode additionally uses Visual Studio's integrated Clang-Tidy, then executes the
same harness, so every translation unit is analyzed with its real MSVC defines,
include paths, PCH, SDK, and per-file options. Build or test failures block
changed files; Clang-Tidy and clone findings are advisory pending human review.
Thresholds, solution/configuration, and test executable live in
`cpp-validation.json`; the implementation is `tools/validate-cpp.ps1`.

`compile_flags.txt` contains only target, define, and repository-relative include
flags for clangd. It intentionally does not pin one developer's Visual Studio or
Windows SDK directories. Let clangd discover the installed MSVC toolchain, or set
its `--query-driver` option locally when discovery is unavailable; do not commit
machine-specific paths. MSBuild project evaluation remains authoritative.

Windows CI runs the Release build, solver tests, and duplicate scan on pushes and
pull requests. A whole-project Clang-Tidy rebuild can also be requested manually
and runs on the weekly schedule. Clang-Tidy and clone findings remain review
signals; configuration, build, or test failures fail the workflow.

A from-source setup is manual — copy `Driver\01questcalibrator` into SteamVR's `drivers`
folder, put the built `driver_01questcalibrator.dll` in its `bin\win64`, and run the
overlay exe (with `openvr_api.dll`, `manifest.vrmanifest`, and `icon.png` beside it)
from a folder of your choice.

## How the math works

The two-stage hand-eye solve is inherited from upstream — see
[math.pdf](https://github.com/pushrax/OpenVR-SpaceCalibrator/blob/master/math.pdf) for the
derivation, **together with [math errata](docs/math-errata.md)**: two steps of the pdf's
algebra are wrong as written (the mount offset composes on the wrong side, and eq. 6 is
really a conjugation), and the errata also states the observability, latency, and scale
properties the implementation depends on. Rotation comes from paired delta-rotation axes
(the rigid mount offset cancels under conjugation) via Kabsch; translation is a linear
least-squares over sample pairs. This fork keeps that core and wraps it in the time
alignment, weighting, and validation described above.

## License

Source-available: build and modify it for your own use; redistribution needs
permission first, and selling it isn't allowed — see `LICENSE` for the exact terms. Portions inherited from OpenVR-SpaceCalibrator,
Copyright (c) 2020 Justin Li (pushrax), remain under their original MIT License
(included in `LICENSE`).
Several solver-quality ideas (outlier rejection, axis-variance conditioning,
raw-driver-pose sampling) were inspired by the
[hyblocker fork](https://github.com/hyblocker/OpenVR-SpaceCalibrator) and
reimplemented from scratch; no code from that fork is included.
MinHook is vendored under its BSD-2-Clause license (`lib/MinHook/LICENSE`).

Maintainers should follow the [release provenance checklist](docs/releasing.md)
before creating a fork release or producing the private store package.
