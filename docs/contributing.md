# Contributing

How to build, test and check QuestCalibrator while working on it.
[How it works](how-it-works.md) explains the design, and
[releasing](releasing.md) the release process.

## Branches

Work happens on `alpha`, and prereleases are tagged there. `stable` is the
default branch and holds the last stable release; it moves only when one is
published ([releasing](releasing.md#branches)).

## Build and test

You need the Visual Studio 2022 build tools (v143) and the Windows 10 SDK.
Everything else is in `lib/`, so there is nothing to restore.

```powershell
tools\validate-cpp.ps1 -Mode Build
```

This builds the solution into `x64\Release\` (`QuestCalibrator.exe`,
`driver_01questcalibrator.dll` and `SolverTests.exe`) and runs the test harness.
First-party code builds at `/W4` with warnings as errors; the vendored code
under `lib\` is exempt. The other modes:

- `-Mode Compile` builds without running the harness.
- `-Mode Analyze -All` rebuilds everything under Clang-Tidy.
- `-Mode Duplicates` scans for copied code with jscpd.

Settings for all of them live in `cpp-validation.json`.

### The test harness

The harness has no framework. `Tests/main.cpp` calls each `Run*Scenarios()`
group in order, and `SolverTests.exe` exits with the number of failed
scenarios. A new test file needs three edits:

1. a `ClCompile` for it in `Tests/SolverTests.vcxproj`;
2. a forward declaration of its group in `Tests/main.cpp`;
3. a call to the group from `main()`.

Groups in other files take `void (*check)(const char *, bool, const char *)`.
There is no way to run a single scenario.

The harness links most overlay sources as they are, including
`CalibrationSpace.cpp` and `CalibrationDriver.cpp`, but not `Calibration.cpp`,
`Configuration.cpp`, the UI or openvr_api. `Tests/OverlayStubs.cpp` defines
what the linked code needs from those: the `CalCtx` context, the session log,
the two saves, and the two openvr_api exports behind `openvr.h`'s accessors.
It reports no runtime, so `vr::VRSystem()` and the other accessors return
null, as they do before SteamVR starts.

`cpp-validation.json` holds the exact scenario counts, with and without the
VirtualQuest scenarios, and validation fails when a run comes in below them, so
a group that stops running is caught. When you add tests, validation prints the
new count: raise the number in the same change.

`SolverTests.exe` takes `--property-trials N` and `--property-seed N` for the
randomized property trials, and `--emit-traces DIR`, which records pose hub runs
for the formal checks in the VirtualQuest submodule.

### Fuzzing

Every input QuestCalibrator does not control (the registry records, the release
feed, SteamVR's log lines, pipe requests, frame recovery) has a target in
`Tests/Fuzz/FuzzTargets.h`. The harness replays each target's seeds and seeded
mutations on every run, and `tools\fuzz.ps1` builds libFuzzer and
AddressSanitizer executables for the long coverage-guided search, which CI runs
weekly.

A new untrusted input gets a target there. Its name also goes into the matrix in
`.github/workflows/fuzz.yml` and the defaults in `tools/fuzz.ps1`; validation
fails when the three lists differ.

## Checking a screen without a headset

The overlay runs without SteamVR in preview mode, which never touches your
saved settings. Each flag opens one screen in a realistic state:

`-uipreview`, `-uipreview-many`, `-uipreview-lighthouse`, `-uipreview-guide`,
`-uipreview-guide-wait`, `-uipreview-result`, `-uipreview-frozen`,
`-uipreview-trackeroff`, `-uipreview-failed`, `-uipreview-empty`,
`-uipreview-settings`

- `-lang ja` or `-lang it` shows it in that language.
- `-i18n-missing <file>` writes, on exit, the English strings drawn without a
  translation.
- `-shot <file.png>` writes the 1200x800 overlay texture to a PNG and exits after
  thirty frames (`-frames N` to change that).

## Layout

Layout comes from Yoga, through `Overlay/UiLayout.h` (`FlexLayout`,
`FlexRect`): build a tree each frame, set styles with the `YGNodeStyleSet*` calls
named after the CSS properties, compute it, then place ImGui items and draw-list
calls at `Rect(node)`. Web defaults are on, so the CSS of a reference design maps
one to one. `BuildHeader` in `Overlay/UserInterface.cpp` is the example to copy.
`LayOutRowSlots` in `Overlay/UiWidgets.cpp` lays out the settings rows the same
way; it marks the nodes that only place text as `YGNodeTypeText`, so they round
down to a whole pixel as ImGui does when it draws text.

## Translations

Player-facing text is written in English in the code and translated as it is
drawn (`Overlay/Localization.h`). The shared widgets call `Tr` themselves; text
drawn directly with ImGui or the draw list needs `Tr`. A new or reworded string
needs its entry in every table, one UTF-8 text file per language
(`Overlay/lang/ja.txt`, `Overlay/lang/it.txt`, built in by
`Overlay/Translations.rc`). Each entry is an `en:` line with the English and a
line with the translation; the format is described at the top of each file.
printf keys become patterns, and their values use `{0}`, `{1}` and so on.

The harness fails when the tables' keys differ, when a file has a line it cannot
read, and when a literal the overlay shows has no translation: it reads the
`Overlay` sources for the strings passed to `Tr`, to the context's messages and
to the shared widgets that translate their labels. To check a screen, run with
`-lang <code> -i18n-missing <file>`. Log and diagnostics lines stay in English.

## Guidance art

The motion-guide atlases in `Overlay/assets/` are rendered from a Blender file
kept outside the repository. The frame layout in `Overlay/UiGuide.cpp` and the
decode bounds in `Overlay/UiWidgets.cpp` must match them.

## Dependencies

Third-party code is vendored in `lib/`. Adding or updating a component means
updating [vendored-dependencies.md](vendored-dependencies.md) in the same
change: its version, the pinned upstream commit, the import steps and the
notice in the release bundle.

## The VirtualQuest submodule

`VirtualQuest` is a private repository holding the simulated headset and its
scenarios. The build does not need it: without it the harness leaves those
scenarios out. With access, a new checkout or worktree needs
`git submodule update --init VirtualQuest`. Commit changes inside the submodule
first, then the updated submodule pointer here.

## Editor setup

`compile_flags.txt` is for clangd only. Never add machine-specific paths to it.
