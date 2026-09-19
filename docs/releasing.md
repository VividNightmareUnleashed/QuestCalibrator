# Fork release and package provenance

The inherited `v*` tags describe OpenVR-SpaceCalibrator releases. QuestCalibrator
tags must therefore use the unambiguous form
`questcalibrator-vMAJOR.MINOR.PATCH` (for example,
`questcalibrator-v1.0.1`). Do not retag or reuse an inherited version tag.

The public repository intentionally ignores `install/`. Official packages are made
with the private/local packaging tooling and attached as ZIP assets to GitHub Releases;
packaging sources and outputs must not be committed to the repository.

## Source preflight

- Confirm the release is authorized by the QuestCalibrator copyright holder.
- Start from a clean `quest` worktree and fetch both `origin` and `upstream`.
- Review `git status`, `git log upstream/master..HEAD`, and
  `git rev-list --left-right --count upstream/master...HEAD`. Record the upstream
  base commit in the release notes.
- Set `common/Version.h` to the intended release version and confirm the overlay and
  driver version resources use it. A final release clears
  `QUESTCAL_VERSION_PRERELEASE_LABEL` to `""` and
  `QUESTCAL_VERSION_PRERELEASE_ORDINAL` to `0`, and drops the suffix from
  `QUESTCAL_VERSION_STRING`, in the same commit. The solver harness asserts that the
  string and the numbers agree, so a half-finished edit fails validation rather than
  shipping.
- Review `docs/vendored-dependencies.md`; resolve missing or changed notice material
  before producing a distributable binary.
- Run the Release build, duplicate scan, and full project-aware analysis:

  ```powershell
  tools\validate-cpp.ps1 -Mode Build
  tools\validate-cpp.ps1 -Mode Duplicates
  tools\validate-cpp.ps1 -Mode Analyze -All
  ```

- Review every advisory clone or Clang-Tidy finding, even though those findings do
  not automatically fail CI.

## Commit, push, and tag

- Commit the version and release-note changes, then push `quest`. Verify the exact
  release commit is visible on `origin`; do not tag an unpushed-only commit.
- Create an annotated `questcalibrator-vMAJOR.MINOR.PATCH` tag on that reviewed
  commit and include the version, validation result, and upstream base in its
  message.
- Push the tag explicitly and verify that the remote tag resolves to the recorded
  commit. Never use a force-updated release tag; issue a new patch version instead.

## Local package and provenance record

- Build the package from the exact tagged source commit using the private/local
  installer tooling. Record the installer script revision or SHA-256 because that
  tooling is intentionally outside Git history.
  The default `install\build-package.ps1` command names the ZIP
  `QuestCalibrator-MAJOR.MINOR.PATCH.zip` from the executable's version resource.
  Do not override `PackageName` for stable releases; custom names are for test builds.
- Record the tag, full commit ID, MSVC toolset, Windows SDK, build command, package
  command, UTC build time, and operator in the private release record.
- Record SHA-256 hashes for the final archive/installer and the included overlay,
  driver, OpenVR runtime DLL, manifest, and license/notice bundle.
- Confirm the executable and driver file properties contain the expected version,
  and confirm the manifest resolves paths relative to the installed executable.
- Test install, SteamVR startup and handshake, calibration, upgrade, and uninstall
  on a clean supported Windows environment. Confirm the original Space Calibrator
  driver is disabled so transforms are not applied twice.
- If artifacts are signed, verify the signatures after final packaging and record
  their signer and timestamp details.

## Publish the GitHub Release

- Create a draft GitHub Release from the pushed, annotated QuestCalibrator tag. Use
  the version as the release title and include the upstream base commit, validation
  result, installation notes, and user-visible changes in the release notes.
- Attach only the exact tested and hashed package ZIP. GitHub's automatically generated
  source archives are not substitutes for the install package.
- Verify the draft's tag and attached ZIP name, size, and SHA-256 against the provenance
  record, then publish it. Record the final GitHub Release URL in that record.
- Download the ZIP from the published release and verify its SHA-256 once more. Never
  replace an asset on an existing release; publish a new patch version if an artifact
  must change.

### Automatic-update contract

The overlay's opt-in updater reads the public GitHub Releases API without a token.
Keep these names exact or the release deliberately fails closed:

- Stable tag: `questcalibrator-vMAJOR.MINOR.PATCH` with no suffix.
- Package asset: `QuestCalibrator-MAJOR.MINOR.PATCH.zip`.
- Exactly one asset with that name, containing the normal single-folder package with
  `Install.ps1`, `Uninstall.ps1`, `app/QuestCalibrator.exe`, and the driver DLL.
- The published asset must expose GitHub's `sha256:` digest. Verify that digest against
  the provenance record before publishing.

Drafts, prereleases, inherited `v*` tags, packages without a SHA-256 digest, duplicate
canonical assets, and download URLs outside this repository are not eligible. The
updater checks and downloads only after the user opts in; applying the package remains
an explicit action because Steam must be fully closed and Windows must approve the
elevated installer.

### Prerelease builds

A prerelease sets both prerelease macros in `common/Version.h` (`"alpha"` and `3` give
`1.2.0-alpha.3`) so the build reports what it actually is. Before this the alphas
declared the bare `1.2.0`, the number of the release they precede, and every alpha
build reported a version it was not.

Prereleases are their own delivery lane, not a step on the stable one. They are
installed by hand and left by hand:

- A build with a prerelease label never asks the stable feed for anything. The updater
  skips the check and reports which prerelease this is, and `SelectReleaseCandidate`
  refuses a prerelease caller outright so no later path can hand a tester a stable
  package by accident.
- Testers move to a stable release by installing it by hand. Say so in the prerelease
  notes: an alpha will not update itself when the release it precedes is published.
- Prerelease tags stay out of the stable feed as before: tag them
  `questcalibrator-vMAJOR.MINOR.PATCH-LABEL.N` and mark the GitHub Release as a
  prerelease.

Two consequences of the suffix reaching the version resources:

- The numeric `FILEVERSION` has only four integers and cannot express a prerelease, so
  `1.2.0-alpha.3` and `1.2.0` both carry `1,2,0,0`. Never distinguish a prerelease from
  its release by the numeric file version; read the `FileVersion` string, which does
  carry the suffix.
- `install\build-package.ps1` names the ZIP from that string, so a prerelease package
  is named for the prerelease. Stable releases are unaffected, and the exact
  `QuestCalibrator-MAJOR.MINOR.PATCH.zip` asset name above still applies to them.
