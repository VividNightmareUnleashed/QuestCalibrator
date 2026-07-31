# Fork release and package provenance

The inherited `v*` tags describe OpenVR-SpaceCalibrator releases. QuestCalibrator
tags must therefore use the unambiguous form
`questcalibrator-vMAJOR.MINOR.PATCH` (for example,
`questcalibrator-v1.0.1`). Do not retag or reuse an inherited version tag.

The public repository intentionally ignores `install/`. Official packages are made
with the private/local packaging tooling and distributed through the authorized
store listing; installer sources or outputs must not be added to this repository.

## Source preflight

- Confirm the release is authorized by the QuestCalibrator copyright holder.
- Start from a clean `quest` worktree and fetch both `origin` and `upstream`.
- Review `git status`, `git log upstream/master..HEAD`, and
  `git rev-list --left-right --count upstream/master...HEAD`. Record the upstream
  base commit in the release notes.
- Set `common/Version.h` to the intended release version and confirm the overlay and
  driver version resources use it.
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

## Private package and provenance record

- Build the package from the exact tagged source commit using the private/local
  installer tooling. Record the installer script revision or SHA-256 because that
  tooling is intentionally outside Git history.
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
  their signer and timestamp details. Upload only the exact hashed artifact to the
  authorized store listing and record the resulting listing/release identifier.
