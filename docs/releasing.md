# Fork release and package provenance

The inherited `v*` tags describe OpenVR-SpaceCalibrator releases. QuestCalibrator
tags must therefore use the unambiguous form
`questcalibrator-vMAJOR.MINOR.PATCH` (for example,
`questcalibrator-v1.0.1`). Do not retag or reuse an inherited version tag.

This repository is private. Releases are published to the public repository
[VividNightmareUnleashed/QuestCalibrator](https://github.com/VividNightmareUnleashed/QuestCalibrator),
which holds only `public/README.md` (as its README), `LICENSE`,
`THIRD-PARTY-NOTICES.txt`, the releases and the issue tracker. Installed copies
look for updates there, so that repository must keep that exact name and stay
public.

Pushing a `questcalibrator-v*` tag runs `.github/workflows/release.yml` on a clean
Windows runner. It checks out the tag and the VirtualQuest commit it pins, builds
with the full solver suite (including the VirtualQuest scenarios and the formal-model
links), replays the pose hub traces through their TLA+ model, runs the duplicate scan
as an advisory gate (Clang-Tidy stays local, in the preflight below), checks the version
against the tag, packages with `install\build-package.ps1`, scans with
`install\virustotal-scan.ps1`, syncs the public repository's files, creates a
**draft** release there with the zip, its `.sha256` and notes carrying the hash and
the VirusTotal table, and then runs the install test on that draft. It refuses to run
if the public repository already has a release for the tag. Its secrets:
`VIRTUALQUEST_DEPLOY_KEY` (a read-only deploy key on VirtualQuest; required, so a
release is never tested on less than a local build), `PUBLIC_RELEASE_TOKEN` (a
fine-grained token for the public repository only, Contents read and write) and
`VT_API_KEY` (without it the scan is skipped and the notes say so). A failed run can
be repeated for an existing tag from the Actions tab (`workflow_dispatch`).

`install\release.ps1` does the same locally, from the tagged commit with PowerShell 7,
with your own `gh` login: the fallback when Actions is unavailable, or to save the
Actions minutes. To keep the tag push from starting the workflow, put `[skip release]`
in the message of the commit the tag points to (the version bump). It checks that the
working tree is clean and that the tag is at HEAD and pushed; `-DryRun` stops before
the public repository. Its VirusTotal key comes from `$env:VT_API_KEY` or the
git-ignored `.env` at the repository root. Package output in `install/out/` and
`install/test-out/` is never committed.

## Source preflight

- Confirm the release is authorized by the QuestCalibrator copyright holder.
- Start from a clean `alpha` checkout and fetch both `origin` and `upstream`.
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
  not fail validation on their own.

## Commit, push, and tag

- Commit the version and release-note changes, then push `alpha`. Verify the exact
  release commit is visible on `origin`; do not tag an unpushed-only commit.
- Create an annotated `questcalibrator-vMAJOR.MINOR.PATCH` tag on that reviewed
  commit and include the version, validation result, and upstream base in its
  message.
- Push the tag explicitly and verify that the remote tag resolves to the recorded
  commit.
  Never use a force-updated release tag; issue a new patch version instead.

## Package and provenance record

- The release workflow runs on the pushed tag. The package's `BUILD-INFO.txt`
  records the tag, commit, the VirtualQuest commit, the scenario count, UTC build
  time, runner image, Visual Studio version, the packaging script's SHA-256 and the
  workflow run. The run keeps the zip, its `.sha256` and the VirusTotal table as an
  artifact for 90 days; copy them into the private release record. The default package name is
  `QuestCalibrator-MAJOR.MINOR.PATCH.zip`, from the executable's version resource.
- `build-package.ps1` writes the SHA-256 of every packaged file to `SHA256SUMS.txt`
  inside the package, and the zip's own hash to `<zip>.sha256` beside it.
- `virustotal-scan.ps1` hashes the executables, DLLs and scripts inside the zip, plus
  the zip itself, uploads any VirusTotal hasn't seen, and writes
  `<zip-name>.virustotal.md`, which `release.ps1` puts in the release notes.
  Investigate any detection before publishing.
- Test install, SteamVR startup and handshake, calibration, upgrade, and uninstall
  on a clean supported Windows environment, using the package from the draft.
  Confirm the original Space Calibrator driver is disabled so transforms are not
  applied twice.
- If artifacts are signed, verify the signatures on the packaged files and record
  their signer and timestamp details.

## Publish the GitHub Release

- Open the draft the release workflow created in the public repository. Replace
  the **Changes** section with the user-visible changes; it starts from the tag
  message. Keep the SHA-256 and VirusTotal sections as generated.
- The draft carries only the package ZIP and its `.sha256`. Never add, replace or
  delete assets by hand; if the package must change, tag a new version.
- Publish only when the install test passed. The release workflow runs it on the
  draft; run it again by hand to test against another earlier release, or after a
  local `release.ps1`:

  ```powershell
  gh workflow run install-test.yml --ref alpha -f tag=questcalibrator-vMAJOR.MINOR.PATCH
  ```

  It installs the attached ZIP on a clean Windows runner against the fake OpenVR
  runtime in `tools/fake-openvr-runtime`, then covers reinstall, removal of leftover
  Space Calibrator drivers, an upgrade from the newest published release (or the
  `previous` input), and uninstall. It checks files, the driver, registry, shortcut,
  and what the overlay registered with the runtime. It does not replace the SteamVR,
  handshake, and calibration checks above. Record the run URL, then publish the draft
  and record the final GitHub Release URL.
- Download the ZIP from the published release and verify its SHA-256 against the
  release notes once more.

### Automatic-update contract

The overlay's opt-in updater reads the GitHub Releases API of the public
`VividNightmareUnleashed/QuestCalibrator` repository without a token.
Keep these names exact or the release deliberately fails closed:

- Stable tag: `questcalibrator-vMAJOR.MINOR.PATCH` with no suffix.
- Package asset: `QuestCalibrator-MAJOR.MINOR.PATCH.zip`.
- Exactly one asset with that name, containing the normal single-folder package with
  `Install.ps1`, `Uninstall.ps1`, `app/QuestCalibrator.exe`, and the driver DLL.
- The published asset must expose GitHub's `sha256:` digest. Verify that digest against
  the provenance record before publishing.

Drafts, prereleases, inherited `v*` tags, packages without a SHA-256 digest, duplicate
canonical assets, and download URLs outside the public repository are not eligible. The
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
