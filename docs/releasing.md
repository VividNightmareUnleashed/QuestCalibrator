# Fork release and package provenance

The inherited `v*` tags describe OpenVR-SpaceCalibrator releases. QuestCalibrator
tags must therefore use the unambiguous form
`questcalibrator-vMAJOR.MINOR.PATCH` (for example,
`questcalibrator-v1.0.1`). Do not retag or reuse an inherited version tag.

Releases are published in this repository,
[VividNightmareUnleashed/QuestCalibrator](https://github.com/VividNightmareUnleashed/QuestCalibrator).
Installed copies look for updates there, so it must keep that exact name and stay
public.

Pushing a `questcalibrator-v*` tag runs `.github/workflows/release.yml`. It stops at
once in any other repository (a fork, or this one under another name). Otherwise, on
a clean Windows runner, it checks out the tag and the VirtualQuest commit it pins,
builds with the full solver suite (including the VirtualQuest scenarios and the
formal-model links), and in parallel jobs replays the pose hub traces through their
TLA+ model, runs the duplicate scan as an advisory gate, and runs Clang-Tidy over every
translation unit (`clang-tidy.yml`, where any first-party finding stops the release).
It checks the version against the tag, packages with `install\build-package.ps1`, scans
with `install\virustotal-scan.ps1`, creates a **draft** release with the zip, its
`.sha256` and notes carrying the hash and the VirusTotal table, and then runs the
install test on that draft. It refuses to run if the tag already has a release. The
release is written with the workflow's own token; its secrets are
`VIRTUALQUEST_DEPLOY_KEY` (a read-only deploy key on VirtualQuest; required, so a
release is never tested on less than a local build) and `VT_API_KEY` (without it the
scan is skipped and the notes say so). A run that failed before it created the draft
can be repeated for its tag from the Actions tab (`workflow_dispatch`); once the tag
has a draft the check refuses, and the way on is a new version.

`install\release.ps1` is the local fallback when Actions is unavailable, run from the
tagged commit with PowerShell 7 and your own `gh` login. It builds with the full solver
suite, runs Clang-Tidy over every translation unit (skipped by `-DryRun`), requires the
complete formal evidence described below, packages, scans and creates the draft. It
does not replay the hub traces itself (the formal evidence carries them), run the
advisory duplicate scan or run the install test. To keep the tag
push from starting the workflow, put `[skip release]` in the message of the commit
the tag points to (the version bump). It checks that the working tree is clean and
that the tag is at HEAD and pushed; `-DryRun` stops before creating the release. Its
VirusTotal key comes from `$env:VT_API_KEY` or the git-ignored `.env` at the
repository root. Package output in `install/out/` and `install/test-out/` is never
committed.

Both draft-release paths now require complete formal assurance for the exact
QuestCalibrator commit and its VirtualQuest gitlink. The hosted workflow verifies
locally collected Linux proof evidence and collects fresh Windows hub traces,
and refuses a draft when any named check or source/tool provenance is missing.
For a local release, assemble and verify the same evidence as described in
[`formal-now.md`](formal-now.md), then pass
`-FormalEvidence /path/to/complete.json` to `install\release.ps1`. A focused Now
report or a Linux-only record cannot satisfy this gate.

VirtualQuest-specific checks V01–V09 and the private core run locally. Normal
QuestCalibrator validation retains its parallel Windows, static-analysis and four
numeric proof jobs. Separate Linux jobs run the twelve Now obligations and the
23 public extension obligations. The public extension compiles without the
VirtualQuest simulation implementation; its six private C++ selections are
explicitly refused. The existing private checkout supplies shared harnesses and
the generated persistence table. Private capture/smoother/binary checks are not
part of ordinary push validation.
The numeric jobs use `-PublicInputValidation`; the four simulator guard harnesses
are checked in the complete local numeric run instead.

Before pushing a release tag, run the local suites for its exact committed source
pair, assemble Linux assurance, and publish the verified metadata:

```bash
python tools/assemble-assurance.py --contracts /tmp/now/result.json --extension /tmp/extension/result.json --binary /tmp/binary/result.json --core /tmp/core-1.json /tmp/core-2.json --numeric /tmp/numeric.json --linux-only --output /tmp/assurance-linux.json
python tools/local-assurance.py --evidence /tmp/assurance-linux.json --publish
```

This uses a private VirtualQuest Git note at
`refs/notes/formal-assurance/<QuestCalibrator commit>`, keyed by the VirtualQuest
commit. It creates no branch and changes neither source tree nor commit identity.
An existing valid record for the pair is reused. Only hashes, tool versions,
named outcomes and assertion metadata are retained; source snippets, local
paths, execution logs and binaries are excluded. The existing read-only deploy
key retrieves the note for release CI, which validates every required check and
both source identities. Missing or stale local evidence stops the release.
The local Windows release path still consumes `-FormalEvidence` directly.

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
- Run the Release build and solver harness locally, which catches a broken version
  edit and gives the scenario count for the tag message:

  ```powershell
  tools\validate-cpp.ps1 -Mode Build
  ```

- Push `alpha` and wait for its validation workflow (build, harness, hub trace
  replay, duplicate scan, input-validation proofs and Clang-Tidy) on the exact
  release commit. Review every advisory clone it reports.
- Run the private VirtualQuest suite locally after committing its changes:

  ```powershell
  pwsh -NoProfile -File VirtualQuest/formal/validate-local.ps1 -Setup
  ```

  Keep the successful `.local-validation/run-*/result.json` and shard logs as the
  local release record. Its `commit` must match the pinned VirtualQuest commit,
  `fullSuite` and `sourcesUnchanged` must be true, and every shard must pass without
  skipped checks. This covers TLC, Lean and GenMC; the public workflow covers
  ESBMC, Gappa and trace replay. Both gates must pass before pushing the tag.
  The private hosted workflow is manual-only and is not required when this local
  record passes. See `VirtualQuest/formal/README.md` for tool setup and resources.

## Remote validation setup

The local private-suite entry point also supports a Linux runner with PowerShell 7.
Keep `VirtualQuest/formal/validate-local.ps1`, `check.ps1`, all model and proof
sources, both formal Dockerfiles and `.github/workflows/formal.yml` in Git.
The manual hosted workflow documents the existing tool setup as a fallback.

Provision Java 11 or newer, PowerShell 7, Git, elan and a working Linux Docker
daemon. Give the runner read access to the private VirtualQuest repository through
its secret store, then initialize the exact submodule commit. Run the same command
shown above with `-Setup`; it verifies TLC's pinned hash, selects the pinned Lean
toolchain and builds GenMC from the committed Dockerfile. Choose `-Shards` and
`-HeapGiB` to fit the runner's memory. A host that cannot run Docker needs a
container-capable executor for GenMC.

`.local-validation/` is disposable: tool downloads can be cached, while each run's
result JSON and shard logs should be collected as job artifacts. Credentials,
Docker Desktop settings, local socket repairs and machine-specific paths are not
inputs to this workflow and must not be committed.

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
  `<zip-name>.virustotal.md`, which the release notes carry. Below the table it
  names every detection (file, engine, label) and says what a model or heuristic
  label means and that Microsoft's engine is Defender's. Investigate any detection
  before publishing, and add what you found (and whether you reported it to the
  vendor) under that list.
- Test install, SteamVR startup and handshake, calibration, upgrade, and uninstall
  on a clean supported Windows environment, using the package from the draft.
  Confirm the original Space Calibrator driver is disabled so transforms are not
  applied twice.
- If artifacts are signed, verify the signatures on the packaged files and record
  their signer and timestamp details.

## Publish the GitHub Release

- Open the draft the release workflow created. Replace the **Changes** section
  with the user-visible changes; it starts from the tag message. Keep the SHA-256
  and VirusTotal sections as generated.
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

The overlay's opt-in updater reads the GitHub Releases API of the
`VividNightmareUnleashed/QuestCalibrator` repository without a token.
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

The extension record is produced by `tools/verify-inventory-extension.py`;
pass `--extension /path/to/extension/result.json` when assembling assurance.
The public runner and release verifier require all 26 compiling mutations to
reach their registered assertion messages. Unrelated runtime exceptions,
compiler errors and incomplete negative-control records cannot satisfy the gate.
The four capture mutations must fail their intended named tests and assertions.
Produce the Now record with `tools/verify-now-contracts.py`; its nine mutations
also require their registered assertions. Use fresh output directories for both
public runners. The private runners remain supplementary conformance commands.
V09 is also required: install `VirtualQuest/formal/binary-requirements.txt`, run
`tools/verify-binary-correspondence.py --source-root . --output-dir /tmp/binary`,
and pass `--binary /tmp/binary/result.json` to the assembler. Its ten registered
failure controls and 45 acceptance fixtures bind the selected original-instruction
comparisons to the exact supplied binary pins and source pair.
Its bounded scopes and platform premises are listed in
`VirtualQuest/formal/inventory-assumptions.json`.
