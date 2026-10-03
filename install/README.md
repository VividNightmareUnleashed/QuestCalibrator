# QuestCalibrator install packages

`build-package.ps1` builds the **zip package**: `Install.ps1` and
`Uninstall.ps1` plus the build outputs. It is the only distribution; the earlier
NSIS installer was dropped, since unsigned NSIS stubs trip Defender's Wacatac
heuristic.

There is also a **test channel**: `build-test-package.ps1` stages the same
package into `test-out\` (never `out\`), named
`QuestCalibrator-TEST-<version>-<commit>-src<source hash>-<timestamp>.zip`, so
several test builds of unpushed or uncommitted trees can coexist, none of them
can clobber (or be mistaken for) a GitHub Release artifact, and the zip's name
records exactly which tracked, non-ignored source state it was built from. Every
test package performs a full rebuild and includes `BUILD-INFO.txt` with the
complete source-state SHA-256, commit, timestamp, and built-binary hashes.
Install and uninstall work identically to the release package; only the
packaging differs.

`Install.ps1` still removes installations made by the old NSIS installer before
upgrading, including a registered custom location whose final directory is
QuestCalibrator.

`build-package.ps1` reads the version from the built `QuestCalibrator.exe`'s
version resource, which comes from `common/Version.h`, the single source of
truth.

## Calling the app from a script

`QuestCalibrator.exe` is a **GUI-subsystem binary**. Two consequences:

- PowerShell's `&` does not wait for it and `$LASTEXITCODE` is meaningless.
  Use `Start-Process -Wait -PassThru` and read `.ExitCode`.
- Pass `-noui` alongside `-installmanifest` / `-removemanifest` /
  `-activatemultipledrivers` / `-openvrpath`. Without it the app reports its
  result in a message box (which is what a manual installer needs) and a
  scripted install would block on it.

Always set the working directory to the install folder as well. The app resolves
`manifest.vrmanifest` next to its own executable, but the installer scripts keep
to that convention.

**Steam must be fully closed** before install or uninstall: `steam.exe` itself
locks OpenVR driver DLLs, not just the VR processes.

The scripts here are tracked; `out\` and `test-out\` are gitignored. Official
packages are built by the release workflow, or locally by `release.ps1`, and
published as draft releases (see `docs/releasing.md`). `virustotal-scan.ps1`
scans a package on VirusTotal for the release notes.
