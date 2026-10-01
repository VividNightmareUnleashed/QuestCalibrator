# QuestCalibrator — install packages

Two distribution paths live here:

- `build-package.ps1` → the **zip package** (`Install.ps1` / `Uninstall.ps1` plus
  the build outputs). This is the primary one; unsigned NSIS stubs trip
  Defender's Wacatac heuristic.
- `installer.nsi` → the legacy **NSIS installer** ("Automatic Setup").

There is also a **test channel**: `build-test-package.ps1` stages the same
package into `test-out\` (never `out\`), named
`QuestCalibrator-TEST-<version>-<commit>-src<source hash>-<timestamp>.zip` — so
several test builds of unpushed/uncommitted trees can coexist, none of them can
clobber (or be mistaken for) a GitHub Release artifact, and the zip's name records
exactly which tracked/non-ignored source state it was built from. Every test
package performs a full rebuild and includes `BUILD-INFO.txt` with the complete
source-state SHA-256, commit, timestamp, and built-binary hashes.
Install/uninstall works identically to the release package; only the packaging
differs.

The NSIS executable stages the same scripts and payload in its private temporary
directory and runs `Install.ps1 -Unattended` with 64-bit PowerShell. It has no
separate deletion implementation or generated uninstaller. Close Steam and remove
conflicting software first; use the zip installer for interactive optional-module
selection. `Install.ps1` still removes old NSIS installations before upgrading,
including a registered custom location whose final directory is QuestCalibrator.

`build-package.ps1` reads the version from the built `QuestCalibrator.exe`'s
version resource, which comes from `common/Version.h`. That header is the single
source of truth — `installer.nsi` carries a hand-kept copy in
`!define QUESTCAL_VERSION`, so bump both.

## Calling the app from a script

`QuestCalibrator.exe` is a **GUI-subsystem binary**. Two consequences:

- PowerShell's `&` does not wait for it and `$LASTEXITCODE` is meaningless.
  Use `Start-Process -Wait -PassThru` and read `.ExitCode`.
- Pass `-noui` alongside `-installmanifest` / `-removemanifest` /
  `-activatemultipledrivers` / `-openvrpath`. Without it the app reports its
  result in a message box (which is what a manual installer needs) and a
  scripted install would block on it.

Always set the working directory to the install folder as well. The app resolves
`manifest.vrmanifest` next to its own executable now, but NSIS's `SetOutPath`
convention is still worth keeping.

**Steam must be fully closed** before install/uninstall — `steam.exe` itself
locks OpenVR driver DLLs, not just the VR processes.

The scripts here are tracked; `out\` and `test-out\` are gitignored. Official
packages are built by the release workflow, or locally by `release.ps1`, and
published as draft releases (see `docs/releasing.md`). `virustotal-scan.ps1`
scans a package on VirusTotal for the release notes.
