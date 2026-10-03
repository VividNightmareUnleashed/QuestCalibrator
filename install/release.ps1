#requires -Version 7
# Builds, packages and scans a pushed questcalibrator-v* tag, then creates a
# DRAFT release for it with your own gh login. It holds the release to what the
# release workflow does: the full solver suite with VirtualQuest at the commit
# the tag pins, Clang-Tidy over every translation unit, and the same packaging
# (install\package-release.ps1). Nothing reaches users until the draft is
# tested and published by hand.
#
#   .\install\release.ps1 -FormalEvidence <complete.json>   # the tag at HEAD
#   .\install\release.ps1 -DryRun                            # build, package and scan only
#
# The VirusTotal key comes from $env:VT_API_KEY, or from VT_API_KEY or
# VIRUSTOTAL_API_KEY in the git-ignored .env at the repository root.
[CmdletBinding()]
param(
    [string]$Tag = '',
    [string]$FormalEvidence = '',
    # Stop before creating the release.
    [switch]$DryRun,
    [switch]$SkipScan,
    # Markdown for the top of the release notes (title line, changes, limits,
    # validation). The generated Download and VirusTotal sections follow it.
    # Without it the notes start with a Changes section taken from the tag message.
    [string]$NotesFile = ''
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
# Installed copies look for updates only here.
$releaseRepo = 'VividNightmareUnleashed/QuestCalibrator'

function Invoke-Native {
    param([string]$What, [scriptblock]$Command)
    $output = & $Command
    if ($LASTEXITCODE -ne 0) { throw "$What failed." }
    $output
}

Push-Location $repoRoot
try {
    # --- Preflight -------------------------------------------------------------
    Invoke-Native 'gh auth status' { gh auth status 2>&1 } | Out-Null

    $dirty = Invoke-Native 'git status' { git status --porcelain }
    if ($dirty) { throw "The working tree has uncommitted changes:`n$($dirty -join "`n")" }

    $head = Invoke-Native 'git rev-parse' { git rev-parse HEAD }
    if (-not $Tag) {
        $Tag = @(git tag --points-at HEAD --list 'questcalibrator-v*') | Select-Object -First 1
        if (-not $Tag) { throw 'HEAD has no questcalibrator-v* tag. Tag it, push the tag, and run again.' }
    }
    $tagCommit = Invoke-Native "Resolving $Tag" { git rev-parse "$Tag^{commit}" }
    if ($tagCommit -ne $head) { throw "$Tag points at $tagCommit, but HEAD is $head. Check out the tag first." }

    # The workflow builds with the VirtualQuest commit the tag pins, so a local
    # release does too: without it the harness leaves those scenarios out.
    $pinned = Invoke-Native 'Reading the VirtualQuest pin' { git rev-parse HEAD:VirtualQuest }
    # An uninitialized submodule is an empty folder, where git would answer
    # for this repository instead.
    if (-not (Test-Path -LiteralPath (Join-Path $repoRoot 'VirtualQuest\.git'))) {
        throw 'VirtualQuest is not checked out. Run: git submodule update --init VirtualQuest'
    }
    $checkedOut = "$(git -C VirtualQuest rev-parse HEAD 2>$null)".Trim()
    if ($checkedOut -ne $pinned) {
        throw "VirtualQuest is at '$checkedOut', not $pinned, which $Tag pins. Run: git submodule update --init VirtualQuest"
    }
    if (git -C VirtualQuest status --porcelain) { throw 'VirtualQuest has uncommitted changes.' }

    $remote = @(git ls-remote origin "refs/tags/$Tag" "refs/tags/$Tag^{}") | ForEach-Object { ($_ -split '\s+')[0] }
    if ($remote -notcontains $head) {
        if (-not $DryRun) { throw "$Tag is not on origin yet. Push it first: git push origin $Tag" }
        Write-Host "$Tag is not on origin yet; fine for a dry run." -ForegroundColor Yellow
    }

    if (-not $DryRun) {
        gh release view $Tag --repo $releaseRepo --json tagName 2>$null | Out-Null
        if ($LASTEXITCODE -eq 0) {
            throw "$releaseRepo already has a release for $Tag. Never replace a release; tag a new version."
        }
        $global:LASTEXITCODE = 0
    }

    if (-not $DryRun) {
        if (-not $FormalEvidence) { throw 'Supply -FormalEvidence with the complete exact-pair assurance result.' }
        & python tools/verify-assurance.py --evidence $FormalEvidence
        if ($LASTEXITCODE -ne 0) { throw 'Formal release evidence is incomplete or stale.' }
    }

    # --- Build and test --------------------------------------------------------
    Write-Host "Building $Tag with the full solver suite" -ForegroundColor Cyan
    powershell -NoProfile -ExecutionPolicy Bypass -File tools\validate-cpp.ps1 -Mode Build | Tee-Object -Variable log | Out-Host
    if ($LASTEXITCODE -ne 0) { throw 'The build or the solver tests failed.' }
    $passed = [regex]::Match(($log -join "`n"), 'Solver tests passed: (\d+) scenario')
    if (-not $passed.Success) { throw 'The solver suite reported no scenario count.' }

    # The hosted release drafts only after Clang-Tidy has passed over every
    # translation unit (clang-tidy.yml); so does this one. A dry run skips it.
    if (-not $DryRun) {
        Write-Host 'Analyzing every translation unit with Clang-Tidy' -ForegroundColor Cyan
        powershell -NoProfile -ExecutionPolicy Bypass -File tools\validate-cpp.ps1 -Mode Analyze -All
        if ($LASTEXITCODE -ne 0) { throw 'Clang-Tidy reported a first-party finding or did not analyze every project.' }
    }

    # --- Package, scan and write the notes --------------------------------------
    if (-not $SkipScan -and -not $env:VT_API_KEY) {
        $envFile = Join-Path $repoRoot '.env'
        if (Test-Path -LiteralPath $envFile) {
            $line = Get-Content -LiteralPath $envFile |
                Where-Object { $_ -match '^\s*(export\s+)?(VT_API_KEY|VIRUSTOTAL_API_KEY)\s*=' } |
                Select-Object -First 1
            if ($line) { $env:VT_API_KEY = ($line -replace '^[^=]*=', '').Trim().Trim('"', "'") }
        }
        if (-not $env:VT_API_KEY) { throw 'No VirusTotal key. Set $env:VT_API_KEY or add it to .env, or pass -SkipScan.' }
    }
    $release = & install\package-release.ps1 -Tag $Tag -Scenarios ([int]$passed.Groups[1].Value) `
        -NotesFile $NotesFile -SkipScan:$SkipScan -BuildInfo @('built_by=install\release.ps1')

    if ($DryRun) {
        Write-Host ''
        Write-Host "Dry run: $($release.Zip)" -ForegroundColor Green
        Write-Host "Release notes: $($release.Notes)" -ForegroundColor Green
        return
    }

    # --verify-tag: without it gh would make the tag itself, on the default
    # branch, if the release repository did not have it.
    $arguments = @(
        'release', 'create', $Tag, $release.Zip, "$($release.Zip).sha256",
        '--repo', $releaseRepo,
        '--verify-tag',
        '--draft',
        '--title', "QuestCalibrator $($release.Version)",
        '--notes-file', $release.Notes
    )
    if ($release.Prerelease) { $arguments += '--prerelease' }
    $url = Invoke-Native 'Creating the draft release' { gh @arguments }

    Write-Host ''
    Write-Host "Draft release: $url" -ForegroundColor Green
    Write-Host 'Write the changes, run the install test, then publish it.'
} finally {
    Pop-Location
}
