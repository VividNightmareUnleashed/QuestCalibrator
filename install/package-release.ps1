#requires -Version 7
# Packages a full build of a questcalibrator-v* tag for its draft release, the
# same way for the release workflow and install\release.ps1: checks the built
# version against the tag and VirtualQuest against the commit the tag pins,
# records how the package was built in BUILD-INFO.txt, packages it with
# build-package.ps1, scans it on VirusTotal when $env:VT_API_KEY is set, and
# writes the release notes to install\out\notes.md. Run it after the build and
# the full solver suite passed; it returns the version, whether it is a
# prerelease, and the package and notes paths.
[CmdletBinding()]
param(
    [Parameter(Mandatory)] [string]$Tag,
    # Scenarios the full solver suite ran for this build.
    [Parameter(Mandatory)] [int]$Scenarios,
    # key=value lines for BUILD-INFO.txt beyond the common ones.
    [string[]]$BuildInfo = @(),
    # Markdown for the top of the notes (title line, changes, limits). The
    # Download and VirusTotal sections follow it. Without it the notes start
    # with a Changes section taken from the tag message.
    [string]$NotesFile = '',
    [switch]$SkipScan
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
Push-Location $repoRoot
try {
    # A checkout of the tagged commit alone can lack the annotated tag, which
    # holds the message the notes start from.
    if ($env:GITHUB_ACTIONS -eq 'true') { git fetch --quiet --force origin "refs/tags/${Tag}:refs/tags/${Tag}" }
    $commit = "$(git rev-parse HEAD)".Trim()
    if ("$(git rev-parse "$Tag^{commit}" 2>$null)".Trim() -ne $commit) { throw "$Tag is not the commit checked out ($commit)." }
    $version = (Get-Item x64\Release\QuestCalibrator.exe).VersionInfo.ProductVersion
    if ("questcalibrator-v$version" -ne $Tag) {
        throw "The build reports version $version, but the tag is $Tag. Fix common/Version.h and tag again."
    }
    # A release is never tested on less than the full suite, which needs the
    # VirtualQuest commit the tag pins.
    $pinned = "$(git rev-parse HEAD:VirtualQuest)".Trim()
    # An uninitialized submodule is an empty folder, where git would answer
    # for this repository instead.
    if (-not (Test-Path -LiteralPath (Join-Path $repoRoot 'VirtualQuest\.git'))) { throw 'VirtualQuest is not checked out.' }
    $checkedOut = "$(git -C VirtualQuest rev-parse HEAD 2>$null)".Trim()
    if ($checkedOut -ne $pinned) {
        throw "VirtualQuest is at '$checkedOut', not $pinned, which $Tag pins; a release is built with the full suite."
    }

    # --- Package ---------------------------------------------------------------
    $info = Join-Path ([IO.Path]::GetTempPath()) "QuestCalibrator-$version-BUILD-INFO.txt"
    $vs = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -property catalog_productDisplayVersion
    @(
        'QuestCalibrator release build'
        "version=$version"
        "tag=$Tag"
        "commit=$commit"
        "virtualquest=$pinned"
        "scenarios=$Scenarios"
        "build_utc=$((Get-Date).ToUniversalTime().ToString('yyyy-MM-ddTHH:mm:ssZ'))"
        "visual_studio=$vs"
        "package_script_sha256=$((Get-FileHash install\build-package.ps1 -Algorithm SHA256).Hash)"
        $BuildInfo
    ) | Set-Content -LiteralPath $info -Encoding utf8
    & install\build-package.ps1 -BuildInfoPath $info | Out-Host
    $zip = Join-Path $repoRoot "install\out\QuestCalibrator-$version.zip"
    if (-not (Test-Path -LiteralPath $zip)) { throw "Expected package $zip was not produced." }

    # --- VirusTotal ------------------------------------------------------------
    $scan = [IO.Path]::ChangeExtension($zip, '.virustotal.md')
    Remove-Item -LiteralPath $scan -ErrorAction SilentlyContinue
    if ($SkipScan) {
        Write-Host 'The VirusTotal scan was skipped; the notes say so.'
    } elseif (-not $env:VT_API_KEY) {
        Write-Host '::warning::VT_API_KEY is not set; skipping the VirusTotal scan.'
    } else {
        & install\virustotal-scan.ps1 -Package $zip | Out-Host
    }

    # --- Release notes ---------------------------------------------------------
    $zipName = Split-Path -Leaf $zip
    $hash = (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash.ToLowerInvariant()
    $message = (git tag -l --format='%(contents:body)' $Tag) -join "`n"
    $top = if ($NotesFile) {
        (Get-Content -LiteralPath $NotesFile -Raw).TrimEnd()
    } else {
        "## Changes`n`n" + $(if ($message.Trim()) { $message.Trim() } else { '_Write the changes before publishing._' })
    }
    $notes = Join-Path $repoRoot 'install\out\notes.md'
    @(
        $top
        ''
        '## Download'
        ''
        "Download ``$zipName`` below, then follow ``README-INSTALL.txt`` inside it."
        ''
        "SHA-256 of ``$zipName``:"
        ''
        '```'
        $hash
        '```'
        ''
        ('Check it with `Get-FileHash .\' + $zipName + ' -Algorithm SHA256`. ' +
            '`SHA256SUMS.txt` in the package lists every file inside it.')
        ''
        '## VirusTotal'
        ''
        $(if (Test-Path -LiteralPath $scan) { Get-Content -LiteralPath $scan } else { '_Not scanned._' })
    ) | Set-Content -LiteralPath $notes -Encoding utf8

    [pscustomobject]@{
        Version    = $version
        Prerelease = $version.Contains('-')
        Zip        = $zip
        Notes      = $notes
    }
} finally {
    Pop-Location
}
