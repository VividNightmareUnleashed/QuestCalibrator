#requires -Version 7
# Builds, packages and scans a pushed questcalibrator-v* tag, then creates a
# DRAFT release in the public releases repository with your own gh login.
# Nothing reaches users until the draft is tested and published by hand.
#
#   .\install\release.ps1              # the tag at HEAD
#   .\install\release.ps1 -DryRun      # build, package and scan only
#
# The VirusTotal key comes from $env:VT_API_KEY, or from VT_API_KEY or
# VIRUSTOTAL_API_KEY in the git-ignored .env at the repository root.
[CmdletBinding()]
param(
    [string]$Tag = '',
    # Stop before touching the public repository.
    [switch]$DryRun,
    [switch]$SkipScan,
    # Markdown for the top of the release notes (title line, changes, limits,
    # validation). The generated Download and VirusTotal sections follow it.
    # Without it the notes start with a Changes section taken from the tag message.
    [string]$NotesFile = ''
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$publicRepo = 'VividNightmareUnleashed/QuestCalibrator'

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

    $remote = @(git ls-remote origin "refs/tags/$Tag" "refs/tags/$Tag^{}") | ForEach-Object { ($_ -split '\s+')[0] }
    if ($remote -notcontains $head) {
        if (-not $DryRun) { throw "$Tag is not on origin yet. Push it first: git push origin $Tag" }
        Write-Host "$Tag is not on origin yet; fine for a dry run." -ForegroundColor Yellow
    }

    if (-not $DryRun) {
        gh release view $Tag --repo $publicRepo --json tagName 2>$null | Out-Null
        if ($LASTEXITCODE -eq 0) {
            throw "$publicRepo already has a release for $Tag. Never replace a release; tag a new version."
        }
        $global:LASTEXITCODE = 0
    }

    # --- Build and test --------------------------------------------------------
    Write-Host "Building $Tag" -ForegroundColor Cyan
    powershell -NoProfile -ExecutionPolicy Bypass -File tools\validate-cpp.ps1 -Mode Build
    if ($LASTEXITCODE -ne 0) { throw 'The build or the solver tests failed.' }

    $version = (Get-Item x64\Release\QuestCalibrator.exe).VersionInfo.ProductVersion
    if ("questcalibrator-v$version" -ne $Tag) {
        throw "The build reports version $version, but the tag is $Tag. Fix common/Version.h and tag again."
    }

    # --- Package ---------------------------------------------------------------
    $info = Join-Path ([IO.Path]::GetTempPath()) "QuestCalibrator-$version-BUILD-INFO.txt"
    $vs = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property catalog_productDisplayVersion
    @(
        'QuestCalibrator release build'
        "version=$version"
        "tag=$Tag"
        "commit=$head"
        "build_utc=$((Get-Date).ToUniversalTime().ToString('yyyy-MM-ddTHH:mm:ssZ'))"
        "visual_studio=$vs"
        "package_script_sha256=$((Get-FileHash install\build-package.ps1 -Algorithm SHA256).Hash)"
    ) | Set-Content -LiteralPath $info -Encoding utf8
    & install\build-package.ps1 -BuildInfoPath $info
    $zip = Join-Path $repoRoot "install\out\QuestCalibrator-$version.zip"
    if (-not (Test-Path -LiteralPath $zip)) { throw "Expected package $zip was not produced." }

    # --- VirusTotal ------------------------------------------------------------
    $scan = [IO.Path]::ChangeExtension($zip, '.virustotal.md')
    Remove-Item -LiteralPath $scan -ErrorAction SilentlyContinue
    if (-not $SkipScan) {
        if (-not $env:VT_API_KEY) {
            $envFile = Join-Path $repoRoot '.env'
            if (Test-Path -LiteralPath $envFile) {
                $line = Get-Content -LiteralPath $envFile |
                    Where-Object { $_ -match '^\s*(export\s+)?(VT_API_KEY|VIRUSTOTAL_API_KEY)\s*=' } |
                    Select-Object -First 1
                if ($line) { $env:VT_API_KEY = ($line -replace '^[^=]*=', '').Trim().Trim('"', "'") }
            }
        }
        if (-not $env:VT_API_KEY) { throw 'No VirusTotal key. Set $env:VT_API_KEY or add it to .env, or pass -SkipScan.' }
        & install\virustotal-scan.ps1 -Package $zip
    }

    # --- Release notes ---------------------------------------------------------
    $zipName = Split-Path -Leaf $zip
    $hash = (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash.ToLowerInvariant()
    $message = (git tag -l --format='%(contents:body)' $Tag) -join "`n"
    $notesPath = Join-Path ([IO.Path]::GetTempPath()) "QuestCalibrator-$version-notes.md"
    $top = if ($NotesFile) {
        (Get-Content -LiteralPath $NotesFile -Raw).TrimEnd()
    } else {
        "## Changes`n`n" + $(if ($message.Trim()) { $message.Trim() } else { '_Write the changes before publishing._' })
    }
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
    ) | Set-Content -LiteralPath $notesPath -Encoding utf8

    if ($DryRun) {
        Write-Host ''
        Write-Host "Dry run: $zip" -ForegroundColor Green
        Write-Host "Release notes: $notesPath" -ForegroundColor Green
        return
    }

    # --- Public repository -----------------------------------------------------
    $public = Join-Path ([IO.Path]::GetTempPath()) ("qc-public-" + [guid]::NewGuid())
    Invoke-Native 'Cloning the public repository' { gh repo clone $publicRepo $public -- --quiet --depth 1 } | Out-Null
    try {
        Copy-Item public\README.md, LICENSE, THIRD-PARTY-NOTICES.txt -Destination $public -Force
        git -C $public add README.md LICENSE THIRD-PARTY-NOTICES.txt
        git -C $public diff --cached --quiet
        if ($LASTEXITCODE -ne 0) {
            Invoke-Native 'Committing the public files' { git -C $public commit --quiet -m "docs: update for $Tag" } | Out-Null
            Invoke-Native 'Pushing the public files' { git -C $public push --quiet origin HEAD 2>&1 } | Out-Null
        }
        $global:LASTEXITCODE = 0
    } finally {
        Remove-Item -LiteralPath $public -Recurse -Force -ErrorAction SilentlyContinue
    }

    $arguments = @(
        'release', 'create', $Tag, $zip, "$zip.sha256",
        '--repo', $publicRepo,
        '--draft',
        '--title', "QuestCalibrator $version",
        '--notes-file', $notesPath
    )
    if ($version.Contains('-')) { $arguments += '--prerelease' }
    $url = Invoke-Native 'Creating the draft release' { gh @arguments }

    Write-Host ''
    Write-Host "Draft release: $url" -ForegroundColor Green
    Write-Host 'Write the changes, run the install test, then publish it.'
} finally {
    Pop-Location
}
