# Builds a TEST package for trying unpushed or uncommitted builds, kept apart
# from the GitHub Release artifacts. Same staging as build-package.ps1, but:
#   - output goes to install\test-out\ (never out\)
#   - the package name carries TEST + version + exact source-state hash + build time, so
#     several test builds (e.g. a 1.0.1 candidate and a 1.1 work tree) sit
#     side by side and none can be mistaken for an official upload.
# Run from anywhere:
#   .\install\build-test-package.ps1
# Test packages always perform a full rebuild; -Build remains accepted for
# compatibility with older invocations.
[CmdletBinding()]
param(
    [switch]$Build,
    [string]$Configuration = 'Release',
    [string]$Platform = 'x64'
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
if ($Build) {
    Write-Verbose '-Build is retained for compatibility; test packages always rebuild.'
}

$msbuild = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe'
if (-not (Test-Path $msbuild)) {
    Write-Error "MSBuild not found: $msbuild"
    exit 1
}
& $msbuild (Join-Path $repoRoot 'QuestCalibrator.sln') /t:Rebuild "/p:Configuration=$Configuration" "/p:Platform=$Platform" /m:1 /v:m /nologo
if ($LASTEXITCODE -ne 0) {
    Write-Error "Build failed."
    exit 1
}

$appExe = Join-Path $repoRoot "$Platform\$Configuration\QuestCalibrator.exe"
if (-not (Test-Path $appExe)) {
    Write-Error "Missing $appExe after rebuild."
    exit 1
}
$version = (Get-Item $appExe).VersionInfo.ProductVersion

# Hash every tracked and non-ignored untracked path by Git blob id, including a
# deletion marker. Unlike `git describe --dirty`, this identifies the exact
# dirty source state rather than only saying that some unknown change exists.
$sourcePaths = @(& git -C $repoRoot -c core.quotepath=false ls-files -co --exclude-standard --deduplicate)
if ($LASTEXITCODE -ne 0) {
    Write-Error "Unable to enumerate source state with git."
    exit 1
}
$stateLines = @(
    $sourcePaths |
        Sort-Object -Unique |
        ForEach-Object {
            $relative = ([string]$_).Trim()
            if (-not $relative) { return }
            $absolute = Join-Path $repoRoot $relative
            if (Test-Path -LiteralPath $absolute -PathType Leaf) {
                $blob = ([string](& git -C $repoRoot hash-object -- $relative)).Trim()
                if ($LASTEXITCODE -ne 0 -or -not $blob) {
                    throw "Unable to hash source file: $relative"
                }
                "$blob`t$relative"
            } else {
                "DELETED`t$relative"
            }
        }
)
$stateText = ($stateLines -join "`n") + "`n"
$sha256 = [System.Security.Cryptography.SHA256]::Create()
try {
    $sourceBytes = [Text.Encoding]::UTF8.GetBytes($stateText)
    $sourceHash = -join ($sha256.ComputeHash($sourceBytes) | ForEach-Object { $_.ToString('x2') })
} finally {
    $sha256.Dispose()
}

$head = ([string](& git -C $repoRoot rev-parse HEAD)).Trim()
if ($LASTEXITCODE -ne 0 -or -not $head) { $head = 'nogit' }
$headShort = if ($head -eq 'nogit') { $head } else { $head.Substring(0, 8) }

$stamp = Get-Date -Format 'yyyyMMdd-HHmm'
$name  = "QuestCalibrator-TEST-$version-$headShort-src$($sourceHash.Substring(0, 12))-$stamp"

$buildDir = Join-Path $repoRoot "$Platform\$Configuration"
$driverDll = Join-Path $buildDir 'driver_01questcalibrator.dll'
$openvrDll = Join-Path $repoRoot 'lib\openvr\lib\win64\openvr_api.dll'
$outDir = Join-Path $PSScriptRoot 'test-out'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$buildInfoPath = Join-Path $outDir '.BUILD-INFO.tmp'
$builtAt = (Get-Date).ToUniversalTime().ToString('o')
$info = @(
    "QuestCalibrator TEST package - never attach to a GitHub Release"
    "version=$version"
    "git_head=$head"
    "source_state_sha256=$sourceHash"
    "built_utc=$builtAt"
    "configuration=$Configuration"
    "platform=$Platform"
    "QuestCalibrator.exe_sha256=$((Get-FileHash -Algorithm SHA256 -LiteralPath $appExe).Hash.ToLowerInvariant())"
    "driver_01questcalibrator.dll_sha256=$((Get-FileHash -Algorithm SHA256 -LiteralPath $driverDll).Hash.ToLowerInvariant())"
    "openvr_api.dll_sha256=$((Get-FileHash -Algorithm SHA256 -LiteralPath $openvrDll).Hash.ToLowerInvariant())"
)
Set-Content -LiteralPath $buildInfoPath -Value $info -Encoding UTF8

try {
    & (Join-Path $PSScriptRoot 'build-package.ps1') `
        -Configuration $Configuration -Platform $Platform `
        -PackageName $name `
        -OutDir $outDir `
        -BuildInfoPath $buildInfoPath
} finally {
    Remove-Item -LiteralPath $buildInfoPath -Force -ErrorAction SilentlyContinue
}

Write-Host ""
Write-Host "TEST package (never attach to a GitHub Release): install\test-out\$name.zip" -ForegroundColor Yellow
Write-Host "Source state SHA-256: $sourceHash" -ForegroundColor Yellow
