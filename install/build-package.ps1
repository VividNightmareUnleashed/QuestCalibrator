# Builds the zip distribution package for QuestCalibrator.
# Run from anywhere after building Release|x64:
#   .\install\build-package.ps1
#
# Produces install\out\<PackageName>.zip ready for a GitHub Release. The zip has a
# single top-level folder so "Extract Here" can't scatter loose files into the
# user's Downloads.
[CmdletBinding()]
param(
    [string]$Configuration = 'Release',
    [string]$Platform = 'x64',
    [string]$PackageName = '',
    # Override the output directory (build-test-package.ps1 points this at
    # test-out\ so test builds never mingle with release artifacts).
    [string]$OutDir = '',
    # Optional provenance manifest supplied by the test-channel wrapper.
    [string]$BuildInfoPath = ''
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$outDir   = if ($OutDir) { $OutDir } else { Join-Path $PSScriptRoot 'out' }
$outDir = [IO.Path]::GetFullPath($outDir)

# --- Locate build outputs ------------------------------------------------------
$buildDir  = Join-Path $repoRoot "$Platform\$Configuration"
$appExe    = Join-Path $buildDir 'QuestCalibrator.exe'
$driverDll = Join-Path $buildDir 'driver_01questcalibrator.dll'
$openvrDll = Join-Path $repoRoot 'lib\openvr\lib\win64\openvr_api.dll'

foreach ($f in @($appExe, $driverDll, $openvrDll)) {
    if (-not (Test-Path $f)) {
        Write-Error "Missing build output: $f`nBuild the solution first ($Configuration|$Platform)."
        exit 1
    }
}

# Version comes from the binary's own resource, which is generated from
# common\Version.h - keeping one source of truth for the whole pipeline.
$version = (Get-Item $appExe).VersionInfo.ProductVersion
if (-not $version) {
    Write-Error "$appExe has no version resource. Check Overlay\QuestCalibrator.rc."
    exit 1
}
Write-Host "Packaging QuestCalibrator $version" -ForegroundColor Cyan
if (-not $PackageName) { $PackageName = "QuestCalibrator-$version" }
if ($PackageName -in @('.', '..') -or
    $PackageName.IndexOfAny([IO.Path]::GetInvalidFileNameChars()) -ge 0) {
    throw 'PackageName must be a folder name, without path separators.'
}

# --- Stage the package ---------------------------------------------------------
# stageRoot holds exactly one folder, and that folder is what gets zipped, so the
# archive always extracts into a single directory.
$stageRoot = [IO.Path]::GetFullPath((Join-Path $outDir 'stage'))
$stageDir  = Join-Path $stageRoot $PackageName
$zipPath   = Join-Path $outDir "$PackageName.zip"

if (-not $stageRoot.StartsWith($outDir.TrimEnd('\', '/') + [IO.Path]::DirectorySeparatorChar,
    [StringComparison]::OrdinalIgnoreCase)) {
    throw 'The staging directory must stay inside the package output directory.'
}
if (Test-Path -LiteralPath $stageRoot) { Remove-Item -LiteralPath $stageRoot -Recurse -Force }
New-Item -ItemType Directory -Force -Path $stageDir | Out-Null

# app/
$appStage = Join-Path $stageDir 'app'
New-Item -ItemType Directory -Force -Path $appStage | Out-Null
Copy-Item $appExe    $appStage
Copy-Item $openvrDll $appStage
Copy-Item (Join-Path $repoRoot 'Overlay\manifest.vrmanifest') $appStage
Copy-Item (Join-Path $repoRoot 'Overlay\icon.png')            $appStage

# driver/01questcalibrator/
$driverStage = Join-Path $stageDir 'driver\01questcalibrator'
New-Item -ItemType Directory -Force -Path $driverStage | Out-Null
Copy-Item (Join-Path $repoRoot 'Driver\01questcalibrator\*') -Destination $driverStage -Recurse -Force

# drop the driver dll into bin/win64 inside the staged driver folder
$driverBin = Join-Path $driverStage 'bin\win64'
New-Item -ItemType Directory -Force -Path $driverBin | Out-Null
Copy-Item $driverDll $driverBin

# scripts + license + readme at package root
Copy-Item (Join-Path $PSScriptRoot 'Install.ps1')        $stageDir
Copy-Item (Join-Path $PSScriptRoot 'Uninstall.ps1')      $stageDir
Copy-Item (Join-Path $PSScriptRoot 'README-INSTALL.txt') $stageDir
Copy-Item (Join-Path $repoRoot 'LICENSE')                $stageDir
Copy-Item (Join-Path $repoRoot 'THIRD-PARTY-NOTICES.txt') $stageDir

# standalone notices for every vendored dependency called out by the release
# checklist (header-embedded notices remain with their source distributions)
$noticesStage = Join-Path $stageDir 'THIRD-PARTY-NOTICES'
New-Item -ItemType Directory -Force -Path $noticesStage | Out-Null
Copy-Item (Join-Path $repoRoot 'lib\openvr\LICENSE') (Join-Path $noticesStage 'OpenVR-LICENSE.txt')
Get-ChildItem (Join-Path $repoRoot 'lib\Eigen\COPYING.*') -File | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $noticesStage ('Eigen-' + $_.Name + '.txt'))
}
Copy-Item (Join-Path $repoRoot 'lib\MinHook\LICENSE')                    (Join-Path $noticesStage 'MinHook-LICENSE.txt')
Copy-Item (Join-Path $repoRoot 'lib\imgui\LICENSE.txt')                  (Join-Path $noticesStage 'Dear-ImGui-LICENSE.txt')
Copy-Item (Join-Path $repoRoot 'lib\glfw\COPYING.txt')                   (Join-Path $noticesStage 'GLFW-COPYING.txt')
Copy-Item (Join-Path $repoRoot 'lib\yoga\LICENSE')                       (Join-Path $noticesStage 'Yoga-LICENSE.txt')
Copy-Item (Join-Path $PSScriptRoot 'notices\picojson-LICENSE.txt') $noticesStage
Copy-Item (Join-Path $PSScriptRoot 'notices\gl3w-LICENSE.txt') $noticesStage
Copy-Item (Join-Path $PSScriptRoot 'notices\Khronos-*-LICENSE.txt') $noticesStage
Copy-Item (Join-Path $repoRoot 'Overlay\assets\guide-credits.txt') (Join-Path $noticesStage 'Motion-model-credits.txt')
if ($BuildInfoPath) {
    if (-not (Test-Path -LiteralPath $BuildInfoPath -PathType Leaf)) {
        Write-Error "Missing build information file: $BuildInfoPath"
        exit 1
    }
    Copy-Item -LiteralPath $BuildInfoPath -Destination (Join-Path $stageDir 'BUILD-INFO.txt')
}

# --- Checksums ------------------------------------------------------------------
# SHA256SUMS.txt lists every file in the package in sha256sum format, so users can
# check what they extracted and match each hash against its VirusTotal report
# (install\virustotal-scan.ps1). The zip's own hash goes beside the zip, since a
# file can't contain its own hash.
$sumsPath = Join-Path $stageDir 'SHA256SUMS.txt'
$sums = Get-ChildItem -LiteralPath $stageDir -Recurse -File |
    Sort-Object FullName |
    ForEach-Object {
        $rel  = $_.FullName.Substring($stageDir.Length + 1).Replace('\', '/')
        $hash = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
        "$hash *$rel"
    }
[IO.File]::WriteAllLines($sumsPath, [string[]]$sums)

# --- Zip ----------------------------------------------------------------------
if (Test-Path $zipPath) { Remove-Item $zipPath -Force }
Compress-Archive -Path $stageDir -DestinationPath $zipPath -CompressionLevel Optimal
$zipHash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash.ToLowerInvariant()
[IO.File]::WriteAllText("$zipPath.sha256", "$zipHash *$(Split-Path -Leaf $zipPath)`n")

Write-Host ""
$sums | ForEach-Object { Write-Host "  $_" }
Write-Host "  $zipHash *$(Split-Path -Leaf $zipPath)"
Write-Host ""
Write-Host "Package built: $zipPath" -ForegroundColor Green
Write-Host ""
Write-Host "Contents:" -ForegroundColor Cyan
Add-Type -AssemblyName System.IO.Compression.FileSystem
$zip = [System.IO.Compression.ZipFile]::OpenRead($zipPath)
try { $zip.Entries | ForEach-Object { Write-Host "  $($_.FullName)" } } finally { $zip.Dispose() }
Write-Host ""
Write-Host "Test: extract the zip, right-click Install.ps1 -> Run with PowerShell (as admin)."

