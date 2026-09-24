# QuestCalibrator uninstaller (script-based).
# Run elevated. Removes the app, driver, SteamVR registration, and shortcuts.
[CmdletBinding()]
param(
    [switch]$Silent,
    # Captured before elevation - see Install.ps1.
    [string]$UserLocalAppData = $env:LOCALAPPDATA
)

$ErrorActionPreference = 'Stop'

function Fail([string]$msg) {
    Write-Host ""
    Write-Host "ERROR: $msg" -ForegroundColor Red
    if (-not $Silent) { Read-Host "Press Enter to close" }
    exit 1
}

# --- Admin check --------------------------------------------------------------
$principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    if ($Silent) { Fail "Uninstall requires administrator rights." }
    Write-Host "Re-launching elevated..."
    # Use -NoExit so the elevated window stays open to show output/errors.
    # Only forward -Silent when it was actually asked for, otherwise an
    # interactive uninstall would silently swallow its own result.
    $relaunchArgs = @(
        '-NoExit', '-NoProfile', '-ExecutionPolicy', 'Bypass',
        '-File', "`"$PSCommandPath`"",
        '-UserLocalAppData', "`"$env:LOCALAPPDATA`""
    )
    if ($Silent) { $relaunchArgs += '-Silent' }
    Start-Process -FilePath "powershell.exe" -Verb RunAs -ArgumentList $relaunchArgs
    exit 0
}

# --- SteamVR must be closed ----------------------------------------------------
# steam.exe itself also locks driver DLLs, not just the VR processes
$steamProcesses = Get-Process -Name 'steam','vrserver','vrmonitor','vrcompositor' -ErrorAction SilentlyContinue
if ($steamProcesses) {
    $names = ($steamProcesses | Select-Object -ExpandProperty Name) -join ', '
    Fail "Steam is still running ($names). Please close Steam completely (check the system tray) and run this again."
}

$installDir = (Get-ItemProperty 'HKLM:\Software\QuestCalibrator\Main' -ErrorAction SilentlyContinue).'(default)'
if (-not $installDir) { $installDir = Join-Path ${env:ProgramFiles} 'QuestCalibrator' }

# --- Deregister from SteamVR ---------------------------------------------------
# GUI binary: use Start-Process -Wait so the app has actually exited before we
# delete it, and set the working directory it resolves manifest.vrmanifest from.
$appExe = Join-Path $installDir 'QuestCalibrator.exe'
if (Test-Path $appExe) {
    $proc = Start-Process -FilePath $appExe -ArgumentList @('-removemanifest', '-noui') `
        -WorkingDirectory $installDir -Wait -PassThru
    if ($proc.ExitCode -ne 0) {
        Write-Host "Warning: could not deregister from SteamVR (exit code $($proc.ExitCode)). Continuing." -ForegroundColor Yellow
    } else {
        Write-Host "Deregistered from SteamVR"
    }
}

# --- Remove driver ---------------------------------------------------------------
# Same discovery as the installer, so a damaged registry still finds the driver.
function Get-SteamLibraryPaths {
    $roots = @()
    foreach ($key in @('HKLM:\SOFTWARE\WOW6432Node\Valve\Steam', 'HKLM:\SOFTWARE\Valve\Steam')) {
        $p = (Get-ItemProperty $key -ErrorAction SilentlyContinue).InstallPath
        if ($p) { $roots += $p }
    }
    $p = (Get-ItemProperty 'HKCU:\SOFTWARE\Valve\Steam' -ErrorAction SilentlyContinue).SteamPath
    if ($p) { $roots += ($p -replace '/', '\') }
    $roots += Join-Path ${env:ProgramFiles(x86)} 'Steam'

    $libraries = @()
    foreach ($root in ($roots | Where-Object { $_ } | Select-Object -Unique)) {
        if (-not (Test-Path $root)) { continue }
        $libraries += $root
        $vdf = Join-Path $root 'steamapps\libraryfolders.vdf'
        if (-not (Test-Path $vdf)) { continue }
        foreach ($line in (Get-Content $vdf -ErrorAction SilentlyContinue)) {
            if ($line -match '"path"\s+"(.+?)"') {
                $libraries += ($Matches[1] -replace '\\\\', '\')
            }
        }
    }
    $libraries | Where-Object { $_ } | Select-Object -Unique
}

function Get-VrRuntimePath {
    $vrpathreg = $null
    foreach ($lib in (Get-SteamLibraryPaths)) {
        $c = Join-Path $lib 'steamapps\common\SteamVR\bin\win64\vrpathreg.exe'
        if (Test-Path $c) { $vrpathreg = $c; break }
    }
    if ($vrpathreg) {
        $out = & $vrpathreg show 2>&1 | Out-String
        if ($out -match 'Runtime path\s*=\s*(.+)') {
            $candidate = $Matches[1].Trim()
            if (Test-Path $candidate) { return $candidate }
        }
    }
    foreach ($base in @($UserLocalAppData, $env:LOCALAPPDATA)) {
        if (-not $base) { continue }
        $vrpath = Join-Path $base 'openvr\openvrpaths.vrpath'
        if (-not (Test-Path $vrpath)) { continue }
        try {
            $json = Get-Content $vrpath -Raw | ConvertFrom-Json
            foreach ($rt in @($json.runtime)) {
                if ($rt -and (Test-Path $rt)) { return $rt }
            }
        } catch { }
    }
    return $null
}

$vrRuntimePath = (Get-ItemProperty 'HKLM:\Software\QuestCalibrator\Driver' -ErrorAction SilentlyContinue).'(default)'
if (-not $vrRuntimePath -or -not (Test-Path $vrRuntimePath)) { $vrRuntimePath = Get-VrRuntimePath }

if ($vrRuntimePath) {
    $driverDir = Join-Path $vrRuntimePath 'drivers\01questcalibrator'
    if (Test-Path $driverDir) {
        Remove-Item $driverDir -Recurse -Force -ErrorAction SilentlyContinue
        if (Test-Path $driverDir) {
            Write-Host "Warning: could not fully remove $driverDir - delete it manually after a reboot." -ForegroundColor Yellow
        } else {
            Write-Host "Removed driver: $driverDir"
        }
    }
} else {
    Write-Host "Warning: could not locate the SteamVR runtime; the driver folder may still be present." -ForegroundColor Yellow
}

# --- Remove installed files ------------------------------------------------------
if (Test-Path $installDir) {
    Remove-Item $installDir -Recurse -Force -ErrorAction SilentlyContinue
    if (Test-Path $installDir) {
        Write-Host "Warning: could not fully remove $installDir - delete it manually after a reboot." -ForegroundColor Yellow
    } else {
        Write-Host "Removed $installDir"
    }
}

# --- Registry + shortcut -----------------------------------------------------------
# SteamVR's activateMultipleDrivers setting is deliberately left enabled: other
# OpenVR tools rely on it, and turning it off would break them.
Remove-Item 'HKLM:\Software\QuestCalibrator' -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item 'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\QuestCalibrator' -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item (Join-Path $env:ProgramData 'Microsoft\Windows\Start Menu\Programs\QuestCalibrator.lnk') -Force -ErrorAction SilentlyContinue

if (-not $Silent) {
    Write-Host ""
    Write-Host "QuestCalibrator uninstalled." -ForegroundColor Green
    Read-Host "Press Enter to close"
}
