# QuestCalibrator installer (script-based, replaces the NSIS installer).
# Run elevated. Installs the overlay app + OpenVR driver and registers the
# app manifest with SteamVR.
[CmdletBinding()]
param(
    # Captured before elevation: an elevated shell can run under a different
    # profile, and openvrpaths.vrpath lives in the *user's* LocalAppData.
    [string]$UserLocalAppData = $env:LOCALAPPDATA,
    # For scripted installs and the install test: never wait for a key press,
    # never elevate (run it elevated), and decline the OpenVR-SpaceCalibrator
    # removal prompt rather than answer it for the user. The exit code is the
    # result.
    [switch]$Unattended,
    # Accepts the license agreement in LICENSE, including the specific approval
    # of its sections 2, 6 and 8. An unattended install needs it.
    [switch]$AcceptEula,
    # Optional modules. An unattended install gets only the ones named here;
    # an interactive one asks, offering what the previous install had.
    [switch]$Lighthouse
)

$ErrorActionPreference = 'Stop'

function Pause-ForUser {
    if (-not $Unattended) { Read-Host "Press Enter to close" | Out-Null }
}

# Ensure the window stays open on any failure so the user can read the error.
trap {
    Write-Host ""
    Write-Host "UNEXPECTED ERROR: $_" -ForegroundColor Red
    Write-Host ""
    Pause-ForUser
    exit 1
}

function Fail([string]$msg) {
    Write-Host ""
    Write-Host "ERROR: $msg" -ForegroundColor Red
    Write-Host ""
    Pause-ForUser
    exit 1
}

# --- Admin check ------------------------------------------------------------
$principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    if ($Unattended) { Fail "An unattended install must be run from an elevated shell." }
    Write-Host "QuestCalibrator needs administrator rights to install (it writes into the SteamVR folder)." -ForegroundColor Yellow
    Write-Host "Re-launching elevated..."
    # Use -NoExit so the elevated window stays open to show output/errors
    $relaunchArgs = @(
        '-NoExit', '-NoProfile', '-ExecutionPolicy', 'Bypass',
        '-File', "`"$PSCommandPath`"",
        '-UserLocalAppData', "`"$env:LOCALAPPDATA`""
    )
    if ($Lighthouse) { $relaunchArgs += '-Lighthouse' }
    if ($AcceptEula) { $relaunchArgs += '-AcceptEula' }
    Start-Process -FilePath "powershell.exe" -Verb RunAs -ArgumentList $relaunchArgs
    exit 0
}

# --- SteamVR must be closed --------------------------------------------------
# steam.exe itself also locks driver DLLs, not just the VR processes
$steamProcesses = Get-Process -Name 'steam','vrserver','vrmonitor','vrcompositor' -ErrorAction SilentlyContinue
if ($steamProcesses) {
    $names = ($steamProcesses | Select-Object -ExpandProperty Name) -join ', '
    Fail "Steam is still running ($names). Please close Steam completely (check the system tray) and run this again."
}

# --- License agreement --------------------------------------------------------
# Asked before anything changes, and read before the upgrade path's uninstall
# clears the registry. An update whose LICENSE is unchanged since the user last
# agreed doesn't ask again; changed terms are asked for again. Some legal
# systems bind users to the restriction, termination and liability clauses of
# standard terms only when they approve them separately, hence the second
# question.
$licensePath = Join-Path $PSScriptRoot 'LICENSE'
if (-not (Test-Path -LiteralPath $licensePath)) {
    Fail "LICENSE is missing from the package. Extract the whole zip and run this again."
}
# Hashed with .NET, not Get-FileHash: started from PowerShell 7, Windows
# PowerShell inherits a module path under which Get-FileHash cannot load.
# Uppercase hex, as Get-FileHash and the overlay write it.
$sha256 = [Security.Cryptography.SHA256]::Create()
try {
    $licenseHash = [BitConverter]::ToString($sha256.ComputeHash([IO.File]::ReadAllBytes($licensePath))) -replace '-', ''
} finally {
    $sha256.Dispose()
}
$licenseKey  = 'HKLM:\Software\QuestCalibrator\License'
$agreedHash  = (Get-ItemProperty $licenseKey -ErrorAction SilentlyContinue).AcceptedSha256

function Stop-NotAccepted {
    Write-Host ""
    Write-Host "Installation cancelled: the license agreement was not accepted. Nothing was changed." -ForegroundColor Yellow
    Write-Host ""
    Pause-ForUser
    exit 1
}

if ($Unattended) {
    if (-not $AcceptEula) {
        Fail "An unattended install must pass -AcceptEula to accept the license agreement in LICENSE."
    }
} elseif (-not $AcceptEula -and $agreedHash -ne $licenseHash) {
    Write-Host ""
    Write-Host "License agreement" -ForegroundColor Cyan
    Write-Host "  QuestCalibrator is free to use under the end user license agreement in"
    Write-Host "  $licensePath"
    Write-Host "  In short: use it on your own computers; don't redistribute, sell, modify"
    Write-Host "  or reverse engineer it, except where the law allows."
    do {
        $answer = Read-Host "Type R to read it here, Y to agree, or N to cancel"
        if ($answer -match '^[Rr]') { Get-Content -LiteralPath $licensePath | Out-Host -Paging }
    } until ($answer -match '^[YyNn]')
    if ($answer -notmatch '^[Yy]') { Stop-NotAccepted }

    Write-Host ""
    Write-Host "  These sections need your separate approval: 2 (Restrictions),"
    Write-Host "  6 (Termination) and 8 (Limitation of liability)."
    $answer = Read-Host "Do you specifically approve sections 2, 6 and 8? [Y/N]"
    if ($answer -notmatch '^[Yy]') { Stop-NotAccepted }
}

# --- Optional modules ---------------------------------------------------------
# Asked before anything changes, and read before the upgrade path's uninstall
# clears the registry, so an upgrade can offer the previous choice.
$modulesKey = 'HKLM:\Software\QuestCalibrator\Modules'
$installLighthouse = [bool]$Lighthouse
if (-not $Unattended -and -not $Lighthouse) {
    $hadLighthouse = (Get-ItemProperty $modulesKey -ErrorAction SilentlyContinue).Lighthouse -eq 1
    Write-Host ""
    Write-Host "Optional module: Lighthouse" -ForegroundColor Cyan
    Write-Host "  Base station tools, starting with a Lighthouse tab that shows each"
    Write-Host "  base station and which of your lighthouse-tracked devices can see it."
    Write-Host "  More tools will come to it in later versions. It is not part of the"
    Write-Host "  classic calibrator; leave it out if you only want calibration."
    $default = if ($hadLighthouse) { 'Y' } else { 'N' }
    $answer = Read-Host "Install the Lighthouse module? [Y/N] (default $default)"
    if (-not $answer) { $answer = $default }
    $installLighthouse = $answer -match '^[Yy]'
}

$packageDir = $PSScriptRoot
$appSrc     = Join-Path $packageDir 'app'
$driverSrc  = Join-Path $packageDir 'driver\01questcalibrator'

foreach ($p in @($appSrc, $driverSrc)) {
    if (-not (Test-Path $p)) { Fail "Package is incomplete: '$p' is missing. Re-extract the zip and try again." }
}

# --- Locate the OpenVR/SteamVR runtime ---------------------------------------
# Steam records its install root in HKLM, and libraryfolders.vdf lists every
# additional library - including custom paths that guessing drive layouts misses.
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
                # paths are backslash-escaped inside the vdf
                $libraries += ($Matches[1] -replace '\\\\', '\')
            }
        }
    }
    $libraries | Where-Object { $_ } | Select-Object -Unique
}

function Find-VrPathReg {
    foreach ($lib in (Get-SteamLibraryPaths)) {
        $c = Join-Path $lib 'steamapps\common\SteamVR\bin\win64\vrpathreg.exe'
        if (Test-Path $c) { return $c }
    }
    # Last resort: sweep fixed drives for the common layouts. Fixed drives only,
    # so a disconnected network drive can't stall the installer.
    $drives = Get-CimInstance Win32_LogicalDisk -Filter 'DriveType=3' -ErrorAction SilentlyContinue |
        Select-Object -ExpandProperty DeviceID
    foreach ($drive in $drives) {
        foreach ($sub in @('SteamLibrary', 'Steam', 'Games\SteamLibrary', 'Games\Steam',
                           'Program Files (x86)\Steam', 'Program Files\Steam')) {
            $c = Join-Path "$drive\" "$sub\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"
            if (Test-Path $c) { return $c }
        }
    }
    return $null
}

function Get-VrRuntimePath {
    $vrpathreg = Find-VrPathReg
    if ($vrpathreg) {
        Write-Host "Using $vrpathreg"
        $out = & $vrpathreg show 2>&1 | Out-String
        if ($out -match 'Runtime path\s*=\s*(.+)') {
            $candidate = $Matches[1].Trim()
            if (Test-Path $candidate) { return $candidate }
        }
    }
    # openvrpaths.vrpath is the file vrpathreg itself reads. Going straight to it
    # covers non-Steam runtimes and layouts the search above didn't guess.
    foreach ($base in @($UserLocalAppData, $env:LOCALAPPDATA)) {
        if (-not $base) { continue }
        $vrpath = Join-Path $base 'openvr\openvrpaths.vrpath'
        if (-not (Test-Path $vrpath)) { continue }
        try {
            $json = Get-Content $vrpath -Raw | ConvertFrom-Json
            foreach ($rt in @($json.runtime)) {
                if ($rt -and (Test-Path $rt)) { return $rt }
            }
        } catch {
            Write-Host "  (could not parse $vrpath)" -ForegroundColor Yellow
        }
    }
    return $null
}

$vrRuntimePath = Get-VrRuntimePath
if (-not $vrRuntimePath) {
    Fail "Could not find the SteamVR runtime. Is SteamVR installed? If it has never been started on this PC, run it once, close it, and try again."
}
Write-Host "VR runtime path: $vrRuntimePath"

# --- Conflicting software: OpenVR-SpaceCalibrator ----------------------------
# Its driver hooks the same vrserver internals; both installed at once
# double-apply offsets, so it has to go before we install.
$spaceCalUninstall = Get-ItemProperty 'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator' -ErrorAction SilentlyContinue
if ($spaceCalUninstall) {
    Write-Host ""
    Write-Host "OpenVR-SpaceCalibrator is installed and conflicts with QuestCalibrator." -ForegroundColor Yellow
    $answer = if ($Unattended) { 'N' } else { Read-Host "Uninstall it and continue? [Y/N]" }
    if ($answer -notmatch '^[Yy]') { Fail "Aborted. Uninstall OpenVR-SpaceCalibrator first, then re-run." }

    $spaceCalDir = (Get-ItemProperty 'HKLM:\Software\OpenVR-SpaceCalibrator\Main' -ErrorAction SilentlyContinue).'(default)'
    if (-not $spaceCalDir) { $spaceCalDir = Join-Path ${env:ProgramFiles} 'OpenVR-SpaceCalibrator' }

    $spaceCalUninstaller = Join-Path $spaceCalDir 'Uninstall.exe'
    if (Test-Path $spaceCalUninstaller) {
        Write-Host "Uninstalling OpenVR-SpaceCalibrator..."
        Start-Process -FilePath $spaceCalUninstaller -ArgumentList '/S', "_?=$spaceCalDir" -Wait
        Remove-Item $spaceCalUninstaller -Force -ErrorAction SilentlyContinue
        Remove-Item $spaceCalDir -Recurse -Force -ErrorAction SilentlyContinue
    } else {
        # Half-removed install: uninstaller is gone, clean up its traces directly
        Remove-Item 'HKLM:\Software\OpenVR-SpaceCalibrator' -Recurse -Force -ErrorAction SilentlyContinue
        Remove-Item 'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator' -Recurse -Force -ErrorAction SilentlyContinue
        Remove-Item (Join-Path $env:APPDATA 'Microsoft\Windows\Start Menu\Programs\OpenVR-SpaceCalibrator.lnk') -Force -ErrorAction SilentlyContinue
        Remove-Item (Join-Path $env:ProgramData 'Microsoft\Windows\Start Menu\Programs\OpenVR-SpaceCalibrator.lnk') -Force -ErrorAction SilentlyContinue
    }
}

# The driver folder is the actual conflict; sweep it even when the
# uninstaller ran (or was never registered), covering forks and manual installs.
foreach ($conflict in @('01spacecalibrator', '000spacecalibrator')) {
    $driversRoot = [IO.Path]::GetFullPath((Join-Path $vrRuntimePath 'drivers'))
    $p = [IO.Path]::GetFullPath((Join-Path $driversRoot $conflict))
    if (-not $p.StartsWith($driversRoot.TrimEnd('\') + '\', [StringComparison]::OrdinalIgnoreCase)) {
        Fail 'Conflicting driver path is outside the SteamVR drivers directory.'
    }
    if (Test-Path -LiteralPath $p) {
        Write-Host "Removing conflicting driver: $p"
        for ($i = 1; $i -le 3; $i++) {
            try {
                Remove-Item -LiteralPath $p -Recurse -Force -ErrorAction Stop
                break
            } catch {
                if ($i -lt 3) {
                    Write-Host "  Locked (attempt $i/3), waiting..."
                    Start-Sleep -Seconds 2
                } else {
                    Fail "Could not remove conflicting driver $p. Restart Windows, remove that driver, then re-run this installer. QuestCalibrator has not been installed or upgraded."
                }
            }
        }
        if (Test-Path -LiteralPath $p) {
            Fail "Conflicting driver still exists: $p. Remove it before installing QuestCalibrator."
        }
    }
}

# --- Install location ---------------------------------------------------------
$installDir = Join-Path ${env:ProgramFiles} 'QuestCalibrator'

# Upgrade path: clear out whichever installer put files here last. Early builds
# shipped an NSIS installer, so Uninstall.exe has to be handled too - otherwise
# it is orphaned in Program Files with its registry entry overwritten by ours.
if (Test-Path (Join-Path $installDir 'Uninstall.ps1')) {
    Write-Host "Existing installation found - upgrading..."
    & (Join-Path $installDir 'Uninstall.ps1') -Silent
} elseif (Test-Path (Join-Path $installDir 'Uninstall.exe')) {
    Write-Host "Existing NSIS installation found - removing it first..."
    Start-Process -FilePath (Join-Path $installDir 'Uninstall.exe') -ArgumentList '/S', "_?=$installDir" -Wait
    Remove-Item (Join-Path $installDir 'Uninstall.exe') -Force -ErrorAction SilentlyContinue
}

New-Item -ItemType Directory -Force -Path $installDir | Out-Null
Copy-Item (Join-Path $appSrc '*') -Destination $installDir -Recurse -Force
Copy-Item (Join-Path $packageDir 'Uninstall.ps1')      -Destination $installDir -Force
Copy-Item (Join-Path $packageDir 'README-INSTALL.txt') -Destination $installDir -Force -ErrorAction SilentlyContinue
Copy-Item (Join-Path $packageDir 'LICENSE')            -Destination $installDir -Force -ErrorAction SilentlyContinue
Copy-Item (Join-Path $packageDir 'THIRD-PARTY-NOTICES.txt') -Destination $installDir -Force -ErrorAction SilentlyContinue
Copy-Item (Join-Path $packageDir 'THIRD-PARTY-NOTICES')     -Destination $installDir -Recurse -Force -ErrorAction SilentlyContinue

$appExe = Join-Path $installDir 'QuestCalibrator.exe'
if (-not (Test-Path $appExe)) { Fail "Copying the application to $installDir failed." }

$version = (Get-Item $appExe).VersionInfo.ProductVersion
if (-not $version) { $version = 'unknown' }

# --- Driver files -------------------------------------------------------------
$driverDest = Join-Path $vrRuntimePath 'drivers\01questcalibrator'
if (Test-Path $driverDest) {
    # The driver DLL may be locked by a recently-closed SteamVR or AV scan.
    # Retry a few times with a delay before giving up.
    $removed = $false
    for ($i = 1; $i -le 3; $i++) {
        try {
            Remove-Item $driverDest -Recurse -Force -ErrorAction Stop
            $removed = $true
            break
        } catch {
            if ($i -lt 3) {
                Write-Host "Driver files locked (attempt $i/3), waiting 2 seconds..."
                Start-Sleep -Seconds 2
            }
        }
    }
    if (-not $removed) {
        Write-Host ""
        Write-Host "Could not remove the old driver - the file is still locked." -ForegroundColor Yellow
        Write-Host "This usually means SteamVR was recently running or an antivirus is scanning it." -ForegroundColor Yellow
        Write-Host ""
        Write-Host "Please:" -ForegroundColor Cyan
        Write-Host "  1. Reboot your PC (this releases all file locks)" -ForegroundColor Cyan
        Write-Host "  2. Run this installer again immediately after logging in" -ForegroundColor Cyan
        Write-Host ""
        Pause-ForUser
        exit 1
    }
}
Copy-Item $driverSrc -Destination $driverDest -Recurse -Force
if (-not (Test-Path (Join-Path $driverDest 'bin\win64\driver_01questcalibrator.dll'))) {
    Fail "Copying the driver to $driverDest failed."
}
Write-Host "Installed driver to $driverDest"

# --- Register with SteamVR ----------------------------------------------------
# QuestCalibrator.exe is a GUI binary: PowerShell's call operator does not wait
# for one and $LASTEXITCODE would be meaningless, so use Start-Process -Wait.
# -WorkingDirectory matters because the app resolves manifest.vrmanifest next to
# itself; -noui suppresses the result dialog so a scripted install never blocks.
function Invoke-App([string]$appArgument) {
    $proc = Start-Process -FilePath $appExe -ArgumentList @($appArgument, '-noui') `
        -WorkingDirectory $installDir -Wait -PassThru
    return $proc.ExitCode
}

$code = Invoke-App '-installmanifest'
if ($code -ne 0) {
    Fail "Failed to register the application manifest with SteamVR (exit code $code). QuestCalibrator will not appear in the dashboard or start automatically."
}
Write-Host "Registered with SteamVR"

# Not fatal: the app also enables this itself every time it starts, so a failure
# here (usually SteamVR having never been run) heals on first launch.
$code = Invoke-App '-activatemultipledrivers'
if ($code -ne 0) {
    Write-Host "Note: could not pre-enable SteamVR's multiple-driver setting (exit code $code)." -ForegroundColor Yellow
    Write-Host "      QuestCalibrator will enable it itself the first time it runs." -ForegroundColor Yellow
}

# --- Registry + shortcut ------------------------------------------------------
New-Item -Path 'HKLM:\Software\QuestCalibrator' -Force | Out-Null
Set-ItemProperty -Path 'HKLM:\Software\QuestCalibrator' -Name '(default)' -Value '' -ErrorAction SilentlyContinue
New-Item -Path 'HKLM:\Software\QuestCalibrator\Main' -Force | Out-Null
Set-ItemProperty -Path 'HKLM:\Software\QuestCalibrator\Main' -Name '(default)' -Value $installDir
New-Item -Path 'HKLM:\Software\QuestCalibrator\Driver' -Force | Out-Null
Set-ItemProperty -Path 'HKLM:\Software\QuestCalibrator\Driver' -Name '(default)' -Value $vrRuntimePath
# The overlay reads this at startup (UserInterface.cpp) to enable each module.
New-Item -Path $modulesKey -Force | Out-Null
Set-ItemProperty -Path $modulesKey -Name 'Lighthouse' -Value ([int]$installLighthouse) -Type DWord
# Which license text was agreed to, and when, so an update with the same terms
# doesn't ask again.
New-Item -Path $licenseKey -Force | Out-Null
Set-ItemProperty -Path $licenseKey -Name 'AcceptedSha256' -Value $licenseHash
Set-ItemProperty -Path $licenseKey -Name 'AcceptedUtc' -Value ((Get-Date).ToUniversalTime().ToString('yyyy-MM-ddTHH:mm:ssZ'))

$installedKb  = [int](((Get-ChildItem $installDir -Recurse -File | Measure-Object -Property Length -Sum).Sum) / 1KB)
$uninstallCmd = "powershell.exe -NoProfile -ExecutionPolicy Bypass -File `"$installDir\Uninstall.ps1`""

$uninstallReg = 'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\QuestCalibrator'
New-Item -Path $uninstallReg -Force | Out-Null
Set-ItemProperty -Path $uninstallReg -Name 'DisplayName'          -Value 'QuestCalibrator'
Set-ItemProperty -Path $uninstallReg -Name 'DisplayVersion'       -Value $version
Set-ItemProperty -Path $uninstallReg -Name 'Publisher'            -Value 'VividNightmare'
Set-ItemProperty -Path $uninstallReg -Name 'DisplayIcon'          -Value $appExe
Set-ItemProperty -Path $uninstallReg -Name 'InstallLocation'      -Value $installDir
Set-ItemProperty -Path $uninstallReg -Name 'UninstallString'      -Value $uninstallCmd
Set-ItemProperty -Path $uninstallReg -Name 'QuietUninstallString' -Value "$uninstallCmd -Silent"
Set-ItemProperty -Path $uninstallReg -Name 'EstimatedSize'        -Value $installedKb -Type DWord
Set-ItemProperty -Path $uninstallReg -Name 'NoModify'             -Value 1 -Type DWord
Set-ItemProperty -Path $uninstallReg -Name 'NoRepair'             -Value 1 -Type DWord

$shortcutPath = Join-Path $env:ProgramData 'Microsoft\Windows\Start Menu\Programs\QuestCalibrator.lnk'
$wsh = New-Object -ComObject WScript.Shell
$shortcut = $wsh.CreateShortcut($shortcutPath)
$shortcut.TargetPath = $appExe
$shortcut.WorkingDirectory = $installDir
$shortcut.IconLocation = "$appExe,0"
$shortcut.Save()

Write-Host ""
Write-Host "QuestCalibrator $version installed successfully." -ForegroundColor Green
Write-Host ("Lighthouse module: " + $(if ($installLighthouse) { 'installed' } else { 'not installed (run this installer again to add it)' }))
Write-Host "Start SteamVR to use it. A Start Menu shortcut was created."
Write-Host ""
Pause-ForUser
exit 0
