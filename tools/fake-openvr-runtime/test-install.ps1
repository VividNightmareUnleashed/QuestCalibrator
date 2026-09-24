# Installs, upgrades and uninstalls a release package against the fake OpenVR
# runtime and checks what each step left on the machine: files, the driver,
# registry keys, the Start Menu shortcut, and what the overlay registered with
# the runtime. Fails on the first scenario with a mismatch.
#
# It installs into Program Files and HKLM for real, so it refuses to run
# anywhere but a disposable GitHub Actions runner unless -OnThisMachine is
# given. Run it elevated.
[CmdletBinding()]
param(
    # Release zip under test.
    [Parameter(Mandatory)] [string]$Package,
    # Earlier release zip for the upgrade scenario; skipped when absent.
    [string]$Previous,
    # Fake runtime built by build.ps1.
    [Parameter(Mandatory)] [string]$Runtime,
    [switch]$OnThisMachine
)

$ErrorActionPreference = 'Stop'
if ($env:GITHUB_ACTIONS -ne 'true' -and -not $OnThisMachine) {
    throw 'This test installs QuestCalibrator into Program Files and HKLM. Run it on a disposable machine, or pass -OnThisMachine.'
}
$principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Run the install test elevated.'
}

$Runtime = (Resolve-Path $Runtime).Path
$work = Join-Path ([IO.Path]::GetTempPath()) ('qc-install-test-' + [Guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Force $work | Out-Null

$installDir = Join-Path ${env:ProgramFiles} 'QuestCalibrator'
$driverDir = Join-Path $Runtime 'drivers\01questcalibrator'
$shortcut = Join-Path $env:ProgramData 'Microsoft\Windows\Start Menu\Programs\QuestCalibrator.lnk'
$uninstallKey = 'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\QuestCalibrator'
$stateFile = Join-Path $Runtime 'fake-openvr\state.txt'
$callsLog = Join-Path $Runtime 'fake-openvr\calls.log'
$appKey = 'burrow.QuestCalibrator'

$script:failures = 0
function Check([string]$name, [bool]$ok, [string]$detail = '') {
    if ($ok) {
        Write-Host "  PASS  $name"
    } else {
        $script:failures++
        Write-Host "  FAIL  $name  $detail" -ForegroundColor Red
        if ($env:GITHUB_ACTIONS -eq 'true') { Write-Host "::error::$name $detail" }
    }
}

# The installer finds the runtime through openvrpaths.vrpath, as it would on
# a machine where SteamVR has run once, and so does the overlay's
# openvr_api.dll when the installer runs it.
$vrpathDir = Join-Path $env:LOCALAPPDATA 'openvr'
New-Item -ItemType Directory -Force $vrpathDir, (Join-Path $work 'config'), (Join-Path $work 'log') | Out-Null
[ordered]@{
    config = @(Join-Path $work 'config')
    external_drivers = $null
    jsonid = 'vrpathreg'
    log = @(Join-Path $work 'log')
    runtime = @($Runtime)
    version = 1
} | ConvertTo-Json | Set-Content (Join-Path $vrpathDir 'openvrpaths.vrpath') -Encoding ascii

function Expand-Package([string]$zip, [string]$name) {
    $dest = Join-Path $work $name
    Expand-Archive -LiteralPath $zip -DestinationPath $dest -Force
    $roots = @(Get-ChildItem $dest -Directory)
    if ($roots.Count -ne 1) { throw "$zip should hold exactly one top-level folder." }
    $roots[0].FullName
}

function Reset-Runtime {
    Remove-Item (Join-Path $Runtime 'fake-openvr') -Recurse -Force -ErrorAction SilentlyContinue
    Get-ChildItem (Join-Path $Runtime 'drivers') -ErrorAction SilentlyContinue | Remove-Item -Recurse -Force
}

# Windows PowerShell, as "Run with PowerShell" uses. A package from before
# -Unattended existed still pauses at the end, so feed it blank lines.
function Invoke-Script([string]$script, [string[]]$arguments) {
    $keys = Join-Path $work 'enter.txt'
    Set-Content $keys ("`r`n" * 8) -NoNewline
    $out = Join-Path $work 'script-out.txt'
    $err = Join-Path $work 'script-err.txt'
    $p = Start-Process powershell.exe -Wait -PassThru -NoNewWindow `
        -ArgumentList (@('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', "`"$script`"") + $arguments) `
        -RedirectStandardInput $keys -RedirectStandardOutput $out -RedirectStandardError $err
    Get-Content $out, $err -ErrorAction SilentlyContinue | ForEach-Object { Write-Host "    | $_" }
    $p.ExitCode
}

function Supports-Parameter([string]$script, [string]$name) {
    $ast = [Management.Automation.Language.Parser]::ParseFile($script, [ref]$null, [ref]$null)
    [bool]($ast.ParamBlock.Parameters | Where-Object { $_.Name.VariablePath.UserPath -eq $name })
}

function Install([string]$packageDir) {
    $script = Join-Path $packageDir 'Install.ps1'
    # Older packages (the upgrade scenario's previous release) predate these.
    $arguments = @()
    if (Supports-Parameter $script 'Unattended') { $arguments += '-Unattended' }
    if (Supports-Parameter $script 'AcceptEula') { $arguments += '-AcceptEula' }
    Invoke-Script $script $arguments
}

function Uninstall {
    Invoke-Script (Join-Path $installDir 'Uninstall.ps1') @('-Silent')
}

function Read-State {
    $state = @{ apps = @{}; bools = @{} }
    if (Test-Path $stateFile) {
        foreach ($line in Get-Content $stateFile) {
            $f = $line -split "`t"
            if ($f[0] -eq 'app') { $state.apps[$f[1]] = @{ manifest = $f[2]; autoLaunch = $f[3] -eq '1' } }
            elseif ($f[0] -eq 'bool') { $state.bools["$($f[1])/$($f[2])"] = $f[3] -eq '1' }
        }
    }
    $state
}

function Get-Tree([string]$root) {
    $map = @{}
    if (Test-Path $root) {
        foreach ($file in Get-ChildItem $root -Recurse -File) {
            $map[$file.FullName.Substring($root.Length).TrimStart('\')] = (Get-FileHash $file.FullName).Hash
        }
    }
    $map
}

function Compare-Tree([string]$name, [hashtable]$expected, [hashtable]$actual) {
    $missing = @($expected.Keys | Where-Object { -not $actual.ContainsKey($_) })
    $extra = @($actual.Keys | Where-Object { -not $expected.ContainsKey($_) })
    $changed = @($expected.Keys | Where-Object { $actual.ContainsKey($_) -and $actual[$_] -ne $expected[$_] })
    Check $name ($missing.Count + $extra.Count + $changed.Count -eq 0) `
        "missing [$($missing -join ', ')] extra [$($extra -join ', ')] changed [$($changed -join ', ')]"
}

function Assert-Installed([string]$packageDir) {
    $expectedApp = Get-Tree (Join-Path $packageDir 'app')
    foreach ($extra in 'Uninstall.ps1', 'README-INSTALL.txt', 'LICENSE', 'THIRD-PARTY-NOTICES.txt') {
        $source = Join-Path $packageDir $extra
        if (Test-Path $source) { $expectedApp[$extra] = (Get-FileHash $source).Hash }
    }
    # The notices folder is installed beside the app as it is in the package.
    $notices = Get-Tree (Join-Path $packageDir 'THIRD-PARTY-NOTICES')
    foreach ($file in $notices.Keys) { $expectedApp["THIRD-PARTY-NOTICES\$file"] = $notices[$file] }
    Compare-Tree 'install folder matches the package' $expectedApp (Get-Tree $installDir)
    Compare-Tree 'driver folder matches the package' (Get-Tree (Join-Path $packageDir 'driver\01questcalibrator')) (Get-Tree $driverDir)

    $version = (Get-Item (Join-Path $packageDir 'app\QuestCalibrator.exe')).VersionInfo.ProductVersion
    $entry = Get-ItemProperty $uninstallKey -ErrorAction SilentlyContinue
    Check 'Programs and Features entry' ($null -ne $entry -and $entry.DisplayVersion -eq $version -and
        $entry.InstallLocation -eq $installDir -and $entry.QuietUninstallString -like '*Uninstall.ps1*-Silent') `
        "version '$($entry.DisplayVersion)' expected '$version'"
    Check 'install and driver paths recorded' (
        (Get-ItemProperty 'HKLM:\Software\QuestCalibrator\Main' -ErrorAction SilentlyContinue).'(default)' -eq $installDir -and
        (Get-ItemProperty 'HKLM:\Software\QuestCalibrator\Driver' -ErrorAction SilentlyContinue).'(default)' -eq $Runtime)
    $link = if (Test-Path $shortcut) { (New-Object -ComObject WScript.Shell).CreateShortcut($shortcut) } else { $null }
    Check 'Start Menu shortcut' ($null -ne $link -and $link.TargetPath -eq (Join-Path $installDir 'QuestCalibrator.exe'))

    $state = Read-State
    $app = $state.apps[$appKey]
    Check 'manifest registered from the install folder' ($null -ne $app -and
        $app.manifest -eq (Join-Path $installDir 'manifest.vrmanifest')) "registered '$($app.manifest)'"
    Check 'auto-launch on' ($null -ne $app -and $app.autoLaunch)
    Check 'multiple drivers enabled' ($state.bools['steamvr/activateMultipleDrivers'] -eq $true)
}

function Assert-Removed {
    Check 'install folder removed' (-not (Test-Path $installDir))
    Check 'driver folder removed' (-not (Test-Path $driverDir))
    Check 'registry keys removed' (-not (Test-Path 'HKLM:\Software\QuestCalibrator') -and -not (Test-Path $uninstallKey))
    Check 'shortcut removed' (-not (Test-Path $shortcut))
    Check 'manifest deregistered' (-not (Read-State).apps.ContainsKey($appKey))
}

function Assert-NoUnexpectedCalls {
    $unexpected = @(Get-Content $callsLog -ErrorAction SilentlyContinue | Where-Object { $_ -match "`tUNEXPECTED`t" })
    Check 'no calls outside the install path' ($unexpected.Count -eq 0) ($unexpected -join ' | ')
}

function Scenario([string]$title, [scriptblock]$body) {
    Write-Host ""
    Write-Host "== $title" -ForegroundColor Cyan
    Reset-Runtime
    & $body
    Assert-NoUnexpectedCalls
    Write-Host '  fake runtime calls:'
    Get-Content $callsLog -ErrorAction SilentlyContinue | ForEach-Object { Write-Host "    $_" }
}

$new = Expand-Package $Package 'new'
if ($Previous) { $old = Expand-Package $Previous 'old' }

Scenario 'fresh install, reinstall, uninstall' {
    Check 'install exits 0' ((Install $new) -eq 0)
    Assert-Installed $new
    Check 'reinstall over itself exits 0' ((Install $new) -eq 0)
    Assert-Installed $new
    Check 'uninstall exits 0' ((Uninstall) -eq 0)
    Assert-Removed
    Check 'multiple drivers left enabled' ((Read-State).bools['steamvr/activateMultipleDrivers'] -eq $true)
}

Scenario 'conflicting Space Calibrator drivers are removed' {
    foreach ($conflict in '01spacecalibrator', '000spacecalibrator') {
        $dir = Join-Path $Runtime "drivers\$conflict\bin\win64"
        New-Item -ItemType Directory -Force $dir | Out-Null
        Set-Content (Join-Path $dir 'driver_placeholder.dll') 'placeholder'
    }
    Check 'install exits 0' ((Install $new) -eq 0)
    Check 'conflicting drivers gone' (-not (Test-Path (Join-Path $Runtime 'drivers\01spacecalibrator')) -and
        -not (Test-Path (Join-Path $Runtime 'drivers\000spacecalibrator')))
    Assert-Installed $new
    Check 'uninstall exits 0' ((Uninstall) -eq 0)
    Assert-Removed
}

if ($Previous) {
    Scenario 'upgrade from the previous release' {
        Check 'previous install exits 0' ((Install $old) -eq 0)
        Assert-Installed $old
        Check 'upgrade exits 0' ((Install $new) -eq 0)
        Assert-Installed $new
        Check 'uninstall exits 0' ((Uninstall) -eq 0)
        Assert-Removed
    }
}

Write-Host ""
if ($script:failures -gt 0) {
    Write-Host "$($script:failures) check(s) failed." -ForegroundColor Red
    exit 1
}
Write-Host 'All install checks passed.' -ForegroundColor Green
