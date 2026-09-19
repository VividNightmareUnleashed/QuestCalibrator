# Builds the fake OpenVR runtime into -Out: <Out>\bin\vrclient_x64.dll, where
# openvr_api.dll loads the client from, and an empty drivers folder for the
# installer to write into. Uses the newest Visual Studio with the x64 C++
# tools, found with vswhere.
[CmdletBinding()]
param(
    [Parameter(Mandatory)] [string]$Out
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path $vswhere)) { throw 'vswhere.exe not found; install Visual Studio with the C++ workload.' }
$vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vs) { throw 'No Visual Studio installation with the x64 C++ tools.' }
$vcvars = Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'

$bin = Join-Path $Out 'bin'
$obj = Join-Path $Out 'obj'
New-Item -ItemType Directory -Force $bin, $obj, (Join-Path $Out 'drivers') | Out-Null
$bin = (Resolve-Path $bin).Path
$obj = (Resolve-Path $obj).Path

$source = Join-Path $PSScriptRoot 'fake_vrclient.cpp'
$include = Join-Path $repo 'lib\openvr'
$dll = Join-Path $bin 'vrclient_x64.dll'
$cl = "cl /nologo /std:c++17 /EHsc /O2 /W4 /WX /LD /I `"$include`" `"$source`" /Fo`"$obj\\`" /Fe`"$dll`" /link /NOLOGO"
cmd /c "`"$vcvars`" >nul && $cl"
if ($LASTEXITCODE -ne 0) { throw "Building the fake runtime failed ($LASTEXITCODE)." }
Remove-Item (Join-Path $bin 'vrclient_x64.lib'), (Join-Path $bin 'vrclient_x64.exp') -ErrorAction SilentlyContinue
Remove-Item $obj -Recurse -Force
Write-Host "Fake OpenVR runtime: $((Resolve-Path $Out).Path)"
