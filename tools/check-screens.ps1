# Draws every preview screen in each language to a PNG, and fails on a run
# that exits non-zero, hangs, writes no picture or draws an empty frame. The
# previews run on fake state, without SteamVR or a headset.
#
#   tools\check-screens.ps1 -Out <folder> [-Exe <QuestCalibrator.exe>] [-OpenGL <folder>]
#
# -OpenGL names a folder holding another OpenGL implementation's opengl32.dll
# and the DLLs it loads. The executable then runs from a copy beside them,
# which Windows loads ahead of its own: CI passes Mesa's software renderer,
# since its runners have no GPU.
#
# A shot draws thirty frames, and a preview window that is not on screen
# waits about a second between frames, so each run takes about half a minute
# while it mostly sleeps. The runs go several at a time.
[CmdletBinding()]
param(
    [Parameter(Mandatory)] [string]$Out,
    [string]$Exe = '',
    [string]$OpenGL = '',
    [int]$Parallel = 8,
    [int]$TimeoutSeconds = 180
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
if (-not $Exe) { $Exe = Join-Path $root 'x64\Release\QuestCalibrator.exe' }
$Exe = (Resolve-Path -LiteralPath $Exe).Path
# Every -uipreview flag Overlay\QuestCalibrator.cpp reads; validate-cpp.ps1
# fails when the two lists differ.
$Screens = @('-uipreview', '-uipreview-many', '-uipreview-guide', '-uipreview-guide-wait', '-uipreview-result',
    '-uipreview-frozen', '-uipreview-trackeroff', '-uipreview-failed', '-uipreview-empty',
    '-uipreview-lighthouse', '-uipreview-settings')
$Languages = @('en', 'ja', 'it')

New-Item -ItemType Directory -Force -Path $Out | Out-Null
$Out = (Resolve-Path -LiteralPath $Out).Path

if ($OpenGL) {
    $run = Join-Path ([IO.Path]::GetTempPath()) ('questcal-screens-' + [Guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Path $run | Out-Null
    Copy-Item -LiteralPath $Exe -Destination $run
    Copy-Item -LiteralPath (Join-Path (Split-Path -Parent $Exe) 'openvr_api.dll') -Destination $run
    Copy-Item -Path (Join-Path $OpenGL '*.dll') -Destination $run
    if (-not (Test-Path -LiteralPath (Join-Path $run 'opengl32.dll'))) { throw "No opengl32.dll in $OpenGL." }
    $Exe = Join-Path $run (Split-Path -Leaf $Exe)
    # Mesa's WGL driver otherwise prefers a hardware or D3D12 path when one
    # exists; llvmpipe is the one every runner has.
    $env:GALLIUM_DRIVER = 'llvmpipe'
}

Add-Type -AssemblyName System.Drawing
# An empty frame is one or two flat colours; every screen draws dozens.
function Get-SampledColours([string]$png) {
    $bitmap = [Drawing.Bitmap]::new($png)
    try {
        $colours = [Collections.Generic.HashSet[int]]::new()
        for ($y = 0; $y -lt $bitmap.Height; $y += 8) {
            for ($x = 0; $x -lt $bitmap.Width; $x += 8) { [void]$colours.Add($bitmap.GetPixel($x, $y).ToArgb()) }
        }
        return $colours.Count
    } finally { $bitmap.Dispose() }
}

$queue = [Collections.Generic.Queue[object]]::new()
foreach ($screen in $Screens) {
    foreach ($language in $Languages) {
        $queue.Enqueue([pscustomobject]@{
            Screen = $screen; Language = $language
            Png = Join-Path $Out ('{0}_{1}.png' -f $screen.TrimStart('-'), $language)
        })
    }
}
$running = [Collections.Generic.List[object]]::new()
$failures = [Collections.Generic.List[string]]::new()
while ($queue.Count -gt 0 -or $running.Count -gt 0) {
    while ($queue.Count -gt 0 -and $running.Count -lt $Parallel) {
        $shot = $queue.Dequeue()
        Remove-Item -LiteralPath $shot.Png -ErrorAction SilentlyContinue
        # -noui: a failure is reported by the exit code, never by a message box
        # nothing would close.
        $process = Start-Process -FilePath $Exe -PassThru -WindowStyle Hidden `
            -WorkingDirectory (Split-Path -Parent $Exe) `
            -ArgumentList @($shot.Screen, '-lang', $shot.Language, '-noui', '-shot', "`"$($shot.Png)`"")
        # Holding the handle keeps the exit code readable after the process
        # ends; Windows PowerShell loses it otherwise.
        $null = $process.Handle
        $running.Add([pscustomobject]@{ Shot = $shot; Process = $process; Started = Get-Date })
    }
    Start-Sleep -Milliseconds 250
    foreach ($entry in @($running)) {
        $shot = $entry.Shot
        $name = "$($shot.Screen) -lang $($shot.Language)"
        if (-not $entry.Process.HasExited) {
            if (((Get-Date) - $entry.Started).TotalSeconds -gt $TimeoutSeconds) {
                $entry.Process.Kill()
                $failures.Add("$name did not finish within $TimeoutSeconds s.")
                [void]$running.Remove($entry)
            }
            continue
        }
        [void]$running.Remove($entry)
        $entry.Process.WaitForExit()
        $code = $entry.Process.ExitCode
        if ($code -ne 0) { $failures.Add("$name exited with $code."); continue }
        if (-not (Test-Path -LiteralPath $shot.Png -PathType Leaf)) { $failures.Add("$name wrote no picture."); continue }
        $colours = Get-SampledColours $shot.Png
        if ($colours -lt 16) { $failures.Add("$name drew an empty frame ($colours colours)."); continue }
        Write-Output "ok $name ($colours colours)"
    }
}

if ($OpenGL) { Remove-Item -LiteralPath $run -Recurse -Force -ErrorAction SilentlyContinue }
$failures | ForEach-Object { Write-Output "FAILED $_" }
Write-Output "$($Screens.Count * $Languages.Count - $failures.Count) of $($Screens.Count * $Languages.Count) screens drawn in $Out."
if ($failures.Count -gt 0) { exit 1 }
