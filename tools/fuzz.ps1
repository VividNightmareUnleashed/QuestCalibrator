<#
.SYNOPSIS
    Builds the coverage-guided fuzzers in Tests/Fuzz with libFuzzer and
    AddressSanitizer, and runs each for a while.

.DESCRIPTION
    One executable per target in Tests/Fuzz/FuzzTargets.h (profile, feed,
    lighthouse, request), built with MSVC's /fsanitize=fuzzer,address into
    x64\fuzz\. Each run starts from the target's seeds and the corpus earlier
    runs grew (x64\fuzz\<target>\corpus), and stops at the time limit or at the
    first failure: a crash, a sanitizer report, an exception nothing catches, or
    a property FuzzTargets.h names. The failing input is saved beside the
    corpus as crash-*; replay it with `x64\fuzz\<target>.exe <file>`.

    The harness (SolverTests.exe) already replays every target's seeds and
    seeded mutations of them on each run; this is the long, coverage-guided
    search on top.

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass -File tools/fuzz.ps1
    powershell -NoProfile -ExecutionPolicy Bypass -File tools/fuzz.ps1 -Seconds 600 -Targets feed
#>
param(
    [int]$Seconds = 60,
    [string[]]$Targets = @('profile', 'feed', 'lighthouse', 'request')
)

# Continue, not Stop: cl and the fuzzers write progress to stderr, which
# Windows PowerShell would otherwise turn into terminating errors.
$ErrorActionPreference = 'Continue'
$root = Split-Path -Parent $PSScriptRoot
$out = Join-Path $root 'x64\fuzz'
New-Item -ItemType Directory -Force $out | Out-Null

# The VS 2022 C++ tools (v143, the toolset the solution builds with) and
# their x64 environment, for cl and the sanitizer runtime on PATH.
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$vs = & $vswhere -version '[17.0,18.0)' -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
    -latest -property installationPath
if (-not $vs) { throw 'The VS 2022 C++ build tools were not found.' }
$vcvars = Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'
foreach ($line in (& cmd.exe /c "`"$vcvars`" >nul 2>nul && set"))
{
    if ($line -match '^([^=]+)=(.*)$') { [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], 'Process') }
}

$sources = @('Tests\Fuzz\FuzzMain.cpp', 'Driver\AlignmentField.cpp', 'Overlay\LighthouseLog.cpp') |
    ForEach-Object { Join-Path $root $_ }

# Tokens the fuzzer may splice in whole: edge numbers and the grammar of each
# input, which byte-level mutation rarely spells out on its own.
$numbers = @('nan', 'inf', '-0', '1e999', '1e-320', '4294967295', '4294967296', '0.25', '4.000000000000001',
    '10000.000000000002', '100.00000000000001', '0.049999999999999996')
$dictionaries = @{
    profile    = $numbers + @('persistence_revision', 'reference_tracking_system', 'target_tracking_system',
        'rotation_quat', 'translation_meters', 'scale', 'time_offset', 'calibration_time', 'universe_unsafe',
        'hmd_serial', 'universe_hmd_serial', 'universe_world_from_driver_rotation_quat',
        'universe_world_from_driver_translation_meters', 'settings_version', 'solve_scale', 'mount_extrinsic',
        'rot_rms_deg', 'pos_rms_m', 'field_anchors', 'position', 'field_enabled', 'continuous_enabled',
        'continuous_tracker_serial', 'continuous_mode', 'legacy', 'calibration_speed', 'true', 'false', 'null')
    feed       = $numbers + @('draft', 'prerelease', 'tag_name', 'questcalibrator-v', 'html_url', 'assets', 'name',
        'size', 'digest', 'sha256:', 'browser_download_url', 'QuestCalibrator-', '.zip',
        'https://github.com/VividNightmareUnleashed/QuestCalibrator/releases/', 'tag/', 'download/', 'true', 'false')
    lighthouse = $numbers + @('lighthouse: LHR-', ' C: ', 'SOB: add ', 'SOB: drop ', 'S-', '(generation changed)',
        'also seeing ', 'seeing ', 'No base stations seen', 'BOOTSTRAPPED base ', 'Trying to start tracking from base ',
        'Fri ', 'Sep ', 'Feb ', ' 2026 ', '23:59:', '60.999', ' [Info] - ')
    request    = @()
}
$failed = @()
foreach ($t in $Targets)
{
    $dir = Join-Path $out $t
    $obj = Join-Path $dir 'obj'
    $corpus = Join-Path $dir 'corpus'
    $seeds = Join-Path $dir 'seeds'
    New-Item -ItemType Directory -Force $obj, $corpus | Out-Null
    $exe = Join-Path $out "$t.exe"

    # Objects and the compiler's PDB land in the working directory.
    Push-Location $obj
    $build = & cl.exe /nologo /std:c++17 /EHsc /O2 /Zi /MD /fsanitize=address /fsanitize=fuzzer `
        /DNOMINMAX /D_SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING /D_SILENCE_CXX17_ADAPTOR_TYPEDEFS_DEPRECATION_WARNING `
        "/DQUESTCAL_FUZZ_TARGET=$t" /I (Join-Path $root 'lib') /I (Join-Path $root 'lib\openvr') `
        $sources "/Fe$exe" /link /INCREMENTAL:NO advapi32.lib 2>&1
    Pop-Location
    if ($LASTEXITCODE -ne 0)
    {
        $build | Where-Object { $_ -match 'error' } | Select-Object -First 10
        throw "building the $t fuzzer failed"
    }

    $dictArg = ''
    if ($dictionaries[$t].Count -gt 0)
    {
        $dict = Join-Path $dir 'tokens.dict'
        [IO.File]::WriteAllLines($dict, [string[]]($dictionaries[$t] | ForEach-Object { '"' + $_.Replace('\', '\\').Replace('"', '\"') + '"' }))
        $dictArg = "`"-dict=$dict`""
    }

    $env:QUESTCAL_FUZZ_SEED_DIR = $seeds
    $log = Join-Path $dir 'last-run.txt'
    # Through cmd, so the fuzzer's stderr reaches the log as plain text.
    Push-Location $dir
    & cmd.exe /c "`"$exe`" `"$corpus`" `"$seeds`" $dictArg -max_total_time=$Seconds -max_len=8192 -print_final_stats=1 -artifact_prefix=./ > `"$log`" 2>&1"
    $code = $LASTEXITCODE
    Pop-Location
    Remove-Item Env:QUESTCAL_FUZZ_SEED_DIR

    $runs = Select-String -Path $log -Pattern 'stat::number_of_executed_units:\s+(\d+)' | Select-Object -Last 1
    $status = Select-String -Path $log -Pattern '#\d+\s+\w+\s+(?:cov: \d+ )?ft: (\d+) corp: (\d+)' | Select-Object -Last 1
    $summary = '{0} runs, {1} features, corpus {2}' -f $(if ($runs) { $runs.Matches[0].Groups[1].Value } else { '?' }),
        $(if ($status) { $status.Matches[0].Groups[1].Value } else { '?' }),
        $(if ($status) { $status.Matches[0].Groups[2].Value } else { '?' })
    if ($code -ne 0)
    {
        $failed += $t
        $why = Select-String -Path $log -Pattern 'property failed|ERROR: AddressSanitizer|ERROR: libFuzzer|deadly signal' |
            Select-Object -First 1
        $artifact = Get-ChildItem $dir -Filter 'crash-*' | Sort-Object LastWriteTime | Select-Object -Last 1
        '{0,-4} {1,-11} {2}  {3}  input: {4}' -f 'FAIL', $t, $summary, $(if ($why) { $why.Line.Trim() } else { "exit $code" }),
            $(if ($artifact) { $artifact.FullName } else { '(none saved)' })
    }
    else
    {
        '{0,-4} {1,-11} {2}' -f 'ok', $t, $summary
    }
}
if ($failed.Count -gt 0) { "Failed: $($failed -join ', '); the log is x64\fuzz\<target>\last-run.txt"; exit 1 }
"No failures in $Seconds s per target."
