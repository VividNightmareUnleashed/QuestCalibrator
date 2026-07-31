[CmdletBinding()]
param(
    [ValidateSet('Build', 'Analyze', 'Duplicates')]
    [string]$Mode = 'Build',
    [string]$Root = '',
    [switch]$All
)

$ErrorActionPreference = 'Stop'
$script:analysisAdvisory = $false
if ([string]::IsNullOrWhiteSpace($Root)) {
    $Root = Split-Path -Parent $PSScriptRoot
}
$Root = [System.IO.Path]::GetFullPath($Root)
$configPath = Join-Path $Root 'cpp-validation.json'
if (-not (Test-Path -LiteralPath $configPath -PathType Leaf)) {
    Write-Output "C++ validation config not found: $configPath"
    exit 2
}
$config = Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json
$solution = [System.IO.Path]::GetFullPath((Join-Path $Root ([string]$config.solution)))
$configuration = [string]$config.configuration
$platform = [string]$config.platform
$testExecutable = [System.IO.Path]::GetFullPath(
    (Join-Path $Root ([string]$config.testExecutable)))
$duplicateMinLines = [int]$config.duplicateMinLines
$duplicateMinTokens = [int]$config.duplicateMinTokens
if (-not $solution.StartsWith($Root, [System.StringComparison]::OrdinalIgnoreCase) -or
    -not [string]$config.testExecutable -or
    -not $testExecutable.StartsWith($Root, [System.StringComparison]::OrdinalIgnoreCase) -or
    -not $configuration -or -not $platform -or
    $duplicateMinLines -lt 1 -or $duplicateMinTokens -lt 1) {
    Write-Output "Invalid C++ validation config: $configPath"
    exit 2
}
$msbuildCandidates = @()
if (${env:ProgramFiles(x86)}) {
    $msbuildCandidates += Join-Path ${env:ProgramFiles(x86)} `
        'Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe'

    $vswhere = Join-Path ${env:ProgramFiles(x86)} `
        'Microsoft Visual Studio\Installer\vswhere.exe'
    if (Test-Path -LiteralPath $vswhere -PathType Leaf) {
        $msbuildCandidates += @(
            & $vswhere -latest -products '*' -requires Microsoft.Component.MSBuild `
                -find 'MSBuild\**\Bin\MSBuild.exe' 2>$null
        )
    }
}

$msbuildOnPath = Get-Command 'MSBuild.exe' -ErrorAction SilentlyContinue
if ($msbuildOnPath) {
    $msbuildCandidates += $msbuildOnPath.Source
}

$msbuild = $msbuildCandidates |
    Where-Object { $_ -and (Test-Path -LiteralPath $_ -PathType Leaf) } |
    Select-Object -First 1

function Reset-ProcessPath {
    # Some hosts can carry both Path and PATH. MSBuild copies environment
    # variables into a case-insensitive dictionary and rejects that duplicate.
    # Preserve action/dev-shell additions (for example setup-node and jscpd)
    # while normalizing the process block to one spelling.
    $processPath = [Environment]::GetEnvironmentVariable('Path', 'Process')
    if (-not $processPath) {
        $processPath = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
            [Environment]::GetEnvironmentVariable('Path', 'User')
    }
    Remove-Item Env:Path -ErrorAction SilentlyContinue
    $env:Path = $processPath
}

function Invoke-MSBuildValidation {
    param([switch]$ClangTidy)

    if (-not $msbuild) {
        Write-Output 'MSBuild not found. Install the VS 2022 C++ build tools with Microsoft.Component.MSBuild.'
        exit 2
    }
    if (-not (Test-Path -LiteralPath $solution -PathType Leaf)) {
        Write-Output "Solution not found: $solution"
        exit 2
    }

    Reset-ProcessPath
    $analysisStarted = Get-Date
    $arguments = @(
        $solution,
        "/p:Configuration=$configuration",
        "/p:Platform=$platform",
        '/m:1',
        '/v:m',
        '/nologo'
    )
    if ($ClangTidy) {
        # Rebuild is intentional: Visual Studio's integrated Clang-Tidy target
        # then receives the exact evaluated MSVC defines, include paths, PCH,
        # SDK, and per-file options for every translation unit.
        $checks = '-*%2Cclang-analyzer-deadcode.*%2Cbugprone-unused-return-value' +
            '%2Cperformance-for-range-copy' +
            '%2Cperformance-inefficient-string-concatenation' +
            '%2Cperformance-inefficient-vector-operation' +
            '%2Cperformance-move-const-arg' +
            '%2Cperformance-no-automatic-move' +
            '%2Cperformance-trivially-destructible' +
            '%2Cperformance-type-promotion-in-math-fn' +
            '%2Cperformance-unnecessary-copy-initialization' +
            '%2Cperformance-unnecessary-value-param' +
            '%2Creadability-redundant-*' +
            '%2Creadability-simplify-boolean-expr'
        $arguments += @(
            '/t:Rebuild',
            '/p:RunCppAnalysis=true',
            '/p:EnableMicrosoftCodeAnalysis=false',
            '/p:EnableClangTidyCodeAnalysis=true',
            "/p:ClangTidyChecks=$checks"
        )
    }

    $output = @(& $msbuild @arguments 2>&1 | ForEach-Object { [string]$_ })
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        $output | Select-Object -Last 80 | Write-Output
        exit $exitCode
    }

    if ($ClangTidy) {
        # MSBuild writes the canonical diagnostics to per-project ClangTidy
        # logs; console formatting varies between PowerShell/MSBuild versions.
        # Third-party sources remain outside the policy boundary.
        $rootPattern = [regex]::Escape($Root.TrimEnd('\', '/'))
        $logs = @(
            Get-ChildItem -LiteralPath (Join-Path $Root 'Driver'), (Join-Path $Root 'Overlay'), (Join-Path $Root 'Tests') `
                -Recurse -Filter '*.ClangTidy.log' -File -ErrorAction SilentlyContinue |
                Where-Object { $_.LastWriteTime -ge $analysisStarted.AddSeconds(-5) }
        )
        $findings = @(
            $logs |
                ForEach-Object { Get-Content -LiteralPath $_.FullName } |
                Where-Object {
                    $_ -match "^$rootPattern[\\/](Driver|Overlay|common|Tests)[\\/].*:\d+:\d+: (warning|error):"
                }
        )
        if ($findings.Count -gt 0) {
            Write-Output "Clang-Tidy reported $($findings.Count) first-party finding(s):"
            $findings | Write-Output
            # Tests still run after advisory findings. This matters for scheduled
            # deep validation, where there is no separate fast-build job and an
            # advisory must not mask a failing solver executable.
            $script:analysisAdvisory = $true
        }
    }

    return
}

function Invoke-SolverTests {
    if (-not (Test-Path -LiteralPath $testExecutable -PathType Leaf)) {
        Write-Output "Solver test executable not found after build: $testExecutable"
        exit 2
    }

    $output = @(& $testExecutable 2>&1 | ForEach-Object { [string]$_ })
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        $output | Write-Output
        # Normalize: the exe returns its failing-scenario count, which would
        # collide with this script's reserved codes (2 = config, 3 = advisory).
        exit 1
    }

    $summary = $output | Select-Object -Last 1
    Write-Output "Solver tests passed: $summary"
}

switch ($Mode) {
    'Build' {
        Invoke-MSBuildValidation
        Invoke-SolverTests
    }

    'Analyze' {
        # Fast scans already compile the affected solution. Whole-project
        # Clang-Tidy is reserved for Deep scans because it rebuilds and analyzes
        # every translation unit.
        if (-not $All) {
            Write-Output 'Clang-Tidy skipped in Fast mode; run the Deep scan for whole-project analysis.'
            exit 0
        }
        Invoke-MSBuildValidation -ClangTidy
        Invoke-SolverTests
        if ($script:analysisAdvisory) {
            exit 3
        }
    }

    'Duplicates' {
        Reset-ProcessPath
        $jscpd = Get-Command jscpd -ErrorAction SilentlyContinue
        if (-not $jscpd) {
            Write-Output 'jscpd is not installed; C++ clone detection was skipped.'
            exit 2
        }

        # High thresholds keep this signal focused on meaningful copy/paste
        # blocks. Results are advisory: similar code can encode intentionally
        # distinct invariants and requires human review before consolidation.
        $paths = @('Driver', 'Overlay', 'common', 'Tests') |
            ForEach-Object { Join-Path $Root $_ }
        $output = @(& $jscpd.Source `
            --min-lines $duplicateMinLines `
            --min-tokens $duplicateMinTokens `
            --mode weak `
            --format 'cpp,cpp-header,c' `
            --reporters console `
            --no-colors `
            --no-tips `
            --exit-code 3 `
            @paths 2>&1 | ForEach-Object { [string]$_ })
        $exitCode = $LASTEXITCODE
        if ($exitCode -ne 0) {
            $output | Write-Output
        }
        exit $exitCode
    }
}
