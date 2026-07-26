[CmdletBinding()]
param(
    [ValidateSet('Build', 'Analyze', 'Duplicates')]
    [string]$Mode = 'Build',
    [string]$Root = (Split-Path -Parent $PSScriptRoot),
    [switch]$All
)

$ErrorActionPreference = 'Stop'
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
$duplicateMinLines = [int]$config.duplicateMinLines
$duplicateMinTokens = [int]$config.duplicateMinTokens
if (-not $solution.StartsWith($Root, [System.StringComparison]::OrdinalIgnoreCase) -or
    -not $configuration -or -not $platform -or
    $duplicateMinLines -lt 1 -or $duplicateMinTokens -lt 1) {
    Write-Output "Invalid C++ validation config: $configPath"
    exit 2
}
$msbuild = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe'

function Reset-ProcessPath {
    # Some hosts can carry both Path and PATH. MSBuild copies environment
    # variables into a case-insensitive dictionary and rejects that duplicate.
    Remove-Item Env:Path -ErrorAction SilentlyContinue
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
        [Environment]::GetEnvironmentVariable('Path', 'User')
}

function Invoke-MSBuildValidation {
    param([switch]$ClangTidy)

    if (-not (Test-Path -LiteralPath $msbuild -PathType Leaf)) {
        Write-Output "MSBuild not found: $msbuild"
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
            exit 3
        }
    }

    exit 0
}

switch ($Mode) {
    'Build' {
        Invoke-MSBuildValidation
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
