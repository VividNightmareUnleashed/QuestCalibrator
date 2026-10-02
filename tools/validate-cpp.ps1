[CmdletBinding()]
param(
    [ValidateSet('Build', 'Compile', 'Analyze', 'Duplicates')]
    [string]$Mode = 'Build',
    [string]$Root = '',
    [switch]$All,
    # With -Mode Analyze, rebuild and analyze one project (its folder) instead
    # of the solution, so CI can analyze the three on separate runners.
    [ValidateSet('Driver', 'Overlay', 'Tests')]
    [string]$Project = '',
    # With -Project, analyze only the k-th of n shares of its files ('k/n'),
    # so CI can spread one project over runners too.
    [string]$Shard = ''
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
$testProject = [System.IO.Path]::GetFullPath(
    (Join-Path $Root ([string]$config.testProject)))
$duplicateMinLines = [int]$config.duplicateMinLines
$duplicateMinTokens = [int]$config.duplicateMinTokens
$minScenarios = [int]$config.minScenarios
if (-not $solution.StartsWith($Root, [System.StringComparison]::OrdinalIgnoreCase) -or
    -not [string]$config.testExecutable -or
    -not $testExecutable.StartsWith($Root, [System.StringComparison]::OrdinalIgnoreCase) -or
    -not [string]$config.testProject -or
    -not $testProject.StartsWith($Root, [System.StringComparison]::OrdinalIgnoreCase) -or
    -not $configuration -or -not $platform -or
    $duplicateMinLines -lt 1 -or $duplicateMinTokens -lt 1 -or $minScenarios -lt 1) {
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

# A share of a project's first-party translation units: every n-th, in project
# order, as MSBuild evaluates them. The vendored sources under lib\ are never
# analyzed, and a file that creates the precompiled header is compiled, and so
# analyzed, in every share.
$projectFiles = @{ Driver = 'Driver\QuestCalibratorDriver.vcxproj'; Overlay = 'Overlay\QuestCalibrator.vcxproj'; Tests = 'Tests\SolverTests.vcxproj' }
function Get-AnalysisShare {
    if (-not $Project -or $Shard -notmatch '^(\d+)/(\d+)$' -or [int]$Matches[1] -lt 1 -or [int]$Matches[1] -gt [int]$Matches[2]) {
        Write-Output "-Shard takes k/n with 1 <= k <= n, and needs -Project; got '$Shard'."
        exit 2
    }
    $k = [int]$Matches[1]
    $n = [int]$Matches[2]
    $ErrorActionPreference = 'Continue'
    $json = & $msbuild (Join-Path $Root $projectFiles[$Project]) -nologo -getItem:ClCompile "-p:Configuration=$configuration" `
        "-p:Platform=$platform" 2>&1 | Out-String
    try { $items = @(($json | ConvertFrom-Json).Items.ClCompile) }
    catch { Write-Output "MSBuild did not list $Project's files (MSBuild 17.8 or later is needed): $json"; exit 2 }
    $firstParty = @($items | Where-Object { $_.ExcludedFromBuild -ne 'true' -and $_.Identity -notlike '..\lib\*' })
    $pch = @($firstParty | Where-Object { $_.PrecompiledHeader -eq 'Create' })
    $rest = @($firstParty | Where-Object { $_.PrecompiledHeader -ne 'Create' })
    $share = @(for ($i = $k - 1; $i -lt $rest.Count; $i += $n) { $rest[$i].Identity })
    if ($share.Count -eq 0) {
        Write-Output "Share $Shard of $Project has no files to analyze."
        exit 2
    }
    return [pscustomobject]@{ Files = $share; Expected = $share.Count + $pch.Count }
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
        # The three projects declare no inter-dependencies, so MSBuild can build
        # them concurrently. Per-project ClangTidy logs are still written per
        # project, which is what the tidy scan below reads — console interleaving
        # does not affect it.
        '/m',
        '/v:m',
        '/nologo'
    )
    if ($ClangTidy) {
        # Eigen exceeds the 32-bit analyzer's address space. Select the binary
        # directly: VS can override PreferredToolArchitecture for this toolset.
        $clangTidyDirectory = [System.IO.Path]::GetFullPath(
            (Join-Path (Split-Path -Parent $msbuild) '../../../VC/Tools/Llvm/x64/bin'))
        if (-not (Test-Path -LiteralPath (Join-Path $clangTidyDirectory 'clang-tidy.exe'))) {
            Write-Output 'The VS x64 Clang-Tidy tools are required for analysis.'
            exit 2
        }
        # Rebuild is intentional: Visual Studio's integrated Clang-Tidy target
        # then receives the exact evaluated MSVC defines, include paths, PCH,
        # SDK, and per-file options for every translation unit.
        # clang-analyzer-core/cplusplus carry the null-deref, uninitialized-read
        # and use-after-free checks; the bugprone entries below are the logic
        # checks that matter in this codebase specifically — memory manipulation
        # of the pose/transform structs (invariant 1 forbids memsetting the
        # slots) and the bounded retry loops on the pose path.
        $checks = '-*%2Cclang-analyzer-core.*%2Cclang-analyzer-cplusplus.*' +
            '%2Cclang-analyzer-deadcode.*%2Cbugprone-unused-return-value' +
            '%2Cbugprone-use-after-move' +
            '%2Cbugprone-dangling-handle' +
            '%2Cbugprone-infinite-loop' +
            '%2Cbugprone-integer-division' +
            '%2Cbugprone-sizeof-expression' +
            '%2Cbugprone-suspicious-memset-usage' +
            '%2Cbugprone-undefined-memory-manipulation' +
            '%2Cbugprone-macro-parentheses' +
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
        # The solution's own target for one project, so its paths stay those of
        # a solution build. A share compiles only its files, which is how
        # Visual Studio analyzes selected files; the list goes through a
        # response file so its semicolons stay inside the property.
        $solutionProjects = @{ Driver = 'QuestCalibratorDriver'; Overlay = 'QuestCalibrator'; Tests = 'SolverTests' }
        $share = $null
        $responseFile = $null
        if ($Shard) {
            $share = Get-AnalysisShare
            $responseFile = [IO.Path]::GetTempFileName()
            [IO.File]::WriteAllText($responseFile, "/p:SelectedFiles=`"$($share.Files -join ';')`"`r`n")
            $arguments[0] = Join-Path $Root $projectFiles[$Project]
            $arguments += @('/t:ClCompile', "@$responseFile")
        }
        elseif ($Project) {
            $arguments += "/t:$($solutionProjects[$Project]):Rebuild"
        }
        else {
            $arguments += '/t:Rebuild'
        }
        # Clang-Tidy reports nothing from a header unless its path matches this
        # filter, and most of the overlay's logic is header-only. It names the
        # first-party folders by their last component, so a header reached
        # through ..\lib\ (Eigen, OpenVR, ImGui) stays out of it.
        $headerFilter = '.*[\\/]([Dd]river|[Oo]verlay|[Cc]ommon|[Tt]ests|[Ff]uzz)[\\/][^\\/]+$'
        $arguments += @(
            "/p:ClangTidyToolPath=$clangTidyDirectory",
            # The Windows SDK otherwise uses MSVC's non-constant offsetof
            # extension, which Clang cannot use in static layout assertions.
            '/p:ClangTidyToolExeAdditionalOptions=--extra-arg=-D_CRT_USE_BUILTIN_OFFSETOF',
            '/p:RunCppAnalysis=true',
            '/p:EnableMicrosoftCodeAnalysis=false',
            '/p:EnableClangTidyCodeAnalysis=true',
            "/p:ClangTidyChecks=$checks",
            "/p:ClangTidyHeaderFilter=$headerFilter"
        )
    }

    $output = @(& $msbuild @arguments 2>&1 | ForEach-Object { [string]$_ })
    $exitCode = $LASTEXITCODE
    if ($responseFile) { Remove-Item -LiteralPath $responseFile -ErrorAction SilentlyContinue }
    if ($exitCode -ne 0) {
        $output | Select-Object -Last 80 | Write-Output
        exit $exitCode
    }

    if ($ClangTidy) {
        # MSBuild writes the canonical diagnostics to per-project ClangTidy
        # logs; console formatting varies between PowerShell/MSBuild versions.
        # Third-party sources remain outside the policy boundary.
        $projectDirectories = @($(if ($Project) { $Project } else { 'Driver', 'Overlay', 'Tests' }) | ForEach-Object { Join-Path $Root $_ })
        $logs = @(
            Get-ChildItem -LiteralPath $projectDirectories `
                -Recurse -Filter '*.ClangTidy.log' -File -ErrorAction SilentlyContinue |
                Where-Object { $_.LastWriteTime -ge $analysisStarted.AddSeconds(-5) }
        )
        # A build that analyzed nothing writes no logs, and would otherwise pass
        # with no findings. Each project writes one, naming every file processed.
        $unanalyzed = @($projectDirectories | Where-Object {
            $directory = $_ + [System.IO.Path]::DirectorySeparatorChar
            -not ($logs | Where-Object { $_.FullName.StartsWith($directory, [StringComparison]::OrdinalIgnoreCase) })
        })
        # Clang-Tidy numbers its files ("[2/7] Processing file ...") only when it
        # has more than one, so a log without those lines is one analyzed file.
        $analyzed = 0
        foreach ($log in $logs) {
            $processed = @(Get-Content -LiteralPath $log.FullName |
                Where-Object { $_ -match '^\[\d+/\d+\] Processing file ' }).Count
            $analyzed += [Math]::Max(1, $processed)
        }
        if ($unanalyzed.Count -gt 0 -or $analyzed -eq 0) {
            Write-Output ('Clang-Tidy did not analyze every project (no log from: ' +
                (($unanalyzed | ForEach-Object { Split-Path -Leaf $_ }) -join ', ') +
                "; $analyzed file(s) processed in all).")
            exit 2
        }
        # A share must analyze exactly its files: fewer would pass unseen.
        if ($share -and $analyzed -ne $share.Expected) {
            Write-Output "Clang-Tidy analyzed $analyzed file(s) of share $Shard of $Project, not the $($share.Expected) it selected."
            exit 2
        }
        Write-Output ("Clang-Tidy analyzed $analyzed translation unit(s) in $($logs.Count) project(s)" +
            $(if ($share) { " (share $Shard of $Project)." } else { '.' }))
        # A header's path keeps the ..\ it was included through, so each finding's
        # path is resolved before it is judged first-party, and a header finding
        # every translation unit including it repeats is listed once. The
        # VirtualQuest scenarios build into the harness, so their findings count.
        $firstPartyPrefixes = @('Driver', 'Overlay', 'common', 'Tests', 'VirtualQuest\Tests') |
            ForEach-Object { [System.IO.Path]::GetFullPath((Join-Path $Root $_)).TrimEnd('\') + '\' }
        $findings = @(
            $logs |
                ForEach-Object { Get-Content -LiteralPath $_.FullName } |
                ForEach-Object {
                    if ($_ -notmatch '^(?<path>.+?)(?<rest>:\d+:\d+: (warning|error):.*)$') { return }
                    $rest = $Matches.rest
                    try { $path = [System.IO.Path]::GetFullPath($Matches.path) } catch { return }
                    if ($firstPartyPrefixes | Where-Object { $path.StartsWith($_, [StringComparison]::OrdinalIgnoreCase) }) {
                        $path + $rest
                    }
                } |
                Sort-Object -Unique
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

    # A Test-Path alone cannot tell a fresh harness from one that detached from
    # the code under test — if the Tests project ever stops building as part of
    # the solution, the build still succeeds and an old executable runs green.
    # Compare against the project's own inputs rather than the whole tree: most
    # of Overlay/ and Driver/ is not compiled by the harness, and editing a file
    # it does not build must not read as staleness. Comparing against sources
    # rather than the build start also lets a no-op incremental build pass.
    $exeWritten = (Get-Item -LiteralPath $testExecutable).LastWriteTime
    if (-not (Test-Path -LiteralPath $testProject -PathType Leaf)) {
        Write-Output "Test project not found: $testProject"
        exit 2
    }
    $projectDir = Split-Path -Parent $testProject
    [xml]$projectXml = Get-Content -LiteralPath $testProject
    $projectInputs = @(
        $projectXml.Project.ItemGroup.ClCompile
        $projectXml.Project.ItemGroup.ClInclude
    ) | ForEach-Object { $_.Include } |
        Where-Object { $_ } |
        ForEach-Object { Join-Path $projectDir $_ }
    # The project file itself is deliberately NOT compared: editing its
    # ClInclude list changes nothing about compilation, so MSBuild correctly
    # relinks nothing and the executable is legitimately older than it. A
    # project change that DOES matter - adding or removing a ClCompile - changes
    # which objects link and so produces a newer executable anyway.
    $newestSource = @($projectInputs) |
        Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } |
        Get-Item |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($newestSource -and $newestSource.LastWriteTime -gt $exeWritten) {
        Write-Output ("Solver test executable is older than $($newestSource.FullName); " +
            'it did not rebuild and would report on stale code.')
        exit 2
    }

    # The IPC rejection tests intentionally log to stderr. Windows PowerShell
    # promotes native stderr records to non-terminating ErrorRecords; with the
    # script-wide Stop policy that aborts the validation before the executable's
    # real exit code and summary can be checked.
    $previousErrorAction = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $output = @(& $testExecutable 2>&1 | ForEach-Object { [string]$_ })
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousErrorAction
    }
    if ($exitCode -ne 0) {
        $output | Write-Output
        # Normalize: the exe returns its failing-scenario count, which would
        # collide with this script's reserved codes (2 = config, 3 = advisory).
        exit 1
    }

    $summary = $output | Select-Object -Last 1
    # A zero exit means "nothing failed", which is also what an empty run
    # reports. Assert the harness actually ran its scenarios, so deleting a
    # Run*Scenarios() call fails here instead of passing quietly.
    if ($summary -notmatch '^(\d+) scenario\(s\) ran, \d+ failed$') {
        Write-Output "Could not read a scenario count from the harness summary: $summary"
        exit 2
    }
    $ran = [int]$Matches[1]
    if ($ran -lt $minScenarios) {
        Write-Output "Solver harness ran $ran scenario(s), below the floor of $minScenarios."
        Write-Output 'Coverage regressed, or the floor in cpp-validation.json needs raising.'
        exit 1
    }

    Write-Output "Solver tests passed: $summary"
}

switch ($Mode) {
    'Build' {
        Invoke-MSBuildValidation
        Invoke-SolverTests
    }

    # The Release build alone, without the solver harness.
    'Compile' {
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
        # Only a build of the test project has a harness to run, and a share
        # compiles without linking.
        if (-not $Shard -and (-not $Project -or $Project -eq 'Tests')) {
            Invoke-SolverTests
        }
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
