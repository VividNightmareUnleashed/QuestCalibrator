# Exercises the installer's actual conflict-removal block with mocked filesystem
# operations. Does not execute elevation, uninstallers, or any installation step.
$ErrorActionPreference = 'Stop'
# The block removes through the shared filesystem policy, as Install.ps1 does.
. (Join-Path $PSScriptRoot 'FilesystemPolicy.ps1')
$tokens = $null
$errors = $null
$ast = [Management.Automation.Language.Parser]::ParseFile(
    (Join-Path $PSScriptRoot 'Install.ps1'), [ref]$tokens, [ref]$errors)
if ($errors.Count) { throw ($errors | Out-String) }
$block = $ast.Find({ param($node)
    $node -is [Management.Automation.Language.ForEachStatementAst] -and
    $node.Variable.VariablePath.UserPath -eq 'conflict'
}, $true)
if (-not $block) { throw 'Conflict-removal block not found.' }
$removeConflicts = [scriptblock]::Create($block.Extent.Text)

function Test-Case([string]$name, [int]$failuresBeforeSuccess, [bool]$noop = $false, [bool]$present = $true) {
    $vrRuntimePath = Join-Path $env:TEMP 'QuestCalibrator-Mocked-SteamVR'
    $state = @{
        attempts = 0
        exists = $present
        nextStep = $false
        failed = $false
    }
    function Test-Path { param([string]$LiteralPath) return $state.exists }
    # The mocked folder has no real tree to vet; test-filesystem-policy.ps1
    # covers the checks themselves.
    function Assert-QuestcalTree { param([string]$Path, [string]$ExpectedLeaf) }
    function Remove-Item {
        param([string]$LiteralPath, [switch]$Recurse, [switch]$Force, [string]$ErrorAction)
        $state.attempts++
        if ($state.attempts -le $failuresBeforeSuccess) { throw 'Mocked file lock' }
        if (-not $noop) { $state.exists = $false }
    }
    function Start-Sleep { param([int]$Seconds) }
    function Write-Host { param([string]$Object) }
    function Fail { param([string]$msg) throw $msg }
    try {
        & $removeConflicts
        $state.nextStep = $true
    } catch {
        $state.failed = $true
    }
    $shouldFail = $present -and ($failuresBeforeSuccess -ge 3 -or $noop)
    if ($state.failed -ne $shouldFail -or $state.nextStep -eq $shouldFail -or
        ($shouldFail -and -not $state.exists)) {
        throw "FAIL $name : $($state | ConvertTo-Json -Compress)"
    }
    Write-Output "PASS $name (removal attempts: $($state.attempts))"
}

Test-Case 'no conflict' 0 -present $false
Test-Case 'removed immediately' 0
Test-Case 'retry succeeds' 2
Test-Case 'locked conflict aborts installation' 3
Test-Case 'remaining directory aborts installation' 0 -noop $true
