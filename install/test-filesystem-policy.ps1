$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'FilesystemPolicy.ps1')
$root = Join-Path ([IO.Path]::GetTempPath()) ('questcal-policy-' + [Guid]::NewGuid().ToString('N'))
$owned = Join-Path $root 'QuestCalibrator'
$foreign = Join-Path $root 'OtherDriver'
$checks = 0
function Require([bool]$value, [string]$why) { $script:checks++; if (-not $value) { throw $why } }
function Refuses([scriptblock]$action, [string]$why) {
    $refused = $false; try { & $action } catch { $refused = $true }
    Require $refused $why
}
try {
    New-Item -ItemType Directory -Path $owned,$foreign | Out-Null
    Set-Content -LiteralPath (Join-Path $foreign 'keep.txt') -Value 'unrelated'
    Assert-QuestcalTree $owned 'QuestCalibrator'
    Refuses { Assert-QuestcalTree $foreign 'QuestCalibrator' } 'Foreign namespace accepted'
    Refuses { Assert-QuestcalTree ([IO.Path]::GetPathRoot($root)) } 'Filesystem root accepted'
    Refuses { Assert-QuestcalTree 'relative/QuestCalibrator' } 'Relative path accepted'
    $link = Join-Path $owned 'outside'
    New-Item -ItemType SymbolicLink -Path $link -Target $foreign | Out-Null
    Refuses { Remove-QuestcalTree $owned 'QuestCalibrator' } 'Linked tree accepted for recursive removal'
    Require (Test-Path -LiteralPath (Join-Path $foreign 'keep.txt')) 'Foreign tree changed'
    Remove-Item -LiteralPath $link -Force
    function Remove-Item { param($LiteralPath,[switch]$Recurse,[switch]$Force,$ErrorAction) }
    Refuses { Remove-QuestcalTree $owned 'QuestCalibrator' } 'No-op removal reported success'
    Microsoft.PowerShell.Management\Remove-Item Function:Remove-Item -Force -ErrorAction SilentlyContinue
    Remove-QuestcalTree $owned 'QuestCalibrator'
    Require (-not (Test-Path -LiteralPath $owned)) 'Owned tree was not removed'
    Require (Test-Path -LiteralPath $foreign) 'Unrelated tree was removed'
    foreach ($name in @('Install.ps1','Uninstall.ps1','FilesystemPolicy.ps1','build-package.ps1')) {
        $tokens = $null; $errors = $null
        [Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $name),[ref]$tokens,[ref]$errors) | Out-Null
        Require ($errors.Count -eq 0) "PowerShell syntax error in $name"
    }
    Write-Output "Filesystem policy: $checks checks passed"
} finally {
    Microsoft.PowerShell.Management\Remove-Item -LiteralPath $root -Recurse -Force -ErrorAction SilentlyContinue
}
