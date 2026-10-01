# Exercises the exact helper shipped by Updater.cpp; no parent wait,
# elevation, installer launch, network access or installed files are touched.
$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot '..\Overlay\Updater.cpp')
$match = [regex]::Match($source, 'R"PS1\((.*?)\)PS1"', [Text.RegularExpressions.RegexOptions]::Singleline)
if (-not $match.Success) { throw 'Embedded update helper is missing.' }
$tokens = $null; $errors = $null
$ast = [Management.Automation.Language.Parser]::ParseInput($match.Groups[1].Value,[ref]$tokens,[ref]$errors)
if ($errors.Count) { throw ($errors | Out-String) }
$function = $ast.Find({ param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Expand-VerifiedUpdate' },$true)
if (-not $function) { throw 'Verified extraction function is missing.' }
. ([scriptblock]::Create($function.Extent.Text))
Add-Type -AssemblyName System.IO.Compression.FileSystem
$root = Join-Path ([IO.Path]::GetTempPath()) ('questcal-update-test-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $root | Out-Null
$checks = 0
function Require([bool]$value,[string]$why) { $script:checks++; if (-not $value) { throw $why } }
function Refuses([scriptblock]$action,[string]$why) { $refused=$false;try { & $action } catch { $refused=$true };Require $refused $why }
function Make-Zip([string]$Path,[string[]]$Names,[switch]$Link) {
    $archive = [IO.Compression.ZipFile]::Open($Path,[IO.Compression.ZipArchiveMode]::Create)
    try { foreach ($name in $Names) {
        $entry = $archive.CreateEntry($name)
        if ($Link) { $entry.ExternalAttributes = 0xa000 -shl 16 }
        $entryStream = $entry.Open()
        try { $data = [Text.Encoding]::UTF8.GetBytes('verified content');$entryStream.Write($data,0,$data.Length) } finally { $entryStream.Dispose() }
    }} finally { $archive.Dispose() }
}
try {
    $valid = Join-Path $root 'valid.zip'; Make-Zip $valid @('app/QuestCalibrator.exe','Install.ps1')
    $hash = (Get-FileHash -LiteralPath $valid -Algorithm SHA256).Hash
    $destination = Join-Path $root 'valid';Expand-VerifiedUpdate $valid $hash $destination
    Require ((Get-Content -Raw -LiteralPath (Join-Path $destination 'Install.ps1')) -eq 'verified content') 'Verified bytes were not extracted'
    Refuses { Expand-VerifiedUpdate $valid ('0'*64) (Join-Path $root 'wrong-hash') } 'Bad digest accepted'
    Require (-not (Test-Path -LiteralPath (Join-Path $root 'wrong-hash'))) 'Hash refusal mutated the destination'
    Refuses { Expand-VerifiedUpdate $valid $hash $destination } 'Existing staging directory reused'
    $index=0
    foreach ($name in @('../escape','/absolute','C:/absolute','app/../../escape','app/CON.txt','app/name.','app\..\escape','app/name?.txt')) {
        $bad = Join-Path $root "bad-$index.zip";Make-Zip $bad @($name)
        $badHash=(Get-FileHash -LiteralPath $bad -Algorithm SHA256).Hash
        Refuses { Expand-VerifiedUpdate $bad $badHash (Join-Path $root "bad-$index") } "Unsafe entry accepted: $name"
        $index++
    }
    $duplicate=Join-Path $root 'duplicate.zip';Make-Zip $duplicate @('app/a','app/A')
    Refuses { Expand-VerifiedUpdate $duplicate (Get-FileHash $duplicate -Algorithm SHA256).Hash (Join-Path $root 'duplicate') } 'Case-insensitive duplicate accepted'
    $linked=Join-Path $root 'linked.zip';Make-Zip $linked @('app/link') -Link
    Refuses { Expand-VerifiedUpdate $linked (Get-FileHash $linked -Algorithm SHA256).Hash (Join-Path $root 'linked') } 'Symbolic-link entry accepted'
    $entries=Join-Path $root 'too-many.zip';Make-Zip $entries (1..4097 | ForEach-Object { "entry-$_" })
    Refuses { Expand-VerifiedUpdate $entries (Get-FileHash $entries -Algorithm SHA256).Hash (Join-Path $root 'too-many') } 'Entry-count budget ignored'
    foreach ($scenario in @('entry','total')) {
        $large=Join-Path $root "large-$scenario.zip"
        $archive=[IO.Compression.ZipFile]::Open($large,[IO.Compression.ZipArchiveMode]::Create)
        try {
            $count=if ($scenario -eq 'entry') { 1 } else { 5 }
            foreach ($i in 1..$count) {
                $entry=$archive.CreateEntry("entry-$i");$stream=$entry.Open()
                try {
                    $blocks=if ($scenario -eq 'total' -and $i -eq 5) { 0 } else { 1024 }
                    $buffer=[byte[]]::new(65536)
                    for ($j=0;$j -lt $blocks;$j++) { $stream.Write($buffer,0,$buffer.Length) }
                    if ($scenario -eq 'entry' -or $i -eq 5) { $stream.WriteByte(0) }
                } finally { $stream.Dispose() }
            }
        } finally { $archive.Dispose() }
        Refuses { Expand-VerifiedUpdate $large (Get-FileHash $large -Algorithm SHA256).Hash (Join-Path $root "large-$scenario") } "Expanded $scenario budget ignored"
    }
    $parent=Join-Path $root 'linked-parent';New-Item -ItemType SymbolicLink -Path $parent -Target $destination | Out-Null
    Refuses { Expand-VerifiedUpdate $valid $hash (Join-Path $parent 'child') } 'Linked staging ancestor accepted'
    Remove-Item -LiteralPath $parent -Force
    Refuses { Expand-VerifiedUpdate $valid $hash 'relative-stage' } 'Relative staging path accepted'
    $truncated=Join-Path $root 'truncated.zip';[IO.File]::WriteAllBytes($truncated,[byte[]](1..32))
    Refuses { Expand-VerifiedUpdate $truncated (Get-FileHash $truncated -Algorithm SHA256).Hash (Join-Path $root 'truncated') } 'Malformed archive accepted'
    Write-Output "Update helper: $checks checks passed"
} finally { Remove-Item -LiteralPath $root -Recurse -Force -ErrorAction SilentlyContinue }
