#requires -Version 7
# Scans the executable content of a release package on VirusTotal and writes a
# Markdown table for the release notes. Run after build-package.ps1:
#   $env:VT_API_KEY = '<your key>'
#   .\install\virustotal-scan.ps1                      # newest zip in install\out
#   .\install\virustotal-scan.ps1 -Package <zip> -NoUpload
#
# It hashes the files inside the zip itself, so the report covers exactly what
# ships. Each file is looked up by SHA-256 first and uploaded only when
# VirusTotal has never seen it. Uploaded files are shared with VirusTotal's
# security partners; only run this on a package you are about to publish.
# The free API allows 4 requests a minute, so the script paces itself.
[CmdletBinding()]
param(
    [string]$Package = '',
    # Only look up hashes; never upload.
    [switch]$NoUpload,
    # Seconds between API requests (free tier: 4 per minute).
    [int]$RequestDelay = 16,
    # Minutes to wait for a fresh upload's analysis to finish.
    [int]$AnalysisTimeout = 15
)

$ErrorActionPreference = 'Stop'

$apiKey = $env:VT_API_KEY
if (-not $apiKey) { throw 'Set $env:VT_API_KEY to your VirusTotal API key first.' }
$headers = @{ 'x-apikey' = $apiKey }
$api = 'https://www.virustotal.com/api/v3'

if (-not $Package) {
    $Package = Get-ChildItem (Join-Path $PSScriptRoot 'out') -Filter '*.zip' -File |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1 -ExpandProperty FullName
    if (-not $Package) { throw 'No package in install\out. Run build-package.ps1 first.' }
}
$Package = (Resolve-Path -LiteralPath $Package).Path
Write-Host "Scanning $Package" -ForegroundColor Cyan

$script:lastRequest = [datetime]::MinValue
function Invoke-VT {
    param([string]$Method = 'Get', [string]$Uri, [hashtable]$Form)
    $wait = $RequestDelay - ((Get-Date) - $script:lastRequest).TotalSeconds
    if ($wait -gt 0) { Start-Sleep -Seconds ([math]::Ceiling($wait)) }
    $script:lastRequest = Get-Date
    $params = @{ Method = $Method; Uri = $Uri; Headers = $headers; SkipHttpErrorCheck = $true
                 StatusCodeVariable = 'status' }
    if ($Form) { $params.Form = $Form }
    $body = Invoke-RestMethod @params
    if ($status -eq 404) { return $null }
    if ($status -ge 400) { throw "VirusTotal returned $status for $Uri`: $($body.error.message)" }
    $body
}

function Format-Stats($stats) {
    $flagged = $stats.malicious + $stats.suspicious
    $total = $stats.malicious + $stats.suspicious + $stats.undetected + $stats.harmless
    "$flagged / $total"
}

# The scanned set: every binary and script in the package, plus the zip itself.
$extract = Join-Path ([IO.Path]::GetTempPath()) ("qc-vt-" + [guid]::NewGuid())
Expand-Archive -LiteralPath $Package -DestinationPath $extract
try {
    $files = @(Get-ChildItem -LiteralPath $extract -Recurse -File |
        Where-Object Extension -in '.exe', '.dll', '.ps1' |
        Sort-Object FullName |
        ForEach-Object {
            [pscustomobject]@{
                Name = $_.FullName.Substring($extract.Length + 1).Replace('\', '/') -replace '^[^/]+/', ''
                Path = $_.FullName
            }
        })
    $files += [pscustomobject]@{ Name = Split-Path -Leaf $Package; Path = $Package }

    $rows = foreach ($f in $files) {
        $hash = (Get-FileHash -LiteralPath $f.Path -Algorithm SHA256).Hash.ToLowerInvariant()
        Write-Host "$($f.Name)  $hash"
        $report = Invoke-VT -Uri "$api/files/$hash"
        $results = $null
        if ($report) {
            $stats = $report.data.attributes.last_analysis_stats
            $results = $report.data.attributes.last_analysis_results
        } elseif ($NoUpload) {
            Write-Host '  not on VirusTotal yet (skipped: -NoUpload)' -ForegroundColor Yellow
            $stats = $null
        } else {
            Write-Host '  uploading...'
            $upload = Invoke-VT -Method Post -Uri "$api/files" -Form @{ file = Get-Item -LiteralPath $f.Path }
            $deadline = (Get-Date).AddMinutes($AnalysisTimeout)
            do {
                $analysis = Invoke-VT -Uri "$api/analyses/$($upload.data.id)"
            } until ($analysis.data.attributes.status -eq 'completed' -or (Get-Date) -gt $deadline)
            if ($analysis.data.attributes.status -ne 'completed') {
                Write-Host '  analysis still running; re-run later for final numbers' -ForegroundColor Yellow
            }
            $stats = $analysis.data.attributes.stats
            $results = $analysis.data.attributes.results
        }
        $result = if ($stats) { Format-Stats $stats } else { 'not scanned' }
        Write-Host "  detections: $result"
        # Which engine said what, so the notes can name each detection.
        $flags = @(if ($results) {
            $results.PSObject.Properties |
                Where-Object { $_.Value.category -in 'malicious', 'suspicious' } |
                ForEach-Object { [pscustomobject]@{ Engine = $_.Name; Label = $_.Value.result } }
        })
        $flags | ForEach-Object { Write-Host "    $($_.Engine): $($_.Label)" -ForegroundColor Yellow }
        [pscustomobject]@{ Name = $f.Name; Hash = $hash; Result = $result; Flags = $flags }
    }
} finally {
    Remove-Item -LiteralPath $extract -Recurse -Force -ErrorAction SilentlyContinue
}

$md = @(
    '| File | SHA-256 | Detections | VirusTotal |'
    '| --- | --- | --- | --- |'
    $rows | ForEach-Object {
        "| ``$($_.Name)`` | ``$($_.Hash)`` | $($_.Result) | [report](https://www.virustotal.com/gui/file/$($_.Hash)) |"
    }
)

# Every detection is named in the notes, with what its kind of label means,
# rather than left as a bare count. Whether it is a false positive is for the
# person publishing to find out and say.
$flagged = @($rows | ForEach-Object { $row = $_; $row.Flags | ForEach-Object {
    [pscustomobject]@{ File = $row.Name; Engine = $_.Engine; Label = $_.Label } } })
if ($flagged) {
    $md += ''
    $md += 'Detections at the time of the scan:'
    $md += ''
    $md += $flagged | ForEach-Object { "- ``$($_.File)``: $($_.Engine), ``$($_.Label)``" }
    # Labels from a machine-learning model or a heuristic, rather than from a
    # match to a known sample: Microsoft's !ml suffix, and the generic names
    # other engines give such verdicts.
    $modelLabel = '!ml\b|\bML\b|Heur|Generic|\bAI\b|Suspicious|Malicious_confidence|Static AI'
    if ($flagged | Where-Object { $_.Label -match $modelLabel }) {
        $md += ''
        $md += ('A label ending in `!ml`, or naming a heuristic or generic verdict, comes from a model ' +
            'that judges a file by its features, not from a match to known malware. New, unsigned ' +
            'builds like these are often flagged this way until they have a reputation.')
    }
    if ($flagged | Where-Object Engine -eq 'Microsoft') {
        $md += ''
        $md += ('Microsoft''s engine is the one in Microsoft Defender, so Windows may warn about or ' +
            'quarantine the flagged file.')
    }
    $md += ''
    $md += ('Each SHA-256 above, and `SHA256SUMS.txt` in the package, lets you check that your download ' +
        'is the file that was scanned, and the source it was built from is in this repository at ' +
        'the release''s tag.')
}
$mdPath = [IO.Path]::ChangeExtension($Package, '.virustotal.md')
[IO.File]::WriteAllLines($mdPath, [string[]]$md)
Write-Host ""
$md | ForEach-Object { Write-Host $_ }
Write-Host ""
Write-Host "Release-notes table: $mdPath" -ForegroundColor Green
