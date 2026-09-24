# CI only: checks out the private VirtualQuest submodule at the commit this tree
# pins, with the read-only deploy key in $env:VIRTUALQUEST_DEPLOY_KEY.
#
# Not actions/checkout's ssh-key: on the Windows runner its OpenSSH waited for a
# passphrase on a key that had picked up CRLF line endings on the way into the
# secret, and the job hung until cancelled. Here the key is normalized, Git's
# own ssh runs in batch mode (it fails instead of prompting), and github.com's
# host key is pinned rather than accepted on first use.
$ErrorActionPreference = 'Stop'
if (-not $env:VIRTUALQUEST_DEPLOY_KEY) { throw 'VIRTUALQUEST_DEPLOY_KEY is empty.' }

$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$sha = (git -C $root rev-parse HEAD:VirtualQuest).Trim()
$temp = if ($env:RUNNER_TEMP) { $env:RUNNER_TEMP } else { [IO.Path]::GetTempPath() }
$key = Join-Path $temp ('vq-' + [guid]::NewGuid().ToString('N'))
$known = "$key.known_hosts"

# https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/githubs-ssh-key-fingerprints
[IO.File]::WriteAllText($known,
    "github.com ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIOMqqnkVzrm0SdG6UOoqKLsabgH5C9okWi0dh2l9GKJl`n")
[IO.File]::WriteAllText($key, (($env:VIRTUALQUEST_DEPLOY_KEY -replace "`r", '').Trim() + "`n"))

$ssh = Join-Path ${env:ProgramFiles} 'Git\usr\bin\ssh.exe'
if (-not (Test-Path $ssh)) { $ssh = 'ssh' }
$unix = { param($p) $p -replace '\\', '/' }
$env:GIT_SSH_COMMAND = ('"{0}" -i "{1}" -o BatchMode=yes -o IdentitiesOnly=yes -o StrictHostKeyChecking=yes -o UserKnownHostsFile="{2}" -o ConnectTimeout=30' -f
    (& $unix $ssh), (& $unix $key), (& $unix $known))

$target = Join-Path $root 'VirtualQuest'
try
{
    git init --quiet $target
    git -C $target remote remove origin 2>$null
    git -C $target remote add origin git@github.com:VividNightmareUnleashed/VirtualQuest.git
    git -C $target fetch --quiet --depth 1 origin $sha
    if ($LASTEXITCODE -ne 0) { throw "Fetching VirtualQuest $sha failed." }
    git -C $target checkout --quiet --detach FETCH_HEAD
    if ($LASTEXITCODE -ne 0) { throw "Checking out VirtualQuest $sha failed." }
    Write-Host "VirtualQuest at $sha"
}
finally
{
    Remove-Item -LiteralPath $key, $known -Force -ErrorAction SilentlyContinue
    Remove-Item Env:GIT_SSH_COMMAND -ErrorAction SilentlyContinue
}
