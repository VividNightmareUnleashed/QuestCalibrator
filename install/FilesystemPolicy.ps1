# Filesystem preconditions for owned install/removal trees. A concurrent process
# must not replace checked paths during the operation; Win32 path races are
# outside this sequential policy. Reparse points are refused, including parents.
function Assert-QuestcalTree([string]$Path, [string]$ExpectedLeaf = '') {
    if (-not $Path -or -not [IO.Path]::IsPathRooted($Path)) { throw 'An installation path must be absolute.' }
    $full = [IO.Path]::GetFullPath($Path).TrimEnd([IO.Path]::DirectorySeparatorChar, [IO.Path]::AltDirectorySeparatorChar)
    if (-not $full -or $full -eq [IO.Path]::GetPathRoot($Path).TrimEnd([IO.Path]::DirectorySeparatorChar)) { throw 'A filesystem root is not an installation directory.' }
    if ($ExpectedLeaf -and [IO.Path]::GetFileName($full) -ine $ExpectedLeaf) { throw "Refusing an unowned directory: $Path" }
    $ancestor = $full
    while ($ancestor) {
        if (Test-Path -LiteralPath $ancestor) {
            $item = Get-Item -LiteralPath $ancestor -Force -ErrorAction Stop
            if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Refusing a reparse point: $ancestor" }
        }
        $ancestor = [IO.Path]::GetDirectoryName($ancestor)
    }
    if (Test-Path -LiteralPath $full) {
        foreach ($item in Get-ChildItem -LiteralPath $full -Recurse -Force -ErrorAction Stop) {
            if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Refusing a reparse point: $($item.FullName)" }
        }
    }
}

function Remove-QuestcalTree([string]$Path, [string]$ExpectedLeaf) {
    Assert-QuestcalTree $Path $ExpectedLeaf
    if (Test-Path -LiteralPath $Path) { Remove-Item -LiteralPath $Path -Recurse -Force -ErrorAction Stop }
    if (Test-Path -LiteralPath $Path) { throw "Directory removal did not complete: $Path" }
}
