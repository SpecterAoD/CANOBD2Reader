param(
    [Parameter(Mandatory = $true)][string]$Version
)

$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

if ($Version -notmatch '^[0-9]+\.[0-9]+\.[0-9]+-beta\.[0-9]+$') {
    throw "Use version format x.y.z-beta.n, for example 2.0.0-beta.1"
}

"V$Version" | Set-Content -Path VERSION.txt -NoNewline
Write-Host "[prepare-beta] VERSION.txt set to V$Version"
python scripts/make_release_notes.py --version "V$Version" --channel beta --target all --output docs/AUTO_RELEASE_NOTES_PREVIEW.md
