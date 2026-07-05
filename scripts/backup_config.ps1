param(
    [string]$Destination = ""
)

$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

if (-not $Destination) {
    $Destination = Join-Path $env:USERPROFILE "CANOBD2Reader-backup"
}

$stamp = Get-Date -Format "yyyyMMdd-HHmmss"
$out = Join-Path $Destination $stamp
New-Item -ItemType Directory -Path $out -Force | Out-Null

foreach ($file in @("include/secrets.h", "VERSION.txt", "platformio.ini")) {
    if (Test-Path $file) {
        Copy-Item $file -Destination $out -Force
    }
}

Write-Host "[backup-config] Backup written to $out"
