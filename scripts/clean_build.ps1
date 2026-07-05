param(
    [switch]$SkipTests
)

$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

Write-Host "[clean-build] Removing .pio"
if (Test-Path ".pio") {
    Remove-Item -LiteralPath ".pio" -Recurse -Force
}

Write-Host "[clean-build] Build sender and display"
platformio run -e sender -e display

if (-not $SkipTests) {
    Write-Host "[clean-build] Native tests"
    platformio test -e native
}

python scripts/export_firmware_info.py --target all --output docs/AUTO_FIRMWARE_ARTIFACTS.md
Write-Host "[clean-build] OK"
