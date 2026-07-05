param(
    [switch]$SkipCheck,
    [switch]$SkipTests
)

$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

Write-Host "[dev-check] Repository: $RepoRoot"
python scripts/check_secrets.py
python scripts/check_line_endings.py
python scripts/verify_docs.py

if (-not $SkipCheck) {
    Write-Host "[dev-check] PlatformIO static checks"
    platformio check -e sender
    platformio check -e display
}

Write-Host "[dev-check] Build sender"
platformio run -e sender

Write-Host "[dev-check] Build display"
platformio run -e display

if (-not $SkipTests) {
    Write-Host "[dev-check] Native tests"
    platformio test -e native
}

python scripts/export_firmware_info.py --target all --output docs/AUTO_FIRMWARE_ARTIFACTS.md
python scripts/generate_project_index.py
python scripts/generate_build_docs.py
python scripts/generate_config_reference.py
python scripts/generate_test_overview.py

Write-Host "[dev-check] OK"
