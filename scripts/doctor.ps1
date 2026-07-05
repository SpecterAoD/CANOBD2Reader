$ErrorActionPreference = "Continue"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

function Test-Cmd($Name, $CommandArgs = @("--version")) {
    Write-Host ""
    Write-Host "== $Name =="
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if (-not $cmd) {
        Write-Host "MISSING: $Name" -ForegroundColor Red
        return
    }
    Write-Host "Path: $($cmd.Source)"
    & $Name @CommandArgs
}

Test-Cmd "git" @("--version")
Test-Cmd "python" @("--version")
Test-Cmd "platformio" @("--version")
Test-Cmd "pio" @("--version")

Write-Host ""
Write-Host "== Python packages =="
python -c "import importlib.util; [print(name + ': ' + ('OK' if importlib.util.find_spec(name) else 'MISSING')) for name in ('serial','esptool','intelhex')]"

Write-Host ""
Write-Host "== PlatformIO environments =="
platformio project config --json-output 2>$null
