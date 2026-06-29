param(
    [switch]$SkipBuild,
    [int]$DisplayRetries = 3,
    [switch]$NoDisplayBootPrompt
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$pioExe = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\platformio.exe'

if (-not (Test-Path $pioExe)) {
    throw "PlatformIO executable not found at: $pioExe"
}

Push-Location $repoRoot
try {
    if (-not $SkipBuild) {
        Write-Host "==> Building sender + display"
        & $pioExe run -e sender -e display
        if ($LASTEXITCODE -ne 0) {
            throw "Build failed with exit code $LASTEXITCODE"
        }
    }

    Write-Host "==> Upload sender"
    & $pioExe run -e sender -t upload
    if ($LASTEXITCODE -ne 0) {
        throw "Sender upload failed with exit code $LASTEXITCODE"
    }

    if (-not $NoDisplayBootPrompt) {
        Write-Host "==> Prepare display bootloader mode"
        Write-Host "Hold BOOT, tap RST, release BOOT after 1-2 seconds."
        Read-Host "Press Enter when ready"
    }

    for ($attempt = 1; $attempt -le $DisplayRetries; $attempt++) {
        Write-Host "==> Upload display (attempt $attempt/$DisplayRetries)"
        & $pioExe run -e display -t upload
        if ($LASTEXITCODE -eq 0) {
            Write-Host "Display upload successful."
            exit 0
        }

        if ($attempt -lt $DisplayRetries) {
            Write-Warning "Display upload failed. Enter bootloader mode again: hold BOOT, tap RST, release BOOT after 1-2 seconds."
            Read-Host "Press Enter to retry display upload"
        }
    }

    throw "Display upload failed after $DisplayRetries attempts"
}
finally {
    Pop-Location
}
