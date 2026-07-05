$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

Write-Host "[ports] PlatformIO devices"
platformio device list

Write-Host ""
Write-Host "[ports] Windows serial devices"
Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name, PNPDeviceID | Format-Table -AutoSize
