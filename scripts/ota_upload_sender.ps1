param([Parameter(Mandatory = $true)][string]$Ip)
$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

platformio run -e sender -t upload --upload-port $Ip --upload-protocol espota
