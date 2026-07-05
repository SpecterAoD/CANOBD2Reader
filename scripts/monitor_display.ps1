param([string]$Port = "")
$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

$args = @("device", "monitor", "-e", "display")
if ($Port) { $args += @("--port", $Port) }
platformio @args
