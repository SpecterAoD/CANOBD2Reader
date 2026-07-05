param([string]$Port = "")
$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

$args = @("device", "monitor", "-e", "sender")
if ($Port) { $args += @("--port", $Port) }
platformio @args
