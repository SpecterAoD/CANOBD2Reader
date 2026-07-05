param([string]$Port = "")
$ErrorActionPreference = "Stop"
$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $RepoRoot

$args = @("run", "-e", "display", "-t", "upload")
if ($Port) { $args += @("--upload-port", $Port) }
platformio @args
