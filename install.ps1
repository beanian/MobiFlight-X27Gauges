# Builds the firmware and installs it as a Community device into MobiFlight Connector.
# Restart the Connector afterwards, then flash the Mega from Extras > Settings > MobiFlight Modules.
#
#   .\install.ps1                 # build 1.0.0 and install
#   .\install.ps1 -Version 1.0.1  # bump the version so the Connector offers a firmware update
param(
    [string]$Version = "1.0.0",
    [string]$Connector = "$env:LOCALAPPDATA\MobiFlight\MobiFlight Connector"
)
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not (Test-Path "$Connector\Community")) {
    throw "MobiFlight Connector not found at '$Connector' (pass -Connector <path>)"
}

# copy_fw_files.py only stamps the version into the board.json on a fresh _build
foreach ($dir in "_build", "_dist") {
    if (Test-Path $dir) { Remove-Item -Recurse -Force $dir }
}

$env:VERSION = $Version
python -m platformio run -e x27gauges_mega
if ($LASTEXITCODE -ne 0) { throw "Firmware build failed" }

$target = "$Connector\Community\X27Gauges"
if (Test-Path $target) { Remove-Item -Recurse -Force $target }
Copy-Item -Recurse "_build\X27Gauges\Community" $target
Write-Host "`nInstalled firmware $Version to $target"
Write-Host "Restart MobiFlight Connector to pick it up."
