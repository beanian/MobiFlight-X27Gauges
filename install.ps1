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

# copy_fw_files.py only stamps the version into the board.json on a fresh _build, and it
# only runs when PlatformIO actually relinks the firmware. Clear the build output as well,
# otherwise an unchanged rebuild of the same version would leave no _build to install from.
foreach ($dir in "_build", "_dist", ".pio\build\x27gauges_mega") {
    if (Test-Path $dir) { Remove-Item -Recurse -Force $dir }
}

$env:VERSION = $Version
python -m platformio run -e x27gauges_mega
if ($LASTEXITCODE -ne 0) { throw "Firmware build failed" }

# Never touch the installed package unless there is a complete one to replace it with
$staged = "_build\X27Gauges\Community"
foreach ($f in "boards\x27gauges_mega.board.json", "devices\x27_postep_vid6606.device.json") {
    if (-not (Test-Path "$staged\$f")) { throw "Build did not produce $staged\$f, nothing installed" }
}
if (-not (Get-ChildItem "$staged\firmware" -Filter "x27gauges_mega_*.hex" -ErrorAction SilentlyContinue)) {
    throw "Build did not produce a firmware .hex in $staged\firmware, nothing installed"
}

$target = "$Connector\Community\X27Gauges"
if (Test-Path $target) { Remove-Item -Recurse -Force $target }
Copy-Item -Recurse $staged $target
Write-Host "`nInstalled firmware $Version to $target"
Write-Host "Restart MobiFlight Connector to pick it up."
