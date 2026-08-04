param(
    [int]$Duration = 30,
    [switch]$NoBrowser,
    [string]$Preset = "windows-ninja"
)

$ErrorActionPreference = "Stop"
$Root = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $Root

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    throw "cmake not found in PATH. Install CMake and Visual Studio Build Tools (Ninja may be used with MSVC)."
}

cmake --preset $Preset
cmake --build --preset $Preset

$BuildDir = if ($Preset -eq "windows-vs") { "build-win-vs" } else { "build-win" }
$BinDir = Join-Path $Root "$BuildDir\bin"
$LibDir = Join-Path $Root "$BuildDir\lib"

cmake -S (Join-Path $Root "modules\adas_nodes") `
      -B (Join-Path $Root "$BuildDir\modules\adas_nodes") `
      -DFLOWENGINE_BUILD="$Root\$BuildDir" `
      -DCMAKE_BUILD_TYPE=Release
cmake --build (Join-Path $Root "$BuildDir\modules\adas_nodes") --config Release

$env:PATH = "$BinDir;$LibDir;$env:PATH"
$env:FLOW_LOG_DIR = Join-Path $env:TEMP "flow_logs"
New-Item -ItemType Directory -Force -Path $env:FLOW_LOG_DIR | Out-Null

$Flowmond = Join-Path $BinDir "flowmond.exe"
$Launcher = Join-Path $BinDir "flow_launcher.exe"
$Html = Join-Path $Root "tools\flowboard\index.html"
$Config = Join-Path $Root "config\pipeline.json"

$flowmondProc = Start-Process -FilePath $Flowmond `
    -ArgumentList @("--html-path", $Html) `
    -PassThru -WindowStyle Minimized

Start-Sleep -Seconds 1

if (-not $NoBrowser) {
    Start-Process "http://localhost:8800"
}

try {
    & $Launcher $Config --duration $Duration
}
finally {
    if ($flowmondProc -and -not $flowmondProc.HasExited) {
        Stop-Process -Id $flowmondProc.Id
    }
}
