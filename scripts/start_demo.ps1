param(
    [int]$Duration = 0,
    [string]$ServerHost = "127.0.0.1",
    [int]$Port = 8800,
    [switch]$NoBrowser,
    [switch]$KeepOldProcesses
)

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$bin = Join-Path $root "build\bin\Release"
$lib = Join-Path $root "build\lib\Release"
$flowmondExe = Join-Path $bin "flowmond.exe"
$launcherExe = Join-Path $bin "flow_launcher.exe"
$htmlPath = Join-Path $root "tools\flowboard\index.html"
$pipeline = Join-Path $root "config\pipeline_windows.json"

if (-not (Test-Path $flowmondExe)) {
    throw "flowmond.exe not found: $flowmondExe"
}
if (-not (Test-Path $launcherExe)) {
    throw "flow_launcher.exe not found: $launcherExe"
}
if (-not (Test-Path $pipeline)) {
    throw "pipeline_windows.json not found: $pipeline"
}

if (-not $KeepOldProcesses) {
    Get-Process flowmond, flow_launcher -ErrorAction SilentlyContinue | Stop-Process -Force
}

$env:PATH = "$bin;$lib;$env:PATH"
if (-not $env:FLOW_LOG_DIR) {
    $env:FLOW_LOG_DIR = Join-Path $env:TEMP "flow_logs"
}

Set-Location $root

Write-Host "[demo] starting flowmond..."
Start-Process $flowmondExe -ArgumentList "--html-path", $htmlPath | Out-Null

$baseUrl = "http://$ServerHost`:$Port"
$healthUrl = "$baseUrl/api/health"
$ready = $false

for ($i = 0; $i -lt 15; $i++) {
    Start-Sleep -Milliseconds 300
    try {
        $r = Invoke-WebRequest -Uri $healthUrl -UseBasicParsing -TimeoutSec 2
        if ($r.StatusCode -eq 200) {
            $ready = $true
            break
        }
    } catch {
    }
}

if (-not $ready) {
    throw "flowmond health check failed: $healthUrl"
}

Write-Host "[demo] flowmond ready at $baseUrl"

if (-not $NoBrowser) {
    Start-Process $baseUrl | Out-Null
}

if ($Duration -gt 0) {
    Write-Host "[demo] running flow_launcher for $Duration seconds..."
} else {
    Write-Host "[demo] running flow_launcher until stopped..."
}
& $launcherExe $pipeline --duration $Duration

Write-Host "[demo] finished"