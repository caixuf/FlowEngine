param(
    [int]$Duration = 30,
    [switch]$NoBrowser,
    [switch]$SkipBuild,
    [string]$Preset = "windows-mingw",
    [string]$Pipeline = ""
)

$ErrorActionPreference = "Stop"
# Windows: CRT maps "/tmp/..." -> "<drive>:\tmp\..." (cwd drive). Ensure it exists.
foreach ($d in @('D:\tmp','C:\tmp', (Join-Path $env:TEMP 'flow_logs'))) {
    New-Item -ItemType Directory -Force -Path $d | Out-Null
}
$env:TMP = if (Test-Path 'D:\tmp') { 'D:\tmp' } else { $env:TEMP }
$env:TEMP = $env:TMP
$env:FLOW_LOG_DIR = Join-Path $env:TMP 'flow_logs'
New-Item -ItemType Directory -Force -Path $env:FLOW_LOG_DIR | Out-Null
$Root = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $Root

function Add-PathFront([string]$p) {
    if ($p -and (Test-Path $p) -and (($env:PATH -split ';') -notcontains $p)) {
        $env:PATH = "$p;$env:PATH"
    }
}

function Find-BinDir([string[]]$candidates) {
    foreach ($c in $candidates) {
        if ($c -and (Test-Path $c)) { return $c }
    }
    return $null
}

# 鈹€鈹€ Toolchain discovery (MinGW primary) 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
$winlibs = Join-Path $Root ".tools\winlibs\mingw64\bin"
Add-PathFront $winlibs

$cmakeBin = Find-BinDir @(
    "C:\Program Files\CMake\bin",
    "C:\Program Files (x86)\CMake\bin"
)
Add-PathFront $cmakeBin

$ninjaHint = Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Filter ninja.exe -Recurse -ErrorAction SilentlyContinue |
    Select-Object -First 1 -ExpandProperty DirectoryName
Add-PathFront $ninjaHint
Add-PathFront (Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Links")

$pyHint = Find-BinDir @(
    (Join-Path $env:LOCALAPPDATA "Programs\Python\Python312"),
    (Join-Path $env:LOCALAPPDATA "Programs\Python\Python311"),
    "C:\Python312",
    "C:\Python311"
)
Add-PathFront $pyHint

if (Test-Path $winlibs) {
    $parts = $env:PATH -split ';' | Where-Object {
        $_ -and ($_ -notmatch '(?i)[\\/]mingw64[\\/]mingw64[\\/]bin$') -and ($_ -notmatch '(?i)^C:\\mingw64\\')
    }
    $env:PATH = ($winlibs + ';' + ($parts -join ';'))
}

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    throw "cmake not found. Install CMake and ensure it is on PATH."
}

$isMingw = $Preset -eq "windows-mingw"
$isVs = $Preset -eq "windows-vs"
if ($isMingw) {
    if (-not (Get-Command gcc -ErrorAction SilentlyContinue) -or -not (Get-Command g++ -ErrorAction SilentlyContinue)) {
        throw @"
MinGW g++/gcc not found.
Place WinLibs under: $winlibs
  or install WinLibs (POSIX+UCRT, GCC>=11) and put its bin/ on PATH.
  Download: https://github.com/brechtsanders/winlibs_mingw/releases
"@
    }
    $ver = (& g++ -dumpversion)
    Write-Host "[demo] MinGW g++ $ver  (primary toolchain)"
    if (-not (Get-Command ninja -ErrorAction SilentlyContinue)) {
        throw "ninja not found (required by windows-mingw preset)."
    }
}

$BuildDirName = switch ($Preset) {
    "windows-vs" { "build-win-vs" }
    "windows-mingw" { "build-win" }
    "windows-ninja" { "build-win-ninja" }
    default { "build-win" }
}
$BuildDir = Join-Path $Root $BuildDirName

if (-not $SkipBuild) {
    Write-Host "[demo] configure preset=$Preset ..."
    if ($isMingw) {
        cmake --preset $Preset
    } elseif ($isVs) {
        cmake --preset $Preset -DCMAKE_BUILD_TYPE=Release
    } else {
        cmake --preset $Preset
    }
    Write-Host "[demo] build core ..."
    if ($isVs) {
        cmake --build --preset $Preset --config Release
    } else {
        cmake --build --preset $Preset
    }

    Write-Host "[demo] build node plugins ..."
    $nodesB = Join-Path $BuildDir "modules\adas_nodes"
    $cfgArgs = @(
        "-S", (Join-Path $Root "modules\adas_nodes"),
        "-B", $nodesB,
        "-G", $(if ($isVs) { "Visual Studio 17 2022" } else { "Ninja" }),
        "-DFLOWENGINE_BUILD=$BuildDir",
        "-DCMAKE_BUILD_TYPE=Release"
    )
    if ($isMingw) {
        $cfgArgs += @("-DCMAKE_C_COMPILER=gcc", "-DCMAKE_CXX_COMPILER=g++")
    }
    & cmake @cfgArgs
    if ($isVs) {
        cmake --build $nodesB --config Release
    } else {
        cmake --build $nodesB
    }
} else {
    Write-Host "[demo] SkipBuild: using existing artifacts in $BuildDirName"
}

$BinDir = $null
$LibDir = $null
foreach ($cand in @(
    (Join-Path $BuildDir "bin"),
    (Join-Path $BuildDir "bin\Release")
)) {
    if (Test-Path (Join-Path $cand "flow_launcher.exe")) { $BinDir = $cand; break }
}
foreach ($cand in @(
    (Join-Path $BuildDir "lib"),
    (Join-Path $BuildDir "lib\Release")
)) {
    if (Test-Path $cand) {
        $hasDll = Get-ChildItem $cand -Filter "*_node.dll" -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($hasDll) { $LibDir = $cand; break }
    }
}
if (-not $BinDir) { throw "flow_launcher.exe not found under $BuildDir\bin" }
if (-not $LibDir) { throw "node plugins (*_node.dll) not found under $BuildDir\lib" }

Write-Host "[demo] bin=$BinDir"
Write-Host "[demo] lib=$LibDir"

$env:PATH = "$BinDir;$LibDir;$env:PATH"
$env:FLOWENGINE_PLUGIN_DIR = $LibDir
$env:FLOW_LOG_DIR = Join-Path $env:TEMP "flow_logs"
New-Item -ItemType Directory -Force -Path $env:FLOW_LOG_DIR | Out-Null

if (-not $Pipeline) {
    $Pipeline = Join-Path $Root "config\pipeline_windows.json"
}
if (-not (Test-Path $Pipeline)) {
    throw "pipeline not found: $Pipeline"
}

$pipelineObj = Get-Content $Pipeline -Raw | ConvertFrom-Json
foreach ($proc in $pipelineObj.processes) {
    if (-not $proc.library_path) { continue }
    $base = [System.IO.Path]::GetFileName(($proc.library_path -replace '\\','/'))
    $base = $base -replace '^lib','' -replace '\.so$','.dll'
    $candidates = @(
        (Join-Path $LibDir $base),
        (Join-Path $LibDir ("lib" + $base))
    )
    $abs = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
    if (-not $abs) {
        $abs = $candidates[0]
        Write-Warning "plugin missing: $abs (also tried lib$base)"
    }
    $proc.library_path = $abs
}
$runtimePipeline = Join-Path $env:TEMP "flow_pipeline_windows_runtime.json"
($pipelineObj | ConvertTo-Json -Depth 30) | ForEach-Object {
    [IO.File]::WriteAllText($runtimePipeline, $_, (New-Object System.Text.UTF8Encoding $false))
}
Write-Host "[demo] pipeline => $runtimePipeline"

$Flowmond = Join-Path $BinDir "flowmond.exe"
$Launcher = Join-Path $BinDir "flow_launcher.exe"
$Html = Join-Path $Root "tools\flowboard\index.html"

Get-Process flowmond, flow_launcher -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue

Write-Host "[demo] starting flowmond..."
$flowmondProc = Start-Process -FilePath $Flowmond `
    -ArgumentList @("--html-path", $Html) `
    -PassThru -WindowStyle Minimized

$ready = $false
for ($i = 0; $i -lt 20; $i++) {
    Start-Sleep -Milliseconds 250
    try {
        $r = Invoke-WebRequest -Uri "http://127.0.0.1:8800/api/health" -UseBasicParsing -TimeoutSec 1
        if ($r.StatusCode -eq 200) { $ready = $true; break }
    } catch {}
}
if ($ready) {
    Write-Host "[demo] flowmond ready http://localhost:8800"
} else {
    Write-Warning "flowmond health check not ready yet; continuing anyway"
}

if (-not $NoBrowser) {
    Start-Process "http://localhost:8800"
}

try {
    Write-Host "[demo] running flow_launcher duration=${Duration}s ..."
    & $Launcher $runtimePipeline --duration $Duration
    $code = $LASTEXITCODE
} finally {
    if ($flowmondProc -and -not $flowmondProc.HasExited) {
        Stop-Process -Id $flowmondProc.Id -Force -ErrorAction SilentlyContinue
    }
}

if ($code -ne 0 -and $null -ne $code) {
    throw "flow_launcher exited with code $code"
}
Write-Host "[demo] finished OK"