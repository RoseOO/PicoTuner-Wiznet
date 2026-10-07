<#
    PicoTuner-WH  -  Windows build script
    ------------------------------------
    Configures (if needed) and builds the PicoTuner-WH firmware using
    CMake + Ninja + the ARM GNU toolchain.

    Usage:
        powershell -ExecutionPolicy Bypass -File .\build.ps1
        powershell -ExecutionPolicy Bypass -File .\build.ps1 -Clean
        powershell -ExecutionPolicy Bypass -File .\build.ps1 -Configure

    The target board defaults come from CMakeLists.txt (this branch builds the
    W5500-EVB-Pico2 / RP2350).  Override per-invocation, e.g.:
        .\build.ps1 -PicoBoard pico -WiznetChip W6100 -WiznetBoard W6100_EVB_PICO -BuildDir build-w6100

    Any of these may be overridden via environment variables:
        PICO_SDK_PATH   - Raspberry Pi Pico SDK
        WIZNET_DIR      - WIZnet ioLibrary_Driver sources
        PORT_DIR        - WIZnet Pico port layer
        PICO_TOOLS_DIR  - folder holding the cmake / mingw / arm toolchain folders
#>

[CmdletBinding()]
param(
    [switch]$Clean,                          # delete the build directory and reconfigure from scratch
    [switch]$Configure,                      # only configure, do not build
    [string]$BuildDir    = 'build',          # build directory name
    [string]$PicoBoard,                      # pico (RP2040) / pico2 (RP2350)
    [string]$WiznetChip,                     # W5100S / W5500 / W6100
    [string]$WiznetBoard                     # board name from board_list.h
)

$ErrorActionPreference = 'Stop'

$Root    = Split-Path -Parent $MyInvocation.MyCommand.Path
$Build   = Join-Path $Root $BuildDir
$SrcParent = Split-Path -Parent $Root

function Resolve-Tool {
    param(
        [string[]]$Candidates,
        [string]  $Name
    )
    foreach ($c in $Candidates) {
        if ($c -and (Test-Path -LiteralPath $c)) { return $c }
    }
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    throw "Could not locate '$Name'. Install it or set PICO_TOOLS_DIR / PATH."
}

# --- locate the toolchain ----------------------------------------------------
$ToolRoot = if ($env:PICO_TOOLS_DIR) { $env:PICO_TOOLS_DIR } else { 'C:\Users\Rose\src\tools' }
$MingwBin = Join-Path $ToolRoot 'mingw64\bin'
$ArmBin   = Join-Path $ToolRoot 'arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi\bin'

$Cmake = Resolve-Tool @(
            (Join-Path $ToolRoot 'cmake-3.29.3-windows-x86_64\bin\cmake.exe'),
            'C:\Program Files\CMake\bin\cmake.exe'
         ) 'cmake'
$Ninja = Resolve-Tool @( (Join-Path $MingwBin 'ninja.exe') ) 'ninja'
$ArmGcc = Resolve-Tool @( (Join-Path $ArmBin 'arm-none-eabi-gcc.exe') ) 'arm-none-eabi-gcc'

# picotool needs the mingw runtime DLLs; the ARM tools supply the compiler/linker
$env:PATH = "$MingwBin;$ArmBin;$(Split-Path -Parent $Cmake);$env:PATH"

# --- locate the SDK / WIZnet sources ----------------------------------------
if (-not $env:PICO_SDK_PATH) {
    $cand = Join-Path $SrcParent 'pico-sdk'
    if (Test-Path -LiteralPath $cand) { $env:PICO_SDK_PATH = $cand }
}
if (-not $env:PICO_SDK_PATH) { throw "PICO_SDK_PATH not set and $(Join-Path $SrcParent 'pico-sdk') not found." }

$WizRoot = Join-Path $SrcParent 'WIZnet-PICO-C'
if (-not $env:WIZNET_DIR) { $env:WIZNET_DIR = Join-Path $WizRoot 'libraries\ioLibrary_Driver' }
if (-not $env:PORT_DIR)   { $env:PORT_DIR   = Join-Path $WizRoot 'port' }

Write-Host "PicoTuner-WH build" -ForegroundColor Cyan
Write-Host "  root        : $Root"
Write-Host "  build dir   : $Build"
Write-Host "  cmake       : $Cmake"
Write-Host "  ninja       : $Ninja"
Write-Host "  arm gcc     : $ArmGcc"
Write-Host "  PICO_SDK    : $env:PICO_SDK_PATH"
Write-Host "  WIZNET_DIR  : $env:WIZNET_DIR"
Write-Host "  PORT_DIR    : $env:PORT_DIR"

if ($Clean -and (Test-Path -LiteralPath $Build)) {
    Write-Host "Cleaning $Build ..." -ForegroundColor Yellow
    Remove-Item -LiteralPath $Build -Recurse -Force
}

$cache = Join-Path $Build 'CMakeCache.txt'
if (-not (Test-Path -LiteralPath $cache)) {
    Write-Host "Configuring ..." -ForegroundColor Yellow
    $cfg = @(
        "-DCMAKE_MAKE_PROGRAM=$Ninja",
        "-DPICO_SDK_PATH=$env:PICO_SDK_PATH",
        "-DWIZNET_DIR=$env:WIZNET_DIR",
        "-DPORT_DIR=$env:PORT_DIR"
    )
    if ($PicoBoard)   { $cfg += "-DPICO_BOARD=$PicoBoard" }
    if ($WiznetChip)  { $cfg += "-DWIZNET_CHIP=$WiznetChip" }
    if ($WiznetBoard) { $cfg += "-DWIZNET_BOARD=$WiznetBoard" }
    & $Cmake -S $Root -B $Build -G Ninja @cfg
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed ($LASTEXITCODE)." }
}

if (-not $Configure) {
    Write-Host "Building ..." -ForegroundColor Yellow
    & $Cmake --build $Build
    if ($LASTEXITCODE -ne 0) { throw "Build failed ($LASTEXITCODE)." }

    $uf2 = Join-Path $Build 'picotunewh.uf2'
    if (Test-Path -LiteralPath $uf2) {
        $item = Get-Item -LiteralPath $uf2
        Write-Host ("`nBuild OK: {0} ({1:N0} bytes)" -f $item.FullName, $item.Length) -ForegroundColor Green
    }
}
