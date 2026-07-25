#Requires -Version 5.1
<#
.SYNOPSIS
    One-shot, idempotent setup for kinect-senses on Windows x64: installs the
    build toolchain, fetches libusb, builds libfreenect, assembles dist/,
    installs Python deps, binds the USB drivers (guided), and verifies.

    Each step detects whether it is already done and skips it, so re-running is
    safe. Requires scoop (https://scoop.sh), git, and Python already installed.
.PARAMETER Force        Rebuild the native DLLs even if dist/ already has them.
.PARAMETER SkipDrivers  Do not touch USB drivers (run bind-drivers.ps1 yourself).
.PARAMETER SkipVerify   Skip the final doctor.ps1 health check.
.PARAMETER LibusbVersion  libusb release to fetch (default 1.0.27).
.EXAMPLE
    ./setup.ps1
#>
[CmdletBinding()]
param(
    [switch]$Force,
    [switch]$SkipDrivers,
    [switch]$SkipVerify,
    [string]$LibusbVersion = '1.0.27'
)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
function Step($m) { Write-Host "==> $m" -ForegroundColor Cyan }
function Info($m) { Write-Host "    $m" -ForegroundColor Gray }
function Ok($m)   { Write-Host "    $m" -ForegroundColor Green }
function Have($c) { $null -ne (Get-Command $c -ErrorAction SilentlyContinue) }
function ScoopBin($app, $rel) {
    $p = (& scoop prefix $app 2>$null); if ($p) { $f = Join-Path $p $rel; if (Test-Path $f) { return $f } }
    return $null
}

# --- 0. prerequisites -------------------------------------------------------
Step 'Checking prerequisites (scoop, git, python)'
if (-not (Have 'scoop')) {
    Write-Host "scoop is required. Install it (no admin) then re-run:" -ForegroundColor Red
    Write-Host "  Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser" -ForegroundColor Yellow
    Write-Host "  irm get.scoop.sh | iex" -ForegroundColor Yellow
    exit 1
}
foreach ($t in 'git', 'python') {
    if (-not (Have $t)) { Write-Host "$t not found on PATH. Install it and re-run." -ForegroundColor Red; exit 1 }
}
Ok 'prerequisites present'

# --- 1. toolchain -----------------------------------------------------------
Step 'Installing build toolchain (cmake, ninja, mingw, 7zip)'
scoop install cmake ninja mingw 7zip | Out-Null
$mingw = (& scoop prefix mingw)
if (-not $mingw) { throw 'mingw install failed' }
$env:Path = "$mingw\bin;$env:Path"
$7z = (ScoopBin '7zip' '7z.exe'); if (-not $7z) { $7z = (Get-Command 7z -ErrorAction SilentlyContinue).Source }
Ok "toolchain ready (mingw at $mingw)"

# --- 2. libusb --------------------------------------------------------------
$deps = Join-Path $root 'deps'
$lu = Join-Path $deps 'libusb'
if (Test-Path (Join-Path $lu 'include\libusb.h')) {
    Step "libusb already present"; Ok "$lu"
} else {
    Step "Fetching libusb $LibusbVersion"
    New-Item -ItemType Directory -Force $deps | Out-Null
    $url = "https://github.com/libusb/libusb/releases/download/v$LibusbVersion/libusb-$LibusbVersion.7z"
    $arc = Join-Path $deps "libusb-$LibusbVersion.7z"
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    Invoke-WebRequest -Uri $url -OutFile $arc -UseBasicParsing
    & $7z x $arc -o"$lu" -y | Out-Null
    Ok "extracted to $lu"
}

# --- 3. libfreenect: clone + patch -----------------------------------------
$lf = Join-Path $root 'libfreenect'
if (-not (Test-Path $lf)) {
    Step 'Cloning libfreenect'
    git clone --depth 1 https://github.com/OpenKinect/libfreenect.git $lf | Out-Null
} else { Step 'libfreenect already cloned' }
$unistd = Join-Path $lf 'platform\windows\unistd.h'
if ((Get-Content $unistd -Raw) -notmatch 'unsigned int sleep') {
    Info 'applying Windows/gcc sleep() patch'
    git -C $lf apply (Join-Path $root 'patches\libfreenect-mingw-sleep.patch')
    Ok 'patch applied'
} else { Ok 'patch already applied' }

# --- 4. build ---------------------------------------------------------------
$dist = Join-Path $root 'dist'
$build = Join-Path $root 'build'
$syncDll = Join-Path $dist 'libfreenect_sync.dll'
if ((Test-Path $syncDll) -and -not $Force) {
    Step 'Native DLLs already built (use -Force to rebuild)'
} else {
    Step 'Building libfreenect (freenect + freenect_sync)'
    $pinc = Join-Path $mingw 'x86_64-w64-mingw32\include'
    $plib = Join-Path $mingw 'x86_64-w64-mingw32\lib\libpthread.a'
    $linc = Join-Path $lu 'include'
    $llib = Join-Path $lu 'MinGW64\static\libusb-1.0.dll.a'
    foreach ($p in $pinc, $plib, $linc, $llib) { if (-not (Test-Path $p)) { throw "missing build input: $p" } }
    cmake -G Ninja -S $lf -B $build `
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ `
        -DBUILD_EXAMPLES=OFF -DBUILD_FAKENECT=OFF -DBUILD_CPP=OFF `
        -DBUILD_C_SYNC=ON -DBUILD_REDIST_PACKAGE=ON `
        -DBUILD_PYTHON=OFF -DBUILD_PYTHON2=OFF -DBUILD_PYTHON3=OFF `
        -DLIBUSB_1_INCLUDE_DIRS="$linc" -DLIBUSB_1_LIBRARIES="$llib" `
        -DTHREADS_PTHREADS_INCLUDE_DIR="$pinc" -DTHREADS_PTHREADS_WIN32_LIBRARY="$plib" | Out-Null
    cmake --build $build -j | Out-Null
    Ok 'build complete'
}

# --- 5. assemble dist/ ------------------------------------------------------
Step 'Assembling dist/'
New-Item -ItemType Directory -Force $dist | Out-Null
Copy-Item (Join-Path $build 'lib\libfreenect.dll') $dist -Force
Copy-Item (Join-Path $build 'lib\libfreenect_sync.dll') $dist -Force
Copy-Item (Join-Path $lu 'MinGW64\dll\libusb-1.0.dll') $dist -Force
Ok "dist/ has $((Get-ChildItem $dist -Filter *.dll).Count) DLLs"

# --- 6. python deps ---------------------------------------------------------
Step 'Installing Python dependencies'
python -m pip install --quiet -r (Join-Path $root 'requirements.txt')
Ok 'python deps installed'

# --- 7. drivers -------------------------------------------------------------
if ($SkipDrivers) {
    Step 'Skipping USB driver binding (-SkipDrivers). Run bind-drivers.ps1 later.'
} else {
    Step 'USB driver binding (guided)'
    & (Join-Path $root 'bind-drivers.ps1')
}

# --- 8. verify --------------------------------------------------------------
if (-not $SkipVerify) {
    Step 'Verifying'
    & (Join-Path $root 'doctor.ps1')
}

Write-Host ''
Write-Host 'Setup done. Try:  python kinect.py look out/glance' -ForegroundColor Green
