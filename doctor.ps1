#Requires -Version 5.1
<#
.SYNOPSIS
    Health check for kinect-senses: toolchain, native DLLs, Python deps,
    USB driver bindings, and (optionally) a live per-sensor capture.
.EXAMPLE
    ./doctor.ps1
    ./doctor.ps1 -Live      # also grab a real depth frame + mic read
#>
[CmdletBinding()]
param([switch]$Live)

$ErrorActionPreference = 'Continue'
$root = $PSScriptRoot
$rows = New-Object System.Collections.Generic.List[object]
function Row($name, $status, $detail) {
    $rows.Add([pscustomobject]@{ Check = $name; Status = $status; Detail = "$detail" })
}
function Have($c) { $null -ne (Get-Command $c -ErrorAction SilentlyContinue) }
function ScoopBin($app, $rel) {
    try { $p = (& scoop prefix $app 2>$null); if ($p) { $f = Join-Path $p $rel; if (Test-Path $f) { return $f } } } catch {}
    return $null
}
function DevService($pattern, [switch]$ParentOnly) {
    $d = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
        Where-Object { $_.InstanceId -match $pattern -and (-not $ParentOnly -or $_.InstanceId -notmatch '&MI_') } |
        Select-Object -First 1
    if (-not $d) { return $null }
    (Get-PnpDeviceProperty -InstanceId $d.InstanceId -KeyName 'DEVPKEY_Device_Service' -ErrorAction SilentlyContinue).Data
}

# --- toolchain ---
Row 'OS 64-bit' ($(if ([Environment]::Is64BitOperatingSystem) { 'PASS' } else { 'FAIL' })) (Get-CimInstance Win32_OperatingSystem).Caption
foreach ($t in 'git', 'python', 'scoop') {
    if (Have $t) { Row $t 'PASS' ((& $t --version 2>$null | Select-Object -First 1)) }
    else { Row $t 'FAIL' 'not found on PATH' }
}
$gcc = ScoopBin 'mingw' 'bin\gcc.exe'
Row 'mingw gcc' ($(if ($gcc) { 'PASS' } else { 'FAIL' })) ($(if ($gcc) { (& $gcc --version | Select-Object -First 1) } else { 'not installed (scoop install mingw)' }))
foreach ($t in 'cmake', 'ninja') {
    $b = (ScoopBin $t "bin\$t.exe"); if (-not $b -and (Have $t)) { $b = (Get-Command $t).Source }
    Row $t ($(if ($b) { 'PASS' } else { 'FAIL' })) ($(if ($b) { $b } else { "not installed (scoop install $t)" }))
}

# --- native DLLs ---
foreach ($d in 'libfreenect.dll', 'libfreenect_sync.dll', 'libusb-1.0.dll') {
    $f = Join-Path $root "dist\$d"
    Row "dist/$d" ($(if (Test-Path $f) { 'PASS' } else { 'FAIL' })) ($(if (Test-Path $f) { '{0:N0} bytes' -f (Get-Item $f).Length } else { 'missing (run setup.ps1)' }))
}

# --- python deps + library load ---
if (Have 'python') {
    $probe = python -c "import importlib.util as u; req=['numpy','scipy','PIL']; opt=['sounddevice','soundfile']; print('REQ='+','.join(m for m in req if u.find_spec(m) is None)); print('OPT='+','.join(m for m in opt if u.find_spec(m) is None))" 2>$null
    $reqMiss = ($probe | Select-String '^REQ=(.*)$').Matches.Groups[1].Value
    $optMiss = ($probe | Select-String '^OPT=(.*)$').Matches.Groups[1].Value
    Row 'python deps (required)' ($(if ($reqMiss) { 'FAIL' } else { 'PASS' })) ($(if ($reqMiss) { "missing: $reqMiss" } else { 'numpy, scipy, pillow' }))
    Row 'python deps (optional)' ($(if ($optMiss) { 'WARN' } else { 'PASS' })) ($(if ($optMiss) { "missing: $optMiss (fallback mic path)" } else { 'sounddevice, soundfile' }))
    if (Test-Path (Join-Path $root 'kinect.py')) {
        python (Join-Path $root 'kinect.py') info *> $null
        Row 'libfreenect load + exports' ($(if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' })) ($(if ($LASTEXITCODE -eq 0) { 'ctypes load OK' } else { 'kinect.py info failed' }))
    }
}

# --- device + driver bindings ---
$cam = DevService 'VID_045E&PID_02AE'
Row 'Kinect connected' ($(if ($cam) { 'PASS' } else { 'FAIL' })) ($(if ($cam) { 'camera 045E:02AE present' } else { 'no Kinect camera detected' }))
Row 'camera driver (02AE)' ($(if ($cam -in 'libusbK', 'WinUSB') { 'PASS' } elseif ($cam) { 'FAIL' } else { 'WARN' })) ("service=$cam (want libusbK)")
$aud = DevService 'VID_045E&PID_02BB' -ParentOnly
Row 'audio driver (02BB parent)' ($(if ($aud -eq 'libusbK') { 'PASS' } elseif ($aud -eq 'usbccgp') { 'WARN' } else { 'WARN' })) ("service=$aud (want libusbK for mic array)")
$mot = DevService 'VID_045E&PID_02B0'
Row 'motor driver (02B0)' ($(if ($mot -in 'WinUSB', 'libusbK') { 'PASS' } else { 'WARN' })) ("service=$mot (want WinUSB/libusbK for tilt)")

# --- informational ---
$hvci = (Get-ItemProperty 'HKLM:\SYSTEM\CurrentControlSet\Control\DeviceGuard\Scenarios\HypervisorEnforcedCodeIntegrity' -ErrorAction SilentlyContinue).Enabled
Row 'Memory Integrity (HVCI)' 'INFO' ($(if ($hvci -eq 1) { 'ON - fine; we bypass via libusb' } else { 'off' }))

# --- optional live capture ---
if ($Live -and (Have 'python')) {
    $tmp = Join-Path $env:TEMP ('ks_doc_{0}' -f $PID)
    python (Join-Path $root 'kinect.py') look $tmp *> $null
    Row 'live depth/RGB capture' ($(if (Test-Path "${tmp}_look.png") { 'PASS' } else { 'FAIL' })) ($(if (Test-Path "${tmp}_look.png") { 'grabbed a frame' } else { 'capture failed (drivers? device?)' }))
    Remove-Item "$tmp*" -ErrorAction SilentlyContinue
}

# --- report ---
$fmt = @{ PASS = 'Green'; WARN = 'Yellow'; FAIL = 'Red'; INFO = 'Cyan' }
Write-Host ''
Write-Host 'kinect-senses doctor' -ForegroundColor White
Write-Host ('-' * 68)
foreach ($r in $rows) {
    Write-Host ('  {0,-6} ' -f $r.Status) -ForegroundColor $fmt[$r.Status] -NoNewline
    Write-Host ('{0,-26} {1}' -f $r.Check, $r.Detail)
}
Write-Host ('-' * 68)
$p = ($rows | Where-Object Status -eq 'PASS').Count
$w = ($rows | Where-Object Status -eq 'WARN').Count
$f = ($rows | Where-Object Status -eq 'FAIL').Count
Write-Host ("  {0} pass, {1} warn, {2} fail" -f $p, $w, $f) -ForegroundColor $(if ($f) { 'Red' } elseif ($w) { 'Yellow' } else { 'Green' })
if ($f) { exit 1 } else { exit 0 }
