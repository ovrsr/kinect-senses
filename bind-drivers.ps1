#Requires -Version 5.1
<#
.SYNOPSIS
    Guided + verified USB driver binding for the Kinect, using Zadig.

    Fully unattended (zero-click) binding would require building libwdi (the
    engine inside Zadig) and is easy to get wrong on the wrong device. Instead
    this detects each device's current driver, launches Zadig ONLY for what
    still needs changing with the exact entry/driver to pick, then re-verifies
    in a loop. Already-correct devices are skipped, so re-running is safe.
.PARAMETER Check
    Report current bindings and exit (no prompts, no changes).
.EXAMPLE
    ./bind-drivers.ps1
    ./bind-drivers.ps1 -Check
#>
[CmdletBinding()]
param([switch]$Check)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$ZADIG_URL = 'https://github.com/pbatard/libwdi/releases/download/v1.5.1/zadig-2.9.exe'

function DevService($pattern, [bool]$ParentOnly) {
    $d = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
        Where-Object { $_.InstanceId -match $pattern -and (-not $ParentOnly -or $_.InstanceId -notmatch '&MI_') } |
        Select-Object -First 1
    if (-not $d) { return $null }
    (Get-PnpDeviceProperty -InstanceId $d.InstanceId -KeyName 'DEVPKEY_Device_Service' -ErrorAction SilentlyContinue).Data
}

# Target devices, in a sensible binding order.
$targets = @(
    @{ Name = 'Camera (depth/RGB/IR)'; Pattern = 'VID_045E&PID_02AE'; Parent = $false
       Want = @('libusbK'); Steps = 'Select the "045E 02AE" entry (Xbox NUI Camera / Kinect Camera), set the driver to libusbK, click Replace Driver.' }
    @{ Name = 'Motor (tilt)'; Pattern = 'VID_045E&PID_02B0'; Parent = $false
       Want = @('WinUSB', 'libusbK'); Steps = 'Select the "045E 02B0" entry (Kinect ... Device), set the driver to WinUSB, click Replace Driver.' }
    @{ Name = 'Audio composite PARENT (mic array)'; Pattern = 'VID_045E&PID_02BB'; Parent = $true
       Want = @('libusbK'); Steps = 'Options -> UNcheck "Ignore Hubs or Composite Parents", then select the "045E 02BB" USB Composite Device PARENT (no "(Interface N)" suffix), set libusbK, click Replace Driver.' }
)

function Show-State {
    Write-Host ''
    Write-Host 'Kinect USB driver bindings:' -ForegroundColor White
    foreach ($t in $targets) {
        $svc = DevService $t.Pattern $t.Parent
        $ok = $svc -and ($t.Want -contains $svc)
        $missing = -not $svc
        $status = if ($missing) { 'MISSING' } elseif ($ok) { 'OK' } else { 'NEEDS CHANGE' }
        $color = if ($missing) { 'Red' } elseif ($ok) { 'Green' } else { 'Yellow' }
        Write-Host ('  {0,-13} ' -f $status) -ForegroundColor $color -NoNewline
        Write-Host ('{0,-34} service={1} (want {2})' -f $t.Name, $svc, ($t.Want -join '/'))
    }
    Write-Host ''
}

Show-State
if ($Check) { return }

# ensure zadig.exe
$zadig = Join-Path $root 'tools\zadig.exe'
if (-not (Test-Path $zadig)) {
    New-Item -ItemType Directory -Force (Split-Path $zadig) | Out-Null
    Write-Host "Downloading Zadig..." -ForegroundColor Cyan
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    Invoke-WebRequest -Uri $ZADIG_URL -OutFile $zadig -UseBasicParsing
}

$changed = 0
foreach ($t in $targets) {
    $svc = DevService $t.Pattern $t.Parent
    if (-not $svc) { Write-Host "! $($t.Name): device not present, skipping." -ForegroundColor Red; continue }
    if ($t.Want -contains $svc) { continue }

    for ($attempt = 1; $attempt -le 3; $attempt++) {
        Write-Host ''
        Write-Host ">> $($t.Name) is on '$svc' but needs '$($t.Want -join '/')'." -ForegroundColor Yellow
        Write-Host "   In Zadig (Options -> List All Devices first):" -ForegroundColor Yellow
        Write-Host "   $($t.Steps)" -ForegroundColor Gray
        Write-Host "   Launching Zadig (accept the UAC prompt)..." -ForegroundColor Cyan
        try { Start-Process -FilePath $zadig -Verb RunAs | Out-Null } catch { Write-Host "   (couldn't auto-launch; run tools\zadig.exe as admin)" -ForegroundColor Red }
        [void](Read-Host "   Press Enter after 'Driver Installation: SUCCESS'")
        Start-Sleep -Seconds 2
        $svc = DevService $t.Pattern $t.Parent
        if ($t.Want -contains $svc) { Write-Host "   OK -> $($t.Name) now on $svc" -ForegroundColor Green; $changed++; break }
        Write-Host "   still '$svc' - let's try again." -ForegroundColor Yellow
        if ($attempt -eq 3) { Write-Host "   giving up on $($t.Name) after 3 tries." -ForegroundColor Red }
    }
}

Write-Host ''
Show-State
$allOk = $true
foreach ($t in $targets) { $svc = DevService $t.Pattern $t.Parent; if (-not ($svc -and ($t.Want -contains $svc))) { $allOk = $false } }
if ($allOk) { Write-Host 'All Kinect drivers bound correctly.' -ForegroundColor Green; exit 0 }
else { Write-Host 'Some drivers still need attention (see above).' -ForegroundColor Yellow; exit 1 }
