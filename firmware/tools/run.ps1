param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$ElfPath
)

$ErrorActionPreference = 'Stop'
$appVidPid = 'VID_303A&PID_4001'
$romVidPid = 'VID_303A&PID_0020'
$firmwareRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = Split-Path -Parent $firmwareRoot
$resetManifest = Join-Path $repoRoot 'tools\s31-reset\Cargo.toml'

function Find-UsbComPort {
    param([string]$VidPid)

    $matchingDevice = $false
    foreach ($line in (& pnputil /enum-devices /connected /class Ports)) {
        if ($line -match '^Instance ID:\s+(.+)$') {
            $matchingDevice = $Matches[1] -like "*$VidPid*"
            continue
        }
        if ($matchingDevice -and $line -match '^Device Description:\s+.*\((COM\d+)\)') {
            return $Matches[1]
        }
    }
    return $null
}

function Wait-UsbComPort {
    param(
        [string]$VidPid,
        [int]$TimeoutSeconds = 15
    )

    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    do {
        $port = Find-UsbComPort $VidPid
        if ($null -ne $port) {
            return $port
        }
        Start-Sleep -Milliseconds 200
    } while ([DateTime]::UtcNow -lt $deadline)

    return $null
}

$romPort = Find-UsbComPort $romVidPid
if ($null -eq $romPort) {
    $appPort = Find-UsbComPort $appVidPid
    if ($null -eq $appPort) {
        throw 'No AEVIA application or ESP32-S31 ROM port found. For the first install only: hold SW7 for 9 seconds and release; hold SW2; tap SW7 for 0.2 seconds; keep holding SW2 for 5 seconds, then release it.'
    }

    Write-Host "Requesting ROM download mode over $appPort..."
    $serial = [System.IO.Ports.SerialPort]::new($appPort, 115200, 'None', 8, 'One')
    $serial.DtrEnable = $false
    $serial.RtsEnable = $false
    $serial.Open()
    $serial.Write("BOOTLOADER`n")
    $serial.BaseStream.Flush()
    Start-Sleep -Milliseconds 200
    $serial.Close()

    $romPort = Wait-UsbComPort $romVidPid
    if ($null -eq $romPort) {
        throw 'The application accepted the reflash request, but the ROM port did not enumerate.'
    }
}

$resolvedElf = (Resolve-Path -LiteralPath $ElfPath).Path
Write-Host "Flashing $resolvedElf through $romPort..."
& espflash flash --chip esp32s31 --port $romPort --before no-reset --after no-reset --non-interactive $resolvedElf
if ($LASTEXITCODE -ne 0) {
    throw "espflash failed with exit code $LASTEXITCODE"
}

$romPort = Wait-UsbComPort $romVidPid 5
if ($null -ne $romPort) {
    Write-Host "Starting the application through the S31 watchdog..."
    Push-Location $repoRoot
    try {
        & cargo run --quiet --release --manifest-path $resetManifest -- $romPort
        if ($LASTEXITCODE -ne 0) {
            throw "S31 reset helper failed with exit code $LASTEXITCODE"
        }
    }
    finally {
        Pop-Location
    }
}

$appPort = Wait-UsbComPort $appVidPid
if ($null -eq $appPort) {
    throw 'Flash completed, but the AEVIA J6 console did not enumerate.'
}

Write-Host "AEVIA peripheral report on $appPort (Ctrl+C to stop):"
$monitor = [System.IO.Ports.SerialPort]::new($appPort, 115200, 'None', 8, 'One')
$monitor.DtrEnable = $false
$monitor.RtsEnable = $false
$monitor.ReadTimeout = 1000
$monitor.NewLine = "`n"
$monitor.Open()

try {
    while ($true) {
        try {
            $line = $monitor.ReadLine().TrimEnd("`r")
            Write-Host $line
        }
        catch [System.TimeoutException] {
        }
    }
}
finally {
    if ($monitor.IsOpen) {
        $monitor.Close()
    }
}
