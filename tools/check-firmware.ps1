param(
    [string]$Port = 'COM5',
    [ValidateRange(5, 60)][int]$Seconds = 15
)

# Hardware smoke test: transport framing, capture progress, PSRAM and STOP.
# A satellite fix, a battery and a working LCD are not required for base bring-up.
$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200)
$serial.ReadTimeout = 1000
$serial.WriteTimeout = 2000
$nonce = [Guid]::NewGuid().ToString('N').Substring(0, 8)
$ready = $false
$imu = 0
$gnss = 0
$first = $null
$last = $null

function Read-Status([string]$Line) {
    $fields = @{}
    foreach ($field in $Line.Split(' ')) {
        $pair = $field.Split('=', 2)
        if ($pair.Length -eq 2) { $fields[$pair[0]] = $pair[1] }
    }
    return $fields
}

try {
    $serial.Open()
    $serial.DtrEnable = $true
    # An embedded bootloader token must not reset a line-framed command parser.
    $serial.Write("XBOOTLOADER`nSTREAM $nonce`n")
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        try { $line = $serial.ReadLine().TrimEnd() }
        catch [System.TimeoutException] { continue }
        if ($line -match "^STREAM READY $nonce ") { $ready = $true; continue }
        if (-not $ready) { continue }
        if ($line -match '^RAW IMU ') { $imu++ }
        if ($line -match '^RAW GNSS ') { $gnss++ }
        if ($line -notmatch '^STATUS ') { continue }
        $status = Read-Status $line
        if ($status.firmware -ne 'base' -or $status.protocol -ne '1') { throw 'Unexpected firmware protocol.' }
        if ($null -ne $last -and [long]$status.'uptime-ms' -le [long]$last.'uptime-ms') {
            throw 'Firmware restarted or uptime stopped advancing.'
        }
        if ($null -eq $first) { $first = $status }
        $last = $status
        Write-Output $line
    }
    if (-not $ready -or $null -eq $first -or $null -eq $last) { throw 'No fresh stream/status handshake.' }
    if ($imu -lt 10 -or $gnss -lt 10 -or
        [long]$last.'imu-records' -le [long]$first.'imu-records' -or
        [long]$last.'gnss-records' -le [long]$first.'gnss-records') {
        throw "Capture did not advance on both sensors: RAW IMU=$imu RAW GNSS=$gnss."
    }
    if ($last.psram -ne 'ready' -or $last.'psram-tested' -ne '16MiB') { throw 'Full PSRAM test did not pass.' }
    if ([long]$last.'capture-errors' -ne [long]$first.'capture-errors') {
        throw "Capture errors increased during the check: $($first.'capture-errors') -> $($last.'capture-errors')."
    }
    $serial.Write("STOP`n")
    $stopped = $false
    $afterStop = $null
    $deadline = [DateTime]::UtcNow.AddSeconds(3)
    while ([DateTime]::UtcNow -lt $deadline) {
        try { $line = $serial.ReadLine().TrimEnd() }
        catch [System.TimeoutException] { continue }
        if ($line -eq 'STREAM STOPPED') { $stopped = $true }
        if (-not $stopped) { continue }
        if ($line -match '^RAW ') { throw 'RAW records continued after STOP acknowledgment.' }
        if ($line -match '^STATUS ') { $afterStop = Read-Status $line }
    }
    if (-not $stopped -or $null -eq $afterStop -or
        [long]$afterStop.'imu-records' -le [long]$last.'imu-records' -or
        [long]$afterStop.'gnss-records' -le [long]$last.'gnss-records') {
        throw 'STOP stalled capture or USB status.'
    }
    Write-Output "PASS: RAW IMU=$imu GNSS=$gnss; full PSRAM check; no reset; STOP preserves capture and status."
    Write-Output "Hardware status: LCD=$($last.lcd), battery=$($last.battery), buttons=$($last.buttons)."
} finally {
    try { $serial.Dispose() } catch {}
}
