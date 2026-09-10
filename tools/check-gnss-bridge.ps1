param(
    [string]$ConsolePort = 'COM5',
    [string]$GnssPort,
    [switch]$ReceiveOnly,
    [switch]$LeaveEnabled
)

$ErrorActionPreference = 'Stop'
function Open-Serial([string]$Port, [int]$Baud) {
    $serial = [System.IO.Ports.SerialPort]::new($Port, $Baud)
    $serial.ReadTimeout = 300
    $serial.WriteTimeout = 2000
    $serial.ReadBufferSize = 65536
    $serial.DtrEnable = $true
    $serial.Open()
    return $serial
}
function Wait-Status($Serial, [string]$Phase, [int]$Seconds = 5) {
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        try { $line = $Serial.ReadLine().TrimEnd() }
        catch [System.TimeoutException] { continue }
        if ($line -match "^STATUS .* gnss=$Phase ") { return $line }
    }
    throw "No gnss=$Phase status on $ConsolePort."
}
function Test-UnicoreCrc([string]$Line) {
    if ($Line -notmatch '^#(.+)\*([0-9a-fA-F]{8})$') { return $false }
    $body = $Matches[1]
    $expected = [Convert]::ToUInt32($Matches[2], 16)
    [uint32]$crc = 0
    foreach ($byte in [System.Text.Encoding]::ASCII.GetBytes($body)) {
        $crc = $crc -bxor $byte
        for ($bit = 0; $bit -lt 8; $bit++) {
            if ($crc -band 1) { $crc = ($crc -shr 1) -bxor [uint32]3988292384 }
            else { $crc = $crc -shr 1 }
        }
    }
    return $crc -eq $expected
}

$console = $null
$gnss = $null
$passed = $false
try {
    $console = Open-Serial $ConsolePort 115200
    $console.DiscardInBuffer()
    $console.Write("`nGNSSBRIDGE`n")
    $before = Wait-Status $console 'bridge'
    if (-not $GnssPort) { throw 'Specify -GnssPort with the second J6 USB serial port.' }
    $gnss = Open-Serial $GnssPort 460800
    # Drain bytes left from an absent/blocked reader, then discard the partial
    # line at the new session boundary. All subsequent complete frames must
    # validate; an arbitrary serial-open boundary is not a complete frame.
    $settle = [DateTime]::UtcNow.AddMilliseconds(300)
    while ([DateTime]::UtcNow -lt $settle) {
        [void]$gnss.ReadExisting()
        Start-Sleep -Milliseconds 20
    }
    $gnss.DiscardInBuffer()
    try { [void]$gnss.ReadLine() } catch [System.TimeoutException] {}
    # Fragment a command across separate USB writes. No COM2 suffix: the
    # response must return on the same receiver UART as the request.
    if (-not $ReceiveOnly) {
        $gnss.Write('VER')
        Start-Sleep -Milliseconds 5
        $gnss.Write("SIONA`r`n")
    }
    $versions = 0
    $records = 0
    $deadline = [DateTime]::UtcNow.AddSeconds(4)
    while ([DateTime]::UtcNow -lt $deadline) {
        try { $line = $gnss.ReadLine().TrimEnd() }
        catch [System.TimeoutException] { continue }
        if ($line -match '^(STATUS |RAW |STREAM )') { throw 'Firmware text leaked into the GNSS port.' }
        if ($line -match '^#(VERSIONA|BESTNAVA),') {
            if (-not (Test-UnicoreCrc $line)) { throw 'Corrupt UM980 frame on the USB bridge.' }
            if ($line.StartsWith('#VERSIONA,')) { $versions++ }
            else { $records++ }
        }
    }
    if (-not $ReceiveOnly -and $versions -lt 1) { throw "No CRC-valid direct VERSIONA reply; received $records valid navigation frames. Receiver-to-host works, host-to-receiver is unverified." }
    if ($records -lt 20) { throw "Insufficient receiver stream: $records BESTNAVA records." }
    $after = Wait-Status $console 'bridge'
    foreach ($field in @('gnss-records')) {
        $pattern = " $field=(\d+)"
        $oldValue = [regex]::Match($before, $pattern).Groups[1].Value
        $newValue = [regex]::Match($after, $pattern).Groups[1].Value
        if ($oldValue -ne $newValue) { throw "$field changed during exclusive bridge ownership: $oldValue -> $newValue" }
    }
    $gnss.Dispose()
    $gnss = $null
    # A fresh host connection must still be able to query the receiver.
    $gnss = Open-Serial $GnssPort 460800
    $gnss.DiscardInBuffer()
    if (-not $ReceiveOnly) { $gnss.Write("VERSIONA`r`n") }
    $reconnected = $false
    $deadline = [DateTime]::UtcNow.AddSeconds(3)
    while ([DateTime]::UtcNow -lt $deadline) {
        try { $line = $gnss.ReadLine().TrimEnd() }
        catch [System.TimeoutException] { continue }
        $prefix = if ($ReceiveOnly) { '#BESTNAVA,' } else { '#VERSIONA,' }
        if ($line.StartsWith($prefix) -and (Test-UnicoreCrc $line)) { $reconnected = $true; break }
    }
    if (-not $reconnected) { throw 'Receiver query failed after closing and reopening the GNSS port.' }
    $passed = $true
    if ($ReceiveOnly) {
        Write-Output "PASS (receive only): $GnssPort, $records CRC-valid stream records, no firmware text, exclusive GNSS ownership, reconnect. Two-way pass-through is NOT verified."
    } else {
        Write-Output "PASS: $GnssPort direct VERSIONA CRC, $records stream records, no firmware text, exclusive GNSS ownership, reconnect."
    }
} finally {
    if ($gnss) { $gnss.Dispose() }
    if ($console) {
        try {
            if (-not ($passed -and $LeaveEnabled)) {
                $console.Write("GNSSNORMAL`n")
                if ($passed) {
                    [void](Wait-Status $console 'ready' 25)
                    Write-Output 'PASS: normal GNSS acquisition restored.'
                }
            }
        } finally { $console.Dispose() }
    }
}
if ($LeaveEnabled -and $passed) {
    Write-Output "Bridge left enabled. Serial settings: $GnssPort, 460800 baud, 8N1, no flow control."
}
