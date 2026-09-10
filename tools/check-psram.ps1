param(
    [string]$Port = 'COM5',
    [ValidateRange(3, 60)][int]$Seconds = 8
)

$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200)
$serial.ReadTimeout = 1000
$first = $null
$last = $null
$ready = 0
try {
    $serial.Open()
    $serial.DtrEnable = $true
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        try { $line = $serial.ReadLine().TrimEnd() }
        catch [System.TimeoutException] { continue }
        if ($line -notmatch '^STATUS firmware=base ') { continue }
        Write-Output $line
        if ($line -match ' psram=fault ') { throw 'PSRAM protection or memory check failed.' }
        if ($line -notmatch ' uptime-ms=(\d+) ') { continue }
        $uptime = [long]$Matches[1]
        if ($null -ne $last -and $uptime -le $last) { throw 'Firmware restarted or stalled.' }
        if ($null -eq $first) { $first = $uptime }
        $last = $uptime
        if ($line -match ' psram=ready psram-tested=16MiB(?: |$)') { $ready++ }
    }
    if ($ready -lt 2 -or $last -le $first) { throw 'No advancing base status with a full PSRAM test.' }
    Write-Output 'PASS: full 16 MiB PSRAM test and advancing firmware uptime.'
} finally {
    try { $serial.Dispose() } catch {}
}
