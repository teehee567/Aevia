param(
    [string]$Port = 'COM5',
    [ValidateRange(3, 60)][int]$Seconds = 8
)

# Hardware smoke check, not an optical test. The panel must return TE pulses
# and the firmware must finish new frames while USB status remains responsive.
$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200)
$serial.ReadTimeout = 1000
$first = $null
$last = $null
$nonce = [Guid]::NewGuid().ToString('N').Substring(0, 8)
$ready = $false
try {
    $serial.Open()
    $serial.DtrEnable = $true
    $serial.Write("LCDTEST`nSTREAM $nonce`n")
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        try { $line = $serial.ReadLine().TrimEnd() }
        catch [System.TimeoutException] { continue }
        if ($line -match "^STREAM READY $nonce ") { $ready = $true }
        if (-not $ready -or $line -notmatch '^STATUS ') { continue }
        Write-Output $line
        if ($null -eq $first -and $line -match ' lcd=(starting|initializing|pattern-sent) ') { continue }
        if ($line -notmatch ' lcd=scanning lcd-te=(\d+) lcd-frames=(\d+)') {
            throw 'LCD is not reporting scanning; inspect firmware status and panel connection.'
        }
        $sample = @([long]$Matches[1], [long]$Matches[2])
        if ($null -eq $first) { $first = $sample }
        $last = $sample
    }
    if ($null -eq $first -or $last[0] -le $first[0] -or $last[1] -le $first[1]) {
        throw 'No fresh LCD frame/TE progress observed.'
    }
    Write-Output "PASS: LCD TE pulses and completed frames are advancing. Confirm the visible pattern separately."
} finally {
    # A USB reset may make Close/Dispose throw; preserve the original failure.
    try { $serial.Dispose() } catch {}
}
