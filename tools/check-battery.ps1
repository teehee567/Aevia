param(
    [string]$Port = 'COM5',
    [ValidateRange(3, 60)][int]$Seconds = 8
)

$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200)
$serial.ReadTimeout = 1000
$nonce = [Guid]::NewGuid().ToString('N').Substring(0, 8)
$ready = $false
$samples = 0
$last = ''
try {
    $serial.Open()
    $serial.DtrEnable = $true
    $serial.Write("STREAM $nonce`n")
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        try { $line = $serial.ReadLine().TrimEnd() }
        catch [System.TimeoutException] { continue }
        if ($line -match "^STREAM READY $nonce ") { $ready = $true }
        if (-not $ready -or $line -notmatch '^STATUS ') { continue }
        $last = $line
        Write-Output $line
        if ($line -match ' battery=(charging|bypass|battery|low) ' -and
            $line -match ' battery-charger=ok ' -and
            $line -match ' led=(green|blue|purple|red) ') { $samples++ }
    }
    if ($samples -lt 3) { throw "Battery monitor did not produce three valid live samples. Last: $last" }
    Write-Output "PASS: $samples valid live battery/LED status records. Confirm visible colour separately."
} finally {
    try { $serial.Dispose() } catch {}
}
