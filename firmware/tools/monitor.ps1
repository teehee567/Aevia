param(
    [string]$Port = 'COM5'
)

$ErrorActionPreference = 'Stop'

while ($true) {
    $monitor = [System.IO.Ports.SerialPort]::new($Port, 115200, 'None', 8, 'One')
    $monitor.DtrEnable = $false
    $monitor.RtsEnable = $false
    $monitor.ReadTimeout = 1000
    $monitor.NewLine = "`n"

    try {
        $monitor.Open()
        Write-Host "Monitoring $Port (Ctrl+C to stop)..."
        while ($monitor.IsOpen) {
            try {
                Write-Host ($monitor.ReadLine().TrimEnd("`r"))
            }
            catch [System.TimeoutException] {
            }
        }
    }
    catch [System.IO.IOException] {
        Write-Host "$Port disconnected; retrying..."
    }
    catch [System.UnauthorizedAccessException] {
        Write-Host "$Port busy; retrying..."
    }
    finally {
        if ($monitor.IsOpen) {
            $monitor.Close()
        }
        $monitor.Dispose()
    }

    Start-Sleep -Milliseconds 500
}
