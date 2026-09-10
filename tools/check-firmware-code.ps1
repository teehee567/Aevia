$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot

function Invoke-Check([string[]]$CargoArgs) {
    & cargo +1.95.0 @CargoArgs
    if ($LASTEXITCODE -ne 0) { throw "cargo $($CargoArgs -join ' ') failed." }
}

Push-Location $repo
try {
    Invoke-Check @('fmt', '-p', 'aevia-firmware', '--check')
    Invoke-Check @('test', '-p', 'aevia-firmware', '--lib', '--no-default-features', '--locked')
    Invoke-Check @('clippy', '-p', 'aevia-firmware', '--lib', '--tests', '--no-default-features', '--locked', '--', '-D', 'warnings')
    Push-Location (Join-Path $repo 'firmware')
    try {
        Invoke-Check @('clippy', '--release', '--locked', '--', '-D', 'warnings')
        Invoke-Check @('build', '--release', '--locked')
    } finally { Pop-Location }
    Push-Location (Join-Path $PSScriptRoot 's31-reset')
    try {
        Invoke-Check @('fmt', '--check')
        Invoke-Check @('test', '--locked')
        Invoke-Check @('clippy', '--all-targets', '--locked', '--', '-D', 'warnings')
    } finally { Pop-Location }
} finally { Pop-Location }
Write-Output 'PASS: firmware and flashing-tool tests, strict Clippy, formatting and release build.'
