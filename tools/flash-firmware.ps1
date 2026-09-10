param(
    [string]$Elf,
    [string]$AppPort,
    [string]$RomPort
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$helperManifest = Join-Path $PSScriptRoot 's31-reset/Cargo.toml'

# Compile both artifacts before asking the board to enter ROM download mode.
if (-not $Elf) {
    Push-Location (Join-Path $repo 'firmware')
    try {
        & cargo +1.95.0 build --release --locked
        if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed.' }
    } finally { Pop-Location }
    $Elf = Join-Path $repo 'target/riscv32imafc-unknown-none-elf/release/aevia-firmware'
}
$Elf = (Resolve-Path -LiteralPath $Elf).Path
Push-Location (Split-Path -Parent $helperManifest)
try {
    & cargo +1.95.0 build --release --locked --bin aevia-flash
    if ($LASTEXITCODE -ne 0) { throw 'Flash helper build failed.' }
} finally { Pop-Location }
$flashArgs = @($Elf)
if ($AppPort) { $flashArgs += @('--app-port', $AppPort) }
if ($RomPort) { $flashArgs += @('--rom-port', $RomPort) }
& (Join-Path $PSScriptRoot 's31-reset/target/release/aevia-flash.exe') @flashArgs
if ($LASTEXITCODE -ne 0) { throw 'Firmware flash or restart verification failed.' }
