<#
.SYNOPSIS
Build the application, matching host decoder and a checksum manifest; never flash.
.EXAMPLE
powershell -ExecutionPolicy Bypass -File firmware/tools/build_release.ps1
#>
[CmdletBinding()]
param(
    [string]$OutputDirectory,
    [string]$Cargo,
    [string]$Espflash,
    [switch]$SkipTests
)

$ErrorActionPreference = 'Stop'
$firmwareRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $firmwareRoot '..')).Path
$targetDirectory = Join-Path $repositoryRoot 'target'
$hostTarget = 'x86_64-pc-windows-msvc'
$deviceTarget = 'riscv32imafc-unknown-none-elf'

function Resolve-Tool([string]$Specified, [string]$Name) {
    if ($Specified) { return (Get-Command $Specified -ErrorAction Stop).Source }
    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($command) { return $command.Source }
    $candidate = Join-Path $env:USERPROFILE ".cargo/bin/$Name.exe"
    if (Test-Path -LiteralPath $candidate) { return $candidate }
    throw "Missing $Name. Install the tool or pass its explicit path."
}

function Invoke-Checked([string]$Executable, [string[]]$Arguments) {
    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Executable failed with exit code $LASTEXITCODE"
    }
}

function Artifact-Entry([string]$Path) {
    $file = Get-Item -LiteralPath $Path
    return [ordered]@{
        file = $file.Name
        bytes = $file.Length
        sha256 = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
}

$cargoTool = Resolve-Tool $Cargo 'cargo'
$espflashTool = Resolve-Tool $Espflash 'espflash'
if (-not $OutputDirectory) {
    $OutputDirectory = Join-Path $firmwareRoot ('build/releases/' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
}
$outputPath = [IO.Path]::GetFullPath($OutputDirectory)
if ((Test-Path -LiteralPath $outputPath) -and (Get-ChildItem -LiteralPath $outputPath -Force | Select-Object -First 1)) {
    throw 'Output directory must be new or empty; existing release evidence is preserved.'
}
New-Item -ItemType Directory -Path $outputPath -Force | Out-Null
$statusPath = Join-Path $outputPath 'build-status.json'
@{ complete = $false; started_utc = [DateTime]::UtcNow.ToString('o') } |
    ConvertTo-Json | Set-Content -LiteralPath $statusPath -Encoding UTF8

Push-Location $firmwareRoot
try {
    if (-not $SkipTests) {
        Invoke-Checked $cargoTool @('test', '--locked', '-p', 'firmware', '--lib',
            '--target', $hostTarget, '--target-dir', $targetDirectory,
            '--config', 'unstable.build-std=[]')
        Invoke-Checked $cargoTool @('test', '--locked', '--manifest-path', 'tools/board-probe/Cargo.toml',
            '--target', $hostTarget, '--target-dir', (Join-Path $targetDirectory 'board-tools'),
            '--config', 'unstable.build-std=[]')
    }
    Invoke-Checked $cargoTool @('build', '--release', '--locked', '-p', 'firmware',
        '--target', $deviceTarget, '--target-dir', $targetDirectory)
    Invoke-Checked $cargoTool @('build', '--locked', '--manifest-path', 'tools/board-probe/Cargo.toml',
        '--target', $hostTarget, '--target-dir', (Join-Path $targetDirectory 'board-tools'),
        '--config', 'unstable.build-std=[]')

    $elf = Join-Path $targetDirectory "$deviceTarget/release/firmware"
    $image = Join-Path $outputPath 'firmware.bin'
    Invoke-Checked $espflashTool @('save-image', '--skip-update-check', '--chip', 'esp32s31',
        '--flash-size', '16mb', '--partition-table', 'partitions.csv',
        '--target-app-partition', 'slot0', $elf, $image)
    Copy-Item -LiteralPath $elf -Destination (Join-Path $outputPath 'firmware.elf')
    Copy-Item -LiteralPath (Join-Path $targetDirectory "board-tools/$hostTarget/debug/aevia-board-decode.exe") -Destination $outputPath
    Copy-Item -LiteralPath (Join-Path $firmwareRoot 'partitions.csv') -Destination $outputPath
    Copy-Item -LiteralPath (Join-Path $firmwareRoot 'docs/app-interface.md') -Destination $outputPath
    Copy-Item -LiteralPath (Join-Path $firmwareRoot 'docs/race-recording.md') -Destination $outputPath
    Copy-Item -LiteralPath (Join-Path $firmwareRoot 'tools/recording.py') -Destination $outputPath
    Copy-Item -LiteralPath (Join-Path $firmwareRoot 'tools/usb_frame.py') -Destination $outputPath
    Copy-Item -LiteralPath (Join-Path $firmwareRoot 'tools/requirements.txt') -Destination $outputPath
    Copy-Item -LiteralPath (Join-Path $repositoryRoot 'Cargo.lock') -Destination (Join-Path $outputPath 'firmware.Cargo.lock')
    Copy-Item -LiteralPath (Join-Path $firmwareRoot 'tools/board-probe/Cargo.lock') -Destination (Join-Path $outputPath 'decoder.Cargo.lock')
    $decoder = Join-Path $outputPath 'aevia-board-decode.exe'
    $description = & $decoder --describe
    if ($LASTEXITCODE -ne 0) { throw 'Host decoder description failed.' }
    $description | Set-Content -LiteralPath (Join-Path $outputPath 'protocol-info.json') -Encoding UTF8
    $buildId = & $decoder --build-id
    if ($LASTEXITCODE -ne 0 -or $buildId -notmatch '^[0-9a-f]{64}$') { throw 'Host decoder build ID failed.' }
    $buildId | Set-Content -LiteralPath (Join-Path $outputPath 'build-id.txt') -Encoding UTF8

    $sourcePaths = & git -C $repositoryRoot ls-files --cached --others --exclude-standard -- firmware traj Cargo.toml Cargo.lock
    if ($LASTEXITCODE -ne 0) { throw 'Could not enumerate release source files.' }
    $sourceManifest = @($sourcePaths | Sort-Object -Unique | ForEach-Object {
        $sourcePath = Join-Path $repositoryRoot $_
        if (Test-Path -LiteralPath $sourcePath -PathType Leaf) {
            [ordered]@{ file = $_; sha256 = (Get-FileHash -LiteralPath $sourcePath -Algorithm SHA256).Hash.ToLowerInvariant() }
        }
    })
    ConvertTo-Json -InputObject $sourceManifest -Depth 4 |
        Set-Content -LiteralPath (Join-Path $outputPath 'source-sha256.json') -Encoding UTF8
    $gitRevision = & git -C $repositoryRoot rev-parse HEAD
    if ($LASTEXITCODE -ne 0) { throw 'Could not read source revision.' }
    $gitChanges = @(& git -C $repositoryRoot status --short -- firmware traj Cargo.toml Cargo.lock)
    if ($LASTEXITCODE -ne 0) { throw 'Could not read source working-tree status.' }
    $cargoVersion = & $cargoTool --version
    $rustcTool = Resolve-Tool '' 'rustc'
    $rustcVersion = & $rustcTool --version
    $espflashVersion = & $espflashTool --version
    $artifacts = @('firmware.bin', 'firmware.elf', 'aevia-board-decode.exe', 'partitions.csv',
        'app-interface.md', 'race-recording.md', 'recording.py', 'usb_frame.py', 'requirements.txt', 'build-id.txt',
        'firmware.Cargo.lock', 'decoder.Cargo.lock', 'protocol-info.json', 'source-sha256.json') |
        ForEach-Object { Artifact-Entry (Join-Path $outputPath $_) }
    [ordered]@{
        built_utc = [DateTime]::UtcNow.ToString('o')
        git_revision = $gitRevision
        working_tree_changes = $gitChanges
        cargo = $cargoVersion
        rustc = $rustcVersion
        espflash = $espflashVersion
        device_target = $deviceTarget
        decoder_target = $hostTarget
        protocol = ($description | ConvertFrom-Json)
        build_id = $buildId
        host_tests = $(if ($SkipTests) { 'skipped' } else { 'passed' })
        hardware_acceptance = 'not performed by this build script'
        artifacts = @($artifacts)
    } | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $outputPath 'manifest.json') -Encoding UTF8
    @{ complete = $true; completed_utc = [DateTime]::UtcNow.ToString('o') } |
        ConvertTo-Json | Set-Content -LiteralPath $statusPath -Encoding UTF8
    Write-Output "Release bundle: $outputPath"
} finally {
    Pop-Location
}
