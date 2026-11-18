param([string]$Destination = '')
$ErrorActionPreference = 'Stop'
$RepositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$PackageRoot = Join-Path $RepositoryRoot 'target/ppk-tools'
$Archive = Join-Path $PackageRoot 'RTKLIB_EX_2.5.1.zip'
$Expanded = Join-Path $PackageRoot 'RTKLIB-EX-2.5.1-bin'
$ReleaseRoot = Join-Path $Expanded 'RTKLIB_EX_2.5.1'
$Expected = '0b20c940b1939b0701f0ee027daf5225221120dbff50c3f563f8705fd0b496f5'
New-Item -ItemType Directory -Force -Path $PackageRoot | Out-Null
if (-not (Test-Path -LiteralPath $Archive)) {
    Invoke-WebRequest -Uri 'https://github.com/rtklibexplorer/RTKLIB/releases/download/v2.5.1/RTKLIB_EX_2.5.1.zip' -OutFile $Archive
}
$ArchiveStream = [IO.File]::OpenRead($Archive)
$Hasher = [Security.Cryptography.SHA256]::Create()
try { $Actual = ([BitConverter]::ToString($Hasher.ComputeHash($ArchiveStream))).Replace('-', '').ToLowerInvariant() }
finally { $ArchiveStream.Dispose(); $Hasher.Dispose() }
if ($Actual -ne $Expected) {
    throw 'RTKLIB release archive checksum differs. No executable was installed.'
}
if ([string]::IsNullOrWhiteSpace($Destination)) { $Destination = $ReleaseRoot }
New-Item -ItemType Directory -Force -Path $Destination | Out-Null
$Destination = (Resolve-Path -LiteralPath $Destination).Path
Add-Type -AssemblyName System.IO.Compression.FileSystem
$Zip = [IO.Compression.ZipFile]::OpenRead($Archive)
try {
    foreach ($Name in @('convbin.exe', 'rnx2rtkp.exe')) {
        $Entry = $Zip.GetEntry("RTKLIB_EX_2.5.1/$Name")
        if ($null -eq $Entry) { throw "Missing executable $Name in verified archive" }
        $InputStream = $Entry.Open()
        $OutputStream = [IO.File]::Create((Join-Path $Destination $Name))
        try { $InputStream.CopyTo($OutputStream) }
        finally { $InputStream.Dispose(); $OutputStream.Dispose() }
    }
} finally { $Zip.Dispose() }
foreach ($Name in @('RTKLIB-LICENSE.txt', 'RTKLIB-NOTICE.txt')) {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $Name) -Destination (Join-Path $Destination $Name) -Force
}
Write-Output "Installed checksum-verified RTKLIB Explorer 2.5.1 command-line tools and notices in $Destination"
