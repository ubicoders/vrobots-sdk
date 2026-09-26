<#
.SYNOPSIS
    Download the VRobots SDK C bundle that matches this checkout, verify it and
    unpack it where the C++ examples' CMake build finds it on its own.

.DESCRIPTION
    Windows x86-64 (MSVC) counterpart of scripts/get_cpp_bundle.sh.

      pwsh scripts/get_cpp_bundle.ps1            # the version of this checkout
      pwsh scripts/get_cpp_bundle.ps1 0.1.11     # a specific version

    Reads the SDK version from crates/vrobots-sdk-sys/Cargo.toml unless one is
    given, downloads vrobots_sdk-cpp-<version>-windows-x86_64.zip and SHA256SUMS
    from the GitHub Release of that version, checks the archive's SHA-256, and
    unpacks it into ..\vrobots_sdk-cpp-<version>-windows-x86_64\ next to this
    repository, the folder examples\cpp\CMakeLists.txt looks for.

    Then, from the repository root:

      cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release
      cmake --build target/cpp-build --config Release
      .\target\cpp-build\Release\ex01_hello_states.exe
#>
[CmdletBinding()]
param([string] $Version = "")

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$Os = 'windows-x86_64'

if (-not $Version) {
    $m = Select-String -Path (Join-Path $RepoRoot 'crates\vrobots-sdk-sys\Cargo.toml') -Pattern '^version\s*=\s*"([^"]+)"' | Select-Object -First 1
    if ($m) { $Version = $m.Matches[0].Groups[1].Value }
}
if (-not $Version) { throw 'could not read the version; pass it as the first argument' }
$Version = $Version.TrimStart('v')

$Archive = "vrobots_sdk-cpp-$Version-$Os.zip"
$BaseUrl = "https://github.com/ubicoders/vrobots-sdk/releases/download/v$Version"
$Dest = Join-Path (Split-Path $RepoRoot -Parent) "vrobots_sdk-cpp-$Version-$Os"

if ((Test-Path (Join-Path $Dest 'include\vrobots_sdk.h')) -and (Test-Path (Join-Path $Dest 'lib\vrobots_sdk_capi.dll'))) {
    Write-Host "already unpacked: $Dest"
    exit 0
}

$Work = Join-Path ([System.IO.Path]::GetTempPath()) ("vrobots-sdk-" + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $Work | Out-Null
try {
    Write-Host "downloading $Archive from $BaseUrl"
    Invoke-WebRequest -Uri "$BaseUrl/$Archive" -OutFile (Join-Path $Work $Archive)
    Invoke-WebRequest -Uri "$BaseUrl/SHA256SUMS" -OutFile (Join-Path $Work 'SHA256SUMS')

    Write-Host 'verifying'
    $expected = (Get-Content (Join-Path $Work 'SHA256SUMS') | Where-Object { $_ -match "\s\*?$([regex]::Escape($Archive))$" }) -replace '\s.*$', ''
    if (-not $expected) { throw "SHA256SUMS does not list $Archive" }
    $actual = (Get-FileHash -Algorithm SHA256 (Join-Path $Work $Archive)).Hash.ToLower()
    if ($actual -ne $expected.ToLower()) { throw "checksum mismatch for $Archive`n  expected $expected`n  got      $actual" }

    New-Item -ItemType Directory -Path $Dest -Force | Out-Null
    Expand-Archive -Path (Join-Path $Work $Archive) -DestinationPath $Dest -Force
    $Dest = (Resolve-Path $Dest).Path

    Write-Host ''
    Write-Host "unpacked SDK $Version into $Dest"
    Write-Host '  include\vrobots_sdk.h, include\vrobots_sdk.hpp, lib\vrobots_sdk_capi.dll (+ .dll.lib)'
    Write-Host ''
    Write-Host 'Build and run the examples from the repository root:'
    Write-Host ''
    Write-Host '  cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release'
    Write-Host '  cmake --build target/cpp-build --config Release'
    Write-Host '  .\target\cpp-build\Release\ex01_hello_states.exe'
    Write-Host ''
    Write-Host "For your own project, add $Dest\include to the include path, link"
    Write-Host "$Dest\lib\vrobots_sdk_capi.dll.lib, and keep vrobots_sdk_capi.dll beside the executable."
}
finally {
    Remove-Item -Recurse -Force $Work -ErrorAction SilentlyContinue
}
