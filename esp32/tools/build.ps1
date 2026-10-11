$ErrorActionPreference = 'Stop'
$bridgeRoot = Split-Path -Parent $PSScriptRoot
$bridgeRootItem = Get-Item -LiteralPath $bridgeRoot -Force
if ($bridgeRootItem.LinkType -eq 'Junction') {
    $bridgeRoot = [IO.Path]::GetFullPath([string]$bridgeRootItem.Target)
}
$bridgePackages = Join-Path $env:USERPROFILE '.platformio\packages'
$bridgeBuild = Join-Path $bridgeRoot '.pio\build\esp32dev'
$bridgeIdfPython = Join-Path $env:USERPROFILE '.platformio\penv\.espidf-5.4.2\Scripts\python.exe'
$bridgeNinja = Join-Path $bridgePackages 'tool-ninja\ninja.exe'
$env:IDF_PATH = Join-Path $bridgePackages 'framework-espidf'
$env:IDF_PYTHON_ENV_PATH = Split-Path -Parent (Split-Path -Parent $bridgeIdfPython)
$env:PYTHONIOENCODING = 'utf-8'
$env:IDF_GITHUB_ASSETS = 'dl.espressif.com/github_assets'
$env:PATH = (Join-Path $bridgePackages 'toolchain-xtensa-esp-elf\bin') + ';' + (Split-Path -Parent $bridgeNinja) + ';' + (Join-Path $bridgePackages 'tool-cmake\bin') + ';' + $env:PATH
& $bridgeIdfPython (Join-Path $bridgeRoot 'tools\configure_minimal.py')
if ($LASTEXITCODE -ne 0) { throw 'Configuration failed' }
& (Join-Path $bridgePackages 'tool-cmake\bin\cmake.exe') -S $bridgeRoot -B $bridgeBuild -G Ninja ('-DCMAKE_MAKE_PROGRAM=' + $bridgeNinja) '-DIDF_TARGET=esp32' '-DPYTHON_DEPS_CHECKED=1' ('-DPYTHON=' + $bridgeIdfPython) ('-DSDKCONFIG=' + (Join-Path $bridgeRoot 'sdkconfig.esp32dev'))
if ($LASTEXITCODE -ne 0) { throw 'CMake failed' }
& $bridgeNinja -C $bridgeBuild -j 8
if ($LASTEXITCODE -ne 0) { throw 'Compilation failed' }
