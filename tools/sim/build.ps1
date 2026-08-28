# Build the display simulator on the host.  Run from the repo root:
#   .\tools\sim\build.ps1
#   .\tools\sim\out\sim.exe
#
# PowerShell has no `sh`, and `bash` on a default Windows install resolves to
# WSL — which cannot see the MSYS2 toolchain.  This calls gcc directly instead.
# build.sh is the equivalent for an MSYS2 / WSL / Linux shell.

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$out  = Join-Path $root 'tools\sim\out'
New-Item -ItemType Directory -Force -Path $out | Out-Null

$gcc = (Get-Command gcc -ErrorAction SilentlyContinue).Source
if (-not $gcc) {
    Write-Error "gcc not found on PATH. Install MSYS2 (pacman -S mingw-w64-ucrt-x86_64-gcc) and add C:\msys64\ucrt64\bin to PATH."
}

# MinGW ships no sanitizer runtime; probe rather than assume.
$san = @()
$probe = Join-Path $out '.probe.c'
Set-Content -Path $probe -Value 'int main(void){return 0;}'
& $gcc '-fsanitize=address,undefined' -o (Join-Path $out '.probe.exe') $probe 2>&1 | Out-Null
if ($LASTEXITCODE -eq 0) { $san = @('-fsanitize=address,undefined') }
else { Write-Host 'note: toolchain has no sanitizer runtime - building without it' }
Remove-Item -Force -ErrorAction SilentlyContinue (Join-Path $out '.probe.c'), (Join-Path $out '.probe.exe')

& $gcc -std=c11 -g -O1 -Wall -Wextra @san `
    -I (Join-Path $root 'src') -I (Join-Path $root 'tools\sim') `
    -o (Join-Path $out 'sim.exe') `
    (Join-Path $root 'src\display.c') `
    (Join-Path $root 'tools\sim\sim_stubs.c') `
    (Join-Path $root 'tools\sim\sim_main.c') `
    -lm

if ($LASTEXITCODE -ne 0) { Write-Error "build failed (gcc exit $LASTEXITCODE)" }
Write-Host "built $out\sim.exe"
