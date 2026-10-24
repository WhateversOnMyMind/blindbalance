param([string]$Compiler = 'C:\Program Files\GNU Octave\Octave-10.3.0\mingw64\bin\g++.exe')
$ErrorActionPreference = 'Stop'
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
Push-Location $PSScriptRoot
try {
  foreach ($suite in @('telemetry_timing_test','telemetry_worker_test','alert_window_test')) {
    & $Compiler -std=c++17 -Wall -Wextra -Werror -I fakes "$suite.cpp" -o "$suite.exe"
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $suite" }
  }
  & .\telemetry_timing_test.exe
  if ($LASTEXITCODE -ne 0) { throw 'Timing tests failed' }
  & .\alert_window_test.exe
  if ($LASTEXITCODE -ne 0) { throw 'Alert tests failed' }
  foreach ($case in @('queue-failure','task-failure','overflow','slow','toggle','offline','redirect','http-error','timeout','begin-failure')) {
    & .\telemetry_worker_test.exe $case
    if ($LASTEXITCODE -ne 0) { throw "Worker test failed: $case" }
  }
} finally { Pop-Location }
