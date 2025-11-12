#!/usr/bin/env pwsh
# Quick build script with timeout protection
param(
    [string]$env = "esp32c6_wifi_test",
    [int]$timeout = 300  # 5 minutes max
)

Write-Host "Building $env with timeout of $timeout seconds..." -ForegroundColor Cyan

$job = Start-Job -ScriptBlock {
    param($environment)
    cd "C:\Users\felix\Documents\Master 1 ECAM\Embbeded_project\Projet_git\EuroBot_2026"
    platformio run -e $environment 2>&1
} -ArgumentList $env

$result = Wait-Job -Job $job -Timeout $timeout

if ($result) {
    # Job completed within timeout
    $output = Receive-Job -Job $job
    Write-Host $output
    Remove-Job -Job $job
    Write-Host "`nBuild completed!" -ForegroundColor Green
} else {
    # Timeout occurred
    Stop-Job -Job $job
    Remove-Job -Job $job
    Write-Host "`nBuild TIMEOUT after $timeout seconds!" -ForegroundColor Red
    Write-Host "The build is taking too long. Possible solutions:" -ForegroundColor Yellow
    Write-Host "1. Use VS Code PlatformIO extension UI (graphical)"
    Write-Host "2. Check internet connection (large downloads)"
    Write-Host "3. Try: platformio run -e $env --verbose 2>&1 | Select-Object -First 100"
}
