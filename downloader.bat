<# :
@echo off
setlocal
:: Catch the first argument as TARGET_IP
set "TARGET_IP=%~1"
:: Change to script directory
cd /d "%~dp0"
:: Execute this file as PowerShell, then exit batch immediately
powershell -NoProfile -ExecutionPolicy Bypass -Command "Invoke-Expression ([System.IO.File]::ReadAllText('%~f0'))"
endlocal
exit /b
#>

# --- POWERSHELL SECTION ---
$TargetIP = $env:TARGET_IP

if ([string]::IsNullOrEmpty($TargetIP)) {
    $TargetIP = "127.0.0.1"
    Write-Host "No IP argument provided. Using default: $TargetIP" -ForegroundColor Gray
}

$TargetPort = "80"
$OutputFileName = "response.txt"

$URL = "http://$($TargetIP):$($TargetPort)/"
$OutputPath = Join-Path -Path (Get-Location) -ChildPath $OutputFileName

Write-Host "Attempting GET request to: $URL"
Write-Host "Saving output to: $OutputPath"

try {
    Invoke-WebRequest -Uri $URL -OutFile $OutputPath -UseBasicParsing -ErrorAction Stop
    Write-Host -ForegroundColor Green "`n[Success] Download complete."
}
catch {
    Write-Host -ForegroundColor Red "`n[Error] Failed to download: $($_.Exception.Message)"
}
