@echo off
setlocal

:: 1. Check if the user provided an argument
if "%~1"=="" (
    echo [ERROR] No argument provided.
    echo Usage: %~nx0 folder_name
    goto :EOF
)

:: 2. Set variables for clarity
:: %~1 removes quotes from the argument if the user added them
set "targetFolder=%~1"
set "sourceFile=sensor_data.csv"
set "destPath=.\data\%targetFolder%\measured.txt"

:: 3. Check if source file exists in current directory
if not exist "%sourceFile%" (
    echo [ERROR] "%sourceFile%" not found in the current directory.
    goto :EOF
)
:: 4. Create destination directory if it doesn't exist
if not exist ".\data\%targetFolder%\" (
    mkdir ".\data\%targetFolder%"
)
:: 5. Copy the file and rename it
:: /Y suppresses the prompt to overwrite if the file already exists
copy /Y "%sourceFile%" "%destPath%"

:: 6. Verify success
if %errorlevel% equ 0 (
    echo Success! Content copied to: %destPath%
) else (
    echo [ERROR] The copy operation failed.
)

endlocal