@echo off
setlocal

set "SRC=%~dp0"
set "DEST=%~1"

if "%DEST%"=="" set "DEST=E:\"
if not "%DEST:~-1%"=="\" set "DEST=%DEST%\"

if not exist "%DEST%" (
    echo Target drive not found: %DEST%
    echo Usage: sync_modules_to_openmv.bat [drive:\]
    exit /b 1
)

echo Sync Python modules from:
echo   %SRC%
echo to:
echo   %DEST%
echo.

for %%F in ("%SRC%*.py") do (
    if /I not "%%~nxF"=="main.py" (
        echo copy %%~nxF
        copy /Y "%%~fF" "%DEST%" >nul
        if errorlevel 1 (
            echo Failed to copy %%~nxF
            exit /b 1
        )
    )
)

echo.
echo Done.
