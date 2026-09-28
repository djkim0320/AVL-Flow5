@echo off
setlocal
cd /d "%~dp0"
title DBF Studio
if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] Project Python environment is missing: .venv\Scripts\python.exe
    pause
    exit /b 1
)
".venv\Scripts\python.exe" "ui\launch.py" %*
if errorlevel 1 (
    echo.
    echo DBF Studio could not open. See the message above.
    pause
    exit /b 1
)
endlocal
