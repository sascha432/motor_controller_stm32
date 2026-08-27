@echo off
REM Launcher for the motor configuration and parameter tuning tool
REM Runs motor_config.py using the project virtual environment.
REM Uses pythonw.exe so no console window appears, only the GUI.

setlocal
cd /d "%~dp0"

set "PYTHONW_EXE=..\.venv\Scripts\pythonw.exe"

if exist "%PYTHONW_EXE%" (
    start "" "%PYTHONW_EXE%" motor_config.py
) else (
    start "" pythonw motor_config.py
)

endlocal
