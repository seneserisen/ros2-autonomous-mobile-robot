@echo off
setlocal
cd /d "%~dp0.."

if "%~1"=="" (
  echo ERROR: No FaultNav workflow action was supplied.
  exit /b 2
)

where py >nul 2>nul
if not errorlevel 1 (
  py -3 "%~dp0faultnav_workflow.py" %~1
  goto finished
)

where python >nul 2>nul
if not errorlevel 1 (
  python "%~dp0faultnav_workflow.py" %~1
  goto finished
)

echo ERROR: Python was not found.
echo.
echo Install Python 3.10 or newer from https://www.python.org/downloads/
echo and enable "Add Python to PATH" during installation.
echo Then run SETUP.bat again.
set "FAULTNAV_EXIT_CODE=1"
goto handle_result

:finished
set "FAULTNAV_EXIT_CODE=%ERRORLEVEL%"

:handle_result
if not "%FAULTNAV_EXIT_CODE%"=="0" (
  echo.
  echo FaultNav did not complete. See START_HERE.md and run DOCTOR.bat.
)
if "%FAULTNAV_EXIT_CODE%"=="0" if /i "%~1"=="run" if not defined FAULTNAV_NO_OPEN (
  echo.
  echo Opening the generated FaultNav report...
  start "" "%~dp0..\artifacts\demo\figure_eight_combined_faults_sensor_report.svg"
)
if not defined FAULTNAV_NO_PAUSE pause
exit /b %FAULTNAV_EXIT_CODE%
