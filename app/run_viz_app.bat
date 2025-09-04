@echo off
setlocal enableextensions

REM Run from the folder where this .bat lives
cd /d "%~dp0"

REM --- pick a Python ---
set "PYEXE="
set "PYARGS="

if exist ".venv\Scripts\python.exe" (
  set "PYEXE=%~dp0.venv\Scripts\python.exe"
) else (
  py -3 -V >nul 2>&1
  if not errorlevel 1 (
    set "PYEXE=py"
    set "PYARGS=-3"
  ) else (
    python -V >nul 2>&1
    if not errorlevel 1 (
      set "PYEXE=python"
    ) else (
      python3 -V >nul 2>&1
      if not errorlevel 1 (
        set "PYEXE=python3"
      ) else (
        echo Could not find Python. Install Python 3 or create a .venv next to this .bat.
        pause
        exit /b 1
      )
    )
  )
)

echo Using %PYEXE% %PYARGS%
echo Starting visualizer on UDP 0.0.0.0:12345 ...
"%PYEXE%" %PYARGS% "%~dp0viz_full_with_gyro_accel_mag_3Dobj.py" --show-3d --show-magnitude

echo.
echo (Press any key to close this window)
pause >nul
endlocal
