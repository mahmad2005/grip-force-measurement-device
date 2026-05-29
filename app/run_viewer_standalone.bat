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
echo Starting standalone viewer...
"%PYEXE%" %PYARGS% "%~dp0web_viz_app\viewer_7371\standalone_viewer.py" %*

if errorlevel 1 (
  echo.
  echo If pywebview is missing, install it with:
  echo   %PYEXE% %PYARGS% -m pip install pywebview
)

echo.
echo (Press any key to close this window)
pause >nul
endlocal
