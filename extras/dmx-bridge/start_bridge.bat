@echo off
rem JMC DMX Bridge - double-click to start (needs Python 3.8+ from python.org)
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
  py jmc_dmx_bridge.py %*
) else (
  python jmc_dmx_bridge.py %*
)
if errorlevel 1 (
  echo.
  echo The bridge did not start. Is Python installed with "Add python.exe to PATH" ticked?
  pause
)
