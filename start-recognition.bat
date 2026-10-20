@echo off
title Maxfield IRL Tracking
rem Double-click to start Maxfield IRL Tracking (installs what it needs the first time).
rem Finds Python: first "py -3" (the Python launcher), then "python". Python 3.9 or newer is needed.
set "MXPY="
py -3 -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1 && set "MXPY=py -3"
if not defined MXPY python -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1 && set "MXPY=python"
if not defined MXPY goto nopython
%MXPY% "%~dp0tools\recognition\start.py" %*
pause
exit /b
:nopython
echo.
echo Python not found: install it from python.org and tick "Add python.exe to PATH".
echo (Python 3.9 or newer. After installing, double-click this file again.)
echo.
pause
exit /b 1
