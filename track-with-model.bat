@echo off
title Maxfield IRL Tracking
rem Double-click to follow objects in a recording or a live camera with a trained model.
rem With nothing typed after it, it asks plain questions (which project, which recording or camera).
rem For a live camera it starts with a roll call on the page: confirm what it sees once, or fix it, then it follows the objects.
rem The first time, it installs the few Python packages it needs.
rem Finds Python: first "py -3" (the Python launcher), then "python". Python 3.9 or newer is needed.
set "MXPY="
py -3 -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1 && set "MXPY=py -3"
if not defined MXPY python -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1 && set "MXPY=python"
if not defined MXPY goto nopython
%MXPY% "%~dp0tools\recognition\use_model.py" %*
pause
exit /b
:nopython
echo.
echo Python not found: install it from python.org and tick "Add python.exe to PATH".
echo (Python 3.9 or newer. After installing, double-click this file again.)
echo.
pause
exit /b 1
