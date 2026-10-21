@echo off
title E02 accuracy experiment
rem Double-click to run the E02 accuracy experiment (about 2 hours). Start the Pi recorder first.
rem If an unfinished session is on this laptop, it asks whether to carry it on.
rem Finds Python: first "py -3" (the Python launcher), then "python". Python 3.9 or newer is needed.
set "MXPY="
py -3 -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1 && set "MXPY=py -3"
if not defined MXPY python -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1 && set "MXPY=python"
if not defined MXPY goto nopython
%MXPY% -c "import cv2, numpy" >nul 2>&1 || %MXPY% -m pip install numpy opencv-python
%MXPY% "%~dp0tools\experiment\e02.py" %*
pause
exit /b
:nopython
echo.
echo Python not found: install it from python.org and tick "Add python.exe to PATH".
echo (Python 3.9 or newer. After installing, double-click this file again.)
echo.
pause
exit /b 1
