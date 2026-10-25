@echo off
title Mark the robot (MIRLT)
rem Double-click to mark the robot on recorded footage (tools/label/mark_robot.py).
rem The page opens in the browser by itself; leave this window open while you mark.
rem Finds Python: first "py -3" (the Python launcher), then "python". Python 3.9 or newer is needed.
set "MXPY="
py -3 -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1 && set "MXPY=py -3"
if not defined MXPY python -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1 && set "MXPY=python"
if not defined MXPY goto nopython
%MXPY% -c "import cv2, numpy" >nul 2>&1 || %MXPY% -m pip install numpy opencv-python
%MXPY% "%~dp0tools\label\mark_robot.py" %*
pause
exit /b
:nopython
echo.
echo Python not found: install it from python.org and tick "Add python.exe to PATH".
echo (Python 3.9 or newer. After installing, double-click this file again.)
echo.
pause
exit /b 1
