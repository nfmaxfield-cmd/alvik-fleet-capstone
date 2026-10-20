@echo off
title Maxfield IRL Tracking
rem Double-click to follow objects in a recording or a live camera with a trained model.
rem With nothing typed after it, it asks plain questions (which project, which recording or camera).
rem The first time, it installs the few Python packages it needs.
python "%~dp0tools\recognition\use_model.py" %*
pause
