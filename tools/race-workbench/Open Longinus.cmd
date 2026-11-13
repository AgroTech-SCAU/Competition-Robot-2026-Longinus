@echo off
where python >nul 2>nul
if errorlevel 1 goto use_py
python "%~dp0launch.py" %*
if errorlevel 1 goto launch_error
exit /b %errorlevel%
:use_py
where py >nul 2>nul
if errorlevel 1 goto missing
py -3 "%~dp0launch.py" %*
if errorlevel 1 goto launch_error
exit /b %errorlevel%
:missing
echo Python 3 is required to launch the local workbench.
pause
exit /b 1
:launch_error
pause
exit /b 1
