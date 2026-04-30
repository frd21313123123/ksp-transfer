@echo off
setlocal
cd /d "%~dp0"

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Tools\build.ps1" %*
set BUILD_EXIT_CODE=%ERRORLEVEL%

if not "%BUILD_EXIT_CODE%"=="0" (
    echo.
    echo Build failed. If KSP is installed in a custom folder, run:
    echo build.bat "D:\Games\Kerbal Space Program"
)

exit /b %BUILD_EXIT_CODE%
