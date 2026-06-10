@echo off
echo ============================================
echo  KTZ12X40030 - Windows Build
echo ============================================

echo [1/3] Installing dependencies...
pip install -r requirements.txt
pip install pyinstaller
if %errorlevel% neq 0 (
    echo ERROR: pip install failed
    pause & exit /b 1
)

echo [2/3] Building executable...
pyinstaller KTZ12X40030.spec --clean --noconfirm
if %errorlevel% neq 0 (
    echo ERROR: PyInstaller failed
    pause & exit /b 1
)

echo [3/3] Done!
echo.
echo Output: dist\KTZ12X40030.exe
echo.
pause
