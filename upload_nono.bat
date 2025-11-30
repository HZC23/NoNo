@echo off
set SKETCH="C:\Users\Hadrien\Documents\Arduino\NoNo\NoNo.ino"
set FQBN=arduino:avr:mega
set BAUD=115200

echo ===== Détection automatique du Bluno Mega =====
for /f "tokens=1,2,* delims= " %%A in ('arduino-cli board list ^| findstr "Mega"') do (
    set PORT=%%A
)

if "%PORT%"=="" (
    echo Aucun Bluno Mega detecte sur les ports COM.
    pause
    exit /b 1
)

echo Bluno Mega detecte sur %PORT%
echo.

:COMPILE
echo ===== Compilation =====
arduino-cli compile --fqbn %FQBN% %SKETCH%
IF %ERRORLEVEL% NEQ 0 (
    echo Erreur lors de la compilation. Nouvelle tentative dans 5 secondes...
    timeout /t 5
    goto COMPILE
)

:UPLOAD
echo ===== Téléversement =====
arduino-cli upload -p %PORT% --fqbn %FQBN% %SKETCH% --verbose
IF %ERRORLEVEL% NEQ 0 (
    echo Erreur lors du téléversement. Nouvelle tentative dans 5 secondes...
    timeout /t 5
    goto UPLOAD
)

REM Petite pause pour laisser la carte redémarrer
timeout /t 2

echo ===== Téléversement terminé =====
echo Ouverture du moniteur série...
arduino-cli monitor -p %PORT% --config baudrate=%BAUD%
pause
exit /b 0
