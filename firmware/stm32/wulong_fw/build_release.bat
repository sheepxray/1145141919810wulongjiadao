@echo off
REM ============================================================================
REM Local build + package for GitHub Release (STM32G474 firmware)
REM ============================================================================
REM
REM WHY LOCAL BUILD INSTEAD OF CI:
REM   1. This is a Keil MDK project. Keil is Windows-only commercial software
REM      and cannot run on a Linux CI runner.
REM   2. Switching to GCC would require supplying CMSIS Core headers (absent
REM      from the repo -- Keil pulls them from its RTE pack manager), rewriting
REM      the linker script, and verifying FPU/FreeRTOS port agreement. High risk.
REM   3. Keil is already installed locally and command-line build is verified
REM      working (UV4 -b returns 0, 0 errors 0 warnings).
REM   4. Firmware releases are infrequent; local build + upload is sufficient.
REM
REM NOTE: This file is deliberately ASCII-only. A .bat containing UTF-8
REM       non-ASCII text is mis-decoded by cmd.exe (GBK on zh-CN Windows),
REM       which corrupts the script body and breaks parsing.
REM
REM USAGE:
REM   build_release.bat                 version = git tag (auto)
REM   build_release.bat v1.0.0          explicit version
REM
REM OUTPUT (in dist\ at repo root):
REM   wulong_fw_<ver>.hex     flash image (STM32CubeProgrammer / ST-Link)
REM   wulong_fw_<ver>.bin     raw binary (custom flasher / OTA)
REM   wulong_fw_<ver>.elf     debug (GDB / Keil)
REM   wulong_fw_<ver>.map     linker map (Flash/RAM usage analysis)
REM   SHA256SUMS.txt          checksums
REM   release_notes.md        release notes template
REM ============================================================================

setlocal enabledelayedexpansion

REM ---- Paths ----
set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "MDK_DIR=%SCRIPT_DIR%\MDK-ARM"
set "UVPROJX=wulong_fw.uvprojx"
set "OUT_NAME=wulong_fw"
set "DIST=%SCRIPT_DIR%\..\..\..\dist"

if not defined KEIL_PATH set "KEIL_PATH=C:\Keil_v5"
set "UV4=%KEIL_PATH%\UV4\UV4.exe"
set "FROMELF=%KEIL_PATH%\ARM\ARMCC\bin\fromelf.exe"

REM ---- Version ----
set "VERSION=%~1"
if "%VERSION%"=="" (
    for /f "delims=" %%i in ('git describe --tags --always --dirty 2^>nul') do set "VERSION=%%i"
)
if "%VERSION%"=="" set "VERSION=dev"

for /f "delims=" %%i in ('git rev-parse --short HEAD 2^>nul') do set "GIT_SHA=%%i"
if "%GIT_SHA%"=="" set "GIT_SHA=unknown"

echo ============================================================
echo  Building STM32G474 firmware
echo ============================================================
echo   Version : %VERSION%
echo   Commit  : %GIT_SHA%
echo   Project : %MDK_DIR%\%UVPROJX%
echo.

REM ---- Preflight ----
if not exist "%UV4%" (
    echo [ERROR] Keil UV4 not found: %UV4%
    echo         Set KEIL_PATH to your Keil install directory.
    exit /b 1
)
if not exist "%MDK_DIR%\%UVPROJX%" (
    echo [ERROR] Project file not found: %MDK_DIR%\%UVPROJX%
    exit /b 1
)

REM ---- Clean previous outputs (avoid stale artifacts) ----
if exist "%MDK_DIR%\%OUT_NAME%" (
    echo [1/5] Cleaning previous build outputs...
    del /q "%MDK_DIR%\%OUT_NAME%\*.axf" 2>nul
    del /q "%MDK_DIR%\%OUT_NAME%\*.hex" 2>nul
    del /q "%MDK_DIR%\%OUT_NAME%\*.bin" 2>nul
) else (
    echo [1/5] No previous outputs, skipping clean
)

REM ---- Build ----
echo [2/5] Invoking Keil build (may take 10-60s)...
pushd "%MDK_DIR%"
"%UV4%" -b "%UVPROJX%" -j0 -o "build_log_release.txt"
set "BUILD_RC=!ERRORLEVEL!"
popd

REM Keil exit codes: 0=no err/warn 1=warnings 2=errors 3=fatal 11=cant open
REM                  12=device err 13=write err 15=read err 20=license
if "!BUILD_RC!"=="0" (
    echo       Build OK, 0 errors 0 warnings
) else if "!BUILD_RC!"=="1" (
    echo       Build finished with warnings, exit code 1 - continuing
) else (
    echo.
    echo [ERROR] Build failed. Keil exit code = !BUILD_RC!
    echo         Codes: 2=errors 3=fatal 11=cannot open 12=device 15=read 20=license
    echo.
    echo ------ tail of build log ------
    if exist "%MDK_DIR%\build_log_release.txt" (
        powershell -NoProfile -Command "Get-Content '!MDK_DIR!\build_log_release.txt' -Tail 30"
    )
    exit /b !BUILD_RC!
)

set "AXF=%MDK_DIR%\%OUT_NAME%\%OUT_NAME%.axf"
if not exist "%AXF%" (
    echo [ERROR] Build reported success but artifact missing: %AXF%
    exit /b 1
)

REM ---- Package ----
echo [3/5] Generating release artifacts...
if not exist "%DIST%" mkdir "%DIST%"

set "REL_NAME=wulong_fw_%VERSION%"

if exist "%MDK_DIR%\%OUT_NAME%\%OUT_NAME%.hex" (
    copy /y "%MDK_DIR%\%OUT_NAME%\%OUT_NAME%.hex" "%DIST%\%REL_NAME%.hex" >nul
) else (
    echo       hex not found, generating with fromelf
    "%FROMELF%" --i32combined --output "%DIST%\%REL_NAME%.hex" "%AXF%"
)

"%FROMELF%" --bin --output "%DIST%\%REL_NAME%.bin" "%AXF%"
if errorlevel 1 (
    echo [ERROR] fromelf failed to produce bin
    exit /b 1
)

copy /y "%AXF%" "%DIST%\%REL_NAME%.elf" >nul

if exist "%MDK_DIR%\%OUT_NAME%\%OUT_NAME%.map" (
    copy /y "%MDK_DIR%\%OUT_NAME%\%OUT_NAME%.map" "%DIST%\%REL_NAME%.map" >nul
)

REM ---- Checksums ----
echo [4/5] Computing SHA256...
pushd "%DIST%"
powershell -NoProfile -Command "Get-ChildItem '%REL_NAME%.*' | ForEach-Object { $h = (Get-FileHash $_.FullName -Algorithm SHA256).Hash.ToLower(); '{0}  {1}' -f $h, $_.Name } | Set-Content -Encoding ASCII 'SHA256SUMS.txt'"
popd

REM ---- Release notes ----
REM NOTE: inside a parenthesised block, '|' and '>' are still special to cmd
REM       even when quoted, so the whole block is emitted with a single
REM       redirected echo per line using the ^| escape. Building the file
REM       with PowerShell instead avoids the escaping minefield entirely.
echo [5/5] Writing release notes...
powershell -NoProfile -Command ^
  "$lines = @(" ^
  "  '# Firmware %VERSION%'," ^
  "  ''," ^
  "  'STM32G474VETx motion control firmware.'," ^
  "  ''," ^
  "  '| Field | Value |'," ^
  "  '|---|---|'," ^
  "  '| Version | %VERSION% |'," ^
  "  '| Commit | %GIT_SHA% |'," ^
  "  '| Target | STM32G474VETx |'," ^
  "  ''," ^
  "  '## Artifacts'," ^
  "  ''," ^
  "  '| File | Purpose |'," ^
  "  '|---|---|'," ^
  "  '| wulong_fw_%VERSION%.hex | Flash image (STM32CubeProgrammer / ST-Link) |'," ^
  "  '| wulong_fw_%VERSION%.bin | Raw binary (custom flasher / OTA) |'," ^
  "  '| wulong_fw_%VERSION%.elf | Debug symbols (GDB / Keil) |'," ^
  "  '| wulong_fw_%VERSION%.map | Linker map (Flash/RAM usage) |'," ^
  "  '| SHA256SUMS.txt | Checksums |'," ^
  "  ''," ^
  "  '## Flashing'," ^
  "  ''," ^
  "  '```sh'," ^
  "  'STM32_Programmer_CLI -c port=SWD -w wulong_fw_%VERSION%.hex -v -rst'," ^
  "  '```'," ^
  "  ''," ^
  "  '## Verify'," ^
  "  ''," ^
  "  '```sh'," ^
  "  'sha256sum -c SHA256SUMS.txt'," ^
  "  '```'" ^
  ");" ^
  "$lines -join [Environment]::NewLine | Set-Content -Encoding UTF8 '%DIST%\release_notes.md'"

REM ---- Summary ----
echo.
echo ============================================================
echo  Build complete
echo ============================================================
echo.
echo Output directory: %DIST%
echo.
for %%f in ("%DIST%\%REL_NAME%.hex" "%DIST%\%REL_NAME%.bin" "%DIST%\%REL_NAME%.elf" "%DIST%\%REL_NAME%.map") do (
    if exist %%f for %%s in (%%f) do echo   %%~nxs  %%~zs bytes
)
echo.
echo Next step -- upload the release:
echo   gh release create %VERSION% "%DIST%\%REL_NAME%.hex" "%DIST%\%REL_NAME%.bin" "%DIST%\%REL_NAME%.elf" "%DIST%\%REL_NAME%.map" "%DIST%\SHA256SUMS.txt" --notes-file "%DIST%\release_notes.md"
echo.
echo Or upload the files in %DIST% via the GitHub web UI.
echo.

endlocal
exit /b 0
