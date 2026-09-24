@echo off
rem ============================================================
rem  generate_bin.bat
rem  Convert the axf to a single APP bin file (ER_IROM1 region).
rem
rem  Usage: generate_bin.bat "<path to fromelf.exe>" [NAME] [OUTDIR]
rem    %1  path to fromelf.exe (uVision passes $K\ARM\ARMCC\bin\fromelf.exe)
rem    %2  output base name (default HIT_TB_PM_APP; the app_boot target
rem        passes HIT_TB_PM_APP_BOOT)
rem    %3  output directory, relative to this .bat (default output\debug;
rem        the app_boot target passes output\debug_boot)
rem
rem The image may contain one or two load regions:
rem   - app  mode (base 0x8000): ER_IROM1 + ICG(0x400) -> two load regions,
rem     fromelf treats --output as a directory and writes one file per
rem     region; this script promotes ER_IROM1 to the final .bin.
rem   - boot mode (base 0x0): the ICG is merged into ER_IROM1 -> a single
rem     load region, exported directly as one .bin.
rem
rem Output: <OUTDIR>\<NAME>.bin
rem Note:   the first byte of the produced bin corresponds to the ER_IROM1
rem         base address (0x00008000 in app mode, 0x00000000 in boot mode),
rem         which is where a Y-MODEM IAP upgrade must program it.
rem
rem NOTE: keep this file ASCII-only. cmd.exe decodes .bat with the system
rem       ANSI code page (GBK on Chinese Windows); non-ASCII UTF-8 bytes in
rem       comments turn into garbage that may be parsed as commands.
rem ============================================================
setlocal

if "%~1"=="" (
    echo [generate_bin] ERROR: fromelf path not given
    exit /b 1
)

set "FROMELF=%~1"
set "NAME=%~2"
if not defined NAME set "NAME=HIT_TB_PM_APP"
set "OUTDIR=%~3"
if not defined OUTDIR set "OUTDIR=output\debug"
rem relative output dirs are resolved against this .bat location (MDK project dir)
set "OUTDIR=%~dp0%OUTDIR%"
set "REGION=ER_IROM1"
set "TMPDIR=%OUTDIR%\_bin_tmp"

if not exist "%OUTDIR%\%NAME%.axf" (
    echo [generate_bin] ERROR: %OUTDIR%\%NAME%.axf not found
    exit /b 1
)

rem cleanup stale outputs (TMPDIR may be a leftover directory or a bad file)
if exist "%TMPDIR%" (
    if exist "%TMPDIR%\" (
        rd /s /q "%TMPDIR%"
    ) else (
        del /f /q "%TMPDIR%"
    )
)
if exist "%OUTDIR%\%NAME%.bin\" (
    rd /s /q "%OUTDIR%\%NAME%.bin"
) else if exist "%OUTDIR%\%NAME%.bin" (
    del /f /q "%OUTDIR%\%NAME%.bin"
)

rem export per-region files (fromelf treats output as a directory
rem whenever the axf contains more than one load region)
if not exist "%TMPDIR%" mkdir "%TMPDIR%"
rem with multiple load regions --output must be a directory; the trailing /
rem avoids the cmd quirk that glues a trailing backslash to the quote
"%FROMELF%" --bin --output "%TMPDIR%/" "%OUTDIR%\%NAME%.axf"
if errorlevel 1 (
    echo [generate_bin] ERROR: fromelf failed
    exit /b 1
)

if exist "%TMPDIR%\%REGION%" (
    copy /y "%TMPDIR%\%REGION%" "%OUTDIR%\%NAME%.bin" >nul
    if errorlevel 1 (
        echo [generate_bin] ERROR: copy failed
        exit /b 1
    )
    rd /s /q "%TMPDIR%"
    goto :done
)

rem single load region (boot mode): fromelf writes the same base name inside
rem the output directory
if exist "%TMPDIR%\%NAME%.bin" (
    copy /y "%TMPDIR%\%NAME%.bin" "%OUTDIR%\%NAME%.bin" >nul
    if errorlevel 1 (
        echo [generate_bin] ERROR: copy failed
        exit /b 1
    )
    rd /s /q "%TMPDIR%"
    goto :done
)
rd /s /q "%TMPDIR%" 2>nul

"%FROMELF%" --bin --output "%OUTDIR%\%NAME%.bin" "%OUTDIR%\%NAME%.axf"
if errorlevel 1 (
    echo [generate_bin] ERROR: fromelf failed
    exit /b 1
)

:done
if not exist "%OUTDIR%\%NAME%.bin" (
    echo [generate_bin] ERROR: %NAME%.bin not generated
    exit /b 1
)

for %%A in ("%OUTDIR%\%NAME%.bin") do echo [generate_bin] OK: %%~fA (%%~zA bytes)
exit /b 0
