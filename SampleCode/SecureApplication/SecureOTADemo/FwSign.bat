@echo off
REM https://docs.mcuboot.com/imgtool.html
REM Usage: $0 bin_file key_file [fw_ver]

set call_path=%cd%
set script_path=%~dp0
set bin_file=%1
@REM Extracting file name
for %%i in (%bin_file%) do set bin_basename=%%~ni
@REM Extracting directory path
for %%I in ("%bin_file%") do set bin_dirpath=%%~dpI
set key_file=%2
for %%i in (%key_file%) do set key_basename=%%~dpni
set fw_ver=%3
if [%3] == [] set fw_ver=0

REM Test script to workaround check if bin_file contain space
REM if not exist %bin_file% (
    REM set bin_file="%bin_file%"

    REM if not exist %bin_file% (
        REM echo "%bin_file% not found !"
        REM exit 1
    REM )
REM )

@REM    To generate ECC P-256 private key
if not exist "%call_path%\%key_file%" (
    "%script_path%\..\..\..\Tool\imgtool" keygen -k "%call_path%\%key_file%" -t ecdsa-p256
)

@REM    To generate ECC P-256 public key
if not exist "%key_basename%_pub.c" (
    "%script_path%\..\..\..\Tool\imgtool" getpub -k "%call_path%\%key_file%" > "%key_basename%_pub.c"
)

@REM    To sign image
"%script_path%\..\..\..\Tool\imgtool" sign ^
-k "%call_path%\%key_file%" --public-key-format hash --align 4 ^
-v 1.2.3 -H 0x400 --pad-header ^
-S 0x40000 -s %fw_ver% %bin_file% "%script_path%\%bin_basename%_signed.bin"

@REM    Align signed image file size to 4 bytes for OTA App
"%script_path%\..\srec_cat.exe" "%script_path%\%bin_basename%_signed.bin" -binary ^
-fill 0x00 -within "%script_path%\%bin_basename%_signed.bin" -binary -range-padding 4 ^
-o "%script_path%\%bin_basename%_signed.bin" -binary

@REM    Generate disassembly
if exist "%bin_dirpath%\%bin_basename%.axf" (
    C:\Keil_v5\ARM\ARMCLANG\bin\fromelf.exe --text -c "%bin_dirpath%\%bin_basename%.axf" --output "%bin_dirpath%\%bin_basename%.txt"
)
