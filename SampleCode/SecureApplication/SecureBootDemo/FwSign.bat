@echo off
rem https://docs.mcuboot.com/imgtool.html

set call_path=%cd%
set script_path=%~dp0
set bin_file=%1
for %%i in (%bin_file%) do set bin_basename=%%~ni
set key_file=%2
for %%i in (%key_file%) do set key_basename=%%~dpni

@REM    To generate ECC P-256 private key
if not exist "%call_path%\%key_file%" (
    "%script_path%\imgtool" keygen -k "%call_path%\%key_file%" -t ecdsa-p256
)

@REM    To generate ECC P-256 public key
if not exist "%key_basename%.pub" (
    "%script_path%\imgtool" getpub -k "%call_path%\%key_file%" > "%key_basename%.pub"
)

@REM    To sign image
"%script_path%\imgtool" sign ^
-k "%call_path%\%key_file%" --public-key-format hash --align 4 ^
-v 1.2.3 -H 0x800 --pad-header ^
-S 0x40000 -s 0 "%bin_file%" "%script_path%\%bin_basename%_signed.bin"
