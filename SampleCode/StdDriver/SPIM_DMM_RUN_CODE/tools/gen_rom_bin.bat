"..\..\tools\srec_cat.exe" %1\SPIM_DMM_RUN_CODE.hex -intel -crop 0x00100000 0x002FFFFF -o SPIM_DMM_RUN_CODE_aprom.hex -intel -obs=16
"..\..\tools\srec_cat.exe" SPIM_DMM_RUN_CODE_aprom.hex -intel -offset -0x00100000  -o ER_ROM -binary
"..\..\tools\srec_cat.exe" %1\SPIM_DMM_RUN_CODE.hex -intel -crop 0x82000000 0x82001000 -offset 0x82000000 -o SPIM.bin -binary
::del "%1\SPIM_DMM_RUN_CODE.bin"
::mkdir "%1\SPIM_DMM_RUN_CODE.bin"
::copy "ER_ROM" "%1/SPIM_DMM_RUN_CODE.bin"
::copy "SPIM.bin" "%1/SPIM_DMM_RUN_CODE.bin"
pause