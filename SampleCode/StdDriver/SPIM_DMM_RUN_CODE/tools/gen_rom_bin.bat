".\srec_cat.exe" ..\GCC\Release\SPIM_DMM_RUN_CODE.hex -intel -crop 0x00100000 0x002FFFFF -o SPIM_DMM_RUN_CODE_aprom.hex -intel -obs=16
".\srec_cat.exe" SPIM_DMM_RUN_CODE_aprom.hex -intel -offset -0x00100000  -o SPIM_DMM_RUN_CODE_aprom.bin -binary
".\srec_cat.exe" ..\GCC\Release\SPIM_DMM_RUN_CODE.hex -intel -crop 0x80000000 0x81ffffff -offset 0x80000000 -o SPIM_DMM_RUN_CODE_spim0rom.bin -binary

pause
