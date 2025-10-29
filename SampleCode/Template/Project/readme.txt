This is a template project for M55M1 series MCU.
Users can create their own application based on this project.
 
This template uses internal RC (HIRC) as APLL0 clock source and UART to print messages.
Users may need to do extra system configuration according to their system design.

I/D-Cache
  I/D-Cache are enabled by default for better performance.
  Users can define NVT_DCACHE_ON=0 in project setting to disable D-Cache.
Debug UART
  system_M55M1.c has three weak functions as below to configure DEBUG_PORT debug port.
    SetDebugUartMFP, SetDebugUartCLK and InitDebugUart
  Users can re-implement these functions according to system design.

