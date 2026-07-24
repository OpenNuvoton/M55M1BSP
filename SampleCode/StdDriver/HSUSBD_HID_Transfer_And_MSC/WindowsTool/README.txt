Windows Test Tool
=================

DESCRIPTION
-----------
The Windows test tool for this sample is located in the BSP directory:

  BSP/Tool/HIDTransferTest/

This tool is used to verify HID transfer functionality between the host PC
and the M55M1 device running the USBD_HID_Transfer firmware.

USAGE
-----
To run the test with a 512-byte page size:

  HIDTransferTest -pagesize 512 -sectorsize 4096

For full usage information, refer to:

  BSP/Tool/HIDTransferTest/README.txt
