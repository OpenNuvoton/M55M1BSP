Windows Test Tool - HIDTransferTest
=====================================

A Windows command-line tool for testing HID Transfer on the Printer + HID composite device.

Location
--------
BSP/Tool/HIDTransferTest

Usage
-----
HIDTransferTest -pid 0xAACC -pagesize 4096

Options
-------
  -pid 0xAACC      Specify the USB Product ID of the HID interface (Printer + HID composite)
  -pagesize 4096   Set the page size to 4096 bytes (matches M55M1 RRAM page size)

Description
-----------
This tool communicates with the M55M1 device running the USBD_Printer_And_HID_Transfer
firmware. It performs read/write/erase operations over the HID interface of the
Printer + HID composite device, using the specified PID to identify the correct
USB device and a page size of 4096 bytes aligned to the M55M1 RRAM page size.
