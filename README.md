# M55M1 Series CMSIS BSP

To experience the powerful features of M55M1 series in few minutes, please select the sample code to download and execute on the NuMaker-M55M1 board. Open the project files to build them with Keil® MDK, IAR, NuEclipse or VS Code, and then download and trace them on the NuMaker board to see how it works.

Please note that M55M1 Series CMSIS BSP enables CPU Level-1 I/D-Cache by default but does not guarantee Cache coherence.


## .\Document\

- CMSIS.html<br>
	Document of CMSIS version 6.1.0.

- NuMicro M55M1 Series CMSIS BSP Driver Reference Guide.chm<br>
	This document describes the usage of drivers in M55M1 Series BSP.

- NuMicro M55M1 Series CMSIS BSP Revision History.pdf<br>
	This document shows the revision history of M55M1 Series BSP.

- VSCode Quick Start Guide
	This documents guide to install, configure and use VS Code.


## .\Library\

- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V6.1.0 definitions by Arm® Corp.<p>
	M55M1 CMSIS-Drivers do not guarantee thread safety and Cache coherence. The source and RTE_Device header files are in the "Driver\Source" sub-folder. Please add source files and copy RTE_Device header files into your project. Projects can define PRJ_RTE_DEVICE_HEADER macro to include the private RTE_Device.h.

- Commu<br>
	Helper functions of communication protocols, e.g., XMODEM.

- CryptoAccelerator<br>
	Crypto accelerator source code for mbedtls.

- Device<br>
	CMSIS compliant device header files.

- JpegAcceleratorLib<br>
	SIMD accelerator library binary and header files for libjpeg.

- PowerDeliveryLib<br>
	Power delivery library binary and header files for dual, source and sink role.

- SmartcardLib<br>
	Smart card library binary and header files.

- StdDriver<br>
	All peripheral driver header and source files.

- Storage<br>
	Disk I/O modules for FatFs.

- UsbHostLib<br>
	USB host library source code.


## .\Sample Code\

- CortexM55<br>
	Cortex®-M55 sample code.

- Crypto<br>
	Crypto sample code using Mbed TLS library.

- FreeRTOS<br>
	Simple FreeRTOS™ demo code.
	
- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened.<p>
	The hard fault handler shows some information including program counter, which is the address where the processor is executed when the hard fault occurs. The listing file (or map file) can show what function and instruction that is.<p>
	It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample code for In-System-Programming.

- MachineLearning<br>
	Sample code for machine learning.<p>
	For more tools/samples about machine learning training and inference (such as face/pose recognition, face/hand landmark, YOLO object detection, etc), please refer to the following repositories.
	- **NuEdgeWise**: https://github.com/OpenNuvoton/NuEdgeWise
	- **ML_M55M1_SampleCode**: https://github.com/OpenNuvoton/ML_M55M1_SampleCode (Private repository, please contact [Nuvoton support team](https://www.nuvoton.com/ai/contact-us/).)

- NuMaker-M55M1<br>
	Sample code for NuMaker-M55M1 board.

- PowerDelivery<br>
	Sample code for power delivery on M55M1_UTCPD board.

- PowerManagement<br>
	Sample code for power management.

- SecureApplication<br>
	Sample code for secure application.<p>
	VS Code projects require Python 3.12 at least for post-build.

- Semihost<br>
	Show how to print and get character through IDE console window.

- StdDriver<br>
	Sample code to demonstrate the usage of M55M1 series MCU peripheral driver APIs.

- Template<br>
	Project template for M55M1 series MCU.

- TrustZone<br>
	Demo of secure code and non-secure code.

- XOM<br>
	Demonstrate how to create XOM library and use it.


## .\ThirdParty\

- FatFs<br>
	An open-source FAT/exFAT file system library. A generic FAT file system module for small embedded systems.

- FreeRTOS<br>
	Real-time operating system for microcontrollers.

- libjpeg<br>
	A software implements JPEG baseline, extended-sequential, and progressive compression processes maintained and published by the Independent JPEG Group (IJG).

- libmad<br>
	A MPEG audio decoder library that currently supports MPEG-1 and the MPEG-2 extension to lower sampling frequencies, as well as the de facto MPEG 2.5 format. All three audio layers — Layer I, Layer II, and Layer III (i.e., MP3) are fully implemented.

- lwIP<br>
	A widely used open source TCP/IP stack designed for embedded systems.

- mbedtls<br>
	Mbed TLS offers an SSL library with an intuitive API and readable source code, so you can actually understand what the code does.

- ml-embedded-evaluation-kit<br>
	ML embedded evaluation kit provides a range of ready to use machine learning (ML) applications for users to develop ML workloads running on the Arm® Ethos-U NPU and Arm® Cortex-M CPUs. You can also access metrics such as inference cycle count to estimate performance.

- openmv<br>
	An open-source, low-cost machine vision platform. It supports image processing functions such as face detection, key-pointes descriptions, color tracking, QR and Bar code decoding and more.<p>
	This BSP only includes imlib (core library). Python interface is not included.

- paho.mqtt.embedded-c<br>
	Eclipse Paho MQTT C/C++ client for Embedded platforms.

- shine<br>
	A blazing fast MP3 encoding library implemented in fixed-point arithmetic.

- tflite_micro<br>
	TensorFlow Lite for Microcontrollers is a port of TensorFlow Lite designed to run machine learning models on DSPs, microcontrollers, and other devices with limited memory.


## .\Tool\

- imgtool.exe<br>
  imgtool.py<br>
	Used to perform the operations that are necessary to manage keys and sign images.

- OTAServerDemo_v2.2.1.apk<br>
	Secure OTA server Android APP to download firmware images with Secure OTA sample codes.


# License

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.<p>
M55M1 Series BSP files are provided under the Apache-2.0 license.
