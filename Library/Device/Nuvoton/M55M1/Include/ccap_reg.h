/**************************************************************************//**
 * @file     ccap_reg.h
 * @version  V1.00
 * @brief    CCAP register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __CCAP_REG_H__
#define __CCAP_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup CCAP Camera Capture Interface Controller (CCAP)
    Memory Mapped Structure for CCAP Controller
    @{
*/

typedef struct
{
    /**
     * @var CCAP_T::CTL
     * Offset: 0x00  Camera Capture Interface Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CCAPEN    |Camera Capture Interface Enable Bit
     * |        |          |0 = Camera Capture Interface Disabled.
     * |        |          |1 = Camera Capture Interface Enabled.
     * |[6]     |PKTEN     |Packet Output Enable Bit
     * |        |          |0 = Packet output Disabled.
     * |        |          |1 = Packet output Enabled.
     * |[7]     |MONO      |Monochrome CMOS Sensor Select
     * |        |          |0 = Color CMOS Sensor.
     * |        |          |1 = Monochrome CMOS Sensor. The U/V components are ignored when the MONO is enabled.
     * |[16]    |SHUTTER   |Camera Capture Interface Automatically Disable the Capture Interface After a Frame Had Been Captured
     * |        |          |0 = Shutter Disabled.
     * |        |          |1 = Shutter Enabled.
     * |[17]    |MY4_SWAP  |Monochrome CMOS Sensor 4-bit Data Nibble Swap
     * |        |          |0 = The 4-bit data input sequence: 1st Pixel is for 1st Nibble (1st pixel at MSB).
     * |        |          |1 = The 4-bit data input sequence: 1st Pixel is for 2nd Nibble (1st pixel at LSB).
     * |[18]    |MY8_MY4   |Monochrome CMOS Sensor Data I/O Interface
     * |        |          |0 = Monochrome CMOS sensor is by the 4-bit data I/O interface.
     * |        |          |1 = Monochrome CMOS sensor is by the 8-bit data I/O interface.
     * |[19]    |Luma_Y_One|Color/Monochrome CMOS Sensor Luminance 8-bit Y to 1-bit Y Conversion
     * |        |          |0 = Color/Monochrome CMOS sensor Luma-Y-One bit Disabled.
     * |        |          |1 = Color/Monochrome CMOS sensor Luma-Y-One bit Enabled.
     * |        |          |Note: Color CMOS sensor U/V components are ignored when the Luma_Y_One is enabled.
     * |[20]    |UPDATE    |Update Register at New Frame
     * |        |          |0 = Update register at new frame Disabled.
     * |        |          |1 = Update register at new frame Enabled (auto cleared to 0 when register updated).
     * |[24]    |VPRST     |Capture Interface Reset
     * |        |          |0 = Capture interface reset Disabled.
     * |        |          |1 = Capture interface reset Enabled.
     * @var CCAP_T::PAR
     * Offset: 0x04  Camera Capture Interface Parameter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INFMT     |Sensor Input Data Format
     * |        |          |If INFMTH (CCAP_PARM[0]) = 0,.
     * |        |          |0 = YCbCr422.
     * |        |          |1 = RGB565.
     * |        |          |If INFMTH (CCAP_PARM[0]) = 1,.
     * |        |          |0 = YCbCr422.
     * |        |          |1 = RGB888.
     * |[1]     |SENTYPE   |Sensor Input Type
     * |        |          |0 = CCIR601.
     * |        |          |1 = CCIR656, Vsync & Hsync embedded in the data signal.
     * |[3:2]   |INDATORD  |Sensor Input Data Order
     * |        |          |If INFMT (CCAP_PAR[0]) = 0 and INFMTH (CCAP_PARM[0]) = 0 (YCbCr),.
     * |        |          |00 = Sensor input data (Byte 0 1 2 3) is Y0 U0 Y1 V0.
     * |        |          |01 = Sensor input data (Byte 0 1 2 3) is Y0 V0 Y1 U0.
     * |        |          |10 = Sensor input data (Byte 0 1 2 3) is U0 Y0 V0 Y1.
     * |        |          |11 = Sensor input data (Byte 0 1 2 3) is V0 Y0 U0 Y1.
     * |        |          |If INFMT (CCAP_PAR[0]) = 1 and INFMTH (CCAP_PARM[0])= 0 (RGB565),.
     * |        |          |00 = Sensor input data (Byte 0) is {R[4:0],G[5:3]}.
     * |        |          |Sensor input data (Byte 1) is {G[2:0],B[4:0]}.
     * |        |          |01 = Sensor input data (Byte 0) is {B[4:0],G[5:3]}.
     * |        |          |Sensor input data (Byte 1) is {G[2:0], R[4:0]}.
     * |        |          |10 = Sensor input data (Byte 0) is {G[2:0],B[4:0]}.
     * |        |          |Sensor input data (Byte 1) is {R[4:0], G[5:3]}.
     * |        |          |11 = Sensor input data (Byte 0) is {G[2:0],R[4:0]}.
     * |        |          |Sensor input data (Byte 1) is {B[4:0], G[5:3]}.
     * |        |          |If INFMT (CCAP_PAR[0]) = 1 and INFMTH (CCAP_PARM[0]) = 1 (RGB888),.
     * |        |          |If INDATORDH (CCAP_PARM[8]) = 0.
     * |        |          |00 = Sensor input data (Byte 0, Byte 1, Byte 2) is R[7:0], G[7:0], B[7:0].
     * |        |          |01 = Sensor input data (Byte 0, Byte 1, Byte 2) is R[7:0], B[7:0], G[7:0].
     * |        |          |10 = Sensor input data (Byte 0, Byte 1, Byte 2) is G[7:0], R[7:0], B[7:0].
     * |        |          |11 = Sensor input data (Byte 0, Byte 1, Byte 2) is G[7:0], B[7:0], R[7:0].
     * |        |          |If INDATORDH (CCAP_PARM[8]) = 1.
     * |        |          |00 = Sensor input data (Byte 0, Byte 1, Byte 2) is B[7:0], R[7:0], G[7:0].
     * |        |          |01 = Sensor input data (Byte 0, Byte 1, Byte 2) is B[7:0], G[7:0], R[7:0].
     * |        |          |10 = Reserved.
     * |        |          |11 = Reserved.
     * |[5:4]   |OUTFMT    |Image Data Format Output to System Memory
     * |        |          |If OUTFMTH (CCAP_PARM[16]) = 0,.
     * |        |          |00 = YCbCr422.
     * |        |          |01 = Only output Y. (Select this format when CCAP_CTL u201CLuma_Y_Oneu201D or u201CMONOu201D enabled).
     * |        |          |10 = RGB555.
     * |        |          |11 = RGB565.
     * |        |          |If OUTFMTH (CCAP_PARM[16]) = 1,.
     * |        |          |x0 = RGB888 or ARGB8888 with UINT8.
     * |        |          |x1 = RGB888 or ARGB8888 with INT8.
     * |[6]     |RANGE     |Scale Input YUV CCIR601 Color Range to Full Range
     * |        |          |0 = Default.
     * |        |          |1 = Scale to full range.
     * |[8]     |PCLKP     |Sensor Pixel Clock Polarity
     * |        |          |0 = Input video data and signals are latched by falling edge of Pixel Clock.
     * |        |          |1 = Input video data and signals are latched by rising edge of Pixel Clock.
     * |[9]     |HSP       |Sensor Hsync Polarity
     * |        |          |0 = Sync Low.
     * |        |          |1 = Sync High.
     * |[10]    |VSP       |Sensor Vsync Polarity
     * |        |          |0 = Sync Low.
     * |        |          |1 = Sync High.
     * |[18]    |FBB       |Field by Blank
     * |        |          |Field by Blank (only in ccir-656 mode) means blanking pixel data(0x80108010) have to transfer to system memory or not.
     * |        |          |0 = Field by blank Disabled. (blank pixel data will transfer to system memory).
     * |        |          |1 = Field by blank Enabled. (only active data will transfer to system memory).
     * @var CCAP_T::INTEN
     * Offset: 0x08  Camera Capture Interface Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VIEN      |Video Frame End Interrupt Enable Bit
     * |        |          |0 = Video frame end interrupt Disabled.
     * |        |          |1 = Video frame end interrupt Enabled.
     * |[1]     |MEIEN     |Bus Master Transfer Error Interrupt Enable Bit
     * |        |          |0 = Bus Master Transfer error interrupt Disabled.
     * |        |          |1 = Bus Master Transfer error interrupt Enabled.
     * |[3]     |ADDRMIEN  |Address Match Interrupt Enable Bit
     * |        |          |0 = Address match interrupt Disabled.
     * |        |          |1 = Address match interrupt Enabled.
     * |[5:4]   |MDIEN     |Motion Detection Modes Interrupt Enable Bit
     * |        |          |00 = Motion detection engine Disabled.
     * |        |          |01 = Only Motion detection Mode 1 interrupt Enabled.
     * |        |          |10 = Only Motion detection Mode 2 interrupt Enabled.
     * |        |          |11 = Both Motion detection Mode 1 and Mode 2 interrupt Enabled.
     * @var CCAP_T::INTSTS
     * Offset: 0x0C  Camera Capture Interface Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VINTF     |Video Frame End Interrupt
     * |        |          |0 = Did not receive a frame completely.
     * |        |          |1 = Received a frame completely.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |MEINTF    |Bus Master Transfer Error Interrupt
     * |        |          |0 = Transfer Error did not occur.
     * |        |          |1 = Transfer Error occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[3]     |ADDRMINTF |Memory Address Match Interrupt
     * |        |          |0 = Memory Address Match Interrupt did not occur.
     * |        |          |1 = Memory Address Match Interrupt occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[5:4]   |MDINTF    |Motion Detection Modes Interrupt
     * |        |          |00 = Motion detection Mode 1 and Mode 2 interrupt did not occur.
     * |        |          |01 = Only Motion detection Mode 1 interrupt occured.
     * |        |          |10 = Only Motion detection Mode 2 interrupt occured.
     * |        |          |11 = Both Motion detection Mode 1 and Mode 2 interrupt occured.
     * |        |          |Note: These two bits are cleared by writing 1 to them.
     * |        |          |Note: When Motion Detection Total SAD (Sum of Absolute Differences) Register (CCAP_MDTSAD) is larger than Motion Detection Total Threshold Register (CCAP_MDTTH), CCAP will trigger Motion detection Mode 1 interrupt.
     * |        |          |Note: When Motion Detection Window Overflow Counter Register (CCAP_MDWOC) is larger than Motion Detection Window Overflow Counter Threshold Register (CCAP_MDWOCTH), CCAP will trigger Motion detection Mode 2 interrupt.
     * @var CCAP_T::PARM
     * Offset: 0x10  Camera Capture Interface Parameter Register (MSB)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INFMTH    |Sensor Input Data Format
     * |        |          |Specify the higher 1-bit of of INFMT (CCAP_PAR[0]).
     * |        |          |Please refer to the registeru201CCCAP_PARu201D to check the cooperation between these two registers.
     * |[8]     |INDATORDH |Sensor Input Data Order
     * |        |          |Specify the higher 1-bit of of INDATORD (CCAP_PAR[3:2]).
     * |        |          |Please refer to the registeru201CCCAP_PARu201D to check the cooperation between these two registers.
     * |[16]    |OUTFMTH   |Image Data Format Output to System Memory
     * |        |          |Specify the higher 1-bit of of OUTFMT(CCAP_PAR[5:4]).
     * |        |          |Please refer to the registeru201CCCAP_PARu201D to check the cooperation between these two registers.
     * |[25:24] |RGB888OUTORD|Sensor RGB888 Output Data Order
     * |        |          |If OUTFMTH (CCAP_PARM[16]) = 1,.
     * |        |          |00 = Sensor output data (Byte 0 1 2) is B, G, R loop for RGB888.
     * |        |          |01 = Sensor output data (Byte 0 1 2) is R, G, B loop for RGB888.
     * |        |          |10 = Sensor output data (Byte 0 1 2 3) is B, G, R, 0xFF loop for ARGB8888.
     * |        |          |11 = Sensor output data (Byte 0 1 2 3) is 0xFF, R, G, B loop for ARGB8888.
     * @var CCAP_T::CWSP
     * Offset: 0x20  Cropping Window Starting Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |CWSADDRH  |Cropping Window Horizontal Starting Address
     * |        |          |Specify the value of the cropping window horizontal start address.
     * |[26:16] |CWSADDRV  |Cropping Window Vertical Starting Address
     * |        |          |Specify the value of the cropping window vertical start address.
     * @var CCAP_T::CWS
     * Offset: 0x24  Cropping Window Size Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |CWW       |Cropping Window Width
     * |        |          |Specify the size of the cropping window width.
     * |[26:16] |CWH       |Cropping Window Height
     * |        |          |Specify the size of the cropping window height.
     * @var CCAP_T::PKTSL
     * Offset: 0x28  Packet Scaling Vertical/Horizontal Factor Register (LSB)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PKTSHML   |Packet Scaling Horizontal Factor M
     * |        |          |Specify the lower 8-bit of denominator part (M) of the horizontal scaling factor.
     * |        |          |The lower 8-bit will be cascaded with higher 8-bit (PKDSHMH) to form a 16-bit denominator (M) of vertical factor.
     * |        |          |The output image width will be equal to the image width * N/M.
     * |        |          |Note: The value of N must be equal to or less than M.
     * |[15:8]  |PKTSHNL   |Packet Scaling Horizontal Factor N
     * |        |          |Specify the lower 8-bit of numerator part (N) of the horizontal scaling factor.
     * |        |          |The lower 8-bit will be cascaded with higher 8-bit (PKDSHNH) to form a 16-bit numerator of horizontal factor.
     * |[23:16] |PKTSVML   |Packet Scaling Vertical Factor M
     * |        |          |Specify the lower 8-bit of denominator part (M) of the vertical scaling factor.
     * |        |          |The lower 8-bit will be cascaded with higher 8-bit (PKDSVMH) to form a 16-bit denominator (M) of vertical factor.
     * |        |          |The output image width will be equal to the image height * N/M.
     * |        |          |Note: The value of N must be equal to or less than M.
     * |[31:24] |PKTSVNL   |Packet Scaling Vertical Factor N
     * |        |          |Specify the lower 8-bit of numerator part (N) of the vertical scaling factor.
     * |        |          |The lower 8-bit will be cascaded with higher 8-bit (PKDSVNH) to form a 16-bit numerator of vertical factor.
     * @var CCAP_T::FRCTL
     * Offset: 0x30  Scaling Frame Rate Factor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |FRM       |Scaling Frame Rate Factor M
     * |        |          |Specify the denominator part (M) of the frame rate scaling factor.
     * |        |          |The output image frame rate will be equal to input image frame rate * (N/M).
     * |        |          |Note: The value of N must be equal to or less than M.
     * |[13:8]  |FRN       |Scaling Frame Rate Factor N
     * |        |          |Specify the numerator part (N) of the frame rate scaling factor.
     * @var CCAP_T::STRIDE
     * Offset: 0x34  Frame Output Pixel Stride Width Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[13:0]  |PKTSTRIDE |Packet Frame Output Pixel Stride Width
     * |        |          |The output pixel stride size of the packet pipe.
     * |        |          |It is a 32-pixel aligned stride width for the Luma-Y-One bit format or a 4-pixel aligned stride with for the Luma-Y-Eight bit format when color or monochrome CMOS sensors used
     * |        |          |This means that every new captured line is by word alignment address when color or monochrome CMOS sensors used.
     * @var CCAP_T::FIFOTH
     * Offset: 0x3C  FIFO Threshold Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[28:24] |PKTFTH    |Packet FIFO Threshold
     * |        |          |Specify the 5-bit value of the packet FIFO threshold.
     * |[31]    |OVF       |FIFO Overflow Flag
     * |        |          |Indicate the FIFO overflow flag.
     * @var CCAP_T::CMPADDR
     * Offset: 0x40  Compare Memory Base Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CMPADDR   |Compare Memory Base Address
     * |        |          |It is a word alignment address, that is, the address is aligned by ignoring the 2 LSB bits [1:0].
     * @var CCAP_T::LUMA_Y1_THD
     * Offset: 0x44  Luminance Y8 to Y1 Threshold Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |LUMA_Y1_THRESH|Luminance Y8 to Y1 Threshold Value
     * |        |          |Specify the 8-bit threshold value for the luminance Y bit-8 to the luminance Y 1-bit conversion.
     * @var CCAP_T::PKTSM
     * Offset: 0x48  Packet Scaling Vertical/Horizontal Factor Register (MSB)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PKTSHMH   |Packet Scaling Horizontal Factor M
     * |        |          |Specify the higher 8-bit of denominator part (M) of the horizontal scaling factor.
     * |        |          |Please refer to the register u201CCCAP_PKTSLu201D for the detailed operation.
     * |[15:8]  |PKTSHNH   |Packet Scaling Horizontal Factor N
     * |        |          |Specify the higher 8-bit of numerator part (N) of the horizontal scaling factor.
     * |        |          |Please refer to the register u201CCCAP_PKTSLu201D for the detailed operation.
     * |[23:16] |PKTSVMH   |Packet Scaling Vertical Factor M
     * |        |          |Specify the higher 8-bit of denominator part (M) of the vertical scaling factor.
     * |        |          |Please refer to the register u201CCCAP_PKTSLu201D to check the cooperation between these two registers.
     * |[31:24] |PKTSVNH   |Packet Scaling Vertical Factor N
     * |        |          |Specify the higher 8-bit of numerator part (N) of the vertical scaling factor.
     * |        |          |Please refer to the register u201CCCAP_PKTSLu201D to check the cooperation between these two registers
     * @var CCAP_T::CURADDRP
     * Offset: 0x50  Current Packet System Memory Address Byte Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CURADDR   |Current Packet Output Memory Address Byte Counter
     * |        |          |Specify the 32-bit value of the current packet output memory address counter
     * |        |          |Thus, the current packet output memory address = CCAP_PKTBA0 + CCAP_CURADDRP.
     * @var CCAP_T::PKTBA0
     * Offset: 0x60  System Memory Packet Base Address 0 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |BASEADDR  |System Memory Packet Base Address 0
     * |        |          |It is a word alignment address, that is, the address is aligned by ignoring the 2 LSB bits [1:0].
     * @var CCAP_T::MDCTL
     * Offset: 0x100  Motion Detection Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WEN0      |Motion Detection Window 0 Enable Bit
     * |        |          |00 = Motion Detection Window 0 Disabled.
     * |        |          |01 = Motion Detection Window 0 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[1]     |WEN1      |Motion Detection Window 1 Enable Bit
     * |        |          |00 = Motion Detection Window 1 Disabled.
     * |        |          |01 = Motion Detection Window 1 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[2]     |WEN2      |Motion Detection Window 2 Enable Bit
     * |        |          |00 = Motion Detection Window 2 Disabled.
     * |        |          |01 = Motion Detection Window 2 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[3]     |WEN3      |Motion Detection Window 3 Enable Bit
     * |        |          |00 = Motion Detection Window 3 Disabled.
     * |        |          |01 = Motion Detection Window 3 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[4]     |WEN4      |Motion Detection Window 4 Enable Bit
     * |        |          |00 = Motion Detection Window 4 Disabled.
     * |        |          |01 = Motion Detection Window 4 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[5]     |WEN5      |Motion Detection Window 5 Enable Bit
     * |        |          |00 = Motion Detection Window 5 Disabled.
     * |        |          |01 = Motion Detection Window 5 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[6]     |WEN6      |Motion Detection Window 6 Enable Bit
     * |        |          |00 = Motion Detection Window 6 Disabled.
     * |        |          |01 = Motion Detection Window 6 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[7]     |WEN7      |Motion Detection Window 7 Enable Bit
     * |        |          |00 = Motion Detection Window 7 Disabled.
     * |        |          |01 = Motion Detection Window 7 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[8]     |WEN8      |Motion Detection Window 8 Enable Bit
     * |        |          |00 = Motion Detection Window 8 Disabled.
     * |        |          |01 = Motion Detection Window 8 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0,.
     * |[9]     |WEN9      |Motion Detection Window 9 Enable Bit
     * |        |          |00 = Motion Detection Window 9 Disabled.
     * |        |          |01 = Motion Detection Window 9 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[10]    |WEN10     |Motion Detection Window 10 Enable Bit
     * |        |          |00 = Motion Detection Window 10 Disabled.
     * |        |          |01 = Motion Detection Window 10 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[11]    |WEN11     |Motion Detection Window 11 Enable Bit
     * |        |          |00 = Motion Detection Window 11 Disabled.
     * |        |          |01 = Motion Detection Window 11 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[12]    |WEN12     |Motion Detection Window 12 Enable Bit
     * |        |          |00 = Motion Detection Window 12 Disabled.
     * |        |          |01 = Motion Detection Window 12 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[13]    |WEN13     |Motion Detection Window 13 Enable Bit
     * |        |          |00 = Motion Detection Window 13 Disabled.
     * |        |          |01 = Motion Detection Window 13 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[14]    |WEN14     |Motion Detection Window 14 Enable Bit
     * |        |          |00 = Motion Detection Window 14 Disabled.
     * |        |          |01 = Motion Detection Window 14 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * |[15]    |WEN15     |Motion Detection Window 15 Enable Bit
     * |        |          |00 = Motion Detection Window 15 Disabled.
     * |        |          |01 = Motion Detection Window 15 Enabled only when CCAP_INTEN[5:4] (MDIEN) is not equal to 0x0.
     * @var CCAP_T::MDTRG_WK
     * Offset: 0x104  Motion Detection Trigger and Wakeup Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |TRGSEL    |Trigger Source Selection
     * |        |          |00 = LPTMR0.
     * |        |          |01 = LPTMR1.
     * |        |          |10 = TTMR0.
     * |        |          |11 = TTMR1.
     * |[16]    |WKEN      |Wake-up Function Enable Bit
     * |        |          |If this bit is set to 1, while Motion Detection Modes interrupt MDINTF (CCAP_INTSTS[5:4]) is 1 and MDIEN (CCAP_INTEN[5:4]) is enabled, the CCAP interrupt signal will generate a wake-up trigger event to CPU.
     * |        |          |0 = Wake-up function Disabled if CCAP interrupt signal generated.
     * |        |          |1 = Wake-up function Enabled if CCAP interrupt signal generated.
     * |[24]    |WKF       |CCAP Wake-up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of CCAP.
     * |        |          |0 = CCAP does not cause CPU wake-up.
     * |        |          |1 = CPU wake-up from Idle or Power-down mode if MDINTF (CCAP_INTSTS[5:4]) are generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var CCAP_T::MDTTH
     * Offset: 0x110  Motion Detection Total Threshold Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[20:0]  |MDTTH     |Motion Detection Total Threshold
     * |        |          |Specify the 21-bit threshold value for checking if the total SAD of all windows (CCAP_MDTSAD) is overflow
     * |        |          |Note: The maximum value of MDTSAD is 0x12AD40.
     * @var CCAP_T::MDTSAD
     * Offset: 0x114  Motion Detection Total SAD Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[20:0]  |MDTSAD    |Motion Detection Total SAD
     * |        |          |The total SAD (Sum of Absolute Differences) of all windows (i.e., CCAP_MDTSAD = CCAP_MDWSAD0 + CCAP_MDWSAD1 + CCAP_MDWSAD2 + CCAP_MDWSAD3 + CCAP_MDWSAD4 + CCAP_MDWSAD5 + CCAP_MDWSAD6 + CCAP_MDWSAD7 + CCAP_MDWSAD8 + CCAP_MDWSAD9 + CCAP_MDWSAD10 + CCAP_MDWSAD11 + CCAP_MDWSAD12 + CCAP_MDWSAD13 + CCAP_MDWSAD14 + CCAP_MDWSAD15).
     * |        |          |Note: The maximum value of MDTSAD is 0x12AD40.
     * @var CCAP_T::MDWOCTH
     * Offset: 0x120  Motion Detection Window Overflow Counter Threshold Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |MDWOCTH   |Motion Detection Window Overflow Counter threshold
     * |        |          |Specify the 4-bit threshold value for checking if CCAP_MDWOC is overflow.
     * @var CCAP_T::MDWOC
     * Offset: 0x124  Motion Detection Window Overflow Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MDWOC     |Motion Detection Window Overflow Counter
     * |        |          |The counter of windows that their CCAP_MDWSADx are larger than their CCAP_MDWTHx.
     * @var CCAP_T::MDWTH[16]
     * Offset: 0x140 ~ 0x17C  Motion Detection Window Threshold 0 ~ 15 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16:0]  |MDWTH     |Motion Detection Window Threshold
     * |        |          |Specify the 17-bit threshold value for checking if the SAD of window x (CCAP_MDWSADx) is overflow.
     * |        |          |Note: The maximum value of MDWSAD is 0x12AD4.
     * @var CCAP_T::MDWSAD[16]
     * Offset: 0x180 ~ 0x1BC  Motion Detection Window SAD 0 ~ 15 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16:0]  |MDWSAD    |Motion Detection Window SAD
     * |        |          |The 17-bit value for the SAD (Sum of Absolute Differences) of window x.
     * |        |          |Note: The maximum value of MDWSAD is 0x12AD4.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] Camera Capture Interface Control Register                        */
    __IO uint32_t PAR;                   /*!< [0x0004] Camera Capture Interface Parameter Register                      */
    __IO uint32_t INTEN;                 /*!< [0x0008] Camera Capture Interface Interrupt Enable Register               */
    __IO uint32_t INTSTS;                /*!< [0x000c] Camera Capture Interface Interrupt Status Register               */
    __IO uint32_t PARM;                  /*!< [0x0010] Camera Capture Interface Parameter Register (MSB)                */
    __I  uint32_t RESERVE0[3];
    __IO uint32_t CWSP;                  /*!< [0x0020] Cropping Window Starting Address Register                        */
    __IO uint32_t CWS;                   /*!< [0x0024] Cropping Window Size Register                                    */
    __IO uint32_t PKTSL;                 /*!< [0x0028] Packet Scaling Vertical/Horizontal Factor Register (LSB)         */
    __IO uint32_t PLNSL;                 /*!< [0x002C] Planar Scaling Vertical/Horizontal Factor Register (LSB)         */
    __IO uint32_t FRCTL;                 /*!< [0x0030] Scaling Frame Rate Factor Register                               */
    __IO uint32_t STRIDE;                /*!< [0x0034] Frame Output Pixel Stride Width Register                         */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t FIFOTH;                /*!< [0x003c] FIFO Threshold Register                                          */
    __IO uint32_t CMPADDR;               /*!< [0x0040] Compare Memory Base Address Register                             */
    __IO uint32_t LUMA_Y1_THD;           /*!< [0x0044] Luminance Y8 to Y1 Threshold Value Register                      */
    __IO uint32_t PKTSM;                 /*!< [0x0048] Packet Scaling Vertical/Horizontal Factor Register (MSB)         */
    __IO uint32_t PLNSM;                 /*!< [0x004C] Planar Scaling Vertical/Horizontal Factor Register (MSB)         */
    __I  uint32_t CURADDRP;              /*!< [0x0050] Current Packet System Memory Address Byte Counter Register       */
    __I  uint32_t CURADDRY;              /*!< [0x0054] Current Planar Y System Memory Address Register                  */
    __I  uint32_t CURADDRU;              /*!< [0x0058] Current Planar U System Memory Address Register                  */
    __I  uint32_t CURADDRV;              /*!< [0x005C] Current Planar V System Memory Address Register                  */
    __IO uint32_t PKTBA0;                /*!< [0x0060] System Memory Packet Base Address 0 Register                     */
    __I  uint32_t RESERVE2[7];
    __IO uint32_t YBA;                   /*!< [0x0080] System Memory Packet Base Address 0 Register                     */
    __IO uint32_t UBA;                   /*!< [0x0084] System Memory Packet Base Address 0 Register                     */
    __IO uint32_t VBA;                   /*!< [0x0088] System Memory Packet Base Address 0 Register                     */
    __I  uint32_t RESERVE3[29];
    __IO uint32_t MDCTL;                 /*!< [0x0100] Motion Detection Control Register                                */
    __IO uint32_t MDTRG_WK;              /*!< [0x0104] Motion Detection Trigger and Wakeup Register                     */
    __I  uint32_t RESERVE4[2];
    __IO uint32_t MDTTH;                 /*!< [0x0110] Motion Detection Total Threshold Register                        */
    __I  uint32_t MDTSAD;                /*!< [0x0114] Motion Detection Total SAD Register                              */
    __I  uint32_t RESERVE5[2];
    __IO uint32_t MDWOCTH;               /*!< [0x0120] Motion Detection Window Overflow Counter Threshold Register      */
    __I  uint32_t MDWOC;                 /*!< [0x0124] Motion Detection Window Overflow Counter Register                */
    __I  uint32_t RESERVE6[6];
    __IO uint32_t MDWTH[16];             /*!< [0x0140] ~ [0x017c] Motion Detection Window Threshold 0 ~ 15 Register      */
    __I  uint32_t MDWSAD[16];            /*!< [0x0180] ~ [0x01bc] Motion Detection Window SAD 0 ~ 15 Register            */
} CCAP_T;
/**
    @addtogroup CCAP_CONST CCAP Bit Field Definition
    Constant Definitions for CCAP Controller
@{ */

#define CCAP_CTL_CCAPEN_Pos              (0)                                               /*!< CCAP_T::CTL: CCAPEN Position           */
#define CCAP_CTL_CCAPEN_Msk              (0x1ul << CCAP_CTL_CCAPEN_Pos)                    /*!< CCAP_T::CTL: CCAPEN Mask               */

#define CCAP_CTL_PLNEN_Pos               (5)                                               /*!< CCAP_T::CTL: PLNEN Position            */
#define CCAP_CTL_PLNEN_Msk               (0x1ul << CCAP_CTL_PLNEN_Pos)                     /*!< CCAP_T::CTL: PLNEN Mask                */

#define CCAP_CTL_PKTEN_Pos               (6)                                               /*!< CCAP_T::CTL: PKTEN Position            */
#define CCAP_CTL_PKTEN_Msk               (0x1ul << CCAP_CTL_PKTEN_Pos)                     /*!< CCAP_T::CTL: PKTEN Mask                */

#define CCAP_CTL_MONO_Pos                (7)                                               /*!< CCAP_T::CTL: MONO Position             */
#define CCAP_CTL_MONO_Msk                (0x1ul << CCAP_CTL_MONO_Pos)                      /*!< CCAP_T::CTL: MONO Mask                 */

#define CCAP_CTL_SHUTTER_Pos             (16)                                              /*!< CCAP_T::CTL: SHUTTER Position          */
#define CCAP_CTL_SHUTTER_Msk             (0x1ul << CCAP_CTL_SHUTTER_Pos)                   /*!< CCAP_T::CTL: SHUTTER Mask              */

#define CCAP_CTL_MY4_SWAP_Pos            (17)                                              /*!< CCAP_T::CTL: MY4_SWAP Position         */
#define CCAP_CTL_MY4_SWAP_Msk            (0x1ul << CCAP_CTL_MY4_SWAP_Pos)                  /*!< CCAP_T::CTL: MY4_SWAP Mask             */

#define CCAP_CTL_MY8_MY4_Pos             (18)                                              /*!< CCAP_T::CTL: MY8_MY4 Position          */
#define CCAP_CTL_MY8_MY4_Msk             (0x1ul << CCAP_CTL_MY8_MY4_Pos)                   /*!< CCAP_T::CTL: MY8_MY4 Mask              */

#define CCAP_CTL_Luma_Y_One_Pos          (19)                                              /*!< CCAP_T::CTL: Luma_Y_One Position       */
#define CCAP_CTL_Luma_Y_One_Msk          (0x1ul << CCAP_CTL_Luma_Y_One_Pos)                /*!< CCAP_T::CTL: Luma_Y_One Mask           */

#define CCAP_CTL_UPDATE_Pos              (20)                                              /*!< CCAP_T::CTL: UPDATE Position           */
#define CCAP_CTL_UPDATE_Msk              (0x1ul << CCAP_CTL_UPDATE_Pos)                    /*!< CCAP_T::CTL: UPDATE Mask               */

#define CCAP_CTL_VPRST_Pos               (24)                                              /*!< CCAP_T::CTL: VPRST Position            */
#define CCAP_CTL_VPRST_Msk               (0x1ul << CCAP_CTL_VPRST_Pos)                     /*!< CCAP_T::CTL: VPRST Mask                */

#define CCAP_PAR_INFMT_Pos               (0)                                               /*!< CCAP_T::PAR: INFMT Position            */
#define CCAP_PAR_INFMT_Msk               (0x1ul << CCAP_PAR_INFMT_Pos)                     /*!< CCAP_T::PAR: INFMT Mask                */

#define CCAP_PAR_SENTYPE_Pos             (1)                                               /*!< CCAP_T::PAR: SENTYPE Position          */
#define CCAP_PAR_SENTYPE_Msk             (0x1ul << CCAP_PAR_SENTYPE_Pos)                   /*!< CCAP_T::PAR: SENTYPE Mask              */

#define CCAP_PAR_INDATORD_Pos            (2)                                               /*!< CCAP_T::PAR: INDATORD Position         */
#define CCAP_PAR_INDATORD_Msk            (0x3ul << CCAP_PAR_INDATORD_Pos)                  /*!< CCAP_T::PAR: INDATORD Mask             */

#define CCAP_PAR_OUTFMT_Pos              (4)                                               /*!< CCAP_T::PAR: OUTFMT Position           */
#define CCAP_PAR_OUTFMT_Msk              (0x3ul << CCAP_PAR_OUTFMT_Pos)                    /*!< CCAP_T::PAR: OUTFMT Mask               */

#define CCAP_PAR_RANGE_Pos               (6)                                               /*!< CCAP_T::PAR: RANGE Position            */
#define CCAP_PAR_RANGE_Msk               (0x1ul << CCAP_PAR_RANGE_Pos)                     /*!< CCAP_T::PAR: RANGE Mask                */

#define CCAP_PAR_PLNFMT_Pos              (7)                                               /*!< CCAP_T::PAR: PLNFMT Position           */
#define CCAP_PAR_PLNFMT_Msk              (0x1ul << CCAP_PAR_PLNFMT_Pos)                    /*!< CCAP_T::PAR: PLNFMT Mask               */

#define CCAP_PAR_PCLKP_Pos               (8)                                               /*!< CCAP_T::PAR: PCLKP Position            */
#define CCAP_PAR_PCLKP_Msk               (0x1ul << CCAP_PAR_PCLKP_Pos)                     /*!< CCAP_T::PAR: PCLKP Mask                */

#define CCAP_PAR_HSP_Pos                 (9)                                               /*!< CCAP_T::PAR: HSP Position              */
#define CCAP_PAR_HSP_Msk                 (0x1ul << CCAP_PAR_HSP_Pos)                       /*!< CCAP_T::PAR: HSP Mask                  */

#define CCAP_PAR_VSP_Pos                 (10)                                              /*!< CCAP_T::PAR: VSP Position              */
#define CCAP_PAR_VSP_Msk                 (0x1ul << CCAP_PAR_VSP_Pos)                       /*!< CCAP_T::PAR: VSP Mask                  */

#define CCAP_PAR_FBB_Pos                 (18)                                              /*!< CCAP_T::PAR: FBB Position              */
#define CCAP_PAR_FBB_Msk                 (0x1ul << CCAP_PAR_FBB_Pos)                       /*!< CCAP_T::PAR: FBB Mask                  */

#define CCAP_INTSTS_VINTF_Pos            (0)                                               /*!< CCAP_T::INTSTS: VINTF Position         */
#define CCAP_INTSTS_VINTF_Msk            (0x1ul << CCAP_INTSTS_VINTF_Pos)                  /*!< CCAP_T::INTSTS: VINTF Mask             */

#define CCAP_INTSTS_MEINTF_Pos           (1)                                               /*!< CCAP_T::INTSTS: MEINTF Position        */
#define CCAP_INTSTS_MEINTF_Msk           (0x1ul << CCAP_INTSTS_MEINTF_Pos)                 /*!< CCAP_T::INTSTS: MEINTF Mask            */

#define CCAP_INTSTS_ADDRMINTF_Pos        (3)                                               /*!< CCAP_T::INTSTS: ADDRMINTF Position     */
#define CCAP_INTSTS_ADDRMINTF_Msk        (0x1ul << CCAP_INTSTS_ADDRMINTF_Pos)              /*!< CCAP_T::INTSTS: ADDRMINTF Mask         */

#define CCAP_INTSTS_MDINTF_Pos           (4)                                               /*!< CCAP_T::INTSTS: MDINTF Position        */
#define CCAP_INTSTS_MDINTF_Msk           (0x3ul << CCAP_INTSTS_MDINTF_Pos)                 /*!< CCAP_T::INTSTS: MDINTF Mask            */

#define CCAP_INTEN_VIEN_Pos              (0)                                               /*!< CCAP_T::INTEN: VIEN Position           */
#define CCAP_INTEN_VIEN_Msk              (0x1ul << CCAP_INTEN_VIEN_Pos)                    /*!< CCAP_T::INTEN: VIEN Mask               */

#define CCAP_INTEN_MEIEN_Pos             (1)                                               /*!< CCAP_T::INTEN: MEIEN Position          */
#define CCAP_INTEN_MEIEN_Msk             (0x1ul << CCAP_INTEN_MEIEN_Pos)                   /*!< CCAP_T::INTEN: MEIEN Mask              */

#define CCAP_INTEN_ADDRMIEN_Pos          (3)                                               /*!< CCAP_T::INTEN: ADDRMIEN Position       */
#define CCAP_INTEN_ADDRMIEN_Msk          (0x1ul << CCAP_INTEN_ADDRMIEN_Pos)                /*!< CCAP_T::INTEN: ADDRMIEN Mask           */

#define CCAP_INTEN_MDIEN_Pos             (4)                                               /*!< CCAP_T::INTEN: MDIEN Position          */
#define CCAP_INTEN_MDIEN_Msk             (0x3ul << CCAP_INTEN_MDIEN_Pos)                   /*!< CCAP_T::INTEN: MDIEN Mask              */

#define CCAP_PARM_INFMTH_Pos             (0)                                               /*!< CCAP_T::PARM: INFMTH Position          */
#define CCAP_PARM_INFMTH_Msk             (0x1ul << CCAP_PARM_INFMTH_Pos)                   /*!< CCAP_T::PARM: INFMTH Mask              */

#define CCAP_PARM_INDATORDH_Pos          (8)                                               /*!< CCAP_T::PARM: INDATORDH Position       */
#define CCAP_PARM_INDATORDH_Msk          (0x1ul << CCAP_PARM_INDATORDH_Pos)                /*!< CCAP_T::PARM: INDATORDH Mask           */

#define CCAP_PARM_OUTFMTH_Pos            (16)                                              /*!< CCAP_T::PARM: OUTFMTH Position         */
#define CCAP_PARM_OUTFMTH_Msk            (0x1ul << CCAP_PARM_OUTFMTH_Pos)                  /*!< CCAP_T::PARM: OUTFMTH Mask             */

#define CCAP_PARM_RGB888OUTORD_Pos       (24)                                              /*!< CCAP_T::PARM: RGB888OUTORD Position    */
#define CCAP_PARM_RGB888OUTORD_Msk       (0x3ul << CCAP_PARM_RGB888OUTORD_Pos)             /*!< CCAP_T::PARM: RGB888OUTORD Mask        */

#define CCAP_CWSP_CWSADDRH_Pos           (0)                                               /*!< CCAP_T::CWSP: CWSADDRH Position        */
#define CCAP_CWSP_CWSADDRH_Msk           (0xffful << CCAP_CWSP_CWSADDRH_Pos)               /*!< CCAP_T::CWSP: CWSADDRH Mask            */

#define CCAP_CWSP_CWSADDRV_Pos           (16)                                              /*!< CCAP_T::CWSP: CWSADDRV Position        */
#define CCAP_CWSP_CWSADDRV_Msk           (0x7fful << CCAP_CWSP_CWSADDRV_Pos)               /*!< CCAP_T::CWSP: CWSADDRV Mask            */

#define CCAP_CWS_CWW_Pos                 (0)                                               /*!< CCAP_T::CWS: CWW Position              */
#define CCAP_CWS_CWW_Msk                 (0xffful << CCAP_CWS_CWW_Pos)                     /*!< CCAP_T::CWS: CWW Mask                  */

#define CCAP_CWS_CWH_Pos                 (16)                                              /*!< CCAP_T::CWS: CWH Position              */
#define CCAP_CWS_CWH_Msk                 (0x7fful << CCAP_CWS_CWH_Pos)                     /*!< CCAP_T::CWS: CWH Mask                  */

#define CCAP_PKTSL_PKTSHML_Pos           (0)                                               /*!< CCAP_T::PKTSL: PKTSHML Position        */
#define CCAP_PKTSL_PKTSHML_Msk           (0xfful << CCAP_PKTSL_PKTSHML_Pos)                /*!< CCAP_T::PKTSL: PKTSHML Mask            */

#define CCAP_PKTSL_PKTSHNL_Pos           (8)                                               /*!< CCAP_T::PKTSL: PKTSHNL Position        */
#define CCAP_PKTSL_PKTSHNL_Msk           (0xfful << CCAP_PKTSL_PKTSHNL_Pos)                /*!< CCAP_T::PKTSL: PKTSHNL Mask            */

#define CCAP_PKTSL_PKTSVML_Pos           (16)                                              /*!< CCAP_T::PKTSL: PKTSVML Position        */
#define CCAP_PKTSL_PKTSVML_Msk           (0xfful << CCAP_PKTSL_PKTSVML_Pos)                /*!< CCAP_T::PKTSL: PKTSVML Mask            */

#define CCAP_PKTSL_PKTSVNL_Pos           (24)                                              /*!< CCAP_T::PKTSL: PKTSVNL Position        */
#define CCAP_PKTSL_PKTSVNL_Msk           (0xfful << CCAP_PKTSL_PKTSVNL_Pos)                /*!< CCAP_T::PKTSL: PKTSVNL Mask            */

#define CCAP_PLNSL_PLNSHML_Pos           (0)                                               /*!< CCAP_T::PLNSL: PLNSHML Position        */
#define CCAP_PLNSL_PLNSHML_Msk           (0xfful << CCAP_PLNSL_PLNSHML_Pos)                /*!< CCAP_T::PLNSL: PLNSHML Mask            */

#define CCAP_PLNSL_PLNSHNL_Pos           (8)                                               /*!< CCAP_T::PLNSL: PLNSHNL Position        */
#define CCAP_PLNSL_PLNSHNL_Msk           (0xfful << CCAP_PLNSL_PLNSHNL_Pos)                /*!< CCAP_T::PLNSL: PLNSHNL Mask            */

#define CCAP_PLNSL_PLNSVML_Pos           (16)                                              /*!< CCAP_T::PLNSL: PLNSVML Position        */
#define CCAP_PLNSL_PLNSVML_Msk           (0xfful << CCAP_PLNSL_PLNSVML_Pos)                /*!< CCAP_T::PLNSL: PLNSVML Mask            */

#define CCAP_PLNSL_PLNSVNL_Pos           (24)                                              /*!< CCAP_T::PLNSL: PLNSVNL Position        */
#define CCAP_PLNSL_PLNSVNL_Msk           (0xfful << CCAP_PLNSL_PLNSVNL_Pos)                /*!< CCAP_T::PLNSL: PLNSVNL Mask            */

#define CCAP_FRCTL_FRM_Pos               (0)                                               /*!< CCAP_T::FRCTL: FRM Position            */
#define CCAP_FRCTL_FRM_Msk               (0x3ful << CCAP_FRCTL_FRM_Pos)                    /*!< CCAP_T::FRCTL: FRM Mask                */

#define CCAP_FRCTL_FRN_Pos               (8)                                               /*!< CCAP_T::FRCTL: FRN Position            */
#define CCAP_FRCTL_FRN_Msk               (0x3ful << CCAP_FRCTL_FRN_Pos)                    /*!< CCAP_T::FRCTL: FRN Mask                */

#define CCAP_STRIDE_PKTSTRIDE_Pos        (0)                                               /*!< CCAP_T::STRIDE: PKTSTRIDE Position     */
#define CCAP_STRIDE_PKTSTRIDE_Msk        (0x3ffful << CCAP_STRIDE_PKTSTRIDE_Pos)           /*!< CCAP_T::STRIDE: PKTSTRIDE Mask         */

#define CCAP_STRIDE_PLNSTRIDE_Pos        (16)                                              /*!< CCAP_T::STRIDE: PLNSTRIDE Position     */
#define CCAP_STRIDE_PLNSTRIDE_Msk        (0x3ffful << CCAP_STRIDE_PLNSTRIDE_Pos)           /*!< CCAP_T::STRIDE: PLNSTRIDE Mask         */

#define CCAP_FIFOTH_PLNVFTH_Pos          (0)                                               /*!< CCAP_T::FIFOTH: PLNVFTH Position       */
#define CCAP_FIFOTH_PLNVFTH_Msk          (0xful << CCAP_FIFOTH_PLNVFTH_Pos)                /*!< CCAP_T::FIFOTH: PLNVFTH Mask           */

#define CCAP_FIFOTH_PLNUFTH_Pos          (8)                                               /*!< CCAP_T::FIFOTH: PLNUFTH Position       */
#define CCAP_FIFOTH_PLNUFTH_Msk          (0xful << CCAP_FIFOTH_PLNUFTH_Pos)                /*!< CCAP_T::FIFOTH: PLNUFTH Mask           */

#define CCAP_FIFOTH_PLNYFTH_Pos          (16)                                              /*!< CCAP_T::FIFOTH: PLNYFTH Position       */
#define CCAP_FIFOTH_PLNYFTH_Msk          (0x1ful << CCAP_FIFOTH_PLNYFTH_Pos)               /*!< CCAP_T::FIFOTH: PLNYFTH Mask           */

#define CCAP_FIFOTH_PKTFTH_Pos           (24)                                              /*!< CCAP_T::FIFOTH: PKTFTH Position        */
#define CCAP_FIFOTH_PKTFTH_Msk           (0x1ful << CCAP_FIFOTH_PKTFTH_Pos)                /*!< CCAP_T::FIFOTH: PKTFTH Mask            */

#define CCAP_FIFOTH_OVF_Pos              (31)                                              /*!< CCAP_T::FIFOTH: OVF Position           */
#define CCAP_FIFOTH_OVF_Msk              (0x1ul << CCAP_FIFOTH_OVF_Pos)                    /*!< CCAP_T::FIFOTH: OVF Mask               */

#define CCAP_CMPADDR_CMPADDR_Pos         (0)                                               /*!< CCAP_T::CMPADDR: CMPADDR Position      */
#define CCAP_CMPADDR_CMPADDR_Msk         (0xfffffffful << CCAP_CMPADDR_CMPADDR_Pos)        /*!< CCAP_T::CMPADDR: CMPADDR Mask          */

#define CCAP_LUMA_Y1_THD_LUMA_Y1_THRESH_Pos (0)                                            /*!< CCAP_T::LUMA_Y1_THD: LUMA_Y1_THRESH Position*/
#define CCAP_LUMA_Y1_THD_LUMA_Y1_THRESH_Msk (0xfful << CCAP_LUMA_Y1_THD_LUMA_Y1_THRESH_Pos) /*!< CCAP_T::LUMA_Y1_THD: LUMA_Y1_THRESH Mask*/

#define CCAP_PKTSM_PKTSHMH_Pos           (0)                                               /*!< CCAP_T::PKTSM: PKTSHMH Position        */
#define CCAP_PKTSM_PKTSHMH_Msk           (0xfful << CCAP_PKTSM_PKTSHMH_Pos)                /*!< CCAP_T::PKTSM: PKTSHMH Mask            */

#define CCAP_PKTSM_PKTSHNH_Pos           (8)                                               /*!< CCAP_T::PKTSM: PKTSHNH Position        */
#define CCAP_PKTSM_PKTSHNH_Msk           (0xfful << CCAP_PKTSM_PKTSHNH_Pos)                /*!< CCAP_T::PKTSM: PKTSHNH Mask            */

#define CCAP_PKTSM_PKTSVMH_Pos           (16)                                              /*!< CCAP_T::PKTSM: PKTSVMH Position        */
#define CCAP_PKTSM_PKTSVMH_Msk           (0xfful << CCAP_PKTSM_PKTSVMH_Pos)                /*!< CCAP_T::PKTSM: PKTSVMH Mask            */

#define CCAP_PKTSM_PKTSVNH_Pos           (24)                                              /*!< CCAP_T::PKTSM: PKTSVNH Position        */
#define CCAP_PKTSM_PKTSVNH_Msk           (0xfful << CCAP_PKTSM_PKTSVNH_Pos)                /*!< CCAP_T::PKTSM: PKTSVNH Mask            */

#define CCAP_PLNSM_PLNSHMH_Pos           (0)                                               /*!< CCAP_T::PLNSM: PLNSHMH Position        */
#define CCAP_PLNSM_PLNSHMH_Msk           (0xfful << CCAP_PLNSM_PLNSHMH_Pos)                /*!< CCAP_T::PLNSM: PLNSHMH Mask            */

#define CCAP_PLNSM_PLNSHNH_Pos           (8)                                               /*!< CCAP_T::PLNSM: PLNSHNH Position        */
#define CCAP_PLNSM_PLNSHNH_Msk           (0xfful << CCAP_PLNSM_PLNSHNH_Pos)                /*!< CCAP_T::PLNSM: PLNSHNH Mask            */

#define CCAP_PLNSM_PLNSVMH_Pos           (16)                                              /*!< CCAP_T::PLNSM: PLNSVMH Position        */
#define CCAP_PLNSM_PLNSVMH_Msk           (0xfful << CCAP_PLNSM_PLNSVMH_Pos)                /*!< CCAP_T::PLNSM: PLNSVMH Mask            */

#define CCAP_PLNSM_PLNSVNH_Pos           (24)                                              /*!< CCAP_T::PLNSM: PLNSVNH Position        */
#define CCAP_PLNSM_PLNSVNH_Msk           (0xfful << CCAP_PLNSM_PLNSVNH_Pos)                /*!< CCAP_T::PLNSM: PLNSVNH Mask            */

#define CCAP_CURADDRP_CURADDR_Pos        (0)                                               /*!< CCAP_T::CURADDRP: CURADDR Position     */
#define CCAP_CURADDRP_CURADDR_Msk        (0xfffffffful << CCAP_CURADDRP_CURADDR_Pos)       /*!< CCAP_T::CURADDRP: CURADDR Mask         */

#define CCAP_CURADDRY_CURADDR_Pos        (0)                                               /*!< CCAP_T::CURADDRY: CURADDR Position     */
#define CCAP_CURADDRY_CURADDR_Msk        (0xfffffffful << CCAP_CURADDRY_CURADDR_Pos)       /*!< CCAP_T::CURADDRY: CURADDR Mask         */

#define CCAP_CURADDRU_CURADDR_Pos        (0)                                               /*!< CCAP_T::CURADDRU: CURADDR Position     */
#define CCAP_CURADDRU_CURADDR_Msk        (0xfffffffful << CCAP_CURADDRU_CURADDR_Pos)       /*!< CCAP_T::CURADDRU: CURADDR Mask         */

#define CCAP_CURADDRV_CURADDR_Pos        (0)                                               /*!< CCAP_T::CURADDRV: CURADDR Position     */
#define CCAP_CURADDRV_CURADDR_Msk        (0xfffffffful << CCAP_CURADDRV_CURADDR_Pos)       /*!< CCAP_T::CURADDRV: CURADDR Mask         */

#define CCAP_PKTBA0_BASEADDR_Pos         (0)                                               /*!< CCAP_T::PKTBA0: BASEADDR Position      */
#define CCAP_PKTBA0_BASEADDR_Msk         (0xfffffffful << CCAP_PKTBA0_BASEADDR_Pos)        /*!< CCAP_T::PKTBA0: BASEADDR Mask          */

#define CCAP_YBA_BASEADDR_Pos            (0)                                               /*!< CCAP_T::YBA: BASEADDR Position         */
#define CCAP_YBA_BASEADDR_Msk            (0xfffffffful << CCAP_YBA_BASEADDR_Pos)           /*!< CCAP_T::YBA: BASEADDR Mask             */

#define CCAP_UBA_BASEADDR_Pos            (0)                                               /*!< CCAP_T::UBA: BASEADDR Position         */
#define CCAP_UBA_BASEADDR_Msk            (0xfffffffful << CCAP_UBA_BASEADDR_Pos)           /*!< CCAP_T::UBA: BASEADDR Mask             */

#define CCAP_VBA_BASEADDR_Pos            (0)                                               /*!< CCAP_T::VBA: BASEADDR Position         */
#define CCAP_VBA_BASEADDR_Msk            (0xfffffffful << CCAP_VBA_BASEADDR_Pos)           /*!< CCAP_T::VBA: BASEADDR Mask             */

#define CCAP_MDCTL_WEN0_Pos              (0)                                               /*!< CCAP_T::MDCTL: WEN0 Position           */
#define CCAP_MDCTL_WEN0_Msk              (0x1ul << CCAP_MDCTL_WEN0_Pos)                    /*!< CCAP_T::MDCTL: WEN0 Mask               */

#define CCAP_MDCTL_WEN1_Pos              (1)                                               /*!< CCAP_T::MDCTL: WEN1 Position           */
#define CCAP_MDCTL_WEN1_Msk              (0x1ul << CCAP_MDCTL_WEN1_Pos)                    /*!< CCAP_T::MDCTL: WEN1 Mask               */

#define CCAP_MDCTL_WEN2_Pos              (2)                                               /*!< CCAP_T::MDCTL: WEN2 Position           */
#define CCAP_MDCTL_WEN2_Msk              (0x1ul << CCAP_MDCTL_WEN2_Pos)                    /*!< CCAP_T::MDCTL: WEN2 Mask               */

#define CCAP_MDCTL_WEN3_Pos              (3)                                               /*!< CCAP_T::MDCTL: WEN3 Position           */
#define CCAP_MDCTL_WEN3_Msk              (0x1ul << CCAP_MDCTL_WEN3_Pos)                    /*!< CCAP_T::MDCTL: WEN3 Mask               */

#define CCAP_MDCTL_WEN4_Pos              (4)                                               /*!< CCAP_T::MDCTL: WEN4 Position           */
#define CCAP_MDCTL_WEN4_Msk              (0x1ul << CCAP_MDCTL_WEN4_Pos)                    /*!< CCAP_T::MDCTL: WEN4 Mask               */

#define CCAP_MDCTL_WEN5_Pos              (5)                                               /*!< CCAP_T::MDCTL: WEN5 Position           */
#define CCAP_MDCTL_WEN5_Msk              (0x1ul << CCAP_MDCTL_WEN5_Pos)                    /*!< CCAP_T::MDCTL: WEN5 Mask               */

#define CCAP_MDCTL_WEN6_Pos              (6)                                               /*!< CCAP_T::MDCTL: WEN6 Position           */
#define CCAP_MDCTL_WEN6_Msk              (0x1ul << CCAP_MDCTL_WEN6_Pos)                    /*!< CCAP_T::MDCTL: WEN6 Mask               */

#define CCAP_MDCTL_WEN7_Pos              (7)                                               /*!< CCAP_T::MDCTL: WEN7 Position           */
#define CCAP_MDCTL_WEN7_Msk              (0x1ul << CCAP_MDCTL_WEN7_Pos)                    /*!< CCAP_T::MDCTL: WEN7 Mask               */

#define CCAP_MDCTL_WEN8_Pos              (8)                                               /*!< CCAP_T::MDCTL: WEN8 Position           */
#define CCAP_MDCTL_WEN8_Msk              (0x1ul << CCAP_MDCTL_WEN8_Pos)                    /*!< CCAP_T::MDCTL: WEN8 Mask               */

#define CCAP_MDCTL_WEN9_Pos              (9)                                               /*!< CCAP_T::MDCTL: WEN9 Position           */
#define CCAP_MDCTL_WEN9_Msk              (0x1ul << CCAP_MDCTL_WEN9_Pos)                    /*!< CCAP_T::MDCTL: WEN9 Mask               */

#define CCAP_MDCTL_WEN10_Pos             (10)                                              /*!< CCAP_T::MDCTL: WEN10 Position          */
#define CCAP_MDCTL_WEN10_Msk             (0x1ul << CCAP_MDCTL_WEN10_Pos)                   /*!< CCAP_T::MDCTL: WEN10 Mask              */

#define CCAP_MDCTL_WEN11_Pos             (11)                                              /*!< CCAP_T::MDCTL: WEN11 Position          */
#define CCAP_MDCTL_WEN11_Msk             (0x1ul << CCAP_MDCTL_WEN11_Pos)                   /*!< CCAP_T::MDCTL: WEN11 Mask              */

#define CCAP_MDCTL_WEN12_Pos             (12)                                              /*!< CCAP_T::MDCTL: WEN12 Position          */
#define CCAP_MDCTL_WEN12_Msk             (0x1ul << CCAP_MDCTL_WEN12_Pos)                   /*!< CCAP_T::MDCTL: WEN12 Mask              */

#define CCAP_MDCTL_WEN13_Pos             (13)                                              /*!< CCAP_T::MDCTL: WEN13 Position          */
#define CCAP_MDCTL_WEN13_Msk             (0x1ul << CCAP_MDCTL_WEN13_Pos)                   /*!< CCAP_T::MDCTL: WEN13 Mask              */

#define CCAP_MDCTL_WEN14_Pos             (14)                                              /*!< CCAP_T::MDCTL: WEN14 Position          */
#define CCAP_MDCTL_WEN14_Msk             (0x1ul << CCAP_MDCTL_WEN14_Pos)                   /*!< CCAP_T::MDCTL: WEN14 Mask              */

#define CCAP_MDCTL_WEN15_Pos             (15)                                              /*!< CCAP_T::MDCTL: WEN15 Position          */
#define CCAP_MDCTL_WEN15_Msk             (0x1ul << CCAP_MDCTL_WEN15_Pos)                   /*!< CCAP_T::MDCTL: WEN15 Mask              */

#define CCAP_MDTRG_WK_TRGSEL_Pos         (0)                                               /*!< CCAP_T::MDTRG_WK: TRGSEL Position      */
#define CCAP_MDTRG_WK_TRGSEL_Msk         (0x3ul << CCAP_MDTRG_WK_TRGSEL_Pos)               /*!< CCAP_T::MDTRG_WK: TRGSEL Mask          */

#define CCAP_MDTRG_WK_WKEN_Pos           (16)                                              /*!< CCAP_T::MDTRG_WK: WKEN Position        */
#define CCAP_MDTRG_WK_WKEN_Msk           (0x1ul << CCAP_MDTRG_WK_WKEN_Pos)                 /*!< CCAP_T::MDTRG_WK: WKEN Mask            */

#define CCAP_MDTRG_WK_WKF_Pos            (24)                                              /*!< CCAP_T::MDTRG_WK: WKF Position         */
#define CCAP_MDTRG_WK_WKF_Msk            (0x1ul << CCAP_MDTRG_WK_WKF_Pos)                  /*!< CCAP_T::MDTRG_WK: WKF Mask             */

#define CCAP_MDTTH_MDTTH_Pos             (0)                                               /*!< CCAP_T::MDTTH: MDTTH Position          */
#define CCAP_MDTTH_MDTTH_Msk             (0x1ffffful << CCAP_MDTTH_MDTTH_Pos)              /*!< CCAP_T::MDTTH: MDTTH Mask              */

#define CCAP_MDTSAD_MDTSAD_Pos           (0)                                               /*!< CCAP_T::MDTSAD: MDTSAD Position        */
#define CCAP_MDTSAD_MDTSAD_Msk           (0x1ffffful << CCAP_MDTSAD_MDTSAD_Pos)            /*!< CCAP_T::MDTSAD: MDTSAD Mask            */

#define CCAP_MDWOCTH_MDWOCTH_Pos         (0)                                               /*!< CCAP_T::MDWOCTH: MDWOCTH Position      */
#define CCAP_MDWOCTH_MDWOCTH_Msk         (0xful << CCAP_MDWOCTH_MDWOCTH_Pos)               /*!< CCAP_T::MDWOCTH: MDWOCTH Mask          */

#define CCAP_MDWOC_MDWOC_Pos             (0)                                               /*!< CCAP_T::MDWOC: MDWOC Position          */
#define CCAP_MDWOC_MDWOC_Msk             (0x1ful << CCAP_MDWOC_MDWOC_Pos)                  /*!< CCAP_T::MDWOC: MDWOC Mask              */

#define CCAP_MDWTH0_MDWTH_Pos            (0)                                               /*!< CCAP_T::MDWTH0: MDWTH Position         */
#define CCAP_MDWTH0_MDWTH_Msk            (0x1fffful << CCAP_MDWTH0_MDWTH_Pos)              /*!< CCAP_T::MDWTH0: MDWTH Mask             */

#define CCAP_MDWTH1_MDWTH_Pos            (0)                                               /*!< CCAP_T::MDWTH1: MDWTH Position         */
#define CCAP_MDWTH1_MDWTH_Msk            (0x1fffful << CCAP_MDWTH1_MDWTH_Pos)              /*!< CCAP_T::MDWTH1: MDWTH Mask             */

#define CCAP_MDWTH2_MDWTH_Pos            (0)                                               /*!< CCAP_T::MDWTH2: MDWTH Position         */
#define CCAP_MDWTH2_MDWTH_Msk            (0x1fffful << CCAP_MDWTH2_MDWTH_Pos)              /*!< CCAP_T::MDWTH2: MDWTH Mask             */

#define CCAP_MDWTH3_MDWTH_Pos            (0)                                               /*!< CCAP_T::MDWTH3: MDWTH Position         */
#define CCAP_MDWTH3_MDWTH_Msk            (0x1fffful << CCAP_MDWTH3_MDWTH_Pos)              /*!< CCAP_T::MDWTH3: MDWTH Mask             */

#define CCAP_MDWTH4_MDWTH_Pos            (0)                                               /*!< CCAP_T::MDWTH4: MDWTH Position         */
#define CCAP_MDWTH4_MDWTH_Msk            (0x1fffful << CCAP_MDWTH4_MDWTH_Pos)              /*!< CCAP_T::MDWTH4: MDWTH Mask             */

#define CCAP_MDWTH5_MDWTH_Pos            (0)                                               /*!< CCAP_T::MDWTH5: MDWTH Position         */
#define CCAP_MDWTH5_MDWTH_Msk            (0x1fffful << CCAP_MDWTH5_MDWTH_Pos)              /*!< CCAP_T::MDWTH5: MDWTH Mask             */

#define CCAP_MDWTH6_MDWTH_Pos            (0)                                               /*!< CCAP_T::MDWTH6: MDWTH Position         */
#define CCAP_MDWTH6_MDWTH_Msk            (0x1fffful << CCAP_MDWTH6_MDWTH_Pos)              /*!< CCAP_T::MDWTH6: MDWTH Mask             */

#define CCAP_MDWTH7_MDWTH_Pos            (0)                                               /*!< CCAP_T::MDWTH7: MDWTH Position         */
#define CCAP_MDWTH7_MDWTH_Msk            (0x1fffful << CCAP_MDWTH7_MDWTH_Pos)              /*!< CCAP_T::MDWTH7: MDWTH Mask             */

#define CCAP_MDWTH8_MDWTH_Pos            (0)                                               /*!< CCAP_T::MDWTH8: MDWTH Position         */
#define CCAP_MDWTH8_MDWTH_Msk            (0x1fffful << CCAP_MDWTH8_MDWTH_Pos)              /*!< CCAP_T::MDWTH8: MDWTH Mask             */

#define CCAP_MDWTH9_MDWTH_Pos            (0)                                               /*!< CCAP_T::MDWTH9: MDWTH Position         */
#define CCAP_MDWTH9_MDWTH_Msk            (0x1fffful << CCAP_MDWTH9_MDWTH_Pos)              /*!< CCAP_T::MDWTH9: MDWTH Mask             */

#define CCAP_MDWTH10_MDWTH_Pos           (0)                                               /*!< CCAP_T::MDWTH10: MDWTH Position        */
#define CCAP_MDWTH10_MDWTH_Msk           (0x1fffful << CCAP_MDWTH10_MDWTH_Pos)             /*!< CCAP_T::MDWTH10: MDWTH Mask            */

#define CCAP_MDWTH11_MDWTH_Pos           (0)                                               /*!< CCAP_T::MDWTH11: MDWTH Position        */
#define CCAP_MDWTH11_MDWTH_Msk           (0x1fffful << CCAP_MDWTH11_MDWTH_Pos)             /*!< CCAP_T::MDWTH11: MDWTH Mask            */

#define CCAP_MDWTH12_MDWTH_Pos           (0)                                               /*!< CCAP_T::MDWTH12: MDWTH Position        */
#define CCAP_MDWTH12_MDWTH_Msk           (0x1fffful << CCAP_MDWTH12_MDWTH_Pos)             /*!< CCAP_T::MDWTH12: MDWTH Mask            */

#define CCAP_MDWTH13_MDWTH_Pos           (0)                                               /*!< CCAP_T::MDWTH13: MDWTH Position        */
#define CCAP_MDWTH13_MDWTH_Msk           (0x1fffful << CCAP_MDWTH13_MDWTH_Pos)             /*!< CCAP_T::MDWTH13: MDWTH Mask            */

#define CCAP_MDWTH14_MDWTH_Pos           (0)                                               /*!< CCAP_T::MDWTH14: MDWTH Position        */
#define CCAP_MDWTH14_MDWTH_Msk           (0x1fffful << CCAP_MDWTH14_MDWTH_Pos)             /*!< CCAP_T::MDWTH14: MDWTH Mask            */

#define CCAP_MDWTH15_MDWTH_Pos           (0)                                               /*!< CCAP_T::MDWTH15: MDWTH Position        */
#define CCAP_MDWTH15_MDWTH_Msk           (0x1fffful << CCAP_MDWTH15_MDWTH_Pos)             /*!< CCAP_T::MDWTH15: MDWTH Mask            */

#define CCAP_MDWSAD0_MDWSAD_Pos          (0)                                               /*!< CCAP_T::MDWSAD0: MDWSAD Position       */
#define CCAP_MDWSAD0_MDWSAD_Msk          (0x1fffful << CCAP_MDWSAD0_MDWSAD_Pos)            /*!< CCAP_T::MDWSAD0: MDWSAD Mask           */

#define CCAP_MDWSAD1_MDWSAD_Pos          (0)                                               /*!< CCAP_T::MDWSAD1: MDWSAD Position       */
#define CCAP_MDWSAD1_MDWSAD_Msk          (0x1fffful << CCAP_MDWSAD1_MDWSAD_Pos)            /*!< CCAP_T::MDWSAD1: MDWSAD Mask           */

#define CCAP_MDWSAD2_MDWSAD_Pos          (0)                                               /*!< CCAP_T::MDWSAD2: MDWSAD Position       */
#define CCAP_MDWSAD2_MDWSAD_Msk          (0x1fffful << CCAP_MDWSAD2_MDWSAD_Pos)            /*!< CCAP_T::MDWSAD2: MDWSAD Mask           */

#define CCAP_MDWSAD3_MDWSAD_Pos          (0)                                               /*!< CCAP_T::MDWSAD3: MDWSAD Position       */
#define CCAP_MDWSAD3_MDWSAD_Msk          (0x1fffful << CCAP_MDWSAD3_MDWSAD_Pos)            /*!< CCAP_T::MDWSAD3: MDWSAD Mask           */

#define CCAP_MDWSAD4_MDWSAD_Pos          (0)                                               /*!< CCAP_T::MDWSAD4: MDWSAD Position       */
#define CCAP_MDWSAD4_MDWSAD_Msk          (0x1fffful << CCAP_MDWSAD4_MDWSAD_Pos)            /*!< CCAP_T::MDWSAD4: MDWSAD Mask           */

#define CCAP_MDWSAD5_MDWSAD_Pos          (0)                                               /*!< CCAP_T::MDWSAD5: MDWSAD Position       */
#define CCAP_MDWSAD5_MDWSAD_Msk          (0x1fffful << CCAP_MDWSAD5_MDWSAD_Pos)            /*!< CCAP_T::MDWSAD5: MDWSAD Mask           */

#define CCAP_MDWSAD6_MDWSAD_Pos          (0)                                               /*!< CCAP_T::MDWSAD6: MDWSAD Position       */
#define CCAP_MDWSAD6_MDWSAD_Msk          (0x1fffful << CCAP_MDWSAD6_MDWSAD_Pos)            /*!< CCAP_T::MDWSAD6: MDWSAD Mask           */

#define CCAP_MDWSAD7_MDWSAD_Pos          (0)                                               /*!< CCAP_T::MDWSAD7: MDWSAD Position       */
#define CCAP_MDWSAD7_MDWSAD_Msk          (0x1fffful << CCAP_MDWSAD7_MDWSAD_Pos)            /*!< CCAP_T::MDWSAD7: MDWSAD Mask           */

#define CCAP_MDWSAD8_MDWSAD_Pos          (0)                                               /*!< CCAP_T::MDWSAD8: MDWSAD Position       */
#define CCAP_MDWSAD8_MDWSAD_Msk          (0x1fffful << CCAP_MDWSAD8_MDWSAD_Pos)            /*!< CCAP_T::MDWSAD8: MDWSAD Mask           */

#define CCAP_MDWSAD9_MDWSAD_Pos          (0)                                               /*!< CCAP_T::MDWSAD9: MDWSAD Position       */
#define CCAP_MDWSAD9_MDWSAD_Msk          (0x1fffful << CCAP_MDWSAD9_MDWSAD_Pos)            /*!< CCAP_T::MDWSAD9: MDWSAD Mask           */

#define CCAP_MDWSAD10_MDWSAD_Pos         (0)                                               /*!< CCAP_T::MDWSAD10: MDWSAD Position      */
#define CCAP_MDWSAD10_MDWSAD_Msk         (0x1fffful << CCAP_MDWSAD10_MDWSAD_Pos)           /*!< CCAP_T::MDWSAD10: MDWSAD Mask          */

#define CCAP_MDWSAD11_MDWSAD_Pos         (0)                                               /*!< CCAP_T::MDWSAD11: MDWSAD Position      */
#define CCAP_MDWSAD11_MDWSAD_Msk         (0x1fffful << CCAP_MDWSAD11_MDWSAD_Pos)           /*!< CCAP_T::MDWSAD11: MDWSAD Mask          */

#define CCAP_MDWSAD12_MDWSAD_Pos         (0)                                               /*!< CCAP_T::MDWSAD12: MDWSAD Position      */
#define CCAP_MDWSAD12_MDWSAD_Msk         (0x1fffful << CCAP_MDWSAD12_MDWSAD_Pos)           /*!< CCAP_T::MDWSAD12: MDWSAD Mask          */

#define CCAP_MDWSAD13_MDWSAD_Pos         (0)                                               /*!< CCAP_T::MDWSAD13: MDWSAD Position      */
#define CCAP_MDWSAD13_MDWSAD_Msk         (0x1fffful << CCAP_MDWSAD13_MDWSAD_Pos)           /*!< CCAP_T::MDWSAD13: MDWSAD Mask          */

#define CCAP_MDWSAD14_MDWSAD_Pos         (0)                                               /*!< CCAP_T::MDWSAD14: MDWSAD Position      */
#define CCAP_MDWSAD14_MDWSAD_Msk         (0x1fffful << CCAP_MDWSAD14_MDWSAD_Pos)           /*!< CCAP_T::MDWSAD14: MDWSAD Mask          */

#define CCAP_MDWSAD15_MDWSAD_Pos         (0)                                               /*!< CCAP_T::MDWSAD15: MDWSAD Position      */
#define CCAP_MDWSAD15_MDWSAD_Msk         (0x1fffful << CCAP_MDWSAD15_MDWSAD_Pos)           /*!< CCAP_T::MDWSAD15: MDWSAD Mask          */

/** @} CCAP_CONST */
/** @} end of CCAP register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __CCAP_REG_H__ */
