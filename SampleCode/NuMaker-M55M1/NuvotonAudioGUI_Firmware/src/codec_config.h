/******************************************************************************
 * @file     codec_config.h
 * @version  V1.00
 * @brief    NAU88C22/NAU88L25/NAU88L21/NAU85L40 codec configuration header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __CODEC_CONFIG_H__
#define __CODEC_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro define                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct
{
    uint8_t u8DeviceAddr;
    uint16_t u16Counter;
    uint16_t u16MaxCount;
    uint8_t *pau8Cmd;
} S_MIC_I2CCTRL;

typedef struct
{
    uint8_t u8Reg[2];
    uint8_t u8Value[2];
} S_MIC_I2CCMD;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
// codec init
void Codec_InitCallback(void);

// NAU88C22 codec init via callback function
void NAU88C22_CodecMst_CallBackInit(void);

// Direct write function for NAU88C22
void I2C_WriteNAU88C22(uint8_t u8Addr, uint16_t u16Data);

// NAU88C22 codec init function
void NAU88C22_CodecMst_Init(uint32_t u32SampleRate);

#ifdef __cplusplus
}
#endif

#endif  // __CODEC_CONFIG_H__
