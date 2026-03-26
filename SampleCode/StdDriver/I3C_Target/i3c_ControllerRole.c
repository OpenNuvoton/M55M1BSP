/**************************************************************************//**
 * @file     i3c_ControllerRole.c
 * @version  V3.00
 * @brief    Functions for I3C Controller Role.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "i3c_DeviceFunc.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static void ExecCTROperation(I3C_DEVICE_T *dev);

/**
  * @brief  SDR Dummy Write to wake up target.
  */
static int32_t SDRReadDummy(I3C_DEVICE_T *dev, uint8_t tgt)
{
    volatile uint32_t i;
    int32_t iRet = I3C_STS_NO_ERR;
    uint16_t len;
    uint8_t g_DevRxData[I3C_DEVICE_TX_BUF_CNT * 4];

    if (dev->port->DEVADR[tgt] & I3C_DEVADR_DEVICE_Msk)
    {
        printf("\nTarget index-%d NOT at I3C mode.\n", tgt);
        return -1;
    }

    // SDR Read operation
    len = 0;
    dev->speed_mode = I3C_DEVI3C_SPEED_SDR0;
    iRet = I3C_FuncSDRRead(dev, tgt, (uint8_t *)g_DevRxData, &len);

    if (iRet != I3C_STS_NO_ERR)
    {
        return -1;
    }

    printf("\n[ SDR Read PASS ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);

    for (i = 0; i < len; i++)
    {
        printf("%02x ", g_DevRxData[i]);

        if ((i % 8) == 7)
        {
            printf("\n\t");
        }
    }

    printf("\n");
    return 0;
}

/**
  * @brief  SDR Write and Read Sample Flow.
  */
static int32_t SDRWRSample(I3C_DEVICE_T *dev, uint8_t tgt)
{
    volatile uint32_t i;
    static uint8_t sInitVal = 0;
    int32_t iRet = I3C_STS_NO_ERR;
    uint16_t len;
    uint8_t g_DevTxData[I3C_DEVICE_TX_BUF_CNT * 4], g_DevRxData[I3C_DEVICE_TX_BUF_CNT * 4];

    if (dev->port->DEVADR[tgt] & I3C_DEVADR_DEVICE_Msk)
    {
        printf("\nTarget index-%d NOT at I3C mode.\n", tgt);
        return -1;
    }

    // SDR Write operation
    len = 16;

    for (i = 0; i < len; i++)
    {
        g_DevTxData[i] = ((0x10 + i) + sInitVal);
    }

    dev->speed_mode = I3C_DEVI3C_SPEED_SDR0;
    iRet = I3C_FuncSDRWrite(dev, tgt, (uint8_t *)g_DevTxData, len);

    if (iRet != I3C_STS_NO_ERR)
    {
        return -1;
    }

    printf("\n[ SDR Write PASS ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);

    for (i = 0; i < len; i++)
    {
        printf("%02x ", dev->tx_buf[i]);

        if ((i % 8) == 7)
        {
            printf("\n\t");
        }
    }

    sInitVal++;
    printf("\n");
    CLK_SysTickDelay(10000);
    // SDR Read operation
    len = 16;
    dev->speed_mode = I3C_DEVI3C_SPEED_SDR0;
    iRet = I3C_FuncSDRRead(dev, tgt, (uint8_t *)g_DevRxData, &len);

    if (iRet != I3C_STS_NO_ERR)
    {
        return -1;
    }

    printf("\n[ SDR Read PASS ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);

    for (i = 0; i < len; i++)
    {
        printf("%02x ", g_DevRxData[i]);

        if ((i % 8) == 7)
        {
            printf("\n\t");
        }
    }

    printf("\n");
    return 0;
}

/**
  * @brief  HDR-DDR Write and Read Sample Flow.
  */
static int32_t HDRDDRWRSample(I3C_DEVICE_T *dev, uint8_t tgt)
{
    volatile uint32_t i;
    static uint8_t sInitVal = 0;
    int32_t iRet = I3C_STS_NO_ERR;
    uint16_t len;
    uint8_t g_DevTxData[I3C_DEVICE_TX_BUF_CNT * 4], g_DevRxData[I3C_DEVICE_TX_BUF_CNT * 4];

    if (dev->port->DEVADR[tgt] & I3C_DEVADR_DEVICE_Msk)
    {
        printf("\nTarget index-%d NOT at I3C mode.\n", tgt);
        return -1;
    }

    // HDR-DDR Write operation
    len = 16;

    for (i = 0; i < len; i++)
    {
        g_DevTxData[i] = ((0x10 + i) + sInitVal);
    }

    iRet = I3C_FuncDDRWrite(dev, tgt, (uint8_t *)g_DevTxData, len);

    if (iRet != I3C_STS_NO_ERR)
    {
        return -1;
    }

    printf("\n[ HDR-DDR Write PASS ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);

    for (i = 0; i < len; i++)
    {
        printf("%02x ", dev->tx_buf[i]);

        if ((i % 8) == 7)
        {
            printf("\n\t");
        }
    }

    sInitVal++;
    printf("\n");
    CLK_SysTickDelay(10000);
    // HDR-DDR Read operation
    len = 16;
    iRet = I3C_FuncDDRRead(dev, tgt, (uint8_t *)g_DevRxData, &len);

    if (iRet != I3C_STS_NO_ERR)
    {
        return -1;
    }

    printf("\n[ HDR-DDR Read PASS ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);

    for (i = 0; i < len; i++)
    {
        printf("%02x ", g_DevRxData[i]);

        if ((i % 8) == 7)
        {
            printf("\n\t");
        }
    }

    printf("\n");
    return 0;
}

#if (DEVICE_CONTROLLER_ROLE == 1)
/**
  * @brief  I2C FM+ Write and Read Sample Flow.
  */
static int32_t I2CFMPWRSample(I3C_DEVICE_T *dev, uint8_t tgt)
{
    volatile uint32_t i;
    static uint8_t sInitVal = 0;
    int32_t iRet = I3C_STS_NO_ERR;
    uint16_t len;
    uint8_t g_DevTxData[I3C_DEVICE_TX_BUF_CNT * 4], g_DevRxData[I3C_DEVICE_TX_BUF_CNT * 4];

    if ((dev->port->DEVADR[tgt] & I3C_DEVADR_DEVICE_Msk) == 0)
    {
        printf("\nTarget index-%d NOT at I2C mode.\n", tgt);
        return -1;
    }

    // I2C FM+ Write operation
    len = 16;

    for (i = 0; i < len; i++)
    {
        g_DevTxData[i] = ((0x10 + i) + sInitVal);
    }

    dev->speed_mode = I3C_DEVI2C_SPEED_I2CFMPLUS;
    iRet = I3C_FuncSDRWrite(dev, tgt, (uint8_t *)g_DevTxData, len);

    if (iRet != I3C_STS_NO_ERR)
    {
        return -1;
    }

    printf("\n[ I2C FM+ Write PASS ] (Tgt: 0x%02x)\n\t", dev->target_sa[tgt]);

    for (i = 0; i < len; i++)
    {
        printf("%02x ", dev->tx_buf[i]);

        if ((i % 8) == 7)
        {
            printf("\n\t");
        }
    }

    sInitVal++;
    printf("\n");
    CLK_SysTickDelay(10000);
    // I2C FM+ Read operation
    len = 16;
    dev->speed_mode = I3C_DEVI2C_SPEED_I2CFMPLUS;
    iRet = I3C_FuncSDRRead(dev, tgt, (uint8_t *)g_DevRxData, &len);

    if (iRet != I3C_STS_NO_ERR)
    {
        return -1;
    }

    printf("\n[ I2C FM+ Read PASS ] (Tgt: 0x%02x)\n\t", dev->target_sa[tgt]);

    for (i = 0; i < len; i++)
    {
        printf("%02x ", g_DevRxData[i]);

        if ((i % 8) == 7)
        {
            printf("\n\t");
        }
    }

    printf("\n");
    return 0;
}

/**
  * @brief  Enter CCC Command.
  */
static uint32_t ParseCCCCommand(I3C_DEVICE_T *dev)
{
    char cInputTemp = 0x00;
    char strInput[16] = {0};
    uint32_t loop = 0;

    while (cInputTemp != 0x0D)
    {
        if (dev->intsts != 0)   // ESC
        {
            break;
        }

        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
        {
            cInputTemp = (char)DEBUG_PORT->DAT;
        }
        else
        {
            continue;
        }

        if (cInputTemp == 27)   // ESC
        {
            break;
        }

        strInput[loop] = cInputTemp;
        printf("%c", cInputTemp);
        loop++;

        if (loop == 16)
        {
            break;
        }
    }

    printf("\n");

    /* For GET CCC */
    if (strncmp(strInput, "GETSTATUS", 9) == 0)
    {
        return I3C_CCC_GETSTATUS;
    }

    if (strncmp(strInput, "GETPID", 6) == 0)
    {
        return I3C_CCC_GETPID;
    }

    if (strncmp(strInput, "GETBCR", 6) == 0)
    {
        return I3C_CCC_GETBCR;
    }

    if (strncmp(strInput, "GETDCR", 6) == 0)
    {
        return I3C_CCC_GETDCR;
    }

    if (strncmp(strInput, "GETMWL", 6) == 0)
    {
        return I3C_CCC_GETMWL;
    }

    if (strncmp(strInput, "GETMRL", 6) == 0)
    {
        return I3C_CCC_GETMRL;
    }

    if (strncmp(strInput, "GETCAPS", 7) == 0)
    {
        return I3C_CCC_GETCAPS;
    }

    if (strncmp(strInput, "GETMXDS", 7) == 0)
    {
        return I3C_CCC_GETMXDS;
    }

    /* For SET CCC */
    if (strncmp(strInput, "SETMWL", 6) == 0)
    {
        return (I3C_CCC_SETMWL(1) | BIT31);
    }

    if (strncmp(strInput, "SETMRL", 6) == 0)
    {
        return (I3C_CCC_SETMRL(1) | BIT31);
    }

    if (strncmp(strInput, "ENEC", 4) == 0)
    {
        return (I3C_CCC_ENEC(1) | BIT31);
    }

    if (strncmp(strInput, "DISEC", 5) == 0)
    {
        return (I3C_CCC_DISEC(1) | BIT31);
    }

    return (uint32_t)I3C_STS_INVALID_INPUT;
}

/**
  * @brief  Common Command Codes Flow.
  */
static int32_t CCCFlow(I3C_DEVICE_T *dev, uint8_t tgt)
{
    uint32_t ccc = 0;
    (void) tgt;

    do
    {
        printf("Input the Common Command Code (CCC). (Invalid input to ESC)\n");
        printf("    - GETSTATUS \n");
        printf("    - GETPID    \n");
        printf("    - GETBCR    \n");
        printf("    - GETDCR    \n");
        printf("    - GETMWL    \n");
        printf("    - GETMRL    \n");
        printf("    - GETCAPS   \n");
        printf("    - GETMXDS   \n");
        printf("    ============\n");
        printf("    - SETMWL    \n");
        printf("    - SETMRL    \n");
        printf("    - ENEC      \n");
        printf("    - DISEC     \n");
        printf("    ============\n");
        ccc = ParseCCCCommand(dev);

        if (ccc != (uint32_t)I3C_STS_INVALID_INPUT)
        {
            if ((ccc & BIT31) == 0)
            {
                ccc &= 0xFF;

                if (I3C_FuncGETCCCResp(dev, ccc, 0) != I3C_STS_NO_ERR)
                {
                    break;
                }
            }
            else
            {
                ccc &= 0xFF;

                if (I3C_FuncSETCCC(dev, ccc, 0) != I3C_STS_NO_ERR)
                {
                    break;
                }
            }
        }
    } while (ccc != (uint32_t)I3C_STS_INVALID_INPUT);

    printf("\nExit Common Command Codes Flow.\n\n");
    return 0;
}
#endif

/**
  * @brief  Polling a specified char for Controller operation.
  */
static void ExecCTROperation(I3C_DEVICE_T *dev)
{
    char mode;

    if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
    {
        mode = (char)DEBUG_PORT->DAT;
#if (DEVICE_CONTROLLER_ROLE == 1)
        uint32_t i = 0;

        switch (mode)
        {
            case 'w':
                // SDR Dummy Read to wake up Target-0;
                SDRReadDummy(dev, 0);
                break;

            case 'f':
                // I2C FM+ Write and Read to Target-0;
                I2CFMPWRSample(dev, 0);
                break;

            case 's':
                // SDR Write and Read to Target-0;
                SDRWRSample(dev, 0);
                break;

            case 'd':
                // HDR-DDR Write and Read to Target-0;
                HDRDDRWRSample(dev, 0);
                break;

            case 'p':
                // Send GETPID CCC to Target-0
                I3C_FuncGETCCCResp(dev, I3C_CCC_GETPID, 0);
                break;

            case 'r':
                // Send RSTDAA CCC to reset all Targets at I2C mode
                I3C_FuncDAAssign(dev, I3C_CCC_RSTDAA, 0, 0);
                break;

            case 'e':
                // Send ENTDAA CCC to assign all I3C Targets
                I3C_FuncDAAssign(dev, I3C_CCC_ENTDAA, 7, 0);
                break;

            case 'C':
                // All Common Command Codes (CCC) flow
                CCCFlow(dev, 0);
                break;

            default:
                printf("\n");
                printf("[f] I2C FM+ : Write 16-bytes to Target-0, then read back 16-bytes to compare \n");
                printf("[s] SDR :     Write 16-bytes to Target-0, then read back 16-bytes to compare \n");
                printf("[d] HDR-DDR : Write 16-bytes to Target-0, then read back 16-bytes to compare \n");
                printf("[w] SDR Dummy Read : Read 0-bytes to wake up Target-0 \n");
                printf("[p] GETPID CCC : Get Target-0's PID \n");
                printf("[e] ENTDAA CCC : Set all Targets in I3C mode \n");
                printf("[r] RSTDAA CCC : Set all Targets in I2C mode \n");
                printf("[C] Common Command Codes (CCC) flow \n");
                /* Dump all Targets info. */
                printf("\nAll Target's info on the bus:\n");

                for (i = 0; i < 7; i++)
                {
                    if (dev->port->DEVADR[i] & I3C_DEVADR_DEVICE_Msk)
                    {
                        // Target at I2C mode
                        dev->target_sa[i] = ((dev->port->DEVADR[i] & I3C_DEVADR_STADR_Msk) >> I3C_DEVADR_STADR_Pos);

                        if (dev->target_sa[i] == 0x0)
                        {
                            continue;
                        }

                        printf("\tTarget #%d:\n", i);
                        printf("\t - SADDR          = 0x%02x \n", dev->target_sa[i]);
                    }
                    else
                    {
                        // Target at I3C mode
                        dev->target_da[i] = ((dev->port->TGTCHAR[i].DADDR & I3C_TGTCHAR4_DADDR_Msk) >> I3C_TGTCHAR4_DADDR_Pos);
                        printf("\tTarget #%d:\n", i);
                        printf("\t - Provisional ID = 0x%08x%02x \n", dev->port->TGTCHAR[i].PIDMSB, dev->port->TGTCHAR[i].PIDLSB);
                        printf("\t - BCR, DCR       = 0x%08x \n", dev->port->TGTCHAR[i].BCRDCR);
                        printf("\t - DADDR          = 0x%02x \n", dev->target_da[i]);
                    }
                }

                printf("\n");
                break;
        }

#else

        switch (mode)
        {
            case 's':
                // SDR WR to Target-0, the Main Controller as Target Role
                SDRWRSample(dev, 0);
                break;

            case 'd':
                // HDR-DDR WR to Target-0, the Main Controller as Target Role
                HDRDDRWRSample(dev, 0);
                break;

            case 'p':
                // Send GETPID CCC to Target-0
                I3C_FuncGETCCCResp(dev, I3C_CCC_GETPID, 0);
                break;

            default:
                printf("\n");
                printf("[s] SDR :     Write 16-bytes to Target (Main Controller), then read back 16-bytes to compare \n");
                printf("[d] HDR-DDR : Write 16-bytes to Target (Main Controller), then read back 16-bytes to compare \n");
                printf("[b] HDR-BT :  Write 16-bytes to Target (Main Controller), then read back 16-bytes to compare \n");
                printf("[p] GETPID CCC : Get Target-0's PID \n");
                printf("\n");
                break;
        }

#endif
    }
}

/**
  * @brief  Run in I3C Controller Role.
  */
void I3C_ControllerRole(I3C_DEVICE_T *dev, uint32_t u32IsInit)
{

    if (u32IsInit == 1)
    {
        /* Initialize the device as I3C Controller Role */
        /* Initializes the I3C device */
        I3C_Init(dev);
    }
    else
    {
        /* Device has switched to I3C Controller Role */
        printf("\n*** Switched to Controller Role ***\n");
        dev->device_role = I3C_CONTROLLER;
#if (DEVICE_CONTROLLER_ROLE == 0)
        volatile uint32_t i;

        /* The I3C Secondary Controller need to allocate Device Address Table for all Targets */
        for (i = 0; i < 7; i++)
        {
            if (i >= dev->target_count)
            {
                dev->target_da[i] = 0;
            }

            I3C_SetDeviceAddr(dev->port, i, I3C_DEVTYPE_I3C, dev->target_da[i], 0x0);
        }

#endif
    }

    while (1)
    {
        if (dev->intsts != 0)
        {
            if (dev->intsts & I3C_INTSTS_IBI_RECEIVED)
            {
                dev->intsts &= ~I3C_INTSTS_IBI_RECEIVED;
                I3C_FuncIBIReceived(dev);
            }

            if (dev->intsts & I3C_INTSTS_BUSOWNER_UPDATED)
            {
                /* Switch to I3C Target Role */
                dev->intsts = 0;
                break;
            }

            dev->intsts = 0;
        }

        /* Polling a specified char for Controller operation */
        ExecCTROperation(dev);
    }

    /* Jump to execute in i3c_TargetRole.c */
}
