/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to switch role between Master and Slave.
 *          This sample code needs to use two boards.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define I3C_MASTER_DA       (0x69)
#define I3C_SLAVE_DA        (0x18)
#define I3C_SLAVE_SA        (0x68)
#define I3C0_MID            (0x8123UL)
#define I3C0_PID            (0xA13573C0UL)
/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t   g_RespQ[I3C_DEVICE_RESP_QUEUE_CNT];
volatile uint32_t   g_u32IntSelMask = 0, g_u32IntOccurredMask = 0;
volatile uint32_t   g_u32RespStatus = I3C_STS_NO_ERR;

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);
    /* Use SystemCoreClockUpdate() to calculate and update SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable peripheral clock */
    CLK_EnableModuleClock(I3C0_MODULE);
    /* Set multi-function pins for I3C0 SDA and SCL */
    SET_I3C0_SCL_PB1();
    SET_I3C0_SDA_PB0();
    SYS_ResetModule(SYS_I3C0RST);
    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Set SCL slew rate to GPIO_SLEWCTL_FAST0, SDA slew rate to GPIO_SLEWCTL_HIGH */
    GPIO_SetSlewCtl(PB, BIT1, GPIO_SLEWCTL_FAST0);
    GPIO_SetSlewCtl(PB, BIT0, GPIO_SLEWCTL_HIGH);
    /* Lock protected registers */
    SYS_LockReg();
}

/**
  * @brief  Polling any char to switch I3C Role.
  */
static void PollingToSwitchRole(void)
{
    if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
    {
        char ch = (char)DEBUG_PORT->DAT;

        do
        {
            if (!I3C_IS_DA_VALID(I3C0))
            {
                printf("\nSlave controller isn't in I3C mode. Please enter to I3C mode first.\n\n");
                break;
            }

            if (!I3C_IS_MASTER(I3C0))
            {
                printf("Slave issue MR request to become Master ...\n");

                if (I3C_SendMRRequest(I3C0) != I3C_STS_NO_ERR)
                {
                    printf("MR request has error.\n");
                }
            }
            else
            {
                NVIC_DisableIRQ(I3C0_IRQn);
                printf("Master send GETACCMST to become Slave ...");
                I3C_SetDeviceAddr(I3C0, 0, I3C_DEVTYPE_I3C, I3C_MASTER_DA, 0x00);

                if (I3C_STS_NO_ERR != I3C_UnicastGETACCMST(I3C0, 0, NULL))
                {
                    printf("send GETACCMST has error.\n");
                }

                NVIC_EnableIRQ(I3C0_IRQn);
            }
        } while (0);
    }
}

/**
  * @brief  The I3C0 default IRQ, declared in startup_M55M1.c.
  */
NVT_ITCM void I3C0_IRQHandler(void)
{
    printf("\n");

    if (I3C0->INTSTS & I3C_INTSTS_BUSOWNER_Msk)
    {
        printf("[ INT ] BUSOWNER\n");
        I3C0->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        I3C0->INTSTS = I3C_INTSTS_BUSOWNER_Msk;
    }

    if (g_u32IntSelMask & I3C_INTEN_TX_EMPTY_THLD)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_TX_EMPTY_THLD))
        {
            printf("[ INT ] TX_EMPTY_THLD\n");
            g_u32IntOccurredMask |= I3C_INTSTS_TX_EMPTY_THLD;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_RX_THLD)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_RX_THLD))
        {
            printf("[ INT ] INTSTS_RX_THLD\n");
            g_u32IntOccurredMask |= I3C_INTSTS_RX_THLD;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_CMDQ_EMPTY_THLD)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_CMDQ_EMPTY_THLD))
        {
            printf("[ INT ] CMDQ_EMPTY_THLD\n");
            g_u32IntOccurredMask |= I3C_INTSTS_CMDQ_EMPTY_THLD;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_RESPQ_READY)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_RESPQ_READY))
        {
            printf("[ INT ] RESPQ_READY\n");
            g_u32IntOccurredMask |= I3C_INTSTS_RESPQ_READY;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_CCC_UPDATED)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_CCC_UPDATED))
        {
            printf("[ INT ] CCC_UPDATED\n");
            I3C_CLEAR_CCC_UPDATED_STATUS(I3C0);
            g_u32IntOccurredMask |= I3C_INTSTS_CCC_UPDATED;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_DA_ASSIGNED)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_DA_ASSIGNED))
        {
            printf("[ INT ] DA_ASSIGNED\n");
            I3C_CLEAR_DA_ASSIGNED_STATUS(I3C0);
            g_u32IntOccurredMask |= I3C_INTSTS_DA_ASSIGNED;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_TRANSFER_ERR)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_TRANSFER_ERR))
        {
            printf("[ INT ] TRANSFER_ERR\n");
            I3C_CLEAR_TRANSFER_ERR_STATUS(I3C0);
            g_u32IntOccurredMask |= I3C_INTSTS_TRANSFER_ERR;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_READ_REQUEST)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_READ_REQUEST))
        {
            printf("[ INT ] READ_REQUEST\n");
            I3C_CLEAR_READ_REQUEST_STATUS(I3C0);
            g_u32IntOccurredMask |= I3C_INTSTS_READ_REQUEST;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_IBI_UPDATED)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_IBI_UPDATED))
        {
            printf("[ INT ] IBI_UPDATED\n");
            I3C_CLEAR_IBI_UPDATED_STATUS(I3C0);
            g_u32IntOccurredMask |= I3C_INTSTS_IBI_UPDATED;
        }
    }

    if (g_u32IntOccurredMask & I3C_INTSTS_RESPQ_READY)
    {
        g_u32RespStatus = (uint32_t)I3C_ParseRespQueue(I3C0, (uint32_t *)(&g_RespQ[0]));
    }

    printf("[ INT EXIT ] INTSTS: 0x%08x. Occurred: 0x%08x.\n\n", I3C0->INTSTS, g_u32IntOccurredMask);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint16_t    i;
    uint32_t    u32Item;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("Date, Time : %s, %s\n", __DATE__, __TIME__);
    printf("+-------------------------------------------------------+\n");
    printf("|       I3C0 Secondary Master Sample Code               |\n");
    printf("|                                                       |\n");
    printf("|  I3C Master (I3C0) <---> I3C Slave(I3C0)              |\n");
    printf("+-------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                   |\n");
    printf("|  [0] Master    [1] Slave                              |\n");
    printf("+-------------------------------------------------------+\n\n");
    u32Item = (uint32_t)getchar();

    if (u32Item == '0')
    {
        printf(" - Secondary Master is Configured to Master\n");
        /* Initial I3C0 as Master */
        I3C_Open(I3C0, I3C_MASTER, I3C_MASTER_DA, 0);
        /* Enable I3C0 controller */
        I3C_Enable(I3C0);
        /* Dynamic Address for Enter Dynamic Address Assignment (ENTDAA) */
        I3C_SetDeviceAddr(I3C0, 0, I3C_DEVTYPE_I3C, I3C_SLAVE_DA, 0x00);
        goto Master_Process;
    }
    else
    {
        printf(" - Secondary Master is Configured to Slave\n");
        printf(" - Waiting for I3C Master to assign DA and then Press any key to change I3C role\n\n");
        /* Initial I3C0 as Slave */
        I3C0->SLVMID = I3C0_MID;
        I3C0->SLVPID = I3C0_PID;
        I3C_Open(I3C0, I3C_SLAVE, I3C_SLAVE_SA, I3C_SUPPORT_ENTDAA);
        /* Enable I3C0 interrupts */
        g_u32IntSelMask = (I3C_INTEN_RESPQ_READY | I3C_INTEN_CCC_UPDATED | I3C_INTEN_DA_ASSIGNED |
                           I3C_INTEN_TRANSFER_ERR | I3C_INTEN_READ_REQUEST | I3C_INTEN_IBI_UPDATED |
                           I3C_INTEN_BUSOWNER_Msk);
        I3C_ENABLE_INT(I3C0, g_u32IntSelMask);
        NVIC_EnableIRQ(I3C0_IRQn);
        I3C_Enable(I3C0);
        /* Added Master DA to Device Address Table, to issue GETACCMST later */
        I3C_SetDeviceAddr(I3C0, 0, I3C_DEVTYPE_I3C, I3C_MASTER_DA, 0x00);
        /* Enable I3C0 controller */
        goto Slave_Process;
    }

Master_Process:

    while (1)
    {
        printf("press any key to broadcast ENTDAA\n");
        getchar();

        if (1 == I3C_BroadcastENTDAA(I3C0, 1))
        {
            printf("I3C Device found:\n");
            printf(" - Provisional ID = 0x%08X%02X\n", I3C0->DEV1CH[0], I3C0->DEV1CH[1]);
            printf(" - BCR, DCR = 0x%08X\n", I3C0->DEV1CH[2]);
            printf(" - DADR = 0x%08X\n\n", I3C0->DEV1CH[3]);
            break;
        }
    }

    while (1)
    {
        /* Accept MR request from Slave and Issue GETACCMST */
        if (I3C0->INTSTS & I3C_INTSTS_IBITH_Msk)
        {
            uint32_t    response, cnt, data;
            response = I3C0->IBISTS;
            cnt = (response & I3C_IBISTS_DATLEN_Msk);
            cnt = (cnt + 3) / 4;

            for (i = 0; i < cnt; i++)
            {
                data = I3C0->IBIQUE;
                printf("IBI Data[%d] = 0x%08X\n", i, data);
            }

            if ((response & I3C_IBISTS_IBIID_Msk) == ((I3C_SLAVE_DA << 1) << I3C_IBISTS_IBIID_Pos))
            {
                if (I3C_STS_NO_ERR != I3C_UnicastGETACCMST(I3C0, 0, NULL))
                {
                    printf("Send GETACCMST FAIL\n");
                }
            }
        }

        if (I3C0->INTSTS & I3C_INTSTS_BUSOWNER_Msk)
        {
            I3C0->DEVCTL |= I3C_DEVCTL_RESUME_Msk;

            while ((I3C0->DEVCTL & I3C_DEVCTL_RESUME_Msk) == I3C_DEVCTL_RESUME_Msk) {}

            I3C0->INTSTS = I3C_INTSTS_BUSOWNER_Msk;

            if (I3C_IS_MASTER(I3C0))
            {
                printf("I3C role change from Slave to Master\n");
            }
            else
            {
                printf("I3C role change from Master to Slave\n");
            }
        }
    }

Slave_Process:

    while (1)
    {
        PollingToSwitchRole();
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
