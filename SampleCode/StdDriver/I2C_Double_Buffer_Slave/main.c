/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief
 *          Demonstrate how to set I2C two-level buffer in Slave mode to receive 256 bytes data from a master.
 *          This sample code needs to work with I2C_MultiBytes_Master.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_LENGTH    256

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[TEST_LENGTH + 1];
volatile uint8_t g_au8SlvRxData[3];

volatile uint8_t g_u8SlvDataLen;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void I2C0_IRQHandler(void)
{
    uint32_t u32Status, temp;
    uint8_t u8data;

    if ((I2C0->STATUS1 & I2C_STATUS1_SARCIF_Msk))
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[slave_buff_addr++]);    /* Prepare first data */
    }

    /* Wait SI flag is set */
    I2C_WAIT_READY(I2C0);
    u32Status = I2C_GET_STATUS(I2C0);
    I2C_CLR_STATUS1_FLAG(I2C0, I2C_STATUS1_DPCIF_Msk | I2C_STATUS1_SARCIF_Msk);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    if (u32Status == 0xA0)                                      /* STOP or Repeat START has been received */
    {
        g_u8SlvDataLen = 0;
    }
    else if (u32Status == 0xA8)                                 /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[slave_buff_addr++]);    /* Prepare second data */
    }
    else if (u32Status == 0x80)                                  /* Previously address with own SLA address
                                                                    Data has been received; ACK has been returned*/
    {
        u8data = (unsigned char) I2C_GET_DATA(I2C0);

        if (g_u8SlvDataLen < 2)
        {
            g_au8SlvRxData[g_u8SlvDataLen++] = u8data;
            temp = (g_au8SlvRxData[0] << 8);
            temp += g_au8SlvRxData[1];
            slave_buff_addr =  temp;
        }
        else
        {
            g_au8SlvData[slave_buff_addr++] = u8data;

            if (slave_buff_addr == TEST_LENGTH)
            {
                slave_buff_addr = 0;
            }
        }
    }
    else if (u32Status == 0xB8)                                 /* Data byte in I2CDAT has been transmitted ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[slave_buff_addr++]);    /* Prepare third and other data */
    }
    else if (u32Status == 0xC0)                                  /* Data byte or last data in I2CDAT has been transmitted
                                                                    Not ACK has been received */
    {
        slave_buff_addr--;
    }
    else if (u32Status == 0x60)                                 /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    u32Status = I2C_GET_STATUS(I2C0);
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);
    /* Switch SCLK clock source to PLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);
    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);
    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    /* Set multi-function pins for I2C0 SDA and SCL */
    SET_I2C0_SDA_PB4();
    SET_I2C0_SCL_PB5();
    /* I2C pins enable schmitt trigger */
    CLK_EnableModuleClock(GPIOB_MODULE);
    GPIO_ENABLE_SCHMITT_TRIGGER(PB, (BIT4 | BIT5));
    /* Lock protected registers */
    SYS_LockReg();
}

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);
    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
    /* Set I2C0 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, I2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, I2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, I2C_GCMODE_DISABLE);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, I2C_GCMODE_DISABLE);   /* Slave Address : 0x75 */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);
    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Multi Bytes Write
        and Multi Bytes Read operations, and check if the read data is equal to the programmed data.
    */
    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code(Slave) for access double buffer |\n");
    printf("|  Needs to work with I2C_MultiBytes_Master sample code. |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)                |\n");
    printf("| !! This sample code requires two borads to test !!     |\n");
    printf("+--------------------------------------------------------+\n");
    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5)\n");
    /* Init I2C0 */
    I2C0_Init();
    /* Enable I2C two buffer mode and set data phase bit count to 6 bit */
    I2C_EnableTwoBufferMode(I2C0, I2C_DATA_PHASE_BIT_6);
    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    for (i = 0; i < TEST_LENGTH; i++)
    {
        g_au8SlvData[i] = 0;
    }

    printf("\n");
    printf("I2C Slave Mode is Running.\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
