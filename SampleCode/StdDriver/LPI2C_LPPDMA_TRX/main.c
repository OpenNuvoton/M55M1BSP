/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate LPI2C LPPDMA mode and need two boards to connect LPI2C0(Master) and LPI2C0(Slave).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LPI2C0_LPPDMA_TX_CH_MASTER    0
#define LPI2C0_LPPDMA_RX_CH_SLAVE     1
#define LPI2C0_LPPDMA_RX_CH_MASTER    2
#define LPI2C0_LPPDMA_TX_CH_SLAVE     3
#define LPPDMA_TEST_LENGTH    5

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* Base address and size of cache buffer must be DCACHE_LINE_SIZE byte aligned */
NVT_NONCACHEABLE uint8_t g_u8MasterTx_Buffer[LPPDMA_TEST_LENGTH];
NVT_NONCACHEABLE uint8_t g_u8MasterRx_Buffer[LPPDMA_TEST_LENGTH];
NVT_NONCACHEABLE uint8_t g_u8SlaveTx_Buffer[LPPDMA_TEST_LENGTH];
NVT_NONCACHEABLE uint8_t g_u8SlaveRx_Buffer[LPPDMA_TEST_LENGTH];

volatile uint32_t LPPDMA_DONE = 0;

volatile uint8_t g_u8DeviceAddr = 0x16;
volatile uint8_t g_u8MasterDataLen = 0;
volatile uint8_t g_u8SlaveDataLen = 0;
volatile uint16_t g_u8SlaveBufferAddr = 0;

typedef void (*LPI2C_FUNC)(uint32_t u32Status);

volatile static LPI2C_FUNC s_LPI2C0HandlerFn = NULL;


NVT_ITCM void LPPDMA_IRQHandler(void)
{
    uint32_t u32Status = LPPDMA->TDSTS;

    //Master TX
    if (u32Status & (0x1 << LPI2C0_LPPDMA_TX_CH_MASTER))
    {
        printf(" LPI2C0 Tx done\n");
        LPPDMA->TDSTS = 0x1 << LPI2C0_LPPDMA_TX_CH_MASTER;
        LPPDMA_DONE = 1;
    }

    //Master RX
    if (u32Status & (0x1 << LPI2C0_LPPDMA_RX_CH_MASTER))
    {
        printf(" LPI2C0 Rx done\n");
        LPPDMA->TDSTS = 0x1 << LPI2C0_LPPDMA_RX_CH_MASTER;
        LPPDMA_DONE = 1;
    }

    //Slave RX
    if (u32Status & (0x1 << LPI2C0_LPPDMA_RX_CH_SLAVE))
    {
        printf(" LPI2C0 Rx done\n");
        LPPDMA->TDSTS = 0x1 << LPI2C0_LPPDMA_RX_CH_SLAVE;
        LPPDMA_DONE = 1;
    }

    //Slave TX
    if (u32Status & (0x1 << LPI2C0_LPPDMA_TX_CH_SLAVE))
    {
        printf(" LPI2C0 Tx done\n");
        LPPDMA->TDSTS = 0x1 << LPI2C0_LPPDMA_TX_CH_SLAVE;
    }

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    u32Status = LPPDMA->TDSTS;
}


NVT_ITCM void LPI2C0_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = LPI2C_GET_STATUS(LPI2C0);

    if (LPI2C_GET_TIMEOUT_FLAG(LPI2C0))
    {
        /* Clear LPI2C0 Timeout Flag */
        LPI2C_ClearTimeoutFlag(LPI2C0);
    }
    else
    {
        if (s_LPI2C0HandlerFn != NULL)
        {
            s_LPI2C0HandlerFn(u32Status);
        }
    }

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    u32Status = LPI2C_GET_STATUS(LPI2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C0 LPPDMA Slave Rx Callback Function                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_LPPDMA_SlaveRx(uint32_t u32Status)
{
    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        /*
            Note:
            During LPPDMA operation, LPI2C controller will not occur receive Address ACK interrupt
        */
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                  Data has been received; ACK has been returned*/
    {
        /*
            Note:
            During LPPDMA operation, LPI2C controller will not occur receive Data ACK interrupt
        */
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);

        while (1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C0 LPPDMA Slave Tx Callback Function                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_LPPDMA_SlaveTx(uint32_t u32Status)
{
    uint8_t u8data;

    if (u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlaveDataLen = 0;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8data = (unsigned char) LPI2C_GET_DATA(LPI2C0);
        g_u8SlaveRx_Buffer[g_u8SlaveDataLen++] = u8data;
        g_u8SlaveBufferAddr = (g_u8SlaveRx_Buffer[0] << 8) + g_u8SlaveRx_Buffer[1];

        if (g_u8SlaveDataLen == 2)
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
        }
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
        /* Enable LPI2C0 Slave TX */
        LPI2C0->CTL1 = LPI2C_CTL1_TXPDMAEN_Msk;
    }
    else if (u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0xB8)                  /* Data byte in LPI2CDAT has been transmitted ACK has been received */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in LPI2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);

        while (1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C0 LPPDMA Master Tx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_LPPDMA_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted */
    {
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur address ACK interrupt
        */
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        LPI2C_STOP(LPI2C0);
        LPI2C_START(LPI2C0);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur data ACK interrupt
        */
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C0 LPPDMA Master Rx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_LPPDMA_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08)                         /* START has been transmitted and prepare SLA+W */
    {
        LPI2C_SET_DATA(LPI2C0, (g_u8DeviceAddr << 1) | 0x00);     /* Write SLA+W to Register I2CDAT */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x18)                    /* SLA+W has been transmitted and ACK has been received */
    {
        g_u8MasterDataLen = 1;
        LPI2C_SET_DATA(LPI2C0, g_u8MasterTx_Buffer[g_u8MasterDataLen++]);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x20)                    /* SLA+W has been transmitted and NACK has been received */
    {
        LPI2C_STOP(LPI2C0);
        LPI2C_START(LPI2C0);
    }
    else if (u32Status == 0x28)                    /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MasterDataLen <= 2)
        {
            LPI2C_SET_DATA(LPI2C0, g_u8MasterTx_Buffer[g_u8MasterDataLen++]);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA_SI);
        }
    }
    else if (u32Status == 0x10)                   /* Repeat START has been transmitted and prepare SLA+R */
    {
        LPI2C_SET_DATA(LPI2C0, (g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x40)                   /* SLA+R has been transmitted and ACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
        /* Enable LPI2C0 Master RX */
        LPI2C0->CTL1 = LPI2C_CTL1_RXPDMAEN_Msk;
    }
    else if (u32Status == 0x50)                   /* DATA has been received and ACK has been returned */
    {
        /*
           Note:
           During LPPDMA operation, LPI2C controller will not occur receive data ACK interrupt
        */
    }
    else if (u32Status == 0x58)                   /* DATA has been received and NACK has been returned */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO | LPI2C_CTL_SI);
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
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
    /* Enable LPPDMA module clock */
    CLK_EnableModuleClock(LPPDMA0_MODULE);
    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(LPI2C0_MODULE);
    /* Set multi-function pins for LPI2C0 SDA and SCL */
    SET_LPI2C0_SDA_PB4();
    SET_LPI2C0_SCL_PB5();
    /* Lock protected registers */
    SYS_LockReg();
}

void LPI2C_Master_Init(void)
{
    /* Open LPI2C0 module and set bus clock */
    LPI2C_Open(LPI2C0, 100000);
    /* Get LPI2C0 Bus Clock */
    printf("Master LPI2C0 clock %d Hz\n", LPI2C_GetBusClockFreq(LPI2C0));
    /* Enable LPI2C0 interrupt */
    LPI2C_EnableInt(LPI2C0);
    NVIC_EnableIRQ(LPI2C0_IRQn);
}

void LPI2C_Slave_Init(void)
{
    /* Open LPI2C0 module and set bus clock */
    LPI2C_Open(LPI2C0, 100000);
    /* Get LPI2C0 Bus Clock */
    printf("Slave LPI2C0 clock %d Hz\n", LPI2C_GetBusClockFreq(LPI2C0));
    /* Set LPI2C0 4 Slave Addresses */
    LPI2C_SetSlaveAddr(LPI2C0, 0, 0x16, 0);   /* Slave Address : 0x16 */
    LPI2C_SetSlaveAddr(LPI2C0, 1, 0x36, 0);   /* Slave Address : 0x36 */
    LPI2C_SetSlaveAddr(LPI2C0, 2, 0x56, 0);   /* Slave Address : 0x56 */
    LPI2C_SetSlaveAddr(LPI2C0, 3, 0x76, 0);   /* Slave Address : 0x76 */
    /* Set LPI2C0 4 Slave Addresses Mask */
    LPI2C_SetSlaveAddrMask(LPI2C0, 0, 0x04);
    LPI2C_SetSlaveAddrMask(LPI2C0, 1, 0x02);
    LPI2C_SetSlaveAddrMask(LPI2C0, 2, 0x04);
    LPI2C_SetSlaveAddrMask(LPI2C0, 3, 0x02);
    /* Enable LPI2C0 interrupt */
    LPI2C_EnableInt(LPI2C0);
    NVIC_EnableIRQ(LPI2C0_IRQn);
}

void LPPDMA_Master_Init(void)
{
    /* Open PDMA Channel */
    LPPDMA_Open(LPPDMA, 1 << LPI2C0_LPPDMA_TX_CH_MASTER); // Channel 0 for LPI2C0 TX
    LPPDMA_Open(LPPDMA, 1 << LPI2C0_LPPDMA_RX_CH_MASTER); // Channel 2 for LPI2C0 RX
    // Select basic mode
    LPPDMA_SetTransferMode(LPPDMA, LPI2C0_LPPDMA_TX_CH_MASTER, LPPDMA_LPI2C0_TX, 0, 0);
    LPPDMA_SetTransferMode(LPPDMA, LPI2C0_LPPDMA_RX_CH_MASTER, LPPDMA_LPI2C0_RX, 0, 0);
    // Set data width and transfer count
    LPPDMA_SetTransferCnt(LPPDMA, LPI2C0_LPPDMA_TX_CH_MASTER, LPPDMA_WIDTH_8, LPPDMA_TEST_LENGTH);
    LPPDMA_SetTransferCnt(LPPDMA, LPI2C0_LPPDMA_RX_CH_MASTER, LPPDMA_WIDTH_8, LPPDMA_TEST_LENGTH - 3); // except Slave Address and two bytes Data Address
    //Set PDMA Transfer Address
    LPPDMA_SetTransferAddr(LPPDMA, LPI2C0_LPPDMA_TX_CH_MASTER, ((uint32_t)(&g_u8MasterTx_Buffer[0])), LPPDMA_SAR_INC, (uint32_t)(&(LPI2C0->DAT)), LPPDMA_DAR_FIX);
    LPPDMA_SetTransferAddr(LPPDMA, LPI2C0_LPPDMA_RX_CH_MASTER, (uint32_t)(&(LPI2C0->DAT)), LPPDMA_SAR_FIX, ((uint32_t)(&g_u8MasterRx_Buffer[0])), LPPDMA_DAR_INC);
    //Select Single Request
    LPPDMA_SetBurstType(LPPDMA, LPI2C0_LPPDMA_TX_CH_MASTER, LPPDMA_REQ_SINGLE, 0);
    LPPDMA_SetBurstType(LPPDMA, LPI2C0_LPPDMA_RX_CH_MASTER, LPPDMA_REQ_SINGLE, 0);
    LPPDMA_EnableInt(LPPDMA, LPI2C0_LPPDMA_TX_CH_MASTER, LPPDMA_INT_TRANS_DONE);
    LPPDMA_EnableInt(LPPDMA, LPI2C0_LPPDMA_RX_CH_MASTER, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA_IRQn);
}

void LPPDMA_Slave_Init(void)
{
    /* Open PDMA Channel */
    LPPDMA_Open(LPPDMA, 1 << LPI2C0_LPPDMA_RX_CH_SLAVE); // Channel 1 for LPI2C0 RX
    LPPDMA_Open(LPPDMA, 1 << LPI2C0_LPPDMA_TX_CH_SLAVE); // Channel 3 for LPI2C0 TX
    // Select basic mode
    LPPDMA_SetTransferMode(LPPDMA, LPI2C0_LPPDMA_RX_CH_SLAVE, LPPDMA_LPI2C0_RX, 0, 0);
    LPPDMA_SetTransferMode(LPPDMA, LPI2C0_LPPDMA_TX_CH_SLAVE, LPPDMA_LPI2C0_TX, 0, 0);
    // Set data width and transfer count
    LPPDMA_SetTransferCnt(LPPDMA, LPI2C0_LPPDMA_RX_CH_SLAVE, LPPDMA_WIDTH_8, LPPDMA_TEST_LENGTH);
    LPPDMA_SetTransferCnt(LPPDMA, LPI2C0_LPPDMA_TX_CH_SLAVE, LPPDMA_WIDTH_8, LPPDMA_TEST_LENGTH - 3); // except Slave Address and two bytes Data Address
    //Set PDMA Transfer Address
    LPPDMA_SetTransferAddr(LPPDMA, LPI2C0_LPPDMA_RX_CH_SLAVE, (uint32_t)(&(LPI2C0->DAT)), LPPDMA_SAR_FIX, ((uint32_t)(&g_u8SlaveRx_Buffer[0])), LPPDMA_DAR_INC);
    LPPDMA_SetTransferAddr(LPPDMA, LPI2C0_LPPDMA_TX_CH_SLAVE, ((uint32_t)(&g_u8SlaveTx_Buffer[0])), LPPDMA_SAR_INC, (uint32_t)(&(LPI2C0->DAT)), LPPDMA_DAR_FIX);
    //Select Single Request
    LPPDMA_SetBurstType(LPPDMA, LPI2C0_LPPDMA_RX_CH_SLAVE, LPPDMA_REQ_SINGLE, 0);
    LPPDMA_SetBurstType(LPPDMA, LPI2C0_LPPDMA_TX_CH_SLAVE, LPPDMA_REQ_SINGLE, 0);
    LPPDMA_EnableInt(LPPDMA, LPI2C0_LPPDMA_RX_CH_SLAVE, LPPDMA_INT_TRANS_DONE);
    LPPDMA_EnableInt(LPPDMA, LPI2C0_LPPDMA_TX_CH_SLAVE, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA_IRQn);
}

void LPI2C_LPPDMA_Master(void)
{
    uint32_t i, u32TimeOutCnt;

    for (i = 0; i < LPPDMA_TEST_LENGTH; i++)
    {
        g_u8MasterTx_Buffer[i] = i;
        g_u8MasterRx_Buffer[i] = 0xff;
    }

    g_u8MasterTx_Buffer[0] = ((g_u8DeviceAddr << 1) | 0x00);   //1 byte SLV + W
    g_u8MasterTx_Buffer[1] = 0x00;                             //2 bytes Data address
    g_u8MasterTx_Buffer[2] = 0x00;
    LPPDMA_Master_Init();
    /* Enable I2C TX */
    LPI2C0->CTL1 = LPI2C_CTL1_TXPDMAEN_Msk;
    s_LPI2C0HandlerFn = (LPI2C_FUNC)LPI2C_LPPDMA_MasterTx;
    LPPDMA_DONE = 0;
    /* Send START condition, start the LPPDMA data transmit */
    LPI2C_START(LPI2C0);
    u32TimeOutCnt = LPI2C_TIMEOUT;

    while (!LPPDMA_DONE)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait MasterTx done time-out!\n");

            while (1);
        }
    }

    printf("Wait slave receive data done and then press any key to start master LPPDMA RX mode.\n\n");
    getchar();
    /* Disable LPI2C0 LPPDMA TX mode */
    LPI2C0->CTL1 &= ~LPI2C_CTL1_TXPDMAEN_Msk;
    /* Test Master RX with LPPDMA function */
    /* LPI2C0 function to Master receive data */
    s_LPI2C0HandlerFn = (LPI2C_FUNC)LPI2C_LPPDMA_MasterRx;
    LPPDMA_DONE = 0;
    /* Send START condition */
    LPI2C_START(LPI2C0);
    u32TimeOutCnt = LPI2C_TIMEOUT;

    while (!LPPDMA_DONE)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait MasterRx done time-out!\n");

            while (1);
        }
    }

    /* Disable LPI2C0 LPPDMA RX mode */
    LPI2C0->CTL1 &= ~LPI2C_CTL1_RXPDMAEN_Msk;

    for (i = 0; i < LPPDMA_TEST_LENGTH - 3; i++)
    {
        if (g_u8MasterRx_Buffer[i] != g_u8MasterTx_Buffer[i + 3])
        {
            printf(" Slave Receive Data Compare Error !!\n");

            while (1);
        }
    }

    printf("LPI2C LPPDMA test Pass.\n");
}

void LPI2C_LPPDMA_Slave(void)
{
    uint32_t i;

    for (i = 0; i < LPPDMA_TEST_LENGTH; i++)
    {
        g_u8MasterTx_Buffer[i] = i;
        g_u8SlaveRx_Buffer[i] = 0xff;
    }

    g_u8MasterTx_Buffer[0] = ((g_u8DeviceAddr << 1) | 0x00);   //1 byte SLV + W
    g_u8MasterTx_Buffer[1] = 0x00;                             //2 bytes Data address
    g_u8MasterTx_Buffer[2] = 0x00;
    LPPDMA_DONE = 0;
    LPPDMA_Slave_Init();
    /* LPI2C enter no address SLV mode */
    LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI | LPI2C_CTL_AA);
    /* Enable LPI2C0 LPPDMA RX mode */
    LPI2C0->CTL1 = LPI2C_CTL1_RXPDMAEN_Msk;
    /* LPI2C0 function to Slave receive data */
    s_LPI2C0HandlerFn = LPI2C_LPPDMA_SlaveRx;

    while (!LPPDMA_DONE);

    printf("LPI2C slave receive data done.\n\n");
    /* Disable LPI2C0 LPPDMA RX mode */
    LPI2C0->CTL1 &= ~LPI2C_CTL1_RXPDMAEN_Msk;

    for (i = 0; i < LPPDMA_TEST_LENGTH; i++)
    {
        if (g_u8SlaveRx_Buffer[i] != g_u8MasterTx_Buffer[i])
        {
            printf(" Receive Data Compare Error !!\n");
            uint32_t j = 0;

            for (j = 0; j < LPPDMA_TEST_LENGTH; j++)
            {
                printf("(0x%02X, 0x%02X) ", g_u8SlaveRx_Buffer[j], g_u8MasterTx_Buffer[j]);
            }

            printf("\n");

            while (1);
        }
        else
        {
            if (i > 2)
            {
                g_u8SlaveTx_Buffer[i - 3] = g_u8MasterTx_Buffer[i];
            }
        }
    }

    /* Test Slave TX with LPPDMA function */
    printf("LPI2C slave wait to transmit data.\n");
    /* LPI2C0 function to Slave transmit data */
    s_LPI2C0HandlerFn = LPI2C_LPPDMA_SlaveTx;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32Item;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+-------------------------------------------------------+\n");
    printf("|       LPI2C Driver Sample Code for LPPDMA             |\n");
    printf("|                                                       |\n");
    printf("|  LPI2C Master (LPI2C0) <---> LPI2C Slave(LPI2C0)      |\n");
    printf("+-------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                   |\n");
    printf("|  [0] Master    [1] Slave                              |\n");
    printf("+-------------------------------------------------------+\n\n");
    u32Item = (uint32_t)getchar();

    if (u32Item == '0')
    {
        /* Init LPI2C0 Master */
        LPI2C_Master_Init();
        LPI2C_LPPDMA_Master();
    }
    else
    {
        /* Init LPI2C0 Slave */
        LPI2C_Slave_Init();
        LPI2C_LPPDMA_Slave();
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
