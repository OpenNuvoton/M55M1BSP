/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Transmit and receive UART data with PDMA.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define UART_RX_DMA_CH 0
#define UART_TX_DMA_CH 1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t UART_TEST_LENGTH = 64;

#if (NVT_DCACHE_ON == 1)
    /* Base address and size of cache buffer must be DCACHE_LINE_SIZE byte aligned */
    uint8_t SrcArray[DCACHE_ALIGN_LINE_SIZE(64)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint8_t DestArray[DCACHE_ALIGN_LINE_SIZE(64)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    uint8_t SrcArray[64] __attribute__((aligned));
    uint8_t DestArray[64] __attribute__((aligned));
#endif

volatile int32_t IntCnt;
volatile int32_t IsTestOver;
volatile uint32_t g_u32TwoChannelPdmaTest = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Clear buffer funcion                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void ClearBuf(uint32_t u32Addr, uint32_t u32Length, uint8_t u8Pattern)
{
    uint8_t *pu8Ptr;
    uint32_t i;

    pu8Ptr = (uint8_t *)u32Addr;

    for (i = 0; i < u32Length; i++)
    {
        *pu8Ptr++ = u8Pattern;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Bulid Src Pattern function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length)
{
    uint32_t i = 0, j, loop;
    uint8_t *pAddr;

    pAddr = (uint8_t *)u32Addr;

    do
    {
        if (u32Length > 256)    /* Pattern from 0 ~ 255 */
            loop = 256;
        else
            loop = u32Length;

        u32Length = u32Length - loop;

        for (j = 0; j < loop; j++)
            *pAddr++ = (uint8_t)(j + i);

        i++;
    } while ((loop != 0) || (u32Length != 0));
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Tx PDMA0 Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_UART_TxTest(void)
{
    /* UART Tx PDMA0 channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, UART_TX_DMA_CH, PDMA_WIDTH_8, UART_TEST_LENGTH);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, UART_TX_DMA_CH, (uint32_t)SrcArray, PDMA_SAR_INC, (uint32_t)&UART1->DAT, PDMA_DAR_FIX);

    /* Single request type */
    PDMA_SetBurstType(PDMA0, UART_TX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, UART_TX_DMA_CH, PDMA_UART1_TX, FALSE, 0);

    /* Disable table interrupt */
    PDMA_DisableInt(PDMA0, UART_TX_DMA_CH, PDMA_INT_TEMPTY);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Rx PDMA0 Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_UART_RxTest(void)
{
    /* UART Rx PDMA0 channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, UART_RX_DMA_CH, PDMA_WIDTH_8, UART_TEST_LENGTH);
#if (NVT_DCACHE_ON == 1)
    /* Clean the data cache for the destination buffer of UART PDMA RX channel */
    SCB_InvalidateDCache_by_Addr(&DestArray, sizeof(DestArray));
#endif
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, UART_RX_DMA_CH, (uint32_t)&UART1->DAT, PDMA_SAR_FIX, (uint32_t)DestArray, PDMA_DAR_INC);

    /* Single request type */
    PDMA_SetBurstType(PDMA0, UART_RX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, UART_RX_DMA_CH, PDMA_UART1_RX, FALSE, 0);

    /* Disable table interrupt */
    PDMA_DisableInt(PDMA0, UART_RX_DMA_CH, PDMA_INT_TEMPTY);
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_Callback_0(void)
{
    printf("\tTransfer Done %d!\r", ++IntCnt);

    /* Use PDMA0 to do UART loopback test 10 times */
    if (IntCnt < 10)
    {
        /* UART Tx and Rx PDMA0 configuration */
        PDMA0_UART_TxTest();
        PDMA0_UART_RxTest();

        /* Enable UART Tx and Rx PDMA0 function */
        UART_ENABLE_INT(UART1, (UART_INTEN_RXPDMAEN_Msk | UART_INTEN_TXPDMAEN_Msk));
    }
    else
    {
        /* Test is over */
        IsTestOver = TRUE;
    }
}

void PDMA0_Callback_1(void)
{
    int32_t i ;

    printf("\tTransfer Done %d!\t", ++IntCnt);

    /* Show UART Rx data */
    for (i = 0; i < UART_TEST_LENGTH; i++)
        printf(" 0x%x(%c),", DestArray[i], DestArray[i]);

    printf("\n");

    /* Use PDMA0 to do UART Rx test 10 times */
    if (IntCnt < 10)
    {
        /* UART Rx PDMA0 configuration */
        PDMA0_UART_RxTest();

        /* Enable UART Rx PDMA0 function */
        UART_ENABLE_INT(UART1, UART_INTEN_RXPDMAEN_Msk);
    }
    else
    {
        /* Test is over */
        IsTestOver = TRUE;
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle PDMA0 interrupt event                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void PDMA0_IRQHandler(void)
{
    /* Get PDMA0 interrupt status */
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if (status & PDMA_INTSTS_ABTIF_Msk)  /* Target Abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF2_Msk)
            IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_GET_ABORT_STS(PDMA0));
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)    /* Transfer Done */
    {
        /* UART Tx PDMA0 transfer done interrupt flag */
        if (PDMA_GET_TD_STS(PDMA0) & (1 << UART_TX_DMA_CH))
        {
            /* Clear PDMA0 transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA0, (1 << UART_TX_DMA_CH));

            /* Disable UART Tx PDMA0 function */
            UART_DISABLE_INT(UART1, UART_INTEN_TXPDMAEN_Msk);
        }

        /* UART Rx PDMA0 transfer done interrupt flag */
        if (PDMA_GET_TD_STS(PDMA0) & (1 << UART_RX_DMA_CH))
        {
            /* Clear PDMA0 transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA0, (1 << UART_RX_DMA_CH));

            /* Disable UART Rx PDMA0 function */
            UART_DISABLE_INT(UART1, UART_INTEN_RXPDMAEN_Msk);

            /* Handle PDMA0 transfer done interrupt event */
            if (g_u32TwoChannelPdmaTest == 1)
            {
                PDMA0_Callback_0();
            }
            else if (g_u32TwoChannelPdmaTest == 0)
            {
                PDMA0_Callback_1();
            }
        }
    }
    else
    {
        printf("unknown interrupt, status=0x%x !!\n", status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void DEBUG_PORT_IRQHandler(void)
{
    /* Get DEBUG_PORT Rx data and send the data to UART1 Tx */
    if (UART_GET_INT_FLAG(DEBUG_PORT, UART_INTSTS_RDAIF_Msk))
        UART_WRITE(UART1, UART_READ(DEBUG_PORT));
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Sample Code:                                                                                       */
/*         i32option : ['1'] UART1 TX/RX PDMA Loopback                                                     */
/*                     [Others] UART1 RX PDMA test                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART(int32_t i32option)
{
#if (NVT_DCACHE_ON == 1)
    /* If DCACHE is enabled, clean the data cache for the two buffers before writing to them and enabling DMA */
    /* This is to ensure that the data written to the cache is actually written to the memory */
    SCB_CleanDCache_by_Addr((uint32_t *)&SrcArray, sizeof(SrcArray));
    SCB_CleanDCache_by_Addr((uint32_t *)&DestArray, sizeof(DestArray));
#endif

    /* Source data initiation */
    BuildSrcPattern((uint32_t)SrcArray, UART_TEST_LENGTH);
    ClearBuf((uint32_t)DestArray, UART_TEST_LENGTH, 0xFF);
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Reset PDMA0 module */
    SYS_ResetModule(SYS_PDMA0RST);
    /* Lock protected registers */
    SYS_LockReg();

    if (i32option == '1')
    {
        printf("  [Using TWO PDMA0 channel].\n");
        printf("  This sample code will use PDMA0 to do UART1 loopback test 10 times.\n");
        printf("  Please connect UART1_RXD(PB.2) <--> UART1_TXD(PB.3) before testing.\n");
        printf("  After connecting PB.2 <--> PB.3, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 1;
        getchar();
    }
    else
    {
        UART_TEST_LENGTH = 2;      /* Test Length */
        printf("  [Using ONE PDMA0 channel].\n");
        printf("  This sample code will use PDMA0 to do UART1 Rx test 10 times.\n");
        printf("  Please connect UART1_RXD(PB.2) <--> UART1_TXD(PB.3) before testing.\n");
        printf("  After connecting PB.2 <--> PB.3, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 0;
        getchar();
        printf("  Please input %d bytes to trigger PDMA0 one time.(Ex: Press 'a''b')\n", UART_TEST_LENGTH);
    }

    if (g_u32TwoChannelPdmaTest == 1)
    {
        /* Enable PDMA0 channel */
        PDMA_Open(PDMA0, (1 << UART_RX_DMA_CH) | (1 << UART_TX_DMA_CH));

        /* UART Tx and Rx PDMA0 configuration */
        PDMA0_UART_TxTest();
        PDMA0_UART_RxTest();

        /* Enable PDMA0 Transfer Done Interrupt */
        PDMA_EnableInt(PDMA0, UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
        PDMA_EnableInt(PDMA0, UART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    }
    else
    {
        /* Enable PDMA0 channel */
        PDMA_Open(PDMA0, (1 << UART_RX_DMA_CH));

        /* UART Rx PDMA0 configuration */
        PDMA0_UART_RxTest();

        /* Enable PDMA0 Transfer Done Interrupt */
        PDMA_EnableInt(PDMA0, UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    }

    /* Enable PDMA0 Transfer Done Interrupt */
    IntCnt = 0;
    IsTestOver = FALSE;
    NVIC_EnableIRQ(PDMA0_IRQn);

    /* Enable DEBUG_PORT RDA interrupt */
    if (g_u32TwoChannelPdmaTest == 0)
    {
        NVIC_EnableIRQ(DEBUG_PORT_IRQn);
        UART_EnableInt(DEBUG_PORT, UART_INTEN_RDAIEN_Msk);
    }

    /* Enable UART Tx and Rx PDMA0 function */
    if (g_u32TwoChannelPdmaTest == 1)
        UART_ENABLE_INT(UART1, UART_INTEN_TXPDMAEN_Msk);
    else
        UART_DISABLE_INT(UART1, UART_INTEN_TXPDMAEN_Msk);

    UART_ENABLE_INT(UART1, UART_INTEN_RXPDMAEN_Msk);

    /* Wait for PDMA0 operation finish */
    while (IsTestOver == FALSE);

    /* Check PDMA0 status */
    if (IsTestOver == 2)
        printf("target abort...\n");

    /* Disable UART Tx and Rx PDMA0 function */
    UART_DISABLE_INT(UART1, (UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk));

    /* Disable PDMA0 channel */
    PDMA_Close(PDMA0);

    /* Disable PDMA0 Interrupt */
    PDMA_DisableInt(PDMA0, UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_DisableInt(PDMA0, UART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Disable DEBUG_PORT RDA interrupt */
    UART_DisableInt(DEBUG_PORT, UART_INTEN_RDAIEN_Msk);
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART1 clock source is HIRC and UART1 module clock divider as 1*/
    CLK_SetModuleClock(UART1_MODULE, CLK_UARTSEL0_UART1SEL_HIRC, CLK_UARTDIV0_UART1DIV(1));

    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PB multi-function pins forUART1 TXD and RXD */
    SET_UART1_RXD_PB2();
    SET_UART1_TXD_PB3();

}



void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS_ResetModule(SYS_UART1RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    uint8_t unItem;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Init UART1 */
    UART1_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART PDMA Sample Program");

    /* UART PDMA sample function */
    do
    {
        printf("\n\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|                      UART PDMA Driver Sample Code                      |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("| [1] Using TWO PDMA channel to test. < TX1(CH1)-->RX1(CH0) >            |\n");
        printf("| [2] Using ONE PDMA channel to test. < TX1-->RX1(CH0) >                 |\n");
        printf("+------------------------------------------------------------------------+\n");
        unItem = getchar();

        IsTestOver = FALSE;

        if ((unItem == '1') || (unItem == '2'))
        {
            PDMA_UART(unItem);
            printf("\n\n  UART PDMA sample code is complete.\n");
        }

    } while (unItem != 27);

    while (1);

}
