/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Configure EBI interface to access SRAM connected on EBI interface.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Variables declaration for PDMA                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
static uint32_t g_u32TransLen = 64;
static volatile uint32_t g_au32SrcArray[64];
static volatile uint32_t g_u32IsTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
extern int32_t SRAM_BS616LV4017(uint32_t u32MaxSize);

int32_t AccessEBIWithPDMA(void);
void PDMA0_IRQHandler(void);
void Configure_EBI_16BIT_Pins(void);
void SYS_Init(void);

/**
 * @brief       DMA IRQ
 * @param       None
 * @return      None
 * @details     The DMA default IRQ, declared in startup_M55M1.c.
 */
NVT_ITCM void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & PDMA_INTSTS_ABTIF_Msk)       /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)  /* done */
    {
        if (PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;

        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt!!\n");

    /* CPU read interrupt flag register to wait write(clear) instruction completement */
    u32Status = PDMA_GET_INT_STATUS(PDMA0);
}

int32_t AccessEBIWithPDMA(void)
{
    uint32_t i;
    uint32_t u32Result0 = 0x5A5A, u32Result1 = 0x5A5A;
    uint32_t u32TimeOutCnt = 0;

    printf("[[ Access EBI with PDMA ]]\n");

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Reset PDMA module */
    SYS_ResetModule(SYS_PDMA0RST);

    for (i = 0; i < 64; i++)
    {
        g_au32SrcArray[i] = 0x76570000 + i;
        u32Result0 += g_au32SrcArray[i];
    }

    /* Open Channel 2 */
    PDMA_Open(PDMA0, (1 << 2));

    //burst size is 4
    PDMA_SetBurstType(PDMA0, 2, PDMA_REQ_BURST, PDMA_BURST_4);

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_32, g_u32TransLen);
    PDMA_SetTransferAddr(PDMA0, 2, (uint32_t)g_au32SrcArray, PDMA_SAR_INC, EBI_BANK0_BASE_ADDR, PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_MEM, FALSE, 0);

    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    g_u32IsTestOver = 0;
    PDMA_Trigger(PDMA0, 2);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA time-out!\n");
            return (-1);
        }
    }

    /* Transfer internal SRAM to EBI SRAM done */

    /* Clear internal SRAM data */
    for (i = 0; i < 64; i++)
    {
        g_au32SrcArray[i] = 0x0;
    }

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_32, g_u32TransLen);
    PDMA_SetTransferAddr(PDMA0, 2, EBI_BANK0_BASE_ADDR, PDMA_SAR_INC, (uint32_t)g_au32SrcArray, PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_MEM, FALSE, 0);

    g_u32IsTestOver = 0;
    PDMA_Trigger(PDMA0, 2);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA time-out!\n");
            return (-1);
        }
    }

    /* Transfer EBI SRAM to internal SRAM done */
    for (i = 0; i < 64; i++)
    {
        u32Result1 += g_au32SrcArray[i];
    }

    if (g_u32IsTestOver == 1)
    {
        if ((u32Result0 == u32Result1) && (u32Result0 != 0x5A5A))
        {
            printf("        PASS (0x%X)\n\n", u32Result0);
        }
        else
        {
            printf("        FAIL - data matched (0x%X)\n\n", u32Result0);
            return (-1);
        }
    }
    else
    {
        printf("        PDMA fail\n\n");
        return (-1);
    }

    PDMA_Close(PDMA0);

    return 0;
}

void Configure_EBI_16BIT_Pins(void)
{
    /* AD0 ~ AD15*/
    SET_EBI_AD0_PC0();
    SET_EBI_AD1_PC1();
    SET_EBI_AD2_PC2();
    SET_EBI_AD3_PC3();
    SET_EBI_AD4_PC4();
    SET_EBI_AD5_PC5();
    SET_EBI_AD6_PD8();
    SET_EBI_AD7_PD9();
    SET_EBI_AD8_PE14();
    SET_EBI_AD9_PE15();
    SET_EBI_AD10_PE1();
    SET_EBI_AD11_PE0();
    SET_EBI_AD12_PH8();
    SET_EBI_AD13_PH9();
    SET_EBI_AD14_PH10();
    SET_EBI_AD15_PH11();

    /* ADDR16 ~ ADDR19; */
    SET_EBI_ADR16_PF9();
    SET_EBI_ADR17_PF8();
    SET_EBI_ADR18_PF7();
    SET_EBI_ADR19_PF6();

    /* EBI nWR and nRD pins on PA.10 and PA.11 */
    SET_EBI_nWR_PA10();
    SET_EBI_nRD_PA11();

    /* EBI nWRH and nWRL pins on PB.6 and PB.7 */
    SET_EBI_nWRH_PB6();
    SET_EBI_nWRL_PB7();

    /* EBI nCS0 pin on PD.12 */
    SET_EBI_nCS0_PD12();

    /* EBI ALE pin on PA.8 */
    SET_EBI_ALE_PA8();
}

void SYS_Init(void)
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

    /* Switch SCLK clock source to HIRC */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);

    /* Enable PLL0 180MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable EBI clock */
    CLK_EnableModuleClock(EBI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Initialize MPU Region */
    InitPreDefMPURegion(NULL, 0);

    /* Lock protected registers */
    SYS_LockReg();
}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------------------+\n");
    printf("|    EBI SRAM Sample Code on Bank0 with PDMA transfer    |\n");
    printf("+--------------------------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect IS61WV102416BLL SRAM to EBI bank0 before accessing !! *\n");
    printf("* EBI pins settings:                                                   *\n");
    printf("*   - AD0 ~ AD5 on PC.0~5                                              *\n");
    printf("*   - AD6 ~ AD7 on PD.8 and PD.9                                       *\n");
    printf("*   - AD8 ~ AD9 on PE.14 and PE.15                                     *\n");
    printf("*   - AD10 ~ AD11 on PE.1 and PE.0                                     *\n");
    printf("*   - AD12 ~ AD15 on PH.8~11                                           *\n");
    printf("*   - ADR16 ~ ADR19 on PF.9~6                                          *\n");
    printf("*   - nWR on PA.10                                                     *\n");
    printf("*   - nRD on PA.11                                                     *\n");
    printf("*   - nWRL on PB.7                                                     *\n");
    printf("*   - nWRH on PB.6                                                     *\n");
    printf("*   - nCS0 on PD.12                                                    *\n");
    printf("*   - ALE on PA.8                                                      *\n");
    printf("************************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank0 to access external SRAM */
    EBI_Open(EBI_BANK0, EBI_BUSWIDTH_16BIT, EBI_TIMING_NORMAL, 0, EBI_CS_ACTIVE_LOW);

    /* Start to test EBI SRAM */
    if (SRAM_BS616LV4017(512 * 1024) < 0) goto lexit;

    /* EBI SRAM with PDMA test */
    if (AccessEBIWithPDMA() < 0) goto lexit;

    printf("*** SRAM Test OK ***\n");

lexit:

    /* Disable EBI function */
    EBI_Close(EBI_BANK0);

    /* Disable EBI clock */
    CLK_DisableModuleClock(EBI0_MODULE);

    while (1) {}
}
