/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show HyperRAM read/write through HyperBus Interface.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"
#include "hyperram_code.h"

//------------------------------------------------------------------------------
#define FLASH_BLOCK_SIZE            (8 * 1024)     /* Flash block size. Depend on the physical flash. */
#define TEST_BLOCK_ADDR             0x10000         /* Test block address on SPI flash. */
#define BUFFER_SIZE                 2048

#define SPIM_PORT                   SPIM1

//------------------------------------------------------------------------------
void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
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
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable SPIM module clock */
    CLK_EnableModuleClock(SPIM0_MODULE);

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    if (SPIM_PORT == SPIM0)
    {
        /* Enable SPIM module clock */
        CLK_EnableModuleClock(SPIM0_MODULE);

        /* Enable SPIM module clock */
        CLK_EnableModuleClock(OTFC0_MODULE);

        /* Init SPIM multi-function pins */
        SET_SPIM0_CLKN_PC5();
        SET_SPIM0_CLK_PC4();
        SET_SPIM0_D2_PC0();
        SET_SPIM0_D3_PG10();
        SET_SPIM0_D4_PG9();
        SET_SPIM0_D5_PG13();
        SET_SPIM0_D6_PG14();
        SET_SPIM0_D7_PG15();
        SET_SPIM0_MISO_PG12();
        SET_SPIM0_MOSI_PG11();
        SET_SPIM0_RESETN_PC2();
        SET_SPIM0_RWDS_PC1();
        SET_SPIM0_SS_PC3();

        PC->SMTEN |= (GPIO_SMTEN_SMTEN0_Msk |
                      GPIO_SMTEN_SMTEN1_Msk |
                      GPIO_SMTEN_SMTEN2_Msk |
                      GPIO_SMTEN_SMTEN3_Msk |
                      GPIO_SMTEN_SMTEN4_Msk |
                      GPIO_SMTEN_SMTEN5_Msk);
        PG->SMTEN |= (GPIO_SMTEN_SMTEN9_Msk |
                      GPIO_SMTEN_SMTEN10_Msk |
                      GPIO_SMTEN_SMTEN11_Msk |
                      GPIO_SMTEN_SMTEN12_Msk |
                      GPIO_SMTEN_SMTEN13_Msk |
                      GPIO_SMTEN_SMTEN14_Msk |
                      GPIO_SMTEN_SMTEN15_Msk);

        /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
        GPIO_SetSlewCtl(PC, BIT0, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT1, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT2, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT3, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT4, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT5, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PG, BIT9, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT10, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT11, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT12, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT13, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT14, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT15, GPIO_SLEWCTL_HIGH);
    }
    else if (SPIM_PORT == SPIM1)
    {
        /* Enable SPIM module clock */
        CLK_EnableModuleClock(SPIM1_MODULE);

        /* Enable SPIM module clock */
        CLK_EnableModuleClock(OTFC1_MODULE);

        /* Init SPIM multi-function pins */
        SET_SPIM1_CLKN_PH12();
        SET_SPIM1_CLK_PH13();
        SET_SPIM1_D2_PJ4();
        SET_SPIM1_D3_PJ3();
        SET_SPIM1_D4_PH15();
        SET_SPIM1_D5_PD7();
        SET_SPIM1_D6_PD6();
        SET_SPIM1_D7_PD5();
        SET_SPIM1_MISO_PJ5();
        SET_SPIM1_MOSI_PJ6();
        SET_SPIM1_RESETN_PJ2();
        SET_SPIM1_RWDS_PH14();
        SET_SPIM1_SS_PJ7();

        PD->SMTEN |= (GPIO_SMTEN_SMTEN5_Msk |
                      GPIO_SMTEN_SMTEN6_Msk |
                      GPIO_SMTEN_SMTEN7_Msk);
        PH->SMTEN |= (GPIO_SMTEN_SMTEN12_Msk |
                      GPIO_SMTEN_SMTEN13_Msk |
                      GPIO_SMTEN_SMTEN14_Msk |
                      GPIO_SMTEN_SMTEN15_Msk);
        PJ->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk |
                      GPIO_SMTEN_SMTEN3_Msk |
                      GPIO_SMTEN_SMTEN4_Msk |
                      GPIO_SMTEN_SMTEN5_Msk |
                      GPIO_SMTEN_SMTEN6_Msk |
                      GPIO_SMTEN_SMTEN7_Msk);

        /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
        GPIO_SetSlewCtl(PD, BIT5, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PD, BIT6, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PD, BIT7, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PH, BIT12, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PH, BIT13, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PH, BIT14, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PH, BIT15, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PJ, BIT2, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT3, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT4, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT5, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT6, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT7, GPIO_SLEWCTL_HIGH);
    }
}

static int32_t Clear4Bytes(uint32_t u32StartAddr)
{
    outp32(u32StartAddr, 0);
    return 0;
}

static int32_t ClearHyperRAM(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t u32Data, i;

    for (i = u32StartAddr; i < u32EndAddr; i += 4)
    {
        if (Clear4Bytes(i) < 0)
        {
            return -1;
        }

        u32Data = inp32(i);

        if (u32Data != 0)
        {
            printf("ClearHyperRAM fail!! Read address:0x%08x  data::0x%08x  expect: 0\n",  i, u32Data);
            return -1;
        }
    }

    return 0;
}

int main(void)
{
    uint32_t u32i, u32Data, u32PatCnt;
    uint32_t u32StartAddr, u32EndAddr;
    uint32_t u32BitPattern;
    uint16_t u16BitPattern;
    uint8_t u8BitPattern;
    uint32_t au32Pat[5] = {0x12345678, 0x5a5a5a5a, 0xa5a5a5a5, 0xFFFFFFFF, 0x9abcdef1};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O    */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("+----------------------------------------+\n");
    printf("|       HyperRAM read/write sample       |\n");
    printf("+----------------------------------------+\n");

    InitPreDefMPURegion(NULL, 0);

    HyperRAM_Init(SPIM_PORT);

    SPIM_HYPER_EnterDirectMapMode(SPIM_PORT);

    /* Memory max space 64MBits --> 8Mbytes --> 0x800000 */
    u32StartAddr = SPIM_HYPER_GetDMMAddress(SPIM_PORT);
    u32EndAddr   = u32StartAddr + 0x1000;

    for (u32PatCnt = 0; u32PatCnt < 5; u32PatCnt++)
    {
        printf("======= Pattern Round[%d] Test Start! ======= \n", u32PatCnt);
        u32BitPattern = au32Pat[u32PatCnt];
        u16BitPattern = u32BitPattern >> 16;
        u8BitPattern  = u16BitPattern >> 8;

        printf("Test Pattern 32 bits: 0x%08x\n", u32BitPattern);
        printf("             16 bits: 0x%08x\n", u16BitPattern);
        printf("              8 bits: 0x%08x\n", u8BitPattern);

        /* Clear HyperRAM */
        if (ClearHyperRAM(u32StartAddr, u32EndAddr) < 0)
            goto lexit;

        /* Fill 4 Byte pattern to HyperRAM */
        printf("4 Byte Write test .....\n");

        for (u32i = u32StartAddr; u32i < u32EndAddr; u32i += 4)
        {
            outp32(u32i, u32BitPattern);
        }

        /* Read 4 Byte pattern to check */
        for (u32i = u32StartAddr; u32i < u32EndAddr; u32i += 4)
        {
            u32Data = inp32(u32i);

            if (u32Data != u32BitPattern)
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n",
                       __LINE__, u32i, u32Data, u32BitPattern);
                goto lexit;
            }
        }

        printf("4 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        if (ClearHyperRAM(u32StartAddr, u32EndAddr) < 0)
            goto lexit;

        /* Fill 2 Byte pattern to HyperRAM */
        printf("2 Byte Write test .....\n");

        for (u32i = u32StartAddr; u32i < u32EndAddr; u32i += 2)
        {
            outp16(u32i, u16BitPattern);
        }

        /* Read 2 Byte pattern to check */
        for (u32i = u32StartAddr; u32i < u32EndAddr; u32i += 2)
        {
            u32Data = inp16(u32i);

            if (u32Data != u16BitPattern)
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n",
                       __LINE__, u32i,  u32Data, u16BitPattern);
                goto lexit;
            }
        }

        printf("2 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        if (ClearHyperRAM(u32StartAddr, u32EndAddr) < 0)
            goto lexit;

        /* Fill 1 Byte pattern to HyperRAM */
        printf("1 Byte Write test .....\n");

        for (u32i = u32StartAddr; u32i < u32EndAddr; u32i += 1)
        {
            outp8(u32i, u8BitPattern);
        }

        /* Read 1 Byte pattern to check */
        for (u32i = u32StartAddr; u32i < u32EndAddr; u32i += 1)
        {
            u32Data = inp8(u32i);

            if (u32Data != u8BitPattern)
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n",
                       __LINE__, u32i,  u32Data, (u8BitPattern << 8 | u8BitPattern));
                goto lexit;
            }
        }

        printf("1 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        if (ClearHyperRAM(u32StartAddr, u32EndAddr) < 0)
            goto lexit;

        printf("======= Pattern Round[%d] Test Pass! ======= \n\n", u32PatCnt);

    }

    printf("\nHyperBus Interface Sample Code Completed.\n");

lexit:

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
