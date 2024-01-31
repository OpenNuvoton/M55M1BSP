/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to make an application booting from APROM
 *          with a sub-routine resided on SPI Flash.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

//------------------------------------------------------------------------------
#define SPIM_PORT                   SPIM0

#define USE_4_BYTES_MODE            0   /* W25Q20 does not support 4-bytes address mode. */
#define SPIM_CIPHER_ON              0
//#define DMM_MODE_TRIM

//------------------------------------------------------------------------------
void spim_routine(void);

//------------------------------------------------------------------------------
void SPIM_SetDMMAddrNonCacheable(void)
{
    uint32_t u32DMMAddr = SPIM_GetDMMAddress(SPIM_PORT);

    /* Disable D-Cache */
    SCB_DisableDCache();

    /* Configure MPU memory attribute */
    /*
     * Attribute 0
     * Memory Type = Normal
     * Attribute   = Outer Non-cacheable, Inner Non-cacheable
     */
    ARM_MPU_SetMemAttr(0UL, ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE));

    /* Configure MPU memory regions */
    ARM_MPU_SetRegion(0UL,                                                          /* Region 0 */
                      ARM_MPU_RBAR((uint32_t)u32DMMAddr, ARM_MPU_SH_NON, 0, 0, 0),  /* Non-shareable, read/write, privileged, non-executable */
                      ARM_MPU_RLAR((uint32_t)u32DMMAddr + 0x10000, 0)               /* Use Attr 0 */
                     );
    /* Enable MPU */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* Enable D-Cache */
    SCB_EnableDCache();
}

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

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    if (SPIM_PORT == SPIM0)
    {
        /* Enable SPIM module clock */
        CLK_EnableModuleClock(SPIM0_MODULE);

        /* Init SPIM multi-function pins */
        SET_SPIM0_CLK_PC4();
        SET_SPIM0_MISO_PG12();
        SET_SPIM0_MOSI_PG11();
        SET_SPIM0_D2_PC0();
        SET_SPIM0_D3_PG10();
        SET_SPIM0_SS_PC3();

        PC->SMTEN |= (GPIO_SMTEN_SMTEN0_Msk |
                      GPIO_SMTEN_SMTEN3_Msk |
                      GPIO_SMTEN_SMTEN4_Msk);

        PG->SMTEN |= (GPIO_SMTEN_SMTEN10_Msk |
                      GPIO_SMTEN_SMTEN11_Msk |
                      GPIO_SMTEN_SMTEN12_Msk);

        /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
        GPIO_SetSlewCtl(PC, BIT0, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT3, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT4, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PG, BIT10, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT11, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT12, GPIO_SLEWCTL_HIGH);
    }
    else if (SPIM_PORT == SPIM1)
    {
        /* Enable SPIM module clock */
        CLK_EnableModuleClock(SPIM1_MODULE);

        /* Init SPIM multi-function pins */
        SET_SPIM1_CLK_PH13();
        SET_SPIM1_MISO_PJ5();
        SET_SPIM1_MOSI_PJ6();
        SET_SPIM1_D2_PJ4();
        SET_SPIM1_D3_PJ3();
        SET_SPIM1_SS_PJ7();

        PH->SMTEN |= (GPIO_SMTEN_SMTEN13_Msk);

        PJ->SMTEN |= (GPIO_SMTEN_SMTEN3_Msk |
                      GPIO_SMTEN_SMTEN4_Msk |
                      GPIO_SMTEN_SMTEN5_Msk |
                      GPIO_SMTEN_SMTEN6_Msk |
                      GPIO_SMTEN_SMTEN7_Msk);

        /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
        GPIO_SetSlewCtl(PH, BIT13, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PJ, BIT3, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT4, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT5, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT6, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT7, GPIO_SLEWCTL_HIGH);
    }
}

int SPIM_TrimRxClkDlyNum(SPIM_T *spim, SPIM_PHASE_T *psWbRdCMD)
{
    volatile uint8_t u8RdDelay = 0;
    uint8_t u8RdDelayIdx = 0;
    uint8_t u8RdDelayRes[0x0F] = {0};
    uint32_t u32SAddr = 0x10000;
    volatile uint32_t u32i = 0;
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(spim);
    uint8_t au8TrimPatten[32] =
    {
        0xff, 0x0F, 0xFF, 0x00, 0xFF, 0xCC, 0xC3, 0xCC,
        0xC3, 0x3C, 0xCC, 0xFF, 0xFE, 0xFF, 0xFE, 0xEF,
        0xFF, 0xDF, 0xFF, 0xDD, 0xFF, 0xFB, 0xFF, 0xFB,
        0xBF, 0xFF, 0x7F, 0xFF, 0x77, 0xF7, 0xBD, 0xEF,
    };
    uint8_t au8DestBuf[32] = {0};
    SPIM_PHASE_T sWbWrCMD =
    {
        CMD_NORMAL_PAGE_PROGRAM,                                    //Command Code
        PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,       //Command Phase
        PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,       //Address Phase
        PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,  //Data Phase
        0,
    };

#ifdef DMM_MODE_TRIM
    uint32_t u32RdDataCnt = 0;
    uint32_t u32DMMAddr = SPIM_GetDMMAddress(spim);
    uint32_t *pu32RdData = NULL;
#endif //

    SPIM_DMADMM_InitPhase(spim, &sWbWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    SPIM_SET_CLOCK_DIVIDER(spim, 8);

    SPIM_EraseBlock(spim, u32SAddr, SPIM_OP_DISABLE, OPCODE_BE_64K, 1, SPIM_OP_ENABLE);

    SPIM_DMA_Write(spim,
                   u32SAddr,
                   (sWbWrCMD.u32AddrWidth == PHASE_WIDTH_32) ? 1UL : 0UL,
                   sizeof(au8TrimPatten),
                   au8TrimPatten,
                   sWbWrCMD.u32CMDCode);

    SPIM_SET_CLOCK_DIVIDER(spim, u32Div);

#ifdef DMM_MODE_TRIM

    SPIM_DMADMM_InitPhase(spim, psWbRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);
    SPIM_EnterDirectMapMode(spim,
                            (psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? 1UL : 0UL,
                            psWbRdCMD->u32CMDCode,
                            1);
#endif

    for (u8RdDelay = 0; u8RdDelay <= 0xF; u8RdDelay++)
    {
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8RdDelay);

        memset(au8DestBuf, 0, sizeof(au8DestBuf));

#ifndef DMM_MODE_TRIM

        SPIM_DMADMM_InitPhase(spim, psWbRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
        SPIM_DMA_Read(spim,
                      u32SAddr,
                      ((psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? 1UL : 0UL),
                      sizeof(au8TrimPatten),
                      au8DestBuf,
                      psWbRdCMD->u32CMDCode,
                      SPIM_OP_ENABLE);
#else
        u32RdDataCnt = 0;
        pu32RdData = (uint32_t *)au8DestBuf;

        for (u32i = u32SAddr; u32i < (u32SAddr + sizeof(au8TrimPatten)); u32i += 4)
        {
            pu32RdData[u32RdDataCnt++] = inpw(u32DMMAddr + u32i);
        }

#endif

        // Compare.
        if (memcmp(au8TrimPatten, au8DestBuf, sizeof(au8TrimPatten)) == 0)
        {
            printf("RX Delay: %d = Pass\r\n", u8RdDelay);
            u8RdDelayRes[u8RdDelayIdx++] = u8RdDelay;
        }
    }

    if (u8RdDelayIdx >= 2)
    {
        u8RdDelayIdx = (u8RdDelayIdx / 2) - 1;
    }
    else
    {
        u8RdDelayIdx = 0;
    }

    printf("\r\nRX Delay = %d\r\n\r\n", u8RdDelayRes[u8RdDelayIdx]);
    SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8RdDelayRes[u8RdDelayIdx]);

    return u8RdDelay;
}

int main()
{
    uint8_t idBuf[3] = {0};
    SPIM_PHASE_T sWb0BhRdCMD =
    {
        /* 0x0B: CMD_DMA_FAST_READ Command Phase Table */
        CMD_DMA_FAST_READ,                                                        // Command Code
        PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                      // Command Phase
        PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                     // Address Phase
        PHASE_NORMAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE, // Data Phase
        8,                                                                        // Dummy Cycle Phase
    };

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O    */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    SPIM_SetDMMAddrNonCacheable();

    /**
     * GCC project users must use the ICP tool to burn binary to APROM and
     * SPI flash separately, and after entering the debugger, only APROM code can source debug.
    */
    printf("+--------------------------------------------------+\n");
    printf("|      SPIM DMM mode running program on flash      |\n");
    printf("+--------------------------------------------------+\n");

    /* Set SPIM clock as HCLK divided by 8 */
    SPIM_SET_CLOCK_DIVIDER(SPIM_PORT, 1);

    SPIM_DISABLE_CIPHER(SPIM_PORT);             /* Disable SPIM Cipher */

    SPIM_SET_RXCLKDLY_RDDLYSEL(SPIM_PORT, 1);   /* Insert 3 delay cycle. Adjust the sampling clock of received data to latch the correct data. */

    if (SPIM_InitFlash(SPIM_PORT, 1) != 0)      /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }

    SPIM_ReadJedecId(SPIM_PORT, idBuf, sizeof(idBuf), 1, 0);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n",
           idBuf[0], idBuf[1], idBuf[2]);

#if (SPIM_REG_CACHE == 1) //TESTCHIP_ONLY not support
    SPIM_ENABLE_CACHE(SPIM_PORT);
    SPIM_PORT->CTL1 |= SPIM_CTL1_CDINVAL_Msk;        // invalid cache
#endif

    SPIM_TrimRxClkDlyNum(SPIM_PORT, &sWb0BhRdCMD);

    SPIM_DMADMM_InitPhase(SPIM_PORT, &sWb0BhRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);
    SPIM_EnterDirectMapMode(SPIM_PORT, USE_4_BYTES_MODE, sWb0BhRdCMD.u32CMDCode, 1);

    while (1)
    {
        printf("\n\nProgram is currently running on APROM flash.\n");
        printf("Press any key to branch to sub-routine on SPIM flash...\n");

        getchar();

        spim_routine();
    }

lexit:

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
