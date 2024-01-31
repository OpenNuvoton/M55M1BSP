/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use G-senor to wake-up by AWF through low power domain IP: LPTMR, LPPDMA, LPI2C.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include "MPU6500.h"

#define LPI2C0_LPPDMA_TX_CH         0
#define LPI2C0_LPPDMA_RX_CH         1
#define SG_TX_LENGTH                3
#define SG_RX_LENGTH                1
#define MAX_SG_TAB_NUM              1
#define LPPDMA_ENABLE_CHANNEL(x)    (LPPDMA->CHCTL |= (1 << x))
#define I2C_TARGET_ADDR             (MPU6500_DEVICE_ID)
#define DESC_SC_TX_ADRRESS          0x20310800
#define DESC_SC_RX_ADRRESS          0x20310820
#define GSensor_Threshold           300
#define SrcArray_Address            0x20310100
#define DstArray_Address            (uint32_t)(&AWF->DAT)

uint32_t g_u32ACCCount, g_u32HTHValue, g_u32LTHValue, g_u32WBINITValue;

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

uint32_t lppdma_channel_set(uint8_t ch)
{
    uint32_t addr;
    addr = ((uint32_t) LPPDMA_BASE + (0x10 * ch));
    return addr;
}

void MPU6500_Init(void)
{
    /* LPI2C multi-function pin */
    SET_LPI2C0_SCL_PC12();
    SET_LPI2C0_SDA_PC11();

    /* LPI2C pins enable schmitt trigger */
    CLK_EnableModuleClock(GPIOC_MODULE);
    GPIO_ENABLE_SCHMITT_TRIGGER(PC, (BIT11 | BIT12));

    /* Open LPI2C0 module and set bus clock */
    LPI2C_Open(LPI2C0, 100000);
}

void LPTMR_Init(void)
{
    /* Open LPTRM0 to periodic mode and timeout 10 times per second */
    LPTMR_Open(LPTMR0, TIMER_PERIODIC_MODE, 10);

    /* Enable LPTMR Power-down engine clock */
    LPTMR0->CTL |= LPTMR_CTL_PDCLKEN_Msk;

    /* Set LPTMR to trigger LPI2C when LPTMR0 timeout */
    LPTMR_SetTriggerSource(LPTMR0, TIMER_TRGSRC_TIMEOUT_EVENT);

    /* Enable LPTMR0 to trigger Low Power IP */
    LPTMR_SetTriggerTarget(LPTMR0, (LPTMR_TRG_TO_LPPDMA | LPTMR_TRGEN));
}

void BuildTxSCTab(uint32_t u32tabNum, uint32_t u32TxfSize, uint32_t pu8StarAddr)
{
    uint32_t i;

    DMA_DESC_T *DMA_DESC_SC_TX = (DMA_DESC_T *)DESC_SC_TX_ADRRESS;

    /* Configre TX scatter-gather table */
    for (i = 0; i < u32tabNum; i++)
    {
        DMA_DESC_SC_TX[i].ctl = ((u32TxfSize - 1) << LPPDMA_DSCT_CTL_TXCNT_Pos) | \
                                (0 << LPPDMA_DSCT_CTL_TXWIDTH_Pos) | \
                                (0 << LPPDMA_DSCT_CTL_SAINC_Pos) | (3 << LPPDMA_DSCT_CTL_DAINC_Pos) | \
                                (0 << LPPDMA_DSCT_CTL_BURSIZE_Pos) | LPPDMA_REQ_SINGLE | PDMA_OP_SCATTER | (1 << 7);
        DMA_DESC_SC_TX[i].src = (uint32_t)(pu8StarAddr + i * u32TxfSize);
        DMA_DESC_SC_TX[i].dest = (uint32_t)(&(LPI2C0->DAT));
        DMA_DESC_SC_TX[i].offset = (uint32_t)&DMA_DESC_SC_TX[0] + 0x10 * (i + 1);
    }

    DMA_DESC_SC_TX[u32tabNum - 1].offset = (uint32_t)&DMA_DESC_SC_TX[0];
}

void BuildRxSCTab(uint32_t u32tabNum, uint32_t u32RxfSize, uint32_t pu8StarAddr)
{
    uint32_t i;
    DMA_DESC_T *DMA_DESC_SC_RX = (DMA_DESC_T *)DESC_SC_RX_ADRRESS;

    /* Configre RX scatter-gather table */
    for (i = 0; i < u32tabNum; i++)
    {
        DMA_DESC_SC_RX[i].ctl = ((u32RxfSize - 1) << LPPDMA_DSCT_CTL_TXCNT_Pos) | \
                                (1 << LPPDMA_DSCT_CTL_TXWIDTH_Pos) | \
                                (3 << LPPDMA_DSCT_CTL_SAINC_Pos) | (0 << LPPDMA_DSCT_CTL_DAINC_Pos) | \
                                (0 << LPPDMA_DSCT_CTL_BURSIZE_Pos) | LPPDMA_REQ_SINGLE | PDMA_OP_SCATTER | (1 << 7);
        DMA_DESC_SC_RX[i].src = (uint32_t)(&(LPI2C0->DAT));
        DMA_DESC_SC_RX[i].dest = pu8StarAddr;
        DMA_DESC_SC_RX[i].offset = (uint32_t)&DMA_DESC_SC_RX[0] + 0x10 * (i + 1);
    }

    DMA_DESC_SC_RX[u32tabNum - 1].offset = (uint32_t)&DMA_DESC_SC_RX[0];
}

void LPDMA_SC_trigger_init(uint32_t PDMAReq, uint8_t u8TestCh, uint32_t u8TestLen)
{
    DSCT_T *ch_dsct;
    DMA_DESC_T *DMA_DESC_SC_TX = (DMA_DESC_T *)DESC_SC_TX_ADRRESS;
    DMA_DESC_T *DMA_DESC_SC_RX = (DMA_DESC_T *)DESC_SC_RX_ADRRESS;

    /* Enable LPPDMA channel */
    LPPDMA_ENABLE_CHANNEL(u8TestCh);

    ch_dsct = (DSCT_T *) lppdma_channel_set(u8TestCh);

    if (PDMAReq == LPPDMA_LPI2C0_TX)
    {
        /* TX scatter-gather initialize */
        BuildTxSCTab(MAX_SG_TAB_NUM, SG_TX_LENGTH, SrcArray_Address);
        ch_dsct->CTL = PDMA_OP_SCATTER;
        ch_dsct->NEXT = (uint32_t)&DMA_DESC_SC_TX[0];
        LPPDMA->REQSEL0_3 = (LPPDMA->REQSEL0_3 & ~ LPPDMA_REQSEL0_3_REQSRC1_Msk) | (LPPDMA_LPI2C0_TX) << 8;
    }
    else
    {
        /* RX scatter-gather initialize */
        BuildRxSCTab(MAX_SG_TAB_NUM, SG_RX_LENGTH, DstArray_Address);
        ch_dsct->CTL = PDMA_OP_SCATTER;
        ch_dsct->NEXT = (uint32_t)&DMA_DESC_SC_RX[0];
        LPPDMA->REQSEL0_3 = (LPPDMA->REQSEL0_3 & ~ LPPDMA_REQSEL0_3_REQSRC0_Msk) | (LPPDMA_LPI2C0_RX) << 0;
    }
}

void LPI2C_Trigger_Init(uint32_t u32Mode, uint32_t u32Src, uint32_t u32RxCnt, uint32_t u32TxCnt)
{
    /* Set Auto-operation Mode */
    LPI2C0->AUTOCTL = (LPI2C0->AUTOCTL & ~LPI2C_AUTOCTL_AUTOMODE_Msk) | (u32Mode);

    /* Trigger source select */
    LPI2C0->AUTOCTL = (LPI2C0->AUTOCTL & ~LPI2C_AUTOCTL_TGSRCSEL_Msk) | (u32Src);
    LPI2C0->AUTOCNT = (LPI2C0->AUTOCNT & ~(LPI2C_AUTOCNT_RXCNT_Msk | LPI2C_AUTOCNT_TXCNT_Msk));

    if (u32RxCnt != 0)
    {
        LPI2C0->AUTOCNT |= ((u32RxCnt - 1) <<  LPI2C_AUTOCNT_RXCNT_Pos);
    }

    if (u32TxCnt != 0)
    {
        LPI2C0->AUTOCNT |= ((u32TxCnt - 1) <<  LPI2C_AUTOCNT_TXCNT_Pos);
    }

    LPI2C0->AUTOCTL |= LPI2C_AUTOCTL_TRGEN_Msk;
}

void LPPDMA_LPI2C_Init(void)
{
    uint32_t  i;

    uint8_t *SrcArray = (uint8_t *) SrcArray_Address;

    /* Device write command */
    for (i = 0; i < MAX_SG_TAB_NUM; i++)
    {
        SrcArray[i * SG_TX_LENGTH] = (I2C_TARGET_ADDR << 1);
    }

    /* Device data command */
    for (i = 0; i < MAX_SG_TAB_NUM; i++)
    {
        SrcArray[i * SG_TX_LENGTH + 1] = ACCEL_XOUT_H;
    }

    /* Device read command */
    for (i = 0; i < MAX_SG_TAB_NUM; i++)
    {
        SrcArray[i * SG_TX_LENGTH + 2] = ((I2C_TARGET_ADDR << 1) | 1);
    }

    /* LPPDMA TX initialize */
    LPDMA_SC_trigger_init(LPPDMA_LPI2C0_RX, LPI2C0_LPPDMA_TX_CH, 3);

    /* LPPDMA TX initialize */
    LPDMA_SC_trigger_init(LPPDMA_LPI2C0_TX, LPI2C0_LPPDMA_RX_CH, 1);

    /* LPI2C trigger initialize */
    LPI2C_Trigger_Init(LPI2C_RANDOM_REPEAT_STA, LPI2C_TRGSRC_LPTMR0, SG_RX_LENGTH, SG_TX_LENGTH);
}

void AWF_Threshold_Update(void)
{
    uint32_t u32AccTemp = 0;

    /* Disable AWF interrupt to update threshold value */
    AWF_Open(AWF_BOTHINT_DISABLE, AWF_BOTHWK_DISABLE, g_u32HTHValue, g_u32LTHValue, g_u32WBINITValue, g_u32ACCCount);

    /* Wait all AWF word buffer updated by Gsensor */
    CLK_SysTickDelay(1000000);

    /* Get accumulation value */
    u32AccTemp = AWF_GET_ACUVAL();

    /* Calculate high threshold value */
    if (u32AccTemp + GSensor_Threshold >= 0x0007FFFF)
    {
        g_u32HTHValue = 0x0007FFFF;
    }
    else
    {
        g_u32HTHValue = u32AccTemp + GSensor_Threshold;
    }

    /* Calculate low threshold value */
    if (u32AccTemp < GSensor_Threshold)
    {
        g_u32LTHValue = 0;
    }
    else
    {
        g_u32LTHValue = u32AccTemp - GSensor_Threshold;
    }

    /* Calculate word buffer initial value */
    g_u32WBINITValue = u32AccTemp / g_u32ACCCount;

    /* Set AWF threshold, interrupt and wake-up function  */
    AWF_Open(AWF_BOTHINT_ENABLE, AWF_BOTHWK_ENABLE, g_u32HTHValue, g_u32LTHValue, g_u32WBINITValue, g_u32ACCCount);

    printf("Update, HTH = %d, LTH = %d\n", g_u32HTHValue, g_u32LTHValue);
}

void AWF_Wakeup_Test(void)
{
    /* Select power-down mode and power level */
    PMC_SetPowerDownMode(PMC_NPD0, PMC_PLCTL_PLSEL_PL1);

    printf("Enter Power-down...\n");

    /* Wait uart tx empty */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    /* Enter to power-down */
    PMC_PowerDown();

    printf("Wake-up!!\n \n");
}

NVT_ITCM void AWF_IRQHandler(void)
{
    uint32_t u32AccumulationValue;
    uint32_t u32HTH_Flag, u32LTH_Flag;

    /* Enable AWF0 module clock */
    CLK_EnableModuleClock(AWF0_MODULE); //TESTCHIP_ONLY

    u32AccumulationValue = AWF_GET_ACUVAL();
    u32HTH_Flag = AWF_GET_HTH_INTFLAG();
    u32LTH_Flag = AWF_GET_LTH_INTFLAG();

    printf("AWF Interrupt occured!!! ,AWF->STATUS = 0x%08x\n", AWF->STATUS);

    if (u32HTH_Flag)
    {
        printf("AWF HTH Interrupt occured!!!, HTH_INT = %d, LTH_INT = %d\n", u32HTH_Flag, u32LTH_Flag);
        printf("AWFHTH = %d, ACUVAL = %d\n", (uint32_t)((AWF->HTH & AWF_HTH_AWFHTH_Msk) >> AWF_HTH_AWFHTH_Pos), u32AccumulationValue);
    }

    if (u32LTH_Flag)
    {
        printf("AWF LTH Interrupt occured!!!, HTH_INT = %d, LTH_INT = %d\n", u32HTH_Flag, u32LTH_Flag);
        printf("AWFLTH = %d, ACUVAL = %d\n", (uint32_t)((AWF->LTH & AWF_LTH_AWFLTH_Msk) >> AWF_LTH_AWFLTH_Pos), u32AccumulationValue);
    }

    /* Update Gsenor threshold */
    AWF_Threshold_Update();

    /* Cleat interrutp status */
    AWF_CLEAR_HTH_INTFLAG();
    AWF_CLEAR_LTH_INTFLAG();

    /* CPU read interrupt flag register to wait write(clear) instruction completement */
    inp32(&AWF->STATUS);

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

    /* Waiting for Internal RC 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set HIRC for SCLK clock source*/
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable LPSRAM0 module clock */
    CLK_EnableModuleClock(LPSRAM0_MODULE);

    /* Enable LPPDMA0 module clock */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Enable AWF0 module clock */
    CLK_EnableModuleClock(AWF0_MODULE);

    /* Select LPTMR0 clock source to HIRC */
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, 0);

    /* Enable LPTMR0 module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Enable LPI2C module clock, for G-sensor*/
    CLK_EnableModuleClock(LPI2C0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|               AWF GSensor_Wakeup Sample Code                |\n");
    printf("+-------------------------------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    NVIC_EnableIRQ(AWF_IRQn);

    /* Set AWF initial value */
    g_u32ACCCount = 8;
    g_u32HTHValue = 0;
    g_u32LTHValue = 0;
    g_u32WBINITValue = 0;

    /* Enable Gsensor interface(LPI2C)*/
    MPU6500_Init();

    /* Low power timer initialize */
    LPTMR_Init();

    /* Low power I2C initialize */
    LPPDMA_LPI2C_Init();

    /* Low power timer start */
    LPTMR_Start(LPTMR0);

    /* Update Gsenor threshold */
    AWF_Threshold_Update();

    while (1)
    {
        printf("Press any key to entering power-down, then move the device to wake-up\n");

        getchar();

        AWF_Wakeup_Test();
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
