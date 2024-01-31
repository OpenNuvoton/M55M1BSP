
//
// Includes
//
#include <stdio.h>
#include "NuMicro.h"
#include "board_m55m1.h"
//
// Defines
//
#define SPI_MASTER_TX_DMA_CH 0
#define SPI_MASTER_RX_DMA_CH 1
#define TEST_COUNT 64

//
// Global variable declaration
//
static uint32_t s_au32MasterToSlaveTestPattern[TEST_COUNT];
static uint32_t s_au32SlaveToMasterTestPattern[TEST_COUNT];
static uint32_t s_au32MasterRxBuffer[TEST_COUNT];
static uint32_t s_au32SlaveRxBuffer[TEST_COUNT];
volatile uint8_t g_u8_PDMAIsBusy = 1;
//
// Functions
//
//
// config_panel_rectwindow
//
//void config_rectwindow(rect_info_t* pRect, uint16_t h, uint16_t w)
//{
//    // Set panel fill rect-window parameter
//    pRect->x= ( PANEL_WIDTH - PREVIEW_WIDTH) / 2;
//    pRect->y= ( PANEL_HEIGHT - PREVIEW_HEIGHT) / 2;
//    pRect->width= w;
//    pRect->height= h;

//}


void config_showresultwindow(rect_info_t *pRect, uint16_t h, uint16_t w)
{
    // Set panel fill rect-window parameter
    pRect->x = 0;
    pRect->y = 0;
    pRect->width = w;
    pRect->height = h;

}


void PDMA_IRQHandler(PDMA_T *PDMA)
{
    int i;

    uint32_t intsts = PDMA_GET_INT_STATUS(PDMA);
    uint32_t abtsts = PDMA_GET_ABORT_STS(PDMA);
    uint32_t tdsts  = PDMA_GET_TD_STS(PDMA);
    uint32_t unalignsts  = PDMA_GET_ALIGN_STS(PDMA);
    //uint32_t reqto  = intsts & PDMA_INTSTS_REQTOFn_Msk;
    //uint32_t reqto_ch = (reqto >> PDMA_INTSTS_REQTOFn_Pos);

    //int allch_sts = (reqto_ch | tdsts | abtsts | unalignsts);

    // Abort
    if (intsts & PDMA_INTSTS_ABTIF_Msk)
    {
        // Clear all Abort flags
        PDMA_CLR_ABORT_FLAG(PDMA, abtsts);
    }

    // Transfer done
    if (intsts & PDMA_INTSTS_TDIF_Msk)
    {
        // Clear all transfer done flags
        PDMA_CLR_TD_FLAG(PDMA, tdsts);
        g_u8_PDMAIsBusy = 0;
    }

    // Unaligned
    if (intsts & PDMA_INTSTS_ALIGNF_Msk)
    {
        // Clear all Unaligned flags
        PDMA_CLR_ALIGN_FLAG(PDMA, unalignsts);
    }

}

void PDMA0_IRQHandler(void)
{
    /* enter interrupt */
    //rt_interrupt_enter();

    PDMA_IRQHandler(PDMA0);

    /* leave interrupt */
    // rt_interrupt_leave();
}


void nuFlush_LCD_WithPDMA(uint8_t *pattern, uint32_t len)
{
    uint32_t u32DataCount, u32TestCycle;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;

    printf("\nSPI2 TX/RX with PDMA ");

    /* Reset PDMA module */
    //SYS_ResetModule(PDMA0_RST);

    /* Enable PDMA channels */
    PDMA_Open(PDMA0, (1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH));

    /* Enable PDMA interrupts */
    PDMA_EnableInt(PDMA0, (1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH), PDMA_INT_TRANS_DONE);

    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_8, len);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, SPI_MASTER_TX_DMA_CH, (uint32_t)pattern, PDMA_SAR_INC, (uint32_t)&SPI2->TX, PDMA_DAR_FIX);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_SPI2_TX, FALSE, 0);

    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    //PDMA0->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;


    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_8, len);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, SPI_MASTER_RX_DMA_CH, (uint32_t)&SPI2->RX, PDMA_SAR_FIX, (uint32_t)s_au32MasterRxBuffer, PDMA_DAR_FIX);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_SPI2_RX, FALSE, 0);

    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    //PDMA0->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;


    /* Enable SPI slave PDMA function */
    SPI_TRIGGER_TX_RX_PDMA(SPI2);

    g_u8_PDMAIsBusy = 0;

    u32TestCycle = 10000;

    while (!g_u8_PDMAIsBusy)
    {
        u32TestCycle--;

        if (u32TestCycle == 0)
        {
            printf("PDMA Busy Timout\r\n");
            i32Err = 1;
            break;
        }
    }

    /* Disable all PDMA channels */
    PDMA_Close(PDMA0);
    return;
}

