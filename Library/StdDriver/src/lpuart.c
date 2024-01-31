/**************************************************************************//**
 * @file     lpuart.c
 * @version  V1.00
 * @brief    M55M1 series LPUART driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPUART_Driver LPUART Driver
  @{
*/

/** @addtogroup LPUART_EXPORTED_FUNCTIONS LPUART Exported Functions
  @{
*/

/**
 *    @brief        Clear LPUART specified interrupt flag
 *
 *    @param[in]    lpuart                The pointer of the specified LPUART module.
 *    @param[in]    u32InterruptFlag    The specified interrupt of LPUART module.
 *                                      - \ref LPUART_INTSTS_WKIF_Msk      : Wake-up interrupt
 *                                      - \ref LPUART_INTSTS_BUFERRINT_Msk : Buffer Error interrupt
 *                                      - \ref LPUART_INTSTS_MODEMINT_Msk  : Modem Status interrupt
 *                                      - \ref LPUART_INTSTS_RLSINT_Msk    : Receive Line Status interrupt
 *
 *    @return       None
 *
 *    @details      The function is used to clear LPUART specified interrupt flag.
 */

void LPUART_ClearIntFlag(LPUART_T *lpuart, uint32_t u32InterruptFlag)
{

    if (u32InterruptFlag & LPUART_INTSTS_RLSINT_Msk)  /* Clear Receive Line Status Interrupt */
    {
        lpuart->FIFOSTS |= (LPUART_FIFOSTS_BIF_Msk | LPUART_FIFOSTS_FEF_Msk | LPUART_FIFOSTS_PEF_Msk);
        lpuart->FIFOSTS |= LPUART_FIFOSTS_ADDRDETF_Msk;
    }

    if (u32InterruptFlag & LPUART_INTSTS_MODEMINT_Msk)  /* Clear Modem Status Interrupt */
    {
        lpuart->MODEMSTS |= LPUART_MODEMSTS_CTSDETF_Msk;
    }

    if (u32InterruptFlag & LPUART_INTSTS_BUFERRINT_Msk)  /* Clear Buffer Error Interrupt */
    {
        lpuart->FIFOSTS = LPUART_FIFOSTS_RXOVIF_Msk | LPUART_FIFOSTS_TXOVIF_Msk;
    }

    if (u32InterruptFlag & LPUART_INTSTS_WKINT_Msk)  /* Clear Wake-up Interrupt */
    {
        lpuart->WKSTS = LPUART_WKSTS_CTSWKF_Msk  | LPUART_WKSTS_DATWKF_Msk  |
                        LPUART_WKSTS_RFRTWKF_Msk | LPUART_WKSTS_RS485WKF_Msk |
                        LPUART_WKSTS_TOUTWKF_Msk;
    }


}


/**
 *  @brief      Disable LPUART interrupt
 *
 *  @param[in]  lpuart The pointer of the specified LPUART module.
 *
 *  @return     None
 *
 *  @details    The function is used to disable LPUART interrupt.
 */
void LPUART_Close(LPUART_T *lpuart)
{
    lpuart->INTEN = 0ul;
}


/**
 *  @brief      Disable LPUART auto flow control function
 *
 *  @param[in]  lpuart The pointer of the specified LPUART module.
 *
 *  @return     None
 *
 *  @details    The function is used to disable LPUART auto flow control.
 */
void LPUART_DisableFlowCtrl(LPUART_T *lpuart)
{
    lpuart->INTEN &= ~(LPUART_INTEN_ATORTSEN_Msk | LPUART_INTEN_ATOCTSEN_Msk);
}


/**
 *    @brief        Disable LPUART specified interrupt
 *
 *    @param[in]    lpuart                The pointer of the specified LPUART module.
 *    @param[in]    u32InterruptFlag    The specified interrupt of LPUART module.
 *                                      - \ref LPUART_INTEN_WKIEN_Msk      : Wake-up interrupt
 *                                      - \ref LPUART_INTEN_BUFERRIEN_Msk  : Buffer Error interrupt
 *                                      - \ref LPUART_INTEN_RXTOIEN_Msk    : Rx time-out interrupt
 *                                      - \ref LPUART_INTEN_MODEMIEN_Msk   : Modem status interrupt
 *                                      - \ref LPUART_INTEN_RLSIEN_Msk     : Receive Line status interrupt
 *                                      - \ref LPUART_INTEN_THREIEN_Msk    : Tx empty interrupt
 *                                      - \ref LPUART_INTEN_RDAIEN_Msk     : Rx ready interrupt *
 *
 *    @return       None
 *
 *    @details      The function is used to disable LPUART specified interrupt and disable NVIC LPUART IRQ.
 */
void LPUART_DisableInt(LPUART_T  *lpuart, uint32_t u32InterruptFlag)
{
    /* Disable LPUART specified interrupt */
    LPUART_DISABLE_INT(lpuart, u32InterruptFlag);
}


/**
 *    @brief        Enable LPUART auto flow control function
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module.
 *
 *    @return       None
 *
 *    @details      The function is used to Enable LPUART auto flow control.
 */
void LPUART_EnableFlowCtrl(LPUART_T *lpuart)
{
    /* Set RTS pin output is low level active */
    lpuart->MODEM |= LPUART_MODEM_RTSACTLV_Msk;

    /* Set CTS pin input is low level active */
    lpuart->MODEMSTS |= LPUART_MODEMSTS_CTSACTLV_Msk;

    /* Set RTS and CTS auto flow control enable */
    lpuart->INTEN |= LPUART_INTEN_ATORTSEN_Msk | LPUART_INTEN_ATOCTSEN_Msk;
}


/**
 *    @brief        The function is used to enable LPUART specified interrupt and enable NVIC LPUART IRQ.
 *
 *    @param[in]    lpuart                The pointer of the specified LPUART module.
 *    @param[in]    u32InterruptFlag    The specified interrupt of LPUART module:
 *                                      - \ref LPUART_INTEN_WKIEN_Msk      : Wake-up interrupt
 *                                      - \ref LPUART_INTEN_BUFERRIEN_Msk  : Buffer Error interrupt
 *                                      - \ref LPUART_INTEN_RXTOIEN_Msk    : Rx time-out interrupt
 *                                      - \ref LPUART_INTEN_MODEMIEN_Msk   : Modem status interrupt
 *                                      - \ref LPUART_INTEN_RLSIEN_Msk     : Receive Line status interrupt
 *                                      - \ref LPUART_INTEN_THREIEN_Msk    : Tx empty interrupt
 *                                      - \ref LPUART_INTEN_RDAIEN_Msk     : Rx ready interrupt *
 *
 *    @return       None
 *
 *    @details      The function is used to enable LPUART specified interrupt and enable NVIC LPUART IRQ.
 */
void LPUART_EnableInt(LPUART_T  *lpuart, uint32_t u32InterruptFlag)
{
    /* Enable LPUART specified interrupt */
    LPUART_ENABLE_INT(lpuart, u32InterruptFlag);
}


/**
 *    @brief        Open and set LPUART function
 *
 *    @param[in]    lpuart          The pointer of the specified LPUART module.
 *    @param[in]    u32baudrate     The baudrate of LPUART module.
 *
 *    @return       None
 *
 *    @details      This function use to enable LPUART function and set baud-rate.
 */
void LPUART_Open(LPUART_T *lpuart, uint32_t u32baudrate)
{
    uint32_t u32LpUartClkSrcSel = 0ul, u32LpUartClkDivNum = 0ul;
    uint32_t u32ClkTbl[4] = {0, __LXT, __MIRC, __HIRC};


    if (lpuart == (LPUART_T *)LPUART0)
    {
        /* Get LPUART clock source selection */
        u32LpUartClkSrcSel = ((CLK->LPUARTSEL & CLK_LPUARTSEL_LPUART0SEL_Msk)) >> CLK_LPUARTSEL_LPUART0SEL_Pos;
        /* Get LPUART clock divider number */
        u32LpUartClkDivNum = (CLK->LPUARTDIV & CLK_LPUARTDIV_LPUART0DIV_Msk) >> CLK_LPUARTDIV_LPUART0DIV_Pos;

    }


    /* Select LPUART function */
    lpuart->FUNCSEL = LPUART_FUNCSEL_LPUART;

    /* Set LPUART line configuration */
    lpuart->LINE = LPUART_WORD_LEN_8 | LPUART_PARITY_NONE | LPUART_STOP_BIT_1;

    /* Set LPUART Rx and RTS trigger level */
    lpuart->FIFO &= ~(LPUART_FIFO_RFITL_Msk | LPUART_FIFO_RTSTRGLV_Msk);

    /* Get PLL clock frequency if LPUART clock source selection is PCLK2 */
    if (u32LpUartClkSrcSel == 0ul)
    {
        u32ClkTbl[0] = CLK_GetPCLK4Freq();
    }

    /* Set LPUART baud rate */
    if (u32baudrate != 0ul)
    {
        uint32_t u32Baud_Div;

        u32Baud_Div = LPUART_BAUD_MODE2_DIVIDER((u32ClkTbl[u32LpUartClkSrcSel]) / (u32LpUartClkDivNum + 1ul), u32baudrate);

        if (u32Baud_Div > 0xFFFFul)
        {
            lpuart->BAUD = (LPUART_BAUD_MODE0 | LPUART_BAUD_MODE0_DIVIDER((u32ClkTbl[u32LpUartClkSrcSel]) / (u32LpUartClkDivNum + 1ul), u32baudrate));
        }
        else
        {
            lpuart->BAUD = (LPUART_BAUD_MODE2 | u32Baud_Div);
        }
    }
}


/**
 *    @brief        Read LPUART data
 *
 *    @param[in]    lpuart          The pointer of the specified LPUART module.
 *    @param[in]    pu8RxBuf        The buffer to receive the data of receive FIFO.
 *    @param[in]    u32ReadBytes    The the read bytes number of data.
 *
 *    @return       u32Count Receive byte count
 *
 *    @details      The function is used to read Rx data from RX FIFO and the data will be stored in pu8RxBuf.
 */
uint32_t LPUART_Read(LPUART_T *lpuart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes)
{
    uint32_t  u32Count, u32delayno;
    uint32_t  u32Exit = 0ul;

    for (u32Count = 0ul; u32Count < u32ReadBytes; u32Count++)
    {
        u32delayno = 0ul;

        while (lpuart->FIFOSTS & LPUART_FIFOSTS_RXEMPTY_Msk)  /* Check RX empty => failed */
        {
            u32delayno++;

            if (u32delayno >= 0x40000000ul)
            {
                u32Exit = 1ul;
                break;
            }
            else
            {
            }
        }

        if (u32Exit == 1ul)
        {
            break;
        }
        else
        {
            pu8RxBuf[u32Count] = (uint8_t)lpuart->DAT; /* Get Data from LPUART RX  */
        }
    }

    return u32Count;

}


/**
 *    @brief        Set LPUART line configuration
 *
 *    @param[in]    lpuart          The pointer of the specified LPUART module.
 *    @param[in]    u32baudrate     The register value of baudrate of LPUART module.
 *                                  If u32baudrate = 0, LPUART baudrate will not change.
 *    @param[in]    u32data_width   The data length of LPUART module.
 *                                  - \ref LPUART_WORD_LEN_5
 *                                  - \ref LPUART_WORD_LEN_6
 *                                  - \ref LPUART_WORD_LEN_7
 *                                  - \ref LPUART_WORD_LEN_8
 *    @param[in]    u32parity       The parity setting (none/odd/even/mark/space) of LPUART module.
 *                                  - \ref LPUART_PARITY_NONE
 *                                  - \ref LPUART_PARITY_ODD
 *                                  - \ref LPUART_PARITY_EVEN
 *                                  - \ref LPUART_PARITY_MARK
 *                                  - \ref LPUART_PARITY_SPACE
 *    @param[in]    u32stop_bits    The stop bit length (1/1.5/2 bit) of LPUART module.
 *                                  - \ref LPUART_STOP_BIT_1
 *                                  - \ref LPUART_STOP_BIT_1_5
 *                                  - \ref LPUART_STOP_BIT_2
 *
 *    @return       None
 *
 *    @details      This function use to config LPUART line setting.
 */
void LPUART_SetLineConfig(LPUART_T *lpuart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits)
{
    uint32_t u32UartClkSrcSel = 0ul, u32UartClkDivNum = 0ul;
    uint32_t u32ClkTbl[4] = {0, __LXT, __MIRC, __HIRC};

    if (lpuart == (LPUART_T *)LPUART0)
    {

        /* Get LPUART clock source selection */
        u32UartClkSrcSel = ((CLK->LPUARTSEL & CLK_LPUARTSEL_LPUART0SEL_Msk)) >> CLK_LPUARTSEL_LPUART0SEL_Pos;
        /* Get LPUART clock divider number */
        u32UartClkDivNum = (CLK->LPUARTDIV & CLK_LPUARTDIV_LPUART0DIV_Msk) >> CLK_LPUARTDIV_LPUART0DIV_Pos;
    }

    /* Get PLL clock frequency if LPUART clock source selection is PCLK2 */
    if (u32UartClkSrcSel == 0ul)
    {
        u32ClkTbl[0] = CLK_GetPCLK4Freq();
    }
    else
    {
    }

    /* Set LPUART baud rate */
    if (u32baudrate != 0ul)
    {
        uint32_t u32Baud_Div;

        u32Baud_Div = LPUART_BAUD_MODE2_DIVIDER((u32ClkTbl[u32UartClkSrcSel]) / (u32UartClkDivNum + 1ul), u32baudrate);

        if (u32Baud_Div > 0xFFFFul)
        {
            lpuart->BAUD = (LPUART_BAUD_MODE0 | LPUART_BAUD_MODE0_DIVIDER((u32ClkTbl[u32UartClkSrcSel]) / (u32UartClkDivNum + 1ul), u32baudrate));
        }
        else
        {
            lpuart->BAUD = (LPUART_BAUD_MODE2 | u32Baud_Div);
        }
    }

    /* Set LPUART line configuration */
    lpuart->LINE = u32data_width | u32parity | u32stop_bits;
}


/**
 *    @brief        Set Rx timeout count
 *
 *    @param[in]    lpuart  The pointer of the specified LPUART module.
 *    @param[in]    u32TOC  Rx timeout counter.
 *
 *    @return       None
 *
 *    @details      This function use to set Rx timeout count.
 */
void LPUART_SetTimeoutCnt(LPUART_T *lpuart, uint32_t u32TOC)
{
    /* Set time-out interrupt comparator */
    lpuart->TOUT = (lpuart->TOUT & ~LPUART_TOUT_TOIC_Msk) | (u32TOC);

    /* Set time-out counter enable */
    lpuart->INTEN |= LPUART_INTEN_TOCNTEN_Msk;
}


/**
 *    @brief        Select and configure RS485 function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module.
 *    @param[in]    u32Mode     The operation mode(NMM/AUD/AAD).
 *                              - \ref LPUART_ALTCTL_RS485NMM_Msk
 *                              - \ref LPUART_ALTCTL_RS485AUD_Msk
 *                              - \ref LPUART_ALTCTL_RS485AAD_Msk
 *    @param[in]    u32Addr     The RS485 address.
 *
 *    @return       None
 *
 *    @details      The function is used to set RS485 relative setting.
 */
void LPUART_SelectRS485Mode(LPUART_T *lpuart, uint32_t u32Mode, uint32_t u32Addr)
{
    /* Select LPUART RS485 function mode */
    lpuart->FUNCSEL = LPUART_FUNCSEL_RS485;

    /* Set RS585 configuration */
    lpuart->ALTCTL &= ~(LPUART_ALTCTL_RS485NMM_Msk | LPUART_ALTCTL_RS485AUD_Msk | LPUART_ALTCTL_RS485AAD_Msk | LPUART_ALTCTL_ADDRMV_Msk);
    lpuart->ALTCTL |= (u32Mode | (u32Addr << LPUART_ALTCTL_ADDRMV_Pos));
}



/**
 *    @brief        Write LPUART data
 *
 *    @param[in]    lpuart          The pointer of the specified LPUART module.
 *    @param[in]    pu8TxBuf        The buffer to send the data to LPUART transmission FIFO.
 *    @param[out]   u32WriteBytes   The byte number of data.
 *
 *    @return       u32Count transfer byte count
 *
 *    @details      The function is to write data into TX buffer to transmit data by LPUART.
 */
uint32_t LPUART_Write(LPUART_T *lpuart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes)
{
    uint32_t  u32Count, u32delayno;
    uint32_t  u32Exit = 0ul;

    for (u32Count = 0ul; u32Count != u32WriteBytes; u32Count++)
    {
        u32delayno = 0ul;

        while (lpuart->FIFOSTS & LPUART_FIFOSTS_TXFULL_Msk)  /* Check Tx Full */
        {
            u32delayno++;

            if (u32delayno >= 0x40000000ul)
            {
                u32Exit = 1ul;
                break;
            }
            else
            {
            }
        }

        if (u32Exit == 1ul)
        {
            break;
        }
        else
        {
            lpuart->DAT = pu8TxBuf[u32Count];    /* Send LPUART Data from buffer */
        }
    }

    return u32Count;
}
/**
 *    @brief        Select and configure Automatic Operation function
 *
 *    @param[in]    lpuart      The pointer of the specified LPUART module.
 *    @param[in]    u32TrigSel  The LPUART Automatic Operation Trigger Source.
 *                                - \ref LPUART_AUTOCTL_TRIGSEL_SOFTWARE
 *                                - \ref LPUART_AUTOCTL_TRIGSEL_LPTMR0
 *                                - \ref LPUART_AUTOCTL_TRIGSEL_LPTMR1
 *                                - \ref LPUART_AUTOCTL_TRIGSEL_TTMR0
 *                                - \ref LPUART_AUTOCTL_TRIGSEL_TTMR1
 *                                - \ref LPUART_AUTOCTL_TRIGSEL_WKIOA0
 *                                - \ref LPUART_AUTOCTL_TRIGSEL_WKIOB0
 *                                - \ref LPUART_AUTOCTL_TRIGSEL_WKIOC0
 *                                - \ref LPUART_AUTOCTL_TRIGSEL_WKIOD0
 *
 *    @return       None
 *
 *    @details      The function is used to set Automatic Operation relative setting.
 */
void LPUART_SelectAutoOperationMode(LPUART_T *lpuart, uint32_t u32TrigSel)
{
    /* Set Automatic Operation Enable */
    lpuart->AUTOCTL |= LPUART_AUTOCTL_AOEN_Msk;

    /*Set Automatic Operation Clock Always-on*/
    lpuart->AUTOCTL |= LPUART_AUTOCTL_CKAWOEN_Msk;


    // Set Auto Operation mode Trigger source
    lpuart->AUTOCTL &= ~(LPUART_AUTOCTL_TRIGSEL_Msk | LPUART_AUTOCTL_TRIGEN_Msk);
    lpuart->AUTOCTL |=  u32TrigSel;


}


/** @} end of group LPUART_EXPORTED_FUNCTIONS */

/** @} end of group LPUART_Driver */

/** @} end of group Standard_Driver */
