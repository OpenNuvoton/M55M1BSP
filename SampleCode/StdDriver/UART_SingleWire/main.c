/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Two Single-Wire Loopback data test.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_72MHZ
#define BUFSIZE   128

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[BUFSIZE] = {0};
uint8_t g_u8TxData [BUFSIZE] = {0};
volatile uint32_t g_u32RecLen  =  0;
volatile int32_t  g_i32RecOK  = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART1_TEST_HANDLE(void);
void UART2_TEST_HANDLE(void);
void UART_FunctionTest(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch SCLK clock source to APLL0 and Enable APLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART1 clock source is HIRC and UART1 module clock divider as 1*/
    CLK_SetModuleClock(UART1_MODULE, CLK_UARTSEL0_UART1SEL_HIRC, CLK_UARTDIV0_UART1DIV(1));

    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART2 clock source is HIRC and UART2 module clock divider as 1*/
    CLK_SetModuleClock(UART2_MODULE, CLK_UARTSEL0_UART2SEL_HIRC, CLK_UARTDIV0_UART2DIV(1));

    /* Enable UART2 peripheral clock */
    CLK_EnableModuleClock(UART2_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Enable GPIOB module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* Anti-floating on there RX pin. */
    GPIO_SetPullCtl(PB, BIT0, GPIO_PUSEL_PULL_UP);

    /* Set PA multi-function pins for UART1 RXD */
    SET_UART1_RXD_PA2();

    /* Set PB multi-function pins for UART2 RXD */
    SET_UART2_RXD_PB0();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                               Init Single Wire(UART1)                                                   */
/*---------------------------------------------------------------------------------------------------------*/

void UART1_Init()
{
    /* Configure Single Wire(UART1) and set Single Wire(UART1) baud rate */
    SYS_ResetModule(SYS_UART1RST);
    UART_Open(UART1, 115200);
    UART_SelectSingleWireMode(UART1);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                               Init Single Wire(UART2)                                                   */
/*---------------------------------------------------------------------------------------------------------*/

void UART2_Init()
{
    /* Configure Single Wire(UART2) and set Single Wire(UART2) baud rate */
    SYS_ResetModule(SYS_UART2RST);
    UART_Open(UART2, 115200);
    UART_SelectSingleWireMode(UART2);

}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* Debug port control the Single wire 1(UART1) send data to Single wire 2(UART2) or Single wire 2(UART2)   */
/* send data to Single wire 1(UART1)                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();
    /* Init UART1 */
    UART1_Init();
    /* Init UART2 */
    UART2_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                           SAMPLE CODE                                                   */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("\nUART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                       ISR to handle UART Channel 1 interrupt event                                      */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void UART1_IRQHandler(void)
{
    UART1_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                  UART1 Callback function                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE()
{

    if (UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAIF_Msk))
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART1))
        {
            uint8_t u8Data = UART_READ(UART1);
            /* Get the character from UART Buffer */
            g_u8RecData[g_u32RecLen] = u8Data;

            if (g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    if (UART_GET_INT_FLAG(UART1, UART_INTSTS_SWBEIF_Msk))
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART1, UART_INTSTS_SWBEINT_Msk);
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 2 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void UART2_IRQHandler(void)
{
    UART2_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART2 Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART2_TEST_HANDLE()
{

    if (UART_GET_INT_FLAG(UART2, UART_INTSTS_RDAIF_Msk))
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART2))
        {
            uint8_t u8Data = UART_READ(UART2);
            /* Get the character from UART Buffer */
            g_u8RecData[g_u32RecLen] = u8Data;

            if (g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    if (UART_GET_INT_FLAG(UART2, UART_INTSTS_SWBEIF_Msk))
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART2, UART_INTSTS_SWBEINT_Msk);

    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*                              Bulid Source Pattern function                                              */
/*---------------------------------------------------------------------------------------------------------*/
void Build_Src_Pattern(uint32_t u32Addr, uint8_t type, uint32_t u32Length)
{
    uint32_t i = 0, pattern = 0;
    uint8_t *pAddr;
    pAddr = (uint8_t *)u32Addr;

    if (type == 0)      pattern = 0x1f;
    else if (type == 1) pattern = 0x3f;
    else if (type == 2) pattern = 0x7f;
    else if (type == 3) pattern = 0xff;
    else  pattern = 0xff;

    for (i = 0; i < u32Length ; i++)
        pAddr[i] = (i & pattern);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                    Verify that the received data is correct                                             */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t Check_Pattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length)
{
    uint32_t i = 0;
    uint8_t result = 1;
    uint8_t *pAddr0;
    uint8_t *pAddr1;
    pAddr0 = (uint8_t *)u32Addr0;
    pAddr1 = (uint8_t *)u32Addr1;

    for (i = 0; i < u32Length ; i++)
    {
        if (pAddr0[i] != pAddr1[i])
        {
            printf("Data Error Idex=%d,tx =%d,rx=%d\n", i, pAddr0[i], pAddr1[i]) ;
            result = 0;
        }
    }

    return result;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest()
{
    char cmmd ;

    printf("+-----------------------------------------------------------+\n");
    printf("|            UART Single Wire Function Test                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    The user must connect the UART1 RX pin(PA2) to         |\n");
    printf("|    UART2 Rx Pin(PB0).                                      |\n");
    printf("|    Single Wire 1(PA2)send data to Single Wire 2(PB0).     |\n");
    printf("|    Single Wire 2(PB0)send data to Single Wire 1(PA2).     |\n");
    printf("|    Please enter any to start    (Press '0' to exit)       |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect DEBUG_PORT and PC.DEBUG_PORT is set to debug port.
          UART1 and UART2 is enable RDA and RLS interrupt.
          The user can use URT0 to control the transmission or reception of UART1(Single Wire mode)
        When UART1(Single Wire 1)transfers data to UART2(Single Wire 2), if data is valid,
          it will enter the interrupt and receive the data.And then check the received data.
        When UART2(Single Wire 2)transfers data to UART1(Single Wire 1), if data is valid,
          it will enter the interrupt and receive the data.And then check the received data.
      */

    /* Enable UART1 RDA/Time-out/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    /* Enable UART2 RDA/Time-out/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART2_IRQn);
    UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));

    do
    {
        printf("+--------------------------------------------------------------+\n");
        printf("|                UART Single Wire Test Item                    |\n");
        printf("+--------------------------------------------------------------+\n");
        printf("|    (1)Single Wire 1(PA2)send data to Single Wire 2(PB0).     |\n");
        printf("|    (2)Single Wire 2(PB0)send data to Single Wire 1(PA2).     |\n");
        printf("|    (E)Exit                                                   |\n");
        printf("+--------------------------------------------------------------+\n");

        cmmd = getchar();

        switch (cmmd)
        {
            case '1':
            {
                printf("SW1(UART1) --> SW2(UART2)Test :");
                g_i32RecOK  = FALSE;
                Build_Src_Pattern((uint32_t)g_u8TxData, UART_WORD_LEN_8, BUFSIZE);

                /* Check the Rx status is Idel */
                while (!UART_RX_IDLE(UART1)) {};

                UART_Write(UART1, g_u8TxData, BUFSIZE);

                while (g_i32RecOK != TRUE) {}

                Check_Pattern((uint32_t)g_u8TxData, (uint32_t)g_u8RecData, BUFSIZE) ? printf(" Pass\n") : printf(" Fail\n");
                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_u8TxData, 0, BUFSIZE);
                memset((uint8_t *)g_u8RecData, 0, BUFSIZE);
            }
            break;

            case '2':
            {
                printf("SW2(UART2) --> SW1(UART1)Test :");
                g_i32RecOK  = FALSE;
                Build_Src_Pattern((uint32_t)g_u8TxData, UART_WORD_LEN_8, BUFSIZE);

                /* Check the Rx status is Idel */
                while (!UART_RX_IDLE(UART2)) {};

                UART_Write(UART2, g_u8TxData, BUFSIZE);

                while (g_i32RecOK != TRUE) {};

                Check_Pattern((uint32_t)g_u8TxData, (uint32_t)g_u8RecData, BUFSIZE) ? printf(" Pass\n") :   printf(" Fail\n");

                /* Clear the Tx and Rx data buffer */
                memset((uint8_t *)g_u8TxData, 0, BUFSIZE);

                memset((uint8_t *)g_u8RecData, 0, BUFSIZE);
            }
            break;

            default:
                break;
        }

    } while ((cmmd != 'E') && (cmmd != 'e'));

    /* Disable UART1 RDA/Time-out interrupt */
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    /* Disable UART2 RDA/Time-out interrupt */
    UART_DisableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    printf("\nUART Sample Demo End.\n");

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/


