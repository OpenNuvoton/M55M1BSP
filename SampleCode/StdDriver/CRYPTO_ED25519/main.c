/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   CRYPTO_ED25519 code for M55M1 series MCU
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "EdDsa.h"

#define TXT_MAX_LEN     8192

volatile uint32_t  g_tick_cnt;

extern int ed25519_test();
int data_compare(uint8_t *answer, uint8_t *cmp_data, int len)
{
    int  i, err;

    err = 0;

    for (i = 0; i < len; i++)
    {
        if (answer[i] != cmp_data[i])
        {
            if (err == 0)
                printf("Data compare error dump:\n");

            printf("[%d]: Answer %02x ! %02x\n", i, answer[i], cmp_data[i]);

            if (err++ > 5)
                break;

        }
    }

    if (err)
    {
        printf("Answer dump ==>\n");
        dump_buff_hex(answer, len);
        printf("Wring data dump ==>\n");
        dump_buff_hex(cmp_data, len);

        //chacha_dump();
        while (1);
    }

    printf("PASS.\n");
    return 0;
}


void do_swap(uint8_t *buff, int len)
{
    int       i;
    uint8_t   val8;

    len = (len + 3) & 0xfffffffc;

    for (i = 0; i < len; i += 4)
    {
        val8 = buff[i];
        buff[i] = buff[i + 3];
        buff[i + 3] = val8;
        val8 = buff[i + 1];
        buff[i + 1] = buff[i + 2];
        buff[i + 2] = val8;
    }
}

void do_swap_to(uint8_t *out, uint8_t *buff, int len)
{
    int       i;

    for (i = 0; i < len; i += 4)
    {
        out[i]   = buff[i + 3];
        out[i + 1] = buff[i + 2];
        out[i + 2] = buff[i + 1];
        out[i + 3] = buff[i];
    }
}

uint64_t EL0_GetCurrentPhysicalValue(void)
{
    return 100;/*Dummy Implementation*/
}


static volatile uint64_t  _start_time = 0;
static volatile int  g_PRNG_done;
extern volatile int  g_CHAPOLY_done;

void random_gen_mass(uint8_t *buff, int count)
{
    uint32_t    *prng_data = (uint32_t *)((uint64_t)CRYPTO_BASE + 0x10);
    int   clen;

    while (count > 0)
    {

        /* Trigger PRNG to generate 32 bytes random data */
        g_PRNG_done = 0;
        CRYPTO->PRNG_CTL = (PRNG_KEY_SIZE_256 << CRYPTO_PRNG_CTL_KEYSZ_Pos) | CRYPTO_PRNG_CTL_START_Msk;

        while (!g_PRNG_done);

        if (count >= 32)
            clen = 32;
        else
            clen = count;

        memcpy(buff, prng_data, clen);

        buff += clen;
        count -= clen;
    }
}

int comapre_hex_dump_err(uint8_t *answer, uint8_t *compare, int len, int err_max)
{
    int  err = 0, k;

    for (k = 0; k < len; k++)
    {
        if (answer[k] != compare[k])
        {
            printf("%d: 0x%02x (Ans) - 0x%02x (Dat)\n", k, answer[k], compare[k]);

            if (err++ > err_max)
                break;
        }
    }

    return err;
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

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);


    /* Enable PLL0 200MHz clock */
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
    CLK_SET_PCLK4DIV(4);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRYPTO0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

}

int main()
{
    SYS_UnlockReg();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();

    NVIC_EnableIRQ(CRYPTO_IRQn);

    SHA_ENABLE_INT(CRYPTO);
    ECC_ENABLE_INT(CRYPTO);
    CRYPTO->PAP_CTL |= CRYPTO_PAP_CTL_PAPEN_Msk;

    printf("+------------------------------------------+\n");
    printf("|   Start ED25519 Example                  |\n");
    printf("+------------------------------------------+\n");


    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");

        while (1);
    }

    printf("\n\nStart ed25519  Sig and Verify!\n");

    ed25519_test();

    printf("\n\ned25519  Sig and Verify Pass!\n");

    /* Got no where to go, just loop forever */
    while (1);
}


void SysTick_Handler(void)
{
    g_tick_cnt++;
    //SendChar_ToUART('#');
}
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

