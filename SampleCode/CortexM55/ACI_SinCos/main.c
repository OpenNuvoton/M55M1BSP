/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demo ACI feature for Sin/Cos function
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "NuMicro.h"
#include "ACI_CDE.h"

#undef PI
#include <math.h>
#include "arm_math.h"

#include "ACI_SinCos.h"

#define ACCUARCY_LOSS (0.000122f)  // 18-bits accuracy loss on Q31
#define RUN_CNT         360

static uint64_t s_u64CPUCycleCount = 0;    /* 64-bit cpu cycle counter */

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

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

/**
 * Enabl CDE function
 */
static void EnableCDE(void)
{
    /* Enable CDE */
    SCB->CPACR = SCB->CPACR | 0xFFFF;
    __DSB();
    __ISB();
}

/**
 * SysTick IRQ handler
 */
void SysTick_Handler(void)
{
    /* Increment the cycle counter based on load value. */
    s_u64CPUCycleCount += SysTick->LOAD + 1;
    __DSB();
    __ISB();
}

/**
 * Gets the current SysTick derived counter value
 */
static uint64_t GetSysTickCycleCount(void)
{
    uint32_t u32SystickVal;
    NVIC_DisableIRQ(SysTick_IRQn);
    u32SystickVal = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);
    return s_u64CPUCycleCount + (SysTick->LOAD - u32SystickVal);
}

/**
 * SysTick initialisation
 */
static int InitSysTick(void)
{
    const uint32_t u32Ticks10ms = SystemCoreClock / 100 + 1;
    int i32Err = 0;

    /* Reset CPU cycle count value. */
    s_u64CPUCycleCount = 0;
    /* Changing configuration for sys tick => guard from being
     * interrupted. */
    NVIC_DisableIRQ(SysTick_IRQn);
    /* SysTick init - this will enable interrupt too. */
    i32Err = SysTick_Config(u32Ticks10ms);
    /* Enable interrupt again. */
    NVIC_EnableIRQ(SysTick_IRQn);

    /* Wait for SysTick to kick off */
    while (!i32Err && !SysTick->VAL)
    {
        __NOP();
    }

    return i32Err;
}


int main(void)
{
    float fDegrees;
    int i32Cnt;
    uint64_t u64StartCycle;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("+-------------------------------------------+\n");
    printf("|    M55M1 ACI SIN/COS samaple code         |\n");
    printf("+-------------------------------------------+\n");

    /* Enable CDE */
    EnableCDE();
    /* Init systick for performance estimation */
    InitSysTick();
    srand(0x67894A);

    printf("+-------------------------------------------+\n");
    printf("|   	SIN function                        |\n");
    printf("+-------------------------------------------+\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("|   Deviation between Cordic Sine and C Sine                          |\n");
    printf("+---------------------------------------------------------------------+\n");

    for (i32Cnt = 0; i32Cnt < RUN_CNT;  i32Cnt ++)
    {
        if (i32Cnt == 0)
        {
            fDegrees = 180.0;
        }
        else if (i32Cnt == 1)
        {
            fDegrees = 179.99;
        }
        else if (i32Cnt == 2)
        {
            fDegrees = 0;
        }
        else if (i32Cnt == 3)
        {
            fDegrees = -179.99;
        }
        else if (i32Cnt == 4)
        {
            fDegrees = -180.0;
        }
        else
        {
            fDegrees = (rand() % 361) + (-180); //random generate (-180 ~ 180) degress
        }

        float fRadians = fDegrees * PI / 180;
        float fCSine;

        float fACISine;

        uint64_t u64CSineCycles;
        uint64_t u64ACISineCycles;

        u64StartCycle = GetSysTickCycleCount();
        fCSine = sinf(fRadians);
        u64CSineCycles = GetSysTickCycleCount() - u64StartCycle;


        u64StartCycle = GetSysTickCycleCount();
        fACISine = sinf_aci(fRadians);
        u64ACISineCycles = GetSysTickCycleCount() - u64StartCycle;

        if ((fabsf(fACISine - fCSine) > ACCUARCY_LOSS))
        {
            printf("Error: Degrees %f, Radians %f, sinf %f, ACI sinf %f \n", fDegrees, fRadians, fCSine, fACISine);
            printf("       ACI sinf - sinf: %f \n", fACISine - fCSine);

            while (1);
        }
        else
        {
            printf("Pass: Degrees %f, Radians %f, sinf %f, ACI sinf %f \n", fDegrees, fRadians, fCSine, fACISine);
            printf("      CRTL sinf spend %llu cycles, ACI sinf spend %llu cycles \n", u64CSineCycles, u64ACISineCycles);
        }

        printf("+---------------------------------------------------------------------+\n");
    }

    printf("+-------------------------------------------+\n");
    printf("|   	COS function                        |\n");
    printf("+-------------------------------------------+\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("|   Deviation between Cordic Cosine and C Cosine                      |\n");
    printf("+---------------------------------------------------------------------+\n");


    for (i32Cnt = 0; i32Cnt < RUN_CNT;  i32Cnt ++)
    {
        if (i32Cnt == 0)
        {
            fDegrees = 180.0;
        }
        else if (i32Cnt == 1)
        {
            fDegrees = 179.99;
        }
        else if (i32Cnt == 2)
        {
            fDegrees = 0;
        }
        else if (i32Cnt == 3)
        {
            fDegrees = -179.99;
        }
        else if (i32Cnt == 4)
        {
            fDegrees = -180.0;
        }
        else
        {
            fDegrees = (rand() % 361) + (-180); //random generate (-180 ~ 180) degress
        }

        float fRadians = fDegrees * PI / 180;
        float fCCosine;

        float fACICosine;

        uint64_t u64CCosineCycles;
        uint64_t u64ACICosineCycles;

        u64StartCycle = GetSysTickCycleCount();
        fCCosine = cosf(fRadians);
        u64CCosineCycles = GetSysTickCycleCount() - u64StartCycle;

        u64StartCycle = GetSysTickCycleCount();
        fACICosine = cosf_aci(fRadians);
        u64ACICosineCycles = GetSysTickCycleCount() - u64StartCycle;


        if ((fabsf(fACICosine - fCCosine) > ACCUARCY_LOSS))
        {
            printf("Error: Degrees %f, Radians %f, cosf %f, ACI cosf %f\n", fDegrees, fRadians, fCCosine, fACICosine);
            printf("       ACI cosf - cosf: %f\n", fACICosine - fCCosine);

            while (1);
        }
        else
        {
            printf("Pass: Degrees %f, Radians %f, cosf %f, ACI cosf %f \n", fDegrees, fRadians, fCCosine, fACICosine);
            printf("      CRTL cosf spend %llu cycles, ACI cosf spend %llu cycles \n", u64CCosineCycles, u64ACICosineCycles);
        }

        printf("+---------------------------------------------------------------------+\n");

    }

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
