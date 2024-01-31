/**************************************************************************//**
* @file     main.c
* @version  V1.00
* @brief    Demonstrate run anomaly detection using CNN-base autoencoder with NPU
*
* @copyright SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include "board_m55m1.h"
#include "ethosu_npu_init.h"
#include "MPU6500.h"
#include "BufAttributes.h"
#include "NNClassify/MainClassify.h"
using namespace std;
/****************************************************************************
 * User Define Macro
 ****************************************************************************/


/****************************************************************************
 * Global Variables
 ****************************************************************************/

//Systick Related
extern uint32_t SystemCoreClock;        /* Expected to come from the cmsis-device lib */
static uint64_t cpu_cycle_count = 0;            /* 64-bit cpu cycle counter */
volatile int32_t g_u32Ticks = 0;


//For whole system timing
volatile uint32_t g_u32Ticks_start = 0;
volatile uint32_t g_u32Ticks_end = 0;

//Inference event related counter and flag
volatile uint8_t u8TimeUpFlag = 0;
volatile uint8_t s_u8CopygsensorData = 0;

//IMU sensor data buffer
static __attribute__((aligned))int8_t ginTensorData_Quant[ IMU_DATAIN_LEN * IMU_DATAIN_AXES_NUM ] = {0};

//For MPU configuration
extern uint8_t  _tensor_arena[] ;
/****************************************************************************
 * User Code
 ****************************************************************************/
#ifdef  __cplusplus
extern  "C" {
#endif

void SysTick_Handler(void);
void sensor_timer_run(void);
void TIMER0_IRQHandler(void);

#ifdef  __cplusplus
}
#endif
void sensor_timer_run(void)
{
    // Set timer frequency to 1HZ
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 200);//Sets TP sample rate to 200Hz

    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TIMER0_IRQn);

    // Start Timer 0
    TIMER_Start(TIMER0);

}

/*------------------------------------------------------------------------------------------*/
/*  TMR0_IRQHandler                                                                         */
/*------------------------------------------------------------------------------------------*/
void TIMER0_IRQHandler(void)
{
    u8TimeUpFlag = 1;
    // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER0);
}

void SysTick_Handler(void)
{
    /* Increment the cycle counter based on load value. */
    cpu_cycle_count += SysTick->LOAD + 1;
}

uint64_t Get_SysTick_Cycle_Count_Export(void)
{
    uint32_t systick_val;

    NVIC_DisableIRQ(SysTick_IRQn);
    systick_val = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);

    return cpu_cycle_count + (SysTick->LOAD - systick_val);
}

int Init_SysTick_Export(void)
{
    const uint32_t ticks_10ms = SystemCoreClock / 100 + 1;
    int err = 0;

    /* Reset CPU cycle count value. */
    cpu_cycle_count = 0;

    /* Changing configuration for sys tick => guard from being
     * interrupted. */
    NVIC_DisableIRQ(SysTick_IRQn);

    /* SysTick init - this will enable interrupt too. */
    err = SysTick_Config(ticks_10ms);

    /* Enable interrupt again. */
    NVIC_EnableIRQ(SysTick_IRQn);

    /* Wait for SysTick to kick off */
    while (!err && !SysTick->VAL)
    {
        __NOP();
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

    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable NPU module clock */
    CLK_EnableModuleClock(NPU0_MODULE);

    /* Enable GPIO module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable FMC0 module clock to keep FMC clock when CPU idle but NPU running*/
    CLK_EnableModuleClock(FMC0_MODULE);

    /* Enable I2C module clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Enable LPI2C module clock, for G-sensor*/
    CLK_EnableModuleClock(LPI2C0_MODULE);
    /* Enable EBI0 module clock */
    CLK_EnableModuleClock(EBI0_MODULE);

    /* Enable EPWM0 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Enable NPU module clock */
    CLK_EnableModuleClock(NPU0_MODULE);

    /* Enable TMR module clock */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_PCLK1, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_TMRSEL_TMR1SEL_PCLK1, 0);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);


    /* Select UART6 module clock source as HIRC and UART6 module clock divider as 1 */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();


}

/* Cache policy function */
enum { NonCache_index, WTRA_index, WBWARA_index };

static void initializeAttributes()
{
    /* Initialize attributes corresponding to the enums defined in mpu.hpp */
    const uint8_t WTRA =
        ARM_MPU_ATTR_MEMORY_(1, 0, 1, 0); // Non-transient, Write-Through, Read-allocate, Not Write-allocate
    const uint8_t WBWARA = ARM_MPU_ATTR_MEMORY_(1, 1, 1, 1); // Non-transient, Write-Back, Read-allocate, Write-allocate

    ARM_MPU_SetMemAttr(NonCache_index, ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE));
    ARM_MPU_SetMemAttr(WTRA_index, ARM_MPU_ATTR(WTRA, WTRA));
    ARM_MPU_SetMemAttr(WBWARA_index, ARM_MPU_ATTR(WBWARA, WBWARA));
}

static void loadAndEnableConfig(ARM_MPU_Region_t const *table, uint32_t cnt)
{
    initializeAttributes();

    ARM_MPU_Load(0, table, cnt);

    // Enable MPU with default priv access to all other regions
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
}



int main(void)
{

    float cc[IMU_DATAIN_SIZE];
    float err_mae, temp;


    g_u32Ticks = 0;
    s_u8CopygsensorData = 0;
    err_mae = 0;
    temp = 0;
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* UART init - will enable valid use of printf (stdout
    * re-directed at this UART (UART6) */
    InitDebugUart();

    /* Enable IMU sensor interface(LPI2C)*/
    MPU6500_low_levle_init();

    SYS_LockReg();                   /* Unlock register lock protect */

    printf("SystemCoreClock=%u\n", SystemCoreClock);


#if defined(ARM_NPU)

    int state;

    /* If Arm Ethos-U NPU is to be used, we initialise it here */
    if (0 != (state = arm_ethosu_npu_init()))
    {
        return state;
    }

    /* Setup cache poicy of tensor arean buffer */
    printf("Set tesnor arena cache policy to WTRA \n");
    const std::vector<ARM_MPU_Region_t> mpuConfig =
    {
        {
            // SRAM for tensor arena
            ARM_MPU_RBAR(((unsigned int)_tensor_arena),        // Base
                         ARM_MPU_SH_NON,    // Non-shareable
                         0,                 // Read-only
                         1,                 // Non-Privileged
                         1),                // eXecute Never enabled
            ARM_MPU_RLAR((((unsigned int)_tensor_arena) + 0x50000 - 1),        // Limit
                         WTRA_index) // Attribute index - Write-Through, Read-allocate
        }
    };

    // Setup MPU configuration
    loadAndEnableConfig(&mpuConfig[0], mpuConfig.size());
#endif /* ARM_NPU */

    /* Check IMU interface communication*/
    MPU6500_test();
    printf("MPU6500_test finish \n");


    /* Construct the object for inference*/
    MainClassify mainclassify(ginTensorData_Quant);
    printf("MainClassify construct  finish \n");

    /* Enable timer to get IMU dada periodocaaly, say 200Hz*/
    sensor_timer_run();

    /*Looping start here*/
    while (1)
    {

        if (u8TimeUpFlag == 1)
        {
            /*
                Get IMU 3-axis raw data, keep in gsensorBuffer, float type
                If this gsensorBuffer is full(>200x3 samples), then we can do inference
            */
            s_u8CopygsensorData = mainclassify.FillSensorData(0);
            u8TimeUpFlag = 0;

        }

        if (s_u8CopygsensorData == 1)
        {

            Init_SysTick_Export();
            mainclassify.QuantizeInputData();
            mainclassify.FillInTensorData();
            mainclassify.Classify();  // Classify the extracted features
            g_u32Ticks_end = Get_SysTick_Cycle_Count_Export();

            //printf(" g_u32Ticks_end=%d ticks\n", g_u32Ticks_end);
            err_mae = 0;
            temp = 0;

            for (uint16_t i = 0; i < IMU_DATAIN_SIZE; i++)
            {

                cc[i]  = mainclassify.outQuantParams.scale *
                         (float)(((int8_t)(mainclassify.output[i])) - mainclassify.outQuantParams.offset);

                /*MAE computation*/
                temp = (float)(cc[i]) - (mainclassify.gsensorBuffer[i]);

                if (temp < 0) temp = (-temp);

                err_mae += temp;

            }

            err_mae /= IMU_DATAIN_SIZE;

            //printf("mae = %f\r\n",err_mae);
            if (mainclassify.GetAnomalyDetectResult(err_mae))
                printf("ANOMAL! Please put the boad still and face up. \r\n");
            else
                printf(".\r\n");

            s_u8CopygsensorData = 0;
        }

    }

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/