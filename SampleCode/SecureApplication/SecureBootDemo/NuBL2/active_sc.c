/**************************************************************************//**
 * @file    active_sc.c
 * @version V1.00
 * @brief   Demonstrate how to active the Secure Conceal Function and
 *          clear I/D-Cache with PMC-100 to prevent potential code leakage.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"
#include "pmc100_mem_config.h"

typedef void (*PFN_FUNC)(void);

/* The Secure Conceal Region becomes inaccessible after activation.
 * Therefore, the activation function must be placed outside the Secure Conceal Region.
 * In this example, ActiveSecureConceal will be executed from SRAM. */
void ActiveSecureConceal(uint32_t u32NuBL32Base)
{
    PFN_FUNC pfnNuBL32Entry;

    if (FMC_Read(FMC_USER_CONFIG_6) != 0xFFFFFFFF)
    {
        /* Clear I/D-Cache with PMC-100 before leave BL2 to prevent
         * potential leakage of Secure Conceal code through the cache. */
        int32_t i32MemIdx;
        Pmc100Context_type sPMC100_Ctx =
        {
            .mem_array = (Pmc100MemInfo_type *) &core_mem_pmc100,
            .params    = (Pmc100Params_type *) &core_params_pmc100
        };

        YAMIN_PMC100_CFG_Type sPMC100_Config =
        {
            .ctx = &sPMC100_Ctx,
            .mem = &sPMC100_Ctx.mem_array[0],
            .suspend_tccr = 0,
            .suspend_tc = 0,
            .loops_before_suspension = 0,
            .double_error = 0,
            .ecc_ar_idx = 0,
            .dont_save_restore = 0,
            .bank_start = 0,
            .bank_end = 0,
            .err_mask_pointer = 0
        };

        // Disable I/D-Cache before PMC-100 operation
        SCB_DisableDCache();
        SCB_DisableICache();
        // Deactivate I/D-Cache
        MEMSYSCTL->MSCR &= ~(MEMSYSCTL_MSCR_ICACTIVE_Msk | MEMSYSCTL_MSCR_DCACTIVE_Msk);
        // Ensure all operations are completed
        __ISB();
        __DSB();
        PMC100_Set_Reg_Zero(sPMC100_Config.ctx);

        // Trigger PMC-100 to clear I/D-Cache
        // Loop to clean each memory
        for (i32MemIdx = eMEM_IDATA; i32MemIdx < eMEM_CNT; i32MemIdx++)
        {
            sPMC100_Config.mem          = &sPMC100_Ctx.mem_array[i32MemIdx];
            sPMC100_Config.bank_start   = sPMC100_Config.mem->banks_number - 1;

            PMC100_CleanData(&sPMC100_Config);
        }

        // Activate I/D-Cache
        MEMSYSCTL->MSCR |= (MEMSYSCTL_MSCR_ICACTIVE_Msk | MEMSYSCTL_MSCR_DCACTIVE_Msk);
        PMC100_Set_Reg_Zero(sPMC100_Config.ctx);

        // Ensure all operations are completed
        __ISB();
        __DSB();
        // Activate Secure Conceal Function
        FMC_SET_SC_ACTIVE();
    }

    /* Disable all interrupt */
    __set_PRIMASK(1);

    /* SCB.VTOR points to the NuBL32 vector table base address. */
    SCB->VTOR = u32NuBL32Base;

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    pfnNuBL32Entry = ((PFN_FUNC)(*(((uint32_t *)SCB->VTOR) + 1)));
    /* execute NuBL32 FW */
    pfnNuBL32Entry();
}

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
