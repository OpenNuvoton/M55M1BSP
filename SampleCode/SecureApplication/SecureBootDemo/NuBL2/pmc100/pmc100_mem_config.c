//------------------------------------------------------------------------------
// The confidential and proprietary information contained in this file may
// only be used by a person authorised under and to the extent permitted
// by a subsisting licensing agreement from Arm Limited or its affiliates.
//
// (C) COPYRIGHT 2019-2021 Arm Limited or its affiliates.
// ALL RIGHTS RESERVED
//
// This entire notice must be reproduced on all copies of this file
// and copies of this file may only be made by a person if such person is
// permitted to do so under the terms of a subsisting license agreement
// from Arm Limited or its affiliates.
//
//  Release Information : Cortex-M55-r1p1-00rel0
//------------------------------------------------------------------------------

/* Yamin Execution Testbench PMC-100 All Test
 *
 * This test runs all the PMC-100 SW library tests on all memories in the current
 * Yamin configuration."
 *
 */

#include "pmc100_mem_config.h"

#define debug_printf(...)

/* Declaration of an array of memories structures.
 * Each structure contains all the IP Core specific
 * memory and memory protection logic configuration
 * information required by the tests.
 */
const Pmc100MemInfo_type core_mem_pmc100[eMEM_CNT] =
{
    /**********
     * IDATA  *
     **********/
    {
        0x2U,   /* IDATA_OPTIONS */
        131U,   /* IDATA_MCR */
        2047U,  /* IDATA_HADDR*/
        0U,     /* IDATA_LADDR */
        11U,    /* IDATA_ADDRW */
        0U,     /* IDATA_CCW */
        2U,     /* IDATA_BANKS */
        1U,     /* IDATA_BANKSW */
        32U,    /* IDATA_VALID_BITS */
        /* ECC_AR array */
        {
            5U, /* IDATA_ECC_AR */
            0U,
            0U,
            0U
        },
        0U      /* IDATA_CFGR */
    },
    /**********
     * DDATA  *
     **********/
    {
        0x3U,   /* DDATA_OPTIONS */
        131U,   /* DDATA_MCR */
        2047U,  /* DDATA_HADDR */
        0U,     /* DDATA_LADDR */
        11U,    /* DDATA_ADDRW */
        0U,     /* DDATA_CCW */
        4U,     /* DDATA_BANKS */
        2U,     /* DDATA_BANKSW */
        32U,    /* DDATA_VALID_BITS */
        /* ECC_AR array */
        {
            0x12407U,   /* DDATA_ECC_AR0 */
            0x11007U,   /* DDATA_ECC_AR1 */
            0x10007U,   /* DDATA_ECC_AR2 */
            0U
        },
        0U      /* DDATA_CFGR */
    }
};

/******************************************************************************/
/* PMC-100 parameters structure                                                */
/******************************************************************************/
const Pmc100Params_type core_params_pmc100 =
{
    (Pmc100_type *)0xE0046000U, /* Cortex-M55 PMC-100 base register address */
    0x0U,           /* REVAND */
    0x1U,           /* REVISION */
    10U,            /* DEVID_MBWIDTH */
    5U,             /* DEVID_MERWIDTH */
    5U,             /* DEVID_MARWIDTH */
    78U,            /* DEVID_MDWIDTH */
    22U,            /* DEVID_MAWIDTH */
    3U,             /* DEVID_RCOWIDTH */
    1U,             /* DEVID_AOWIDTH */
    1U,             /* DEVID_AIWIDTH */
    3U,             /* DEVID_PDWIDTH */
    14U,            /* DEVID_PROGSIZE */
    4U,             /* DEVID_MCWIDTH */
    0x28551396U,    /* DEVID */
    0xc1059c4U,     /* DEVID1 */
    0x0U,           /* DEVID2 */
    3U,             /* NUM_OF_DATA_WORDS */
    2U,             /* BANK_SEL_WIDTH */
    6U,             /* NUM_OF_MEMORIES */
    0x30ff87feU,    /* CTRL_RW_MASK */
    0xfU,           /* CFGR_RW_MASK */
    0x7dddce7U,     /* MCR_RW_MASK */
    0x3ff07U,       /* AR_RW_MASK */
    0x3ffU,         /* BER_RW_MASK */
    0xfU,           /* PCR_RW_MASK */
    0x3fffffU,      /* HIGHADDR_RW_MASK */
    0x3fffffU,      /* LOWADDR_RW_MASK */
    0x1fU,          /* CADDR_RW_MASK */
    0x3fffffU,      /* RADDR_RW_MASK */
    0x1U,           /* AIR_RW_MASK */
    0x1U,           /* AOR_RW_MASK */
    0x1fU,          /* MER_RW_MASK */
    0x80ff0000U,    /* LCR_RW_MASK */
    0x80ff0000U,    /* LSCR_RW_MASK */
    0xffff0000U     /* TCCR_RW_MASK */
};

/*
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from Arm Limited or its affiliates.
 *
 * (C) COPYRIGHT 2020-2021 Arm Limited or its affiliates.
 * ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from Arm Limited or its affiliates.
 *
 *  Release Information : PMC-100-r0p1-01rel0
 */

/*
 * Function:  Check_Test_result
 * -------------------------------------------------------
 *  Checks that the Test_fail Flag is not set and in case of error prints out
 *  some error information
 *
 *  Parameters:
 *     psCtx: pointer to the library context
 *
 *  returns: 1 test passed
 *           0 test failed
 */
int32_t Check_Test_result(const Pmc100Context_type *psCtx)
{
    Pmc100_type *PMC100 = psCtx->params->PMC100_BASE;
    int32_t rv = PMC100LIB_TEST_PASS;

    // Set Test End to 0
    PMC100->CTRL &= ~(PMC100_CTRL_TE);

    /* CHECK END OF Test */
    if ((PMC100->CTRL & PMC100_CTRL_TF) != 0U)
    {
        /*
         * For SRAM test fails, the SW should calculate the faulty SRAM address and faulty data bit.
         * depends on:
         *   - RADDR / CADDR
         *   - PIPELINE depth
         *   - Number of addr update done by Px
         *   - Use RPR reg
         */
        PMC100->CTRL &= ~(PMC100_CTRL_TF);// Set Test fail to 0
        debug_printf("TEST FAIL, TF=1\n");
        PMC100->MER = 0U;
        rv = PMC100LIB_TEST_FAIL;
    }
    else
    {
        if ((PMC100->MER & psCtx->mer_error) != 0U)
        {
            debug_printf("MER FAIL ! MER=0x%08X\n", PMC100->MER);
            debug_printf("mask=0x%08X\n", ctx->mer_error);
            PMC100->MER = 0U;
            rv = PMC100LIB_TEST_FAIL;
        }
    }

    debug_printf("Test Pass\n");
    return rv;
}

void WaitForTestComplete(const Pmc100_type *PMC100)
{
    volatile int32_t i32Timeout = SystemCoreClock;

    while ((PMC100->CTRL & (PMC100_CTRL_TE | PMC100_CTRL_TF)) == 0U)
    {
        if (i32Timeout-- < 1)
            return ;
    }
}

int32_t CheckTestResultsSync(const Pmc100Context_type *psCtx)
{
    const Pmc100_type *PMC100 = psCtx->params->PMC100_BASE;
    WaitForTestComplete(PMC100);
    return Check_Test_result(psCtx);
}

/*
 * Function:  Exec_RAM_Test
 * -------------------------------------------------------
 *  Programs the last registers and run trigger the beginning of a RAM test
 *  (short_burst, March)
 *  The registers programmed in this function are strictly related to the
 *  memory under test
 *
 * Parameters:
 *      psCtx: pointer to the library context
 *      psMemInfo: structure that stores all the information needed for a particular
 *           memory type
 *      suspend_tccr: enables suspend mode using TCCR register value
 *                  and used as the TCCR.TCCI value.
 *      suspend_tc: enables suspend mode using the TC input signal.
 *      loops_before_suspension: number of loops performed before suspending - 1
 *      addrcd: Address change direction. PMC100_CTRL.ADDRCD value. It effects the
 *              next PMC100_RADDR and PMC100_CADDR address register values as follows:
 *              0 - PMC100_CADDR is changed first. All PMC100_CADDR values are
 *                  accessed before the PMC100_RADDR is changed
 *              1 - PMC100_RADDR is changed first. All PMC100_RADDR values are
 *                  accessed before the PMC100_CADDR is changed.
 *
 *      haddr: high address
 *      laddr: low address
 *
 *  returns: None
 */
int32_t Exec_RAM_Test(const Pmc100Context_type *psCtx,
                      const Pmc100MemInfo_type *psMemInfo,
                      uint32_t suspend_tccr,
                      uint32_t suspend_tc,
                      uint32_t loops_before_suspension,
                      uint32_t addrcd,
                      uint32_t haddr,
                      uint32_t laddr)
{
    Pmc100_type *PMC100 = psCtx->params->PMC100_BASE;
    int32_t rv = PMC100LIB_TEST_PASS;

    /* Program Counter Register */
    PMC100->PCR = 0x00000000U;
    /* MBISTOLERR Register */
    PMC100->MER = 0x00000000U;
    /* MBISTOLCFG */
    PMC100->CFGR = psMemInfo->cfgr;

    /* Main Control Register */
    PMC100->CTRL =
        PMC100_CTRL_STOPF  |
        PMC100_CTRL_TESEN  |
        PMC100_CTRL_TFSEN  |
        PMC100_CTRL_ADDRID |
        PMC100_CTRL_FP1;

    if (addrcd != 0U)
    {
        PMC100->CTRL |= PMC100_CTRL_ADDRCD;
    }
    else
    {
        PMC100->CTRL &= ~PMC100_CTRL_ADDRCD;
    }

    /* Byte enable register */
    PMC100->BER = 0xFFFFFFFFU; // Enable all bytes

    if (suspend_tccr != 0U)
    {
        PMC100->CTRL &= ~PMC100_CTRL_TCSEN;
        PMC100->CTRL |= PMC100_CTRL_TCCEN;
        PMC100->LSCR = 0x80000000U |
                       (loops_before_suspension << 16);
        PMC100->TCCR  = suspend_tccr << 16;
    }
    else if (suspend_tc != 0U)
    {
        PMC100->CTRL &= ~PMC100_CTRL_TCCEN;
        PMC100->CTRL |= PMC100_CTRL_TCSEN;
        PMC100->LSCR = 0x80000000U |
                       (loops_before_suspension << 16);
        PMC100->TCCR = 0U;
    }
    else
    {
        PMC100->CTRL &= ~(PMC100_CTRL_TCCEN | PMC100_CTRL_TCSEN);
        PMC100->LSCR = 0U;
        PMC100->TCCR = 0U;
    }

    /* Memory Control Register */
    PMC100->MCR = psMemInfo->mcr |
                  (psMemInfo->ccw << 18) |
                  ((psMemInfo->addrw - psMemInfo->ccw - 2U) << 22);

    /* High Address Register */
    PMC100->HIGHADDR = haddr;

    /* Low Address Register */
    PMC100->LOWADDR = laddr;

    /* Set starting addresses */
    PMC100->RADDR = laddr; // if addr is incrementing
    PMC100->CADDR = 0U;

    /* Array Register */
    PMC100->AR = psMemInfo->ecc_ar[0];

    /* Loop suspend counter register */
    PMC100->LSCR = 0U;

    /* Data Mask */
    //for (i = 0; i < DATA_WORDS_NUM; i++) {
    //    PMC100->DM[i] = psMemInfo->dm_noecc[i];
    //    PMC100->XM[i] = 0U;
    //}

    /* start test */
    PMC100->CTRL |= PMC100_CTRL_PEEN;

    if ((suspend_tccr == 0U) && (suspend_tc == 0U))
    {
        /* Suspend is disabled. Wait for end of test */
        rv = CheckTestResultsSync(psCtx);
    }

    /* Assumed that the end of test will be indicated by interrupt
     * and interrupt test result will be checked in the interrupt
     * handler.
     */

    return rv;
}

/*
 * Function:  PMC100_Set_Reg_Zero
 * -------------------------------------------------------
 *  Set all PMC100 registers with write permission to a known value equal to 0
 *
 *  Parameters:
 *    psCtx: pointer to the library context
 *
 *  returns: 1 - test pass
 *           0 - test fail
 */
int32_t PMC100_Set_Reg_Zero(const Pmc100Context_type *psCtx)
{
    Pmc100_type *PMC100 = psCtx->params->PMC100_BASE;
    uint32_t i;

    PMC100->CTRL       = 0x00000000U;
    PMC100->MCR        = 0x00000000U;
    PMC100->BER        = 0x00000000U;
    PMC100->PCR        = 0x00000000U;
    PMC100->HIGHADDR   = 0x00000000U;
    PMC100->CADDR      = 0x00000000U;
    PMC100->RADDR      = 0x00000000U;
    PMC100->AIR        = 0x00000000U;
    PMC100->AOR        = 0x00000000U;
    PMC100->MER        = 0x00000000U;
    PMC100->LCR        = 0x00000000U;
    PMC100->AR         = 0x00000000U;
    PMC100->CFGR       = 0x00000000U;
    PMC100->TCCR       = 0x00000000U;
    PMC100->LOWADDR    = 0x00000000U;
    PMC100->LSCR       = 0x00000000U;

    for (i = 0; i < psCtx->params->NUM_OF_DATA_WORDS; i++)
    {
        PMC100->X[i]       = 0x00000000U;
        PMC100->Y[i]       = 0x00000000U;
        PMC100->XM[i]      = 0x00000000U;
        PMC100->DM[i]      = 0x00000000U;
    }

    for (i = 0; i < psCtx->params->DEVID_PROGSIZE; i++)
    {
        PMC100->P[i]         = 0x00000000U;
    }

    return 1;
}

int32_t PMC100_CleanData(YAMIN_PMC100_CFG_Type *psPMC100_Config)
{
    Pmc100_type *psPMC100 = psPMC100_Config->ctx->params->PMC100_BASE;
    int32_t i32TestPass = PMC100LIB_TEST_PASS;

    if (psPMC100_Config->mem->haddr <= psPMC100_Config->mem->laddr)
    {
        debug_printf("Invalid Address Parameters !\n");
        i32TestPass = PMC100LIB_INVPARAM;
    }
    else
    {
        if ((psPMC100_Config->suspend_tccr != 0U) && (psPMC100_Config->suspend_tc != 0U))
        {
            debug_printf("Invalid Suspend Parameters !\n");
            i32TestPass = PMC100LIB_INVPARAM;
        }
    }

    if (i32TestPass == PMC100LIB_TEST_PASS)
    {
        psPMC100->P[0] = 0b0001011100100; // Write, LOOP-LAL, update addr
        psPMC100_Config->ctx->mer_error = 0xffffffffU;
        i32TestPass = Exec_RAM_Test(psPMC100_Config->ctx,
                                    psPMC100_Config->mem,
                                    psPMC100_Config->suspend_tccr,
                                    psPMC100_Config->suspend_tc,
                                    psPMC100_Config->loops_before_suspension,
                                    ADDRCD,
                                    psPMC100_Config->mem->haddr /*& 0x1f*/,
                                    psPMC100_Config->mem->laddr);

    }

    if (i32TestPass != PMC100LIB_TEST_PASS)
        debug_printf("Error occurred in function Exec_RAM_Test !\n");

    return i32TestPass;
}