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

#ifndef PMC100_H
#define PMC100_H

/* This file shall not be modified */

#include <stdint.h>
#include <stdbool.h>
//#include <pmc100_defs.h>

/* PMC100 lib error codes */
#define PMC100LIB_TEST_PASS     1
#define PMC100LIB_TEST_FAIL     0
#define PMC100LIB_INVPARAM      -1
#define PMC100LIB_TEST_SKIP     -2

/* ECC logic test double_error parameter value */
#define DOUBLE_ERROR            1U
#define SINGLE_ERROR            0U

/* The bits definition of 'options' field of Pmc100MemInfo_type structure */
#define PMC100_OPT_HAS_REPAIR   0x00000000U /* repair logic is present in the current mem ecc logic */
#define PMC100_OPT_HAS_CORK     0x00000002U /* cork is implemented in the curren mem */


#define PMC100_CONST            const
/* IO definitions (access restrictions to peripheral registers)
 *
 * IO Type Qualifiers are used to specify the access to peripheral variables.
 * for automatic generation of peripheral register debug information.
 */

// DEFINE PMC100->CTRL mask bits
#define PMC100_CTRL_BAMEN       0x20000000U // [29]       Bank address mode enable.
#define PMC100_CTRL_DMDIS       0x10000000U // [28]       Data masking disable.
// [27:24] Reserved
#define PMC100_CTRL_TCCEN       0x00800000U // [23]       Test continue counter enable
#define PMC100_CTRL_SRTEEN      0x00400000U // [22]       SW read triggered execution enable
#define PMC100_CTRL_TFPCHKUE    0x00200000U // [21]       Test fail protection error check result with uncorrectable error
#define PMC100_CTRL_NOTRANS     0x00100000U // [20]       No MBIST transaction
#define PMC100_CTRL_PCHKR       0x000C0000U // [19:18]    Protection error check result
#define PMC100_CTRL_PREN        0x00020000U // [17]       MBISTOLPREN signal control
#define PMC100_CTRL_FP3         0x00018000U // [16:15]    Pattern 2'b11 RESERVED
#define PMC100_CTRL_FP2         0x00010000U // [16:15]    Pattern 2'b10 0xa5
#define PMC100_CTRL_FP1         0x00008000U // [16:15]    Pattern 2'b01 0xaa
#define PMC100_CTRL_FP0         0x00000000U // [16:15]    Pattern 2'b00 0xff
// READ only bits         0x00007800U // [14:11]
#define PMC100_CTRL_ADDRID      0x00000400U // [10]       Address increment/decrement control
#define PMC100_CTRL_ADDRCD      0x00000200U // [9]        Address change direction
#define PMC100_CTRL_TFSEN       0x00000100U // [8]        Test failed signal enable
#define PMC100_CTRL_TF          0x00000080U // [7]        Test failed status bit
#define PMC100_CTRL_TESEN       0x00000040U // [6]        Test ended signal enable
#define PMC100_CTRL_TE          0x00000020U // [5]        Test ended status bit
#define PMC100_CTRL_STOPF       0x00000010U // [4]        Stop on failure
#define PMC100_CTRL_EXECO       0x00000008U // [3]        Execute once
#define PMC100_CTRL_TCSEN       0x00000004U // [2]        Test continue signal enable
#define PMC100_CTRL_PES         0x00000002U // [1]        Program execution suspended
#define PMC100_CTRL_PEEN        0x00000001U // [0]        Program execution enable

// DEFINE PMC100->MCR mask bits
#define PMC100_MCR_RCW          0x07c00000U // [26:22] Row counter width
#define PMC100_MCR_CCW          0x001c0000U // [20:18] Column counter width
#define PMC100_MCR_RCOW         0x0003c000U // [17:14] RAM cycles per operation for writes
#define PMC100_MCR_RCOR         0x00003c00U // [13:10] RAM cycles per operation for reads
#define PMC100_MCR_PDP          0x000003e0U // [9:5]   Pipeline depth for protection logic
#define PMC100_MCR_PD           0x0000001fU // [4:0]   Pipeline depth

#define FATAL_ERROR             1 //multiple bit error or memory without repair logic

#define PMC100_ITCTRL           0x00000000U
#define PMC100_CLAIMSET         0x0000000FU
#define PMC100_CLAIMCLR         0x00000000U
#define PMC100_DEVARCH          0x47710A55U
#define PMC100_DEVTYPE          0x00000055U
#define PMC100_AUTHSTATUS       0x00000000U
#define PMC100_PIDR4            0x00000004U
#define PMC100_PIDR5            0x00000000U
#define PMC100_PIDR6            0x00000000U
#define PMC100_PIDR7            0x00000000U
#define PMC100_PIDR0            0x000000BAU
#define PMC100_PIDR1            0x000000B9U
#define PMC100_CIDR0            0x0000000DU
#define PMC100_CIDR1            0x00000090U
#define PMC100_CIDR2            0x00000005U
#define PMC100_CIDR3            0x000000B1U

/* Definition of the memory structure */
typedef struct
{
    const uint32_t options;             /* Memory specific options */
    const uint32_t mcr;                 /* Pipeline depth and cycles per operation */
    const uint32_t haddr;               /* High address */
    const uint32_t laddr;               /* Low address */
    const uint32_t addrw;               /* Address width */
    uint32_t ccw;                       /* RAM column address width. User modifiable */
    const uint32_t banks_number;        /* Number of RAM banks memory consists of */
    const uint32_t bank_width;          /* Width of bank value -1 */
    const uint32_t valid_bits;          /* RAM data field width (not including ECC bits) */

    //    const uint8_t addr_protected_bits;  /* Number of address bits protected by ECC scheme */
    //    const uint32_t dm_ecc[8];           /* Data mask with ECC fields */
    //    const uint32_t dm_noecc[8];         /* Data mask without ECC fields */
    //    const uint32_t xm[8];               /* XOR mask */
    //    const uint8_t ecc_num_units;        /* Number of ECC units, ecc_ar elements */

    const uint32_t ecc_ar[4];           /* AR register value for each ECC unit */
    uint32_t cfgr;                      /* CFGR register value, MBISTOLCFG output. User modifiable */
} Pmc100MemInfo_type;

/******************************************************************************/
/*                PMC100 register address offset structure                    */
/******************************************************************************/
typedef struct
{
    __IO  uint32_t CTRL;                        /* Offset 0x000     (RW) Control Register                  */
    __IO  uint32_t MCR;                         /* Offset 0x004     (RW) Memory Control Register           */
    __IO  uint32_t BER;                         /* Offset 0x008     (RW) Byte Enable Register              */
    __IO  uint32_t PCR;                         /* Offset 0x00C     (RW) Program Control Register          */
    __I   uint32_t RPR;                         /* Offset 0x010     (RO) Read Pipeline Register            */
    __IO  uint32_t HIGHADDR;                    /* Offset 0x014     (RW) High Address Register             */
    __IO  uint32_t CADDR;                       /* Offset 0x018     (RW) Column Address Register           */
    __IO  uint32_t RADDR;                       /* Offset 0x01C     (RW) Row Address Register              */
    __IO  uint32_t AIR;                         /* Offset 0x020     (RW) Auxiliary Input Register          */
    __IO  uint32_t AOR;                         /* Offset 0x024     (RW) Auxiliary Output Register         */
    __IO  uint32_t MER;                         /* Offset 0x028     (RW) MBISTOLERR Input Register         */
    __IO  uint32_t LSPR;                        /* Offset 0x02C     (RW) Loop Start Register               */
    __IO  uint32_t LCR;                         /* Offset 0x030     (RW) Loop Counter Register             */
    __IO  uint32_t AR;                          /* Offset 0x034     (RW) Array Register                    */
    __IO  uint32_t CFGR;                        /* Offset 0x038     (RW) MBISTOLCFG Output Register        */
    __IO  uint32_t TCCR;                        /* Offset 0x03C     (RW) Test Continue Counter Register    */
    __IO  uint32_t LOWADDR;                     /* Offset 0x040     (RW) Low Address Register              */
    __IO  uint32_t LSCR;                        /* Offset 0x044     (RW) Loop Suspend Counter Register     */
    __I   uint32_t RESERVED0[14];               /* Offset 0x048-07C                                        */
    __IO  uint32_t X[8];                        /* Offset 0x080-09C (RW) Data Register Xx                  */
    __I   uint32_t RESERVED1[24];               /* Offset 0x0A0-0FC                                        */
    __IO  uint32_t Y[8];                        /* Offset 0x100-11C (RW) Data Register Yx                  */
    __I   uint32_t RESERVED2[24];               /* Offset 0x120-17C                                        */
    __IO  uint32_t DM[8];                       /* Offset 0x180-19C (RW) Data Mask Register DMx            */
    __I   uint32_t RESERVED3[24];               /* Offset 0x1A0-1FC                                        */
    __IO  uint32_t XM[8];                       /* Offset 0x200-21C (RW) XOR mask Register XMx             */
    __I   uint32_t RESERVED4[24];               /* Offset 0x220-27C                                        */
    __I   uint32_t RESERVED5[32];               /* Offset 0x280-2FC                                        */
    __IO  uint32_t P[32];                       /* Offset 0x300-37C (RW) Program Register Px               */
    __I   uint32_t RESERVED6[736];              /* Offset 0x380-EFC                                        */
    __I   uint32_t ITCTRL;                      /* Offset 0xF00     (RO) Integration Mode Control Register */
    __I   uint32_t RESERVED7[39];               /* Offset 0xF04-F9C                                        */
    __IO  uint32_t CLAIMSET;                    /* Offset 0xFA0     (RW) Claim tag set Register            */
    __IO  uint32_t CLAIMCLR;                    /* Offset 0xFA4     (RW) Claim Tag Clear Register          */
    __I   uint32_t DEVAFF0;                     /* Offset 0xFA8     (RO) Device Affinity 0 Register        */
    __I   uint32_t DEVAFF1;                     /* Offset 0xFAC     (RO) Device Affinity 1 Register        */
    __O   uint32_t LAR;                         /* Offset 0xFB0     (WO) Lock Access Register              */
    __I   uint32_t LSR;                         /* Offset 0xFB4     (RO) Lock Status Register              */
    __I   uint32_t AUTHSTATUS;                  /* Offset 0xFB8     (RO) Authentication Status Register    */
    __I   uint32_t DEVARCH;                     /* Offset 0xFBC     (RO) Device Architecture Register      */
    __I   uint32_t DEVID2;                      /* Offset 0xFC0     (RO) Device ID Register                */
    __I   uint32_t DEVID1;                      /* Offset 0xFC4     (RO) Device ID Register                */
    __I   uint32_t DEVID;                       /* Offset 0xFC8     (RO) Device ID Register                */
    __I   uint32_t DEVTYPE;                     /* Offset 0xFCC     (RO) Device Type Register              */
    __I   uint32_t PIDR4;                       /* Offset 0xFD0     (RO) Peripheral ID Register            */
    __I   uint32_t PIDR5;                       /* Offset 0xFD4     (RO) Peripheral ID Register            */
    __I   uint32_t PIDR6;                       /* Offset 0xFD8     (RO) Peripheral ID Register            */
    __I   uint32_t PIDR7;                       /* Offset 0xFDC     (RO) Peripheral ID Register            */
    __I   uint32_t PIDR0;                       /* Offset 0xFE0     (RO) Peripheral ID Register            */
    __I   uint32_t PIDR1;                       /* Offset 0xFE4     (RO) Peripheral ID Register            */
    __I   uint32_t PIDR2;                       /* Offset 0xFE8     (RO) Peripheral ID Register            */
    __I   uint32_t PIDR3;                       /* Offset 0xFEC     (RO) Peripheral ID Register            */
    __I   uint32_t CIDR0;                       /* Offset 0xFF0     (RO) Component ID Register             */
    __I   uint32_t CIDR1;                       /* Offset 0xFF4     (RO) Component ID Register             */
    __I   uint32_t CIDR2;                       /* Offset 0xFF8     (RO) Component ID Register             */
    __I   uint32_t CIDR3;                       /* Offset 0xFFC     (RO) Component ID Register             */
} Pmc100_type;

/******************************************************************************/
/* PMC100 parameters structure                                                */
/*                                                                            */
/* This structure should be populated with CPU specific parameters generated  */
/* during rendering process. This data structure should be the part of PMC    */
/* API library context, thus the parameters would be visible to the APIs.     */
/******************************************************************************/
typedef struct
{
    Pmc100_type *PMC100_BASE;       /* PMC100 base address. User modifiable. */
    const uint32_t REVAND;
    const uint32_t REVISION;
    const uint32_t DEVID_MBWIDTH;   /* MBIST byte enable width */
    const uint32_t DEVID_MERWIDTH;  /* MBIST register and signal width */
    const uint32_t DEVID_MARWIDTH;  /* MBIST array width */
    const uint32_t DEVID_MDWIDTH;   /* MBIST data width */
    const uint32_t DEVID_MAWIDTH;   /* MBIST address width */
    const uint32_t DEVID_RCOWIDTH;  /* RAM cycles of operation field width */
    const uint32_t DEVID_AOWIDTH;   /* AOR register and AUXOUT signal width */
    const uint32_t DEVID_AIWIDTH;   /* AIR register and AUXIN signal width */
    const uint32_t DEVID_PDWIDTH;   /* Pipeline depth field width */
    const uint32_t DEVID_PROGSIZE;  /* Program size */
    const uint32_t DEVID_MCWIDTH;   /* MBIST configuration width */
    const uint32_t DEVID;
    const uint32_t DEVID1;
    const uint32_t DEVID2;
    const uint32_t NUM_OF_DATA_WORDS;
    const uint32_t BANK_SEL_WIDTH;
    const uint32_t NUM_OF_MEMORIES;
    const uint32_t CTRL_RW_MASK;
    const uint32_t CFGR_RW_MASK;
    const uint32_t MCR_RW_MASK;
    const uint32_t AR_RW_MASK;
    const uint32_t BER_RW_MASK;
    const uint32_t PCR_RW_MASK;
    const uint32_t HIGHADDR_RW_MASK;
    const uint32_t LOWADDR_RW_MASK;
    const uint32_t CADDR_RW_MASK;
    const uint32_t RADDR_RW_MASK;
    const uint32_t AIR_RW_MASK;
    const uint32_t AOR_RW_MASK;
    const uint32_t MER_RW_MASK;
    const uint32_t LCR_RW_MASK;
    const uint32_t LSCR_RW_MASK;
    const uint32_t TCCR_RW_MASK;
} Pmc100Params_type;

/* PMC context */
typedef struct
{
    /* The array of structures which contains
     * all the IP Core specific memory and memory
     * protection logic configuration information required
     * by the tests.
     */
    Pmc100MemInfo_type *mem_array;

    /* This structure holds the PMC-100 instance parameter values and other
     * related values used by PMC-100 library.  All the fields in this
     * structure are read only.
     */
    Pmc100Params_type *params;

    uint32_t mer_error;

} Pmc100Context_type;

typedef struct
{
    Pmc100Context_type *ctx;
    Pmc100MemInfo_type *mem;
    uint32_t suspend_tccr;
    uint32_t suspend_tc;
    uint32_t loops_before_suspension;
    uint32_t double_error;
    uint32_t ecc_ar_idx;
    uint32_t dont_save_restore;
    uint32_t bank_start;
    uint32_t bank_end;
    uint32_t *err_mask_pointer;
} YAMIN_PMC100_CFG_Type;

#endif  /* PMC100_H */
