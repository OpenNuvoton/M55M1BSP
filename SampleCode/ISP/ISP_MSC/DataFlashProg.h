/******************************************************************************
 * @file     DataFlashProg.h
 * @version  V1.00
 * @brief    Flash programming driver header
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __DATA_FLASH_PROG_H__
#define __DATA_FLASH_PROG_H__

#define BYTE_PER_SEC        512
#define RSVD_SEC_CNT        2
#define ROOT_ENT_CNT        32
#define NUM_FAT             2
#define FAT_SZ              7
#define DATA_SEC_ADDR       ((RSVD_SEC_CNT + FAT_SZ * NUM_FAT + ROOT_ENT_CNT) * BYTE_PER_SEC)

#endif  /* __DATA_FLASH_PROG_H__ */
