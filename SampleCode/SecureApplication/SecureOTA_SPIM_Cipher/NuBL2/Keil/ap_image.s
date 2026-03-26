;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2023 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

.section    .rodata
.global     NUBL32_IMAGE_BASE, NUBL32_IMAGE_LIMIT, NUBL32_IMAGE_SIZE
.global     NUBL33_IMAGE_BASE, NUBL33_IMAGE_LIMIT, NUBL33_IMAGE_SIZE

.align 4
NUBL32_IMAGE_BASE:
.incbin	"./NuBL32_signed.bin"
NUBL32_IMAGE_LIMIT:
NUBL32_IMAGE_SIZE = NUBL32_IMAGE_LIMIT - NUBL32_IMAGE_BASE

.align 4
NUBL33_IMAGE_BASE:
.incbin	"./NuBL33_signed.bin"
NUBL33_IMAGE_LIMIT:
NUBL33_IMAGE_SIZE = NUBL33_IMAGE_LIMIT - NUBL33_IMAGE_BASE
