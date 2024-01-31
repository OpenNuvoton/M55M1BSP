;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2023 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

.section	.rodata
.global		LDROM_IMAGE_BASE, LDROM_IMAGE_LIMIT, LDROM_IMAGE_SIZE
.align		4

LDROM_IMAGE_BASE:
.incbin	"./FMC_IAP_LDROM.bin"
LDROM_IMAGE_LIMIT:
LDROM_IMAGE_SIZE = LDROM_IMAGE_LIMIT - LDROM_IMAGE_BASE
