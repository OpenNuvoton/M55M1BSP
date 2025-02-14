;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.                                       */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

	.syntax	unified
	.arch	armv8-m.base

	.globl	LDROM_IMAGE_BASE
	.globl	LDROM_IMAGE_LIMIT

	.align	4

	.text

LDROM_IMAGE_BASE:
    .incbin  "FMC_IAP_LDROM.bin"
LDROM_IMAGE_LIMIT:
    .space   4

    .end
