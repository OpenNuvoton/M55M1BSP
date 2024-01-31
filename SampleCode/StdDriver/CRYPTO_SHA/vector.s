;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

;//	.syntax	unified

	.globl	g_u32VectorDataBase
	.globl	g_u32VectorDataLimit

	.align	4
 
	.text
        
g_u32VectorDataBase:
    .incbin "../sha_test_vector"    // OK


g_u32VectorDataLimit:
    .space   4
    
    .end
		
		