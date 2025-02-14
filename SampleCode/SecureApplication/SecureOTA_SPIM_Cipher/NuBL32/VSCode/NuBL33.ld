/*
 * Copyright (c) 2009-2021 Arm Limited. All rights reserved.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * +-----------------------+-------------+-------------+----+----------------------------------+
 * |  Memory region name   | Base addr   |    Size     |IDAU| Description                      |
 * +-----------------------+-------------+-------------+----+----------------------------------+
 * | ITCM                  | 0x0000_0000 | 0x0001_0000 | S  | 64 kiB;                          |
 * | ITCM                  | 0x1000_0000 | 0x0001_0000 | NS | Non-Secure alias for ITCM        |
 * | DTCM                  | 0x2000_0000 | 0x0002_0000 | S  | 128 kiB;                         |
 * | DTCM                  | 0x3000_0000 | 0x0002_0000 | NS | Non-Secure alias for DTCM        |
 * | SRAM01                | 0x2010_0000 | 0x0010_0000 | S  | 2 banks of 512 kiB;              |
 * | SRAM01                | 0x3010_0000 | 0x0010_0000 | NS | Non-Secure alias for SRAM01      |
 * | SRAM2                 | 0x2020_0000 | 0x0005_0000 | S  | 320 kiB                          |
 * | SRAM2                 | 0x3020_0000 | 0x0005_0000 | NS | Non-Secure alias for SRAM2       |
 * | SRAM3                 | 0x2030_0000 | 0x0000_2000 | S  | 8 kiB                            |
 * | SRAM3                 | 0x3030_0000 | 0x0000_2000 | NS | Non-Secure alias for SRAM3       |
 * | LPSRAM                | 0x2031_0000 | 0x0000_2000 | S  | 8 kiB                            |
 * | LPSRAM                | 0x3031_0000 | 0x0000_2000 | NS | Non-Secure alias for LPSRAM      |
 * | FLASH                 | 0x0010_0000 | 0x0020_0000 | S  | 2 MiB                            |
 * | FLASH                 | 0x1010_0000 | 0x0020_0000 | NS | Non-Secure alias for Flash       |
 * | EBI                   | 0x6000_0000 | 0x0030_0000 | S  | 3 MiB                            |
 * | EBI                   | 0x7000_0000 | 0x0030_0000 | NS | Non-Secure alias for EBI         |
 * | Aliased SRAM01        | 0x81F0_0000 | 0x0010_0000 | S  | 2 banks of 512 kiB;              |
 * | Aliased SRAM01        | 0x91F0_0000 | 0x0010_0000 | NS | Non-Secure alias for SRAM01      |
 * | SPIM0                 | 0x8200_0000 | 0x0200_0000 | S  | 32 MiB                           |
 * | SPIM0                 | 0x9200_0000 | 0x0200_0000 | NS | Non-Secure alias for SPIM0       |
 * +-----------------------+-------------+-------------+----+----------------------------------+
 *
 *  Note: Ethos-U55 can access FLASH, internal SRAM 0,1 => activation buffers and
 *        the model should only be placed in those regions.
 *
 *  Note: Alias regions means that secure and non-secure addresses are mapped to the same physical
 *        memory banks.
 */

__STACK_SIZE = 0x00000800;
__HEAP_SIZE  = 0x00000800;
/* 0x400 is secure image header size */
__SIGNED_HDR_SIZE = 0x400;

/* System memory brief */
MEMORY
{
  FLASH  (rx)  : ORIGIN = (0x10100000 + 0x80000 + __SIGNED_HDR_SIZE), LENGTH = (0x00180000 - __SIGNED_HDR_SIZE)
  SRAM   (rwx) : ORIGIN = (0x30100000 + 0x100000), LENGTH = 0x00050000
}


/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions ITCM and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __copy_table_start__
 *   __copy_table_end__
 *   __zero_table_start__
 *   __zero_table_end__
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 */

ENTRY(Reset_Handler)

SECTIONS
{
  .text :
  {
    KEEP(*(.vectors))
    *startup_*.o(.text*)

    *(.text*)

    KEEP(*(.init))
    KEEP(*(.fini))

    /* .ctors */
    *crtbegin.o(.ctors)
    *crtbegin?.o(.ctors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
    *(SORT(.ctors.*))
    *(.ctors)

    /* .dtors */
    *crtbegin.o(.dtors)
    *crtbegin?.o(.dtors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
    *(SORT(.dtors.*))
    *(.dtors)

    *(.rodata*)

    KEEP(*(.eh_frame*))
  } > FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > FLASH

  .zero.table :
  {
    . = ALIGN(4);
    __zero_table_start__ = .;
    LONG (__bss_start__)
    LONG ((__bss_end__ - __bss_start__) / 4)
    __zero_table_end__ = .;
  } > FLASH

  .copy.table :
  {
    . = ALIGN(4);
    __copy_table_start__ = .;

    /* Section to be copied - part 1: RW data for SRAM */
    LONG (__etext)
    LONG (__data_start__)
    LONG ((__data_end__ - __data_start__) / 4)

    __copy_table_end__ = .;
  } > FLASH

  /**
   * Location counter can end up 2 byte aligned with narrow Thumb code but
   * __etext is assumed by startup code to be the LMA of a section
   * which must be 4byte aligned
   */
  __etext = ALIGN (4);

  .data : AT(__etext)
  {
    __data_start__ = .;
    *(vtable)
    *(.data)
    *(.data.*)

    . = ALIGN(4);
    /* preinit data */
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP(*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);

    . = ALIGN(4);
    /* init data */
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    PROVIDE_HIDDEN (__init_array_end = .);

    . = ALIGN(4);
    /* finit data */
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array))
    PROVIDE_HIDDEN (__fini_array_end = .);

    KEEP(*(.jcr*))
    . = ALIGN(4);
    /* All data end */
    __data_end__ = .;
  } > SRAM
  
  .sram :
  {
    . = ALIGN(16);
    *(.bss.tensor_arena)
  } > SRAM

  .bss :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > SRAM AT > SRAM

  __esram_data = __etext + (__data_end__ - __data_start__);

  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    . = . + __HEAP_SIZE;
    . = ALIGN(8);
    __HeapLimit = .;
  } > SRAM

  .stack (ORIGIN(SRAM) + LENGTH(SRAM) - __STACK_SIZE) (NOLOAD) :
  {
    . = ALIGN(8);
    __StackLimit = .;
    . = . + __STACK_SIZE;
    . = ALIGN(8);
    __StackTop = .;
  } > SRAM
  PROVIDE(__stack = __StackTop);

  /* Because stack placed in DTCM, cannot check if data + heap + stack exceeds SRAM limit */
  /*ASSERT(__StackLimit >= __HeapLimit, "region SRAM overflowed with stack")*/
}
