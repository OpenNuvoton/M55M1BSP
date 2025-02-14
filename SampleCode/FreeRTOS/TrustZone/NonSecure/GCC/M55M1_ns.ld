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
 *
 * +-----------------------+-------------+-------------+----+-----------------------------------------------------+
 * |  Memory region name   | Base addr   |    Size     |IDAU|  MCC load address + remarks                         |
 * +-----------------------+-------------+-------------+----+-----------------------------------------------------+
 * | ITCM                  | 0x1000_0000 | 0x0001_0000 | NS | 0x1000_0000; 68 kiB.                               |
 * | ITCM                  | 0x0000_0000 | 0x0001_0000 | S  | Secure alias for NS ITCM                            |
 * | FLASH                 | 0x1010_0000 | 0x0020_0000 | NS | 0x1010_0000; 2 MiB                                  |
 * | FLASH                 | 0x0010_0000 | 0x0020_0000 | S  | Secure alias for NS BRAM                            |
 * | SRAM01                | 0x3010_0000 | 0x0010_0000 | NS | 2 banks of 512 kiB; 3cc latency)                    |
 * | SRAM01                | 0x2010_0000 | 0x0010_0000 | S  | Secure alias for NS internal SRAM                   |
 * | DTCM                  | 0x3000_0000 | 0x0002_0000 | NS | 0x3000_0000; 128 kiB.                               |
 * | DTCM                  | 0x2000_0000 | 0x0002_0000 | S  | Secure alias for NS DTCM                            |
 * +-----------------------+-------------+-------------+----+-----------------------------------------------------+
 *
 *  Note: Ethos-U55 can access FLASH, internal SRAM 0,1 => activation buffers and
 *        the model should only be placed in those regions.
 *
 *  Note: Alias regions means that secure and non-secure addresses are mapped to the same physical
 *        memory banks.
 */

__STACK_SIZE = 0x00008000;
__HEAP_SIZE  = 0x00010000;

/* System memory brief */
MEMORY
{
  FLASH  (rx)  : ORIGIN = 0x10200000, LENGTH = 0x00100000
  DTCM   (rwx) : ORIGIN = 0x30010000, LENGTH = 0x00010000
  SRAM01 (rwx) : ORIGIN = 0x30200000, LENGTH = 0x00050000
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
  .text.at_flash :
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

  __exidx_start = .;
  .ARM.exidx.at_flash :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > FLASH
  __exidx_end = .;

  .zero.table.at_flash :
  {
    . = ALIGN(4);
    __zero_table_start__ = .;

    LONG (__bss_start__)
    LONG ((__bss_end__ - __bss_start__)/4) /* Size is in 32-bit words */

    __zero_table_end__ = .;
  } > FLASH

  .copy.table.at_flash :
  {
    . = ALIGN(4);
    __copy_table_start__ = .;

    /* Section to be copied - part 1: any data to be placed in SRAM */
    LONG (__etext)
    LONG (__data_start__)
    LONG ((__data_end__ - __data_start__)/4) /* Size is in 32-bit words */

    __copy_table_end__ = .;
  } > FLASH

  __flash_total = ALIGN(4);

  ASSERT( __flash_total < (ORIGIN(FLASH) + LENGTH(FLASH)), "FLASH overflow")

  /**
   * Location counter can end up 2byte aligned with narrow Thumb code but
   * __etext is assumed by startup code to be the LMA of a section in DTCM
   * which must be 4byte aligned
   */
  __etext = ALIGN (4);

  .data :  AT (__etext)
  {
    __data_start__ = .;
    *(vtable)
    *(.data)
    *(.data.*)
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP(*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array))
    PROVIDE_HIDDEN (__fini_array_end = .);
    KEEP(*(.jcr*))
    . = ALIGN(4);

    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
    __data_end__ = .;
  } > SRAM01

  .bss :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > SRAM01

  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    . = . + __HEAP_SIZE;
    . = ALIGN(8);
    __HeapLimit = .;
  } > SRAM01

  .stack (NOLOAD) :
  {
    . = ALIGN(8);
    __StackLimit = .;
    . = . + __STACK_SIZE;
    . = ALIGN(8);
    __StackTop = .;
  } > SRAM01
  PROVIDE(__stack = __StackTop);
 
  /* Check if data + heap + stack exceeds SRAM limit */
  ASSERT(__StackLimit >= __HeapLimit, "region SRAM overflowed with stack")
}
