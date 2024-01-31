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
 * This is a simplified picture of the M55A1 memory system.
 * Please refer to the M55A1 Technical Reference Manual for
 * further information.
 *
 * +---------------+   +---------------+  +---------------+
 * | Ethos-U55     |   | Cortex-M55    +--+ ITCM (64 KB)  |
 * |               |   |               |  +---------------+
 * |               |   |               |
 * |               |   |               |  +---------------+
 * |  M1      M0   |   |               +--+ DTCM (128 KB) |
 * +---+-------+---+   +-------+-------+  +---------------+
 *     |       |               |
 *     |   +---+---------------+-------+
 *     |   |   AMBA AXI NIC-400-Lite   |
 *     |   +---+--------+---------+----+
 *     |       |        |         |
 * +---+-------+--+ +---+---+ +---+---+
 * | AMBA AHB     | | SRAM0 | | SRAM2 |
 * +---+----------+ | SRAM1 | +-------+
 *     |            +-------+
 * +---+---+
 * | FLASH |
 * +-------+
 *
 * +-----------------------+-------------+-------------+----+------------------------------------------------+
 * |  Memory region name   | Base addr   |    Size     |IDAU| Description                                    |
 * +-----------------------+-------------+-------------+----+------------------------------------------------+
 * | ITCM                  | 0x0000_0000 | 0x0001_0000 | S  | 64 kiB;                                        |
 * | ITCM                  | 0x1000_0000 | 0x0001_0000 | NS | Non-Secure alias for ITCM                      |
 * | DTCM                  | 0x2000_0000 | 0x0002_0000 | S  | 128 kiB;                                       |
 * | DTCM                  | 0x3000_0000 | 0x0002_0000 | NS | Non-Secure alias for DTCM                      |
 * | SRAM01                | 0x2010_0000 | 0x0010_0000 | S  | 2 banks of 512 kiB; (3cc latency)              |
 * | SRAM01                | 0x3010_0000 | 0x0010_0000 | NS | Non-Secure alias for SRAM01                    |
 * | SRAM2                 | 0x2020_0000 | 0x0005_0000 | S  | 320 kiB                                        |
 * | SRAM2                 | 0x3020_0000 | 0x0005_0000 | NS | Non-Secure alias for SRAM2                     |
 * | LPSRAM                | 0x2031_0000 | 0x0000_2000 | S  | 8 kiB                                          |
 * | LPSRAM                | 0x3031_0000 | 0x0000_2000 | NS | Non-Secure alias for LPSRAM                    |
 * | FLASH                 | 0x0010_0000 | 0x0020_0000 | S  | 2 MiB                                          |
 * | FLASH                 | 0x1010_0000 | 0x0020_0000 | NS | Non-Secure alias for Flash                     |
 * | EBI                   | 0x6000_0000 | 0x0030_0000 | S  | 3 MiB                                          |
 * | EBI                   | 0x7000_0000 | 0x0030_0000 | NS | Non-Secure alias for EBI                       |
 * | SPIM0                 | 0x8000_0000 | 0x0200_0000 | S  | 32 MiB                                         |
 * | SPIM0                 | 0x9000_0000 | 0x0200_0000 | NS | Non-Secure alias for SPIM0                     |
 * +-----------------------+-------------+-------------+----+------------------------------------------------+
 *
 *  Note: Ethos-U55 can access FLASH, internal SRAM 0,1 => activation buffers and
 *        the model should only be placed in those regions.
 *
 *  Note: Alias regions means that secure and non-secure addresses are mapped to the same physical
 *        memory banks.
 */

__STACK_SIZE = 0x00008000;
__HEAP_SIZE  = 0x00008000;

/* System memory brief */
MEMORY
{
  ITCM   (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00020000
  FLASH  (rx)  : ORIGIN = 0x00100000, LENGTH = 0x00200000
  SRAM012 (rw) : ORIGIN = 0x20100000, LENGTH = 0x00150000
  DTCM   (rw) : ORIGIN = 0x20000000, LENGTH = 0x00020000
  HYPERFLASH (rwx) : ORIGIN = 0x80000000, LENGTH = 0x02000000
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
   
    /* Section to be copied - part 1: RW data for for DTCM */
    LONG (__eitcm)
    LONG (__data_start__)
    LONG ((__data_end__ - __data_start__) / 4)

    /* Section to be copied - part 2: RO code for for ITCM */
    LONG (__etext)
    LONG (__itcm_text_start__)
    LONG ((__itcm_text_end__ - __itcm_text_start__ ) / 4)

    __copy_table_end__ = .;
  } > FLASH

  /**
   * Location counter can end up 2byte aligned with narrow Thumb code but
   * __etext is assumed by startup code to be the LMA of a section 
   * which must be 4byte aligned
   */
  __etext = ALIGN (4);

  .itcm : AT (__etext)
  {
    __itcm_text_start__ = .;
    *(itcm)
    __itcm_text_end__ = .;
  } > ITCM

  __eitcm = __etext + (__itcm_text_end__ - __itcm_text_start__);

  .data : AT(__eitcm)
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
  } > DTCM

  __edtcm_data = __etext + (__data_end__ - __data_start__);

  .sram012 (NOLOAD):
  {
    . = ALIGN(16);
    *(.bss.NoInit.activation_buf_sram)
    *(.bss.sram.data)
  } > SRAM012 AT > SRAM012

  .bss :
  {
    __data_end__ = .;
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > DTCM AT > DTCM

  .heap (COPY) :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    . = . + __HEAP_SIZE;
    . = ALIGN(8);
    __HeapLimit = .;
  } > DTCM

  .stack (ORIGIN(DTCM) + LENGTH(DTCM) - __STACK_SIZE) (COPY) :
  {
    . = ALIGN(8);
    __StackLimit = .;
    . = . + __STACK_SIZE;
    . = ALIGN(8);
    __StackTop = .;
  } > DTCM
  PROVIDE(__stack = __StackTop);

  /* Check if data + heap + stack exceeds DTCM limit */
  ASSERT(__StackLimit >= __HeapLimit, "region DTCM overflowed with stack")

  .hyperflash :
  {
    . = ALIGN(16);
    *(ifm)
    *(nn_model)
  } > HYPERFLASH AT > HYPERFLASH
}
