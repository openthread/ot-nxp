/*
** ###################################################################
**     Processors:          KW45B41Z83AFPA
**                          KW45B41Z83AFTA
**
**     Compiler:            GNU C Compiler
**     Reference manual:    KW45B41, Rev. 0 Draft F, July 2020
**     Version:             rev. 1.0, 2020-05-12
**     Build:               b201013
**
**     Abstract:
**         Linker file for the GNU C Compiler
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2021 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
** ###################################################################
*/

GROUP (
  libm.a
)

/* Entry Point */
ENTRY(Reset_Handler)

m_flash_start                       = 0x00000000;
m_flash_end                         = 0x000FFFFF;
m_flash_size                        = 0x00100000;
m_sector_size                       = 0x2000;
m_vector_table_size                 = 0x400;

m_interrupts_start                  = m_flash_start;
m_interrupts_end                    = m_interrupts_start + m_vector_table_size - 1;
__VECTOR_TABLE                      = m_interrupts_start;

m_flash_config_start                = 0x00000400;
m_flash_config_size                 = 0x00000010;
m_flash_config_end                  = m_flash_config_start + m_flash_config_size - 1;

m_fsl_prodInfo_size                 = m_sector_size;
m_fsl_prodInfo_start                = m_flash_end - m_fsl_prodInfo_size + 1;
m_fsl_prodInfo_end                  = m_flash_end;
FREESCALE_PROD_DATA_BASE_ADDR       = m_fsl_prodInfo_start;
PROD_DATA_BASE_ADDR                 = m_fsl_prodInfo_start;

m_ot_settings_page_size             = m_sector_size;
m_ot_settings_page_count            = DEFINED(gUseOtSettingsStorage_d) ? 2 : 0;
m_ot_settings_size                  = DEFINED(gUseOtSettingsStorage_d) ? m_ot_settings_page_size * m_ot_settings_page_count : 0;
m_ot_settings_start                 = m_fsl_prodInfo_start - m_ot_settings_size;
m_ot_settings_end                   = m_fsl_prodInfo_start - 1;

NV_STORAGE_MAX_SECTORS              = DEFINED(gUseNVMLink_d) ? 4 : 0;
NV_STORAGE_SECTOR_SIZE              = m_sector_size;
NV_STORAGE_END_ADDRESS              = m_ot_settings_start - 1;
m_nvm_storage_size                  = NV_STORAGE_MAX_SECTORS * NV_STORAGE_SECTOR_SIZE;
NV_STORAGE_START_ADDRESS            = NV_STORAGE_END_ADDRESS - m_nvm_storage_size + 1;
NV_VIRTUAL_PAGE_BOUND               = NV_STORAGE_END_ADDRESS - (NV_STORAGE_MAX_SECTORS / 2 * NV_STORAGE_SECTOR_SIZE);

INT_STORAGE_END                     = NV_STORAGE_START_ADDRESS - 1;
INT_STORAGE_SIZE                    = DEFINED(gUseInternalStorageLink_d) ? m_sector_size * ( ( (INT_STORAGE_END + 1 - m_interrupts_start) / 2 ) / m_sector_size + 1) : 0;
INT_STORAGE_START                   = INT_STORAGE_END - INT_STORAGE_SIZE + 1;
INT_STORAGE_SECTOR_SIZE             = m_sector_size;

m_text_start                        = m_flash_config_end + 1;
m_text_end                          = INT_STORAGE_START - 1;
m_text_size                         = m_text_end - m_text_start;

m_ram0_start                        = 0x14000000;
m_ram0_end                          = 0x14003FFF;
m_ram1_start                        = 0x20000000;
m_ram1_end                          = 0x2001BFFF;
_RAM_START_                         = m_ram1_start;
_RAM_END_                           = m_ram1_end;
m_ram_vector_table_size             = m_vector_table_size;
HEAP_SIZE                           = DEFINED(__heap_size__)  ? __heap_size__  : 0x0800;
STACK_SIZE                          = DEFINED(__stack_size__) ? __stack_size__ : 0x01000;

m_data0_start                       = m_ram0_start;
m_data0_end                         = m_ram0_end;
m_data0_size                        = m_ram0_end - m_data0_start + 1;

m_cstack_start                      = m_ram1_start;
m_cstack_end                        = m_cstack_start + STACK_SIZE - 1;

m_interrupts_ram_start              = m_cstack_end + 1;
m_interrupts_ram_end                = m_interrupts_ram_start + m_ram_vector_table_size - 1;
__VECTOR_RAM                        = m_interrupts_ram_start;
__RAM_VECTOR_TABLE_SIZE             = m_ram_vector_table_size;

m_data1_start                       = m_interrupts_ram_end + 1;
m_data1_end                         = m_ram1_end;
m_data1_size                        = m_data1_end - m_data1_start + 1;

__HEAP_start__                      = m_data1_end - HEAP_SIZE;
__HEAP_end__                        = m_data1_end;

firmware_ram_upper_limit         = m_data1_end;
firmware_ram_lower_limit         = m_data1_start;
m_warmboot_stack_end             = m_ram1_start;

m_sqram_btblebuf_size = 0x6300; /* was 0x7700 but was reduced to give space to sh_mem and ext_log - can be readjusted if needed*/
                                /*Increased from 0x5400 to 0x6300 to hold 12 additionnal TX data buffers to support 24 connections (pairing)*/
rpmsg_sh_mem_size = 0x1800;
rpmsg_sh_mem_end = 0x489C9FFF;
rpmsg_sh_mem_start = rpmsg_sh_mem_end - rpmsg_sh_mem_size + 1;

gLowpowerPowerDownEnable_d = 0;

/* Specify the memory areas */
MEMORY
{
  m_interrupts          (RX)  : ORIGIN = m_interrupts_start, LENGTH = m_vector_table_size
  m_flash_config        (RX)  : ORIGIN = m_flash_config_start, LENGTH = m_flash_config_size
  m_text                (RX)  : ORIGIN = m_text_start, LENGTH = m_text_size
  m_internal_storage    (RX)  : ORIGIN = INT_STORAGE_START, LENGTH = INT_STORAGE_SIZE
  fNVM_region           (RX)  : ORIGIN = NV_STORAGE_START_ADDRESS, LENGTH = m_nvm_storage_size / 2
  sNVM_region           (RX)  : ORIGIN = (NV_VIRTUAL_PAGE_BOUND + 1), LENGTH = m_nvm_storage_size / 2
  m_ot_settings         (RX)  : ORIGIN = m_ot_settings_start, LENGTH = m_ot_settings_size
  m_fsl_prod_info       (RX)  : ORIGIN = m_fsl_prodInfo_start, LENGTH = m_fsl_prodInfo_size
  m_data0               (RW)  : ORIGIN = m_data0_start, LENGTH = m_data0_size
  m_cstack              (RW)  : ORIGIN = m_cstack_start, LENGTH = STACK_SIZE
  m_ram_interrupts      (RW)  : ORIGIN = m_interrupts_ram_start, LENGTH = m_ram_vector_table_size
  m_data1               (RW)  : ORIGIN = m_data1_start, LENGTH = m_data1_size
  rpmsg_sh_mem		      (RW)  : ORIGIN = rpmsg_sh_mem_start, LENGTH = rpmsg_sh_mem_size
  m_nbu_flash           (RW)  : ORIGIN = 0x48800000, LENGTH = 0x00040000
}

TARGET(binary)
/* Indicate NBU RAM binary to link */
/*
 * in file k32w1.cmake
 * "NBU_BINARY_IMAGE_PATH" is defined for specifying path to connectivity_test.bin binary ... is set to SDK path at this moment:
 * $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/boards/k32w1/connectivity_test_cm3/bm/iar/Debug/Exe/
 */
INPUT(k32w1_nbu_ble_15_4_dyn_1_9_10_0.sb3)
OUTPUT_FORMAT(default)

/* Define output sections */
SECTIONS
{
  .nbu_flash :
  {
    k32w1_nbu_ble_15_4_dyn_1_9_10_0.sb3
  } > m_nbu_flash

  /* NOINIT section for rpmsg_sh_mem */
  .noinit_rpmsg_sh_mem (NOLOAD) : ALIGN(4)
  {
     __RPMSG_SH_MEM_START__ = .;
     *(.noinit.$rpmsg_sh_mem*)
     . = ALIGN(4) ;
     __RPMSG_SH_MEM_END__ = .;
  } > rpmsg_sh_mem

  /* The startup code goes first into internal flash */
  .interrupts :
  {
    __VECTOR_TABLE = .;
    . = ALIGN(4);
    KEEP(*(.isr_vector))     /* Startup code */
    . = ALIGN(4);
  } > m_interrupts

  .flash_config :
  {
    . = ALIGN(4);
    KEEP(*(.FlashConfig))    /* Flash Configuration Field (FCF) */
    . = ALIGN(4);
  } > m_flash_config

  /* The program code and other data goes into internal flash */
  .text :
  {
    . = ALIGN(4);
    *(.text)                 /* .text sections (code) */
    *(.text*)                /* .text* sections (code) */
    *(.rodata)               /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)               /* glue arm to thumb code */
    *(.glue_7t)              /* glue thumb to arm code */
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
  } > m_text

  .NVM_TABLE :
  {
    . = ALIGN(4);
    PROVIDE(__start_NVM_TABLE = .);
    KEEP(*(.NVM_TABLE));
    PROVIDE(__stop_NVM_TABLE = .);
    . = ALIGN(4);
  } > m_text

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > m_text

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > m_text

 .ctors :
  {
    __CTOR_LIST__ = .;
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
       from the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    __CTOR_END__ = .;
  } > m_text

  .dtors :
  {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __DTOR_END__ = .;
  } > m_text

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > m_text

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > m_text

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > m_text

  __etext = .;    /* define a global symbol at end of code */
  __DATA_ROM = .; /* Symbol is used by startup for data initialization */

  .fsl_prod_info :
  {
    . = ALIGN(m_sector_size);
    *(.FREESCALE_PROD_DATA*)
  } > m_fsl_prod_info

  .fEraseNVM :
  {
    . = ALIGN(m_sector_size);
    *(.fEraseNVM)
  } > fNVM_region

  .sEraseNVM :
  {
    . = ALIGN(m_sector_size);
    *(.sEraseNVM*)
  } > sNVM_region

  .data : AT(__DATA_ROM)
  {
    . = ALIGN(4);
    __DATA_RAM = .;
    __data_start__ = .;      /* create a global symbol at data start */
    *(.ramfunc*)             /* for functions in ram */
    *(.data)                 /* .data sections */
    *(.data*)                /* .data* sections */
    KEEP(*(.jcr*))
    . = ALIGN(4);
    __data_end__ = .;        /* define a global symbol at data end */
  } > m_data0

  __DATA_END = __DATA_ROM + (__data_end__ - __data_start__);
  text_end = ORIGIN(m_text) + LENGTH(m_text);
  ASSERT(__DATA_END <= text_end, "region m_text overflowed with text and data")

  .stack :
  {
    . = ALIGN(8);
    . += STACK_SIZE;
  } > m_cstack

  /* Initializes stack on the end of block */
  __StackTop   = m_cstack_end;
  __StackLimit = m_cstack_start;
  PROVIDE(__stack = __StackTop);

  .interrupts_ram :
  {
    . = ALIGN(4);
    __interrupts_ram_start__ = .; /* Create a global symbol at data start */
    *(.m_interrupts_ram)     /* This is a user defined section */
    . = ALIGN(4);
    __interrupts_ram_end__ = .; /* Define a global symbol at data end */
  } > m_ram_interrupts

  /* Uninitialized data section */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    . = ALIGN(4);
    __START_BSS = .;
    __bss_start__ = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    __END_BSS = .;
  } > m_data1

  .heap :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    __HeapBase = .;
    *(.heap)
    __HeapLimit = __HEAP_end__;
  } > m_data1



  .ARM.attributes 0 : { *(.ARM.attributes) }

  /* ASSERT(__StackLimit >= __HeapLimit, "region m_data1 overflowed with stack and heap") */
}
