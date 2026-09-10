/*----------------------------------------------------------------------------
 * link.ld -- GNU ld linker script for TMS570LS1224, flat memory map.
 *
 * No MPU wrappers in this port, so there's no need to split flash/RAM into
 * separate "privileged kernel" vs "application" regions the way the old,
 * MPU-based port did -- everything just lives in one FLASH region and one
 * RAM region.
 *
 * Memory map:
 *   FLASH  0x00000000  1.25 MB  Vector table (first 32 B) + all code/rodata
 *   STACKS 0x08000000     2 KB  CPU-mode stacks, set up by startup/sys_core.s
 *                                (_coreInitStackPointer_ hardcodes these
 *                                addresses -- must not overlap anything else)
 *   RAM    0x08000800   190 KB  .data / .bss / heap
 *----------------------------------------------------------------------------*/

ENTRY(_c_int00)

MEMORY
{
    FLASH  (rx) : ORIGIN = 0x00000000, LENGTH = 0x00140000
    STACKS (rw) : ORIGIN = 0x08000000, LENGTH = 0x00000800
    RAM    (rw) : ORIGIN = 0x08000800, LENGTH = 0x0002F800
}

SECTIONS
{
    /* Interrupt vector table -- must be at address 0x0. */
    .intvecs :
    {
        KEEP(*(.intvecs))
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >FLASH

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    PROVIDE (end  = _ebss);
    PROVIDE (_end = _ebss);

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
