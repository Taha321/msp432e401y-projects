ENTRY(Reset_Handler)
MEMORY
{
    FLASH(rx) :ORIGIN =0x00000000,LENGTH =1024k
    SRAM(rwx) :ORIGIN =0x20000000, LENGTH =256k
}

SECTIONS
{
    .text :
    {
        *(.isr_vector)
        *(.text)
        *(.rodata)
        . = ALIGN(4);
        _etext = .;
    }> FLASH

    _la_data = LOADADDR(.data);

    .data :
    {
        _sdata = .;
        *(.data)
        . = ALIGN(4);
        _edata = .;

    } >SRAM AT> FLASH

    .bss :
    {
        _sbss = .;
        *(.bss)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;

    }> SRAM
}