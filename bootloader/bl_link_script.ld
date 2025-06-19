ENTRY(boot_code)

SECTIONS {
    . = 0x8000000;
    .text : { *(.isr_vector) }
}
