        .syntax unified
        .cpu cortex-m3

        .section    .isr_vector,   "a"
        .word       0x20008000       
        .word       boot_code

        .section    .text
        .global boot_code
        .type boot_code, %function

        .global boot_main
        .type boot_main, %function

boot_code:
    nop
    ldr             r0, =boot_main
    bx              r0
    .end
