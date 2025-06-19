ENTRY(Reset_Handler)

MEMORY {
    FLASH (rx) : ORIGIN = 0x08004000, LENGTH = 512K
    RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 64K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

_stack_size = 8K;     /* 2KB栈 */
_heap_size = 20K;      /* 40KB堆 */

SECTIONS {
    .isr_vector : {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH
    
    .text : {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        . = ALIGN(4);
        _etext = .;
    } >FLASH
    
    .data : {
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT>FLASH
    
    _sidata = LOADADDR(.data);

    .bss : {
        _sbss = .;
        *(.bss)
        *(.bss*)
        . = ALIGN(4);
        _ebss = .;
    } >RAM

    .heap (NOLOAD): ALIGN(4) {
        _sheap = .;
        . = . + _heap_size;
        . = ALIGN(4);
        _eheap = .;
    } >RAM

    .stack (COPY): ALIGN(4) {
        __StackLimit = .;
        . = . + _stack_size;
        __StackTop = .;
    } > RAM

    . = ALIGN(4);
}