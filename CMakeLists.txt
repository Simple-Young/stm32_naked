cmake_minimum_required(VERSION 3.10)
project(stm32f103xE NONE)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -nostdlib -nodefaultlibs -nostartfiles")

set(ld_cmd "-Wl,--trace")
set(common_compile_options
    -mcpu=cortex-m3
    -mthumb
    -c
    -g
    -Wextra
    -std=gnu11
    -ffunction-sections
    -fdata-sections
    -nostdlib
    -lgcc
)

add_subdirectory(bootloader)
add_subdirectory(app)

