cmake_minimum_required(VERSION 3.13)
project(bootloader C ASM)


set(BOOTLOADER_SRCS
    bl_main.c
    bl_startup.s
)

add_executable(bootloader.elf ${BOOTLOADER_SRCS})

target_link_options(bootloader.elf PRIVATE
    ${ld_cmd}
    -T${CMAKE_CURRENT_SOURCE_DIR}/bl_link_script.ld
)

target_include_directories(bootloader.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_compile_options(bootloader.elf PRIVATE
    ${common_compile_options}
)


#设置生成elf的路径
set_target_properties(bootloader.elf PROPERTIES
    OUTPUT_NAME "bootloader"
    SUFFIX ".elf"
)
# 设置生成bin的路径
add_custom_command(TARGET bootloader.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:bootloader.elf> ${CMAKE_CURRENT_BINARY_DIR}/bootloader.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:bootloader.elf>
    COMMENT "Generating bootloader.bin"
)
# 设置生成hex的路径
add_custom_command(TARGET bootloader.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:bootloader.elf> ${CMAKE_CURRENT_BINARY_DIR}/bootloader.hex
    COMMENT "Generating bootloader.hex"
)

# add_custom_target(bootloader_hex ALL
#     DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/bootloader.hex
# )