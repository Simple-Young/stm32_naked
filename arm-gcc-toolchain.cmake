# 设置 C 编译器、C++ 编译器、汇编器
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m3)  # 视具体 STM32 型号调整

set(TOOLCHAIN_DIR /home/atlantis/develop/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin)  # 工具链安装路径

# 设置交叉编译器路径
set(TOOLCHAIN_PREFIX arm-none-eabi-)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

# set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXE_LINKER_FLAGS "-nostartfiles -Wl,-nostdlib")
# 可选：设置工具链搜索路径（例如你没把工具链加入 PATH）
# set(CMAKE_FIND_ROOT_PATH /opt/gcc-arm-none-eabi)
