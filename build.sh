#!/bin/zsh

rm -rf build/*

cmake -DCMAKE_TOOLCHAIN_FILE=arm-gcc-toolchain.cmake -B build

cd build

make -j$(nproc) all

# Merge the bootloader and application hex files
#如果存在app/app.hex  和 bootloader/bootloader.hex
if  [[ ! -f app/app.hex || ! -f bootloader/bootloader.hex ]]; then
    echo "Error: Required hex files not found."
    exit 1
fi

srec_cat bootloader/bootloader.hex -Intel app/app.hex -Intel -o merge.hex -Intel
cd ..

JLinkExe -device STM32F103VE -if SWD -speed 4000 -autoconnect 1 -CommanderScript flash.jlink