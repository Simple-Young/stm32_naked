rm -rf build/*

arm-none-eabi-as -mthumb -g -o build/bl_startup.o bootloader/bl_startup.s
arm-none-eabi-gcc -mcpu=cortex-m3 -c -mthumb -g -Wall -Wextra -std=gnu11 -ffunction-sections -fdata-sections -Os -I bootloader/ -o build/bl_main.o bootloader/bl_main.c
arm-none-eabi-ld -T bootloader/bl_link_script.ld -o build/bootloader.elf build/bl_startup.o build/bl_main.o -Map=build/bootloader.map

arm-none-eabi-objcopy -O binary build/bootloader.elf build/bootloader.bin
arm-none-eabi-objcopy -O ihex build/bootloader.elf build/bootloader.hex
arm-none-eabi-size --format=berkeley build/bootloader.elf
arm-none-eabi-objdump -s -d build/bootloader.elf > build/bootloader.dump
arm-none-eabi-objdump -s -d build/bl_startup.o > build/bl_startup.dump
arm-none-eabi-objdump -D -b binary -m arm build/bootloader.bin > build/bootloader_bin.dump
arm-none-eabi-objdump -D -b ihex -m arm build/bootloader.hex > build/bootloader_hex.dump


arm-none-eabi-as -mthumb -g -o build/startup2.o app/startup2.s
arm-none-eabi-gcc -mcpu=cortex-m3 -c -mthumb -g -Wall -Wextra -std=gnu11 -ffunction-sections -fdata-sections -Os -I app/inc/ -o build/main.o app/main.c
arm-none-eabi-ld -T app/link_script2.ld -o build/app.elf build/startup2.o build/main.o -Map=build/app.map

arm-none-eabi-objcopy -O binary build/app.elf build/app.bin
arm-none-eabi-objcopy -O ihex build/app.elf build/app.hex
arm-none-eabi-size --format=berkeley build/app.elf
arm-none-eabi-objdump -s -d build/app.elf > build/app.dump
arm-none-eabi-objdump -s -d build/startup2.o > build/startup2.dump
arm-none-eabi-objdump -D -b binary -m arm build/app.bin > build/app_bin.dump
arm-none-eabi-objdump -D -b ihex -m arm build/app.hex > build/app_hex.dump


# 合并bootloader和app的bin文件
srec_cat build/bootloader.hex -Intel build/app.hex -Intel -o build/combined.hex -Intel