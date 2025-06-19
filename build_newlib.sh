#!/bin/zsh

rm -rf build_newlib/*
rm -rf lib/*

cd build_newlib

TARGET=arm-none-eabi
PREFIX=/home/atlantis/develop/code/startup_01/lib

../newlib-cygwin/configure \
    --target=arm-none-eabi \
    --prefix=/home/atlantis/develop/code/startup_01/lib \
    --disable-newlib-supplied-syscalls \
    --disable-libgloss \
    --disable-multilib \
    --disable-nls \
    --enable-lite-exit \
    --enable-newlib-global-atexit \
    --disable-newlib-fvwrite-in-streamio \
    --disable-newlib-fseek-optimization \
    --disable-newlib-wide-orient \
    --enable-newlib-unbuf-stream-opt \
    CFLAGS_FOR_TARGET="-marm" \
    CFLAGS_FOR_TARGET="-g -O0"

# make all-target-newlib -j$(nproc)
# make install-target-newlib