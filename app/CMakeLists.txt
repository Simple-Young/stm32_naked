cmake_minimum_required(VERSION 3.13)
project(app C ASM)

set(libc_dir ${CMAKE_CURRENT_SOURCE_DIR}/libc)


file(GLOB_RECURSE APP_SRCS 
    main.c
    startup.s
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.s

    ${libc_dir}/signal/signal.c
    
    ${libc_dir}/errno/*.c
    ${libc_dir}/reent/reent.c
    ${libc_dir}/reent/impure.c
    ${libc_dir}/reent/readr.c
    ${libc_dir}/reent/lseekr.c
    ${libc_dir}/reent/writer.c
    ${libc_dir}/reent/closer.c
    ${libc_dir}/reent/sbrkr.c
    ${libc_dir}/reent/isattyr.c
    ${libc_dir}/reent/fstatr.c
    ${libc_dir}/reent/signalr.c

    ${libc_dir}/locale/locale.c
    ${libc_dir}/locale/localeconv.c

    ${libc_dir}/ctype/ctype_.c
    
    ${libc_dir}/stdio/stdio.c
    ${libc_dir}/stdio/findfp.c
    ${libc_dir}/stdio/fwalk.c
    ${libc_dir}/stdio/fclose.c
    ${libc_dir}/stdio/fflush.c
    ${libc_dir}/stdio/puts.c
    ${libc_dir}/stdio/wbuf.c
    ${libc_dir}/stdio/wsetup.c
    ${libc_dir}/stdio/makebuf.c
    ${libc_dir}/stdio/putchar.c
    ${libc_dir}/stdio/putc.c
    ${libc_dir}/stdio/printf.c
    ${libc_dir}/stdio/vfprintf.c
    ${libc_dir}/stdio/sfputs_r.c
    ${libc_dir}/stdio/fputc.c
    ${libc_dir}/stdio/fiprintf.c
    ${libc_dir}/stdio/vfiprintf.c
    ${libc_dir}/stdio/viprintf.c
    ${libc_dir}/stdio/viprintfr.c

    "${libc_dir}/string/memset.c"
    "${libc_dir}/string/strcmp.c"
    "${libc_dir}/string/strlen.c"
    "${libc_dir}/string/memchr.c"
    "${libc_dir}/string/memcpy.c"

    ${libc_dir}/stdlib/mallocr.c
    ${libc_dir}/stdlib/malloc.c
    ${libc_dir}/stdlib/freer.c
    ${libc_dir}/stdlib/mlock.c
    # ${libc_dir}/stdlib/local.c
    ${libc_dir}/stdlib/wctomb_r.c
    ${libc_dir}/stdlib/mbtowc_r.c
    ${libc_dir}/stdlib/dtoa.c
    ${libc_dir}/stdlib/mprec.c
    ${libc_dir}/stdlib/assert.c
    ${libc_dir}/stdlib/abort.c
    ${libc_dir}/stdlib/calloc.c
    ${libc_dir}/stdlib/callocr.c
    ${libc_dir}/stdlib/msize.c
    ${libc_dir}/stdlib/msizer.c
    # ${libc_dir}/stdlib/reallocr.c
    # ${libc_dir}/stdlib/strtol.c
    # ${libc_dir}/stdlib/strtoul.c
    # ${libc_dir}/stdlib/mbrtowc.c

    ${libc_dir}/sys/arm/sysconf.c
)

# 过滤掉nano开头的文件
list(FILTER APP_SRCS EXCLUDE REGEX "nano.*\\.c")

include_directories()
link_directories(/usr/lib/gcc/arm-none-eabi/12.2.1)

add_executable(app.elf 
    ${APP_SRCS}
)

add_compile_definitions(_LDBL_EQ_DBL HAVE_MMAP=0 _LIBC HAVE_CONFIG_H __NO_SYSCALLS__ HAVE_SYSCONF_PAGESIZE _HAVE_INIT_FINI)

target_link_options(app.elf PRIVATE
    ${ld_cmd}
    -T${CMAKE_CURRENT_SOURCE_DIR}/link_script.ld
)

target_include_directories(app.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${libc_dir}/include
    ${libc_dir}/include/machine
    ${libc_dir}/include/sys
    ${libc_dir}/include/rpc
    ${libc_dir}/include/ssp
    ${libc_dir}/stdio
    "${libc_dir}/string"
    ${libc_dir}/stdlib
    ${libc_dir}/locale
    ${libc_dir}/ctype
)

target_compile_options(app.elf PRIVATE
    ${common_compile_options}
    -fno-builtin
)

# 添加gcc库
target_link_libraries(app.elf PRIVATE gcc)


#设置生成elf的路径
set_target_properties(app.elf PROPERTIES
    OUTPUT_NAME "app"
    SUFFIX ".elf"
)

# 设置生成bin的路径
add_custom_command(TARGET app.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:app.elf> ${CMAKE_CURRENT_BINARY_DIR}/app.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:app.elf>
    COMMENT "Generating app.bin"
)
# 设置生成hex的路径
add_custom_command(TARGET app.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:app.elf> ${CMAKE_CURRENT_BINARY_DIR}/app.hex
    COMMENT "Generating app.hex"
)

add_custom_command(TARGET app.elf POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} -d --source $<TARGET_FILE:app.elf> > ${CMAKE_CURRENT_BINARY_DIR}/app.lst)

# add_custom_target(app_hex ALL
#     DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/app.hex
# )