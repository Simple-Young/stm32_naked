#include <stdio.h>
#include "swi.h"
#include "times.h"
#include "sys/time.h"
#include "systemcall.h"

extern const uint8_t _sheap;
extern const uint8_t _eheap;

extern const uint8_t _estack;

extern const uint8_t _sdata;
extern const uint8_t _edata;

extern const uint8_t _sbss;
extern const uint8_t _ebss;

static char *heap_end = (char *)&_eheap;

void *_sbrk(ptrdiff_t incr) {
    char *prev_heap_end = heap_end;
    heap_end += incr;
    return (void *)prev_heap_end;
}

int _write(int file, char *ptr, int len) {
    for(int i = 0; i < len; i++) {
        uart1_send_char(ptr[i]); // 逐个字符发送
    }
    return len;
}

int _read(int file, char *ptr, int len) {
    return -1;
}

int _close(int file) {
    return -1;
}

int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file) {
    return 1;
}

int _lseek(int file, int ptr, int dir) {
    return 0;
}

int __aeabi_uidiv(unsigned int numerator, unsigned int denominator) {
    if (denominator == 0) {
        return 0;
    }
    return numerator / denominator;
}

void _exit(int status) {
    while (1);
}

int _kill(int pid, int sig) {
    return -1;
}

int _getpid(void) {
    return 1; // 固定返回一个伪PID
}

clock_t _times (struct tms *tp){
    clock_t timeval;
    printf("times called\n");
// #ifdef ARM_RDI_MONITOR
// timeval = do_AngelSWI (AngelSWI_Reason_Clock, NULL);
// #else
// register int r0 asm("r0");
// asm ("swi %a1" : "=r" (r0): "i" (SWI_Clock));
// timeval = (clock_t) r0;
// #endif

//     if (tp)
//         {
//         tp->tms_utime  = timeval;	/* user time */
//         tp->tms_stime  = 0;	/* system time */
//         tp->tms_cutime = 0;	/* user time, children */
//         tp->tms_cstime = 0;	/* system time, children */
//         }

    return timeval;

}
/**
 * @brief 获取当前时间
 * @param tp 指向 timeval 结构体的指针，用于存储当前时间
 * @param tzvp 指向 timezone 结构体的指针，用于存储时区信息
 * @return 成功返回 0，失败返回 -1
 */
int _gettimeofday (struct timeval * tp, void * tzvp){
    struct timezone *tzp = tzvp;
    if(tzp) {
        tzp->tz_minuteswest = 8 * 60; // 设置为北京时间（UTC+8）
        tzp->tz_dsttime = 0; // 不使用夏令时
    }
    if (!tp) {
        return -1;
    }
    
    tp->tv_sec = 1000; // 设置为 0 秒
    tp->tv_usec = 1000*1000; // 设置为 0 微秒   

  return 0;

}