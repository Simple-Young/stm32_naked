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