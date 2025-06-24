#pragma once

#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

void *_sbrk(ptrdiff_t incr);
int _write(int file, char *ptr, int len);
int _read(int file, char *ptr, int len);
int _close(int file);
int _fstat(int file, struct stat *st);
int _isatty(int file);
int _lseek(int file, int ptr, int dir);
int __aeabi_uidiv(unsigned int numerator, unsigned int denominator);
void _exit(int status);
int _kill(int pid, int sig);
int _getpid(void);



