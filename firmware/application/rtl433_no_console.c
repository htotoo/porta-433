#include <stdarg.h>
#include <stdio.h>

/*
 * Silences rtl_433 console output without modifying rtl_433 upstream files.
 * These symbols are only referenced from rtl_433 translation units via
 * per-source compile definitions in CMake.
 */

int rtl433_noop_printf(const char* format, ...) {
    (void)format;
    return 0;
}

int rtl433_noop_fprintf(FILE* stream, const char* format, ...) {
    (void)stream;
    (void)format;
    return 0;
}

int rtl433_noop_vprintf(const char* format, va_list args) {
    (void)format;
    (void)args;
    return 0;
}

int rtl433_noop_vfprintf(FILE* stream, const char* format, va_list args) {
    (void)stream;
    (void)format;
    (void)args;
    return 0;
}

int rtl433_noop_puts(const char* string) {
    (void)string;
    return 0;
}

int rtl433_noop_fputs(const char* string, FILE* stream) {
    (void)string;
    (void)stream;
    return 0;
}

int rtl433_noop_fputc(int character, FILE* stream) {
    (void)stream;
    return character;
}

int rtl433_noop_putchar(int character) {
    return character;
}

void rtl433_noop_perror(const char* string) {
    (void)string;
}