// Mock ESP32 APIs for native Linux compilation
#pragma once
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>

// PSRAM allocation → regular malloc
#define MALLOC_CAP_SPIRAM 0
#define MALLOC_CAP_8BIT   0
static inline void* heap_caps_malloc(size_t sz, int) { return malloc(sz); }
static inline void* heap_caps_calloc(size_t n, size_t sz, int) { return calloc(n, sz); }
static inline void* heap_caps_realloc(void* p, size_t sz, int) { return realloc(p, sz); }
static inline void  heap_caps_free(void* p) { free(p); }

// Display constants matching firmware
#define DISP_W 320
#define DISP_H 176
#define SCALED_W 410
#define SCALED_H 360
#define RAW_FRAME_BYTES ((size_t)DISP_W * (size_t)DISP_H * 2U)

// Byte swap — __builtin_bswap16 is a GCC/Clang builtin, always available
