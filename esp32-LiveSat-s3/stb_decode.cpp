/*
 * stb_decode.cpp — Progressive JPEG decoder for ESP32 via stb_image
 * Uses a persistent 16KB FreeRTOS worker task to avoid DRAM fragmentation.
 */
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#define STBI_NO_HDR
#define STBI_NO_LINEAR
#define STBI_NO_THREAD_LOCALS
#define STBI_MALLOC(sz)        heap_caps_malloc((sz), MALLOC_CAP_SPIRAM)
#define STBI_REALLOC(p, newsz) heap_caps_realloc((p), (newsz), MALLOC_CAP_SPIRAM)
#define STBI_FREE(p)           heap_caps_free(p)
#include "stb_image.h"

char g_stbLastError[64] = "none";

struct StbDecodeArgs {
    const uint8_t* data;
    size_t len;
    uint16_t* buf;
    int w, h;
    bool swap;
    bool done;
    bool ok;
};

static StbDecodeArgs* volatile s_stbJob = nullptr;
static TaskHandle_t s_stbTask = nullptr;
static SemaphoreHandle_t s_stbJobReady = nullptr;
static SemaphoreHandle_t s_stbJobDone = nullptr;

static void stbDecodeWorker(void*) {
    for (;;) {
        xSemaphoreTake(s_stbJobReady, portMAX_DELAY);
        StbDecodeArgs* a = s_stbJob;
        int dw = 0, dh = 0, ch = 0;
        unsigned char* rgb = stbi_load_from_memory(a->data, (int)a->len, &dw, &dh, &ch, 3);
        if (!rgb || dw != a->w || dh != a->h) {
            snprintf(g_stbLastError, sizeof(g_stbLastError), "%s dw=%d dh=%d",
                     rgb ? "size-mismatch" : stbi_failure_reason(), dw, dh);
            if (rgb) stbi_image_free(rgb);
            a->ok = false;
        } else {
            for (int i = 0; i < a->w * a->h; i++) {
                uint16_t c = ((rgb[i*3] >> 3) << 11) | ((rgb[i*3+1] >> 2) << 5) | (rgb[i*3+2] >> 3);
                a->buf[i] = a->swap ? __builtin_bswap16(c) : c;
            }
            stbi_image_free(rgb);
            a->ok = true;
        }
        a->done = true;
        xSemaphoreGive(s_stbJobDone);
    }
}

static bool ensureStbWorker() {
    if (s_stbTask) return true;
    s_stbJobReady = xSemaphoreCreateBinary();
    s_stbJobDone  = xSemaphoreCreateBinary();
    if (!s_stbJobReady || !s_stbJobDone) return false;
    if (xTaskCreatePinnedToCore(stbDecodeWorker, "stbdec", 16384, nullptr, 1,
                                 &s_stbTask, 1) != pdPASS) {
        s_stbTask = nullptr;
        return false;
    }
    return true;
}

bool decodeProgressiveJpegToSprite(const uint8_t* data, size_t len,
                                    uint16_t* buf, int w, int h, bool swap) {
    if (!data || !buf || len == 0) return false;
    if (!ensureStbWorker()) return false;
    StbDecodeArgs args = {data, len, buf, w, h, swap, false, false};
    s_stbJob = &args;
    xSemaphoreGive(s_stbJobReady);
    xSemaphoreTake(s_stbJobDone, portMAX_DELAY);
    return args.ok;
}

static int s_rgbaW = 0, s_rgbaH = 0;
static uint8_t* s_rgbaResult = nullptr;
static SemaphoreHandle_t s_rgbaJobReady = nullptr;
static SemaphoreHandle_t s_rgbaJobDone = nullptr;
static const uint8_t* s_rgbaData = nullptr;
static size_t s_rgbaLen = 0;

static void stbRgbaWorker(void*) {
    for (;;) {
        xSemaphoreTake(s_rgbaJobReady, portMAX_DELAY);
        int w = 0, h = 0, ch = 0;
        s_rgbaResult = stbi_load_from_memory(s_rgbaData, (int)s_rgbaLen, &w, &h, &ch, 4);
        if (s_rgbaResult) {
            s_rgbaW = w;
            s_rgbaH = h;
        } else {
            snprintf(g_stbLastError, sizeof(g_stbLastError), "png: %s", stbi_failure_reason());
            s_rgbaW = 0;
            s_rgbaH = 0;
        }
        xSemaphoreGive(s_rgbaJobDone);
    }
}

static TaskHandle_t s_rgbaTask = nullptr;

uint8_t* decodePngToRgba(const uint8_t* data, size_t len, int* outW, int* outH) {
    if (!data || len == 0) return nullptr;
    if (!s_rgbaTask) {
        s_rgbaJobReady = xSemaphoreCreateBinary();
        s_rgbaJobDone  = xSemaphoreCreateBinary();
        if (!s_rgbaJobReady || !s_rgbaJobDone) return nullptr;
        if (xTaskCreatePinnedToCore(stbRgbaWorker, "stbpng", 16384, nullptr, 1,
                                     &s_rgbaTask, 1) != pdPASS) {
            s_rgbaTask = nullptr;
            return nullptr;
        }
    }
    s_rgbaData = data;
    s_rgbaLen = len;
    s_rgbaResult = nullptr;
    xSemaphoreGive(s_rgbaJobReady);
    xSemaphoreTake(s_rgbaJobDone, portMAX_DELAY);
    if (outW) *outW = s_rgbaW;
    if (outH) *outH = s_rgbaH;
    return s_rgbaResult;
}

void freeRgba(uint8_t* p) {
    if (p) heap_caps_free(p);
}
