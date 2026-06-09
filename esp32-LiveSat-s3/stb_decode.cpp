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

bool decodeProgressiveJpegToSprite(const uint8_t* data, size_t len,
                                    uint16_t* buf, int w, int h, bool swap) {
    if (!data || !buf || len == 0) return false;
    if (!s_stbTask) {
        s_stbJobReady = xSemaphoreCreateBinary();
        s_stbJobDone  = xSemaphoreCreateBinary();
        if (!s_stbJobReady || !s_stbJobDone) return false;
        if (xTaskCreatePinnedToCore(stbDecodeWorker, "stbdec", 16384, nullptr, 1,
                                     &s_stbTask, 1) != pdPASS) {
            s_stbTask = nullptr;
            return false;
        }
    }
    StbDecodeArgs args = {data, len, buf, w, h, swap, false, false};
    s_stbJob = &args;
    xSemaphoreGive(s_stbJobReady);
    xSemaphoreTake(s_stbJobDone, portMAX_DELAY);
    return args.ok;
}
