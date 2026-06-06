/*
 * stb_decode.cpp — Progressive JPEG decoder for ESP32 via stb_image
 * Runs decode in a separate 16KB FreeRTOS task to avoid stack overflow.
 */
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
    volatile bool done;
    volatile bool ok;
};

static void stbDecodeTask(void* param) {
    StbDecodeArgs* a = (StbDecodeArgs*)param;
    int dw = 0, dh = 0, ch = 0;
    unsigned char* rgb = stbi_load_from_memory(a->data, (int)a->len, &dw, &dh, &ch, 3);
    if (!rgb || dw != a->w || dh != a->h) {
        snprintf(g_stbLastError, sizeof(g_stbLastError), "%s dw=%d dh=%d",
                 rgb ? "size-mismatch" : stbi_failure_reason(), dw, dh);
        if (rgb) stbi_image_free(rgb);
        a->ok = false;
        a->done = true;
        vTaskDelete(nullptr);
        return;
    }
    for (int i = 0; i < a->w * a->h; i++) {
        uint16_t c = ((rgb[i*3] >> 3) << 11) | ((rgb[i*3+1] >> 2) << 5) | (rgb[i*3+2] >> 3);
        a->buf[i] = a->swap ? __builtin_bswap16(c) : c;
    }
    stbi_image_free(rgb);
    a->ok = true;
    a->done = true;
    vTaskDelete(nullptr);
}

bool decodeProgressiveJpegToSprite(const uint8_t* data, size_t len,
                                    uint16_t* buf, int w, int h, bool swap) {
    if (!data || !buf || len == 0) return false;
    // Log failure reason to Serial for debugging
    static char s_stbFailReason[64] = {};
    StbDecodeArgs args = {data, len, buf, w, h, swap, false, false};
    TaskHandle_t task = nullptr;
    BaseType_t rc = xTaskCreatePinnedToCore(stbDecodeTask, "stbdec", 16384,
                                             &args, 1, &task, 1);
    if (rc != pdPASS) return false;
    while (!args.done) { vTaskDelay(1); }
    return args.ok;
}
