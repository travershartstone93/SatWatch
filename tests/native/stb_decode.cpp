// stb_image in a proper .cpp file — NOT .ino
// This is how it must be compiled for ESP32 Arduino too.
#include "mock_esp32.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO
#define STBI_NO_HDR
#define STBI_NO_LINEAR
#define STBI_MALLOC(sz)           malloc(sz)
#define STBI_REALLOC(p,newsz)     realloc(p, newsz)
#define STBI_FREE(p)              free(p)
#include "../../esp32-LiveSat-s3/stb_image.h"

// Decode progressive JPEG from memory buffer to RGB565 sprite buffer.
// Returns allocated uint16_t buffer (caller frees), or nullptr on failure.
uint16_t* stb_decode_jpeg_to_rgb565(const uint8_t* jpegData, size_t jpegLen,
                                     int expectedW, int expectedH, bool byteSwap) {
    int w = 0, h = 0, channels = 0;
    unsigned char* rgb = stbi_load_from_memory(jpegData, (int)jpegLen, &w, &h, &channels, 3);
    if (!rgb) {
        fprintf(stderr, "stb_image decode failed: %s\n", stbi_failure_reason());
        return nullptr;
    }
    if (w != expectedW || h != expectedH) {
        fprintf(stderr, "stb_image size mismatch: got %dx%d, expected %dx%d\n", w, h, expectedW, expectedH);
        stbi_image_free(rgb);
        return nullptr;
    }

    uint16_t* buf = (uint16_t*)malloc(w * h * sizeof(uint16_t));
    if (!buf) { stbi_image_free(rgb); return nullptr; }

    for (int i = 0; i < w * h; i++) {
        uint8_t r = rgb[i * 3];
        uint8_t g = rgb[i * 3 + 1];
        uint8_t b = rgb[i * 3 + 2];
        uint16_t c = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
        buf[i] = byteSwap ? __builtin_bswap16(c) : c;
    }

    stbi_image_free(rgb);
    return buf;
}
