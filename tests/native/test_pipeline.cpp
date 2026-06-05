// Native test for the international coverage decode pipeline.
// Tests: stb_image progressive JPEG, validation logic, screen blend composite.
// Compile: make
// Run:    ./test_pipeline

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>

#include "mock_esp32.h"

// stb_decode.cpp provides this
extern uint16_t* stb_decode_jpeg_to_rgb565(const uint8_t* jpegData, size_t jpegLen,
                                            int expectedW, int expectedH, bool byteSwap);

// ─── Validation functions (extracted from firmware) ───

static bool spriteLooksCompletelyBlack(const uint16_t* px, int total) {
    for (int i = 0; i < total; i += 16) {
        if (px[i] != 0x0000) return false;
    }
    return true;
}

static bool colorShiftCheck(const uint16_t* px, int total) {
    uint32_t rSum = 0, gSum = 0, bSum = 0, count = 0;
    for (int i = 0; i < total; i += 8) {
        uint16_t c = px[i];
        rSum += (c >> 11) & 0x1F;
        gSum += (c >> 5) & 0x3F;
        bSum += c & 0x1F;
        count++;
    }
    if (count == 0) return true;
    uint32_t rAvg = rSum * 255 / (count * 31);
    uint32_t gAvg = gSum * 255 / (count * 63);
    uint32_t bAvg = bSum * 255 / (count * 31);
    if (bAvg > 0 && rAvg > 120 && (rAvg * 100 / bAvg) > 180) {
        printf("  COLOR-SHIFT: r=%u g=%u b=%u\n", rAvg, gAvg, bAvg);
        return false;
    }
    return true;
}

// ─── Screen blend composite (from international.ino) ───

static void compositeIrOverTerrain(uint16_t* irBuf, const uint16_t* terrainBuf,
                                    int w, int h, bool invertIr) {
    // Find min/max luminance
    uint8_t minL = 255, maxL = 0;
    for (int i = 0; i < w * h; i += 4) {
        uint16_t c = irBuf[i];
        uint8_t r5 = (c >> 11) & 0x1F;
        uint8_t g6 = (c >> 5) & 0x3F;
        uint8_t b5 = c & 0x1F;
        uint8_t lum = (uint8_t)(((r5 << 3) * 77 + (g6 << 2) * 150 + (b5 << 3) * 29) >> 8);
        if (lum < minL) minL = lum;
        if (lum > maxL) maxL = lum;
    }
    if (maxL - minL < 10) { minL = 0; maxL = 255; }
    float range = (float)(maxL - minL);

    for (int i = 0; i < w * h; i++) {
        uint16_t irPx = irBuf[i];
        uint16_t tPx = terrainBuf[i];

        uint8_t r5 = (irPx >> 11) & 0x1F;
        uint8_t g6 = (irPx >> 5) & 0x3F;
        uint8_t b5 = irPx & 0x1F;
        uint8_t lum = (uint8_t)(((r5 << 3) * 77 + (g6 << 2) * 150 + (b5 << 3) * 29) >> 8);

        float norm = (float)(lum - minL) / range;
        if (norm < 0) norm = 0; if (norm > 1) norm = 1;
        if (invertIr) norm = 1.0f - norm;
        norm = (norm > 0.05f) ? (norm - 0.05f) / 0.95f : 0.0f;

        uint8_t tr5 = (tPx >> 11) & 0x1F;
        uint8_t tg6 = (tPx >> 5) & 0x3F;
        uint8_t tb5 = tPx & 0x1F;

        uint8_t cr5 = (uint8_t)(norm * 31.0f);
        uint8_t cg6 = (uint8_t)(norm * 63.0f);
        uint8_t cb5 = (uint8_t)(norm * 31.0f);

        uint8_t or5 = 31 - (uint8_t)((uint16_t)(31 - tr5) * (uint16_t)(31 - cr5) / 31);
        uint8_t og6 = 63 - (uint8_t)((uint16_t)(63 - tg6) * (uint16_t)(63 - cg6) / 63);
        uint8_t ob5 = 31 - (uint8_t)((uint16_t)(31 - tb5) * (uint16_t)(31 - cb5) / 31);

        irBuf[i] = ((uint16_t)or5 << 11) | ((uint16_t)og6 << 5) | ob5;
    }
}

// ─── File I/O helpers ───

static uint8_t* loadFile(const char* path, size_t* outLen) {
    FILE* f = fopen(path, "rb");
    if (!f) { *outLen = 0; return nullptr; }
    fseek(f, 0, SEEK_END);
    *outLen = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* buf = (uint8_t*)malloc(*outLen);
    fread(buf, 1, *outLen, f);
    fclose(f);
    return buf;
}

static void saveRGB565AsPPM(const char* path, const uint16_t* buf, int w, int h) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int i = 0; i < w * h; i++) {
        uint16_t c = buf[i];
        uint8_t r = ((c >> 11) & 0x1F) << 3;
        uint8_t g = ((c >> 5) & 0x3F) << 2;
        uint8_t b = (c & 0x1F) << 3;
        fwrite(&r, 1, 1, f);
        fwrite(&g, 1, 1, f);
        fwrite(&b, 1, 1, f);
    }
    fclose(f);
    printf("  Saved: %s\n", path);
}

// ─── Download helper ───

static uint8_t* downloadUrl(const char* url, size_t* outLen) {
    char cmd[1024];
    snprintf(cmd, sizeof(cmd), "curl -s -o /tmp/native_test_dl.bin '%s'", url);
    int rc = system(cmd);
    if (rc != 0) { *outLen = 0; return nullptr; }
    return loadFile("/tmp/native_test_dl.bin", outLen);
}

// ─── Tests ───

static int test_stb_progressive_jpeg() {
    printf("\n=== Test: stb_image progressive JPEG decode ===\n");

    // Download a progressive JPEG from EUMETView
    const char* url =
        "https://view.eumetsat.int/geoserver/ows"
        "?service=WMS&version=1.1.1&request=GetMap"
        "&layers=mtg_fd:rgb_geocolour&styles=&srs=EPSG:4326"
        "&bbox=-13.5,47.0,13.3,56.0&width=320&height=176"
        "&format=image/jpeg"
        "&TIME=2026-06-01T12:00:00Z";

    printf("  Downloading EUMETView progressive JPEG...\n");
    size_t jpegLen = 0;
    uint8_t* jpegData = downloadUrl(url, &jpegLen);
    if (!jpegData || jpegLen < 1000) {
        printf("  Download failed or too small (%zu bytes), trying older time...\n", jpegLen);
        free(jpegData);
        // Try with different time
        url = "https://view.eumetsat.int/geoserver/ows"
              "?service=WMS&version=1.1.1&request=GetMap"
              "&layers=mtg_fd:rgb_geocolour&styles=&srs=EPSG:4326"
              "&bbox=-13.5,47.0,13.3,56.0&width=320&height=176"
              "&format=image/jpeg"
              "&TIME=2026-05-30T12:00:00Z";
        jpegData = downloadUrl(url, &jpegLen);
        if (!jpegData || jpegLen < 1000) {
            printf("  FAIL: could not download test JPEG\n");
            free(jpegData);
            return 1;
        }
    }
    printf("  Downloaded: %zu bytes\n", jpegLen);

    // Check if it's progressive
    bool isProgressive = false;
    for (size_t i = 0; i < jpegLen - 1; i++) {
        if (jpegData[i] == 0xFF && jpegData[i+1] == 0xC2) { isProgressive = true; break; }
        if (jpegData[i] == 0xFF && jpegData[i+1] == 0xC0) { break; }
    }
    printf("  Progressive: %s\n", isProgressive ? "YES" : "no (baseline)");

    // Decode with stb_image
    printf("  Decoding with stb_image...\n");
    uint16_t* rgb565 = stb_decode_jpeg_to_rgb565(jpegData, jpegLen, DISP_W, DISP_H, false);
    free(jpegData);

    if (!rgb565) {
        printf("  FAIL: stb_image decode returned null\n");
        return 1;
    }
    printf("  PASS: decoded %dx%d to RGB565\n", DISP_W, DISP_H);

    // Run validation checks
    bool black = spriteLooksCompletelyBlack(rgb565, DISP_W * DISP_H);
    printf("  Black check: %s\n", black ? "FAIL (all black)" : "PASS");

    bool colorOk = colorShiftCheck(rgb565, DISP_W * DISP_H);
    printf("  Color-shift check: %s\n", colorOk ? "PASS" : "FAIL (rejected)");

    // Save output for visual inspection
    saveRGB565AsPPM("test_eumetview_decoded.ppm", rgb565, DISP_W, DISP_H);

    free(rgb565);
    return (black || !colorOk) ? 1 : 0;
}

static int test_himawari_composite() {
    printf("\n=== Test: Himawari IR + terrain screen blend ===\n");

    // Download Himawari IR
    printf("  Downloading Himawari IR...\n");
    size_t irLen = 0;
    uint8_t* irData = downloadUrl(
        "https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi"
        "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap&STYLES=&SRS=EPSG:4326"
        "&LAYERS=Himawari_AHI_Band13_Clean_Infrared"
        "&BBOX=129.4,31.2,150.0,40.2&WIDTH=320&HEIGHT=176"
        "&FORMAT=image%2Fjpeg&TIME=2026-05-30T12:00:00Z", &irLen);

    // Download S2 terrain
    printf("  Downloading Sentinel-2 terrain...\n");
    size_t s2Len = 0;
    uint8_t* s2Data = downloadUrl(
        "https://tiles.maps.eox.at/wms"
        "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
        "&LAYERS=s2cloudless-2024&STYLES=&SRS=EPSG:4326"
        "&BBOX=129.4,31.2,150.0,40.2&WIDTH=320&HEIGHT=176"
        "&FORMAT=image%2Fjpeg", &s2Len);

    if (!irData || irLen < 1000 || !s2Data || s2Len < 1000) {
        printf("  FAIL: download failed (ir=%zu s2=%zu)\n", irLen, s2Len);
        free(irData); free(s2Data);
        return 1;
    }

    // Decode both (baseline JPEG — stb handles both)
    uint16_t* irBuf = stb_decode_jpeg_to_rgb565(irData, irLen, DISP_W, DISP_H, false);
    uint16_t* s2Buf = stb_decode_jpeg_to_rgb565(s2Data, s2Len, DISP_W, DISP_H, false);
    free(irData); free(s2Data);

    if (!irBuf || !s2Buf) {
        printf("  FAIL: decode failed\n");
        free(irBuf); free(s2Buf);
        return 1;
    }

    saveRGB565AsPPM("test_himawari_ir.ppm", irBuf, DISP_W, DISP_H);
    saveRGB565AsPPM("test_terrain_s2.ppm", s2Buf, DISP_W, DISP_H);

    // Screen blend composite (invert IR for Himawari)
    printf("  Compositing...\n");
    compositeIrOverTerrain(irBuf, s2Buf, DISP_W, DISP_H, true);
    saveRGB565AsPPM("test_himawari_composite.ppm", irBuf, DISP_W, DISP_H);

    printf("  PASS: composite saved\n");
    free(irBuf); free(s2Buf);
    return 0;
}

static int test_goes_baseline() {
    printf("\n=== Test: GOES GeoColor baseline JPEG ===\n");

    size_t len = 0;
    uint8_t* data = downloadUrl(
        "https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi"
        "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap&STYLES=&SRS=EPSG:4326"
        "&LAYERS=GOES-East_ABI_GeoColor"
        "&BBOX=-85.0,36.2,-63.0,45.2&WIDTH=320&HEIGHT=176"
        "&FORMAT=image%2Fjpeg&TIME=2026-05-30T18:00:00Z", &len);

    if (!data || len < 1000) {
        printf("  FAIL: download failed (%zu bytes)\n", len);
        free(data);
        return 1;
    }
    printf("  Downloaded: %zu bytes\n", len);

    uint16_t* buf = stb_decode_jpeg_to_rgb565(data, len, DISP_W, DISP_H, false);
    free(data);

    if (!buf) {
        printf("  FAIL: decode failed\n");
        return 1;
    }

    bool black = spriteLooksCompletelyBlack(buf, DISP_W * DISP_H);
    printf("  Black: %s  ", black ? "YES" : "no");
    bool colorOk = colorShiftCheck(buf, DISP_W * DISP_H);
    printf("  Color-shift: %s\n", colorOk ? "PASS" : "FAIL");

    saveRGB565AsPPM("test_goes_decoded.ppm", buf, DISP_W, DISP_H);
    free(buf);
    return (black && len > 5000) ? 1 : 0;  // black is ok for nighttime GOES
}

int main() {
    printf("LiveSat Native Pipeline Test\n");
    printf("============================\n");

    int fails = 0;
    fails += test_stb_progressive_jpeg();
    fails += test_himawari_composite();
    fails += test_goes_baseline();

    printf("\n============================\n");
    printf("Results: %d test(s) failed\n", fails);
    printf("Output files: test_*.ppm (view with: xdg-open test_*.ppm)\n");
    return fails;
}
