// International satellite coverage + stb_image test endpoint

extern bool decodeProgressiveJpegToSprite(const uint8_t* data, size_t len,
                                          uint16_t* buf, int w, int h, bool swap);
extern char g_stbLastError[64];

static bool isEumetviewSource() { return s_activeSourceIsEumetview; }

static void handleTestStb() {
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    time_t t = time(nullptr) - 3600;
    t -= (t % 600);
    char ts[32];
    struct tm ti;
    gmtime_r(&t, &ti);
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", &ti);

    char url[512];
    snprintf(url, sizeof(url),
        "https://view.eumetsat.int/geoserver/ows"
        "?service=WMS&version=1.1.1&request=GetMap&styles=&srs=EPSG:4326"
        "&layers=mtg_fd:rgb_geocolour"
        "&bbox=-13.5,47.0,13.3,56.0&width=%d&height=%d"
        "&format=image/jpeg&TIME=%s",
        DISP_W, DISP_H, ts);

    http.begin(client, url);
    http.setTimeout(20000);
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        char msg[64];
        snprintf(msg, sizeof(msg), "EUMETView HTTP %d", code);
        http.end();
        s_wifiPortalServer.send(200, "text/plain", msg);
        return;
    }

    // Read body using writeToStream which handles chunked encoding
    FixedBufferWriteStream sink(s_dlBuf, DL_BUF_BYTES);
    int wrote = http.writeToStream(&sink);
    http.end();
    size_t rd = (wrote > 0) ? (size_t)wrote : 0;

    if (rd < 1000) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Download too small: %u bytes", (unsigned)rd);
        s_wifiPortalServer.send(200, "text/plain", msg);
        return;
    }

    if (!ensureSprite()) {
        s_wifiPortalServer.send(200, "text/plain", "ensureSprite failed");
        return;
    }
    sprite.fillScreen(TFT_BLACK);
    uint16_t* px = (uint16_t*)sprite.getBuffer();

    // Free animation cache to make PSRAM available for stb_image (~6.3MB)
    if (s_animCache) { heap_caps_free(s_animCache); s_animCache = nullptr; s_animCacheCount = 0; }

    // Check if progressive
    bool progressive = false;
    for (size_t i = 0; i + 1 < rd; i++) {
        if (s_dlBuf[i] == 0xFF && s_dlBuf[i+1] == 0xC2) { progressive = true; break; }
        if (s_dlBuf[i] == 0xFF && s_dlBuf[i+1] == 0xC0) { break; }
    }

    // Try decode
    uint32_t freeBefore = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    uint32_t t0 = millis();
    bool ok = decodeProgressiveJpegToSprite(s_dlBuf, rd, px, DISP_W, DISP_H,
                                             s_mainSpritePixelsByteSwapped);
    uint32_t ms = millis() - t0;
    uint32_t freeAfter = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    char msg[384];
    snprintf(msg, sizeof(msg),
        "Downloaded: %u bytes\nProgressive: %d\nPSRAM before: %u\nPSRAM after: %u\nPSRAM used: %d\nstb decode: %s (%lu ms)\nstb error: %s",
        (unsigned)rd, (int)progressive,
        (unsigned)freeBefore, (unsigned)freeAfter, (int)(freeBefore - freeAfter),
        ok ? "SUCCESS" : "FAILED", (unsigned long)ms, g_stbLastError);
    s_wifiPortalServer.send(200, "text/plain", msg);
    appendDiagLog("VLD", "msg=teststb len=%u prog=%d ok=%d ms=%lu psram=%u err=%s hdr=%02X%02X%02X%02X\n",
                  (unsigned)rd, (int)progressive, (int)ok, (unsigned long)ms, (unsigned)freeAfter, g_stbLastError,
                  rd>0?s_dlBuf[0]:0, rd>1?s_dlBuf[1]:0, rd>2?s_dlBuf[2]:0, rd>3?s_dlBuf[3]:0);
}

// Download-only test — no stb_image decode
static void handleTestDl() {
    appendDiagLog("DL", "msg=testdl_start\n");
    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(10);  // 10 second TLS timeout
    HTTPClient http;

    time_t t = time(nullptr) - 3600;
    t -= (t % 600);
    char ts[32];
    struct tm ti;
    gmtime_r(&t, &ti);
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", &ti);

    char url[512];
    snprintf(url, sizeof(url),
        "https://view.eumetsat.int/geoserver/ows"
        "?service=WMS&version=1.1.1&request=GetMap&styles=&srs=EPSG:4326"
        "&layers=mtg_fd:rgb_geocolour"
        "&bbox=-13.5,47.0,13.3,56.0&width=%d&height=%d"
        "&format=image/jpeg&TIME=%s",
        DISP_W, DISP_H, ts);

    appendDiagLog("DL", "msg=testdl_begin_http\n");
    http.begin(client, url);
    http.setTimeout(15000);
    appendDiagLog("DL", "msg=testdl_GET_start\n");
    int code = http.GET();
    appendDiagLog("DL", "msg=testdl_GET_done code=%d\n", code);
    if (code != HTTP_CODE_OK) {
        char msg[64];
        snprintf(msg, sizeof(msg), "HTTP %d", code);
        http.end();
        s_wifiPortalServer.send(200, "text/plain", msg);
        return;
    }

    WiFiClient* stream = http.getStreamPtr();
    size_t rd = 0;
    uint32_t dlStart = millis();
    while (rd < DL_BUF_BYTES) {
        size_t avail = stream->available();
        if (avail == 0) {
            if (!stream->connected()) break;
            if (millis() - dlStart > 20000) break;
            delay(1);
            continue;
        }
        size_t got = stream->readBytes((char*)(s_dlBuf + rd), min(avail, DL_BUF_BYTES - rd));
        if (got == 0) break;
        rd += got;
    }
    http.end();

    char msg[128];
    snprintf(msg, sizeof(msg), "Downloaded: %u bytes in %lu ms\nFirst bytes: %02X%02X%02X%02X",
        (unsigned)rd, millis() - dlStart,
        rd > 0 ? s_dlBuf[0] : 0, rd > 1 ? s_dlBuf[1] : 0,
        rd > 2 ? s_dlBuf[2] : 0, rd > 3 ? s_dlBuf[3] : 0);
    s_wifiPortalServer.send(200, "text/plain", msg);
}

// Test the full validation path used by sync loop
static void handleTestVld() {
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    time_t t = time(nullptr) - 3600;
    t -= (t % 600);
    char ts[32];
    struct tm ti;
    gmtime_r(&t, &ti);
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", &ti);

    char url[512];
    snprintf(url, sizeof(url),
        "%s&LAYERS=%s&BBOX=-13.5,47.0,13.3,56.0&WIDTH=%d&HEIGHT=%d&FORMAT=image%%2Fjpeg&TIME=%s",
        EUMETVIEW_WMS_BASE, WEATHER_LAYER_MTG_GEOCOLOR,
        DISP_W, DISP_H, ts);

    http.begin(client, url);
    http.setTimeout(20000);
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        char msg[64]; snprintf(msg, sizeof(msg), "HTTP %d", code);
        http.end(); s_wifiPortalServer.send(200, "text/plain", msg); return;
    }

    size_t jpegLen = 0;
    if (!readHttpJpegBodyToDlBuf(http, "TESTVLD", &jpegLen)) {
        s_wifiPortalServer.send(200, "text/plain", "readBody failed");
        return;
    }

    // Free animation cache so stb_image has PSRAM
    if (s_animCache) { heap_caps_free(s_animCache); s_animCache = nullptr; s_animCacheCount = 0; }

    uint32_t t0 = millis();
    bool valid = validateBufferedWeatherFrameJpeg(jpegLen, "TESTVLD");
    uint32_t ms = millis() - t0;

    char msg[256];
    snprintf(msg, sizeof(msg),
        "len=%u prog=%d valid=%d ms=%lu stb_err=%s psram_free=%u",
        (unsigned)jpegLen, (int)isProgressiveJpeg(s_dlBuf, jpegLen),
        (int)valid, (unsigned long)ms, g_stbLastError,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    s_wifiPortalServer.send(200, "text/plain", msg);
}

static void registerInternationalHandlers() {
    s_wifiPortalServer.on("/teststb", HTTP_GET, handleTestStb);
    s_wifiPortalServer.on("/testdl", HTTP_GET, handleTestDl);
    s_wifiPortalServer.on("/testvld", HTTP_GET, handleTestVld);
}
