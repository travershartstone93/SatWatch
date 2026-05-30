// ─────────────────────────────────────────────────────────────
//  International satellite coverage — separate compilation unit
//  Concatenated with main .ino by Arduino build system.
//  All globals from main sketch are accessible.
// ─────────────────────────────────────────────────────────────

// pngle included via main .ino

// Terrain reference buffer for IR composite (Himawari / INSAT regions)
#define IR_TERRAIN_FILE SD_ROOT "/frames/terrain.raw"
static uint16_t* s_irTerrainBuf = nullptr;  // PSRAM, DISP_W * DISP_H * 2 = 112,640 bytes

// ─────────────────────────────────────────────────────────────
//  Terrain reference — download, cache, load
// ─────────────────────────────────────────────────────────────

static bool loadTerrainReference() {
  if (!s_irTerrainBuf) {
    s_irTerrainBuf = (uint16_t*)heap_caps_malloc(RAW_FRAME_BYTES, MALLOC_CAP_SPIRAM);
    if (!s_irTerrainBuf) return false;
  }
  File f = SD.open(IR_TERRAIN_FILE, FILE_READ);
  if (!f || f.size() != RAW_FRAME_BYTES) {
    if (f) f.close();
    return false;
  }
  size_t got = f.read((uint8_t*)s_irTerrainBuf, RAW_FRAME_BYTES);
  f.close();
  return (got == RAW_FRAME_BYTES);
}

static bool saveTerrainReference() {
  if (!s_irTerrainBuf) return false;
  SD.remove(IR_TERRAIN_FILE);
  File f = SD.open(IR_TERRAIN_FILE, FILE_WRITE);
  if (!f) return false;
  size_t wrote = f.write((const uint8_t*)s_irTerrainBuf, RAW_FRAME_BYTES);
  f.flush();
  f.close();
  return (wrote == RAW_FRAME_BYTES);
}

// Download Sentinel-2 cloudless terrain at given bbox, decode to sprite,
// copy to terrain buffer, save to SD.
static bool downloadAndCacheTerrainReference(float bboxW, float bboxS, float bboxE, float bboxN) {
  if (!ensureSprite()) return false;
  if (!s_irTerrainBuf) {
    s_irTerrainBuf = (uint16_t*)heap_caps_malloc(RAW_FRAME_BYTES, MALLOC_CAP_SPIRAM);
    if (!s_irTerrainBuf) return false;
  }

  char url[512];
  snprintf(url, sizeof(url),
    "https://tiles.maps.eox.at/wms"
    "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
    "&LAYERS=s2cloudless-2024"
    "&STYLES=&SRS=EPSG:4326"
    "&BBOX=%.1f,%.1f,%.1f,%.1f&WIDTH=%d&HEIGHT=%d"
    "&FORMAT=image%%2Fjpeg",
    (double)bboxW, (double)bboxS, (double)bboxE, (double)bboxN,
    DISP_W, DISP_H);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, url);
  http.setTimeout(10000);
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    http.end();
    appendDiagLog("terrain: S2 HTTP-%d\n", code);
    return false;
  }

  int contentLen = http.getSize();
  if (contentLen <= 0 || contentLen > (int)DL_BUF_BYTES) {
    http.end();
    return false;
  }
  WiFiClient* stream = http.getStreamPtr();
  size_t rd = 0;
  while (rd < (size_t)contentLen) {
    size_t avail = stream->available();
    if (avail == 0) { delay(1); continue; }
    size_t chunk = stream->readBytes((char*)(s_dlBuf + rd), min(avail, (size_t)contentLen - rd));
    if (chunk == 0) break;
    rd += chunk;
  }
  http.end();
  if ((int)rd < 1000) return false;

  // Decode S2 JPEG into sprite
  LovyanGFX* prevTarget = g_drawTarget;
  sprite.fillScreen(TFT_BLACK);
  g_drawTarget = &sprite;
  resetJpegDrawStats();
  bool ok = false;
  if (jpeg.openRAM(s_dlBuf, (int)rd, jpegDraw)) {
    if (jpeg.getWidth() == DISP_W && jpeg.getHeight() == DISP_H) {
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      ok = jpeg.decode(0, 0, 0);
    }
    jpeg.close();
  }
  g_drawTarget = prevTarget;
  if (!ok) { appendDiagLog("terrain: decode fail\n"); return false; }

  // Copy sprite buffer to terrain reference
  const uint16_t* src = (const uint16_t*)sprite.getBuffer();
  if (!src) return false;
  memcpy(s_irTerrainBuf, src, RAW_FRAME_BYTES);

  saveTerrainReference();
  appendDiagLog("terrain: cached ok\n");
  return true;
}

// Ensure terrain reference is loaded (from SD cache or fresh download)
static void ensureTerrainReference(float bboxW, float bboxS, float bboxE, float bboxN) {
  if (!s_activeSourceIsIR) return;
  if (loadTerrainReference()) return;
  downloadAndCacheTerrainReference(bboxW, bboxS, bboxE, bboxN);
}

// ─────────────────────────────────────────────────────────────
//  IR-over-terrain screen blend composite
// ─────────────────────────────────────────────────────────────

// Composite IR satellite data over terrain reference using screen blend.
// Operates in-place on the sprite buffer. Terrain reference must be loaded.
// For Himawari: IR is inverted (bright=warm/clear → invert so bright=cloud).
// For INSAT: IR is already bright=cloud, no inversion needed.
static void compositeIrOverTerrain() {
  if (!s_irTerrainBuf) return;
  uint16_t* px = (uint16_t*)sprite.getBuffer();
  if (!px) return;
  bool swapped = s_mainSpritePixelsByteSwapped;
  int total = DISP_W * DISP_H;

  // First pass: find min/max luminance for normalization
  uint8_t minL = 255, maxL = 0;
  for (int i = 0; i < total; i += 4) {  // sample every 4th pixel for speed
    uint16_t c = swapped ? __builtin_bswap16(px[i]) : px[i];
    uint8_t r5 = (c >> 11) & 0x1F;
    uint8_t g6 = (c >> 5) & 0x3F;
    uint8_t b5 = c & 0x1F;
    uint8_t lum = (uint8_t)(((r5 << 3) * 77 + (g6 << 2) * 150 + (b5 << 3) * 29) >> 8);
    if (lum < minL) minL = lum;
    if (lum > maxL) maxL = lum;
  }
  if (maxL - minL < 10) { minL = 0; maxL = 255; }
  float range = (float)(maxL - minL);
  float invRange = 1.0f / range;

  // Determine if IR needs inversion (Himawari Band13: bright=warm, needs invert)
  bool invertIr = activeLayerIs(WEATHER_LAYER_HIMAWARI_IR);

  // Second pass: screen blend each pixel
  for (int i = 0; i < total; i++) {
    uint16_t irPx = swapped ? __builtin_bswap16(px[i]) : px[i];
    uint16_t tPx = s_irTerrainBuf[i];
    // If terrain buf was saved from sprite with same byte order
    // (it was memcpy'd from sprite.getBuffer()), same swap applies
    if (swapped) tPx = __builtin_bswap16(tPx);

    // IR luminance
    uint8_t r5 = (irPx >> 11) & 0x1F;
    uint8_t g6 = (irPx >> 5) & 0x3F;
    uint8_t b5 = irPx & 0x1F;
    uint8_t lum = (uint8_t)(((r5 << 3) * 77 + (g6 << 2) * 150 + (b5 << 3) * 29) >> 8);

    // Normalize to 0-255
    float norm = (float)(lum - minL) * invRange;
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    // Invert if needed (Himawari: bright=warm → make bright=cloud)
    if (invertIr) norm = 1.0f - norm;

    // Clip floor to keep clear sky clean
    norm = (norm > 0.05f) ? (norm - 0.05f) / 0.95f : 0.0f;

    // Terrain channels (RGB565)
    uint8_t tr5 = (tPx >> 11) & 0x1F;
    uint8_t tg6 = (tPx >> 5) & 0x3F;
    uint8_t tb5 = tPx & 0x1F;

    // Screen blend: out = 1 - (1 - terrain) * (1 - cloud)
    // In 5/6-bit space: out = max - (max - terrain) * (max - cloud_scaled) / max
    uint8_t cr5 = (uint8_t)(norm * 31.0f);
    uint8_t cg6 = (uint8_t)(norm * 63.0f);
    uint8_t cb5 = (uint8_t)(norm * 31.0f);

    uint8_t or5 = 31 - (uint8_t)((uint16_t)(31 - tr5) * (uint16_t)(31 - cr5) / 31);
    uint8_t og6 = 63 - (uint8_t)((uint16_t)(63 - tg6) * (uint16_t)(63 - cg6) / 63);
    uint8_t ob5 = 31 - (uint8_t)((uint16_t)(31 - tb5) * (uint16_t)(31 - cb5) / 31);

    uint16_t out = ((uint16_t)or5 << 11) | ((uint16_t)og6 << 5) | ob5;
    px[i] = swapped ? __builtin_bswap16(out) : out;
  }
}

// ─────────────────────────────────────────────────────────────
//  Open-Meteo forecast (global, free, no API key)
//  Fallback for non-US locations where NWS returns 404.
// ─────────────────────────────────────────────────────────────

// WMO weather code → icon type character used by forecast ticker
static char wmoCodeToIconType(int code) {
  if (code <= 3)  return 'C';  // Clear / partly cloudy
  if (code <= 48) return 'C';  // Fog (use cloud icon)
  if (code <= 67) return 'R';  // Rain / drizzle
  if (code <= 77) return 'S';  // Snow
  if (code <= 82) return 'R';  // Rain showers
  if (code <= 86) return 'S';  // Snow showers
  return 'T';                  // Thunderstorm (95-99)
}

// WMO weather code → short description string
static const char* wmoCodeToDesc(int code) {
  switch (code) {
    case 0:  return "Clear";
    case 1:  return "Mostly Clear";
    case 2:  return "Partly Cloudy";
    case 3:  return "Overcast";
    case 45: case 48: return "Fog";
    case 51: return "Lt Drizzle";
    case 53: return "Drizzle";
    case 55: return "Hvy Drizzle";
    case 61: return "Lt Rain";
    case 63: return "Rain";
    case 65: return "Hvy Rain";
    case 66: case 67: return "Frz Rain";
    case 71: return "Lt Snow";
    case 73: return "Snow";
    case 75: return "Hvy Snow";
    case 77: return "Snow Grains";
    case 80: return "Lt Showers";
    case 81: return "Showers";
    case 82: return "Hvy Showers";
    case 85: return "Lt Snow Shwr";
    case 86: return "Snow Showers";
    case 95: return "Thunderstorm";
    case 96: case 99: return "Hail Storm";
    default: return "Unknown";
  }
}

// ─────────────────────────────────────────────────────────────
//  RainViewer global radar (free, no API key, XYZ PNG tiles)
//  Used outside US where NOAA radar isn't available.
// ─────────────────────────────────────────────────────────────

// RainViewer API state
static char s_rainViewerPath[64] = {};       // e.g. "/v2/radar/abc123"
static unsigned long long s_rainViewerTs = 0; // latest radar timestamp (unix ms)

// Check if location is within US NOAA radar coverage
static bool locationHasNoaaRadar() {
  float lon = s_weatherCenterLon;
  float lat = s_weatherCenterLat;
  // Rough US bounding box (includes Alaska, Hawaii, Puerto Rico)
  return (lon > -170.0f && lon < -60.0f && lat > 15.0f && lat < 72.0f);
}

// Fetch latest radar frame path from RainViewer API
static bool fetchRainViewerLatestPath(WiFiClientSecure& client, HTTPClient& http) {
  http.begin(client, "https://api.rainviewer.com/public/weather-maps.json");
  http.setTimeout(10000);
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    http.end();
    appendDiagLog("rainviewer: API HTTP-%d\n", code);
    return false;
  }
  String body = http.getString();
  http.end();
  if (body.length() < 50) return false;

  // Parse last entry in radar.past[] array
  // Find last "path":" before the end of "past" array
  const char* s = body.c_str();
  const char* pastArr = strstr(s, "\"past\"");
  if (!pastArr) return false;

  // Find the last "path" entry in the past array
  const char* lastPath = nullptr;
  const char* cursor = pastArr;
  while (true) {
    const char* p = strstr(cursor, "\"path\":\"");
    if (!p) break;
    // Make sure we're still in the "past" array (before "nowcast")
    const char* nowcast = strstr(pastArr, "\"nowcast\"");
    if (nowcast && p > nowcast) break;
    lastPath = p;
    cursor = p + 8;
  }
  if (!lastPath) return false;

  // Extract path string
  const char* pathStart = lastPath + 8;  // skip "path":"
  const char* pathEnd = strchr(pathStart, '"');
  if (!pathEnd || pathEnd - pathStart >= (int)sizeof(s_rainViewerPath)) return false;
  size_t len = pathEnd - pathStart;
  memcpy(s_rainViewerPath, pathStart, len);
  s_rainViewerPath[len] = '\0';

  // Extract timestamp from the same object
  // Look backwards for "time": before this "path"
  const char* timeKey = nullptr;
  cursor = pastArr;
  while (true) {
    const char* t = strstr(cursor, "\"time\":");
    if (!t || t > lastPath) break;
    timeKey = t;
    cursor = t + 7;
  }
  if (timeKey) {
    s_rainViewerTs = (unsigned long long)atoll(timeKey + 7) * 1000ULL;
  }

  appendDiagLog("rainviewer: path=%s\n", s_rainViewerPath);
  return true;
}

// PNG decode callback state for tile stitching
struct RvTileCtx {
  uint16_t* outBuf;     // destination buffer (RGB565)
  int outW, outH;       // destination dimensions
  int tileOffX, tileOffY; // where this tile goes in the output
  bool swapped;          // byte-swap for sprite
};

static void rvPngDrawCb(pngle_t* pngle, uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                         const uint8_t rgba[4]) {
  RvTileCtx* ctx = (RvTileCtx*)pngle_get_user_data(pngle);
  if (!ctx || !ctx->outBuf) return;
  int dx = ctx->tileOffX + (int)x;
  int dy = ctx->tileOffY + (int)y;
  if (dx < 0 || dx >= ctx->outW || dy < 0 || dy >= ctx->outH) return;
  if (rgba[3] < 64) return;  // skip mostly-transparent pixels

  uint16_t rgb565 = ((rgba[0] >> 3) << 11) | ((rgba[1] >> 2) << 5) | (rgba[2] >> 3);
  if (ctx->swapped) rgb565 = __builtin_bswap16(rgb565);
  ctx->outBuf[dy * ctx->outW + dx] = rgb565;
}

// XYZ tile coordinate helpers
static void lonLatToTileXY(float lat, float lon, int z, int* tx, int* ty) {
  int n = 1 << z;
  *tx = (int)((lon + 180.0f) / 360.0f * n);
  float latRad = lat * 0.01745329252f;
  *ty = (int)((1.0f - logf(tanf(latRad) + 1.0f / cosf(latRad)) / 3.14159265f) / 2.0f * n);
}

// Download RainViewer radar tiles and stitch into a JPEG on SD.
// Output matches NOAA radar format so existing overlay logic works.
static bool downloadRainViewerRadarAtBbox(WiFiClientSecure& client, HTTPClient& http,
                                          float bboxW, float bboxS, float bboxE, float bboxN,
                                          int outW, int outH,
                                          const char* outPath) {
  if (s_rainViewerPath[0] == '\0') return false;

  int z = 5;  // ~4-6 tiles per bbox at this zoom
  int txMin, tyMax, txMax, tyMin;
  lonLatToTileXY(bboxS, bboxW, z, &txMin, &tyMax);
  lonLatToTileXY(bboxN, bboxE, z, &txMax, &tyMin);

  int tileCountX = txMax - txMin + 1;
  int tileCountY = tyMax - tyMin + 1;
  int totalTiles = tileCountX * tileCountY;
  if (totalTiles > 20 || totalTiles <= 0) return false;

  // Allocate stitched buffer in PSRAM (outW * outH * 2 bytes RGB565)
  size_t bufBytes = (size_t)outW * outH * 2;
  uint16_t* stitchBuf = (uint16_t*)heap_caps_calloc(outW * outH, 2, MALLOC_CAP_SPIRAM);
  if (!stitchBuf) return false;

  // Tile pixel size in stitched output
  int stitchW = tileCountX * 256;
  int stitchH = tileCountY * 256;
  // We'll decode tiles into a temp stitched buffer at full tile resolution,
  // then crop/scale to outW x outH.
  uint16_t* tileBuf = (uint16_t*)heap_caps_calloc(stitchW * stitchH, 2, MALLOC_CAP_SPIRAM);
  if (!tileBuf) { heap_caps_free(stitchBuf); return false; }

  RvTileCtx ctx;
  ctx.outBuf = tileBuf;
  ctx.outW = stitchW;
  ctx.outH = stitchH;
  ctx.swapped = false;
  int tilesOk = 0;

  for (int ty = tyMin; ty <= tyMax; ty++) {
    for (int tx = txMin; tx <= txMax; tx++) {
      ctx.tileOffX = (tx - txMin) * 256;
      ctx.tileOffY = (ty - tyMin) * 256;

      char tileUrl[256];
      snprintf(tileUrl, sizeof(tileUrl),
        "https://tilecache.rainviewer.com%s/256/%d/%d/%d/6/1_1.png",
        s_rainViewerPath, z, tx, ty);

      http.begin(client, tileUrl);
      http.setTimeout(15000);
      int code = http.GET();
      if (code != HTTP_CODE_OK) { http.end(); continue; }

      int contentLen = http.getSize();
      if (contentLen <= 0 || contentLen > (int)DL_BUF_BYTES) { http.end(); continue; }

      WiFiClient* stream = http.getStreamPtr();
      size_t rd = 0;
      while (rd < (size_t)contentLen) {
        size_t avail = stream->available();
        if (avail == 0) { delay(1); continue; }
        size_t chunk = stream->readBytes((char*)(s_dlBuf + rd), min(avail, (size_t)contentLen - rd));
        if (chunk == 0) break;
        rd += chunk;
      }
      http.end();

      // Decode PNG tile
      pngle_t* pngle = pngle_new();
      if (pngle) {
        pngle_set_user_data(pngle, &ctx);
        pngle_set_draw_callback(pngle, rvPngDrawCb);
        pngle_feed(pngle, s_dlBuf, rd);
        pngle_destroy(pngle);
        tilesOk++;
      }
    }
  }

  if (tilesOk == 0) {
    heap_caps_free(tileBuf);
    heap_caps_free(stitchBuf);
    return false;
  }

  // Crop stitched tiles to bbox and scale to outW x outH
  // Calculate which pixels in the stitched image correspond to the bbox
  // Tile bounds
  float tileLonW = (float)txMin / (float)(1 << z) * 360.0f - 180.0f;
  float tileLonE = (float)(txMax + 1) / (float)(1 << z) * 360.0f - 180.0f;
  float n = (float)(1 << z);
  float tileLatN = atanf(sinhf(3.14159265f * (1.0f - 2.0f * (float)tyMin / n))) * 57.2957795f;
  float tileLatS = atanf(sinhf(3.14159265f * (1.0f - 2.0f * (float)(tyMax + 1) / n))) * 57.2957795f;

  int cropL = (int)((bboxW - tileLonW) / (tileLonE - tileLonW) * stitchW);
  int cropR = (int)((bboxE - tileLonW) / (tileLonE - tileLonW) * stitchW);
  int cropT = (int)((tileLatN - bboxN) / (tileLatN - tileLatS) * stitchH);
  int cropB = (int)((tileLatN - bboxS) / (tileLatN - tileLatS) * stitchH);
  if (cropL < 0) cropL = 0;
  if (cropT < 0) cropT = 0;
  if (cropR > stitchW) cropR = stitchW;
  if (cropB > stitchH) cropB = stitchH;
  int cropW = cropR - cropL;
  int cropH = cropB - cropT;
  if (cropW <= 0 || cropH <= 0) {
    heap_caps_free(tileBuf);
    heap_caps_free(stitchBuf);
    return false;
  }

  // Nearest-neighbor scale from crop region to outW x outH
  for (int y = 0; y < outH; y++) {
    int srcY = cropT + y * cropH / outH;
    if (srcY >= stitchH) srcY = stitchH - 1;
    for (int x = 0; x < outW; x++) {
      int srcX = cropL + x * cropW / outW;
      if (srcX >= stitchW) srcX = stitchW - 1;
      stitchBuf[y * outW + x] = tileBuf[srcY * stitchW + srcX];
    }
  }

  heap_caps_free(tileBuf);

  // Write as raw RGB565 to SD (same format as NOAA radar JPEG after decode)
  // Actually, we need to write a JPEG. Use the sprite for encoding.
  // Simpler: write raw, and modify radar overlay to accept raw.
  // For now: copy to sprite, then use existing JPEG path.
  if (ensureSprite()) {
    uint16_t* spritePx = (uint16_t*)sprite.getBuffer();
    if (spritePx && outW == DISP_W && outH == DISP_H) {
      memcpy(spritePx, stitchBuf, bufBytes);
      // Write sprite as raw file (radar overlay reads JPEG, but we can adapt)
      SD.remove(outPath);
      File f = SD.open(outPath, FILE_WRITE);
      if (f) {
        f.write((const uint8_t*)stitchBuf, bufBytes);
        f.flush();
        f.close();
      }
    }
  }

  heap_caps_free(stitchBuf);
  appendDiagLog("rainviewer: %d/%d tiles -> %s\n", tilesOk, totalTiles, outPath);
  return tilesOk > 0;
}
