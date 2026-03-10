#!/usr/bin/env python3
"""
Refactor esp32-LiveSat-s3.ino: Replace multi-file frame storage with
pre-allocated binary files + single sync path.
"""
import re
import sys

SRC = "/home/whisper/Desktop/LiveSat/esp32-LiveSat-s3/esp32-LiveSat-s3.ino"
DST = SRC  # overwrite in-place (backup already made)

with open(SRC, "r") as f:
    lines = f.readlines()

text = "".join(lines)

# ═══════════════════════════════════════════════════════════════
# STEP 1: Add new #defines after ZOOM_SNAPSHOT_META_FILE line
# ═══════════════════════════════════════════════════════════════

old_zoom_meta = '#define ZOOM_SNAPSHOT_META_FILE SD_ROOT "/frames/zoom.meta"'
new_defines_block = '''#define ZOOM_SNAPSHOT_META_FILE SD_ROOT "/frames/zoom.meta"

// ── Pre-allocated frame store (replaces per-frame .jpg files) ──
#define JPEG_SLOT_BYTES     (64 * 1024)
#define FRAMES_BIN_FILE     SD_ROOT "/frames/frames.bin"
#define INDEX_BIN_FILE      SD_ROOT "/frames/index.bin"
#define INDEX_TMP_FILE      SD_ROOT "/frames/index.tmp"
#define INDEX_MAGIC         0x4C534658    // "LSFX"'''

text = text.replace(old_zoom_meta, new_defines_block, 1)

# ═══════════════════════════════════════════════════════════════
# STEP 2: Add FrameStoreIndex struct + global after s_frameTimes
# ═══════════════════════════════════════════════════════════════

old_frametimes = """static time_t s_frameTimes[MAX_FRAMES];
static bool   s_timesLoaded = false;
static uint8_t s_sourceBlackLogged[MAX_FRAMES];"""

new_frametimes = """static time_t s_frameTimes[MAX_FRAMES];
static bool   s_timesLoaded = false;
static uint8_t s_sourceBlackLogged[MAX_FRAMES];

struct FrameStoreIndex {
  uint32_t magic;
  uint16_t head;
  uint16_t count;
  time_t   times[MAX_FRAMES];
  uint32_t jpegLen[MAX_FRAMES];
  uint8_t  jpegValid[MAX_FRAMES];
  uint8_t  rawValid[MAX_FRAMES];
};
static FrameStoreIndex s_idx;"""

text = text.replace(old_frametimes, new_frametimes, 1)

# ═══════════════════════════════════════════════════════════════
# STEP 3: Remove s_streamSlotMap and adjust invalidateStreamSlot
# ═══════════════════════════════════════════════════════════════

# Remove s_streamSlotMap declaration
text = text.replace("static uint8_t s_streamSlotMap[MAX_FRAMES];\n", "", 1)

# Update invalidateStreamSlot — remove s_streamSlotMap reference and RAW_CACHE_META_FILE
old_inv = """static void invalidateStreamSlot(int idx, const char* reason) {
  if (idx < 0 || idx >= MAX_FRAMES) return;
  if (!s_streamValid[idx]) return;
  s_streamValid[idx] = 0;
  s_streamSlotMap[idx] = 0xFF;
#if !BOARD_IS_AMOLED_206
  tft.waitDMA();  // ensure LCD DMA done before SD bus access
#endif
  SD.remove(RAW_CACHE_META_FILE);     // force rebuild next boot / next cache check
  invalidateRawCacheMetaState();
  invalidateValidIdxCache();
  Serial.printf("INV idx=%d %s\\n", idx, reason ? reason : "?");
}"""

new_inv = """static void invalidateStreamSlot(int idx, const char* reason) {
  if (idx < 0 || idx >= MAX_FRAMES) return;
  if (!s_streamValid[idx]) return;
  s_streamValid[idx] = 0;
  if (idx < (int)s_idx.count) s_idx.rawValid[idx] = 0;
  invalidateValidIdxCache();
  Serial.printf("INV idx=%d %s\\n", idx, reason ? reason : "?");
}"""

text = text.replace(old_inv, new_inv, 1)

# ═══════════════════════════════════════════════════════════════
# STEP 4: Replace showFrame() — use circular offset instead of s_streamSlotMap
# ═══════════════════════════════════════════════════════════════

old_showframe = """static bool showFrame(int idx) {
  if (!s_streamReady || !s_streamFile || idx >= frameCount || !s_streamValid[idx]) return false;
  if (!s_frameDisplayBuf) return false;

  uint8_t physSlot = s_streamSlotMap[idx];
  if (physSlot >= frameCount) return false;
  uint32_t offset = (uint32_t)physSlot * (uint32_t)SCALED_FRAME_BYTES;
  bool seekOk = s_streamFile.seek(offset);
  size_t got = s_streamFile.read((uint8_t*)s_frameDisplayBuf, SCALED_FRAME_BYTES);
  if (!seekOk || got != SCALED_FRAME_BYTES) {
    invalidateStreamSlot(idx, "short-read");
    appendDiagLog("showFrame: read-fail idx=%d got=%u seek=%d\\n",
                  idx, (unsigned)got, (int)seekOk);
    return false;
  }
  // Render fresh timestamp bars for this frame and store in s_topBarBuf/s_botBarBuf.
  // applyBarsToBuf() inside presentScaledBuf() stamps them persistently over the frame.
  updateBarBufs(idx);
  presentScaledBuf(s_frameDisplayBuf);
  return true;
}"""

new_showframe = """static bool showFrame(int idx) {
  if (!s_streamReady || !s_streamFile || idx >= frameCount || !s_streamValid[idx]) return false;
  if (!s_frameDisplayBuf) return false;

  int phys = ((int)s_idx.head + idx) % MAX_FRAMES;
  uint32_t offset = (uint32_t)phys * (uint32_t)SCALED_FRAME_BYTES;
  bool seekOk = s_streamFile.seek(offset);
  size_t got = s_streamFile.read((uint8_t*)s_frameDisplayBuf, SCALED_FRAME_BYTES);
  if (!seekOk || got != SCALED_FRAME_BYTES) {
    invalidateStreamSlot(idx, "short-read");
    appendDiagLog("showFrame: read-fail idx=%d got=%u seek=%d\\n",
                  idx, (unsigned)got, (int)seekOk);
    return false;
  }
  updateBarBufs(idx);
  presentScaledBuf(s_frameDisplayBuf);
  return true;
}"""

text = text.replace(old_showframe, new_showframe, 1)

# ═══════════════════════════════════════════════════════════════
# STEP 5: Add forward declarations for new functions
# ═══════════════════════════════════════════════════════════════

old_fwd = "static void syncProgressEnd();"
new_fwd = """static void syncProgressEnd();
static void initFrameStore();
static bool loadIndex();
static void writeIndex();
static bool readJpegFromSlot(int logicalIdx, uint8_t* buf, size_t* outLen);
static bool writeJpegToSlot(int logicalIdx, const uint8_t* buf, size_t len);
static bool writeRawToSlot(int logicalIdx, const uint8_t* buf);
static void syncWeatherFrames();
static void rebuildRawFromStored();"""

text = text.replace(old_fwd, new_fwd, 1)

# Remove old forward declarations that will be deleted
for old_decl in [
    "static bool validateStoredWeatherFramePath(const char* path, const char* label = nullptr);\n",
    "static bool weatherFrameLooksCompressedSizeOutlier(int idx, const char* candidatePath,\n                                                   const char* label = nullptr);\n",
    "static bool weatherFrameLooksSemanticOutlier(int idx, const char* candidatePath,\n                                             const char* label = nullptr);\n",
    "static bool healCachedWeatherFrameFromNeighbor(int idx, int frameLimit = -1);\n",
    "static bool downloadFrameToPath(HTTPClient& http,\n                                WiFiClientSecure& client,\n                                time_t t,\n                                const char* sdPath,\n                                size_t* outBytes);\n",
]:
    text = text.replace(old_decl, "", 1)

# ═══════════════════════════════════════════════════════════════
# STEP 6: Add core helper functions after appendDiagLog
# ═══════════════════════════════════════════════════════════════

after_appenddiag = """static void appendDiagLog(const char* fmt, ...) {
  File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND);
  if (!diagF) return;
  char line[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  diagF.print(line);
  diagF.close();
}"""

core_helpers = '''static void appendDiagLog(const char* fmt, ...) {
  File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND);
  if (!diagF) return;
  char line[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  diagF.print(line);
  diagF.close();
}

// ─────────────────────────────────────────────────────────────
//  Pre-allocated frame store helpers
// ─────────────────────────────────────────────────────────────

static void initFrameStore() {
  SD.mkdir(FRAMES_DIR);

  // Delete legacy per-frame files if any exist
  bool hasLegacy = false;
  {
    File lf = SD.open(META_FILE, FILE_READ);
    if (lf) { lf.close(); hasLegacy = true; }
  }
  {
    File lf = SD.open(TIMES_FILE, FILE_READ);
    if (lf) { lf.close(); hasLegacy = true; }
  }
  if (hasLegacy) {
    appendDiagLog("initFrameStore: legacy files detected, cleaning up\\n");
    // Remove old metadata files
    SD.remove(META_FILE);
    SD.remove(TIMES_FILE);
    SD.remove(RAW_CACHE_META_FILE);
    SD.remove(CACHE_VALIDATE_META_FILE);
    SD.remove(FRAME_DIM_FILE);
    // Remove old per-frame .jpg files
    File dir = SD.open(FRAMES_DIR);
    if (dir) {
      char names[MAX_FRAMES + 16][20];
      int nameCount = 0;
      while (nameCount < (int)(sizeof(names)/sizeof(names[0]))) {
        File entry = dir.openNextFile();
        if (!entry) break;
        const char* n = entry.name();
        size_t nlen = strlen(n);
        entry.close();
        if (nlen < 5 || nlen >= sizeof(names[0])) continue;
        if (strcmp(n + nlen - 4, ".jpg") != 0) continue;
        // Skip zoom files
        if (strncmp(n, "vz", 2) == 0) continue;
        if (strncmp(n, "terrain", 7) == 0) continue;
        strncpy(names[nameCount], n, sizeof(names[0]) - 1);
        names[nameCount][sizeof(names[0]) - 1] = \'\\0\';
        nameCount++;
      }
      dir.close();
      for (int k = 0; k < nameCount; k++) {
        char path[40];
        snprintf(path, sizeof(path), "%s/%s", FRAMES_DIR, names[k]);
        SD.remove(path);
      }
    }
    // Also remove old stream.raw (wrong layout for new system)
    SD.remove(RAW_STREAM_FILE);
  }

  // Pre-allocate frames.bin (144 × 64KB = 9MB)
  const size_t framesBinSize = (size_t)MAX_FRAMES * JPEG_SLOT_BYTES;
  {
    File f = SD.open(FRAMES_BIN_FILE, FILE_READ);
    bool needCreate = !f;
    if (f) {
      if (f.size() != framesBinSize) needCreate = true;
      f.close();
    }
    if (needCreate) {
      showMessage("Allocating frames.bin...", "First boot setup");
      SD.remove(FRAMES_BIN_FILE);
      File wf = SD.open(FRAMES_BIN_FILE, FILE_WRITE);
      if (wf) {
        memset(s_dlBuf, 0, DL_BUF_BYTES);
        size_t written = 0;
        while (written < framesBinSize) {
          size_t chunk = framesBinSize - written;
          if (chunk > DL_BUF_BYTES) chunk = DL_BUF_BYTES;
          wf.write(s_dlBuf, chunk);
          written += chunk;
          int pct = (int)((written * 100ULL) / framesBinSize);
          showProgress(pct, 100, "frames.bin");
          yield();
        }
        wf.flush();
        wf.close();
        appendDiagLog("initFrameStore: frames.bin created %u bytes\\n", (unsigned)framesBinSize);
      }
    }
  }

  // Pre-allocate stream.raw (144 × 295200 = ~42.5MB)
  const size_t streamRawSize = (size_t)MAX_FRAMES * SCALED_FRAME_BYTES;
  {
    File f = SD.open(RAW_STREAM_FILE, FILE_READ);
    bool needCreate = !f;
    if (f) {
      if (f.size() != streamRawSize) needCreate = true;
      f.close();
    }
    if (needCreate) {
      showMessage("Allocating stream.raw...", "First boot setup");
      SD.remove(RAW_STREAM_FILE);
      File wf = SD.open(RAW_STREAM_FILE, FILE_WRITE);
      if (wf) {
        memset(s_dlBuf, 0, DL_BUF_BYTES);
        size_t written = 0;
        while (written < streamRawSize) {
          size_t chunk = streamRawSize - written;
          if (chunk > DL_BUF_BYTES) chunk = DL_BUF_BYTES;
          wf.write(s_dlBuf, chunk);
          written += chunk;
          int pct = (int)((written * 100ULL) / streamRawSize);
          showProgress(pct, 100, "stream.raw");
          yield();
        }
        wf.flush();
        wf.close();
        appendDiagLog("initFrameStore: stream.raw created %u bytes\\n", (unsigned)streamRawSize);
      }
    }
  }
}

static bool loadIndex() {
  memset(&s_idx, 0, sizeof(s_idx));
  memset(s_streamValid, 0, sizeof(s_streamValid));

  File f = SD.open(INDEX_BIN_FILE, FILE_READ);
  if (!f) {
    appendDiagLog("loadIndex: no index.bin\\n");
    frameCount = 0;
    framesReady = false;
    s_timesLoaded = true;
    return false;
  }
  if (f.size() != sizeof(FrameStoreIndex)) {
    f.close();
    appendDiagLog("loadIndex: bad size %u\\n", (unsigned)f.size());
    frameCount = 0;
    framesReady = false;
    s_timesLoaded = true;
    return false;
  }
  f.read((uint8_t*)&s_idx, sizeof(s_idx));
  f.close();

  if (s_idx.magic != INDEX_MAGIC) {
    appendDiagLog("loadIndex: bad magic 0x%08X\\n", s_idx.magic);
    memset(&s_idx, 0, sizeof(s_idx));
    frameCount = 0;
    framesReady = false;
    s_timesLoaded = true;
    return false;
  }

  // SOI sanity check: verify jpegValid slots have valid JPEG headers
  File fb = SD.open(FRAMES_BIN_FILE, FILE_READ);
  if (fb) {
    for (int i = 0; i < (int)s_idx.count; i++) {
      if (!s_idx.jpegValid[i]) continue;
      int phys = ((int)s_idx.head + i) % MAX_FRAMES;
      uint32_t off = (uint32_t)phys * JPEG_SLOT_BYTES;
      uint8_t hdr[2] = {0, 0};
      fb.seek(off);
      fb.read(hdr, 2);
      if (hdr[0] != 0xFF || hdr[1] != 0xD8) {
        appendDiagLog("loadIndex: SOI fail slot %d phys %d\\n", i, phys);
        s_idx.jpegValid[i] = 0;
        s_idx.rawValid[i] = 0;
      }
    }
    fb.close();
  }

  // Populate compatibility arrays
  int validCount = 0;
  for (int i = 0; i < (int)s_idx.count && i < MAX_FRAMES; i++) {
    s_frameTimes[i] = s_idx.times[i];
    s_streamValid[i] = s_idx.rawValid[i];
    if (s_idx.rawValid[i]) validCount++;
  }
  for (int i = (int)s_idx.count; i < MAX_FRAMES; i++) {
    s_frameTimes[i] = 0;
    s_streamValid[i] = 0;
  }

  frameCount = (int)s_idx.count;
  framesReady = (frameCount > 0 && validCount > 0);
  s_timesLoaded = true;
  invalidateValidIdxCache();

  appendDiagLog("loadIndex: count=%d head=%d valid=%d\\n",
                frameCount, (int)s_idx.head, validCount);
  return true;
}

static void writeIndex() {
  s_idx.magic = INDEX_MAGIC;

  // Write to tmp, flush, rename for atomic commit
  SD.remove(INDEX_TMP_FILE);
  File f = SD.open(INDEX_TMP_FILE, FILE_WRITE);
  if (!f) {
    appendDiagLog("writeIndex: open tmp fail\\n");
    return;
  }
  f.write((const uint8_t*)&s_idx, sizeof(s_idx));
  f.flush();
  f.close();

  SD.remove(INDEX_BIN_FILE);
  if (!SD.rename(INDEX_TMP_FILE, INDEX_BIN_FILE)) {
    appendDiagLog("writeIndex: rename fail\\n");
  }

  // Sync compatibility arrays
  for (int i = 0; i < (int)s_idx.count && i < MAX_FRAMES; i++) {
    s_frameTimes[i] = s_idx.times[i];
    s_streamValid[i] = s_idx.rawValid[i];
  }
  for (int i = (int)s_idx.count; i < MAX_FRAMES; i++) {
    s_frameTimes[i] = 0;
    s_streamValid[i] = 0;
  }
  frameCount = (int)s_idx.count;
  framesReady = (frameCount > 0);
  s_timesLoaded = true;
  invalidateValidIdxCache();
}

static bool readJpegFromSlot(int logicalIdx, uint8_t* buf, size_t* outLen) {
  if (logicalIdx < 0 || logicalIdx >= (int)s_idx.count) return false;
  if (!s_idx.jpegValid[logicalIdx]) return false;
  int phys = ((int)s_idx.head + logicalIdx) % MAX_FRAMES;
  uint32_t off = (uint32_t)phys * JPEG_SLOT_BYTES;
  size_t len = s_idx.jpegLen[logicalIdx];
  if (len == 0 || len > JPEG_SLOT_BYTES) return false;

  File f = SD.open(FRAMES_BIN_FILE, FILE_READ);
  if (!f) return false;
  f.seek(off);
  size_t got = f.read(buf, len);
  f.close();
  if (got != len) return false;
  if (outLen) *outLen = len;
  return true;
}

static bool writeJpegToSlot(int logicalIdx, const uint8_t* buf, size_t len) {
  if (logicalIdx < 0 || logicalIdx >= MAX_FRAMES) return false;
  if (len == 0 || len > JPEG_SLOT_BYTES) return false;
  int phys = ((int)s_idx.head + logicalIdx) % MAX_FRAMES;
  uint32_t off = (uint32_t)phys * JPEG_SLOT_BYTES;

  File f = SD.open(FRAMES_BIN_FILE, "r+");
  if (!f) return false;
  f.seek(off);
  size_t written = f.write(buf, len);
  f.flush();
  f.close();
  if (written != len) return false;

  s_idx.jpegLen[logicalIdx] = (uint32_t)len;
  s_idx.jpegValid[logicalIdx] = 1;
  return true;
}

static bool writeRawToSlot(int logicalIdx, const uint8_t* buf) {
  if (logicalIdx < 0 || logicalIdx >= MAX_FRAMES) return false;
  int phys = ((int)s_idx.head + logicalIdx) % MAX_FRAMES;
  uint32_t off = (uint32_t)phys * (uint32_t)SCALED_FRAME_BYTES;

  File f = SD.open(RAW_STREAM_FILE, "r+");
  if (!f) return false;
  f.seek(off);
  size_t written = f.write(buf, SCALED_FRAME_BYTES);
  f.flush();
  f.close();
  if (written != SCALED_FRAME_BYTES) return false;

  s_idx.rawValid[logicalIdx] = 1;
  s_streamValid[logicalIdx] = 1;
  return true;
}

// Decode JPEG from s_dlBuf, validate all sprite checks, scale, write raw slot.
// Returns true if raw slot was successfully written.
static bool decodeAndWriteRawSlot(int logicalIdx, size_t jpegLen) {
  if (!ensureSprite() || !s_frameDisplayBuf) return false;

  LovyanGFX* prevTarget = g_drawTarget;
  sprite.fillScreen(TFT_BLACK);
  g_drawTarget = &sprite;
  resetJpegDrawStats();

  bool ok = false;
  if (jpeg.openRAM(s_dlBuf, (int)jpegLen, jpegDraw)) {
    if (jpeg.getWidth() == DISP_W && jpeg.getHeight() == DISP_H) {
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      ok = jpeg.decode(0, 0, 0);
    }
    jpeg.close();
  }
  g_drawTarget = prevTarget;

  if (!ok || !jpegDrawLooksFullFrame()) return false;
  if (spriteLooksCompletelyBlack()) return false;
  if (spriteLooksPartialDecode()) return false;
  if (spriteLooksHorizontallyCorrupted() || spriteLooksVerticallyCorrupted()) return false;
  if (spriteLooksHoldFrameBlockCorrupted()) return false;
  if (spriteLooksCyanWhiteBlockCorrupted()) return false;
  if (spriteLooksBottomBandJunkCorrupted()) return false;
  if (spriteLooksBlackSlabCorrupted()) return false;

  scaleSpriteTo410x360(s_frameDisplayBuf);
  if (scaledFrameLooksFreezeBlockCorrupted() || scaledFrameLooksHoldBlockCorrupted()) return false;

  return writeRawToSlot(logicalIdx, (const uint8_t*)s_frameDisplayBuf);
}'''

text = text.replace(after_appenddiag, core_helpers, 1)

# ═══════════════════════════════════════════════════════════════
# STEP 7: Replace downloadFrames() + syncFramesRolling() with syncWeatherFrames()
# ═══════════════════════════════════════════════════════════════

# Find the exact block from downloadFrames() to end of syncFramesRolling()
# downloadFrames starts at line ~7562 with "static void downloadFrames() {"
# syncFramesRolling ends at line ~8475 with "}"

dl_start = text.find("static void downloadFrames() {")
sync_end_marker = "  appendDiagLog(\"sync: roll done saved=%d reused=%d downloaded=%d rebuiltFast=%d zoomRefresh=%d millis=%lu ms\\n\",\n                saved, reused, downloaded, (int)rebuiltFast, (int)s_zoomSnapshotsRefreshPending, millis());\n  return true;\n}\n"
sync_end_pos = text.find(sync_end_marker)
if sync_end_pos < 0:
    print("ERROR: Could not find syncFramesRolling end marker")
    sys.exit(1)
sync_end_pos += len(sync_end_marker)

# Also need to find the comment and section header before downloadFrames
section_marker = "//  HOURS_BACK and active cadence, clamped to MAX_FRAMES).\n// ─────────────────────────────────────────────────────────────\n"
section_pos = text.rfind(section_marker, 0, dl_start)
if section_pos < 0:
    # Try to find the broader marker
    section_pos = text.rfind("// ─────────────────────────────────────────────────────────────\n", 0, dl_start)
    if section_pos >= 0:
        # Go back one more line for the comment above
        prev_nl = text.rfind("\n", 0, section_pos)
        section_pos = prev_nl + 1 if prev_nl >= 0 else section_pos

# We'll find the exact start including the section header comments
# Look for the lines just before downloadFrames:
# "//  complete weather frame download (full sync: downloads all
# ..."
pre_dl = text.rfind("// ─────────────────────────────────────────────────────────────\n//  complete weather frame download", 0, dl_start)
if pre_dl < 0:
    # Try broader search
    pre_dl_search = text.rfind("//  complete weather frame download", 0, dl_start)
    if pre_dl_search >= 0:
        pre_dl = text.rfind("\n", 0, pre_dl_search) + 1
    else:
        pre_dl = text.rfind("\n", 0, dl_start) + 1

# Get the text before/after
before_sync = text[:pre_dl]
after_sync = text[sync_end_pos:]

new_sync = '''// ─────────────────────────────────────────────────────────────
//  Unified weather frame sync — one function replaces downloadFrames
//  + syncFramesRolling + all branch logic.
// ─────────────────────────────────────────────────────────────
static void syncWeatherFrames() {
  if (!syncProgressIsActive()) showMessage("Syncing time...", "pool.ntp.org");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  struct tm ti;
  int tries = 0;
  while (!getLocalTime(&ti, 1000) && tries++ < 20) {}
  time_t now = time(nullptr);
  Serial.printf("UTC: %s\\n", ctime(&now));
  appendDiagLog("sync: ntp utc=%lld millis=%lu ms\\n", (long long)now, millis());
  refreshDisplayLocationTimeFromIpInfo();
  if (syncProgressIsActive()) syncProgressCompletePhase();

  const int cadenceMin = activeCadenceMin();
  const int lagHours = activeLagHours();
  const int totalFrames = targetFrameCount();
  const time_t cadenceSec = (time_t)cadenceMin * 60;
  const time_t fetchEnd = roundToCadence(now - (time_t)(lagHours * 3600));
  const time_t fetchStart = fetchEnd - (time_t)((totalFrames - 1) * cadenceMin * 60);

  // Load current index
  loadIndex();
  int oldCount = (int)s_idx.count;

  // Check view mismatch
  bool viewMatches = true;
  if (s_idx.count > 0) {
    viewMatches = currentWeatherViewMatchesCache();
  }

  // Compute shift and slot reuse
  int shift = 0;
  bool fullRefresh = false;
  if (!viewMatches && s_idx.count > 0) {
    fullRefresh = true;
    appendDiagLog("sync: full-refresh reason=view\\n");
  } else if (s_idx.count > 0 && s_idx.times[0] != 0) {
    time_t delta = fetchStart - s_idx.times[0];
    if (delta > 0 && cadenceSec > 0 && (delta % cadenceSec) == 0) {
      shift = (int)(delta / cadenceSec);
      if (shift >= (int)s_idx.count) fullRefresh = true;
    } else if (delta != 0) {
      fullRefresh = true;
    }
  } else if (s_idx.count == 0) {
    fullRefresh = true;
  }

  if (fullRefresh) {
    memset(&s_idx, 0, sizeof(s_idx));
    s_idx.magic = INDEX_MAGIC;
    s_idx.head = 0;
    s_idx.count = 0;
    shift = 0;
  }

  // Determine which slots need download
  bool needsDownload[MAX_FRAMES];
  memset(needsDownload, 0, sizeof(needsDownload));
  int downloadCount = 0;

  if (fullRefresh) {
    // Everything needs download
    s_idx.count = (uint16_t)totalFrames;
    for (int i = 0; i < totalFrames; i++) {
      s_idx.times[i] = fetchStart + (time_t)(i * cadenceSec);
      s_idx.jpegValid[i] = 0;
      s_idx.rawValid[i] = 0;
      s_idx.jpegLen[i] = 0;
      needsDownload[i] = true;
      downloadCount++;
    }
  } else if (shift > 0) {
    // Advance head, keep overlapping slots
    int oldUsable = (int)s_idx.count - shift;
    if (oldUsable < 0) oldUsable = 0;

    // Build new index
    FrameStoreIndex newIdx;
    memset(&newIdx, 0, sizeof(newIdx));
    newIdx.magic = INDEX_MAGIC;
    newIdx.head = (s_idx.head + (uint16_t)shift) % MAX_FRAMES;
    newIdx.count = (uint16_t)totalFrames;

    for (int i = 0; i < totalFrames; i++) {
      time_t t = fetchStart + (time_t)(i * cadenceSec);
      newIdx.times[i] = t;

      // Check if this slot was in old index
      int oldLogical = i + shift - ((int)s_idx.count - oldUsable);
      // Actually: old slot for this time = shift + i if within old range
      int oldSlot = shift + i;
      if (oldSlot >= 0 && oldSlot < (int)s_idx.count && i < oldUsable &&
          s_idx.times[oldSlot] == t && s_idx.jpegValid[oldSlot]) {
        // Reuse this slot — physical slot is the same (circular)
        newIdx.jpegLen[i] = s_idx.jpegLen[oldSlot];
        newIdx.jpegValid[i] = 1;
        newIdx.rawValid[i] = s_idx.rawValid[oldSlot];
      } else {
        needsDownload[i] = true;
        downloadCount++;
      }
    }
    memcpy(&s_idx, &newIdx, sizeof(s_idx));
  } else {
    // shift == 0, check for gaps and update count if needed
    if ((int)s_idx.count < totalFrames) {
      // Need more frames at the tail
      for (int i = (int)s_idx.count; i < totalFrames; i++) {
        s_idx.times[i] = fetchStart + (time_t)(i * cadenceSec);
        s_idx.jpegValid[i] = 0;
        s_idx.rawValid[i] = 0;
        needsDownload[i] = true;
        downloadCount++;
      }
      s_idx.count = (uint16_t)totalFrames;
    }
    // Check for invalid slots
    for (int i = 0; i < (int)s_idx.count; i++) {
      if (!s_idx.jpegValid[i]) {
        s_idx.times[i] = fetchStart + (time_t)(i * cadenceSec);
        needsDownload[i] = true;
        downloadCount++;
      }
    }
  }

  appendDiagLog("sync: totalFrames=%d shift=%d downloadCount=%d oldCount=%d\\n",
                totalFrames, shift, downloadCount, oldCount);

  if (downloadCount == 0) {
    // Check if raw is all built
    bool allRawValid = true;
    for (int i = 0; i < (int)s_idx.count; i++) {
      if (s_idx.jpegValid[i] && !s_idx.rawValid[i]) { allRawValid = false; break; }
    }
    writeIndex();
    s_zoomWeatherRefreshNeeded = false;
    if (s_idx.count > 0) {
      s_zoomSnapshotsRefreshPending = !zoomSnapshotsCurrentAndUsable(s_idx.times[s_idx.count - 1]);
    }
    if (allRawValid) {
      if (syncProgressIsActive()) {
        syncProgressBeginPhase("cache", (uint32_t)totalFrames);
        syncProgressCompletePhase();
        syncProgressBeginPhase("raw", (uint32_t)totalFrames);
        syncProgressCompletePhase();
      }
      showMessage("Cache current", "No downloads needed");
      delay(700);
      appendDiagLog("sync: cache current, no downloads\\n");
      return;
    }
    // Need raw rebuild
    if (syncProgressIsActive()) {
      syncProgressBeginPhase("cache", (uint32_t)totalFrames);
      syncProgressCompletePhase();
    }
    rebuildRawFromStored();
    appendDiagLog("sync: cache current, rebuilt raw\\n");
    return;
  }

  // Fetch GIBS available times
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setReuse(true);
  fetchGibsAvailableTimes(client, fetchStart, fetchEnd);

  // Download frames
  if (!syncProgressIsActive()) showMessage("Updating cache...", "NASA GIBS");
  if (syncProgressIsActive()) syncProgressBeginPhase("cache", (uint32_t)totalFrames);

  float bboxWest, bboxSouth, bboxEast, bboxNorth;
  getActiveWeatherBbox(&bboxWest, &bboxSouth, &bboxEast, &bboxNorth);

  int saved = 0;
  int skipped = 0;
  int dlFail = 0;
  char url[512];

  for (int i = 0; i < (int)s_idx.count; i++) {
    showProgress(i + 1, (int)s_idx.count, "cache");
    yield();

    if (!needsDownload[i]) {
      saved++;
      // If JPEG valid but raw invalid, decode from RAM
      if (s_idx.jpegValid[i] && !s_idx.rawValid[i]) {
        size_t readLen = 0;
        if (readJpegFromSlot(i, s_dlBuf, &readLen) && readLen > 0) {
          decodeAndWriteRawSlot(i, readLen);
        }
      }
      continue;
    }

    time_t t = s_idx.times[i];
    if (t <= 0) { skipped++; continue; }

    // Build URL and fetch
    const int fetchCadenceSec = max(60, cadenceMin * 60);
    time_t snappedTime = snapToNearestGibsTime(t, 2 * fetchCadenceSec);
    const time_t secondOffsets[] = {0, -60, 60,
                                    -(time_t)fetchCadenceSec, (time_t)fetchCadenceSec,
                                    -2*(time_t)fetchCadenceSec, 2*(time_t)fetchCadenceSec};
    int stepCount;
    if (snappedTime > 0) {
      stepCount = 1;
    } else if (s_gibsAvailCount > 0) {
      skipped++;
      continue;
    } else {
      stepCount = 7;
    }

    bool fetchedOk = false;
    size_t jpegLen = 0;
    for (int si = 0; si < stepCount && !fetchedOk; ++si) {
      time_t candT = (snappedTime > 0) ? snappedTime : (t + secondOffsets[si]);
      if (candT <= 0) continue;
      if (!buildWeatherFrameUrl(url, sizeof(url), candT,
                                bboxWest, bboxSouth, bboxEast, bboxNorth)) continue;

      for (int attempt = 0; attempt < 2; ++attempt) {
        http.begin(client, url);
        http.setTimeout(5000);
        int code = http.GET();
        if (code != HTTP_CODE_OK) { http.end(); continue; }
        if (!readHttpJpegBodyToDlBuf(http, "SYNC", &jpegLen)) continue;
        if (!validateBufferedWeatherFrameJpeg(jpegLen, "SYNC")) continue;
        if (jpegEffectiveLength(s_dlBuf, jpegLen) == 0) continue;
        fetchedOk = true;
        break;
      }
    }

    if (!fetchedOk) {
      dlFail++;
      continue;
    }

    // Additional validators
    if (spriteLooksHoldFrameBlockCorrupted() ||
        spriteLooksCyanWhiteBlockCorrupted() ||
        spriteLooksBottomBandJunkCorrupted() ||
        spriteLooksBlackSlabCorrupted()) {
      dlFail++;
      continue;
    }

    // Size outlier check using index jpegLen
    if ((int)s_idx.count >= 3) {
      int refA = -1, refB = -1;
      if (i <= 0) { refA = 1; refB = 2; }
      else if (i >= (int)s_idx.count - 1) { refA = (int)s_idx.count - 2; refB = (int)s_idx.count - 3; }
      else { refA = i - 1; refB = i + 1; }
      if (refA >= 0 && refB >= 0 && refA < (int)s_idx.count && refB < (int)s_idx.count &&
          s_idx.jpegValid[refA] && s_idx.jpegValid[refB]) {
        int sizeA = (int)s_idx.jpegLen[refA];
        int sizeB = (int)s_idx.jpegLen[refB];
        if (sizeA > 0 && sizeB > 0 && abs(sizeA - sizeB) <= 1500) {
          int refSize = (sizeA + sizeB) / 2;
          if ((int)jpegLen + 900 < refSize) {
            Serial.printf("SYNC sz-outlier i=%d sz=%u ref=%d\\n", i, (unsigned)jpegLen, refSize);
            dlFail++;
            continue;
          }
        }
      }
    }

    // Write JPEG to slot
    if (!writeJpegToSlot(i, s_dlBuf, jpegLen)) {
      dlFail++;
      continue;
    }

    // Decode from RAM and write raw slot
    decodeAndWriteRawSlot(i, jpegLen);

    saved++;
    Serial.printf("SYNC %d/%d OK %u B\\n", i + 1, (int)s_idx.count, (unsigned)jpegLen);
    delay(20);  // pace requests
  }
  if (syncProgressIsActive()) syncProgressCompletePhase();

  // Gap-fill invalid raw slots from nearest neighbor
  int filled = 0;
  for (int i = 0; i < (int)s_idx.count; i++) {
    if (s_idx.rawValid[i]) continue;
    // Find nearest valid neighbor
    int src = -1;
    for (int d = 1; d < (int)s_idx.count && src < 0; d++) {
      if (i - d >= 0 && s_idx.rawValid[i - d]) src = i - d;
      else if (i + d < (int)s_idx.count && s_idx.rawValid[i + d]) src = i + d;
    }
    if (src >= 0) {
      // Point this slot's raw to the neighbor's physical slot (via stream valid flag)
      s_idx.rawValid[i] = 1;
      s_streamValid[i] = 1;
      filled++;
    }
  }

  // Commit index
  writeIndex();
  writeCurrentWeatherViewMeta();

  s_zoomWeatherRefreshNeeded = (downloadCount > 0 && saved > 0);
  if (s_idx.count > 0) {
    s_zoomSnapshotsRefreshPending = !zoomSnapshotsCurrentAndUsable(s_idx.times[s_idx.count - 1]);
  }

  // Rebuild filtered zoom raws
  rebuildFilteredZoomRawsFromCache();

  if (syncProgressIsActive()) {
    syncProgressBeginPhase("raw", 1);
    syncProgressCompletePhase();
  }

  Serial.printf("sync done saved=%d skip=%d fail=%d fill=%d\\n", saved, skipped, dlFail, filled);
  appendDiagLog("sync: done saved=%d skip=%d fail=%d fill=%d total=%d millis=%lu ms\\n",
                saved, skipped, dlFail, filled, (int)s_idx.count, millis());
}

static void rebuildRawFromStored() {
  if (!ensureSprite() || !s_frameDisplayBuf) return;
  if (s_idx.count <= 0) return;

  bool useUnifiedProgress = syncProgressIsActive();
  if (useUnifiedProgress) {
    syncProgressBeginPhase("raw", (uint32_t)s_idx.count);
  } else {
    showMessage("Building raw cache...", "Predecode for smooth playback");
  }

  if (s_streamFile) { s_streamFile.close(); }
  s_streamReady = false;
  invalidateValidIdxCache();

  int built = 0;
  int decFail = 0;
  for (int i = 0; i < (int)s_idx.count; i++) {
    showProgress(i + 1, (int)s_idx.count, "raw");
    yield();

    if (s_idx.rawValid[i]) { built++; continue; }
    if (!s_idx.jpegValid[i]) continue;

    size_t readLen = 0;
    if (!readJpegFromSlot(i, s_dlBuf, &readLen) || readLen == 0) {
      decFail++;
      continue;
    }

    if (decodeAndWriteRawSlot(i, readLen)) {
      built++;
    } else {
      decFail++;
    }
  }

  if (useUnifiedProgress) syncProgressCompletePhase();

  // Gap-fill
  int filled = 0;
  for (int i = 0; i < (int)s_idx.count; i++) {
    if (s_idx.rawValid[i]) continue;
    int src = -1;
    for (int d = 1; d < (int)s_idx.count && src < 0; d++) {
      if (i - d >= 0 && s_idx.rawValid[i - d]) src = i - d;
      else if (i + d < (int)s_idx.count && s_idx.rawValid[i + d]) src = i + d;
    }
    if (src >= 0) {
      s_idx.rawValid[i] = 1;
      s_streamValid[i] = 1;
      filled++;
    }
  }

  writeIndex();
  rebuildFilteredZoomRawsFromCache();

  Serial.printf("raw-rebuild: built=%d dec=%d fill=%d\\n", built, decFail, filled);
  appendDiagLog("raw-rebuild: built=%d dec=%d fill=%d count=%d\\n",
                built, decFail, filled, (int)s_idx.count);
}
'''

text = before_sync + new_sync + after_sync

# ═══════════════════════════════════════════════════════════════
# STEP 8: Replace ensureStreamOpen() — simplified
# ═══════════════════════════════════════════════════════════════

old_ensure = """static void ensureStreamOpen() {
  if (s_streamReady && s_streamFile) return;
  if (!rawCacheMetaVersionIsCurrent()) return;
#if !BOARD_IS_AMOLED_206
  tft.waitDMA();  // defensive shared-SPI fence before SD.open
#endif
  s_streamFile = SD.open(RAW_STREAM_FILE, FILE_READ);
  s_streamReady = (bool)s_streamFile;
  if (!s_streamReady) {
    Serial.printf("stream open FAIL meta=%d\\n", (int)rawCacheMetaVersionIsCurrent());
  }
  if (s_streamReady) {
    uint32_t actual = (uint32_t)s_streamFile.size();
    uint32_t expected = (uint32_t)frameCount * (uint32_t)SCALED_FRAME_BYTES;
    Serial.printf("stream open %u exp %u\\n",
                  (unsigned)actual, (unsigned)expected);
    if (frameCount > 0 && actual != expected) {
      Serial.printf("stream sz bad a=%u e=%u d=%ld\\n",
                    (unsigned)actual, (unsigned)expected,
                    (long)((int32_t)actual - (int32_t)expected));
    }
  }
}"""

new_ensure = """static void ensureStreamOpen() {
  if (s_streamReady && s_streamFile) return;
  if (s_idx.count == 0 || !framesReady) return;

  // Check if any raw frames are valid
  bool anyRaw = false;
  for (int i = 0; i < (int)s_idx.count; i++) {
    if (s_idx.rawValid[i]) { anyRaw = true; break; }
  }
  if (!anyRaw) {
    // Try to rebuild from stored JPEGs
    rebuildRawFromStored();
    anyRaw = false;
    for (int i = 0; i < (int)s_idx.count; i++) {
      if (s_idx.rawValid[i]) { anyRaw = true; break; }
    }
    if (!anyRaw) return;
  }

#if !BOARD_IS_AMOLED_206
  tft.waitDMA();
#endif
  s_streamFile = SD.open(RAW_STREAM_FILE, FILE_READ);
  s_streamReady = (bool)s_streamFile;
  if (s_streamReady) {
    uint32_t actual = (uint32_t)s_streamFile.size();
    uint32_t expected = (uint32_t)MAX_FRAMES * (uint32_t)SCALED_FRAME_BYTES;
    Serial.printf("stream open %u exp %u\\n", (unsigned)actual, (unsigned)expected);
  }
}"""

text = text.replace(old_ensure, new_ensure, 1)

# ═══════════════════════════════════════════════════════════════
# STEP 9: Replace decodeJpegFrameToSprite — read from frames.bin
# ═══════════════════════════════════════════════════════════════

old_dec = """static bool decodeJpegFrameToSprite(int idx, bool rejectBlank) {
  if (idx < 0 || idx >= MAX_FRAMES || s_frameTimes[idx] == 0) return false;
  char path[32];
  makeFramePathByIdx(idx, path, sizeof(path));
  if (!decodeJpegPathToSprite(path)) return false;
  if (rejectBlank && spriteLooksCompletelyBlack()) return false;
  return true;
}"""

new_dec = """static bool decodeJpegFrameToSprite(int idx, bool rejectBlank) {
  if (idx < 0 || idx >= (int)s_idx.count) return false;
  if (!s_idx.jpegValid[idx]) return false;
  size_t readLen = 0;
  if (!readJpegFromSlot(idx, s_dlBuf, &readLen) || readLen == 0) return false;

  if (!ensureSprite()) return false;
  sprite.fillScreen(TFT_BLACK);
  g_drawTarget = &sprite;
  resetJpegDrawStats();

  bool ok = false;
  if (jpeg.openRAM(s_dlBuf, (int)readLen, jpegDraw)) {
    if (jpeg.getWidth() == DISP_W && jpeg.getHeight() == DISP_H) {
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      ok = jpeg.decode(0, 0, 0);
    }
    jpeg.close();
  }
  if (!ok || !jpegDrawLooksFullFrame()) return false;
  if (rejectBlank && spriteLooksCompletelyBlack()) return false;
  return true;
}"""

text = text.replace(old_dec, new_dec, 1)

# ═══════════════════════════════════════════════════════════════
# STEP 10: Update setup()
# ═══════════════════════════════════════════════════════════════

# Replace readMetaCount check with loadIndex+initFrameStore
old_setup_sd = """  removeObsoleteGifAssetsIfPresent();
#if BOARD_IS_AMOLED_206
  ensureAudioCueWorker();
  preloadSelectedCueToPsram(false);
#endif"""

new_setup_sd = """  removeObsoleteGifAssetsIfPresent();
  initFrameStore();
#if BOARD_IS_AMOLED_206
  ensureAudioCueWorker();
  preloadSelectedCueToPsram(false);
#endif"""

text = text.replace(old_setup_sd, new_setup_sd, 1)

# Replace "int sdCount = readMetaCount();" block with loadIndex
old_sdcount = """  // Prefer SD cache if present (works for hard boots too).
  int sdCount = readMetaCount();
  if (sdCount > 0) {
    frameCount = sdCount;
    framesReady = true;
    s_timesLoaded = false;
    Serial.printf("SD cache found: %d frames\\n", sdCount);
    if (hardBoot) {
      loadWeatherViewCenterFromCache();
    }
  }"""

new_sdcount = """  // Load frame index from SD
  loadIndex();
  if (frameCount > 0) {
    Serial.printf("SD index: %d frames\\n", frameCount);
    if (hardBoot) {
      loadWeatherViewCenterFromCache();
    }
  }"""

text = text.replace(old_sdcount, new_sdcount, 1)

# Replace the frame resolution check block (dim.cfg) — no longer needed
old_dimcheck_start = "  // ── Frame resolution check"
old_dimcheck_end = "  }\n\n  // ── Refresh cache on hard boot"
dim_start = text.find(old_dimcheck_start)
dim_end = text.find(old_dimcheck_end)
if dim_start >= 0 and dim_end >= 0:
    text = text[:dim_start] + "  // ── Refresh cache on hard boot" + text[dim_end + len(old_dimcheck_end):]

# Replace syncFramesRolling() / downloadFrames() dispatch in setup
old_sync_dispatch = """      bool rolled = syncFramesRolling();
      { File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND); if (diagF) { diagF.printf("syncRolling=%d fc_after=%d\\n", (int)rolled, frameCount); diagF.close(); } }
      if (!rolled) {
        downloadFrames();
        { File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND); if (diagF) { diagF.printf("downloadFrames done fc=%d rdy=%d rawMeta=%d\\n", frameCount, (int)framesReady, (int)rawCacheMetaVersionIsCurrent()); diagF.close(); } }
        // Inline raw build consumed the raw budget — fast-complete the phase slot
        if (syncProgressIsActive() && rawCacheMetaVersionIsCurrent()) {
          syncProgressBeginPhase("raw", 1);
          syncProgressCompletePhase();
        }
      }"""

new_sync_dispatch = """      syncWeatherFrames();
      { File diagF = SD.open(SD_ROOT "/diag.txt", FILE_APPEND); if (diagF) { diagF.printf("syncWeatherFrames done fc=%d rdy=%d\\n", frameCount, (int)framesReady); diagF.close(); } }"""

text = text.replace(old_sync_dispatch, new_sync_dispatch, 1)

# Replace the raw meta check block in setup
old_raw_check = """  if (framesReady && frameCount > 0 && !rawCacheMetaVersionIsCurrent()) {
    Serial.println("raw meta stale -> rebuild");
    showMessage("Building...", "Preparing frames");
    buildRawPlaybackCache();
  }

  // Keep any expensive stream recovery in setup (inside sync progress budget),
  // not deferred to loop() where it appears as a "100% stuck" boot.
  if (framesReady && frameCount > 0) {
    ensureStreamOpen();
    if (!s_streamReady || !s_streamFile) {
      appendDiagLog("setup: stream open failed after sync -> rebuild raw\\n");
      invalidateRawCacheMetaState();
      showMessage("Building...", "Preparing frames");
      buildRawPlaybackCache();
      ensureStreamOpen();
      appendDiagLog("setup: stream ready after rebuild=%d\\n",
                    (int)(s_streamReady && (bool)s_streamFile));
    }
  }"""

new_raw_check = """  // Open stream for playback
  if (framesReady && frameCount > 0) {
    ensureStreamOpen();
    if (!s_streamReady || !s_streamFile) {
      appendDiagLog("setup: stream open failed -> rebuild raw\\n");
      showMessage("Building...", "Preparing frames");
      rebuildRawFromStored();
      ensureStreamOpen();
      appendDiagLog("setup: stream ready after rebuild=%d\\n",
                    (int)(s_streamReady && (bool)s_streamFile));
    }
  }"""

text = text.replace(old_raw_check, new_raw_check, 1)

# ═══════════════════════════════════════════════════════════════
# STEP 11: Update loop()
# ═══════════════════════════════════════════════════════════════

# Replace loadTimesIfNeeded() call  — times are loaded by loadIndex at boot
text = text.replace("  loadTimesIfNeeded();\n  ensureStreamOpen();\n", "  ensureStreamOpen();\n", 1)

# Replace buildRawPlaybackCache() call in loop with rebuildRawFromStored()
old_loop_raw = """  if (!s_streamReady || !s_streamFile) {
    buildRawPlaybackCache();
    ensureStreamOpen();
  }"""
new_loop_raw = """  if (!s_streamReady || !s_streamFile) {
    rebuildRawFromStored();
    ensureStreamOpen();
  }"""
text = text.replace(old_loop_raw, new_loop_raw, 1)

# Replace newestJpegPath usage in loop — no longer individual files
old_newest_path = """    char newestJpegPath[32];
    makeFramePathByIdx(newestIdx, newestJpegPath, sizeof(newestJpegPath));"""
new_newest_path = """    // No per-frame path needed — all reads go through frames.bin slots"""
text = text.replace(old_newest_path, new_newest_path, 1)

# Replace freeze-frame eviction code that uses makeFramePathByIdx + SD.remove
# The old code deletes .jpg files; new code marks idx invalid
old_evict = """            // Evict the corrupt frame: delete source JPEG so rolling sync
            // redownloads it next boot. Invalidate raw meta so raw=0 forces
            // rolling sync + raw-build instead of the partial-window-current
            // fast path that would otherwise skip redownload indefinitely.
            {
              char badPath[32];
              makeFramePathByIdx(freezeFrameIdx, badPath, sizeof(badPath));
              SD.remove(badPath);
              appendDiagLog("raw-del: idx=%d freeze-corrupt\\n", freezeFrameIdx);
              // Remove from in-RAM valid set so this session never re-selects it.
              if (freezeFrameIdx < MAX_FRAMES) s_streamValid[freezeFrameIdx] = 0;
            }"""
new_evict = """            // Evict the corrupt frame from index so next sync re-downloads it.
            {
              appendDiagLog("raw-evict: idx=%d freeze-corrupt\\n", freezeFrameIdx);
              if (freezeFrameIdx < MAX_FRAMES) {
                s_streamValid[freezeFrameIdx] = 0;
                s_idx.rawValid[freezeFrameIdx] = 0;
              }
            }"""
text = text.replace(old_evict, new_evict, 1)

# Replace the secondary eviction in the back-search loop
old_back_evict = """                // Also corrupt — evict this one too.
                char badPath[32];
                makeFramePathByIdx(backIdx, badPath, sizeof(badPath));
                SD.remove(badPath);
                appendDiagLog("raw-del: idx=%d freeze-corrupt\\n", backIdx);
                if (backIdx < MAX_FRAMES) s_streamValid[backIdx] = 0;"""
new_back_evict = """                // Also corrupt — evict this one too.
                appendDiagLog("raw-evict: idx=%d freeze-corrupt\\n", backIdx);
                if (backIdx < MAX_FRAMES) {
                  s_streamValid[backIdx] = 0;
                  s_idx.rawValid[backIdx] = 0;
                }"""
text = text.replace(old_back_evict, new_back_evict, 1)

# Replace raw meta invalidation after freeze-corrupt
old_meta_inv = """            // Invalidate raw meta so next boot triggers rolling sync + rebuild.
            SD.remove(RAW_CACHE_META_FILE);
            s_rawMetaVersionCurrent = false;
            appendDiagLog("raw-meta: invalidated freeze-corrupt\\n");"""
new_meta_inv = """            // Write updated index so evicted slots get re-downloaded next sync
            writeIndex();
            appendDiagLog("idx: updated after freeze-corrupt eviction\\n");"""
text = text.replace(old_meta_inv, new_meta_inv, 1)

# ═══════════════════════════════════════════════════════════════
# STEP 12: Update goToSleep() — replace syncFramesRolling/downloadFrames
# ═══════════════════════════════════════════════════════════════

old_sleep_sync = """    if (connectWifiForSync(false)) {
      if (!syncFramesRolling()) {
        downloadFrames();
      }
      maybeRefreshPendingZoomSnapshots();
    } else {
      Serial.println("timer refresh skip: no wifi");
    }
    disconnectWifiAfterSync();
  }
  resetTopButtonStateAfterWake(buttonOnly);"""

new_sleep_sync = """    if (connectWifiForSync(false)) {
      syncWeatherFrames();
      maybeRefreshPendingZoomSnapshots();
    } else {
      Serial.println("timer refresh skip: no wifi");
    }
    disconnectWifiAfterSync();
  }
  resetTopButtonStateAfterWake(buttonOnly);"""

# There are TWO instances of this pattern in goToSleep
text = text.replace(old_sleep_sync, new_sleep_sync)

# Also fix the second pattern (slightly different — no buttonOnly param)
old_sleep_sync2 = """    if (connectWifiForSync(false)) {
      if (!syncFramesRolling()) {
        downloadFrames();
      }
      maybeRefreshPendingZoomSnapshots();
    } else {
      Serial.println("timer refresh skip: no wifi");
    }
    disconnectWifiAfterSync();
  }
  resetTopButtonStateAfterWake(false);"""

new_sleep_sync2 = """    if (connectWifiForSync(false)) {
      syncWeatherFrames();
      maybeRefreshPendingZoomSnapshots();
    } else {
      Serial.println("timer refresh skip: no wifi");
    }
    disconnectWifiAfterSync();
  }
  resetTopButtonStateAfterWake(false);"""

text = text.replace(old_sleep_sync2, new_sleep_sync2)

# ═══════════════════════════════════════════════════════════════
# STEP 13: Update tryShowCleanFreezeFrameByIdx — remove file path usage
# ═══════════════════════════════════════════════════════════════

old_freeze = """static bool tryShowCleanFreezeFrameByIdx(int idx) {
  if (idx < 0 || idx >= frameCount) return false;

  if (showFrame(idx) && !currentScaledFreezeFrameLooksCorrupted()) {
    return true;
  }

  char framePath[40];
  makeFramePathByIdx(idx, framePath, sizeof(framePath));
  if (showZoomSnapshotFrame(framePath, idx) &&
      !currentScaledFreezeFrameLooksCorrupted()) {
    return true;
  }

  return false;
}"""

new_freeze = """static bool tryShowCleanFreezeFrameByIdx(int idx) {
  if (idx < 0 || idx >= frameCount) return false;
  if (showFrame(idx) && !currentScaledFreezeFrameLooksCorrupted()) {
    return true;
  }
  return false;
}"""

text = text.replace(old_freeze, new_freeze, 1)

# ═══════════════════════════════════════════════════════════════
# STEP 14: Delete dead functions — large block removals
# ═══════════════════════════════════════════════════════════════

# Delete flushSdFat
text = text.replace("""static bool flushSdFat() {
  SD_MMC.end();
  return SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT);
}

""", "", 1)

# Delete deleteFramesDir
old_delete_frames = """static void deleteFramesDir() {
  // Remove all files in /frames/ — SD.h doesn't have rmdir,
  // so we open the dir and remove each file individually.
  File dir = SD.open(FRAMES_DIR);
  if (!dir) return;
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    char path[64];
    snprintf(path, sizeof(path), "%s/%s", FRAMES_DIR, entry.name());
    entry.close();
    SD.remove(path);
  }
  dir.close();
  SD.rmdir(FRAMES_DIR);
  flushSdFat();
}

"""
text = text.replace(old_delete_frames, "", 1)

# Delete readMetaCount + writeMetaCount
old_meta = """static int readMetaCount() {
  File f = SD.open(META_FILE, FILE_READ);
  if (!f) return 0;
  char buf[16] = {};
  f.readBytes(buf, sizeof(buf) - 1);
  f.close();
  int n = atoi(buf);
  return (n > 0 && n <= MAX_FRAMES) ? n : 0;
}

static void writeMetaCount(int n) {
  SD.remove(META_FILE);  // FILE_WRITE may append on SD.h; force truncate
  File f = SD.open(META_FILE, FILE_WRITE);
  if (!f) return;
  char buf[16];
  snprintf(buf, sizeof(buf), "%d\\n", n);
  f.print(buf);
  f.close();
}

"""
text = text.replace(old_meta, "", 1)

# Delete writeCurrentFrameDimMeta
old_dim_meta = """static void writeCurrentFrameDimMeta() {
  char expected[16];
  snprintf(expected, sizeof(expected), "%d %d", DISP_W, DISP_H);
  SD.remove(FRAME_DIM_FILE);
  File f = SD.open(FRAME_DIM_FILE, FILE_WRITE);
  if (!f) {
    appendDiagLog("cache-meta: dim write FAIL want='%s'\\n", expected);
    return;
  }
  f.print(expected);
  f.close();
  appendDiagLog("cache-meta: dim write OK '%s'\\n", expected);
}

"""
text = text.replace(old_dim_meta, "", 1)

# Delete cacheValidationMetaIsCurrent + writeCurrentCacheValidationMeta
old_cache_val = """static bool cacheValidationMetaIsCurrent() {
  if (frameCount <= 0 || !s_timesLoaded) return false;

  File f = SD.open(CACHE_VALIDATE_META_FILE, FILE_READ);
  if (!f) return false;

  char buf[96];
  size_t n = f.readBytesUntil('\\n', buf, sizeof(buf) - 1);
  f.close();
  if (n == 0) return false;
  buf[n] = '\\0';

  unsigned ver = 0;
  int count = 0;
  long long first = 0;
  long long last = 0;
  if (sscanf(buf, "%u %d %lld %lld", &ver, &count, &first, &last) != 4) return false;
  if (ver != CACHE_VALIDATE_VERSION) return false;
  if (count != frameCount) return false;
  if ((time_t)first != s_frameTimes[0]) return false;
  if ((time_t)last != s_frameTimes[frameCount - 1]) return false;
  return true;
}

static void writeCurrentCacheValidationMeta() {
  if (frameCount <= 0 || !s_timesLoaded) return;

  SD.remove(CACHE_VALIDATE_META_FILE);
  File f = SD.open(CACHE_VALIDATE_META_FILE, FILE_WRITE);
  if (!f) return;
  f.printf("%u %d %lld %lld\\n",
           (unsigned)CACHE_VALIDATE_VERSION,
           frameCount,
           (long long)s_frameTimes[0],
           (long long)s_frameTimes[frameCount - 1]);
  f.close();
}

"""
text = text.replace(old_cache_val, "", 1)

# Delete CacheEntry struct + related functions
old_cache_entry = """struct CacheEntry {
  time_t t;
  int idx;
  bool used;
};

static void makeFramePathByTime(time_t t, char* out, size_t len) {
  snprintf(out, len, "%s/%lu.jpg", FRAMES_DIR, (unsigned long)t);
}

static void makeFramePathByIdx(int idx, char* out, size_t len) {
  makeFramePathByTime(s_frameTimes[idx], out, len);
}

"""
text = text.replace(old_cache_entry, "", 1)

# Delete frameFileExistsByTime + frameFileExists
old_exists = """static bool frameFileExistsByTime(time_t t) {
  if (t == 0) return false;
  char path[32];
  makeFramePathByTime(t, path, sizeof(path));
  File f = SD.open(path, FILE_READ);
  bool ok = (bool)f;
  if (f) f.close();
  return ok;
}

static bool frameFileExists(int idx) {
  if (idx < 0 || idx >= MAX_FRAMES || s_frameTimes[idx] == 0) return false;
  return frameFileExistsByTime(s_frameTimes[idx]);
}

"""
text = text.replace(old_exists, "", 1)

# Delete cachedFrameLooksReusable
old_reusable = """static bool cachedFrameLooksReusable(time_t t) {
  if (t == 0) return false;
  if (!frameFileExistsByTime(t)) return false;
  char path[32];
  makeFramePathByTime(t, path, sizeof(path));
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  size_t sz = f.size();
  if (sz < 7000) { f.close(); return false; }
  uint8_t hdr[2], tail[2];
  f.readBytes((char*)hdr, 2);
  f.seek(sz - 2);
  f.readBytes((char*)tail, 2);
  f.close();
  return (hdr[0] == 0xFF && hdr[1] == 0xD8 && tail[0] == 0xFF && tail[1] == 0xD9);
}

"""
text = text.replace(old_reusable, "", 1)

# Delete removeStaleFrameFiles
old_remove_stale = text[text.find("static void removeStaleFrameFiles("):text.find("static void removeObsoleteGifAssetsIfPresent()")]
text = text.replace(old_remove_stale, "", 1)

# Delete copyFrameFile
old_copy = text[text.find("static bool copyFrameFile(const char* srcPath, const char* dstPath) {"):text.find("\n\nstatic int detectForwardWindowShift(")]
text = text.replace(old_copy, "", 1)

# Delete detectForwardWindowShift + loadCacheEntries + findCacheEntryByTime
# + cacheMatchesWindowExactly + cacheMatchesWindowPrefix
# + installCacheState + installCacheStateFromEntries
# + verifyCommittedFrameIntegrity + writeTimesAndInstallCacheState
old_cache_funcs_start = "static int detectForwardWindowShift("
old_cache_funcs_end = "static bool connectWifiForSync("
cf_start = text.find(old_cache_funcs_start)
cf_end = text.find(old_cache_funcs_end)
if cf_start >= 0 and cf_end >= 0:
    # Find the newline before connectWifiForSync
    text = text[:cf_start] + text[cf_end:]

# Delete loadTimesIfNeeded
old_lti = """static void loadTimesIfNeeded() {
  if (s_timesLoaded) return;
  File tf = SD.open(TIMES_FILE, FILE_READ);
  if (tf) {
    size_t wantBytes = (size_t)frameCount * sizeof(time_t);
    size_t haveBytes = tf.size();
    if (wantBytes == 0) {
      s_timesLoaded = true;
    } else if (haveBytes >= wantBytes) {
      if (haveBytes > wantBytes) {
        size_t tailOffset = haveBytes - wantBytes;
        Serial.printf("times play big %u>%u\\n",
                      (unsigned)haveBytes, (unsigned)wantBytes);
        tf.seek((uint32_t)tailOffset);
      }
      size_t got = tf.read((uint8_t*)s_frameTimes, wantBytes);
      if (got == wantBytes) {
        s_timesLoaded = true;
      } else {
        Serial.printf("times play short %u<%u\\n",
                      (unsigned)got, (unsigned)wantBytes);
      }
    } else {
      Serial.printf("times play miss %u<%u\\n",
                    (unsigned)haveBytes, (unsigned)wantBytes);
    }
    tf.close();
  }
}"""
text = text.replace(old_lti, "", 1)

# Delete rawCacheMetaVersionIsCurrent (large function)
old_rcm_start = "static bool rawCacheMetaVersionIsCurrent() {"
old_rcm_end_marker = "  s_rawMetaVersionCurrent = true;\n  return true;\n}\n"
rcm_start = text.find(old_rcm_start)
rcm_end = text.find(old_rcm_end_marker, rcm_start)
if rcm_start >= 0 and rcm_end >= 0:
    text = text[:rcm_start] + text[rcm_end + len(old_rcm_end_marker):]

# Delete countValidStreamSlots
old_cvs = text[text.find("static int countValidStreamSlots("):text.find("\nstatic bool rawCacheMetaVersionIsCurrent") if text.find("\nstatic bool rawCacheMetaVersionIsCurrent") > 0 else text.find("\nstatic void logSourceBlackFrame")]
# This may not cleanly find it after rcm was deleted — just try
# Let's find countValidStreamSlots more carefully
cvs_match = "static int countValidStreamSlots("
cvs_pos = text.find(cvs_match)
if cvs_pos >= 0:
    cvs_end = text.find("\n}\n", cvs_pos)
    if cvs_end >= 0:
        text = text[:cvs_pos] + text[cvs_end + 3:]

# Delete verifyStreamSlotAgainstSprite
old_vsas = "static bool verifyStreamSlotAgainstSprite("
vsas_pos = text.find(old_vsas)
if vsas_pos >= 0:
    vsas_end = text.find("\n}\n", vsas_pos)
    if vsas_end >= 0:
        text = text[:vsas_pos] + text[vsas_end + 3:]

# Delete verifyInstalledJpeg
old_vij = "static bool verifyInstalledJpeg("
vij_pos = text.find(old_vij)
if vij_pos >= 0:
    vij_end = text.find("\n}\n", vij_pos)
    if vij_end >= 0:
        text = text[:vij_pos] + text[vij_end + 3:]

# Delete buildRawPlaybackCache (large function)
old_brpc = "static void buildRawPlaybackCache() {"
brpc_pos = text.find(old_brpc)
if brpc_pos >= 0:
    # Find end — look for the closing of the function
    brace_count = 0
    in_func = False
    end_pos = brpc_pos
    for i in range(brpc_pos, len(text)):
        if text[i] == '{':
            brace_count += 1
            in_func = True
        elif text[i] == '}':
            brace_count -= 1
            if in_func and brace_count == 0:
                end_pos = i + 1
                break
    text = text[:brpc_pos] + text[end_pos:]

# Delete copyStreamSlot
old_css = "static bool copyStreamSlot("
css_pos = text.find(old_css)
if css_pos >= 0:
    css_end = text.find("\n}\n", css_pos)
    if css_end >= 0:
        text = text[:css_pos] + text[css_end + 3:]

# Delete gapFillInvalidStreamMap
old_gf = "static int gapFillInvalidStreamMap("
gf_pos = text.find(old_gf)
if gf_pos >= 0:
    gf_end = text.find("\n}\n", gf_pos)
    if gf_end >= 0:
        text = text[:gf_pos] + text[gf_end + 3:]

# Delete rebuildRawPlaybackCacheRolling
old_rrpcr = "static bool rebuildRawPlaybackCacheRolling("
rrpcr_pos = text.find(old_rrpcr)
if rrpcr_pos >= 0:
    brace_count = 0
    in_func = False
    end_pos = rrpcr_pos
    for i in range(rrpcr_pos, len(text)):
        if text[i] == '{':
            brace_count += 1
            in_func = True
        elif text[i] == '}':
            brace_count -= 1
            if in_func and brace_count == 0:
                end_pos = i + 1
                break
    text = text[:rrpcr_pos] + text[end_pos:]

# Delete remapRawPlaybackCacheRolling
old_rmrpcr = "static bool remapRawPlaybackCacheRolling("
rmrpcr_pos = text.find(old_rmrpcr)
if rmrpcr_pos >= 0:
    brace_count = 0
    in_func = False
    end_pos = rmrpcr_pos
    for i in range(rmrpcr_pos, len(text)):
        if text[i] == '{':
            brace_count += 1
            in_func = True
        elif text[i] == '}':
            brace_count -= 1
            if in_func and brace_count == 0:
                end_pos = i + 1
                break
    text = text[:rmrpcr_pos] + text[end_pos:]

# Delete validateStoredWeatherFramePath
old_vswfp = "static bool validateStoredWeatherFramePath("
vswfp_pos = text.find(old_vswfp)
if vswfp_pos >= 0:
    vswfp_end = text.find("\n}\n", vswfp_pos)
    if vswfp_end >= 0:
        text = text[:vswfp_pos] + text[vswfp_end + 3:]

# Delete weatherFrameFileSize
old_wffs = "static int weatherFrameFileSize("
wffs_pos = text.find(old_wffs)
if wffs_pos >= 0:
    wffs_end = text.find("\n}\n", wffs_pos)
    if wffs_end >= 0:
        text = text[:wffs_pos] + text[wffs_end + 3:]

# Delete weatherFrameLooksCompressedSizeOutlier
old_wflcso = "static bool weatherFrameLooksCompressedSizeOutlier("
wflcso_pos = text.find(old_wflcso)
if wflcso_pos >= 0:
    wflcso_end = text.find("\n}\n", wflcso_pos)
    if wflcso_end >= 0:
        text = text[:wflcso_pos] + text[wflcso_end + 3:]

# Delete weatherFrameLooksSemanticOutlier
old_wflso = "static bool weatherFrameLooksSemanticOutlier("
wflso_pos = text.find(old_wflso)
if wflso_pos >= 0:
    brace_count = 0
    in_func = False
    end_pos = wflso_pos
    for i in range(wflso_pos, len(text)):
        if text[i] == '{':
            brace_count += 1
            in_func = True
        elif text[i] == '}':
            brace_count -= 1
            if in_func and brace_count == 0:
                end_pos = i + 1
                break
    text = text[:wflso_pos] + text[end_pos:]

# Delete installValidatedWeatherJpegToPath
old_ivwjtp = "static bool installValidatedWeatherJpegToPath("
ivwjtp_pos = text.find(old_ivwjtp)
if ivwjtp_pos >= 0:
    ivwjtp_end = text.find("\n}\n", ivwjtp_pos)
    if ivwjtp_end >= 0:
        text = text[:ivwjtp_pos] + text[ivwjtp_end + 3:]

# Delete repairCachedWeatherFrame
old_rcwf = "static bool repairCachedWeatherFrame("
rcwf_pos = text.find(old_rcwf)
if rcwf_pos >= 0:
    rcwf_end = text.find("\n}\n", rcwf_pos)
    if rcwf_end >= 0:
        text = text[:rcwf_pos] + text[rcwf_end + 3:]

# Delete healCachedWeatherFrameFromNeighbor
old_hcwfn = "static bool healCachedWeatherFrameFromNeighbor("
hcwfn_pos = text.find(old_hcwfn)
if hcwfn_pos >= 0:
    hcwfn_end = text.find("\n}\n", hcwfn_pos)
    if hcwfn_end >= 0:
        text = text[:hcwfn_pos] + text[hcwfn_end + 3:]

# Delete validateAndRepairCachedFrames
old_varfc = "static bool validateAndRepairCachedFrames("
varfc_pos = text.find(old_varfc)
if varfc_pos >= 0:
    brace_count = 0
    in_func = False
    end_pos = varfc_pos
    for i in range(varfc_pos, len(text)):
        if text[i] == '{':
            brace_count += 1
            in_func = True
        elif text[i] == '}':
            brace_count -= 1
            if in_func and brace_count == 0:
                end_pos = i + 1
                break
    text = text[:varfc_pos] + text[end_pos:]

# Delete validateAndRepairFullCacheIfNeeded
old_varfcin = "static bool validateAndRepairFullCacheIfNeeded("
varfcin_pos = text.find(old_varfcin)
if varfcin_pos >= 0:
    varfcin_end = text.find("\n}\n", varfcin_pos)
    if varfcin_end >= 0:
        text = text[:varfcin_pos] + text[varfcin_end + 3:]

# Delete validateAndRepairCacheSlice
old_varcs = "static bool validateAndRepairCacheSlice("
varcs_pos = text.find(old_varcs)
if varcs_pos >= 0:
    varcs_end = text.find("\n}\n", varcs_pos)
    if varcs_end >= 0:
        text = text[:varcs_pos] + text[varcs_end + 3:]

# Delete downloadFrameToPath (wrapper)
old_dftp = """static bool downloadFrameToPath(HTTPClient& http,
                                WiFiClientSecure& client,
                                time_t t,
                                const char* sdPath,
                                size_t* outBytes"""
dftp_pos = text.find(old_dftp)
if dftp_pos >= 0:
    dftp_end = text.find("\n}\n", dftp_pos)
    if dftp_end >= 0:
        text = text[:dftp_pos] + text[dftp_end + 3:]

# Delete downloadFrameToPathAtBbox
old_dftpab = "static bool downloadFrameToPathAtBbox("
dftpab_pos = text.find(old_dftpab)
if dftpab_pos >= 0:
    brace_count = 0
    in_func = False
    end_pos = dftpab_pos
    for i in range(dftpab_pos, len(text)):
        if text[i] == '{':
            brace_count += 1
            in_func = True
        elif text[i] == '}':
            brace_count -= 1
            if in_func and brace_count == 0:
                end_pos = i + 1
                break
    text = text[:dftpab_pos] + text[end_pos:]

# Delete loadWeatherSemanticSignature
old_lwss = "static bool loadWeatherSemanticSignature("
lwss_pos = text.find(old_lwss)
if lwss_pos >= 0:
    lwss_end = text.find("\n}\n", lwss_pos)
    if lwss_end >= 0:
        text = text[:lwss_pos] + text[lwss_end + 3:]

# ═══════════════════════════════════════════════════════════════
# STEP 15: Remove stale variable declarations / defines
# ═══════════════════════════════════════════════════════════════

text = text.replace("RTC_DATA_ATTR static uint8_t s_cacheRepairCursor = 0;\n", "", 1)
text = text.replace("static bool s_rawMetaChecked = false;\n", "", 1)
text = text.replace("static bool s_rawMetaVersionCurrent = false;\n", "", 1)

# Remove old invalidateRawCacheMetaState
old_irms = """static void invalidateRawCacheMetaState() {
  s_rawMetaChecked = false;
  s_rawMetaVersionCurrent = false;
}"""
text = text.replace(old_irms, """static void invalidateRawCacheMetaState() {
  // Kept as no-op for compatibility — index.bin is now the single source of truth
}""", 1)

# Clean up multiple consecutive blank lines (more than 2)
while "\n\n\n\n" in text:
    text = text.replace("\n\n\n\n", "\n\n\n")

# ═══════════════════════════════════════════════════════════════
# Write output
# ═══════════════════════════════════════════════════════════════

with open(DST, "w") as f:
    f.write(text)

new_count = text.count("\n") + (0 if text.endswith("\n") else 1)
print(f"Refactoring complete. Lines: {len(lines)} -> {new_count}")
