# Critical Pitfalls — Distilled "Don't Do This" List

Source: `The-fuckup-sofar.txt` + observed fix history. Organized by severity.

## HARDWARE

### GPIO21 Shared Pin (SHOW-STOPPER if missed)
GPIO21 = AMOLED_PWR_EN = QMI8658_INT1. Two push-pull drivers. Display holds pin HIGH.
IMU cannot toggle it. ALL shake-to-wake failures trace here.
- Fix: `gpio_reset_pin()` + `INPUT_PULLUP` before sleep, `OUTPUT HIGH` after wake
- DO NOT adjust threshold/polarity/defaultPinValue — that's not the problem

### Wrong FQBN = Silent Bootloader Hang
Generic `esp32:esp32:esp32s3` assumes 4 MB flash on a 32 MB chip → bootloader hangs with no output.
Must use: `esp32:esp32:waveshare_esp32_s3_touch_amoled_206:PSRAM=enabled,PartitionScheme=max_app_32MB`

### SD is SDMMC, Not SPI
`SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_*)`. NOT `SD.begin(CS_PIN, SPI, freq)`.

### SD Path Double-Prepend
VFS prepends "/sdcard". Paths must NOT include it. `#define SD_ROOT ""`, paths start at card root.

### DISP_W/H Must Be Multiples of 16
JPEGDEC MCU blocks are 16×16. Non-multiples cause stride mismatch → horizontal smearing.

## MEMORY

### WiFiClientSecure Needs ~35-40 KB Contiguous Internal DRAM
Large static buffers in BSS → SSL handshake fails silently ("connection refused").
Rule: anything > few KB → `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)`.

### LovyanGFX uint32_t Color Routing
`uint32_t` color values route through RGB888 path. RGB565 constants like `TFT_GREEN = 0x07E0` become garbled.
Always use `uint16_t` or `int` for RGB565 colors. NEVER `uint32_t`.

### PSRAM: OK for Sequential Reads, NOT for DMA
Display reads from PSRAM fine. I2S/DMA peripherals need `MALLOC_CAP_DMA` (internal DRAM only).

## FRAME PIPELINE

### Meta Invalidation in buildRawPlaybackCache() = Boot Loop
DO NOT add `SD.remove(RAW_CACHE_META_FILE)` on decFail in `buildRawPlaybackCache()`.
decFail includes missing-file failures (not just bad JPEG). 59 missing files → decFail=59 → meta invalidated every boot → infinite loop.
Safe eviction: `SD.remove(fNNN.jpg)` for corrupt source files. Rolling sync redownloads.
Meta invalidation is ONLY safe in `remapRawPlaybackCacheRolling()` and `rebuildRawPlaybackCacheRolling()`.

### GIBS Lag Boundary = Most Likely Corrupt Frame
Newest frame slot often has incomplete GIBS composite (black rectangles). Passes SOI/EOI checks.
Handled by: black slab detector + retry offsets + freeze-back eviction.

### Structural JPEG Validity ≠ Weather Content Validity
`cachedFrameLooksReusable()` only checks: exists, size >= 7000, SOI, EOI. Does NOT decode.
Frames can still be: black, cyan, horizontally banded, semantically wrong.
Every validation layer is necessary.

### Both Validation Functions Must Stay In Sync
`validateBufferedWeatherFrameJpeg()` (download) and `validateStoredWeatherFramePath()` (stored) must wire the SAME detectors. Missing one side = corruption enters through the other gate.

### BotBand Threshold Was Wrong for Years
Original: >= 3 transitions. Real corruption: exactly 2 transitions. Off-by-one in threshold = months of bad frames. Current: >= 2. Verify thresholds against real data, not logic.

### Black Slab Playback Thresholds False-Positive on Dark Ocean
`scaledFrameLooksBlackSlabCorrupted()` playback version: maxCh <= 5, spread <= 2.
Original maxCh <= 8 triggered on real dark Pacific Ocean pixels. Caused animation to jump back 5 hours every loop.

### dim.cfg Must Be Written After Every Cache Install
Missing dim.cfg → dimension mismatch → entire cache purged. `writeCurrentFrameDimMeta()` required.

### Rolling Sync Only Redownloads MISSING Files
`cachedFrameLooksReusable()` returns true for present-but-corrupt files. To force redownload: `SD.remove()` the corrupt file.

### Remap Copy Pass Can Reuse Old Gap-Fills
`remapRawPlaybackCacheRolling()` Pass 1 copies old stream slots. If source file was deleted then redownloaded, Pass 1 reuses the OLD gap-fill data. The `!frameFileExists(i, 'f')` check forces those slots to Pass 2 (fresh decode). Do not remove this check.

### commitTempFrames() Is Non-Transactional
Power loss between delete-all-f and rename-all-n = partially destroyed cache. Known structural risk.

## SLEEP & WAKE

### WoM Interrupt Is a Toggle, Not a Level
INT1 TOGGLES on each event. ESP32 light sleep only supports level-triggered GPIO wakeup.
Strategy: read current level, arm OPPOSITE level. Next toggle wakes.

### configMotion()+enableMotionDetect() Doesn't Work
Pulse too brief for level-triggered light sleep. Must use `configWakeOnMotion()`.

## WIFI & PORTAL

### s_wifiSyncInProgress Is False During Playback
Don't gate WiFi status display on it. Use `s_wifiRssi < 0` instead.

### disconnectWifiAfterSync() Must Use startWifiPortalServer(false)
`true` = AP+STA dual mode = AP stays broadcasting after sync. `false` = STA-only portal.

## MISC

### Semantic Outlier Check Must Run at Raw-Build Time
Not just at download time. Cached JPEGs can be structurally valid but wrong content.

### Full-Cache Repair Must Run on Fast-Path Boots
latest-current and exact-current paths skip redownload but must still call `validateAndRepairFullCacheIfNeeded()`.

### "vld: MISS BLOCK-CORR" Appears Every Boot
Unknown origin, predates all fixes. Not blocking anything. Don't chase it for unrelated symptoms.
