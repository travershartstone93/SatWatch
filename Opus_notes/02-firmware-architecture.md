# Firmware Architecture Overview

## Runtime Model
Two-phase state machine:
1. **Sync/prepare phase** — WiFi, SD, cache update/rebuild (in `setup()`)
2. **Playback phase** — weather animation + zoom/terrain/clock sequence (in `loop()`)

## setup() Sequence
1. Serial, Wire (I2C), AXP2101 PMIC init, display init (board-specific)
2. SD mount (SD_MMC for AMOLED)
3. PSRAM allocation: `s_frameDisplayBuf`, `s_terrainDisplayBuf`, `s_topBarBuf`, `s_botBarBuf`
4. Portal config load from NVS (WiFi slots, 12/24h, update mode, start cue, sleep mode)
5. Button poll task init (FreeRTOS, core 1)
6. Audio cue preload to PSRAM
7. Boot type detection: hard boot / timer wake / GPIO wake
8. SD cache check: `loadIndex()`, dimension check via `dim.cfg`
9. **Sync decision**: `needSync = hardBootSyncDue || timerWake || !framesReady || autoUpdateDue`
10. If needSync: WiFi connect → `syncWeatherFrames()` → zoom/terrain refresh
11. If no sync but hard boot: WiFi → NTP → IP geolocation → zoom refresh if needed
12. Raw cache stale check → `rebuildRawFromStored()` if needed
13. `ensureStreamOpen()`, rebuild if needed
14. Restore brightness, arm start cue, enter `loop()`

## loop() Sequence (~24s per iteration)
1. `serviceWifiPortalServer()`, `serviceUserButtons()`
2. `ensureStreamOpen()` — open `stream.raw` if not open
3. **10s weather animation**: time-based frame selection from `s_validIdx[]`, `showFrame()`
   - Uses monotonic time mapping: `srcPos = elapsed * validCount / animationDurationMs`
   - NOT slot-sequential (avoids timeline truncation)
4. **2s freeze** on last animation frame + `runFreezeZoom3LocatorCue()` (flashing red rect)
   - Freeze-back corruption detection fires here
5. **3× zoom snapshots** (ZOOM1→ZOOM2→ZOOM3), ~1s each via `showZoomSnapshotFrame()`
6. **Terrain crossfade** ~1s (`runTerrainCrossfadeSegment()`) — 18-step smoothstep wipe
7. **7s clock overlay** (`runCurrentTimeSweepOverlaySegment()`) — 2s sweep in, 3s hold, 2s sweep out
8. Auto-update boundary check → may trigger reboot/rescan
9. After `LOOPS_BEFORE_SLEEP` loops: `goToSleep(false)`

## Key State Variables (RTC_DATA_ATTR — survive deep sleep)
- `framesReady`: bool — SD cache has playable frames
- `loopsDone`: int — loop iterations since last boot
- `frameCount`: int — number of cached frames
- `s_displayUtcOffsetSec`: timezone offset (also persisted in NVS key "utcoff")
- `s_weatherCenterLat/Lon`: location center
- `s_activeGibsLayer[48]`: current GIBS layer name
- `s_activeWeatherSource[20]`: human-readable source ("GOES-East")
- `s_activeCadenceMin`, `s_activeLagHours`: active source timing
- `s_sleepModeEnabled`, `s_autoUpdateInSleep`: sleep policy

## Key Runtime State (regular RAM, lost on any sleep)
- `s_idx` (`FrameStoreIndex`): ring-buffer index for frames.bin (magic, head, count, times, jpegLen, validity)
- `s_frameTimes[MAX_FRAMES]`: per-frame UTC timestamps (populated from `s_idx.times[]`)
- `s_streamValid[MAX_FRAMES]`: validity bitmap for stream.raw slots (from `s_idx.rawValid[]`)
- `s_validIdx[MAX_FRAMES]`: indices of valid frames for playback
- `s_validCount`: count of valid frames
- `s_streamFile`: open File handle for stream.raw during playback
- `s_batPct`, `s_batChargeState`: battery state from AXP2101
- `s_wifiRssi`: last measured WiFi RSSI (negative dBm)

## Display Pipeline
```
JPEG (320×176) → decode to sprite → structural validation
    ↓
scaleSpriteTo410x360() → s_frameDisplayBuf (PSRAM, 410×360 RGB565)
    ↓
presentSpriteToDisplay() → CO5300 QSPI AMOLED
    (overlays s_topBarBuf/s_botBarBuf before present)
```

## Satellite Source Selection
`selectSatelliteForLon(lonDeg)`:
- lon >= -110°: GOES-East (10 min cadence, 2h lag)
- lon < -110°: GOES-West (10 min cadence, 2h lag)
- lon >= 60°: Himawari-IR (varies)
- 2° hysteresis prevents flip-flopping at boundaries

## Stack Size
`getArduinoLoopTaskStackSize()` returns 32768 (32 KB) because:
- SSL handshake + lwIP TCP ACK = ~35 frames deep
- `blendRadarIntoTerrainRaw()` allocates ~7 KB local arrays
