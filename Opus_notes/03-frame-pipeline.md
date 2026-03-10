# Frame Pipeline — Download → Validate → Cache → Raw Build → Playback

## Pipeline Overview
```
NASA GIBS WMS API → HTTPS download → RAM buffer (s_dlBuf, 128KB)
    ↓
Transport validation (SOI/EOI, size, effective length)
    ↓
Decode viability (JPEGDEC, geometry check)
    ↓
Structural pixel validation (10+ detectors)
    ↓
Atomic install (*.part → verify → re-decode → rename to fNNN.jpg)
    ↓
times.bin (timestamps), meta.txt (frame count), dim.cfg (geometry)
    ↓
Raw cache build: decode fNNN.jpg → scale 410×360 → stream.raw slot
    (semantic + size outlier checks at admission)
    ↓
Gap fill: nearest-neighbor copy for invalid slots
    ↓
Playback: stream.raw → s_frameDisplayBuf → AMOLED
```

## Source: NASA GIBS WMS
- Base URL: `https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi`
- No API key required
- Layers: GOES-East_ABI_GeoColor, GOES-West_ABI_GeoColor, Himawari_AHI_Band13_Clean_Infrared
- Request: WMS GetMap, EPSG:4326, JPEG format, 320×176 pixels
- Time parameter: ISO 8601 UTC timestamp

## Temporal Window
- `targetFrameCount()` = HOURS_BACK * 60 / cadence = 24 * 60 / 10 = 144 frames max
- `fetchEnd = roundToCadence(now - lagHours)` (lag = 2h default)
- `fetchStart = fetchEnd - (targetFrameCount - 1) * cadence`
- Clamped to `MAX_FRAMES = 144`

## Download Validation Stack (in order)

### Gate 1: Transport/Body Integrity
- Function: `readHttpJpegBodyToDlBuf()`
- Buffer: `s_dlBuf` (128 KB, PSRAM)
- Checks: HTTP body read success, size within bounds, starts with FFD8 (SOI), contains FFD9 (EOI)
- `jpegEffectiveLength()`: finds last FFD9, trims trailing noise bytes
- Output: canonical JPEG effective length

### Gate 2: Decode Viability
- Functions: `validateBufferedWeatherFrameJpeg()` / `decodeJpegPathToSprite()`
- JPEGDEC `jpeg.openRAM()` or `jpeg.openFILE()` + decode
- Expected dimensions: DISP_W × DISP_H (320×176)
- `jpegDrawLooksFullFrame()`: all expected MCU blocks drawn

### Gate 3: Structural Pixel Detectors
See [04-corruption-detectors.md](04-corruption-detectors.md) for full catalog.

### Gate 4: Atomic Install
- Function: `installValidatedWeatherJpegToPath()`
- Flow: write to `*.part` → verify head/tail bytes → re-decode from disk → re-run validation → atomic rename
- Failure: temp removed, final frame unchanged

### Gate 5: Temporal-Neighbor Outliers (at raw-build time)
- `weatherFrameLooksCompressedSizeOutlier()`: JPEG file size vs neighbors
- `weatherFrameLooksSemanticOutlier()`: per-tile color signatures vs neighbors

## SD File Layout (`/frames/`)
| File | Purpose | Size per entry |
|------|---------|----------------|
| `fNNN.jpg` | Final validated JPEG frames (N=000..143) | ~15-30 KB |
| `nNNN.jpg` | Temp frames during rolling update | ~15-30 KB |
| `meta.txt` | Frame count (single integer) | tiny |
| `times.bin` | `time_t[]` array of frame UTC timestamps | 4 bytes × N |
| `stream.raw` | Contiguous pre-scaled RGB565 playback stream | 295,200 bytes/slot |
| `raw.meta` | Version + validity bitmap + slot map | ~300 bytes |
| `dim.cfg` | Frame dimensions "320 176" | tiny |
| `view.meta` | Bbox/layer/cadence signature | tiny |
| `validate.meta` | Cache integrity marker | tiny |
| `zoom.meta` | Zoom snapshot freshness | tiny |
| `radar.meta` | Last radar UTC | tiny |

## Raw Cache (stream.raw)
- Slot size: 410 × 360 × 2 = 295,200 bytes (pre-scaled RGB565)
- Total for 144 frames: ~42.5 MB
- Build paths:
  1. `buildRawPlaybackCache()`: full rebuild — decode every JPEG
  2. `rebuildRawPlaybackCacheRolling()`: copy unchanged slots + rebuild new ones (temp file → rename)
  3. `remapRawPlaybackCacheRolling()`: in-place reuse via `s_streamSlotMap[]` indirection
- All paths run validation: slab → partial → hold-block → cyan-block → botband → size outlier → semantic outlier
- Failed slots zeroed → gap-filled by nearest neighbor

## raw.meta Format
- Byte 0: version (RAW_CACHE_VERSION = 34)
- Bytes 1-144: `s_streamValid[]` — 1=valid, 0=invalid per slot
- Bytes 145-288: `s_streamSlotMap[]` — logical→physical slot mapping
- Runtime guard: if meta=1 but zero valid slots (vbm=0) → force invalidate + rebuild

## Gap Fill
- `gapFillInvalidStreamSlotsInFile()`: copies nearest valid neighbor's raw data into zero slots
- `gapFillInvalidStreamMap()`: updates validity map for gap-filled slots
- Feature, not error — animation plays smoothly with neighbor duplication

## Download Retry Strategy
- `downloadFrames()`: 7 time offsets per slot: {0, -60, 60, -120, 120, -180, 180} seconds (±3 min)
- Rolling sync `downloadFrameToPath()`: {0, -60, 60, -cadence, +cadence, -2*cadence, +2*cadence}
- Min JPEG size floor: 7000 bytes (below = placeholder rejection)

## Cache Repair
- `validateAndRepairCachedFrames()` / `validateAndRepairFullCacheIfNeeded()` / `validateAndRepairCacheSlice()`
- Strategy: re-download exact timestamp → if fail, heal from nearest good neighbor
- `CACHE_VALIDATE_META_FILE` tracks validated cache signature
- Failed full pass → validation meta removed → retries next boot

## Playback
- `showFrame(idx)`: seeks to `slot * SCALED_FRAME_BYTES` in stream.raw, reads into `s_frameDisplayBuf`
- Only `s_streamValid[]` slots played; invalid slots skipped
- `s_validIdx[]` + `s_validCount` = flattened list of playable frame indices
- 0 valid frames → explicit error message, no playback
