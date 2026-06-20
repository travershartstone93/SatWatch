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
Write to frames.bin slot + update index.bin (ring buffer, timestamps, validity)
    ↓
dim.cfg (geometry), view.meta (bbox/layer signature)
    ↓
Raw cache build: decode JPEG from frames.bin → scale 410×360 → stream.raw slot
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
| File | Purpose | Size |
|------|---------|------|
| `frames.bin` | Contiguous JPEG store (64KB slots × 144) | ~9 MB |
| `index.bin` | FrameStoreIndex struct (ring head, timestamps, validity) | ~2.5 KB |
| `stream.raw` | Pre-scaled RGB565 playback stream (295,200 bytes/slot) | ~42.5 MB |
| `dim.cfg` | Frame dimensions "320 176" | tiny |
| `view.meta` | Bbox/layer/cadence signature | tiny |
| `validate.meta` | Cache integrity marker | tiny |
| `zoom.meta` | Zoom snapshot freshness | tiny |
| `radar.meta` | Last radar UTC | tiny |

## Raw Cache (stream.raw)
- Slot size: 410 × 360 × 2 = 295,200 bytes (pre-scaled RGB565)
- Total for 144 frames: ~42.5 MB
- Build path: `rebuildRawFromStored()` — iterates index, decodes each valid JPEG from frames.bin into stream.raw
- Validation gates: slab → partial → hold-block → cyan-block → botband → topband-white → size outlier → semantic outlier
- Failed slots zeroed → gap-filled by nearest neighbor

## index.bin Format (`FrameStoreIndex` struct)
- `magic` (uint32): `INDEX_MAGIC` = 0x4C534658 ("LSFX")
- `head` (uint16): ring buffer head position
- `count` (uint16): number of slots in use
- `times[144]` (time_t): UNIX timestamp per frame
- `jpegLen[144]` (uint32): JPEG byte length per slot in frames.bin
- `jpegValid[144]` (uint8): 1=valid JPEG, 0=invalid
- `rawValid[144]` (uint8): 1=valid decoded raw, 0=needs rebuild
- On load: SOI-byte integrity check on each jpegValid slot; clears both jpegValid+rawValid on fail
- `s_streamValid[]` populated from `s_idx.rawValid[]` at load time

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
