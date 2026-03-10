# Playback & Overlay Systems

## Weather Animation (10 seconds)

### Frame Selection
- Valid frames stored in `s_validIdx[]` array, count in `s_validCount`
- Time-based position mapping (NOT slot-sequential):
  ```
  srcPos = elapsed * validCount / animationDurationMs
  ```
- Prevents timeline truncation — all frames shown even if decode is slow
- Wall-clock deadlines: break when `millis() >= animDeadline`, not after N iterations
- Target: ~30fps with `FRAME_DELAY_MS = 33`

### showFrame(idx)
- Seeks to `slot * SCALED_FRAME_BYTES` in `s_streamFile` (stream.raw)
- Reads 295,200 bytes into `s_frameDisplayBuf`
- Validates read length; short read → invalidate slot + raw meta

### presentSpriteToDisplay()
- Overlays `s_topBarBuf` and `s_botBarBuf` onto `s_frameDisplayBuf`
- Writes to CO5300 AMOLED via `s_amoledOut->draw16bitRGBBitmap()`
- Dirty-rect optimization: when `s_dirtyRectDstW > 0`, scales only clock sub-rect

## Top/Bottom Bars
- Top bar: WiFi glyph + SSID, battery icon + %, date/time + "Xh ago" / "Xm ago"
- Bottom bar: location line, radar status
- Rendered into PSRAM buffers (`s_topBarBuf`, `s_botBarBuf`)
- Height: `SCALED_BAR_H` ≈ 31 pixels at 410-wide
- Stamped over every presented frame

## Freeze Frame (2 seconds)
- Holds last animation frame
- `currentScaledFreezeFrameLooksCorrupted()` fires:
  - If corrupt: walks backward to clean frame (freeze-back)
  - Evicts corrupt source JPEGs + invalidates raw meta
- `runFreezeZoom3LocatorCue()`: flashing red rectangle outline showing ZOOM3 crop area
  - `computeZoom3LocatorRectScaled()` computes the rect position
  - 250ms on/250ms off blink pattern

## Zoom Stages (3 × ~1 second each)

### Zoom Level Computation
- Geographic bounding boxes computed via `computeBboxFromCenterKm()`
- Geometric √ interpolation between zoom levels
- `ZOOM3_FINAL_W_KM = 250.0f`, `ZOOM3_FINAL_H_KM = 135.0f`

### Zoom Files
| Stage | JPEG | Raw | Purpose |
|-------|------|-----|---------|
| ZOOM1 | vz1.jpg | vz1.raw | Wide regional view |
| ZOOM2 | vz2.jpg | vz2.raw | Medium zoom |
| ZOOM3 | vz3.jpg | vz3.raw | Close zoom (250×135 km) |

### Supersample Strategy
- Fetch at 2x resolution (`ZOOM_FETCH_W = 640`, `ZOOM_FETCH_H = 344`)
- Decode back to 320×176 sprite → scale to 410×360
- If 2x fails at deeper zoom levels → fallback to 1x (never abort stage)

### showZoomSnapshotFrame()
- Reads from pre-scaled `.raw` file or decodes `.jpg` on the fly
- Overlays timestamp bars
- Each stage held for ~1 second

## Terrain Crossfade (~1 second)

### Terrain Sources
- **BlueMarble day**: NASA GIBS `BlueMarble_NextGeneration`
- **VIIRS night**: `VIIRS_Black_Marble` (fixed 2016-01-01 date)
- Day/night selection: `terrainUsesNightLayerForUtc()` via USNO sunrise/sunset math
- Terrain fetch uses `TERRAIN_FETCH_H = DISP_H * 2 = 352` (NOT zoom fetch height!)

### NEXRAD Radar Overlay
- Source: `mapservices.weather.noaa.gov/eventdriven`
- `fetchRadarLatestTimeMs()`: gets latest radar scan timestamp
- `countRadarSignalPixelsInSprite()`: saturation-based precipitation detection
- `blendRadarIntoTerrainRaw()`: 3×3 neighborhood despeckle then alpha blend
- `s_radarNoSignatures`: download succeeded but no precipitation ("Clear")
- `s_radarDownloadFailed`: download failed — outside coverage ("no sig")

### Crossfade Execution
- `runTerrainCrossfadeSegment()`: 18-step smoothstep wipe from ZOOM3 to terrain
- `s_terrainDisplayBuf` pre-loaded from SD before wipe starts
- Top bar switches to radar scan time during terrain stage (`s_topBarUseRadarScanTime`)

### Terrain Pair Tolerance
- If one layer (day or night) fails, use the other. Copy successful side into missing slot.
- Only fail when neither side available.

## Clock Overlay (7 seconds: 2s in + 3s hold + 2s out)

### Layout
- `ClockOverlayLayout`: computed once per overlay segment
- Font: DejaVu56 (56pt) in `s_clockFxSprite`
- Sprite created at computed text bounds + padding

### Sweep Animation
- `runCurrentTimeSweepOverlaySegment()`:
  - 2s sweep-in: directional fade from edge to center
  - 3s hold: full opacity
  - 2s sweep-out: directional fade from center to edge
- `directionalFadeAlphaForColumn()`: wide-feather directional alpha function
- `blend565()`: per-pixel alpha blend RGB565
- `smoothstep8()`: 8-bit smoothstep easing

### Background Save/Restore
- `saveSpriteRegionToDlBuf()`: saves terrain background under clock text area
- `restoreSpriteRegionFromDlBuf()`: restores background after overlay segment
- Uses `s_dlBuf` (128 KB download buffer) as temp storage since download is not active

### Location Pin
- `drawApproxLocationPinOnClockFxSprite()`: red teardrop at screen center
- Drawn on clock overlay sprite alongside time text

### Dirty-Rect Optimization
- Only the clock sub-rect is re-scaled and pushed to display each tick
- `s_dirtyRectSrcX/Y/W/H` → `s_dirtyRectDstX/Y/W/H`
- Significantly reduces per-frame scaling work during overlay

## Sleep Transition
- After `LOOPS_BEFORE_SLEEP` complete iterations: `goToSleep(false)`
- Auto-update boundary check can trigger `ESP.restart()` instead of sleep
- Button short-press triggers `goToSleep(true)` (immediate sleep)
