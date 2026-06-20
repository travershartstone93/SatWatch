# Corruption Detectors — Full Catalog

Every detector, its purpose, thresholds, and where it's wired.

## Sprite-Level Detectors (operate on decoded 320×176 JPEG sprite)

### 1. spriteLooksCompletelyBlack()
- **Detects**: exact-zero blank frames (all pixels == 0x0000)
- **Wired in**: download validation, raw-build admission

### 2. spriteLooksNearlyBlack()
- **Detects**: visually-black frames tolerant to JPEG compression noise
- **Wired in**: download validation, raw-build admission

### 3. spriteLooksPartialDecode()
- **Detects**: large exact-zero ratio + quadrant concentration (incomplete decode footprint)
- **Pattern**: top portion has content, bottom is all zeros
- **Wired in**: download validation, raw-build admission

### 4. spriteLooksHorizontallyCorrupted()
- **Detects**: horizontal band half-empty pattern (partial HTTP read signature)
- **Pattern**: alternating bands of content and zeros across rows
- **Wired in**: download validation, raw-build admission

### 5. spriteLooksVerticallyCorrupted()
- **Detects**: left/right half-empty split pattern
- **Wired in**: download validation, raw-build admission

### 6. spriteLooksHoldFrameBlockCorrupted()
- **Detects**: 16×16 MCU-hole/band patterns (isolated zero blocks + row-band patterns)
- **Has 3 sub-detectors**: single block holes, row bands, column bands
- **Wired in**: download validation, raw-build admission

### 7. spriteLooksCyanWhiteBlockCorrupted()
- **Detects**: cyan/white tiled artifact — known GIBS corruption mode
- **Pattern**: tiles with cyan (0,255,255) or white (255,255,255) fill
- **Wired in**: download validation, raw-build admission

### 8. spriteLooksBlackSlabCorrupted()
- **Detects**: dark slab clusters — partially-composited GIBS frames with missing tile regions
- **Pattern**: black rectangles in corners/edges of otherwise valid weather image
- **Method**: connected-component BFS for dark tile clusters
- **Thresholds (download)**: calibrated separately from playback version
- **Wired in**: download validation (`validateBufferedWeatherFrameJpeg`, `validateStoredWeatherFramePath`), raw-build admission

### 9. spriteLooksBottomBandJunkCorrupted()
- **Detects**: junk-pixel band at bottom of frame
- **Pattern**: dark/light MCU-row transitions in bottom third
- **Threshold**: >= 2 transitions (was >= 3, which missed real corruption for years!)
- **Location in source**: ~line 8951
- **Wired in**: download validation, raw-build admission

## Scaled-Frame Level Detectors (operate on 410×360 stream.raw pixels)

### 10. scaledFrameLooksBlackSlabCorrupted()
- **Detects**: same as sprite slab detector but on pre-scaled playback frame
- **Thresholds (tighter than download)**: maxCh <= 5, spread <= 2, darkish ratio >= 50%
- **CAUTION**: original maxCh <= 8 false-positived on dark Pacific Ocean (maxCh ~6-8)
- **Wired in**: freeze-back eviction check (`currentScaledFreezeFrameLooksCorrupted`)

### 11. scaledFrameLooksFreezeBlockCorrupted()
- **Detects**: freeze-block artifacts in scaled frame
- **Wired in**: freeze-back eviction check

### 12. scaledFrameLooksHoldBlockCorrupted()
- **Detects**: hold-block artifacts in scaled frame
- **Wired in**: freeze-back eviction check

### 13. currentScaledPlaybackFrameLooksCorrupted()
- **Aggregate**: calls scaledFrameLooksFreezeBlockCorrupted + scaledFrameLooksHoldBlockCorrupted + scaledFrameLooksBlackSlabCorrupted
- **Purpose**: composite check for any corruption in current playback frame

## Semantic/Statistical Detectors

### 14. weatherFrameLooksCompressedSizeOutlier()
- **Detects**: JPEG file size significantly different from temporal neighbors
- **Method**: compares candidate size against adjacent frame sizes with tight tolerance
- **Wired in**: raw-build admission, cache repair validation

### 15. weatherFrameLooksSemanticOutlier()
- **Detects**: color/texture content mismatch vs adjacent frames
- **Method**: `WeatherSemanticSignature` (11×20 tile mean RGB + texture gradient + cyan tile count)
- **Uses**: `weatherSemanticDistance()` for comparison
- **Wired in**: raw-build admission, cache repair validation, post-download repair

## Validation Coverage Matrix

| Detector | Download Time | Stored Frame | Raw-Build | Playback |
|----------|:---:|:---:|:---:|:---:|
| completelyBlack | Y | Y | Y | — |
| nearlyBlack | Y | Y | Y | — |
| partialDecode | Y | Y | Y | — |
| horizontallyCorrupted | Y | Y | Y | — |
| verticallyCorrupted | Y | Y | Y | — |
| holdFrameBlock | Y | Y | Y | — |
| cyanWhiteBlock | Y | Y | Y | — |
| blackSlab (sprite) | Y | Y | Y | — |
| bottomBandJunk | Y | Y | Y | — |
| blackSlab (scaled) | — | — | — | Y |
| freezeBlock (scaled) | — | — | — | Y |
| holdBlock (scaled) | — | — | — | Y |
| compressedSizeOutlier | — | repair | Y | — |
| semanticOutlier | — | repair | Y | — |

**CRITICAL RULE**: Any new detector must be wired into BOTH `validateBufferedWeatherFrameJpeg()` (download) AND `validateStoredWeatherFramePath()` (stored). Wiring only one = corruption enters through the other gate.

## Freeze-Back Eviction
- Fires at freeze-hold time (end of animation loop)
- Walks backward from newest frame until a clean frame is found
- For each corrupt frame detected:
  - Clears `s_idx.jpegValid[i]` and `s_idx.rawValid[i]`, writes index
  - Marks `s_streamValid[i] = 0` so playback skips the slot
- Next sync: rolling sync redownloads slots with cleared validity
- Only fires on NEWEST frames (lag boundary) — mid-animation corruption is not caught here
