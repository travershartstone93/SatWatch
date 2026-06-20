# Sync Decision Tree — syncFramesRolling()

## Overview
`syncFramesRolling()` is the primary sync entry point. It decides whether to:
- Skip entirely (cache is current)
- Tail-fill (download only new frames)
- Roll forward (shift existing + download new)
- Full refresh (download everything)

## Window Model
```
targetFrameCount = HOURS_BACK * 60 / cadence  (e.g., 144 for 24h @ 10min)
fetchEnd = roundToCadence(now - lagHours)
fetchStart = fetchEnd - (targetFrameCount - 1) * cadence
```

## Decision Flow

### Gate 0: View Changed?
`view.meta` stores bbox/layer/cadence signature.
- Mismatch → **full refresh** (different geographic region or satellite source)

### Gate 1: In-Window Coverage?
Count existing cached timestamps that fall within [fetchStart, fetchEnd].
- `recentCount == 0` → **full refresh** (no overlap at all)

### Gate 2: Cache Exact Match?
```
if (cacheCount == targetFrameCount && exact timeline match):
    if (newest == fetchEnd):
        "latest-current" → if raw current: SKIP ENTIRELY
                          → if raw stale: rebuild raw only
    else:
        "exact-current" → same logic
```
**Fast boot path**: no network activity for weather JPEGs.

### Gate 3: Prefix Match? (Tail-Fill)
```
if (cacheMatchesWindowPrefix):
    reuse existing prefix frames
    download only missing tail frames
    write updated times/meta
    fast raw remap/rebuild
```

### Gate 4: Forward Shift Detected?
```
if (detectForwardWindowShift > 0):
    logically shift existing frames left
    download only newest slots entering window
    special tail-only optimization when at target count:
        move existing files in-place
        download shifted tail set
```

### Gate 5: General Rolling Path
For each target slot timestamp:
- If exact timestamp exists in index and slot has valid JPEG: reuse
- Else: download that slot → write to frames.bin ring slot → update index.bin

### Gate 6: Partial-Window Tolerance
```
if (all cached frames still reusable && source doesn't expose full window):
    "partial-stable keep" → keep current cache, avoid full-refresh loop

if (assembled set is partial but improved/usable):
    "partial-keep" → commit partial set, continue
```
Cache count < target is VALID state — not treated as failure.

### Gate 7: Escalation to Full Refresh
- `saved < target` and partial-keep criteria not met
- Tail-fill/tail-only path fails coherence
- File move/install failures break coherence
- On escalate: clear temp files, call `downloadFrames()`

## downloadFrames() — Full Refresh Pipeline
1. NTP sync
2. IP geolocation → satellite selection
3. Download all target frames with retry offsets: `{0, -60, 60, -120, 120, -180, 180}` seconds
4. Each frame: transport → decode → structural validation → atomic install
5. Cache repair: `validateAndRepairFullCacheIfNeeded()`
6. Build raw playback cache
7. Refresh zoom snapshots

## Raw Cache Coupling
After weather sync completes:
- `rebuildRawFromStored()`: iterates `s_idx`, decodes each valid JPEG slot from frames.bin → stream.raw
- Skips slots already marked `rawValid` in the index (incremental when possible)

## Key Functions
- `loadIndex()` / `writeIndex()`: read/write `FrameStoreIndex` struct (index.bin)
- `ensureStreamOpen()`: opens stream.raw, triggers `rebuildRawFromStored()` if needed
- `writeCurrentFrameDimMeta()`: writes dim.cfg (must be called after every cache install)

## Caution: Index Write Atomicity
`writeIndex()` writes to `index.tmp` then renames to `index.bin`. Power loss mid-write loses temp but preserves last good index.
