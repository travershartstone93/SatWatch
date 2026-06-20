# Constants, Paths & Versions

## Display Dimensions
| Constant | Value | Purpose |
|----------|-------|---------|
| `DISP_W` | 320 | JPEG decode width (must be multiple of 16) |
| `DISP_H` | 176 | JPEG decode height (must be multiple of 16) |
| `SCALED_W` | 410 | Pre-scaled output width |
| `SCALED_H` | 360 | Pre-scaled output height |
| `RAW_FRAME_BYTES` | 112,640 | 320×176×2 (source sprite size) |
| `SCALED_FRAME_BYTES` | 295,200 | 410×360×2 (raw stream slot size) |
| `SCALED_BAR_H` | ~31 | Top/bottom bar row height at scaled resolution |

## Zoom Dimensions
| Constant | Value | Purpose |
|----------|-------|---------|
| `ZOOM_FETCH_W` | 640 | 2x supersampled zoom width |
| `ZOOM_FETCH_H` | 344 | 2x supersampled zoom height (172×2, not 176×2) |
| `TERRAIN_FETCH_W` | 640 | 2x terrain width |
| `TERRAIN_FETCH_H` | 352 | 2x terrain height (176×2, NOT same as ZOOM_FETCH_H) |
| `ZOOM3_FINAL_W_KM` | 250.0 | Final zoom floor width in km |
| `ZOOM3_FINAL_H_KM` | 135.0 | Final zoom floor height in km |

## Timing & Animation (config-s3.h)
| Constant | Value | Purpose |
|----------|-------|---------|
| `HOURS_BACK` | 24 | Hours of imagery to download |
| `CADENCE_MIN` | 10 | GOES full-disk cadence (minutes) |
| `GIBS_LAG_HOURS` | 2 | GIBS lags real-time |
| `FRAME_DELAY_MS` | 33 | Target ~30fps |
| `LOOPS_BEFORE_SLEEP` | 10 | Full animation loops before sleep |
| `SLEEP_HOURS` | 6 | Hours to sleep between sessions |
| `MAX_FRAMES` | 144 | Maximum frame slots (24h × 60min / 10min) |
| `MAX_JPEG_BYTES` | 131,072 | 128 KB max JPEG buffer |

## Meta/Cache Versions
| Constant | Value | Purpose |
|----------|-------|---------|
| `INDEX_MAGIC` | 0x4C534658 | "LSFX" — index.bin format magic |
| `CACHE_VALIDATE_VERSION` | 2 | Bump: force cache repair scan |
| `ZOOM_META_VERSION` | 6 | Bump: force zoom asset rebuild |
| `WEATHER_VIEW_VERSION` | 5 | Bump: force weather redownload |
| `WIFI_CONFIG_SLOTS` | 5 | Number of WiFi credential slots |

## SD Card File Paths
All paths relative to card root (no "/sdcard" prefix):

### Frame Cache
```
/frames/frames.bin        Contiguous JPEG store (64KB slots × 144)
/frames/index.bin         FrameStoreIndex struct (magic, ring head, timestamps, validity)
/frames/index.tmp         Temp index during atomic writes
/frames/stream.raw        Contiguous pre-scaled RGB565 playback stream
/frames/dim.cfg           Frame dimensions "320 176"
/frames/view.meta         Bbox/layer/cadence signature
/frames/validate.meta     Cache integrity marker
```

### Zoom & Terrain
```
/frames/zoom.meta                Zoom snapshot freshness
/frames/vz1.jpg, vz2.jpg, vz3.jpg     Zoom JPEG snapshots
/frames/vz1.raw, vz2.raw, vz3.raw     Pre-scaled zoom raw files
/frames/terrain_day.jpg          BlueMarble day terrain
/frames/terrain_night.jpg        VIIRS Black Marble night terrain
/frames/terrain_day.raw          Pre-scaled day terrain raw
/frames/terrain_night.raw        Pre-scaled night terrain raw
/frames/terrain_radar_z3.jpg     NEXRAD radar overlay
/frames/.terrain_radar.raw       Temp radar raw (dot-prefix = hidden)
```

### Radar & Diagnostics
```
/frames/radar.meta        Last radar UTC timestamp
/diag.txt                 Runtime diagnostics log
```

### Audio Cues
```
/power_up.raw             Default startup chime (raw PCM)
/chime2.raw               Reserved chime 2
/chime3.raw               Reserved chime 3
```

## GIBS WMS URL Template
```
https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi
  ?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap
  &STYLES=&SRS=EPSG:4326
  &LAYERS={layer_name}
  &TIME={ISO_8601_UTC}
  &BBOX={west},{south},{east},{north}
  &WIDTH={width}&HEIGHT={height}
  &FORMAT=image/jpeg
```

## Weather Layers
| Constant | GIBS Layer Name |
|----------|----------------|
| `WEATHER_LAYER_GOES_EAST` | `GOES-East_ABI_GeoColor` |
| `WEATHER_LAYER_GOES_WEST` | `GOES-West_ABI_GeoColor` |
| `WEATHER_LAYER_HIMAWARI_IR` | `Himawari_AHI_Band13_Clean_Infrared` |

## Terrain Layers
- Day: `BlueMarble_NextGeneration` (dynamic date based on DOY)
- Night: `VIIRS_Black_Marble` (fixed date: 2016-01-01)
- Radar: `mapservices.weather.noaa.gov/eventdriven` (NEXRAD)

## WiFi Portal
| Constant | Value |
|----------|-------|
| `WIFI_PORTAL_AP_SSID` | "Sat Watch" |
| `WIFI_PORTAL_AP_PASS` | "123456789" |
| `WIFI_PORTAL_HOSTNAME` | "satwatch" |
| Portal URL | `http://satwatch.local` or `http://192.168.4.1` |

## NVS Keys (namespace "satwatch")
- WiFi slots: `ssid0`..`ssid4`, `pass0`..`pass4`
- Clock: `clk12h`
- Update: `upmode`, `upint`, `uptoh`
- Schedule: `schcnt`, `schm0`..`schm7`
- Cue: `cuemode`, `cuepath`, `cuevol`
- Sleep: `slpena`, `slpauto`
- Location: `utcoff`
- Sync skip: `skipnext`

## Key Buffer Sizes
| Buffer | Location | Size |
|--------|----------|------|
| `s_dlBuf` | PSRAM | 128 KB (MAX_JPEG_BYTES) |
| `s_frameDisplayBuf` | PSRAM | 295,200 bytes |
| `s_terrainDisplayBuf` | PSRAM | 295,200 bytes |
| `s_topBarBuf` | PSRAM | ~25,420 bytes (410×31×2) |
| `s_botBarBuf` | PSRAM | ~25,420 bytes |
| `s_audioCueBuf` | PSRAM | up to 1 MB |
| `sprite` | PSRAM | 112,640 bytes (320×176×2) |
| `s_clockFxSprite` | PSRAM | variable (computed text bounds) |
| Arduino loop stack | DRAM | 32,768 bytes |
