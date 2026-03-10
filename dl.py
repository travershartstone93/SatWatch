#!/usr/bin/env python3
"""
Test downloader that exactly replicates esp32-LiveSat-s3 GIBS fetch logic.

Replicates:
  - GIBS_WMS_BASE (including STYLES= and SRS=EPSG:4326)
  - computeWeatherBboxFromCenter(lat=50.26, lon=-119.27)  [halfLat=4.5, aspect=320/172]
  - BBOX formatted with %.1f
  - toISO() -> %Y-%m-%dT%H:%M:%SZ
  - SIZE = 320x176
  - LAYER = GOES-West_ABI_GeoColor (lon=-119.27 < -110 split)
  - MIN_WEATHER_JPEG_BYTES = 7000
  - 7 secondOffsets: [0, +60, -60, +600, -600, +1200, -1200]
  - lag=2h, cadence=10min, totalFrames=144

Downloads frames across the 24h window and saves them so you can SEE what
GIBS is actually returning. Run: python3 dl.py [--slot N] [--all]
"""

import math, os, sys, time, datetime, urllib.request, urllib.error, shutil
import struct

# ── Firmware constants ────────────────────────────────────────────────────────
LAT           = 50.26
LON           = -119.27
DISP_W        = 320
DISP_H        = 176
LAYER         = "GOES-West_ABI_GeoColor"
MIN_JPEG_BYTES = 7000
CADENCE_MIN   = 10
LAG_HOURS     = 2
TOTAL_FRAMES  = 144
SECOND_OFFSETS = [0, 60, -60, 600, -600, 1200, -1200]

GIBS_WMS_BASE = (
    "https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi"
    "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap&STYLES=&SRS=EPSG:4326"
)

# ── BBOX computation (exact replica of computeWeatherBboxFromCenter) ──────────
def compute_bbox(lat, lon):
    half_lat = 4.5
    aspect   = 320.0 / 172.0           # kAspect — hardcoded in firmware
    cos_lat  = math.cos(math.radians(lat))
    if cos_lat < 0.25:
        cos_lat = 0.25
    half_lon = (half_lat * aspect) / cos_lat
    if half_lon > 60.0:
        half_lon = 60.0
    w = lon - half_lon
    e = lon + half_lon
    s = lat - half_lat
    n = lat + half_lat
    # clamp
    if s < -89.5: s = -89.5
    if n >  89.5: n =  89.5
    if w < -180.0: w = -180.0
    if e >  180.0: e =  180.0
    return w, s, e, n

W, S, E, N = compute_bbox(LAT, LON)
BBOX = f"{W:.1f},{S:.1f},{E:.1f},{N:.1f}"

# ── Time window ───────────────────────────────────────────────────────────────
now_utc   = int(time.time())
cadence_s = CADENCE_MIN * 60
fetch_end = (now_utc - LAG_HOURS * 3600) // cadence_s * cadence_s
fetch_start = fetch_end - (TOTAL_FRAMES - 1) * cadence_s

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frames")
if os.path.exists(OUT_DIR):
    shutil.rmtree(OUT_DIR)
os.makedirs(OUT_DIR)

# ── Helpers ───────────────────────────────────────────────────────────────────
def build_url(t):
    dt = datetime.datetime.utcfromtimestamp(t)
    ts = dt.strftime("%Y-%m-%dT%H:%M:%SZ")
    return (
        f"{GIBS_WMS_BASE}"
        f"&LAYERS={LAYER}"
        f"&BBOX={BBOX}"
        f"&WIDTH={DISP_W}&HEIGHT={DISP_H}"
        f"&FORMAT=image%2Fjpeg"
        f"&TIME={ts}"
    ), ts

def fetch(url):
    req = urllib.request.Request(url, headers={"User-Agent": "LiveSat-test/1.0"})
    try:
        with urllib.request.urlopen(req, timeout=20) as r:
            data = r.read()
            code = r.status
        return data, code
    except urllib.error.HTTPError as e:
        return b"", e.code
    except Exception as e:
        return b"", -1

def is_jpeg(data):
    return len(data) >= 4 and data[0] == 0xFF and data[1] == 0xD8

def dark_pct_rgb565_be(data):
    """
    Rough dark-pixel analysis matching spriteLooksBlackSlabCorrupted tile check.
    GIBS returns standard JPEG (not raw RGB565), so we do this on the raw JPEG
    bytes as a proxy — not exact, just shows if GIBS returned a mostly-black file.
    Actually we can use pillow if available, else skip.
    """
    try:
        from PIL import Image
        import io
        img = Image.open(io.BytesIO(data)).convert("RGB")
        w, h = img.size
        pixels = list(img.getdata())
        dark = 0
        for r, g, b in pixels:
            # scale to 5-bit (like RGB565): r5=r>>3, g5=g>>3, b5=b>>3
            r5 = r >> 3
            g5 = g >> 3
            b5 = b >> 3
            max_ch = max(r5, g5, b5)
            min_ch = min(r5, g5, b5)
            if max_ch <= 6 and (max_ch - min_ch) <= 2:
                dark += 1
        return dark * 100 // len(pixels), w, h
    except Exception:
        return -1, 0, 0

# ── Main download loop ────────────────────────────────────────────────────────
print(f"BBOX (%.1f): {BBOX}")
print(f"Layer:       {LAYER}")
print(f"Size:        {DISP_W}x{DISP_H}")
print(f"Window:  {datetime.datetime.utcfromtimestamp(fetch_start).isoformat()}Z")
print(f"       → {datetime.datetime.utcfromtimestamp(fetch_end).isoformat()}Z")
print(f"Output:      {OUT_DIR}")
print()

# Which slots to download
args = sys.argv[1:]
if "--slot" in args:
    idx = args.index("--slot")
    slots = [int(args[idx + 1])]
else:
    slots = list(range(TOTAL_FRAMES))  # default: all 144

print(f"{'i':>4} {'UTC time':>20} {'off':>6} {'bytes':>7} {'dark%':>6}  status")
print("─" * 70)

for i in slots:
    t = fetch_start + i * cadence_s
    dt = datetime.datetime.utcfromtimestamp(t)
    utc_str = dt.strftime("%Y-%m-%dT%H:%MZ")

    best = None
    best_offset = None
    all_results = []

    for offset in SECOND_OFFSETS:
        cand = t + offset
        url, ts = build_url(cand)
        data, code = fetch(url)

        if not is_jpeg(data):
            all_results.append((offset, 0, code, None))
            continue

        sz = len(data)
        dark, iw, ih = dark_pct_rgb565_be(data)
        all_results.append((offset, sz, code, dark))

        if sz >= MIN_JPEG_BYTES and best is None:
            best = (data, sz, dark, offset, ts)
            best_offset = offset

    if best:
        data, sz, dark, offset, ts = best
        dark_str = f"{dark:3d}%" if dark >= 0 else "  n/a"
        fname = f"slot{i:03d}_{dt.strftime('%Y%m%d_%H%M')}UTC.jpg"
        fpath = os.path.join(OUT_DIR, fname)
        with open(fpath, "wb") as f:
            f.write(data)
        print(f"{i:>4} {utc_str:>20} {offset:>+6}s {sz:>7} {dark_str}  OK → {fname}")
    else:
        # Show what we got from offset=0
        sz0 = all_results[0][1] if all_results else 0
        codes = [str(r[2]) for r in all_results if r[1] == 0]
        print(f"{i:>4} {utc_str:>20} {'all':>6}  {sz0:>7}  {'n/a':>5}  MISS (codes: {', '.join(set(codes))})")

print()

print(f"Saved to: {OUT_DIR}")
print("MISS = GIBS has no data for that slot at any of the 7 time offsets.")
