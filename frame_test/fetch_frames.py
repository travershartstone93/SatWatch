#!/usr/bin/env python3
"""
Replicates esp32-LiveSat-s3 frame download logic exactly.

Parameters mirror the firmware:
  - Layer:        GOES-West_ABI_GeoColor  (ip-geo: src=GOES-West)
  - Center:       lat=50.27, lon=-119.27  (BC, Canada from ip-api.com)
  - kHalfLatDeg:  4.5  (firmware constant)
  - kAspect:      320/172  (firmware constant, DISP_W/original DISP_H)
  - DISP_W x DISP_H: 320 x 176  (from dim.cfg on device)
  - HOURS_BACK:   24, CADENCE_MIN: 10, GIBS_LAG_HOURS: 2
  - Total frames: 144
"""

import math
import time
import urllib.request
import os
import sys

# ── Firmware constants ──────────────────────────────────────────────────────
LAYER        = "GOES-West_ABI_GeoColor"
LAT          = 50.27
LON          = -119.27
DISP_W       = 320
DISP_H       = 176
HOURS_BACK   = 24
CADENCE_MIN  = 10
LAG_HOURS    = 2
TOTAL_FRAMES = (HOURS_BACK * 60) // CADENCE_MIN   # 144
CADENCE_SEC  = CADENCE_MIN * 60
MIN_JPEG     = 5500   # firmware MIN_WEATHER_JPEG_BYTES

GIBS_BASE = (
    "https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi"
    "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
    "&STYLES=&SRS=EPSG:4326"
)

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# ── Bbox (same formula as computeWeatherBboxFromCenter) ─────────────────────
def compute_bbox(lat, lon):
    kHalfLatDeg = 4.5
    kAspect     = 320.0 / 172.0
    cosLat = math.cos(lat * 0.01745329252)
    if cosLat < 0.25:
        cosLat = 0.25
    halfLon = (kHalfLatDeg * kAspect) / cosLat
    if halfLon > 60.0:
        halfLon = 60.0
    w = lon - halfLon;  e = lon + halfLon
    s = lat - kHalfLatDeg; n = lat + kHalfLatDeg
    if s < -89.5: s = -89.5
    if n >  89.5: n =  89.5
    if w < -180.0: w = -180.0
    if e >  180.0: e =  180.0
    return w, s, e, n

# ── Round timestamp to cadence (firmware roundToCadence) ────────────────────
def round_to_cadence(t, cad=CADENCE_SEC):
    return (t // cad) * cad

# ── ISO 8601 UTC (firmware toISO) ───────────────────────────────────────────
def to_iso(t):
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t))

# ── Build URL (mirrors buildWeatherFrameUrl) ─────────────────────────────────
def build_url(t, w, s, e, n):
    return (
        f"{GIBS_BASE}"
        f"&LAYERS={LAYER}"
        f"&BBOX={w:.1f},{s:.1f},{e:.1f},{n:.1f}"
        f"&WIDTH={DISP_W}&HEIGHT={DISP_H}"
        f"&FORMAT=image%2Fjpeg"
        f"&TIME={to_iso(t)}"
    )

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    w, s, e, n = compute_bbox(LAT, LON)
    print(f"Center:  lat={LAT}  lon={LON}")
    print(f"BBOX:    W={w:.1f} S={s:.1f} E={e:.1f} N={n:.1f}")
    print(f"Layer:   {LAYER}")
    print(f"Size:    {DISP_W}x{DISP_H}")
    print(f"Frames:  {TOTAL_FRAMES}  (every {CADENCE_MIN} min, {HOURS_BACK}h back, {LAG_HOURS}h lag)")
    print()

    now       = int(time.time())
    fetch_end = round_to_cadence(now - LAG_HOURS * 3600)
    fetch_start = fetch_end - (TOTAL_FRAMES - 1) * CADENCE_SEC

    print(f"Window:  {to_iso(fetch_start)}  →  {to_iso(fetch_end)}")
    print(f"UTC now: {to_iso(now)}")
    print()

    ok = 0; fail = 0; small = 0; bad_jpeg = 0
    results = []

    for i in range(TOTAL_FRAMES):
        t = fetch_start + i * CADENCE_SEC
        url = build_url(t, w, s, e, n)
        fname = os.path.join(OUT_DIR, f"f{i:03d}.jpg")
        iso   = to_iso(t)

        try:
            req = urllib.request.Request(url, headers={"User-Agent": "LiveSat/1.0"})
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = resp.read()
        except Exception as ex:
            status = f"ERR {ex}"
            fail  += 1
            results.append((i, iso, 0, status))
            print(f"[{i+1:3d}/{TOTAL_FRAMES}] {iso}  FAIL  {ex}")
            sys.stdout.flush()
            continue

        size = len(data)

        # JPEG magic check (0xFF 0xD8)
        is_jpeg = (size >= 2 and data[0] == 0xFF and data[1] == 0xD8)
        if not is_jpeg:
            status = f"NOT-JPEG size={size}"
            bad_jpeg += 1
            results.append((i, iso, size, status))
            print(f"[{i+1:3d}/{TOTAL_FRAMES}] {iso}  BAD-JPEG  {size}B  hdr={data[:4].hex()}")
            sys.stdout.flush()
            continue

        if size < MIN_JPEG:
            status = f"TOO-SMALL size={size}"
            small += 1
            results.append((i, iso, size, status))
            print(f"[{i+1:3d}/{TOTAL_FRAMES}] {iso}  SMALL  {size}B")
            sys.stdout.flush()
            continue

        with open(fname, "wb") as f:
            f.write(data)

        ok += 1
        results.append((i, iso, size, "OK"))
        print(f"[{i+1:3d}/{TOTAL_FRAMES}] {iso}  OK  {size}B")
        sys.stdout.flush()

    print()
    print("=" * 60)
    print(f"DONE:  ok={ok}  fail={fail}  small={small}  bad_jpeg={bad_jpeg}  total={TOTAL_FRAMES}")

    if fail + small + bad_jpeg > 0:
        print()
        print("FAILURES:")
        for idx, iso, sz, st in results:
            if st != "OK":
                print(f"  f{idx:03d}  {iso}  {sz}B  {st}")

if __name__ == "__main__":
    main()
