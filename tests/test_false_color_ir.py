#!/usr/bin/env python3
"""
International satellite coverage test harness for LiveSat.

Downloads frames from all planned sources at watch-accurate bboxes,
outputs stills and animation GIFs at watch display size (410x360).

Sources:
  GOES GeoColor (GIBS)      — Americas, direct display
  MTG GeoColor (EUMETView)  — Europe/Africa/IO, direct display
  Himawari IR (GIBS)        — Asia-Pacific, IR + Sentinel-2 terrain composite
"""

import sys, math
from datetime import datetime, timedelta, timezone
from pathlib import Path
from io import BytesIO

try:
    import numpy as np
    from PIL import Image
    import requests
    from scipy.ndimage import gaussian_filter
except ImportError:
    print("pip install numpy pillow requests scipy")
    sys.exit(1)

OUTPUT_DIR = Path(__file__).parent / "output"
OUTPUT_DIR.mkdir(exist_ok=True)

FETCH_W = 640
FETCH_H = 352
WATCH_W = 410
WATCH_H = 360

# ─── Watch bbox (matches firmware computeWeatherBboxFromCenter) ───

def watch_bbox(lat, lon):
    half_lat = 4.5
    aspect = 320.0 / 172.0
    cos_lat = math.cos(math.radians(lat))
    if cos_lat < 0.25:
        cos_lat = 0.25
    half_lon = (half_lat * aspect) / cos_lat
    if half_lon > 60:
        half_lon = 60
    return (lon - half_lon, lat - half_lat, lon + half_lon, lat + half_lat)

# ─── Satellite selection (matches firmware selectSatelliteForLon) ───

def select_source(lon):
    if lon >= 80:
        return "himawari"
    elif lon >= -15:
        return "mtg"
    elif lon >= -110:
        return "goes_east"
    else:
        return "goes_west"

def clamp_bbox_to_source(bbox, source):
    """Clamp bbox to avoid satellite disk edge artifacts."""
    w, s, e, n = bbox
    if source == "mtg":
        # MTG disk edge at ~±81° lon — keep 3° margin
        if e > 78: e = 78
        if w < -78: w = -78
    return (w, s, e, n)

# ─── Download functions ───

def recent_time(cadence_min=10, lag_hours=3):
    now = datetime.now(timezone.utc) - timedelta(hours=lag_hours)
    minute = (now.minute // cadence_min) * cadence_min
    return now.replace(minute=minute, second=0, microsecond=0)

def download_gibs(layer, bbox, time_dt):
    ts = time_dt.strftime("%Y-%m-%dT%H:%M:%SZ")
    url = (
        "https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi"
        "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap&STYLES=&SRS=EPSG:4326"
        f"&LAYERS={layer}"
        f"&BBOX={bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
        f"&WIDTH={FETCH_W}&HEIGHT={FETCH_H}&FORMAT=image%2Fjpeg&TIME={ts}"
    )
    resp = requests.get(url, timeout=30)
    if resp.status_code == 200 and len(resp.content) > 1000:
        return resp.content
    return None

def download_eumetview(layer, bbox, time_dt):
    ts = time_dt.strftime("%Y-%m-%dT%H:%M:%SZ")
    url = (
        "https://view.eumetsat.int/geoserver/ows"
        "?service=WMS&version=1.1.1&request=GetMap"
        f"&layers={layer}&styles=&srs=EPSG:4326"
        f"&bbox={bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
        f"&width={FETCH_W}&height={FETCH_H}&format=image/jpeg&TIME={ts}"
    )
    resp = requests.get(url, timeout=30)
    if resp.status_code == 200 and len(resp.content) > 1000:
        return resp.content
    return None

def download_sentinel2(bbox):
    url = (
        "https://tiles.maps.eox.at/wms"
        "?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
        "&LAYERS=s2cloudless-2024&STYLES=&SRS=EPSG:4326"
        f"&BBOX={bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
        f"&WIDTH={FETCH_W}&HEIGHT={FETCH_H}&FORMAT=image%2Fjpeg"
    )
    resp = requests.get(url, timeout=30)
    if resp.status_code == 200 and len(resp.content) > 1000:
        return resp.content
    return None

def fetch_with_lag_search(download_fn, cadence_min=10, max_lag=24):
    """Try increasing lag hours until we get real data (not blank tiles)."""
    for lag in range(1, max_lag + 1):
        t = recent_time(cadence_min=cadence_min, lag_hours=lag)
        data = download_fn(t)
        if data and len(data) > 5000:  # blank GIBS tiles are ~1604 bytes
            return data, t, lag
    return None, None, None

# ─── Himawari IR + terrain composite ───

def composite_ir_over_terrain(ir_gray, terrain_rgb):
    h, w = ir_gray.shape
    terrain = np.array(Image.fromarray(terrain_rgb).resize((w, h), Image.LANCZOS)).astype(float) / 255.0

    # Normalize and invert IR (Band13: bright=warm/clear → inverted: bright=cloud)
    ir_f = ir_gray.astype(float)
    p5 = float(np.percentile(ir_f, 5))
    p95 = float(np.percentile(ir_f, 95))
    if p95 - p5 < 20:
        p5, p95 = 0.0, 255.0
    norm = np.clip((ir_f - p5) / (p95 - p5), 0.0, 1.0)
    inv = 1.0 - norm

    # Clip floor to keep clear sky perfectly clean
    inv = np.clip(inv - 0.05, 0.0, 1.0) / 0.95

    # Screen blend: preserves all IR cloud texture
    inv_3d = np.stack([inv] * 3, axis=-1)
    result = 1.0 - (1.0 - terrain) * (1.0 - inv_3d)

    return (np.clip(result, 0.0, 1.0) * 255).astype(np.uint8)

# ─── Frame processing per source type ───

def process_direct(jpeg_data, city):
    """GOES or MTG GeoColor — just display as-is."""
    img = Image.open(BytesIO(jpeg_data)).convert("RGB")
    return img.resize((WATCH_W, WATCH_H), Image.LANCZOS)

def process_himawari(ir_data, terrain_data, city):
    """Himawari IR composited over Sentinel-2 terrain."""
    ir_gray = np.array(Image.open(BytesIO(ir_data)).convert("L"))
    terrain_rgb = np.array(Image.open(BytesIO(terrain_data)).convert("RGB"))

    # Save intermediates
    Image.fromarray(ir_gray).resize((WATCH_W, WATCH_H), Image.LANCZOS).save(
        OUTPUT_DIR / f"{city}_ir_raw.png")
    Image.fromarray(terrain_rgb).resize((WATCH_W, WATCH_H), Image.LANCZOS).save(
        OUTPUT_DIR / f"{city}_terrain.png")

    result = composite_ir_over_terrain(ir_gray, terrain_rgb)
    return Image.fromarray(result).resize((WATCH_W, WATCH_H), Image.LANCZOS)

# ─── Main ───

CITIES = {
    "newyork": {"lat": 40.7, "lon": -74.0},
    "london":  {"lat": 51.5, "lon": -0.1},
    "mumbai":  {"lat": 19.1, "lon": 72.9},
    "tokyo":   {"lat": 35.7, "lon": 139.7},
}

def download_frame(source, bbox, time_dt):
    if source in ("goes_east", "goes_west"):
        layer = "GOES-East_ABI_GeoColor" if source == "goes_east" else "GOES-West_ABI_GeoColor"
        return download_gibs(layer, bbox, time_dt)
    elif source == "mtg":
        return download_eumetview("mtg_fd:rgb_geocolour", bbox, time_dt)
    elif source == "himawari":
        return download_gibs("Himawari_AHI_Band13_Clean_Infrared", bbox, time_dt)
    return None

def main():
    print("=== LiveSat International Coverage Test ===\n")

    terrain_cache = {}  # city → terrain JPEG bytes (Himawari only)

    # ─── Stills ───
    for city, info in CITIES.items():
        source = select_source(info["lon"])
        bbox = clamp_bbox_to_source(watch_bbox(info["lat"], info["lon"]), source)
        print(f"[{city}] source={source} bbox=({bbox[0]:.1f},{bbox[1]:.1f},{bbox[2]:.1f},{bbox[3]:.1f})")

        # Download terrain for Himawari
        if source == "himawari":
            print(f"  Sentinel-2 terrain...", end=" ", flush=True)
            s2_data = download_sentinel2(bbox)
            if not s2_data:
                print("FAILED")
                continue
            terrain_cache[city] = s2_data
            print("ok")

        # Download frame
        print(f"  {source} frame...", end=" ", flush=True)
        data, t, lag = fetch_with_lag_search(
            lambda t: download_frame(source, bbox, t),
            cadence_min=10, max_lag=12)
        if not data:
            print("FAILED (all lags)")
            continue
        print(f"ok (lag={lag}h, {t.strftime('%H:%M')} UTC)")

        # Process
        if source == "himawari":
            frame = process_himawari(data, terrain_cache[city], city)
        else:
            frame = process_direct(data, city)

        frame.save(OUTPUT_DIR / f"{city}_frame.png")
        print(f"  -> {city}_frame.png")

    # ─── Animation GIFs ───
    print("\n--- Animation GIFs ---")
    for city, info in CITIES.items():
        source = select_source(info["lon"])
        bbox = clamp_bbox_to_source(watch_bbox(info["lat"], info["lon"]), source)

        # Find working start time
        _, start_t, start_lag = fetch_with_lag_search(
            lambda t: download_frame(source, bbox, t),
            cadence_min=10, max_lag=12)
        if not start_t:
            print(f"[{city}] No data for animation")
            continue

        print(f"[{city}] {source} from {start_t.strftime('%H:%M')} UTC, 24 frames...", end=" ", flush=True)
        frames = []
        for i in range(24):
            t = start_t - timedelta(minutes=i * 10)
            data = download_frame(source, bbox, t)
            if not data or len(data) < 5000:
                continue
            if source == "himawari" and city in terrain_cache:
                frame = process_himawari(data, terrain_cache[city], city)
            else:
                frame = process_direct(data, city)
            frames.append(frame)

        if not frames:
            print("no frames")
            continue

        frames.reverse()  # chronological order
        gif_path = OUTPUT_DIR / f"{city}_anim.gif"
        frames[0].save(gif_path, save_all=True, append_images=frames[1:],
                        duration=150, loop=0)
        print(f"{len(frames)} frames -> {city}_anim.gif")

    print(f"\nOutput: {OUTPUT_DIR}")

if __name__ == "__main__":
    main()
