#!/usr/bin/env python3
"""Build animation GIFs for each satellite source using the false-color IR palette."""

import sys
from pathlib import Path
from datetime import datetime, timedelta, timezone
from io import BytesIO

try:
    import numpy as np
    from PIL import Image
    import requests
except ImportError:
    print("pip install numpy pillow requests")
    sys.exit(1)

# Import palette and helpers from the test harness
sys.path.insert(0, str(Path(__file__).parent))
from test_false_color_ir import (
    build_palette_land, build_palette_ocean,
    download_bluemarble_reference,
    composite_ir_over_terrain, download_gibs_himawari, download_eumetview,
    DISP_W, DISP_H, OUTPUT_DIR
)

def get_recent_time(cadence_min=10, lag_hours=3):
    now = datetime.now(timezone.utc) - timedelta(hours=lag_hours)
    minute = (now.minute // cadence_min) * cadence_min
    return now.replace(minute=minute, second=0, microsecond=0)

def find_working_start_time(download_fn, cadence_min=10, max_lag_hours=24):
    """Probe backwards to find the most recent working timestamp."""
    for lag in range(3, max_lag_hours + 1):
        t = get_recent_time(cadence_min=cadence_min, lag_hours=lag)
        data = download_fn(t)
        if data:
            print(f"  Found working data at lag={lag}h")
            return t
    return None

def colorize_frame(jpeg_data, terrain_rgb):
    img = Image.open(BytesIO(jpeg_data)).convert("L")
    gray = np.array(img)
    return Image.fromarray(composite_ir_over_terrain(gray, terrain_rgb))

def build_animation(name, bbox, download_fn, download_args, terrain_rgb,
                    num_frames=24, cadence_min=10):
    print(f"\n[{name}] Finding most recent data...")
    dl = lambda t: download_fn(*download_args, t) if download_args else download_fn(t)
    t = find_working_start_time(dl, cadence_min=cadence_min)
    if not t:
        print(f"  No data found for {name}!")
        return
    print(f"  Downloading {num_frames} frames from {t.strftime('%H:%M')} UTC...")
    frames = []
    for i in range(num_frames):
        t_frame = t - timedelta(minutes=i * cadence_min)
        data = dl(t_frame)
        if data:
            frame = colorize_frame(data, terrain_rgb)
            frames.append(frame)
            print(f"  frame {i:02d} ok")
        else:
            print(f"  frame {i:02d} SKIP")

    if not frames:
        print(f"  No frames for {name}!")
        return

    # Reverse so animation plays forward in time
    frames.reverse()

    gif_path = OUTPUT_DIR / f"anim_{name}.gif"
    frames[0].save(
        gif_path,
        save_all=True,
        append_images=frames[1:],
        duration=150,  # ms per frame
        loop=0
    )
    print(f"  Saved: {gif_path} ({len(frames)} frames)")

def main():
    import math
    def watch_bbox(lat, lon):
        half_lat = 4.5
        aspect = 320.0 / 172.0
        cos_lat = math.cos(math.radians(lat))
        if cos_lat < 0.25: cos_lat = 0.25
        half_lon = (half_lat * aspect) / cos_lat
        if half_lon > 60: half_lon = 60
        return (lon - half_lon, lat - half_lat, lon + half_lon, lat + half_lat)

    bbox_himawari = watch_bbox(35.7, 139.7)   # Tokyo
    bbox_fes = watch_bbox(51.5, -0.1)         # London
    bbox_iodc = watch_bbox(19.1, 72.9)        # Mumbai

    # Download Sentinel-2 terrain references
    print("--- Downloading Sentinel-2 terrain ---")
    terrains = {}
    for name, bbox in [("himawari", bbox_himawari), ("fes", bbox_fes), ("iodc", bbox_iodc)]:
        bm = download_bluemarble_reference(bbox)
        if bm:
            terrains[name] = np.array(Image.open(BytesIO(bm)).convert("RGB"))

    # Himawari (24 frames, 10min cadence = 4 hours)
    build_animation(
        "himawari", bbox_himawari,
        lambda t: download_gibs_himawari(t, bbox_himawari),
        None, terrains.get("himawari"),
        num_frames=24, cadence_min=10
    )

    # Meteosat FES (24 frames, 15min cadence = 6 hours)
    build_animation(
        "meteosat_fes", bbox_fes,
        lambda t: download_eumetview("msg_fes:ir108", t, bbox_fes),
        None, terrains.get("fes"),
        num_frames=24, cadence_min=15
    )

    # Meteosat IODC (24 frames, 15min cadence = 6 hours)
    build_animation(
        "meteosat_iodc", bbox_iodc,
        lambda t: download_eumetview("msg_iodc:ir108", t, bbox_iodc),
        None, terrains.get("iodc"),
        num_frames=24, cadence_min=15
    )

    print(f"\nAll animations saved to: {OUTPUT_DIR}")

if __name__ == "__main__":
    main()
