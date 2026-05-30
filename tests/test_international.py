#!/usr/bin/env python3
"""
International coverage test harness for LiveSat.

Tests all three international features:
1. Satellite: GOES GeoColor, MTG GeoColor, Himawari IR+S2 composite
2. Radar: RainViewer (global) for non-US, NOAA for US
3. Forecast: Open-Meteo (global) for non-US, NWS for US

Outputs stills, radar overlays, and animation GIFs at watch display size (410x360).
"""

import sys, math, json
from datetime import datetime, timedelta, timezone
from pathlib import Path
from io import BytesIO

try:
    import numpy as np
    from PIL import Image
    import requests
except ImportError:
    print("pip install numpy pillow requests")
    sys.exit(1)

OUTPUT_DIR = Path(__file__).parent / "output"
OUTPUT_DIR.mkdir(exist_ok=True)

FETCH_W = 640
FETCH_H = 352
WATCH_W = 410
WATCH_H = 360

# ─── Watch bbox (matches firmware) ───

def watch_bbox(lat, lon):
    half_lat = 4.5
    aspect = 320.0 / 172.0
    cos_lat = math.cos(math.radians(lat))
    if cos_lat < 0.25: cos_lat = 0.25
    half_lon = (half_lat * aspect) / cos_lat
    if half_lon > 60: half_lon = 60
    return (lon - half_lon, lat - half_lat, lon + half_lon, lat + half_lat)

# ─── Source selection (matches firmware plan) ───

def select_sat_source(lon):
    if lon >= 80: return "himawari"
    elif lon >= -15: return "mtg"
    elif lon >= -110: return "goes_east"
    else: return "goes_west"

def select_radar_source(lon):
    if -130 < lon < -15: return "noaa"
    return "rainviewer"

def select_forecast_source(lon):
    if -130 < lon < -15: return "nws"
    return "openmeteo"

def clamp_bbox(bbox, source):
    w, s, e, n = bbox
    if source == "mtg":
        if e > 78: e = 78
        if w < -78: w = -78
    return (w, s, e, n)

# ─── Satellite downloads ───

def recent_time(cadence_min=10, lag_hours=3):
    now = datetime.now(timezone.utc) - timedelta(hours=lag_hours)
    minute = (now.minute // cadence_min) * cadence_min
    return now.replace(minute=minute, second=0, microsecond=0)

def download_gibs(layer, bbox, time_dt):
    ts = time_dt.strftime("%Y-%m-%dT%H:%M:%SZ")
    url = (f"https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi"
           f"?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap&STYLES=&SRS=EPSG:4326"
           f"&LAYERS={layer}&BBOX={bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
           f"&WIDTH={FETCH_W}&HEIGHT={FETCH_H}&FORMAT=image%2Fjpeg&TIME={ts}")
    r = requests.get(url, timeout=30)
    return r.content if r.status_code == 200 and len(r.content) > 5000 else None

def download_eumetview(layer, bbox, time_dt):
    ts = time_dt.strftime("%Y-%m-%dT%H:%M:%SZ")
    url = (f"https://view.eumetsat.int/geoserver/ows"
           f"?service=WMS&version=1.1.1&request=GetMap"
           f"&layers={layer}&styles=&srs=EPSG:4326"
           f"&bbox={bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
           f"&width={FETCH_W}&height={FETCH_H}&format=image/jpeg&TIME={ts}")
    r = requests.get(url, timeout=30)
    return r.content if r.status_code == 200 and len(r.content) > 5000 else None

def download_sentinel2(bbox):
    url = (f"https://tiles.maps.eox.at/wms"
           f"?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
           f"&LAYERS=s2cloudless-2024&STYLES=&SRS=EPSG:4326"
           f"&BBOX={bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
           f"&WIDTH={FETCH_W}&HEIGHT={FETCH_H}&FORMAT=image%2Fjpeg")
    r = requests.get(url, timeout=30)
    return r.content if r.status_code == 200 and len(r.content) > 5000 else None

def fetch_sat_frame(source, bbox, max_lag=24):
    for lag in range(1, max_lag + 1):
        t = recent_time(cadence_min=10, lag_hours=lag)
        if source in ("goes_east", "goes_west"):
            layer = "GOES-East_ABI_GeoColor" if source == "goes_east" else "GOES-West_ABI_GeoColor"
            data = download_gibs(layer, bbox, t)
        elif source == "mtg":
            data = download_eumetview("mtg_fd:rgb_geocolour", bbox, t)
        elif source == "himawari":
            data = download_gibs("Himawari_AHI_Band13_Clean_Infrared", bbox, t)
        else:
            data = None
        if data:
            return data, t, lag
    return None, None, None

# ─── Himawari IR composite (screen blend) ───

def composite_ir_over_terrain(ir_gray, terrain_rgb):
    h, w = ir_gray.shape
    terrain = np.array(Image.fromarray(terrain_rgb).resize((w, h), Image.LANCZOS)).astype(float) / 255.0
    ir_f = ir_gray.astype(float)
    p5 = float(np.percentile(ir_f, 5))
    p95 = float(np.percentile(ir_f, 95))
    if p95 - p5 < 20: p5, p95 = 0.0, 255.0
    norm = np.clip((ir_f - p5) / (p95 - p5), 0.0, 1.0)
    inv = 1.0 - norm
    inv = np.clip(inv - 0.05, 0.0, 1.0) / 0.95
    inv_3d = np.stack([inv] * 3, axis=-1)
    result = 1.0 - (1.0 - terrain) * (1.0 - inv_3d)
    return (np.clip(result, 0.0, 1.0) * 255).astype(np.uint8)

# ─── RainViewer radar ───

def tile_xy(lat, lon, z):
    n = 2**z
    tx = int((lon + 180) / 360 * n)
    lat_rad = math.radians(max(-85, min(85, lat)))
    ty = int((1 - math.log(math.tan(lat_rad) + 1/math.cos(lat_rad)) / math.pi) / 2 * n)
    return tx, ty

def tile_bounds(tx, ty, z):
    n = 2**z
    lon1 = tx / n * 360 - 180
    lon2 = (tx + 1) / n * 360 - 180
    lat1 = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (ty + 1) / n))))
    lat2 = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))
    return lon1, lat1, lon2, lat2

def fetch_rainviewer_radar(bbox, path=None):
    """Fetch RainViewer radar tiles, stitch, crop to bbox."""
    if not path:
        rv = requests.get('https://api.rainviewer.com/public/weather-maps.json', timeout=30).json()
        path = rv['radar']['past'][-1]['path']

    z = 5
    tx_min, ty_max = tile_xy(bbox[1], bbox[0], z)
    tx_max, ty_min = tile_xy(bbox[3], bbox[2], z)

    tile_size = 256
    img_w = (tx_max - tx_min + 1) * tile_size
    img_h = (ty_max - ty_min + 1) * tile_size
    stitched = Image.new('RGBA', (img_w, img_h), (0, 0, 0, 0))

    for ty in range(ty_min, ty_max + 1):
        for tx in range(tx_min, tx_max + 1):
            url = f'https://tilecache.rainviewer.com{path}/256/{z}/{tx}/{ty}/6/1_1.png'
            try:
                r = requests.get(url, timeout=30)
                if r.status_code == 200 and len(r.content) > 100:
                    tile = Image.open(BytesIO(r.content)).convert('RGBA')
                    stitched.paste(tile, ((tx - tx_min) * tile_size, (ty - ty_min) * tile_size))
            except:
                pass

    # Crop to bbox
    tb_tl = tile_bounds(tx_min, ty_min, z)
    tb_br = tile_bounds(tx_max, ty_max, z)
    full_w, full_n = tb_tl[0], tb_tl[3]
    full_e, full_s = tb_br[2], tb_br[1]

    if full_e - full_w == 0 or full_n - full_s == 0:
        return stitched.resize((WATCH_W, WATCH_H), Image.LANCZOS)

    px_l = int((bbox[0] - full_w) / (full_e - full_w) * img_w)
    px_r = int((bbox[2] - full_w) / (full_e - full_w) * img_w)
    px_t = int((full_n - bbox[3]) / (full_n - full_s) * img_h)
    px_b = int((full_n - bbox[1]) / (full_n - full_s) * img_h)

    cropped = stitched.crop((max(0, px_l), max(0, px_t), min(img_w, px_r), min(img_h, px_b)))
    return cropped.resize((WATCH_W, WATCH_H), Image.LANCZOS)

# ─── Open-Meteo forecast ───

WMO_CODES = {
    0: "Clear", 1: "Mostly Clear", 2: "Partly Cloudy", 3: "Overcast",
    45: "Fog", 48: "Rime Fog",
    51: "Light Drizzle", 53: "Drizzle", 55: "Heavy Drizzle",
    61: "Light Rain", 63: "Rain", 65: "Heavy Rain",
    66: "Freezing Rain", 67: "Heavy Freezing Rain",
    71: "Light Snow", 73: "Snow", 75: "Heavy Snow", 77: "Snow Grains",
    80: "Light Showers", 81: "Showers", 82: "Heavy Showers",
    85: "Light Snow Showers", 86: "Snow Showers",
    95: "Thunderstorm", 96: "Hail Thunderstorm", 99: "Heavy Hail Thunderstorm",
}

def fetch_openmeteo_forecast(lat, lon):
    url = (f"https://api.open-meteo.com/v1/forecast"
           f"?latitude={lat}&longitude={lon}"
           f"&hourly=temperature_2m,precipitation_probability,wind_speed_10m,wind_direction_10m,weather_code"
           f"&daily=temperature_2m_max,temperature_2m_min,precipitation_probability_max,weather_code"
           f"&timezone=auto&forecast_days=5")
    r = requests.get(url, timeout=15)
    if r.status_code != 200:
        return None
    data = r.json()

    # Parse hourly (next 12 entries)
    hourly = []
    h = data.get("hourly", {})
    for i in range(min(12, len(h.get("time", [])))):
        hourly.append({
            "time": h["time"][i],
            "temp_c": h["temperature_2m"][i],
            "precip_pct": h["precipitation_probability"][i],
            "wind_kmh": h["wind_speed_10m"][i],
            "wind_dir": h["wind_direction_10m"][i],
            "code": h["weather_code"][i],
            "desc": WMO_CODES.get(h["weather_code"][i], "Unknown"),
        })

    # Parse daily (5 days)
    daily = []
    d = data.get("daily", {})
    for i in range(min(5, len(d.get("time", [])))):
        daily.append({
            "date": d["time"][i],
            "high_c": d["temperature_2m_max"][i],
            "low_c": d["temperature_2m_min"][i],
            "precip_pct": d["precipitation_probability_max"][i],
            "code": d["weather_code"][i],
            "desc": WMO_CODES.get(d["weather_code"][i], "Unknown"),
        })

    return {"hourly": hourly, "daily": daily, "timezone": data.get("timezone")}

# ─── Main ───

CITIES = {
    "newyork": {"lat": 40.7, "lon": -74.0, "name": "New York"},
    "london":  {"lat": 51.5, "lon": -0.1, "name": "London"},
    "mumbai":  {"lat": 19.1, "lon": 72.9, "name": "Mumbai"},
    "tokyo":   {"lat": 35.7, "lon": 139.7, "name": "Tokyo"},
}

def main():
    print("=== LiveSat International Coverage Test ===\n")

    terrain_cache = {}

    # ─── SATELLITE FRAMES ───
    print("--- Satellite Frames ---")
    for city, info in CITIES.items():
        sat_src = select_sat_source(info["lon"])
        bbox = clamp_bbox(watch_bbox(info["lat"], info["lon"]), sat_src)
        print(f"[{city}] sat={sat_src} bbox=({bbox[0]:.1f},{bbox[1]:.1f},{bbox[2]:.1f},{bbox[3]:.1f})")

        if sat_src == "himawari":
            s2 = download_sentinel2(bbox)
            if not s2:
                print("  Sentinel-2 FAILED"); continue
            terrain_cache[city] = s2

        data, t, lag = fetch_sat_frame(sat_src, bbox)
        if not data:
            print("  Satellite FAILED"); continue
        print(f"  ok lag={lag}h {t.strftime('%H:%M')} UTC")

        if sat_src == "himawari":
            ir_gray = np.array(Image.open(BytesIO(data)).convert("L"))
            terrain_rgb = np.array(Image.open(BytesIO(terrain_cache[city])).convert("RGB"))
            Image.fromarray(ir_gray).resize((WATCH_W, WATCH_H), Image.LANCZOS).save(OUTPUT_DIR / f"{city}_ir_raw.png")
            Image.fromarray(terrain_rgb).resize((WATCH_W, WATCH_H), Image.LANCZOS).save(OUTPUT_DIR / f"{city}_terrain.png")
            result = composite_ir_over_terrain(ir_gray, terrain_rgb)
            Image.fromarray(result).resize((WATCH_W, WATCH_H), Image.LANCZOS).save(OUTPUT_DIR / f"{city}_frame.png")
        else:
            img = Image.open(BytesIO(data)).convert("RGB")
            img.resize((WATCH_W, WATCH_H), Image.LANCZOS).save(OUTPUT_DIR / f"{city}_frame.png")
        print(f"  -> {city}_frame.png")

    # ─── RADAR ───
    print("\n--- Radar ---")
    rv_api = None
    for city, info in CITIES.items():
        radar_src = select_radar_source(info["lon"])
        bbox = watch_bbox(info["lat"], info["lon"])

        if radar_src == "noaa":
            print(f"[{city}] radar=NOAA (existing firmware, skip test)")
            continue

        print(f"[{city}] radar=RainViewer...", end=" ", flush=True)
        try:
            if not rv_api:
                rv_api = requests.get('https://api.rainviewer.com/public/weather-maps.json', timeout=30).json()
            path = rv_api['radar']['past'][-1]['path']
            radar_img = fetch_rainviewer_radar(bbox, path)
            radar_img.save(OUTPUT_DIR / f"{city}_radar.png")

            # Composite over satellite frame
            sat_path = OUTPUT_DIR / f"{city}_frame.png"
            if sat_path.exists():
                sat = Image.open(sat_path).convert("RGBA")
                sat.paste(radar_img, (0, 0), radar_img)
                sat.save(OUTPUT_DIR / f"{city}_with_radar.png")
            print(f"ok -> {city}_radar.png, {city}_with_radar.png")
        except Exception as e:
            print(f"FAILED: {e}")

    # ─── FORECAST ───
    print("\n--- Forecast ---")
    for city, info in CITIES.items():
        fc_src = select_forecast_source(info["lon"])
        if fc_src == "nws":
            print(f"[{city}] forecast=NWS (existing firmware, skip test)")
            continue

        print(f"[{city}] forecast=Open-Meteo...", end=" ", flush=True)
        fc = fetch_openmeteo_forecast(info["lat"], info["lon"])
        if not fc:
            print("FAILED"); continue

        # Save parsed forecast
        with open(OUTPUT_DIR / f"{city}_forecast.json", "w") as f:
            json.dump(fc, f, indent=2)

        h = fc["hourly"][0] if fc["hourly"] else {}
        d = fc["daily"][0] if fc["daily"] else {}
        print(f"ok tz={fc['timezone']}")
        print(f"  Now: {h.get('temp_c')}°C {h.get('desc')} precip={h.get('precip_pct')}% wind={h.get('wind_kmh')}km/h")
        print(f"  Today: {d.get('high_c')}°/{d.get('low_c')}° {d.get('desc')}")

    # ─── ANIMATION GIFs ───
    print("\n--- Animation GIFs ---")
    for city, info in CITIES.items():
        sat_src = select_sat_source(info["lon"])
        bbox = clamp_bbox(watch_bbox(info["lat"], info["lon"]), sat_src)

        _, start_t, _ = fetch_sat_frame(sat_src, bbox)
        if not start_t:
            print(f"[{city}] No data for animation"); continue

        print(f"[{city}] {sat_src} from {start_t.strftime('%H:%M')} UTC...", end=" ", flush=True)
        frames = []
        for i in range(24):
            t = start_t - timedelta(minutes=i * 10)
            if sat_src in ("goes_east", "goes_west"):
                layer = "GOES-East_ABI_GeoColor" if sat_src == "goes_east" else "GOES-West_ABI_GeoColor"
                data = download_gibs(layer, bbox, t)
            elif sat_src == "mtg":
                data = download_eumetview("mtg_fd:rgb_geocolour", bbox, t)
            elif sat_src == "himawari":
                data = download_gibs("Himawari_AHI_Band13_Clean_Infrared", bbox, t)
            else:
                data = None

            if not data: continue

            if sat_src == "himawari" and city in terrain_cache:
                ir_gray = np.array(Image.open(BytesIO(data)).convert("L"))
                terrain_rgb = np.array(Image.open(BytesIO(terrain_cache[city])).convert("RGB"))
                result = composite_ir_over_terrain(ir_gray, terrain_rgb)
                frames.append(Image.fromarray(result).resize((WATCH_W, WATCH_H), Image.LANCZOS))
            else:
                frames.append(Image.open(BytesIO(data)).convert("RGB").resize((WATCH_W, WATCH_H), Image.LANCZOS))

        if not frames:
            print("no frames"); continue

        frames.reverse()
        gif_path = OUTPUT_DIR / f"{city}_anim.gif"
        frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=150, loop=0)
        print(f"{len(frames)} frames -> {city}_anim.gif")

    print(f"\nOutput: {OUTPUT_DIR}")

if __name__ == "__main__":
    main()
