#!/usr/bin/env python3
"""
Satellite image alignment tool for LiveSat.

Automatically aligns a pre-rendered satellite image (like INSAT-3D from IMD)
to a georeferenced terrain reference (Sentinel-2) using edge cross-correlation.

Searches over position and scale to find the Mercator projection bounds of the
source image, then crops and composites IR over terrain using screen blend.

Usage:
    from align_satellite import find_projection_bounds, composite_aligned

    # One-time calibration: find the projection bounds of a source image
    bounds = find_projection_bounds(
        source_path="insat_3Dasiasec_ir1.jpg",
        calibration_bbox=(64.0, 14.6, 81.8, 23.6),  # known-good region
        data_top=50, data_bot=-30,  # skip header/footer pixels
    )

    # Per-frame: crop and composite
    result = composite_aligned(
        source_path="insat_latest.jpg",
        bounds=bounds,
        target_bbox=(64.0, 14.6, 81.8, 23.6),
        terrain_rgb=terrain_array,
    )
"""

import math
import numpy as np
from PIL import Image
from scipy.ndimage import sobel


def lat_to_merc(lat):
    return math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))


def find_projection_bounds(source_path, calibration_bbox, terrain_path_or_array,
                           data_top=50, data_bot_offset=30,
                           west_range=(39, 43), south_range=(-8, -4),
                           span_lon_range=(64, 72), span_lat_range=(48, 54),
                           step=0.25, span_step=0.5):
    """
    Find the Mercator projection bounds of a pre-rendered satellite image
    by correlating edges with a georeferenced terrain reference.

    Args:
        source_path: path to the satellite image (JPEG)
        calibration_bbox: (west, south, east, north) of a known region to use for alignment
        terrain_path_or_array: path to terrain JPEG or numpy RGB array at calibration_bbox
        data_top: pixel row where satellite data starts (skip header)
        data_bot_offset: pixels to skip from bottom (footer)
        west_range: (min, max) for west bound search
        south_range: (min, max) for south bound search
        span_lon_range: (min, max) for longitude span search
        span_lat_range: (min, max) for latitude span search
        step: search step for position (degrees)
        span_step: search step for span (degrees)

    Returns:
        dict with keys: west, south, east, north, correlation
    """
    source = np.array(Image.open(source_path).convert('L')).astype(float)
    h_full, w_full = source.shape
    data_bot = h_full - data_bot_offset

    if isinstance(terrain_path_or_array, (str, type(None))):
        terrain_gray = np.array(Image.open(terrain_path_or_array).convert('L')).astype(float)
    else:
        if terrain_path_or_array.ndim == 3:
            terrain_gray = np.mean(terrain_path_or_array.astype(float), axis=2)
        else:
            terrain_gray = terrain_path_or_array.astype(float)

    # Edge detection on terrain reference
    t_edges = np.sqrt(sobel(terrain_gray, axis=0)**2 + sobel(terrain_gray, axis=1)**2)
    t_edges = (t_edges - t_edges.mean()) / (t_edges.std() + 1e-6)

    best_corr = -1
    best_params = None

    for west in np.arange(west_range[0], west_range[1], step):
        for south in np.arange(south_range[0], south_range[1], step):
            for span_lon in np.arange(span_lon_range[0], span_lon_range[1], span_step):
                for span_lat in np.arange(span_lat_range[0], span_lat_range[1], span_step):
                    east = west + span_lon
                    north = south + span_lat

                    merc_s = lat_to_merc(south)
                    merc_n = lat_to_merc(north)

                    px_l = int((calibration_bbox[0] - west) / (east - west) * w_full)
                    px_r = int((calibration_bbox[2] - west) / (east - west) * w_full)
                    merc_bs = lat_to_merc(calibration_bbox[1])
                    merc_bn = lat_to_merc(calibration_bbox[3])
                    px_t = int(data_top + (merc_n - merc_bn) / (merc_n - merc_s) * (data_bot - data_top))
                    px_b = int(data_top + (merc_n - merc_bs) / (merc_n - merc_s) * (data_bot - data_top))

                    if px_l < 0 or px_r > w_full or px_t < 0 or px_b > h_full:
                        continue
                    if px_r - px_l < 50 or px_b - px_t < 50:
                        continue

                    crop = source[px_t:px_b, px_l:px_r]
                    crop_resized = np.array(Image.fromarray(crop.astype(np.uint8)).resize(
                        (terrain_gray.shape[1], terrain_gray.shape[0]), Image.LANCZOS)).astype(float)

                    c_edges = np.sqrt(sobel(crop_resized, axis=0)**2 + sobel(crop_resized, axis=1)**2)
                    c_edges = (c_edges - c_edges.mean()) / (c_edges.std() + 1e-6)

                    corr = np.sum(c_edges * t_edges)

                    if corr > best_corr:
                        best_corr = corr
                        best_params = (west, south, east, north)

    return {
        "west": best_params[0],
        "south": best_params[1],
        "east": best_params[2],
        "north": best_params[3],
        "correlation": best_corr,
    }


def crop_to_bbox(source_path, bounds, target_bbox, data_top=50, data_bot_offset=30):
    """
    Crop a pre-rendered satellite image to a target geographic bbox
    using known projection bounds.

    Args:
        source_path: path to source image
        bounds: dict with west/south/east/north (from find_projection_bounds)
        target_bbox: (west, south, east, north) to crop to
        data_top: pixel row where data starts
        data_bot_offset: pixels to skip from bottom

    Returns:
        PIL Image of the cropped region
    """
    img = Image.open(source_path)
    w_full, h_full = img.size
    data_bot = h_full - data_bot_offset

    merc_s = lat_to_merc(bounds["south"])
    merc_n = lat_to_merc(bounds["north"])

    px_l = int((target_bbox[0] - bounds["west"]) / (bounds["east"] - bounds["west"]) * w_full)
    px_r = int((target_bbox[2] - bounds["west"]) / (bounds["east"] - bounds["west"]) * w_full)
    merc_bs = lat_to_merc(target_bbox[1])
    merc_bn = lat_to_merc(target_bbox[3])
    px_t = int(data_top + (merc_n - merc_bn) / (merc_n - merc_s) * (data_bot - data_top))
    px_b = int(data_top + (merc_n - merc_bs) / (merc_n - merc_s) * (data_bot - data_top))

    return img.crop((max(0, px_l), max(0, px_t), min(w_full, px_r), min(h_full, px_b)))


def composite_aligned(source_path, bounds, target_bbox, terrain_rgb,
                      output_size=(410, 360), data_top=50, data_bot_offset=30,
                      invert_ir=False):
    """
    Crop source satellite image and screen-blend over terrain.

    Args:
        source_path: path to IR satellite image
        bounds: projection bounds dict
        target_bbox: geographic bbox to crop
        terrain_rgb: numpy array (H, W, 3) of terrain at target_bbox
        output_size: (width, height) for output
        data_top/data_bot_offset: header/footer skip
        invert_ir: if True, invert IR (for sensors where bright=warm)

    Returns:
        numpy array (H, W, 3) uint8 composited result
    """
    crop = crop_to_bbox(source_path, bounds, target_bbox, data_top, data_bot_offset)

    fetch_w, fetch_h = 640, 352
    ir_gray = np.array(crop.convert('L').resize((fetch_w, fetch_h), Image.LANCZOS))
    terrain = np.array(Image.fromarray(terrain_rgb).resize((fetch_w, fetch_h), Image.LANCZOS))

    t_img = terrain.astype(float) / 255.0
    ir_f = ir_gray.astype(float)
    p5 = float(np.percentile(ir_f, 5))
    p95 = float(np.percentile(ir_f, 95))
    if p95 - p5 < 20:
        p5, p95 = 0.0, 255.0
    norm = np.clip((ir_f - p5) / (p95 - p5), 0.0, 1.0)

    if invert_ir:
        norm = 1.0 - norm

    inv = np.clip(norm - 0.05, 0.0, 1.0) / 0.95
    inv_3d = np.stack([inv] * 3, axis=-1)
    result = 1.0 - (1.0 - t_img) * (1.0 - inv_3d)
    result = (np.clip(result, 0.0, 1.0) * 255).astype(np.uint8)

    return np.array(Image.fromarray(result).resize(output_size, Image.LANCZOS))


# Known calibrated bounds (from grid search)
INSAT_3DS_ASIA_SECTOR = {
    "west": 41.75,
    "south": -5.5,
    "east": 107.75,
    "north": 47.0,
    "data_top": 50,
    "data_bot_offset": 30,
}


if __name__ == "__main__":
    import requests
    from io import BytesIO
    from pathlib import Path

    output_dir = Path(__file__).parent / "output"

    # Example: align and composite INSAT for Mumbai
    bbox = (64.0, 14.6, 81.8, 23.6)
    bounds = INSAT_3DS_ASIA_SECTOR

    print("Downloading Sentinel-2 terrain...")
    s2 = requests.get(
        f"https://tiles.maps.eox.at/wms?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap"
        f"&LAYERS=s2cloudless-2024&STYLES=&SRS=EPSG:4326"
        f"&BBOX={bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
        f"&WIDTH=640&HEIGHT=352&FORMAT=image%2Fjpeg", timeout=30).content
    terrain = np.array(Image.open(BytesIO(s2)).convert("RGB"))

    print("Downloading INSAT-3D latest...")
    insat = requests.get("https://mausam.imd.gov.in/Satellite/3Dasiasec_ir1.jpg",
                          timeout=30, headers={"User-Agent": "Mozilla/5.0"})
    insat_path = output_dir / "insat_latest.jpg"
    insat_path.write_bytes(insat.content)

    print("Compositing...")
    result = composite_aligned(
        str(insat_path), bounds, bbox, terrain, invert_ir=False)
    Image.fromarray(result).save(output_dir / "mumbai_insat_final.png")
    print(f"Saved {output_dir / 'mumbai_insat_final.png'}")
