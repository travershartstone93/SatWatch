"""Parse BLE scan binary logs from LiveSat watch."""

import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

HEADER_SIZE = 16
MAGIC = b"BLSC"

# Path loss model defaults (indoor)
PATH_LOSS_N = 2.7  # path loss exponent (2.0=free space, 2.5-3.5=indoor)
TX_POWER_DEFAULT = -59  # assumed 1m RSSI when TX power unknown (127)


@dataclass
class BleLogHeader:
    version: int
    record_size: int
    boot_number: int


@dataclass
class BleRecord:
    timestamp_ms: int
    mac: str
    rssi: int
    adv_type: int
    payload_len: int
    payload_hash: int
    tx_power: int
    mfg_id: int = 0
    apple_type: int = 0
    extra_flags: int = 0


def parse_header(data: bytes) -> BleLogHeader:
    if len(data) < HEADER_SIZE:
        raise ValueError(f"File too small: {len(data)} bytes")
    magic = data[:4]
    if magic != MAGIC:
        raise ValueError(f"Bad magic: {magic!r}, expected {MAGIC!r}")
    ver, rec_sz = struct.unpack_from("<HH", data, 4)
    boot = struct.unpack_from("<I", data, 8)[0]
    return BleLogHeader(version=ver, record_size=rec_sz, boot_number=boot)


def parse_records(data: bytes) -> list[BleRecord]:
    hdr = parse_header(data)
    rec_sz = hdr.record_size
    has_mfg = rec_sz >= 20
    n = (len(data) - HEADER_SIZE) // rec_sz
    records = []
    for i in range(n):
        off = HEADER_SIZE + i * rec_sz
        ts = struct.unpack_from("<I", data, off)[0]
        mac_bytes = data[off + 4 : off + 10]
        mac = ":".join(f"{b:02X}" for b in mac_bytes)
        rssi = struct.unpack_from("b", data, off + 10)[0]
        adv_type = data[off + 11]
        payload_len = data[off + 12]
        payload_hash = struct.unpack_from("<I", data, off + 13)[0]
        tx_power = struct.unpack_from("b", data, off + 17)[0]
        mfg_id = struct.unpack_from("<H", data, off + 18)[0] if has_mfg else 0
        has_apple = rec_sz >= 22
        apple_type = data[off + 20] if has_apple else 0
        extra_flags = data[off + 21] if has_apple else 0
        records.append(
            BleRecord(ts, mac, rssi, adv_type, payload_len, payload_hash,
                      tx_power, mfg_id, apple_type, extra_flags)
        )
    return records


def load_log(path: str | Path) -> pd.DataFrame:
    data = Path(path).read_bytes()
    records = parse_records(data)
    if not records:
        return pd.DataFrame()
    df = pd.DataFrame([r.__dict__ for r in records])
    df["timestamp_s"] = df["timestamp_ms"] / 1000.0
    df["payload_hash_hex"] = df["payload_hash"].apply(lambda h: f"0x{h:08X}")
    ef = df["extra_flags"].values
    df["apple_dev_type"] = (ef // 16) & 0x0F
    df["connectable"] = (ef & 0x08).astype(bool)
    df["has_name"] = (ef & 0x04).astype(bool)
    df["le_mode"] = ef & 0x03
    return df


def estimate_distance(rssi: float, tx_power: int = TX_POWER_DEFAULT,
                       n: float = PATH_LOSS_N) -> float:
    """Estimate distance in meters from RSSI using log-distance path loss model.

    tx_power is the expected RSSI at 1m. BLE devices report this as a signed byte.
    Values > 20 or == 127 are treated as unknown and use the default.
    Small positive values (e.g. 8, 12) are real TX power levels in dBm — but they
    represent transmit power, not measured RSSI at 1m. Typical BLE: TX +8dBm ≈ -59dBm at 1m.
    """
    if tx_power == 127 or tx_power > 20 or tx_power > 0:
        tx_power = TX_POWER_DEFAULT
    return 10.0 ** ((tx_power - rssi) / (10.0 * n))


def _first_nonzero(series):
    nz = series[series != 0]
    return int(nz.iloc[0]) if len(nz) > 0 else 0


def build_device_summary(df: pd.DataFrame) -> pd.DataFrame:
    """Aggregate per-device stats from raw records."""
    if df.empty:
        return pd.DataFrame()

    def agg(g):
        return pd.Series({
            "hits": len(g),
            "rssi_mean": g["rssi"].mean(),
            "rssi_min": g["rssi"].min(),
            "rssi_max": g["rssi"].max(),
            "rssi_std": g["rssi"].std(),
            "first_seen_s": g["timestamp_s"].min(),
            "last_seen_s": g["timestamp_s"].max(),
            "duration_s": g["timestamp_s"].max() - g["timestamp_s"].min(),
            "payload_hash_hex": g["payload_hash_hex"].iloc[0],
            "tx_power": g["tx_power"].iloc[0],
            "adv_type": g["adv_type"].iloc[0],
            "payload_len": g["payload_len"].mode().iloc[0] if len(g) > 0 else 0,
            "dist_est_m": estimate_distance(g["rssi"].median(), g["tx_power"].iloc[0]),
            "mfg_id": _first_nonzero(g["mfg_id"]) if "mfg_id" in g.columns else 0,
            "apple_type": _first_nonzero(g["apple_type"]) if "apple_type" in g.columns else 0,
            "apple_dev_type": _first_nonzero(g["apple_dev_type"]) if "apple_dev_type" in g.columns else 0,
            "le_mode": _first_nonzero(g["le_mode"]) if "le_mode" in g.columns else 0,
            "connectable": g["connectable"].any() if "connectable" in g.columns else True,
        })

    summary = df.groupby("mac").apply(agg, include_groups=False).reset_index()
    summary = summary.sort_values("hits", ascending=False).reset_index(drop=True)
    return summary


def build_presence_matrix(df: pd.DataFrame, bin_seconds: float = 5.0) -> pd.DataFrame:
    """Build a time × device presence matrix (RSSI values, NaN = absent)."""
    if df.empty:
        return pd.DataFrame()

    t_min = df["timestamp_s"].min()
    t_max = df["timestamp_s"].max()
    bins = np.arange(t_min, t_max + bin_seconds, bin_seconds)
    df = df.copy()
    df["time_bin"] = pd.cut(df["timestamp_s"], bins=bins, labels=bins[:-1])
    pivot = df.pivot_table(
        values="rssi", index="time_bin", columns="mac", aggfunc="median",
        observed=False,
    )
    pivot.index = pivot.index.astype(float)
    pivot.index.name = "time_s"
    return pivot


def compute_cooccurrence(df: pd.DataFrame, bin_seconds: float = 5.0) -> pd.DataFrame:
    """Compute device co-occurrence matrix (how often two devices appear in the same time bin)."""
    presence = build_presence_matrix(df, bin_seconds)
    if presence.empty:
        return pd.DataFrame()
    binary = (~presence.isna()).astype(int)
    co = binary.T.dot(binary).astype(float)
    totals = binary.sum()
    for i in co.columns:
        for j in co.columns:
            denom = np.sqrt(totals[i] * totals[j])
            co.loc[i, j] = co.loc[i, j] / denom if denom > 0 else 0.0
    return co
