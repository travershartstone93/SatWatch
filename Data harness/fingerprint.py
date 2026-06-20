"""BLE device fingerprinting — track devices across MAC address rotation."""

import pandas as pd
import numpy as np


def is_random_mac(mac: str) -> bool:
    """Check if MAC is locally administered (random/rotated)."""
    first_byte = int(mac.split(":")[0], 16)
    return bool(first_byte & 0x02)


def device_fingerprint(row) -> str:
    """Build a fingerprint string from advertisement characteristics.

    Devices that rotate MACs still keep consistent:
    - Manufacturer ID (mfg_id)
    - Payload length
    - Advertisement type
    - Payload hash (often stable for long periods)
    """
    mfg = int(row.get("mfg_id", 0))
    plen = int(row.get("payload_len", 0))
    atype = int(row.get("adv_type", 0))
    apple_type = int(row.get("apple_type", 0))
    apple_dev = int(row.get("apple_dev_type", 0))
    return f"{mfg:04x}:{plen:02d}:{atype}:{apple_type:02x}:{apple_dev}"


def cluster_devices(summary: pd.DataFrame) -> pd.DataFrame:
    """Assign stable device IDs by clustering rotated MACs.

    Non-random MACs keep their MAC as ID.
    Random MACs with matching fingerprints are grouped into clusters.
    Each cluster gets a synthetic ID like 'FP-001'.
    """
    summary = summary.copy()
    summary["is_random"] = summary["mac"].apply(is_random_mac)
    summary["fingerprint"] = summary.apply(device_fingerprint, axis=1)

    # Non-random MACs: use OUI prefix as part of identity
    summary["device_id"] = summary["mac"]

    # Group random MACs by fingerprint
    random_devs = summary[summary["is_random"]].copy()
    if random_devs.empty:
        summary["cluster_size"] = 1
        return summary

    # Cluster by fingerprint — devices with same (mfg, payload_len, adv_type, apple)
    # are likely the same physical device rotating its MAC
    fp_groups = random_devs.groupby("fingerprint")

    cluster_id = 1
    cluster_map = {}
    cluster_sizes = {}

    for fp, group in fp_groups:
        if fp == "0000:00:0:00:0":
            continue  # skip totally empty fingerprints, can't cluster
        if len(group) == 1:
            continue  # single device, no clustering needed

        # Within a fingerprint group, further split by timing overlap
        # Two devices with the same fingerprint seen at the same time = different devices
        sub_clusters = _split_by_timing(group)
        for sc in sub_clusters:
            cid = f"FP-{cluster_id:03d}"
            for mac in sc:
                cluster_map[mac] = cid
                cluster_sizes[mac] = len(sc)
            cluster_id += 1

    for mac, cid in cluster_map.items():
        mask = summary["mac"] == mac
        summary.loc[mask, "device_id"] = cid
        summary.loc[mask, "cluster_size"] = cluster_sizes.get(mac, 1)

    summary["cluster_size"] = summary.get("cluster_size", pd.Series(1, index=summary.index)).fillna(1).astype(int)
    return summary


def _split_by_timing(group: pd.DataFrame) -> list[list[str]]:
    """Split a fingerprint group into sub-clusters based on temporal overlap.

    If two MACs are seen in overlapping time windows, they're different devices.
    If they're seen sequentially (no overlap), they're likely the same device.
    """
    intervals = []
    for _, row in group.iterrows():
        intervals.append({
            "mac": row["mac"],
            "start": row.get("first_seen_s", 0),
            "end": row.get("last_seen_s", 0),
        })

    # Sort by start time
    intervals.sort(key=lambda x: x["start"])

    clusters = []
    for iv in intervals:
        placed = False
        for cluster in clusters:
            # Check if this interval overlaps with any in the cluster
            overlaps = False
            for existing in cluster:
                if iv["start"] < existing["end"] and iv["end"] > existing["start"]:
                    overlaps = True
                    break
            if not overlaps:
                cluster.append(iv)
                placed = True
                break
        if not placed:
            clusters.append([iv])

    return [[iv["mac"] for iv in c] for c in clusters]


def build_clustered_summary(summary: pd.DataFrame) -> pd.DataFrame:
    """Build a summary that merges clustered devices into single entries."""
    clustered = cluster_devices(summary)

    # For clustered devices, merge their stats
    groups = []
    for device_id, g in clustered.groupby("device_id"):
        groups.append({
            "mac": g["mac"].iloc[0],
            "all_macs": ", ".join(g["mac"].tolist()),
            "device_id": device_id,
            "hits": g["hits"].sum(),
            "rssi_mean": np.average(g["rssi_mean"], weights=g["hits"]),
            "rssi_min": g["rssi_min"].min(),
            "rssi_max": g["rssi_max"].max(),
            "rssi_std": g["rssi_std"].mean() if "rssi_std" in g.columns else 0,
            "first_seen_s": g["first_seen_s"].min(),
            "last_seen_s": g["last_seen_s"].max(),
            "duration_s": g["last_seen_s"].max() - g["first_seen_s"].min(),
            "dist_est_m": np.average(g["dist_est_m"], weights=g["hits"]),
            "mfg_id": g["mfg_id"].max(),
            "apple_type": g["apple_type"].max() if "apple_type" in g.columns else 0,
            "apple_dev_type": g["apple_dev_type"].max() if "apple_dev_type" in g.columns else 0,
            "tx_power": g["tx_power"].iloc[0],
            "payload_hash_hex": g["payload_hash_hex"].iloc[0],
            "payload_len": g["payload_len"].iloc[0],
            "adv_type": g["adv_type"].iloc[0],
            "is_random": g["is_random"].any(),
            "le_mode": g["le_mode"].max() if "le_mode" in g.columns else 0,
            "connectable": g["connectable"].any() if "connectable" in g.columns else True,
            "cluster_size": len(g),
            "fingerprint": g["fingerprint"].iloc[0],
        })

    merged = pd.DataFrame(groups)
    merged = merged.sort_values("hits", ascending=False).reset_index(drop=True)
    return merged
