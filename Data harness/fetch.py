"""Fetch BLE scan data from the LiveSat watch."""

import subprocess
import sys
from pathlib import Path

DEFAULT_HOST = "satwatch.local"
TOKEN = "4c70cf"
DATA_DIR = Path(__file__).parent / "data"


def fetch_device_info(host: str = DEFAULT_HOST) -> list[dict]:
    """Fetch full device info from live summary (names, appearance, svcUuid)."""
    import json
    result = subprocess.run(
        ["curl", "-s", "--connect-timeout", "5", f"http://{host}/blejson"],
        capture_output=True, text=True,
    )
    if result.returncode != 0 or not result.stdout.strip():
        return []
    try:
        data = json.loads(result.stdout)
        return data.get("devices", [])
    except (json.JSONDecodeError, KeyError):
        return []


def fetch_names(host: str = DEFAULT_HOST) -> dict[str, str]:
    """Fetch MAC->name mapping from the live device summary."""
    devices = fetch_device_info(host)
    return {d["mac"]: d["name"] for d in devices if d.get("name")}


def save_names(names: dict[str, str], path: Path | None = None):
    """Save MAC->name mapping to a JSON file (accumulates across fetches)."""
    import json
    DATA_DIR.mkdir(exist_ok=True)
    if path is None:
        path = DATA_DIR / "device_names.json"
    existing = {}
    if path.exists():
        try:
            existing = json.loads(path.read_text())
        except json.JSONDecodeError:
            pass
    existing.update(names)
    path.write_text(json.dumps(existing, indent=2))
    return existing


def save_device_meta(devices: list[dict], path: Path | None = None):
    """Save per-device metadata (appearance, svcUuid, etc.) keyed by MAC."""
    import json
    DATA_DIR.mkdir(exist_ok=True)
    if path is None:
        path = DATA_DIR / "device_meta.json"
    existing = {}
    if path.exists():
        try:
            existing = json.loads(path.read_text())
        except json.JSONDecodeError:
            pass
    for d in devices:
        mac = d.get("mac", "")
        if not mac:
            continue
        entry = existing.get(mac, {})
        if d.get("name"):
            entry["name"] = d["name"]
        if d.get("appearance", 0):
            entry["appearance"] = d["appearance"]
        if d.get("svcUuid", 0):
            entry["svcUuid"] = d["svcUuid"]
        if d.get("mfgId", 0):
            entry["mfgId"] = d["mfgId"]
        entry["connectable"] = d.get("connectable", False)
        entry["payloadLen"] = d.get("payloadLen", 0)
        entry["txPower"] = d.get("txPower", 127)
        if d.get("appleType", 0):
            entry["appleType"] = d["appleType"]
        if d.get("appleDevType", 0):
            entry["appleDevType"] = d["appleDevType"]
        if d.get("appleStatus", 0):
            entry["appleStatus"] = d["appleStatus"]
        if d.get("airpodsBatL", 255) < 255:
            entry["airpodsBatL"] = d["airpodsBatL"]
        if d.get("airpodsBatR", 255) < 255:
            entry["airpodsBatR"] = d["airpodsBatR"]
        if d.get("airpodsBatC", 255) < 255:
            entry["airpodsBatC"] = d["airpodsBatC"]
        if d.get("hotspotBat", 255) < 255:
            entry["hotspotBat"] = d["hotspotBat"]
        existing[mac] = entry
    path.write_text(json.dumps(existing, indent=2))
    return existing


def load_device_meta(path: Path | None = None) -> dict[str, dict]:
    """Load saved per-device metadata."""
    import json
    if path is None:
        path = DATA_DIR / "device_meta.json"
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return {}


def load_names(path: Path | None = None) -> dict[str, str]:
    """Load saved MAC->name mapping."""
    import json
    if path is None:
        path = DATA_DIR / "device_names.json"
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return {}


def fetch_log(host: str = DEFAULT_HOST, output: Path | None = None) -> Path:
    DATA_DIR.mkdir(exist_ok=True)
    if output is None:
        existing = sorted(DATA_DIR.glob("ble_scan_*.bin"))
        idx = len(existing)
        output = DATA_DIR / f"ble_scan_{idx:03d}.bin"

    url = f"http://{host}/bledl"
    result = subprocess.run(
        ["curl", "-s", "--connect-timeout", "5", "-o", str(output), url],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"Error fetching: {result.stderr}", file=sys.stderr)
        sys.exit(1)

    size = output.stat().st_size
    if size < 16:
        content = output.read_bytes()
        print(f"No scan log available ({content.decode(errors='replace')})", file=sys.stderr)
        output.unlink()
        sys.exit(1)

    print(f"Downloaded {size} bytes -> {output}")

    # Also grab device info from live summary
    devices = fetch_device_info(host)
    if devices:
        names = {d["mac"]: d["name"] for d in devices if d.get("name")}
        if names:
            save_names(names)
        meta = save_device_meta(devices)
        print(f"Saved metadata for {len(meta)} devices")

    return output


def start_scan(host: str = DEFAULT_HOST) -> str:
    result = subprocess.run(
        ["curl", "-s", "--connect-timeout", "5",
         f"http://{host}/bletoggle?k={TOKEN}"],
        capture_output=True, text=True,
    )
    print(result.stdout.strip())
    return result.stdout.strip()


def stop_scan(host: str = DEFAULT_HOST) -> str:
    return start_scan(host)  # toggle


def get_status(host: str = DEFAULT_HOST) -> dict:
    import json
    result = subprocess.run(
        ["curl", "-s", "--connect-timeout", "5", f"http://{host}/blejson"],
        capture_output=True, text=True,
    )
    return json.loads(result.stdout)


def clear_logs(host: str = DEFAULT_HOST):
    result = subprocess.run(
        ["curl", "-s", "--connect-timeout", "5",
         f"http://{host}/bleclear?k={TOKEN}"],
        capture_output=True, text=True,
    )
    print(result.stdout.strip())


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="Fetch BLE data from LiveSat watch")
    p.add_argument("action", choices=["fetch", "start", "stop", "status", "clear"])
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("-o", "--output", type=Path, default=None)
    args = p.parse_args()

    if args.action == "fetch":
        fetch_log(args.host, args.output)
    elif args.action == "start":
        start_scan(args.host)
    elif args.action == "stop":
        stop_scan(args.host)
    elif args.action == "status":
        import json
        print(json.dumps(get_status(args.host), indent=2))
    elif args.action == "clear":
        clear_logs(args.host)
