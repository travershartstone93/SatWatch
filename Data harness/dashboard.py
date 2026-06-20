"""BLE Analyzer Dashboard — interactive visualization of scan data."""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from dash import Dash, Input, Output, callback, dcc, html
from plotly.subplots import make_subplots

from parse_ble import (
    build_device_summary,
    build_presence_matrix,
    compute_cooccurrence,
    estimate_distance,
    load_log,
)
from categorize import (
    categorize_device, get_category_icon, get_manufacturer_name, get_service_label,
    get_apple_device_label, get_apple_activity_label, get_oui_manufacturer,
    get_best_manufacturer, get_behavior_label, get_device_type_label,
)
from fetch import load_device_meta, load_names
from fingerprint import build_clustered_summary, is_random_mac
from slam import assign_path_positions, build_slam_map, segment_walk

DEVICE_NAMES: dict[str, str] = {}
DEVICE_META: dict[str, dict] = {}


DEVICE_IDS: dict[str, str] = {}

def display_name(mac: str) -> str:
    name = DEVICE_NAMES.get(mac)
    if name:
        return name
    did = DEVICE_IDS.get(mac)
    if did and did != mac:
        return did
    return mac[-8:]


SUMMARY_MFG: dict[str, int] = {}
SUMMARY_APPLE: dict[str, dict] = {}
SUMMARY_EXTRA: dict[str, dict] = {}

def device_category(mac: str) -> str:
    meta = DEVICE_META.get(mac, {})
    mfg = meta.get("mfgId", 0) or SUMMARY_MFG.get(mac, 0)
    apple = SUMMARY_APPLE.get(mac, {})
    extra = SUMMARY_EXTRA.get(mac, {})
    return categorize_device(
        name=DEVICE_NAMES.get(mac, ""),
        appearance=meta.get("appearance", 0),
        svc_uuid=meta.get("svcUuid", 0),
        connectable=extra.get("connectable", meta.get("connectable", True)),
        payload_len=extra.get("payload_len", 0) or meta.get("payloadLen", 0),
        tx_power=meta.get("txPower", 127),
        adv_type=extra.get("adv_type", 0),
        mfg_id=mfg,
        apple_dev_type=apple.get("dev_type", 0) or meta.get("appleDevType", 0),
        apple_msg_type=apple.get("msg_type", 0) or meta.get("appleType", 0),
        mac=mac,
        le_mode=extra.get("le_mode", 0),
    )


app = Dash(__name__)

DATA_DIR = Path(__file__).parent / "data"


def find_latest_log() -> Path | None:
    logs = sorted(DATA_DIR.glob("ble_scan*.bin"))
    if not logs:
        logs = sorted(Path(__file__).parent.glob("ble_scan*.bin"))
    return logs[-1] if logs else None


def _make_device_id_label(row) -> str:
    """Generate a descriptive device ID."""
    did = row.get("device_id", row["mac"]) if "device_id" in row.index else row["mac"]
    if not str(did).startswith("FP-"):
        return did
    cat = device_category(row["mac"])
    num = did.split("-")[1]
    cat_short = {
        "Phone": "Phone", "Audio": "Audio", "Wearable": "Watch",
        "Beacon": "Beacon", "Tracker": "Tag", "Computer": "PC",
        "Appliance": "Appl", "Network": "Net", "IoT": "IoT",
    }.get(cat, "Dev")
    return f"{cat_short} #{int(num)}"


def make_device_table(summary: pd.DataFrame) -> go.Figure:
    cols = ["mac", "hits", "rssi_mean", "rssi_min", "rssi_max",
            "dist_est_m", "duration_s", "tx_power", "payload_hash_hex"]
    display = summary[cols].copy()
    display["name"] = display["mac"].map(display_name)
    display["category"] = display["mac"].map(
        lambda m: f"{get_category_icon(device_category(m))} {device_category(m)}"
    )
    mfg_col = []
    for i, m in enumerate(display["mac"]):
        name = DEVICE_NAMES.get(m, "")
        mfg_name = get_best_manufacturer(
            mfg_id=DEVICE_META.get(m, {}).get("mfgId", 0) or SUMMARY_MFG.get(m, 0),
            mac=m,
            name=name,
        )
        mfg_col.append(mfg_name)
    display["manufacturer"] = mfg_col
    # Type: specific device type guess (e.g., "Air Conditioner", "iPhone", "Washing Machine")
    type_col = []
    for i, m in enumerate(display["mac"]):
        apple = SUMMARY_APPLE.get(m, {})
        meta = DEVICE_META.get(m, {})
        mfg_id = meta.get("mfgId", 0) or SUMMARY_MFG.get(m, 0)
        cat = device_category(m)
        label = get_device_type_label(
            name=DEVICE_NAMES.get(m, ""),
            manufacturer=mfg_col[i],
            apple_dev_type=apple.get("dev_type", 0) or meta.get("appleDevType", 0),
            apple_msg_type=apple.get("msg_type", 0) or meta.get("appleType", 0),
            category=cat,
            mfg_id=mfg_id,
        )
        type_col.append(label)
    display["type"] = type_col
    # Activity: Apple activity or behavioral label for unknowns
    activity_col = []
    for i, m in enumerate(display["mac"]):
        apple = SUMMARY_APPLE.get(m, {})
        meta = DEVICE_META.get(m, {})
        if apple.get("msg_type", 0) == 0x10 or meta.get("appleType", 0) == 0x10:
            activity_col.append(get_apple_activity_label(meta.get("appleStatus", 0)))
        elif device_category(m) == "Unknown":
            activity_col.append(get_behavior_label(
                hits=int(display["hits"].iloc[i]),
                duration_s=float(display["duration_s"].iloc[i]),
                rssi_mean=float(display["rssi_mean"].iloc[i]),
                dist_est_m=float(display["dist_est_m"].iloc[i]),
            ))
        else:
            activity_col.append("")
    display["activity"] = activity_col
    display["rssi_mean"] = display["rssi_mean"].round(1)
    display["dist_est_m"] = display["dist_est_m"].round(1)
    display["duration_s"] = display["duration_s"].round(1)
    # Show device_id for clustered/fingerprinted devices
    display["device_id"] = summary.apply(_make_device_id_label, axis=1).values
    if "cluster_size" in summary.columns:
        display["macs"] = summary["cluster_size"].apply(lambda n: f"{n} MACs" if n > 1 else "")
    else:
        display["macs"] = ""
    cols = ["name", "device_id", "category", "type", "manufacturer", "activity", "macs"] + cols

    fig = go.Figure(data=[go.Table(
        header=dict(
            values=["Name", "Device ID", "Category", "Type", "Manufacturer", "Activity", "MACs",
                    "MAC", "Hits", "RSSI (avg)", "Min", "Max",
                    "Est. Dist (m)", "Duration (s)", "TX Pwr", "Payload Hash"],
            fill_color="#1e293b",
            font=dict(color="white", size=12),
            align="left",
        ),
        cells=dict(
            values=[display[c] for c in cols],
            fill_color="#0f172a",
            font=dict(color="#e2e8f0", size=11),
            align="left",
        ),
    )])
    fig.update_layout(
        margin=dict(l=0, r=0, t=30, b=0),
        paper_bgcolor="#0f172a",
        height=min(400, 50 + len(display) * 28),
        title=dict(text=f"{len(display)} Devices", font=dict(color="#38bdf8", size=14)),
    )
    return fig


def make_category_chart(summary: pd.DataFrame) -> go.Figure:
    cats = summary["mac"].map(device_category)
    counts = cats.value_counts()
    labels = [f"{get_category_icon(c)} {c}" for c in counts.index]

    fig = go.Figure(data=[go.Pie(
        labels=labels, values=counts.values,
        hole=0.4,
        textinfo="label+value",
        textfont=dict(size=11),
        marker=dict(colors=px.colors.qualitative.Set2[:len(counts)]),
    )])
    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0f172a",
        plot_bgcolor="#1e293b",
        height=350,
        title=dict(text="Device Categories", font=dict(color="#38bdf8", size=14)),
        showlegend=False,
    )
    return fig


def make_rssi_timeline(df: pd.DataFrame, top_n: int = 6) -> go.Figure:
    top_macs = df.groupby("mac").size().nlargest(top_n).index
    filtered = df[df["mac"].isin(top_macs)]

    # Downsample per device to max 200 points for rendering performance
    parts = []
    for mac in top_macs:
        sub = filtered[filtered["mac"] == mac]
        if len(sub) > 200:
            sub = sub.iloc[:: len(sub) // 200]
        parts.append(sub)
    filtered = pd.concat(parts) if parts else filtered
    filtered = filtered.copy()
    filtered["device"] = filtered["mac"].map(display_name)

    fig = px.scatter(
        filtered, x="timestamp_s", y="rssi", color="device",
        opacity=0.5, size_max=4,
        labels={"timestamp_s": "Time (s)", "rssi": "RSSI (dBm)", "mac": "Device"},
    )
    fig.update_traces(marker=dict(size=3))
    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0f172a",
        plot_bgcolor="#1e293b",
        height=350,
        title=dict(text="RSSI Over Time", font=dict(color="#38bdf8", size=14)),
        legend=dict(font=dict(size=9)),
    )
    return fig


def make_presence_heatmap(df: pd.DataFrame, bin_s: float = 5.0) -> go.Figure:
    presence = build_presence_matrix(df, bin_s)
    if presence.empty:
        return go.Figure()

    # Sort columns by total presence (most present first)
    col_order = (~presence.isna()).sum().sort_values(ascending=False).index
    presence = presence[col_order]

    fig = go.Figure(data=go.Heatmap(
        z=presence.values.T,
        x=presence.index,
        y=presence.columns,
        colorscale="Viridis",
        colorbar=dict(title="RSSI"),
        zmin=-100, zmax=-40,
    ))
    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0f172a",
        plot_bgcolor="#1e293b",
        height=max(250, len(presence.columns) * 25 + 80),
        title=dict(text="Presence Heatmap", font=dict(color="#38bdf8", size=14)),
        xaxis_title="Time (s)",
        yaxis=dict(tickfont=dict(size=9)),
    )
    return fig


def make_distance_chart(summary: pd.DataFrame) -> go.Figure:
    s = summary.head(15).copy()
    s["label"] = s["mac"].map(display_name)
    s = s.sort_values("dist_est_m")

    colors = ["#4ade80" if d < 5 else "#fbbf24" if d < 15 else "#fb923c" if d < 30 else "#f87171"
              for d in s["dist_est_m"]]

    fig = go.Figure(go.Bar(
        y=s["label"], x=s["dist_est_m"],
        orientation="h",
        marker_color=colors,
        text=s["dist_est_m"].round(1).astype(str) + "m",
        textposition="outside",
    ))
    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0f172a",
        plot_bgcolor="#1e293b",
        height=max(200, len(s) * 28 + 60),
        title=dict(text="Estimated Distance", font=dict(color="#38bdf8", size=14)),
        xaxis_title="Distance (m)",
        yaxis=dict(tickfont=dict(size=10)),
    )
    return fig


def make_cooccurrence_heatmap(df: pd.DataFrame) -> go.Figure:
    co = compute_cooccurrence(df, bin_seconds=5.0)
    if co.empty:
        return go.Figure()

    labels = [display_name(m) for m in co.columns]
    fig = go.Figure(data=go.Heatmap(
        z=co.values,
        x=labels, y=labels,
        colorscale="Blues",
        zmin=0, zmax=1,
        colorbar=dict(title="Co-occ"),
    ))
    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0f172a",
        plot_bgcolor="#1e293b",
        height=max(300, len(labels) * 25 + 80),
        title=dict(text="Device Co-occurrence", font=dict(color="#38bdf8", size=14)),
        yaxis=dict(tickfont=dict(size=9)),
        xaxis=dict(tickfont=dict(size=9)),
    )
    return fig


def make_slam_map(df: pd.DataFrame) -> go.Figure:
    slam = build_slam_map(df, min_hits=5, min_positions=3)
    path_df = assign_path_positions(df)

    fig = go.Figure()

    # Receiver path
    fig.add_trace(go.Scatter(
        x=path_df["rx_x"], y=path_df["rx_y"],
        mode="lines",
        line=dict(color="#64748b", width=1),
        name="Walk path",
        opacity=0.4,
    ))

    if not slam.empty:
        # Color by distance
        colors = slam["dist_est_m"].values
        fig.add_trace(go.Scatter(
            x=slam["x"], y=slam["y"],
            mode="markers+text",
            marker=dict(
                size=np.clip(slam["hits"].values / 5, 8, 40),
                color=colors,
                colorscale="Turbo",
                colorbar=dict(title="Est. dist (m)"),
                line=dict(color="white", width=1),
            ),
            text=slam["mac"].map(display_name),
            textposition="top center",
            textfont=dict(size=9, color="#94a3b8"),
            name="Devices",
            hovertext=[
                f"MAC: {r.mac}<br>Hits: {r.hits}<br>RSSI: {r.rssi_mean:.0f}<br>"
                f"Est: {r.dist_est_m:.1f}m<br>Residual: {r.residual:.1f}"
                for _, r in slam.iterrows()
            ],
            hoverinfo="text",
        ))

    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0f172a",
        plot_bgcolor="#1e293b",
        height=450,
        title=dict(text="SLAM Map (estimated device positions)",
                   font=dict(color="#38bdf8", size=14)),
        xaxis_title="X (m)", yaxis_title="Y (m)",
        xaxis=dict(scaleanchor="y"),
    )
    return fig


def make_rssi_distribution(df: pd.DataFrame) -> go.Figure:
    fig = go.Figure()
    top_macs = df.groupby("mac").size().nlargest(6).index
    for mac in top_macs:
        vals = df[df["mac"] == mac]["rssi"]
        fig.add_trace(go.Violin(
            y=vals, name=display_name(mac),
            box_visible=True, meanline_visible=True,
        ))
    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#0f172a",
        plot_bgcolor="#1e293b",
        height=300,
        title=dict(text="RSSI Distribution by Device", font=dict(color="#38bdf8", size=14)),
        yaxis_title="RSSI (dBm)",
        showlegend=False,
    )
    return fig


# ── Layout ──────────────────────────────────────────────────────────────────

app.layout = html.Div(style={"backgroundColor": "#0f172a", "minHeight": "100vh", "padding": "20px"}, children=[
    html.H1("BLE Analyzer", style={"color": "#38bdf8", "textAlign": "center", "marginBottom": "5px"}),
    html.Div(id="subtitle", style={"color": "#64748b", "textAlign": "center", "marginBottom": "20px", "fontSize": "14px"}),

    html.Div(style={"display": "flex", "gap": "10px", "justifyContent": "center", "marginBottom": "20px"}, children=[
        dcc.Dropdown(
            id="file-select",
            style={"width": "400px", "backgroundColor": "#1e293b", "color": "#e2e8f0"},
            placeholder="Select log file...",
        ),
        html.Button("Refresh", id="refresh-btn",
                     style={"padding": "8px 16px", "backgroundColor": "#1e40af",
                            "color": "white", "border": "none", "borderRadius": "6px", "cursor": "pointer"}),
    ]),

    html.Div(id="stats-bar", style={
        "display": "flex", "justifyContent": "center", "gap": "30px",
        "marginBottom": "20px", "color": "#94a3b8", "fontSize": "14px",
    }),

    html.Div(style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "15px"}, children=[
        dcc.Graph(id="rssi-timeline"),
        dcc.Graph(id="category-chart"),
        dcc.Graph(id="distance-chart"),
        dcc.Graph(id="rssi-distribution"),
        dcc.Graph(id="presence-heatmap"),
        dcc.Graph(id="cooccurrence"),
        dcc.Graph(id="slam-map"),
    ]),

    dcc.Graph(id="device-table", style={"marginTop": "15px"}),
])


@callback(
    Output("file-select", "options"),
    Input("refresh-btn", "n_clicks"),
)
def update_file_list(_):
    logs = sorted(DATA_DIR.glob("ble_scan*.bin")) + sorted(Path(__file__).parent.glob("ble_scan*.bin"))
    seen = set()
    options = []
    for p in logs:
        if p.name not in seen:
            seen.add(p.name)
            size_kb = p.stat().st_size / 1024
            options.append({"label": f"{p.name} ({size_kb:.1f} KB)", "value": str(p)})
    return options


@callback(
    Output("subtitle", "children"),
    Output("stats-bar", "children"),
    Output("rssi-timeline", "figure"),
    Output("category-chart", "figure"),
    Output("distance-chart", "figure"),
    Output("rssi-distribution", "figure"),
    Output("presence-heatmap", "figure"),
    Output("cooccurrence", "figure"),
    Output("slam-map", "figure"),
    Output("device-table", "figure"),
    Input("file-select", "value"),
)
def update_dashboard(filepath):
    empty = go.Figure()
    empty.update_layout(template="plotly_dark", paper_bgcolor="#0f172a", plot_bgcolor="#1e293b")
    empties = [empty] * 8

    if not filepath:
        return "Select a log file above", [], *empties

    global DEVICE_NAMES, DEVICE_META
    DEVICE_NAMES = load_names()
    DEVICE_META = load_device_meta()

    try:
        df = load_log(filepath)
    except Exception as e:
        return f"Error: {e}", [], *empties

    if df.empty:
        return "No records in file", [], *empties

    raw_summary = build_device_summary(df)
    summary = build_clustered_summary(raw_summary)

    global SUMMARY_MFG, SUMMARY_APPLE, SUMMARY_EXTRA, DEVICE_IDS
    SUMMARY_MFG = {}
    SUMMARY_APPLE = {}
    SUMMARY_EXTRA = {}
    DEVICE_IDS = {}
    for _, row in summary.iterrows():
        mac = row["mac"]
        if row.get("mfg_id", 0):
            SUMMARY_MFG[mac] = int(row["mfg_id"])
        at = int(row.get("apple_type", 0))
        adt = int(row.get("apple_dev_type", 0))
        if at or adt:
            SUMMARY_APPLE[mac] = {"msg_type": at, "dev_type": adt}
        SUMMARY_EXTRA[mac] = {
            "le_mode": int(row.get("le_mode", 0)),
            "connectable": bool(row.get("connectable", True)),
            "payload_len": int(row.get("payload_len", 0)),
            "adv_type": int(row.get("adv_type", 0)),
        }
        if "device_id" in row and row["device_id"] != mac:
            DEVICE_IDS[mac] = row["device_id"]

    n_records = len(df)
    n_devices = len(summary)
    n_physical = len(summary["device_id"].unique()) if "device_id" in summary.columns else n_devices
    duration = df["timestamp_s"].max() - df["timestamp_s"].min()
    rate = n_records / duration if duration > 0 else 0

    subtitle = f"{Path(filepath).name}"
    dev_label = f"{n_physical} devices" if n_physical == n_devices else f"{n_physical} devices ({n_devices} MACs)"
    stats = [
        html.Span(f"{n_records:,} records"),
        html.Span(dev_label),
        html.Span(f"{duration:.0f}s duration"),
        html.Span(f"{rate:.1f} rec/s"),
    ]

    return (
        subtitle,
        stats,
        make_rssi_timeline(df),
        make_category_chart(summary),
        make_distance_chart(summary),
        make_rssi_distribution(df),
        make_presence_heatmap(df),
        make_cooccurrence_heatmap(df),
        make_slam_map(df),
        make_device_table(summary),
    )


def main():
    log = find_latest_log()
    if log:
        print(f"Latest log: {log}")
    else:
        print("No log files found. Use 'python fetch.py fetch' to download from watch.")
        print(f"Or place .bin files in {DATA_DIR}/")

    DATA_DIR.mkdir(exist_ok=True)
    print("Starting dashboard at http://127.0.0.1:8050")
    app.run(debug=True, host="127.0.0.1", port=8050)


if __name__ == "__main__":
    main()
