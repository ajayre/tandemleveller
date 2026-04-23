"""Plot raw vs fused horizontal paths in UTM (WGS84 -> EPSG:32612) from CAN trace."""
import bisect
import os
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from pyproj import Transformer

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "Sensor Fusion Test 23.csv")
OUT_PATH = os.path.join(HERE, "Sensor_Fusion_Test23_raw_vs_fused_UTM.png")
OUT_ZOOM_PATH = os.path.join(
    HERE, "Sensor_Fusion_Test23_raw_vs_fused_UTM_zoom_n3552950_e516595.png"
)

# Zoom window for Test 23 only (fixed geographic clip).
ZOOM_TEST23 = {
    "type": "test23",
    "n_lo": 3552950.0,
    "n_hi": 3553000.0,
    "e_max": 516595.0,
}

# Raster output: 5× prior DPI → ~5× linear pixels per inch (25× pixels per unit area).
DPI_OVERVIEW = 750
DPI_ZOOM = 875

TRANSFORMER = Transformer.from_crs("EPSG:4326", "EPSG:32612", always_xy=True)


def lonlat_to_utm(lon, lat):
    e, n = TRANSFORMER.transform(lon, lat)
    return e, n


def nearest_imu_attitude(imu_samples, t_ms):
    """imu_samples: list of (t_ms, pitch, roll, heading_deg_or_none)."""
    ts = [s[0] for s in imu_samples]
    if not ts:
        return 0.0, 0.0, None
    i = bisect.bisect_left(ts, t_ms)
    cand = []
    if i > 0:
        cand.append(i - 1)
    if i < len(ts):
        cand.append(i)
    k = min(cand, key=lambda x: abs(ts[x] - t_ms))
    s = imu_samples[k]
    return s[1], s[2], s[3]


# Line weights (thin for readability on dense paths)
LW_OVERVIEW = 0.35
LW_ZOOM_RAW = 0.4
LW_ZOOM_FUSED = 0.45
RAW_COLOR = "#4a4a4a"
# Fused path: tilt vector in heading frame (pitch along heading, roll ⟂); α=0.5
ARROW_ALPHA = 0.5
# 4× denser sampling vs prior 12 m / 2 m
ARROW_SPACING_OVERVIEW_M = 3.0
ARROW_SPACING_ZOOM_M = 0.5
# Arrow shaft length in UTM metres = ARROW_M_PER_COMBINED_DEG * hypot(pitch_deg, roll_deg)
# (linear in combined tilt magnitude; direction = pitch along heading, roll ⟂ heading)
ARROW_M_PER_COMBINED_DEG = 0.25


def roll_abs_band_rgba(roll_deg):
    """Fused segment color from |roll| (deg): black / green / orange / red."""
    a = abs(roll_deg)
    if a < 1.0:
        return (0.0, 0.0, 0.0, 0.92)
    if a < 2.0:
        return (0.15, 0.62, 0.28, 0.92)
    if a < 3.0:
        return (0.90, 0.49, 0.13, 0.92)
    return (0.86, 0.18, 0.18, 0.92)


def fused_line_collection(e_list, n_list, roll_list, linewidth):
    """Build LineCollection for fused path; color each segment by midpoint |roll|."""
    if len(e_list) < 2:
        return None
    pts = np.column_stack([np.asarray(e_list, dtype=float), np.asarray(n_list, dtype=float)])
    segs = np.stack([pts[:-1], pts[1:]], axis=1)
    cols = []
    for i in range(len(e_list) - 1):
        rm = 0.5 * (roll_list[i] + roll_list[i + 1])
        cols.append(roll_abs_band_rgba(rm))
    lc = LineCollection(segs, colors=cols, linewidths=linewidth)
    return lc


def _indices_along_path_m(e, n, spacing_m):
    """Indices roughly every spacing_m along the polyline (easting, northing)."""
    e = np.asarray(e, dtype=float)
    n = np.asarray(n, dtype=float)
    m = len(e)
    if m == 0:
        return []
    if m == 1:
        return [0]
    de = np.diff(e)
    dn = np.diff(n)
    seg = np.hypot(de, dn)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])
    if total < 1e-9:
        return [0, m - 1]
    idxs = [0]
    t = float(spacing_m)
    while t < total:
        i = int(np.searchsorted(cum, t, side="left"))
        i = min(i, m - 1)
        if i != idxs[-1]:
            idxs.append(i)
        t += float(spacing_m)
    if idxs[-1] != m - 1:
        idxs.append(m - 1)
    return idxs


def add_pitch_roll_quiver(
    ax, e, n, pitch, roll, heading_deg, spacing_m, alpha=0.5, clip_bounds=None
):
    """
    Tilt vector in UTM: pitch along heading_deg, roll along heading_deg+90°.
    heading_deg must match the axis you want as “forward” on the map — we use
    fused-track course (atan2(de,dn)), not IMU heading, so pitch aligns with
    direction of travel on the map.
    Shaft length in metres = ARROW_M_PER_COMBINED_DEG * √(pitch²+roll²).
    clip_bounds: optional (e_lo, e_hi, n_lo, n_hi) to omit arrows outside zoom.
    """
    e = np.asarray(e, dtype=float)
    n = np.asarray(n, dtype=float)
    idxs = _indices_along_path_m(e, n, spacing_m)
    if clip_bounds is not None:
        e_lo, e_hi, n_lo, n_hi = clip_bounds
        idxs = [
            i
            for i in idxs
            if e_lo <= e[i] <= e_hi and n_lo <= n[i] <= n_hi
        ]
    if not idxs:
        return
    p = np.array([pitch[i] for i in idxs], dtype=float)
    r = np.array([roll[i] for i in idxs], dtype=float)
    h = np.radians(np.array([heading_deg[i] for i in idxs], dtype=float))
    sh = np.sin(h)
    ch = np.cos(h)
    k = ARROW_M_PER_COMBINED_DEG
    # Forward (along heading): (sin h, cos h); starboard (heading+90°): (cos h, -sin h)
    # hypot(u, v) == k * hypot(p, r) — length tracks combined |tilt| in °, direction from (p, r).
    u = k * (p * sh + r * ch)
    v = k * (p * ch - r * sh)
    ax.quiver(
        e[idxs],
        n[idxs],
        u,
        v,
        angles="xy",
        scale_units="xy",
        scale=1.0,
        color="#202020",
        alpha=alpha,
        width=0.0012,
        headwidth=3.2,
        headlength=4.0,
        headaxislength=3.5,
        zorder=6,
    )


def _auto_zoom_bounds(raw_e, raw_n, fus_e, fus_n, pad_frac=0.06):
    e_all = np.concatenate([np.asarray(raw_e, float), np.asarray(fus_e, float)])
    n_all = np.concatenate([np.asarray(raw_n, float), np.asarray(fus_n, float)])
    e_lo, e_hi = float(np.min(e_all)), float(np.max(e_all))
    n_lo, n_hi = float(np.min(n_all)), float(np.max(n_all))
    span = max(e_hi - e_lo, n_hi - n_lo, 1e-9)
    pad = max(0.5, pad_frac * span)
    return e_lo - pad, e_hi + pad, n_lo - pad, n_hi + pad


def plot_csv(
    csv_path,
    out_path,
    out_zoom_path,
    title_overview,
    footer_csv_name,
    zoom_cfg,
):
    re_row = re.compile(r'^"(\d+)","([0-9.]+)"')
    re_raw = re.compile(r"Raw Latitude:([0-9.]+), Raw Longitude:([-0-9.]+)")
    re_fused = re.compile(r"Latitude:([0-9.]+), Longitude:([-0-9.]+)")
    re_imu_full = re.compile(
        r"Tctr Heading:([-0-9.]+)deg,\s*Tctr Pitch:([-0-9.]+)deg,\s*Tctr Roll:([-0-9.]+)deg"
    )
    re_imu_legacy = re.compile(
        r"Tctr Pitch:([-0-9.]+)deg, Tctr Roll:([-0-9.]+)deg"
    )

    raw_ll = []
    fused_ll = []
    fused_times = []
    imu_samples = []

    cum_ms = 0.0
    with open(csv_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = re_row.match(line)
            if not m:
                continue
            cum_ms += float(m.group(2))

            if "0x182" in line and "Tctr Pitch" in line:
                mf = re_imu_full.search(line)
                if mf:
                    imu_samples.append(
                        (
                            cum_ms,
                            float(mf.group(2)),
                            float(mf.group(3)),
                            float(mf.group(1)),
                        )
                    )
                else:
                    ml = re_imu_legacy.search(line)
                    if ml:
                        imu_samples.append(
                            (
                                cum_ms,
                                float(ml.group(1)),
                                float(ml.group(2)),
                                None,
                            )
                        )

            if "0x2F1" in line and "Raw Latitude" in line:
                mr = re_raw.search(line)
                if mr:
                    raw_ll.append((float(mr.group(1)), float(mr.group(2))))

            if "0x381" in line and "Latitude:" in line and "Raw" not in line:
                mf = re_fused.search(line)
                if mf:
                    fused_ll.append((float(mf.group(1)), float(mf.group(2))))
                    fused_times.append(cum_ms)

    n = min(len(raw_ll), len(fused_ll), len(fused_times))
    raw_ll = raw_ll[:n]
    fused_ll = fused_ll[:n]
    fused_times = fused_times[:n]

    raw_e = []
    raw_n = []
    fus_e = []
    fus_n = []
    for i in range(n):
        la, lo = raw_ll[i]
        fla, flo = fused_ll[i]
        e, nn = lonlat_to_utm(lo, la)
        raw_e.append(e)
        raw_n.append(nn)
        fe, fn = lonlat_to_utm(flo, fla)
        fus_e.append(fe)
        fus_n.append(fn)

    fe_arr = np.asarray(fus_e, dtype=float)
    fn_arr = np.asarray(fus_n, dtype=float)
    de = np.gradient(fe_arr)
    dn = np.gradient(fn_arr)
    heading_from_track = np.degrees(np.arctan2(de, dn)) % 360.0

    pitch_at = []
    roll_at = []
    for i in range(n):
        p, r, _ = nearest_imu_attitude(imu_samples, fused_times[i])
        pitch_at.append(p)
        roll_at.append(r)

    # Course from fused path tangent (not Tctr heading): avoids pitch projecting
    # perpendicular to the track when magnetic/nose heading ≠ ground track.
    quiver_heading = heading_from_track.astype(float)

    fig, ax = plt.subplots(figsize=(10, 10), dpi=DPI_OVERVIEW)
    ax.plot(
        raw_e,
        raw_n,
        color=RAW_COLOR,
        linewidth=LW_OVERVIEW,
        alpha=0.9,
    )
    lc_full = fused_line_collection(fus_e, fus_n, roll_at, LW_OVERVIEW)
    if lc_full is not None:
        ax.add_collection(lc_full)
    add_pitch_roll_quiver(
        ax,
        fus_e,
        fus_n,
        pitch_at,
        roll_at,
        quiver_heading,
        ARROW_SPACING_OVERVIEW_M,
        alpha=ARROW_ALPHA,
    )
    ax.scatter([raw_e[0]], [raw_n[0]], c="#27ae60", s=36, zorder=5)
    ax.scatter([raw_e[-1]], [raw_n[-1]], c="#8e44ad", s=36, zorder=5)
    ax.scatter([fus_e[-1]], [fus_n[-1]], c="#f39c12", s=36, zorder=5, marker="s")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("UTM Easting (m) — WGS84 / EPSG:32612")
    ax.set_ylabel("UTM Northing (m) — WGS84 / EPSG:32612")
    ax.set_title(title_overview)
    ax.grid(True, linestyle=":", alpha=0.6)

    fig.text(
        0.5,
        0.02,
        "{} — {} paired fixes. Grey = raw GNSS; fused by |roll| "
        "(black<1°, green<2°, orange<3°, red≥3°). Arrows: pitch along fused-track course, roll perpendicular to track; "
        "shaft = {:.2f} m×√(pitch²+roll²) °.".format(
            footer_csv_name, n, ARROW_M_PER_COMBINED_DEG
        ),
        ha="center",
        fontsize=8,
        color="gray",
    )
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.06)
    fig.savefig(out_path, bbox_inches="tight")
    print("Wrote", out_path)

    if zoom_cfg["type"] == "test23":
        n_lo, n_hi = zoom_cfg["n_lo"], zoom_cfg["n_hi"]
        e_max = zoom_cfg["e_max"]

        def in_window(ne_list, ee_list):
            return [
                i
                for i in range(len(ee_list))
                if n_lo <= ne_list[i] <= n_hi and ee_list[i] < e_max
            ]

        idx_raw = set(in_window(raw_n, raw_e)) | set(in_window(fus_n, fus_e))
        if idx_raw:
            e_vals = [raw_e[i] for i in idx_raw] + [fus_e[i] for i in idx_raw]
            e_pad = 1.5
            e_lo = min(e_vals) - e_pad
            e_hi = min(e_max, max(e_vals) + e_pad)
        else:
            e_lo, e_hi = 516540.0, e_max
    else:
        pad_frac = zoom_cfg.get("pad_frac", 0.06)
        e_lo, e_hi, n_lo, n_hi = _auto_zoom_bounds(
            raw_e, raw_n, fus_e, fus_n, pad_frac=pad_frac
        )

        def in_window(ne_list, ee_list):
            return [
                i
                for i in range(len(ee_list))
                if n_lo <= ne_list[i] <= n_hi and e_lo <= ee_list[i] <= e_hi
            ]

        idx_raw = set(in_window(raw_n, raw_e)) | set(in_window(fus_n, fus_e))

    ds = os.path.splitext(footer_csv_name)[0]
    if zoom_cfg["type"] == "test23":
        zoom_ax_title = "Test 23 — zoom: N {:.0f}–{:.0f} m, E < {:.0f} m".format(
            n_lo, n_hi, zoom_cfg["e_max"]
        )
    else:
        zoom_ax_title = "{} — zoom: E {:.1f}–{:.1f} m, N {:.1f}–{:.1f} m".format(
            ds, e_lo, e_hi, n_lo, n_hi
        )

    win_idx = sorted(idx_raw)
    if not win_idx:
        win_idx = list(range(n))

    # Contiguous index runs inside zoom (for broken LineCollection)
    runs = []
    cur = [win_idx[0]]
    for j in range(1, len(win_idx)):
        if win_idx[j] == win_idx[j - 1] + 1:
            cur.append(win_idx[j])
        else:
            runs.append(cur)
            cur = [win_idx[j]]
    runs.append(cur)

    fig_z, ax_z = plt.subplots(figsize=(11, 8), dpi=DPI_ZOOM)
    ax_z.plot(
        raw_e,
        raw_n,
        color=RAW_COLOR,
        linewidth=LW_ZOOM_RAW,
        alpha=0.9,
    )

    for run in runs:
        if len(run) < 2:
            continue
        ee = [fus_e[i] for i in run]
        nn = [fus_n[i] for i in run]
        rr = [roll_at[i] for i in run]
        lc = fused_line_collection(ee, nn, rr, LW_ZOOM_FUSED)
        if lc is not None:
            ax_z.add_collection(lc)

    add_pitch_roll_quiver(
        ax_z,
        fus_e,
        fus_n,
        pitch_at,
        roll_at,
        quiver_heading,
        ARROW_SPACING_ZOOM_M,
        alpha=ARROW_ALPHA,
        clip_bounds=(e_lo, e_hi, n_lo, n_hi),
    )

    ax_z.set_aspect("equal", adjustable="box")
    ax_z.set_xlim(e_lo, e_hi)
    ax_z.set_ylim(n_lo, n_hi)
    ax_z.set_xlabel("UTM Easting (m) — EPSG:32612")
    ax_z.set_ylabel("UTM Northing (m) — EPSG:32612")
    ax_z.set_title(zoom_ax_title)
    ax_z.grid(True, linestyle=":", alpha=0.65)

    fig_z.text(
        0.5,
        0.02,
        "Grey = raw; fused by |roll| (black<1°, green<2°, orange<3°, red≥3°). "
        "Arrows: pitch parallel to fused track, roll perpendicular; shaft = {:.2f} m×√(pitch²+roll²) °.".format(
            ARROW_M_PER_COMBINED_DEG
        ),
        ha="center",
        fontsize=8,
        color="gray",
    )
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.10)
    fig_z.savefig(out_zoom_path, bbox_inches="tight")
    print("Wrote", out_zoom_path)


def main():
    csv_path = CSV_PATH
    out_path = OUT_PATH
    out_zoom_path = OUT_ZOOM_PATH
    title_overview = "Sensor Fusion Test 23 — Raw vs fused path (plan view)"
    footer_csv = "Sensor Fusion Test 23.csv"
    zoom_cfg = ZOOM_TEST23

    if len(sys.argv) >= 2:
        arg = sys.argv[1]
        csv_path = arg if os.path.isabs(arg) else os.path.join(HERE, arg)
        base = os.path.splitext(os.path.basename(csv_path))[0]
        safe = "".join(
            c if (c.isalnum() or c in " -_") else "_" for c in base
        ).strip()
        safe = "_".join(safe.split())
        out_path = os.path.join(HERE, "{}_raw_vs_fused_UTM.png".format(safe))
        out_zoom_path = os.path.join(HERE, "{}_raw_vs_fused_UTM_zoom.png".format(safe))
        title_overview = "{} — Raw vs fused path (plan view)".format(base)
        footer_csv = os.path.basename(csv_path)
        zoom_cfg = {"type": "auto", "pad_frac": 0.06}

    plot_csv(
        csv_path,
        out_path,
        out_zoom_path,
        title_overview,
        footer_csv,
        zoom_cfg,
    )


if __name__ == "__main__":
    main()
