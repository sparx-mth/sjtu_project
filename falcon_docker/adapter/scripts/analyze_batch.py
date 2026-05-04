#!/usr/bin/env python3
"""
analyze_batch.py — produce an HTML report from one batch_runner output dir.

Usage
═════
    python3 analyze_batch.py <batch_dir> [output_html]

Default output: <batch_dir>/report.html

The report is a single self-contained HTML file with interactive Plotly
charts (Plotly's JS is inlined, so the file works offline). Open it in
any browser.

What it shows
═════════════
1. Headline KPIs — successes, mean duration, mean voxels, mean speed.
2. Coverage curves — n_voxels vs t for every successful run, overlaid.
   This is THE chart for understanding mapping speed: shape tells you
   when discovery saturated, slope tells you discovery rate, the gap
   between curves tells you run-to-run variability.
3. Trajectory map — top-down (x, y) path per run, with the final voxel
   cloud (clipped to drone height) as a faint background.
4. Per-run metrics — bar chart side by side, easy at-a-glance compare.
5. Coverage milestones — t_50%, t_90%, t_99% per run. The gap between
   t_90 and t_99 is FALCON's "long tail" — small late-stage frontiers
   often dominate runtime in topologically complex maps.
6. Per-run table — the runs.csv content, sortable.
7. Aggregate stats — pretty-printed mean/std/min/max for each metric.

Requirements
════════════
    pip install numpy plotly      (the only external deps)

Run inside the FALCON container, or on the host machine — either is
fine, as long as it can read /home/falcon/runs/<batch_dir>/.
"""

import argparse
import glob
import json
import os
import sys
from html import escape

import numpy as np
import plotly.graph_objects as go
from plotly.io import to_html


# ─────────────────── Loaders ──────────────────────────────────────────────

def _load_csv(path, expected_cols):
    """numpy.loadtxt with header skip, returning None on any failure."""
    if not os.path.exists(path):
        return None
    try:
        data = np.loadtxt(path, delimiter=",", skiprows=1)
    except (ValueError, OSError):
        return None
    if data.size == 0:
        return None
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < expected_cols:
        return None
    return data


def load_run(run_dir):
    """Returns dict with keys present iff their file existed and parsed."""
    out = {"run_name": os.path.basename(run_dir)}

    sp = os.path.join(run_dir, "summary.json")
    if os.path.exists(sp):
        try:
            with open(sp) as f:
                out["summary"] = json.load(f)
        except (json.JSONDecodeError, OSError):
            pass

    cov = _load_csv(os.path.join(run_dir, "coverage.csv"), 2)
    if cov is not None:
        out["coverage"] = cov           # Nx2: t_sec, n_voxels

    traj = _load_csv(os.path.join(run_dir, "trajectory_gt.csv"), 5)
    if traj is not None:
        out["trajectory"] = traj         # Nx5: t, x, y, z, yaw

    vp = os.path.join(run_dir, "voxels.npy")
    if os.path.exists(vp):
        try:
            v = np.load(vp)
            if v.ndim == 2 and v.shape[1] >= 3 and v.shape[0] > 0:
                out["voxels"] = v   # full count; downsampling happens at viz time
        except (ValueError, OSError):
            pass

    # Optional FALCON layers — only present if recorder.py captured them
    # (added in v3 of run_recorder.py). Older batches won't have these
    # files; that's fine, the figure just renders fewer layers.
    for key, fname in [("free_voxels",     "free_voxels.npy"),
                       ("frontier_voxels", "frontier_voxels.npy")]:
        fp = os.path.join(run_dir, fname)
        if os.path.exists(fp):
            try:
                a = np.load(fp)
                if a.ndim == 2 and a.shape[1] >= 3 and a.shape[0] > 0:
                    out[key] = a
            except (ValueError, OSError):
                pass

    return out


# ─────────────────── Figures ──────────────────────────────────────────────

# Single-color palette per run — Plotly's default works fine, but we use
# a known palette so the same run gets the same color across all figures.
PALETTE = [
    "#2E86AB",  # steel blue
    "#E63946",  # warm red
    "#06A77D",  # green
    "#F4A261",  # orange
    "#8E5572",  # mauve
    "#264653",  # deep teal
    "#9D4EDD",  # purple
    "#FCBF49",  # yellow
    "#577590",  # slate
    "#43AA8B",  # teal-green
]


def _color_for(idx):
    return PALETTE[idx % len(PALETTE)]


def fig_coverage(runs):
    fig = go.Figure()
    for i, r in enumerate(runs):
        if "coverage" not in r:
            continue
        cov = r["coverage"]
        fig.add_trace(go.Scatter(
            x=cov[:, 0], y=cov[:, 1],
            mode="lines",
            name=r["run_name"],
            line=dict(color=_color_for(i), width=2),
            hovertemplate="t=%{x:.1f}s<br>voxels=%{y:.0f}<extra>%{fullData.name}</extra>",
        ))
    fig.update_layout(
        title=dict(text="Voxel discovery curve — one line per run", x=0.02),
        xaxis_title="time (s)",
        yaxis_title="voxels mapped",
        hovermode="x unified",
        margin=dict(l=60, r=20, t=50, b=50),
        height=420,
    )
    return fig


def fig_trajectory(runs):
    fig = go.Figure()

    # Background: voxel cloud from the first run with voxels, projected
    # to 2D and clipped to roughly drone height so floor/ceiling don't
    # drown out the walls.
    bg_added = False
    for r in runs:
        if "voxels" not in r:
            continue
        v = r["voxels"]
        if v.shape[1] >= 3:
            mask = (v[:, 2] >= 0.3) & (v[:, 2] <= 1.8)
            v = v[mask]
        if v.shape[0] > 0:
            fig.add_trace(go.Scatter(
                x=v[:, 0], y=v[:, 1],
                mode="markers",
                marker=dict(size=2, color="#cccccc", opacity=0.6),
                name="voxels (background)",
                hoverinfo="skip",
                showlegend=True,
            ))
            bg_added = True
            break

    for i, r in enumerate(runs):
        if "trajectory" not in r:
            continue
        t = r["trajectory"]
        color = _color_for(i)
        # Path
        fig.add_trace(go.Scatter(
            x=t[:, 1], y=t[:, 2],
            mode="lines",
            name=r["run_name"],
            line=dict(color=color, width=2),
            hovertemplate=(
                "x=%{x:.2f} y=%{y:.2f}<br>t=%{customdata:.1f}s"
                "<extra>%{fullData.name}</extra>"
            ),
            customdata=t[:, 0],
        ))
        # Start marker
        fig.add_trace(go.Scatter(
            x=[t[0, 1]], y=[t[0, 2]],
            mode="markers",
            marker=dict(size=11, symbol="circle",
                        color=color, line=dict(color="white", width=2)),
            showlegend=False,
            hovertemplate=f"{r['run_name']} START<extra></extra>",
        ))
        # End marker
        fig.add_trace(go.Scatter(
            x=[t[-1, 1]], y=[t[-1, 2]],
            mode="markers",
            marker=dict(size=11, symbol="x", color=color,
                        line=dict(color="white", width=1)),
            showlegend=False,
            hovertemplate=f"{r['run_name']} END<extra></extra>",
        ))

    fig.update_layout(
        title=dict(text="Drone trajectories (top-down)" +
                        ("  ·  voxel map shaded" if bg_added else ""),
                   x=0.02),
        xaxis_title="x (m)",
        yaxis_title="y (m)",
        hovermode="closest",
        margin=dict(l=60, r=20, t=50, b=50),
        height=520,
    )
    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    return fig


def fig_metrics_bars(runs):
    """Side-by-side bars for duration, path length, voxels — normalized
    via three subplots-as-rows so the y-axes don't fight each other."""
    names, durations, paths, voxels, vps, vpm, spd = [], [], [], [], [], [], []
    for r in runs:
        s = r.get("summary")
        if not s:
            continue
        names.append(r["run_name"])
        d = s.get("duration_sec", 0.0)
        p = s.get("path_length_m", 0.0)
        v = s.get("final_voxels", 0)
        durations.append(d)
        paths.append(p)
        voxels.append(v)
        vps.append(s.get("avg_discovery_voxels_per_sec", 0.0))
        vpm.append(v / p if p > 1e-3 else 0.0)
        spd.append(p / d if d > 1e-3 else 0.0)

    fig = go.Figure()
    metrics = [
        ("duration (s)",       durations,  "#2E86AB"),
        ("path length (m)",    paths,      "#06A77D"),
        ("final voxels (×1k)", [v/1000 for v in voxels], "#F4A261"),
        ("voxels/sec",         vps,        "#9D4EDD"),
        ("voxels/meter",       vpm,        "#8E5572"),
        ("avg speed (m/s)",    spd,        "#E63946"),
    ]
    # Use grouped bars: x = run name, color = metric. But mixed scales
    # make the bars unreadable. Better: a small-multiples-style figure
    # where each metric is its own trace and visibility is toggleable.
    for label, values, color in metrics:
        fig.add_trace(go.Bar(
            x=names, y=values, name=label,
            marker_color=color,
            hovertemplate=("%{x}<br>" + label + "=%{y:.2f}<extra></extra>"),
        ))
    fig.update_layout(
        title=dict(text="Per-run metrics — click legend to toggle",
                   x=0.02),
        barmode="group",
        margin=dict(l=60, r=20, t=50, b=50),
        height=420,
        yaxis_title="value (mixed units — see legend)",
    )
    return fig


def _voxel_grid_downsample(points, target_count):
    """
    Voxel-grid downsampling: bin every voxel onto a coarser grid, take
    unique resulting positions. Unlike random sampling, this PRESERVES
    STRUCTURE — a wall stays a continuous wall instead of becoming a
    sparse scatter of unrelated points. Returns (binned_points, bin_size)
    where bin_size is the effective render resolution.

    The bin size is chosen as an integer multiple of the native voxel
    size so bins align cleanly with the original grid.
    """
    if points is None or points.shape[0] == 0:
        return None, 0.1

    native_size = _estimate_voxel_size(points)

    # Already small enough — render at native resolution
    if points.shape[0] <= target_count:
        return points.astype(np.float32), native_size

    # Volume scales with bin_size^3 — doubling bin size cuts count ~8×.
    # Pick the smallest integer multiplier that gets us under target.
    ratio = points.shape[0] / target_count
    bin_multiplier = max(1, int(round(ratio ** (1.0 / 3.0))))
    bin_size = native_size * bin_multiplier

    binned = np.round(points / bin_size) * bin_size
    unique_binned = np.unique(binned, axis=0)

    # Pathological case: very large environment → still too many bins.
    # Fall back to random over the structure-preserved set.
    if unique_binned.shape[0] > target_count * 1.5:
        rng = np.random.default_rng(42)
        idx = rng.choice(unique_binned.shape[0], target_count, replace=False)
        unique_binned = unique_binned[idx]

    return unique_binned.astype(np.float32), float(bin_size)


def merged_voxel_cloud(runs, key="voxels", max_points=None):
    """
    Concatenate `runs[i][key]` arrays from all runs and downsample to
    something Mesh3d can render. Returns (Nx3 points, render_voxel_size)
    or (None, native_size). `key` selects the layer: "voxels" (occupied),
    "free_voxels", or "frontier_voxels".

    Uses voxel-grid downsampling — the cubes you see are actual binned
    voxels, not random samples — so walls look like walls.
    """
    if max_points is None:
        max_points = int(os.environ.get("VOXEL_MAX_CUBES", "10000"))
    arrays = [r[key] for r in runs if key in r and r[key].size]
    if not arrays:
        return None, 0.1
    merged = np.vstack(arrays)
    merged = np.unique(np.round(merged, 4), axis=0)
    return _voxel_grid_downsample(merged, target_count=max_points)


def _estimate_voxel_size(points, default=0.1):
    """
    FALCON publishes voxel CENTRES on a regular grid. Recover the grid
    spacing from the data: smallest non-trivial diff between sorted
    unique x-coordinates of a sample.
    """
    if points is None or len(points) < 10:
        return default
    rng = np.random.default_rng(0)
    if len(points) > 3000:
        idx = rng.choice(len(points), 3000, replace=False)
        sample = points[idx]
    else:
        sample = points
    ux = np.unique(np.round(sample[:, 0], 4))
    if len(ux) < 2:
        return default
    diffs = np.diff(ux)
    pos = diffs[diffs > 0.02]
    if len(pos) == 0:
        return default
    return float(np.min(pos))


def _build_cube_mesh(centres, voxel_size, color_by_z=True,
                     fixed_color=None, opacity=1.0, name="layer"):
    """
    Build a single Mesh3d trace where each row of `centres` becomes one
    cube of edge `voxel_size * 0.96`.

    If color_by_z, cubes are tinted by their centre z (rainbow → matches
    FALCON's RViz default). If fixed_color is set, use that solid colour
    for every cube instead.
    """
    if centres is None or centres.shape[0] == 0:
        return None
    h = voxel_size * 0.48
    n = centres.shape[0]

    offsets = np.array([
        [-h, -h, -h], [+h, -h, -h], [+h, +h, -h], [-h, +h, -h],
        [-h, -h, +h], [+h, -h, +h], [+h, +h, +h], [-h, +h, +h],
    ], dtype=np.float32)
    cube_tris = np.array([
        [0, 2, 1], [0, 3, 2],   # bottom
        [4, 5, 6], [4, 6, 7],   # top
        [0, 1, 5], [0, 5, 4],   # front
        [2, 3, 7], [2, 7, 6],   # back
        [1, 2, 6], [1, 6, 5],   # right
        [0, 4, 7], [0, 7, 3],   # left
    ], dtype=np.int32)

    verts = (centres[:, None, :].astype(np.float32) +
             offsets[None, :, :]).reshape(-1, 3)
    base = (np.arange(n, dtype=np.int32)[:, None, None] * 8)
    tris = (base + cube_tris[None, :, :]).reshape(-1, 3)

    kwargs = dict(
        x=verts[:, 0], y=verts[:, 1], z=verts[:, 2],
        i=tris[:, 0], j=tris[:, 1], k=tris[:, 2],
        flatshading=True,
        # Flatter lighting than the previous version — closer to RViz,
        # which doesn't add specular highlights to occupancy cubes.
        lighting=dict(ambient=0.75, diffuse=0.35, specular=0.0,
                      roughness=1.0, fresnel=0.0),
        lightposition=dict(x=200, y=200, z=400),
        opacity=opacity,
        name=name,
        hoverinfo="skip",
    )
    if color_by_z:
        kwargs.update(
            intensity=np.repeat(centres[:, 2], 8),
            intensitymode="vertex",
            # FALCON's RViz uses an HSV/rainbow ramp by Z (red low, blue
            # high). Plotly's "Rainbow" matches that.
            colorscale="Rainbow",
            colorbar=dict(title="z (m)", thickness=12, len=0.7),
            showscale=True,
        )
    else:
        kwargs.update(color=fixed_color, showscale=False)
    return go.Mesh3d(**kwargs)


def _wireframe_bbox(xmin, xmax, ymin, ymax, zmin, zmax,
                    name="map bounds", color="#888"):
    """A wireframe box made from 12 line segments — gives RViz-like map
       extents indication."""
    cx = [xmin, xmax, xmax, xmin, xmin,
          xmin, xmax, xmax, xmin, xmin,
          None, xmax, xmax, None, xmax, xmax, None, xmin, xmin]
    cy = [ymin, ymin, ymax, ymax, ymin,
          ymin, ymin, ymax, ymax, ymin,
          None, ymin, ymin, None, ymax, ymax, None, ymax, ymax]
    cz = [zmin, zmin, zmin, zmin, zmin,
          zmax, zmax, zmax, zmax, zmax,
          None, zmin, zmax, None, zmin, zmax, None, zmin, zmax]
    return go.Scatter3d(
        x=cx, y=cy, z=cz,
        mode="lines",
        line=dict(color=color, width=2),
        name=name,
        hoverinfo="skip",
    )


def fig_voxel_3d(runs):
    """
    RViz-style voxel map. Renders up to three layers when present:
      - OCCUPIED (rainbow by Z, opaque)        — /sdf_map/occupancy_all
      - FREE     (light cyan, semi-transparent) — /sdf_map/free
      - FRONTIER (bright yellow, opaque)        — /planning_vis/frontier
    Plus a wireframe box showing the spatial extent of the map.

    Cubes are voxel-grid-downsampled (not random-sampled), so walls
    look like walls — every cube you see corresponds to a real binned
    region of voxels, never a sparse scatter. Default budget is 10000
    cubes per layer (override via VOXEL_MAX_CUBES env var).

    Use the legend toggles to show/hide each layer like RViz's display
    tree. Free space is hidden by default — toggle it on if you want
    to see explored empty volume.
    """
    occ, occ_size = merged_voxel_cloud(runs, "voxels")
    free, free_size = merged_voxel_cloud(
        runs, "free_voxels",
        max_points=int(os.environ.get("VOXEL_MAX_CUBES_FREE", "5000")))
    frontier, front_size = merged_voxel_cloud(
        runs, "frontier_voxels",
        max_points=int(os.environ.get("VOXEL_MAX_CUBES_FRONTIER", "3000")))

    fig = go.Figure()
    if occ is None:
        fig.update_layout(
            title=dict(text="Voxel map — no voxel data", x=0.02),
            height=560,
        )
        return fig

    # OCCUPIED — rainbow cubes, opaque, rendered at the (possibly
    # coarsened) bin size from voxel-grid downsampling.
    occ_trace = _build_cube_mesh(
        occ, occ_size, color_by_z=True,
        opacity=1.0, name="occupied")
    fig.add_trace(occ_trace)

    if free is not None and free.shape[0] > 0:
        free_trace = _build_cube_mesh(
            free, free_size, color_by_z=False,
            fixed_color="#7ec8e3", opacity=0.18, name="free")
        if free_trace is not None:
            free_trace.visible = "legendonly"
            fig.add_trace(free_trace)

    if frontier is not None and frontier.shape[0] > 0:
        # Frontiers slightly bigger than their natural size to stand out
        front_trace = _build_cube_mesh(
            frontier, front_size * 1.15, color_by_z=False,
            fixed_color="#ffd400", opacity=1.0, name="frontier")
        if front_trace is not None:
            fig.add_trace(front_trace)

    # Wireframe bounding box around the map extent
    pad = occ_size * 0.5
    xs = [a[:, 0] for a in (occ, free, frontier) if a is not None and a.size]
    ys = [a[:, 1] for a in (occ, free, frontier) if a is not None and a.size]
    zs = [a[:, 2] for a in (occ, free, frontier) if a is not None and a.size]
    if xs:
        all_x = np.concatenate(xs); all_y = np.concatenate(ys); all_z = np.concatenate(zs)
        fig.add_trace(_wireframe_bbox(
            float(all_x.min()) - pad, float(all_x.max()) + pad,
            float(all_y.min()) - pad, float(all_y.max()) + pad,
            float(all_z.min()) - pad, float(all_z.max()) + pad,
        ))

    # Total raw voxels (pre-downsample) and native resolution for
    # transparency in the title — so the user sees both the real map
    # and how aggressively we had to bin it for plotly.
    raw_arrays = [r["voxels"] for r in runs
                  if "voxels" in r and r["voxels"].size]
    if raw_arrays:
        raw_merged = np.unique(np.round(np.vstack(raw_arrays), 4), axis=0)
        raw_total = raw_merged.shape[0]
        native_size = _estimate_voxel_size(raw_merged)
    else:
        raw_total = occ.shape[0]
        native_size = occ_size

    coarsened = abs(occ_size - native_size) > 1e-4
    if coarsened:
        ds_str = (f"{raw_total} voxels @ {native_size:.2f} m → "
                  f"{occ.shape[0]} cubes @ {occ_size:.2f} m (voxel-grid binned)")
    else:
        ds_str = f"{occ.shape[0]} occupied @ {occ_size:.2f} m"

    extras = []
    if free is not None and free.shape[0] > 0:
        extras.append(f"{free.shape[0]} free (toggle in legend)")
    if frontier is not None and frontier.shape[0] > 0:
        extras.append(f"{frontier.shape[0]} frontier")
    extra_str = "  ·  " + "  ·  ".join(extras) if extras else ""

    fig.update_layout(
        title=dict(
            text=f"Voxel map (RViz-style) — {ds_str}{extra_str}  ·  drag to rotate",
            x=0.02,
        ),
        scene=dict(
            xaxis_title="x (m)",
            yaxis_title="y (m)",
            zaxis_title="z (m)",
            aspectmode="data",
            bgcolor="#222",
            # Default camera angle: a bit elevated, looking at the room
            # from a 45° angle. Better than plotly's default for showing
            # 3D structure rather than near-top-down.
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.0)),
        ),
        margin=dict(l=0, r=0, t=50, b=0),
        height=600,
        legend=dict(orientation="h", x=0.02, y=-0.05),
    )
    return fig


def fig_area_bars(runs):
    """Per-run area-based metrics — the headline numbers in m² terms."""
    names, swept, m2ps, spm2, mpm2 = [], [], [], [], []
    for r in runs:
        if "_swept_area_m2" not in r:
            continue
        s = r.get("summary") or {}
        d = float(s.get("duration_sec", 0.0))
        p = float(s.get("path_length_m", 0.0))
        a = float(r["_swept_area_m2"])
        if a < 1e-3:
            continue
        names.append(r["run_name"])
        swept.append(a)
        m2ps.append(a / d if d > 1e-3 else 0.0)
        spm2.append(d / a)
        mpm2.append(p / a if a > 1e-3 else 0.0)

    fig = go.Figure()
    for label, values, color in [
        ("swept area (m²)",       swept, "#2E86AB"),
        ("m² / sec",              m2ps,  "#06A77D"),
        ("sec / m²",              spm2,  "#F4A261"),
        ("m flown / m² mapped",   mpm2,  "#E63946"),
    ]:
        fig.add_trace(go.Bar(
            x=names, y=values, name=label, marker_color=color,
            hovertemplate=("%{x}<br>" + label + " = %{y:.3f}<extra></extra>"),
        ))
    fig.update_layout(
        title=dict(text="Per-run area metrics — click legend to toggle scales",
                   x=0.02),
        barmode="group",
        margin=dict(l=60, r=20, t=50, b=50),
        height=380,
        yaxis_title="value (mixed units — see legend)",
    )
    return fig


def fig_milestones(runs):
    """For each run, when did coverage first hit 50%/90%/99% of its final?"""
    names, t50, t90, t99, t100 = [], [], [], [], []
    for r in runs:
        if "coverage" not in r:
            continue
        cov = r["coverage"]
        if cov.shape[0] == 0:
            continue
        names.append(r["run_name"])
        final = cov[-1, 1]
        if final <= 0:
            t50.append(0); t90.append(0); t99.append(0); t100.append(cov[-1, 0])
            continue

        def first_at(frac):
            thr = frac * final
            mask = cov[:, 1] >= thr
            if not mask.any():
                return cov[-1, 0]
            return float(cov[mask, 0][0])

        t50.append(first_at(0.50))
        t90.append(first_at(0.90))
        t99.append(first_at(0.99))
        t100.append(cov[-1, 0])

    fig = go.Figure()
    for label, vals, color in [
        ("t at 50% coverage",  t50,  "#06A77D"),
        ("t at 90% coverage",  t90,  "#F4A261"),
        ("t at 99% coverage",  t99,  "#E63946"),
        ("total duration",     t100, "#264653"),
    ]:
        fig.add_trace(go.Bar(
            x=names, y=vals, name=label, marker_color=color,
            hovertemplate=("%{x}<br>" + label + " = %{y:.1f}s<extra></extra>"),
        ))
    fig.update_layout(
        title=dict(text="Coverage milestones — gap between 90% and 100% is the long tail",
                   x=0.02),
        barmode="group",
        margin=dict(l=60, r=20, t=50, b=50),
        height=380,
        yaxis_title="time (s)",
    )
    return fig


# ─────────────────── HTML assembly ────────────────────────────────────────

CSS = """
* { box-sizing: border-box; }
body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto,
                 "Helvetica Neue", Arial, sans-serif;
    color: #1a1a1a;
    background: #fafaf7;
    margin: 0;
    padding: 0 32px 64px;
    line-height: 1.5;
}
.header {
    padding: 28px 0 12px;
    border-bottom: 1px solid #e5e5e0;
    margin-bottom: 28px;
}
.header h1 {
    font-size: 22px;
    margin: 0 0 4px;
    color: #1a1a1a;
}
.header .subtitle {
    color: #777;
    font-size: 13px;
    font-family: ui-monospace, "SF Mono", Menlo, Consolas, monospace;
}
.kpis {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
    gap: 14px;
    margin: 24px 0;
}
.kpi {
    background: white;
    border: 1px solid #e5e5e0;
    border-radius: 8px;
    padding: 16px 18px;
}
.kpi .label {
    font-size: 11px;
    color: #888;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    margin-bottom: 4px;
}
.kpi .value {
    font-size: 22px;
    font-weight: 600;
    color: #1a1a1a;
}
.kpi .sub {
    font-size: 11px;
    color: #999;
    margin-top: 2px;
    font-family: ui-monospace, monospace;
}
.section {
    background: white;
    border: 1px solid #e5e5e0;
    border-radius: 8px;
    padding: 8px 8px 4px;
    margin-bottom: 24px;
}
.section .desc {
    padding: 0 12px 8px;
    font-size: 13px;
    color: #555;
}
h2 {
    font-size: 14px;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: #555;
    margin: 32px 0 12px;
}
table {
    width: 100%;
    border-collapse: collapse;
    font-size: 13px;
    font-family: ui-monospace, "SF Mono", Menlo, Consolas, monospace;
}
table th, table td {
    text-align: right;
    padding: 8px 12px;
    border-bottom: 1px solid #f0f0eb;
}
table th {
    background: #f5f5f0;
    font-weight: 600;
    color: #555;
    text-transform: uppercase;
    font-size: 11px;
    letter-spacing: 0.04em;
    border-bottom: 1px solid #e5e5e0;
}
table th:first-child, table td:first-child {
    text-align: left;
}
.failures {
    background: #fff5f5;
    border-color: #f5d5d5;
}
.failures h2 {
    color: #c44;
}
.aggregate-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    gap: 12px;
    padding: 8px;
    font-family: ui-monospace, "SF Mono", Menlo, Consolas, monospace;
    font-size: 12px;
}
.agg-card {
    background: #fafaf7;
    border-radius: 6px;
    padding: 10px 12px;
}
.agg-card .key {
    color: #777;
    font-weight: 600;
    margin-bottom: 4px;
}
.agg-card .stats {
    color: #333;
}
.env-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    gap: 10px;
    padding: 12px;
}
.env-fact {
    background: #fafaf7;
    border-radius: 6px;
    padding: 10px 14px;
}
.env-fact .key {
    font-size: 11px;
    color: #888;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    margin-bottom: 3px;
}
.env-fact .val {
    font-size: 15px;
    font-weight: 600;
    color: #1a1a1a;
    font-family: ui-monospace, "SF Mono", Menlo, Consolas, monospace;
}
"""


def _kpi(label, value, sub=""):
    return f"""
    <div class="kpi">
        <div class="label">{escape(label)}</div>
        <div class="value">{escape(str(value))}</div>
        <div class="sub">{escape(str(sub))}</div>
    </div>
    """


def _table_from_runs(runs):
    """Build an HTML table of headline metrics from each run's summary."""
    rows = []
    headers = ["run", "duration (s)", "path (m)", "swept area (m²)",
               "m² / sec", "sec / m²", "m / m²", "voxels", "vox / m²"]
    for r in runs:
        s = r.get("summary")
        if not s:
            continue
        d = s.get("duration_sec", 0.0)
        p = s.get("path_length_m", 0.0)
        v = s.get("final_voxels", 0)
        a = float(r.get("_swept_area_m2", 0.0))
        m2ps = (a / d) if d > 1e-3 and a > 1e-3 else 0.0
        spm2 = (d / a) if a > 1e-3 else 0.0
        mpm2 = (p / a) if a > 1e-3 else 0.0
        vpm2 = (v / a) if a > 1e-3 else 0.0
        rows.append([
            r["run_name"],
            f"{d:.1f}", f"{p:.2f}", f"{a:.1f}",
            f"{m2ps:.3f}", f"{spm2:.2f}", f"{mpm2:.2f}",
            f"{v}", f"{vpm2:.0f}",
        ])

    if not rows:
        return "<p style='padding:12px;color:#888'>No runs with summaries.</p>"

    th = "".join(f"<th>{escape(h)}</th>" for h in headers)
    body_rows = []
    for r in rows:
        tds = "".join(f"<td>{escape(c)}</td>" for c in r)
        body_rows.append(f"<tr>{tds}</tr>")
    return f"""
    <table>
        <thead><tr>{th}</tr></thead>
        <tbody>{''.join(body_rows)}</tbody>
    </table>
    """


# Default radius (m) for "swept area" — area within this distance of
# the trajectory is considered well-mapped. Smaller than the depth
# camera's 5 m max range, so it's a conservative estimate.
SWEPT_RADIUS_M = float(os.environ.get("SWEPT_RADIUS_M", "2.0"))
SWEPT_GRID_CELL_M = 0.5


def _swept_area_m2(traj_xy, swept_radius=SWEPT_RADIUS_M, cell_size=SWEPT_GRID_CELL_M):
    """
    Approximate the area the drone "explored" — the union of disks of
    radius `swept_radius` around every trajectory point. Rasterised
    onto a 2D `cell_size`-grid; cells whose centres fall within
    `swept_radius` of any trajectory point are counted as mapped.
    Returns area in m². Pure numpy.
    """
    if traj_xy is None or len(traj_xy) == 0:
        return 0.0

    xs = traj_xy[:, 0]
    ys = traj_xy[:, 1]

    # Downsample trajectory to one waypoint per swept_radius/2 of motion
    keep = [0]
    step2 = (swept_radius * 0.5) ** 2
    for i in range(1, len(traj_xy)):
        dx = xs[i] - xs[keep[-1]]
        dy = ys[i] - ys[keep[-1]]
        if dx * dx + dy * dy >= step2:
            keep.append(i)
    if keep[-1] != len(traj_xy) - 1:
        keep.append(len(traj_xy) - 1)
    pxs = xs[keep]
    pys = ys[keep]

    xmin = float(pxs.min() - swept_radius)
    xmax = float(pxs.max() + swept_radius)
    ymin = float(pys.min() - swept_radius)
    ymax = float(pys.max() + swept_radius)
    nx = int(np.ceil((xmax - xmin) / cell_size))
    ny = int(np.ceil((ymax - ymin) / cell_size))
    if nx <= 0 or ny <= 0:
        return 0.0

    gx = xmin + (np.arange(nx) + 0.5) * cell_size
    gy = ymin + (np.arange(ny) + 0.5) * cell_size

    grid = np.zeros((ny, nx), dtype=bool)
    r2 = swept_radius ** 2
    r_cells = int(np.ceil(swept_radius / cell_size))

    for x, y in zip(pxs, pys):
        cxi = int((x - xmin) / cell_size)
        cyi = int((y - ymin) / cell_size)
        x0 = max(0, cxi - r_cells)
        x1 = min(nx, cxi + r_cells + 1)
        y0 = max(0, cyi - r_cells)
        y1 = min(ny, cyi + r_cells + 1)
        sub_x = gx[x0:x1]
        sub_y = gy[y0:y1]
        dx = sub_x - x
        dy = sub_y - y
        d2 = dx[None, :] ** 2 + dy[:, None] ** 2
        grid[y0:y1, x0:x1] |= (d2 <= r2)

    return float(grid.sum() * cell_size * cell_size)


def _bbox_area_m2(traj_xy):
    """Width × height of the (x, y) bounding box of the trajectory."""
    if traj_xy is None or len(traj_xy) == 0:
        return 0.0
    xs, ys = traj_xy[:, 0], traj_xy[:, 1]
    return float((xs.max() - xs.min()) * (ys.max() - ys.min()))


def compute_aggregate_from_runs(runs, failures):
    """
    Build an aggregate dict identical in shape to batch_runner's
    aggregate.json, but computed from the per-run summary.json +
    coverage.csv + trajectory_gt.csv files so the report works even
    when batch_runner didn't get to write aggregate.json.
    """
    durations, paths, voxels, vps, vpm, spd = [], [], [], [], [], []
    t50, t90, t99 = [], [], []
    swept_areas, bbox_areas = [], []
    m2_per_sec, sec_per_m2, m_per_m2, vox_per_m2 = [], [], [], []

    for r in runs:
        s = r.get("summary")
        d = p = 0.0
        v = 0
        if s:
            d = float(s.get("duration_sec", 0.0))
            p = float(s.get("path_length_m", 0.0))
            v = int(s.get("final_voxels", 0))
            a = float(s.get("avg_discovery_voxels_per_sec", 0.0))
            durations.append(d)
            paths.append(p)
            voxels.append(v)
            vps.append(a)
            vpm.append((v / p) if p > 1e-3 else 0.0)
            spd.append((p / d) if d > 1e-3 else 0.0)

        cov = r.get("coverage")
        if cov is not None and cov.shape[0] > 0 and cov[-1, 1] > 0:
            final = cov[-1, 1]
            for thr_frac, lst in [(0.50, t50), (0.90, t90), (0.99, t99)]:
                mask = cov[:, 1] >= thr_frac * final
                if mask.any():
                    lst.append(float(cov[mask, 0][0]))

        traj = r.get("trajectory")
        if traj is not None and traj.shape[0] > 0:
            txy = traj[:, 1:3]
            swept = _swept_area_m2(txy)
            bbox = _bbox_area_m2(txy)
            r["_swept_area_m2"] = swept
            r["_bbox_area_m2"] = bbox
            if swept > 1e-3:
                swept_areas.append(swept)
                bbox_areas.append(bbox)
                if d > 1e-3:
                    m2_per_sec.append(swept / d)
                    sec_per_m2.append(d / swept)
                if p > 1e-3:
                    m_per_m2.append(p / swept)
                if v > 0:
                    vox_per_m2.append(v / swept)

    def st(xs):
        import statistics as _s
        if not xs:
            return {"mean": 0.0, "std": 0.0, "min": 0.0, "max": 0.0, "n": 0}
        return {
            "mean": float(_s.mean(xs)),
            "std":  float(_s.stdev(xs)) if len(xs) > 1 else 0.0,
            "min":  float(min(xs)),
            "max":  float(max(xs)),
            "n":    len(xs),
        }

    n_succ = len([r for r in runs if "summary" in r])
    n_fail = len(failures or [])
    n_total = n_succ + n_fail
    return {
        "n_successes":   n_succ,
        "n_failures":    n_fail,
        "n_attempts":    n_total,
        "success_rate":  (n_succ / n_total) if n_total else 0.0,
        "swept_radius_m": SWEPT_RADIUS_M,
        "metrics": {
            "duration_sec":         st(durations),
            "path_length_m":        st(paths),
            "final_voxels":         st(voxels),
            "voxels_per_sec":       st(vps),
            "voxels_per_meter":     st(vpm),
            "avg_speed_m_per_sec":  st(spd),
            "swept_area_m2":        st(swept_areas),
            "bbox_area_m2":         st(bbox_areas),
            "m2_per_sec":           st(m2_per_sec),
            "sec_per_m2":           st(sec_per_m2),
            "m_per_m2":             st(m_per_m2),
            "voxels_per_m2":        st(vox_per_m2),
            "t_50pct_coverage_sec": st(t50),
            "t_90pct_coverage_sec": st(t90),
            "t_99pct_coverage_sec": st(t99),
        },
    }


def _aggregate_cards(aggregate):
    metrics = aggregate.get("metrics", {})
    if not metrics:
        return "<p style='padding:12px;color:#888'>No metrics could be computed (no runs with summary.json).</p>"

    cards = []
    pretty = {
        # Area-based: the headline metrics
        "swept_area_m2":        "swept area (m²)",
        "m2_per_sec":           "m² mapped / sec",
        "sec_per_m2":           "sec / m²",
        "m_per_m2":             "m flown / m² mapped",
        "voxels_per_m2":        "voxels / m²",
        "bbox_area_m2":         "trajectory bbox (m²)",
        # Time / distance
        "duration_sec":         "duration (s)",
        "path_length_m":        "path length (m)",
        "avg_speed_m_per_sec":  "avg speed (m/s)",
        # Coverage milestones
        "t_50pct_coverage_sec": "t at 50% coverage (s)",
        "t_90pct_coverage_sec": "t at 90% coverage (s)",
        "t_99pct_coverage_sec": "t at 99% coverage (s)",
        # Voxel-based
        "final_voxels":         "final voxels",
        "voxels_per_sec":       "voxels / sec",
        "voxels_per_meter":     "voxels / meter",
    }
    for key, label in pretty.items():
        st = metrics.get(key)
        if not st or st.get("n", 0) == 0:
            continue
        cards.append(f"""
        <div class="agg-card">
            <div class="key">{escape(label)}</div>
            <div class="stats">
                mean&nbsp;{st['mean']:.2f}&nbsp;·&nbsp;std&nbsp;{st['std']:.2f}<br>
                min&nbsp;{st['min']:.2f}&nbsp;·&nbsp;max&nbsp;{st['max']:.2f}&nbsp;(n={st['n']})
            </div>
        </div>
        """)
    return f"""<div class="aggregate-grid">{''.join(cards)}</div>"""


def _failures_section(failures):
    if not failures:
        return ""
    rows = "".join(
        f"<tr><td>{escape(f.get('run_name','?'))}</td>"
        f"<td>{escape(f.get('outcome','?'))}</td>"
        f"<td>{f.get('attempt_index','?')}</td></tr>"
        for f in failures
    )
    return f"""
    <h2>Failures</h2>
    <div class="section failures">
        <div class="desc">
            {len(failures)} attempt(s) discarded (timeout or crash).
            Their directories were deleted; only run names + outcomes are kept.
        </div>
        <table>
            <thead><tr><th>run name</th><th>outcome</th><th>attempt #</th></tr></thead>
            <tbody>{rows}</tbody>
        </table>
    </div>
    """


def build_report(batch_dir, output_html=None):
    if output_html is None:
        output_html = os.path.join(batch_dir, "report.html")

    # Load aggregate (may be missing if user is analyzing a partial batch)
    aggregate = {}
    agg_path = os.path.join(batch_dir, "aggregate.json")
    if os.path.exists(agg_path):
        try:
            with open(agg_path) as f:
                aggregate = json.load(f)
        except (json.JSONDecodeError, OSError):
            pass

    # Load failures
    failures = []
    fails_path = os.path.join(batch_dir, "failures.json")
    if os.path.exists(fails_path):
        try:
            with open(fails_path) as f:
                failures = json.load(f)
        except (json.JSONDecodeError, OSError):
            pass

    # Load all run subdirectories
    run_dirs = sorted(d for d in glob.glob(os.path.join(batch_dir, "run_*"))
                      if os.path.isdir(d))
    runs = [load_run(d) for d in run_dirs]

    # Always compute aggregate from per-run data; this works even when
    # batch_runner didn't get to write aggregate.json (e.g. user
    # interrupted the batch). If aggregate.json existed and was loaded
    # above, we override with our recomputation — same numbers, but
    # avoids stale-file headaches.
    aggregate = compute_aggregate_from_runs(runs, failures)

    # Identifying header info
    batch_id = os.path.basename(os.path.normpath(batch_dir))

    # Build figures (each as an inline div). Inline plotly.js into the
    # FIRST figure only — subsequent figures reuse the same JS bundle.
    fig_html_parts = []
    for i, fig in enumerate([
            fig_voxel_3d(runs),       # NEW: env preview at top
            fig_coverage(runs),
            fig_trajectory(runs),
            fig_area_bars(runs),      # NEW: area-based per-run metrics
            fig_metrics_bars(runs),
            fig_milestones(runs),
    ]):
        fig_html_parts.append(to_html(
            fig,
            full_html=False,
            include_plotlyjs="inline" if i == 0 else False,
            config={"displaylogo": False},
        ))

    # KPI cards
    n_succ = aggregate.get("n_successes", len([r for r in runs if "summary" in r]))
    n_fail = aggregate.get("n_failures", len(failures))
    n_total = n_succ + n_fail
    success_rate = aggregate.get("success_rate", n_succ / max(1, n_total))
    metrics = aggregate.get("metrics", {})

    def _stat(key, fmt="{:.1f}"):
        s = metrics.get(key)
        if not s or s.get("n", 0) == 0:
            return "—"
        return fmt.format(s["mean"])

    def _stat_sub(key, fmt="±{:.1f}"):
        s = metrics.get(key)
        if not s or s.get("n", 0) == 0:
            return ""
        return fmt.format(s["std"])

    # KPIs are now AREA-FIRST. m²/sec and sec/m² are the headline numbers
    # for "how fast does FALCON map" — voxel-based numbers are still in
    # the aggregate cards section below for reference.
    kpis_html = "".join([
        _kpi("successes", f"{n_succ} / {n_total}",
             f"success rate {success_rate:.0%}"),
        _kpi("avg duration", _stat("duration_sec", "{:.0f} s"),
             _stat_sub("duration_sec", "± {:.0f} s")),
        _kpi("avg swept area", _stat("swept_area_m2", "{:.1f} m²"),
             _stat_sub("swept_area_m2", "± {:.1f} m²")),
        _kpi("avg m² / sec", _stat("m2_per_sec", "{:.3f}"),
             _stat_sub("m2_per_sec", "± {:.3f}")),
        _kpi("avg sec / m²", _stat("sec_per_m2", "{:.2f}"),
             _stat_sub("sec_per_m2", "± {:.2f}")),
        _kpi("avg m flown / m²", _stat("m_per_m2", "{:.2f}"),
             _stat_sub("m_per_m2", "± {:.2f}")),
        _kpi("avg path length", _stat("path_length_m", "{:.1f} m"),
             _stat_sub("path_length_m", "± {:.1f} m")),
        _kpi("avg voxels / m²", _stat("voxels_per_m2", "{:.0f}"),
             _stat_sub("voxels_per_m2", "± {:.0f}")),
    ])

    # Environment dry-fact cards — sit next to the 3D preview.
    # Try to extract the env name from the batch_id ("hospital_batch_..." → "hospital").
    env_name = batch_id.split("_batch_")[0] if "_batch_" in batch_id else batch_id

    # Total points across all merged voxel clouds (PRE-downsample) — gives a
    # truer "how big is the map" number than the 15k-cap visualization.
    total_voxels = 0
    for r in runs:
        if "voxels" in r and r["voxels"].size > 0:
            total_voxels += r["voxels"].shape[0]

    # Environment extent: bbox of all trajectories combined
    all_xy = []
    for r in runs:
        traj = r.get("trajectory")
        if traj is not None and traj.shape[0] > 0:
            all_xy.append(traj[:, 1:3])
    bbox_str = "—"
    if all_xy:
        all_xy_arr = np.vstack(all_xy)
        x_extent = float(all_xy_arr[:, 0].max() - all_xy_arr[:, 0].min())
        y_extent = float(all_xy_arr[:, 1].max() - all_xy_arr[:, 1].min())
        bbox_str = f"{x_extent:.1f} × {y_extent:.1f} m"

    swept_radius = aggregate.get("swept_radius_m", SWEPT_RADIUS_M)

    # Auto-detect voxel resolution from the raw voxel data (not the
    # downsampled cloud — we want the NATIVE resolution as a dry fact,
    # independent of how many cubes the figure ends up rendering).
    raw_for_size = []
    for r in runs:
        if "voxels" in r and r["voxels"].size:
            raw_for_size.append(r["voxels"])
    voxel_res_str = "—"
    if raw_for_size:
        merged_raw = np.vstack(raw_for_size)
        voxel_res_str = f"{_estimate_voxel_size(merged_raw):.2f} m"

    env_facts_html = f"""
    <div class="env-grid">
        <div class="env-fact"><div class="key">map name</div><div class="val">{escape(env_name)}</div></div>
        <div class="env-fact"><div class="key">runs</div><div class="val">{n_succ} successful, {n_fail} failed</div></div>
        <div class="env-fact"><div class="key">trajectory bbox (combined)</div><div class="val">{bbox_str}</div></div>
        <div class="env-fact"><div class="key">avg swept area / run</div><div class="val">{_stat('swept_area_m2', '{:.1f}')} m² {_stat_sub('swept_area_m2', '(± {:.1f})')}</div></div>
        <div class="env-fact"><div class="key">total voxels (sum)</div><div class="val">{total_voxels:,}</div></div>
        <div class="env-fact"><div class="key">voxel resolution</div><div class="val">{voxel_res_str}</div></div>
        <div class="env-fact"><div class="key">swept radius (used to compute m²)</div><div class="val">{swept_radius:.1f} m</div></div>
    </div>
    """

    # Page assembly
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>FALCON batch report — {escape(batch_id)}</title>
<style>{CSS}</style>
</head>
<body>

<div class="header">
    <h1>FALCON batch report — {escape(env_name)}</h1>
    <div class="subtitle">{escape(batch_id)}</div>
</div>

<h2>Environment</h2>
<div class="section">
    <div class="desc">
        Dry facts about this batch and the map FALCON produced. The
        voxel map below is the union of voxels from all successful runs,
        deduplicated and rendered as cubes at FALCON's actual map
        resolution — what you'd see in RViz with the occupancy display.
        Drag to rotate, scroll to zoom.
    </div>
    {env_facts_html}
    {fig_html_parts[0]}
</div>

<div class="kpis">
    {kpis_html}
</div>

<h2>Voxel discovery curve</h2>
<div class="section">
    <div class="desc">
        How fast voxels were added to the map over time. Steep early
        slope = mapping scan + first room rapidly filled in. Plateau =
        exploration done. Gaps between curves show how repeatable each
        environment is.
    </div>
    {fig_html_parts[1]}
</div>

<h2>Trajectories (top-down)</h2>
<div class="section">
    <div class="desc">
        The path the drone flew (ground truth). Circle = start, ✕ = end.
        Faint gray dots show the final voxel cloud projected to 2D
        (clipped to drone height) — the "shape" of the environment.
    </div>
    {fig_html_parts[2]}
</div>

<h2>Per-run area metrics</h2>
<div class="section">
    <div class="desc">
        Headline area numbers per run. <b>m² / sec</b> is mapping speed,
        <b>sec / m²</b> is its inverse, <b>m flown / m² mapped</b> is path
        efficiency (lower is better — less zig-zag per area discovered).
        Swept area = area within {swept_radius:.1f} m of the trajectory,
        so it's a conservative estimate of confidently-mapped floor area.
    </div>
    {fig_html_parts[3]}
</div>

<h2>Per-run voxel-based metrics</h2>
<div class="section">
    <div class="desc">
        For reference. Voxel counts depend on map resolution and
        environment clutter, so use these for sanity-checking trends
        rather than cross-environment comparisons.
    </div>
    {fig_html_parts[4]}
</div>

<h2>Coverage milestones</h2>
<div class="section">
    <div class="desc">
        Wall-clock time at which each run first hit 50% / 90% / 99% of
        its final voxel count. The gap between t at 99% and total
        duration is the time spent on the "long tail" of small
        late-stage frontiers — often disproportionately long.
    </div>
    {fig_html_parts[5]}
</div>

<h2>Per-run table</h2>
<div class="section">
    {_table_from_runs(runs)}
</div>

<h2>Aggregate statistics</h2>
<div class="section">
    {_aggregate_cards(aggregate)}
</div>

{_failures_section(failures)}

</body>
</html>
"""

    with open(output_html, "w") as f:
        f.write(html)
    return output_html


# ─────────────────── Main ─────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Generate an HTML report from a FALCON batch directory.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("batch_dir",
                   help="Path to a batch directory "
                        "(e.g. /home/falcon/runs/playground_batch_20260504_103836)")
    p.add_argument("output_html", nargs="?", default=None,
                   help="Output HTML path. Default: <batch_dir>/report.html")
    args = p.parse_args()

    if not os.path.isdir(args.batch_dir):
        sys.stderr.write(f"error: {args.batch_dir} is not a directory\n")
        sys.exit(2)

    out = build_report(args.batch_dir, args.output_html)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()