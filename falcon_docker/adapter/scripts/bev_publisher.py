#!/usr/bin/env python3
"""
bev_publisher.py — FALCON voxel map → de-noised 2D BEV (nav_msgs/OccupancyGrid).

v4 — height-weighted column projection + 3D neighbour reasoning
═════════════════════════════════════════════════════════════════════════════
WHAT FALCON GIVES US (and why this node looks the way it does)
  FALCON's `voxel_mapping` is a probabilistic log-odds occupancy grid (the same
  family as FUEL). It fuses every depth frame over time, clamps, and thresholds.
  It does NOT publish the probabilities — it publishes the *thresholded* result
  as two point clouds of voxel CENTRES:
      /voxel_mapping/occupancy_grid_occupied   (PointCloud2, XYZ)  -> "occupied"
      /voxel_mapping/occupancy_grid_free        (PointCloud2, XYZ)  -> "known free"
  Voxels never observed appear in neither cloud  -> "unknown".
  Because the map is already temporally fused, our job is purely SPATIAL:
  collapse the 3D voxels into a clean 2D grid the A* planner can use.

  The noise you see ("one stray voxel == an occupied cell") is FALCON's
  occupancy threshold letting through a few bad voxels from the DA3 monocular
  depth (3 Hz, no real range sensor). The fix is to stop trusting any single
  voxel and instead reason about the whole vertical column + 3D neighbourhood.

THE PIPELINE (every stage is a private rosparam; set a gate to its disabling
value to isolate one idea, or run them all together):

  1. COLUMN PROJECTION  (the core de-noiser)
     For every (x,y) cell, look at all occupied voxels stacked above it.
       • occ_count  = how many occupied voxels are in the column
       • occ_weight = sum of per-voxel height weights w(z)
     w(z) peaks at the drone's flight height (`z_peak`, ~1 m) and decays to 0 at
     the floor (`z_floor`) and ceiling (`z_ceil`) — so a voxel at flight height
     counts ~1.0, a voxel near the floor/ceiling counts ~0. A cell is occupied
     only if  occ_weight >= occ_weight_thresh  AND  occ_count >= min_occ_voxels.
     => a lone floating voxel can never create a wall.

  2. 3D NEIGHBOUR CONFIRM  (kills DA3 floaters)
     Build the sparse 3D voxel volume and drop any occupied voxel with fewer
     than `min_occ_neighbors_3d` occupied neighbours (6/18/26-connectivity).
     Real surfaces are locally dense; monocular-depth speckle is isolated.

  3. DOOR / WINDOW PROTECTION  (never wall an opening that is open at flight ht)
     If a cell is occupied from the column sum BUT the band around `z_peak` is
     clearly FREE (>= `door_free_voxels` free voxels, <= `door_occ_tol` occ
     voxels there), it's a doorway/window — force it FREE. This is checked
     before, and protected from, wall completion + dilation.

  4. WALL COMPLETION  (fill FOV / occlusion gaps in continuous walls)
     2D, neighbour-aware, and it ONLY fills UNKNOWN cells (never observed-free,
     never a protected opening). "directional" mode fills a cell only if it
     bridges occupied cells on two opposite sides (L&R, U&D, or a diagonal) —
     this closes a one-cell hole in a wall line but cannot flood an open room
     (open cells have no opposite-side support). `wall_fill_iters` widens the
     gap it can bridge; keep it small (1–2) so completion can't snowball.

  5. TEMPORAL HYSTERESIS  (optional; off by default — FALCON already fuses)
     Per-cell evidence accumulator with on/off thresholds to de-flicker.

  6. MANUAL WALLS / BACK-WALL / DILATION / AUTO-BOUNDS  (kept from v3)

RUNTIME
  Heaviest op is the 3D neighbour count (~a few ms at 0.15 m / typical indoor
  bbox with 6-connectivity). The pipeline is rebuilt only when a new cloud
  arrives (depth ~3 Hz) and the grid is (re)published at `publish_hz` (10 Hz),
  latched. Set `always_recompute:=true` to force the full pipeline every tick.

TOPICS / MSG TYPES ARE UNCHANGED from v2/v3 — drop-in replacement.
Defaults already improve the map with the existing launch; expose the new args
(see the README block at the bottom of this file) to tune per map.
"""

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid

UNK, FREE, OCC = -1, 0, 100
_F32 = PointField.FLOAT32  # == 7


# ─────────────────────────── cloud parsing ──────────────────────────────────
def cloud_xyz(msg):
    """Fast (x,y,z) float32 extraction -> (N,3) float32, NaNs dropped.
    Uses a zero-copy structured view (≈100× faster than read_points for big
    clouds); falls back to pc2.read_points for exotic layouts."""
    try:
        f = {p.name: p for p in msg.fields}
        if (not msg.is_bigendian
                and all(k in f for k in ("x", "y", "z"))
                and all(f[k].datatype == _F32 for k in ("x", "y", "z"))):
            dt = np.dtype({"names": ["x", "y", "z"],
                           "formats": [np.float32, np.float32, np.float32],
                           "offsets": [f["x"].offset, f["y"].offset,
                                       f["z"].offset],
                           "itemsize": msg.point_step})
            n = msg.width * msg.height
            a = np.frombuffer(msg.data, dt, count=n)
            xyz = np.stack((a["x"], a["y"], a["z"]), axis=1)
        else:
            raise ValueError("non-float32/bigendian cloud")
    except Exception:
        xyz = np.array(list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)), np.float32)
        if xyz.size == 0:
            return np.empty((0, 3), np.float32)
    if xyz.size == 0:
        return np.empty((0, 3), np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float32, copy=False)


# ─────────────────────────── array helpers ──────────────────────────────────
def _shift_add(acc, src, dz, dy, dx):
    """acc += src shifted by (dz,dy,dx), border-clamped (no wrap)."""
    Z, Y, X = src.shape

    def rng(d, n):
        return max(0, -d), n - max(0, d), max(0, d), n - max(0, -d)
    zs0, zs1, zd0, zd1 = rng(dz, Z)
    ys0, ys1, yd0, yd1 = rng(dy, Y)
    xs0, xs1, xd0, xd1 = rng(dx, X)
    acc[zd0:zd1, yd0:yd1, xd0:xd1] += src[zs0:zs1, ys0:ys1, xs0:xs1]


def count_neighbors_3d(occ, conn):
    """Per-voxel count of occupied neighbours (uint8). conn ∈ {6,18,26}."""
    lim = 1 if conn == 6 else (2 if conn == 18 else 3)
    acc = np.zeros(occ.shape, np.uint8)
    occ8 = occ.view(np.uint8)
    for dz in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                l1 = abs(dz) + abs(dy) + abs(dx)
                if 0 < l1 <= lim:
                    _shift_add(acc, occ8, dz, dy, dx)
    return acc


def _shift2(m, dy, dx):
    """2D border-clamped shift (no wrap)."""
    out = np.zeros_like(m)
    Y, X = m.shape
    ys0, ys1, yd0, yd1 = (max(0, -dy), Y - max(0, dy),
                          max(0, dy), Y - max(0, -dy))
    xs0, xs1, xd0, xd1 = (max(0, -dx), X - max(0, dx),
                          max(0, dx), X - max(0, -dx))
    out[yd0:yd1, xd0:xd1] = m[ys0:ys1, xs0:xs1]
    return out


def _dilate4(mask, iters):
    """4-connected binary dilation by `iters` steps (pure numpy)."""
    m = mask
    for _ in range(iters):
        o = m.copy()
        o[1:, :] |= m[:-1, :]
        o[:-1, :] |= m[1:, :]
        o[:, 1:] |= m[:, :-1]
        o[:, :-1] |= m[:, 1:]
        m = o
    return m


def height_weights(zc, z_floor, z_ceil, z_peak, profile, sigma):
    """Per-z weight in [0,1]; 0 outside [z_floor,z_ceil], peak 1.0 at z_peak."""
    w = np.zeros_like(zc, np.float32)
    inb = (zc >= z_floor) & (zc <= z_ceil)
    if profile == "flat":
        w[inb] = 1.0
    elif profile == "gaussian":
        w[inb] = np.exp(-0.5 * ((zc[inb] - z_peak) / max(1e-3, sigma)) ** 2)
    else:  # triangular (default)
        up = inb & (zc <= z_peak)
        dn = inb & (zc > z_peak)
        w[up] = ((zc[up] - z_floor) / (z_peak - z_floor)
                 if z_peak > z_floor else 1.0)
        w[dn] = ((z_ceil - zc[dn]) / (z_ceil - z_peak)
                 if z_ceil > z_peak else 1.0)
    return np.clip(w, 0.0, 1.0)


class BevPublisher:
    def __init__(self):
        rospy.init_node("bev_publisher")
        G = rospy.get_param

        # ── geometry / IO (kept from v3) ─────────────────────────────────
        self.frame      = G("~frame_id",   "world")
        self.res        = float(G("~resolution", 0.15))
        self.publish_hz = float(G("~publish_hz", 10.0))
        self.out_topic  = G("~out_topic",  "/falcon/bev_2d")
        self.occ_topic  = G("~occ_topic",  "/voxel_mapping/occupancy_grid_occupied")
        self.free_topic = G("~free_topic", "/voxel_mapping/occupancy_grid_free")
        self.occ_dilate = int(G("~occ_dilate_cells", 0))
        margin          = float(G("~bbox_margin_m", 1.0))

        # ── vertical column / height weighting ───────────────────────────
        self.z_floor = float(G("~z_floor", 0.30))
        self.z_ceil  = float(G("~z_ceil",  2.20))
        self.z_peak  = float(G("~z_peak",  1.00))      # drone flight altitude
        self.profile = str(G("~weight_profile", "triangular"))
        self.sigma   = float(G("~weight_sigma", 0.50))
        self.vz      = float(G("~voxel_size_m", self.res))  # z-layer thickness

        # ── occupancy decision ───────────────────────────────────────────
        self.occ_w_thr     = float(G("~occ_weight_thresh", 1.2))
        self.min_occ_vox   = int(G("~min_occ_voxels", 2))
        self.min_free_vox  = int(G("~min_free_voxels", 1))

        # ── 3D neighbour confirm ─────────────────────────────────────────
        self.confirm_3d    = bool(G("~confirm_3d", True))
        self.nbr_conn      = int(G("~neighbors_3d", 6))
        self.min_nbr_3d    = int(G("~min_occ_neighbors_3d", 1))

        # ── door / window protection ─────────────────────────────────────
        self.protect_open  = bool(G("~protect_openings", True))
        self.door_band     = float(G("~door_band_m", 0.60))
        self.door_free_vox = int(G("~door_free_voxels", 2))
        self.door_occ_tol  = int(G("~door_occ_tol", 0))

        # ── wall completion (2D) ─────────────────────────────────────────
        self.wall_mode  = str(G("~wall_fill_mode", "directional"))  # off|directional|count
        self.wall_nbrs  = int(G("~wall_fill_neighbors", 5))
        self.wall_iters = int(G("~wall_fill_iters", 1))

        # ── temporal hysteresis (optional) ───────────────────────────────
        self.temporal = bool(G("~temporal_filter", False))
        self.t_inc, self.t_dec = float(G("~t_inc", 1.0)), float(G("~t_dec", 1.0))
        self.t_max = float(G("~t_max", 5.0))
        self.t_on,  self.t_off = float(G("~t_on", 2.0)), float(G("~t_off", 0.5))

        # ── runtime ──────────────────────────────────────────────────────
        self.always_recompute = bool(G("~always_recompute", False))
        self.skip_unchanged   = bool(G("~skip_unchanged_publish", True))

        # ── BEV bounds: launch override > /map_config yaml > hard fallback
        self.xmin, sx0 = self._resolve_bound("bbox_xmin", "map_min_x", -12.0, -margin)
        self.ymin, sy0 = self._resolve_bound("bbox_ymin", "map_min_y", -12.0, -margin)
        self.xmax, sx1 = self._resolve_bound("bbox_xmax", "map_max_x",  12.0, +margin)
        self.ymax, sy1 = self._resolve_bound("bbox_ymax", "map_max_y",  12.0, +margin)
        if self.xmax <= self.xmin or self.ymax <= self.ymin:
            rospy.logfatal("bev_publisher: invalid bounds x=[%.2f,%.2f] "
                           "y=[%.2f,%.2f]", self.xmin, self.xmax,
                           self.ymin, self.ymax)
            raise RuntimeError("bad BEV bbox")

        # virtual back-wall + manual walls (per-map, kept from v3)
        bw = rospy.get_param("/map_config/behind_wall_x", None)
        self.behind_wall_x = None if bw is None else float(bw)
        self.walls = []
        for w in (rospy.get_param("/map_config/walls", []) or []):
            try:
                yc, t = float(w["y"]), float(w.get("thickness", 0.10))
                self.walls.append({"x_min": float(w["x_min"]),
                                   "x_max": float(w["x_max"]),
                                   "y_min": yc - 0.5 * t, "y_max": yc + 0.5 * t})
            except (KeyError, TypeError, ValueError) as e:
                rospy.logwarn("bev_publisher: bad wall %r: %s", w, e)

        # grid sizing + z-layer geometry
        self.W = int(round((self.xmax - self.xmin) / self.res))
        self.H = int(round((self.ymax - self.ymin) / self.res))
        self.nz = max(1, int(round((self.z_ceil - self.z_floor) / self.vz)))
        zc = self.z_floor + (np.arange(self.nz) + 0.5) * self.vz   # layer centres
        self.wz = height_weights(zc, self.z_floor, self.z_ceil, self.z_peak,
                                 self.profile, self.sigma).astype(np.float32)
        self.band_idx = np.where(
            (zc >= self.z_peak - 0.5 * self.door_band) &
            (zc <= self.z_peak + 0.5 * self.door_band))[0]

        # latest cloud snapshots + filter state
        self._occ_xy = np.empty((0, 3), np.float32)
        self._free_xy = np.empty((0, 3), np.float32)
        self._dirty = True
        self._grid = None
        self._ev = np.zeros((self.H, self.W), np.float32)
        self._occ_state = np.zeros((self.H, self.W), bool)
        self._stats = {}

        self.pub = rospy.Publisher(self.out_topic, OccupancyGrid,
                                   queue_size=1, latch=True)
        rospy.Subscriber(self.occ_topic, PointCloud2, self._occ_cb, queue_size=2)
        rospy.Subscriber(self.free_topic, PointCloud2, self._free_cb, queue_size=2)
        self.hb = dict(occ=0, free=0, pub=0)
        rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self._tick)
        rospy.Timer(rospy.Duration(5.0), self._heartbeat)

        self._banner(sx0, sx1, sy0, sy1, margin)

    # ─────────────────────────── banner ─────────────────────────────────
    def _banner(self, sx0, sx1, sy0, sy1, margin):
        L = rospy.loginfo
        L("=" * 70)
        L("  BEV publisher v4  (height-weighted column + 3D neighbour reasoning)")
        L("  grid %d×%d @ %.3fm   z=[%.2f,%.2f] peak=%.2f  %d layers @%.3fm",
          self.W, self.H, self.res, self.z_floor, self.z_ceil, self.z_peak,
          self.nz, self.vz)
        L("  bounds x=[%.2f,%.2f] (%s/%s)  y=[%.2f,%.2f] (%s/%s) margin=%.2f",
          self.xmin, self.xmax, sx0, sx1, self.ymin, self.ymax, sy0, sy1, margin)
        L("  occ: weight>=%.2f AND count>=%d   weight=%s",
          self.occ_w_thr, self.min_occ_vox, self.profile)
        L("  confirm_3d=%s (conn=%d, min_nbr=%d)   free: count>=%d",
          self.confirm_3d, self.nbr_conn, self.min_nbr_3d, self.min_free_vox)
        L("  protect_openings=%s (band=%.2fm free>=%d occ<=%d)",
          self.protect_open, self.door_band, self.door_free_vox, self.door_occ_tol)
        L("  wall_fill=%s (nbrs=%d iters=%d)   temporal=%s   occ_dilate=%d",
          self.wall_mode, self.wall_nbrs, self.wall_iters, self.temporal,
          self.occ_dilate)
        if self.behind_wall_x is not None:
            L("  behind_wall_x=%.2f m", self.behind_wall_x)
        if self.walls:
            L("  manual walls: %d", len(self.walls))
        L("  pub@%.1fHz latched  always_recompute=%s  skip_unchanged=%s",
          self.publish_hz, self.always_recompute, self.skip_unchanged)
        L("  in  occ=%s", self.occ_topic)
        L("  in  free=%s", self.free_topic)
        L("  out=%s", self.out_topic)
        L("=" * 70)

    @staticmethod
    def _resolve_bound(local, mapcfg, fallback, margin_signed):
        """~<local> (launch override, exact) > /map_config/map_size/<mapcfg>
        (+margin) > fallback (+margin)."""
        if rospy.has_param("~" + local):
            return float(rospy.get_param("~" + local)), "launch"
        g = "/map_config/map_size/" + mapcfg
        if rospy.has_param(g):
            return float(rospy.get_param(g)) + margin_signed, "mapcfg"
        return float(fallback) + margin_signed, "fallback"

    # ─────────────────────────── subscribers ────────────────────────────
    def _occ_cb(self, msg):
        self._occ_xy = cloud_xyz(msg); self.hb["occ"] += 1; self._dirty = True

    def _free_cb(self, msg):
        self._free_xy = cloud_xyz(msg); self.hb["free"] += 1; self._dirty = True

    # ─────────────────────────── cell mapping ───────────────────────────
    def _cells(self, xy):
        cx = ((xy[:, 0] - self.xmin) / self.res).astype(np.int32)
        cy = ((xy[:, 1] - self.ymin) / self.res).astype(np.int32)
        ok = (cx >= 0) & (cx < self.W) & (cy >= 0) & (cy < self.H)
        return cx, cy, ok

    def _occ_volume(self, xy):
        """Sparse 3D occupied volume (nz,H,W) bool from voxel centres."""
        vol = np.zeros((self.nz, self.H, self.W), bool)
        if xy.shape[0]:
            cx, cy, ok = self._cells(xy)
            lz = np.floor((xy[:, 2] - self.z_floor) / self.vz).astype(np.int32)
            ok &= (lz >= 0) & (lz < self.nz)
            if ok.any():
                vol.reshape(-1)[(lz[ok] * self.H + cy[ok]) * self.W + cx[ok]] = True
        return vol

    def _col_count(self, xy, zlo=None, zhi=None):
        """2D count of points per cell, optionally only those in [zlo,zhi]."""
        out = np.zeros((self.H, self.W), np.int32)
        if xy.shape[0] == 0:
            return out
        cx, cy, ok = self._cells(xy)
        if zlo is not None:
            ok &= (xy[:, 2] >= zlo) & (xy[:, 2] <= zhi)
        if ok.any():
            out += np.bincount(cy[ok] * self.W + cx[ok],
                               minlength=self.H * self.W).reshape(self.H, self.W)
        return out

    # ─────────────────────────── pipeline ───────────────────────────────
    def _recompute(self):
        occ_xy, free_xy = self._occ_xy, self._free_xy   # atomic refs (snapshot)

        # 1) 3D occupied volume (+ 2) optional neighbour confirm)
        vol = self._occ_volume(occ_xy)
        n_raw = int(vol.sum())
        if self.confirm_3d and n_raw:
            vol &= count_neighbors_3d(vol, self.nbr_conn) >= self.min_nbr_3d
        n_conf = int(vol.sum())

        # 1) column projection: weighted mass + raw count + flight-band occ
        occ_w = np.tensordot(self.wz, vol.astype(np.float32), axes=([0], [0]))
        occ_c = vol.sum(axis=0).astype(np.int32)
        occ_band = (vol[self.band_idx].sum(axis=0).astype(np.int32)
                    if self.band_idx.size else np.zeros_like(occ_c))

        base_occ = (occ_w >= self.occ_w_thr) & (occ_c >= self.min_occ_vox)

        # free: known-free anywhere in column + free within the flight band
        free_c = self._col_count(free_xy)
        free_band = self._col_count(free_xy,
                                    self.z_peak - 0.5 * self.door_band,
                                    self.z_peak + 0.5 * self.door_band)
        observed_free = free_c >= self.min_free_vox

        # 3) door / window protection — open at flight height ⇒ force FREE
        protected = np.zeros_like(base_occ)
        if self.protect_open:
            protected = (base_occ & (free_band >= self.door_free_vox)
                         & (occ_band <= self.door_occ_tol))
            base_occ &= ~protected

        # 4) wall completion — fill UNKNOWN gaps only, never seen-free/openings
        occ = base_occ.copy()
        n_fill = 0
        if self.wall_mode != "off" and self.wall_iters > 0:
            blocked = observed_free | protected
            for _ in range(self.wall_iters):
                if self.wall_mode == "count":
                    cnt = (_shift2(occ, 0, -1).astype(np.uint8) + _shift2(occ, 0, 1)
                           + _shift2(occ, -1, 0) + _shift2(occ, 1, 0)
                           + _shift2(occ, -1, -1) + _shift2(occ, 1, 1)
                           + _shift2(occ, -1, 1) + _shift2(occ, 1, -1))
                    cand = cnt >= self.wall_nbrs
                else:  # directional: bridge two opposite occupied sides
                    cand = ((_shift2(occ, 0, -1) & _shift2(occ, 0, 1))
                            | (_shift2(occ, -1, 0) & _shift2(occ, 1, 0))
                            | (_shift2(occ, -1, -1) & _shift2(occ, 1, 1))
                            | (_shift2(occ, -1, 1) & _shift2(occ, 1, -1)))
                cand &= ~occ & ~blocked
                if not cand.any():
                    break
                n_fill += int(cand.sum())
                occ |= cand

        # 5) temporal hysteresis (optional)
        if self.temporal:
            self._ev += self.t_inc * occ - self.t_dec * (observed_free & ~occ)
            np.clip(self._ev, 0.0, self.t_max, out=self._ev)
            self._occ_state = ((self._occ_state & ~(self._ev <= self.t_off))
                               | (self._ev >= self.t_on))
            occ = self._occ_state.copy()

        # 6) compose label grid  (OCC > FREE > UNK), then carve openings,
        #    then stamp hard manual/back walls, then optional dilation.
        grid = np.full((self.H, self.W), UNK, np.int8)
        grid[observed_free] = FREE
        grid[occ] = OCC
        grid[protected] = FREE                       # keep doors/windows open

        if self.behind_wall_x is not None:
            cxm = min(self.W, int((self.behind_wall_x - self.xmin) / self.res))
            if cxm > 0:
                grid[:, :cxm] = OCC
        for w in self.walls:
            cx0 = max(0, int(np.floor((w["x_min"] - self.xmin) / self.res)))
            cx1 = min(self.W, int(np.ceil((w["x_max"] - self.xmin) / self.res)))
            cy0 = max(0, int(np.floor((w["y_min"] - self.ymin) / self.res)))
            cy1 = min(self.H, int(np.ceil((w["y_max"] - self.ymin) / self.res)))
            if cx1 > cx0 and cy1 > cy0:
                grid[cy0:cy1, cx0:cx1] = OCC

        if self.occ_dilate > 0:
            occ_all = grid == OCC
            new = _dilate4(occ_all, self.occ_dilate) & ~occ_all & ~protected
            grid[new] = OCC

        self._stats = dict(raw=n_raw, conf=n_conf,
                           occ=int((grid == OCC).sum()),
                           free=int((grid == FREE).sum()),
                           unk=int((grid == UNK).sum()),
                           open=int(protected.sum()), fill=n_fill)
        return grid

    # ─────────────────────────── publish ────────────────────────────────
    def _tick(self, _evt):
        if self._dirty or self._grid is None or self.always_recompute:
            self._dirty = False
            self._grid = self._recompute()
            self._publish_grid()
        elif not self.skip_unchanged:
            self._publish_grid()

    def _publish_grid(self):
        m = OccupancyGrid()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self.frame
        m.info.map_load_time = m.header.stamp
        m.info.resolution = self.res
        m.info.width, m.info.height = self.W, self.H
        m.info.origin.position.x = self.xmin
        m.info.origin.position.y = self.ymin
        m.info.origin.orientation.w = 1.0
        m.data = self._grid.flatten().tolist()
        self.pub.publish(m)
        self.hb["pub"] += 1

    def _heartbeat(self, _evt):
        s = self._stats
        rospy.loginfo("bev hb  in occ=%d free=%d  pub=%d  |  voxels raw=%d "
                      "conf=%d  |  grid occ=%d free=%d unk=%d open=%d fill=%d",
                      self.hb["occ"], self.hb["free"], self.hb["pub"],
                      s.get("raw", 0), s.get("conf", 0), s.get("occ", 0),
                      s.get("free", 0), s.get("unk", 0), s.get("open", 0),
                      s.get("fill", 0))
        if self._occ_xy.shape[0]:
            cx, cy, ok = self._cells(self._occ_xy)
            n_in = int(ok.sum()); n_tot = int(self._occ_xy.shape[0])
            if n_tot > 100 and n_in < 0.9 * n_tot:
                rospy.logwarn_throttle(
                    20.0, "bev_publisher: %d/%d occ points OUTSIDE bbox "
                    "x=[%.1f,%.1f] y=[%.1f,%.1f] — bounds may be wrong",
                    n_tot - n_in, n_tot, self.xmin, self.xmax,
                    self.ymin, self.ymax)
        self.hb = dict(occ=0, free=0, pub=0)


if __name__ == "__main__":
    try:
        BevPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# ═════════════════════════════════════════════════════════════════════════════
# PARAMETERS  (all private ~params; defaults in parentheses)
# Set a gate to its "off" value to isolate one stage; combine them for the full
# de-noiser. Drop-in: topics & msg types are unchanged from v2/v3.
#
#   ── geometry / IO ──────────────────────────────────────────────────────────
#   ~resolution            (0.15)  metres / cell. Match FALCON's voxel size.
#   ~publish_hz            (10.0)  OccupancyGrid publish rate (latched).
#   ~bbox_{xmin,xmax,ymin,ymax}    explicit bounds; else /map_config, else ±12.
#   ~bbox_margin_m         (1.0)   buffer added to /map_config bounds.
#   ~occ_dilate_cells      (0)     inflate OCC by N cells (planner also inflates).
#
#   ── vertical column / height weighting (stage 1) ────────────────────────────
#   ~z_floor ~z_ceil       (0.30, 2.20)  column z-range considered.
#   ~z_peak                (1.00)  drone altitude — weight peaks here.
#   ~weight_profile        (triangular)  triangular | gaussian | flat.
#   ~weight_sigma          (0.50)  gaussian width (if profile=gaussian).
#   ~voxel_size_m          (=resolution)  z-layer thickness.
#   ~occ_weight_thresh     (1.2)   min weighted column mass for OCC.
#   ~min_occ_voxels        (2)     AND min raw voxel count for OCC.
#   ~min_free_voxels       (1)     min voxels for a cell to read FREE.
#
#   ── 3D neighbour confirm (stage 2) ──────────────────────────────────────────
#   ~confirm_3d            (True)  drop voxels with too few occupied neighbours.
#   ~neighbors_3d          (6)     connectivity: 6 | 18 | 26.
#   ~min_occ_neighbors_3d  (1)     min occupied neighbours to survive.
#
#   ── door / window protection (stage 3) ──────────────────────────────────────
#   ~protect_openings      (True)  never wall a cell that is open at flight ht.
#   ~door_band_m           (0.60)  z-band around z_peak inspected for openness.
#   ~door_free_voxels      (2)     free voxels in the band ⇒ it's an opening.
#   ~door_occ_tol          (0)     max occupied voxels allowed in the band.
#
#   ── wall completion (stage 4) ───────────────────────────────────────────────
#   ~wall_fill_mode        (directional)  off | directional | count.
#   ~wall_fill_neighbors   (5)     (count mode) occupied 8-neighbours to fill.
#   ~wall_fill_iters       (1)     max bridge width; keep small (1–2).
#
#   ── temporal hysteresis (stage 5, optional) ─────────────────────────────────
#   ~temporal_filter       (False) FALCON already fuses in time; usually off.
#   ~t_inc ~t_dec ~t_max ~t_on ~t_off  (1,1,5,2,0.5)  evidence accumulator.
#
#   ── runtime ──────────────────────────────────────────────────────────────────
#   ~always_recompute      (False) rerun full pipeline every tick (else on new cloud)
#   ~skip_unchanged_publish(True)  don't republish an unchanged grid.
#
# Suggested launch <node> (replaces the existing bev_publisher block):
#
#   <node pkg="falcon_adapter" type="bev_publisher.py" name="bev_publisher"
#         output="screen">
#     <param name="resolution"            value="0.15"/>
#     <param name="publish_hz"            value="10.0"/>
#     <param name="z_floor"               value="0.30"/>
#     <param name="z_ceil"                value="2.20"/>
#     <param name="z_peak"                value="1.00"/>   <!-- flight altitude -->
#     <param name="occ_weight_thresh"     value="1.2"/>
#     <param name="min_occ_voxels"        value="2"/>
#     <param name="confirm_3d"            value="true"/>
#     <param name="neighbors_3d"          value="6"/>
#     <param name="min_occ_neighbors_3d"  value="1"/>
#     <param name="protect_openings"      value="true"/>
#     <param name="door_band_m"           value="0.60"/>
#     <param name="door_free_voxels"      value="2"/>
#     <param name="wall_fill_mode"        value="directional"/>
#     <param name="wall_fill_iters"       value="1"/>
#     <param name="occ_dilate_cells"      value="0"/>
#   </node>
# ═════════════════════════════════════════════════════════════════════════════