#!/usr/bin/env python3
"""
bev_publisher.py — FALCON-side 2D BEV publisher for the MORE perception node.

v3 — auto-bounds from /map_config/map_size
  vs v2:
  • Default BEV bbox now reads `/map_config/map_size/{map_min,map_max}_{x,y}`
    — the same per-map yaml that gazebo_waypoint_nav.launch already loads
    via `<rosparam command="load" file="$(find exploration_manager)/config/map/$(arg map_name).yaml"/>`.
    So small_house gets ±10 m, hospital gets its asymmetric −34..+17,
    etc., automatically. No more "BEV defaults to ±12 m regardless of
    which world you loaded."
  • Optional `~bbox_margin_m` (default 1.0) extends the BEV slightly
    beyond the FALCON map bounds so cells right at the edge aren't
    clipped.
  • Logs which source supplied each bound (`launch override`,
    `/map_config`, or `hard fallback`) so the active bounds are
    obvious from the bringup banner.
  • The local `~bbox_*` params still win when set explicitly — useful
    for testing or for maps whose yaml is missing/wrong.

Behavior is otherwise identical to v2 (consistent-snapshot rebuild,
OCC > FREE projection, optional OCC dilation).
"""

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid


UNK, FREE, OCC = -1, 0, 100


def _dilate4(mask, iters):
    """4-connected binary dilation by `iters` steps. Pure numpy — no scipy."""
    m = mask
    for _ in range(iters):
        out = m.copy()
        out[1:,  :] |= m[:-1, :]
        out[:-1, :] |= m[1:,  :]
        out[:, 1:]  |= m[:, :-1]
        out[:, :-1] |= m[:, 1:]
        m = out
    return m


class BevPublisher:
    def __init__(self):
        rospy.init_node("bev_publisher")

        G = rospy.get_param
        self.frame       = G("~frame_id",    "world")
        self.res         = float(G("~resolution",  0.15))
        self.z_min       = float(G("~z_slab_min",  0.30))
        self.z_max       = float(G("~z_slab_max",  1.80))
        self.publish_hz  = float(G("~publish_hz",  2.0))
        self.out_topic   = G("~out_topic",   "/falcon/bev_2d")
        self.occ_topic   = G("~occ_topic",
                             "/voxel_mapping/occupancy_grid_occupied")
        self.free_topic  = G("~free_topic",
                             "/voxel_mapping/occupancy_grid_free")
        self.occ_dilate  = int(G("~occ_dilate_cells", 1))
        margin           = float(G("~bbox_margin_m", 1.0))

        # ── Resolve BEV bounds with the priority:
        #    1. ~bbox_<name> set on this node (launch override)
        #    2. /map_config/map_size/<name>     (the per-map yaml)
        #    3. hard fallback (±12 m)
        self.xmin, src_xmin = self._resolve_bound("bbox_xmin", "map_min_x", -12.0, -margin)
        self.ymin, src_ymin = self._resolve_bound("bbox_ymin", "map_min_y", -12.0, -margin)
        self.xmax, src_xmax = self._resolve_bound("bbox_xmax", "map_max_x",  12.0, +margin)
        self.ymax, src_ymax = self._resolve_bound("bbox_ymax", "map_max_y",  12.0, +margin)

        if self.xmax <= self.xmin or self.ymax <= self.ymin:
            rospy.logfatal("bev_publisher: invalid bounds  x=[%.2f, %.2f] y=[%.2f, %.2f] "
                           "— check /map_config/map_size/*",
                           self.xmin, self.xmax, self.ymin, self.ymax)
            raise RuntimeError("bad BEV bbox")

        self.W = int(round((self.xmax - self.xmin) / self.res))
        self.H = int(round((self.ymax - self.ymin) / self.res))
        self.grid = np.full((self.H, self.W), UNK, dtype=np.int8)

        self._free_xy = np.empty((0, 2), np.float32)
        self._occ_xy  = np.empty((0, 2), np.float32)

        self.pub = rospy.Publisher(self.out_topic, OccupancyGrid,
                                   queue_size=1, latch=True)
        rospy.Subscriber(self.occ_topic,  PointCloud2,
                         self._occ_cb,  queue_size=2)
        rospy.Subscriber(self.free_topic, PointCloud2,
                         self._free_cb, queue_size=2)

        self.hb = dict(occ=0, free=0, pub=0)
        rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self._publish)
        rospy.Timer(rospy.Duration(5.0), self._heartbeat)

        rospy.loginfo("=" * 64)
        rospy.loginfo("  BEV publisher v3  (auto-bounds from /map_config)")
        rospy.loginfo("  Grid: %d × %d  @ %.3f m   margin=%.2f m",
                      self.W, self.H, self.res, margin)
        rospy.loginfo("  bounds  x=[%.2f, %.2f]  src: %s / %s",
                      self.xmin, self.xmax, src_xmin, src_xmax)
        rospy.loginfo("  bounds  y=[%.2f, %.2f]  src: %s / %s",
                      self.ymin, self.ymax, src_ymin, src_ymax)
        rospy.loginfo("  z-slab=[%.2f, %.2f]m   occ_dilate=%d   pub@%.1fHz (latched)",
                      self.z_min, self.z_max, self.occ_dilate, self.publish_hz)
        rospy.loginfo("  out=%s",  self.out_topic)
        rospy.loginfo("  in occ =%s",  self.occ_topic)
        rospy.loginfo("  in free=%s",  self.free_topic)
        rospy.loginfo("=" * 64)

    @staticmethod
    def _resolve_bound(local_name, mapcfg_name, fallback, margin_signed):
        """Return (value, source_label).

        Priority:
          1. `~<local_name>` if set on this node              → launch override
          2. `/map_config/map_size/<mapcfg_name>` if set      → per-map yaml
          3. `fallback`                                       → hard default

        `margin_signed` is added to map_config and fallback values to
        give a small buffer beyond FALCON's voxel-mapping bounds. It is
        NOT applied to launch overrides — when the user sets
        `bbox_xmin:=-15`, they mean exactly -15."""
        priv = "~" + local_name
        if rospy.has_param(priv):
            return float(rospy.get_param(priv)), "launch override"
        global_path = "/map_config/map_size/" + mapcfg_name
        if rospy.has_param(global_path):
            return float(rospy.get_param(global_path)) + margin_signed, "/map_config"
        return float(fallback) + margin_signed, "hard fallback"

    def _parse_xy(self, msg):
        pts = np.array(list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32)
        if pts.size == 0:
            return np.empty((0, 2), np.float32)
        z = pts[:, 2]
        sel = (z >= self.z_min) & (z <= self.z_max)
        return pts[sel, :2]

    def _occ_cb(self,  msg):
        self._occ_xy = self._parse_xy(msg); self.hb['occ']  += 1
    def _free_cb(self, msg):
        self._free_xy = self._parse_xy(msg); self.hb['free'] += 1

    def _write(self, xy, state):
        if xy.size == 0:
            return
        cx = ((xy[:, 0] - self.xmin) / self.res).astype(np.int32)
        cy = ((xy[:, 1] - self.ymin) / self.res).astype(np.int32)
        ok = (cx >= 0) & (cx < self.W) & (cy >= 0) & (cy < self.H)
        if ok.any():
            self.grid[cy[ok], cx[ok]] = state

    def _publish(self, _evt):
        self.grid.fill(UNK)
        self._write(self._free_xy, FREE)
        self._write(self._occ_xy,  OCC)
        if self.occ_dilate > 0:
            occ_mask = (self.grid == OCC)
            dilated  = _dilate4(occ_mask, self.occ_dilate)
            self.grid[dilated & ~occ_mask] = OCC

        m = OccupancyGrid()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self.frame
        m.info.map_load_time = m.header.stamp
        m.info.resolution = self.res
        m.info.width      = self.W
        m.info.height     = self.H
        m.info.origin.position.x = self.xmin
        m.info.origin.position.y = self.ymin
        m.info.origin.position.z = 0.0
        m.info.origin.orientation.w = 1.0
        m.data = self.grid.flatten().tolist()
        self.pub.publish(m)
        self.hb['pub'] += 1

    def _heartbeat(self, _evt):
        nf = int((self.grid == FREE).sum())
        no = int((self.grid == OCC).sum())
        nu = int((self.grid == UNK).sum())

        def in_bounds_count(xy):
            if xy.size == 0:
                return 0
            cx = ((xy[:, 0] - self.xmin) / self.res).astype(np.int32)
            cy = ((xy[:, 1] - self.ymin) / self.res).astype(np.int32)
            return int(((cx >= 0) & (cx < self.W) &
                        (cy >= 0) & (cy < self.H)).sum())

        occ_in  = in_bounds_count(self._occ_xy)
        free_in = in_bounds_count(self._free_xy)

        rospy.loginfo(
            "bev hb  in: occ=%d free=%d  out: pub=%d  |  "
            "grid free=%d occ=%d unk=%d",
            self.hb['occ'], self.hb['free'], self.hb['pub'], nf, no, nu)
        rospy.loginfo(
            "        snapshots: occ=%d (%d in-bbox) free=%d (%d in-bbox)",
            int(self._occ_xy.shape[0]),  occ_in,
            int(self._free_xy.shape[0]), free_in)
        # Warn if a substantial fraction of points are outside the BEV
        # bbox — that's the symptom of bounds being wrong.
        n_occ_total  = int(self._occ_xy.shape[0])
        n_free_total = int(self._free_xy.shape[0])
        for label, n_in, n_total in (("occ", occ_in, n_occ_total),
                                      ("free", free_in, n_free_total)):
            if n_total > 100 and n_in < 0.9 * n_total:
                rospy.logwarn_throttle(20.0,
                    "bev_publisher: %d/%d %s points OUTSIDE bbox  "
                    "x=[%.1f,%.1f] y=[%.1f,%.1f] — bounds may be wrong",
                    n_total - n_in, n_total, label,
                    self.xmin, self.xmax, self.ymin, self.ymax)
        if self._occ_xy.size > 0:
            rospy.loginfo(
                "        occ  xy: x=[%.2f, %.2f]  y=[%.2f, %.2f]",
                float(self._occ_xy[:, 0].min()),
                float(self._occ_xy[:, 0].max()),
                float(self._occ_xy[:, 1].min()),
                float(self._occ_xy[:, 1].max()))
        if self._free_xy.size > 0:
            rospy.loginfo(
                "        free xy: x=[%.2f, %.2f]  y=[%.2f, %.2f]",
                float(self._free_xy[:, 0].min()),
                float(self._free_xy[:, 0].max()),
                float(self._free_xy[:, 1].min()),
                float(self._free_xy[:, 1].max()))
        self.hb = dict(occ=0, free=0, pub=0)


if __name__ == "__main__":
    try:
        BevPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass