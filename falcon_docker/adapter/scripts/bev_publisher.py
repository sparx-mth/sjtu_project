#!/usr/bin/env python3
"""
bev_publisher.py — FALCON-side 2D BEV publisher for the MORE perception node.

Runs INSIDE the FALCON container (ROS 1 Noetic), subscribes to FALCON's
occupied + free voxel clouds, and publishes a single nav_msgs/OccupancyGrid
at /falcon/bev_2d.

    /voxel_mapping/occupancy_grid_occupied  ──┐   rebuild 2D grid
    /voxel_mapping/occupancy_grid_free       ──┘   each publish tick
                                                   │
                                                   ▼
                        nav_msgs/OccupancyGrid  on  /falcon/bev_2d

Design (v2 — consistent-snapshot rebuild):

  FALCON publishes each classification (occupied, free, unknown) on its
  own timer, independently. We treat each received cloud as a SNAPSHOT
  of that classification's current state, cache the latest, and on the
  publish timer REBUILD the 2D grid from scratch using priority:

      1. start all UNK (-1)
      2. mark every FREE point's cell as 0
      3. mark every OCCUPIED point's cell as 100  ← overwrites stale FREE

  Why this matters:
    - Accumulating marks as callbacks fire caused the 2D grid to blink
      between states because the three snapshot streams are decoupled
      (a late 'unknown' snapshot would overwrite a 'free' mark made
      moments earlier).
    - 3D→2D collapse: two voxels in the same z-column can have different
      states. Applying OCC last means "2D cell is occupied if ANY voxel
      in its column is occupied" — the correct projection rule.
    - No need for 'occupancy_grid_unknown'; unknown is the default state
      of any cell not named in a free or occupied snapshot.

Encoding (standard nav_msgs/OccupancyGrid):
  -1 = unknown, 0 = free, 100 = occupied
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
        out[1:,  :] |= m[:-1, :]    # grow down
        out[:-1, :] |= m[1:,  :]    # grow up
        out[:, 1:]  |= m[:, :-1]    # grow right
        out[:, :-1] |= m[:, 1:]     # grow left
        m = out
    return m


class BevPublisher:
    def __init__(self):
        rospy.init_node("bev_publisher")

        # ── Parameters ──
        G = rospy.get_param
        self.frame       = G("~frame_id",    "world")
        self.res         = G("~resolution",  0.15)
        self.xmin        = G("~bbox_xmin",  -12.0)
        self.ymin        = G("~bbox_ymin",  -34.0)
        self.xmax        = G("~bbox_xmax",   12.0)
        self.ymax        = G("~bbox_ymax",   17.0)
        self.z_min       = G("~z_slab_min",  0.30)
        self.z_max       = G("~z_slab_max",  1.80)
        self.publish_hz  = G("~publish_hz",  2.0)
        self.out_topic   = G("~out_topic",   "/falcon/bev_2d")
        self.occ_topic   = G("~occ_topic",
                             "/voxel_mapping/occupancy_grid_occupied")
        self.free_topic  = G("~free_topic",
                             "/voxel_mapping/occupancy_grid_free")
        # How many cells to dilate OCC by after rasterization. 1 is a
        # small but very effective defense against single-cell wall
        # erosion from FALCON's log-odds reclassification when the
        # drone moves vertically and rays graze wall voxels. Set to 0
        # to disable; raise to 2 if wall erosion is more aggressive.
        self.occ_dilate  = int(G("~occ_dilate_cells", 1))

        # ── Grid dimensions (fixed at init; rebuilt from snapshots) ──
        self.W = int(round((self.xmax - self.xmin) / self.res))
        self.H = int(round((self.ymax - self.ymin) / self.res))
        self.grid = np.full((self.H, self.W), UNK, dtype=np.int8)

        # ── Latest snapshots (xy points in the z-slab) ──
        self._free_xy = np.empty((0, 2), np.float32)
        self._occ_xy  = np.empty((0, 2), np.float32)

        # ── ROS glue ──
        self.pub = rospy.Publisher(self.out_topic, OccupancyGrid,
                                   queue_size=1, latch=True)
        rospy.Subscriber(self.occ_topic,  PointCloud2,
                         self._occ_cb,  queue_size=2)
        rospy.Subscriber(self.free_topic, PointCloud2,
                         self._free_cb, queue_size=2)

        self.hb = dict(occ=0, free=0, pub=0)
        rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self._publish)
        rospy.Timer(rospy.Duration(5.0), self._heartbeat)

        rospy.loginfo("=" * 60)
        rospy.loginfo("  BEV publisher v2  (snapshot rebuild, OCC > FREE)")
        rospy.loginfo("  Grid: %d x %d @ %.3f m  x=[%.1f, %.1f]  y=[%.1f, %.1f]",
                      self.W, self.H, self.res,
                      self.xmin, self.xmax, self.ymin, self.ymax)
        rospy.loginfo("  z-slab: [%.2f, %.2f] m", self.z_min, self.z_max)
        rospy.loginfo("  Publish: %s @ %.1f Hz (latched)",
                      self.out_topic, self.publish_hz)
        rospy.loginfo("  Listen:  occ =%s", self.occ_topic)
        rospy.loginfo("           free=%s", self.free_topic)
        rospy.loginfo("=" * 60)

    def _parse_xy(self, msg):
        """Extract (N,2) xy of every point in the z-slab."""
        pts = np.array(list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32)
        if pts.size == 0:
            return np.empty((0, 2), np.float32)
        z = pts[:, 2]
        sel = (z >= self.z_min) & (z <= self.z_max)
        return pts[sel, :2]

    # Callbacks just cache the latest snapshot. Grid is rebuilt on the
    # publish timer — never on a callback — so arrival order doesn't
    # matter and no transient "blinking" can happen.
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
        # Rebuild from scratch so the published grid is always consistent
        # with the most recent free+occ snapshots. Order is FREE then OCC
        # so occupied cells in the z-column correctly shadow free voxels
        # at other heights in the same column.
        self.grid.fill(UNK)
        self._write(self._free_xy, FREE)
        self._write(self._occ_xy,  OCC)

        # Thicken walls so single-cell erosion (caused by FALCON's
        # log-odds reclassification when rays graze walls at new
        # altitudes) can't produce a hole that merges two rooms.
        # OCC grows, taking cells from FREE and UNK equally.
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

        # How many of the cached points actually land in the grid?
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