#!/usr/bin/env python3
"""
voxel_reset_watcher.py — detect when FALCON's voxel map "resets".

Subscribes to FALCON's occupied-voxels PointCloud2 (default
/voxel_mapping/occupancy_grid_occupied). Each callback we count points
and compare with the previous count. If the count collapses by more
than `drop_frac` AND the previous count was at least `min_prev`, we
log a RESET event with timestamp and a summary.

Why this is useful: it tells you exactly WHEN the map disappears.
Cross-reference with `exploration_node` logs:
  • If at the same instant `exploration_node` printed a startup banner
    ("[exploration_node] starting…" / param load lines), it's a
    process respawn (fix: respawn="false" in the launch).
  • If `exploration_node` kept running and the map still dropped, it's
    an in-process clear; grep your FALCON src for the call site (see
    the grep recipe in the assistant's message).

Run it alongside FALCON:
    rosrun falcon_adapter voxel_reset_watcher.py \
        _topic:=/voxel_mapping/occupancy_grid_occupied \
        _drop_frac:=0.5 _min_prev:=500
"""
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2


class VoxelResetWatcher:
    def __init__(self):
        rospy.init_node("voxel_reset_watcher")
        G = rospy.get_param
        self.topic     = G("~topic", "/voxel_mapping/occupancy_grid_occupied")
        self.drop_frac = float(G("~drop_frac", 0.5))   # ≥50% drop is a "reset"
        self.min_prev  = int  (G("~min_prev", 500))    # only if we had ≥500 cells
        self.print_hz  = float(G("~print_hz", 1.0))    # heartbeat rate

        self.prev_count   = 0
        self.peak_count   = 0
        self.last_count   = 0
        self.reset_events = 0
        self.last_reset_t = None

        rospy.Subscriber(self.topic, PointCloud2, self._cb, queue_size=2)
        rospy.Timer(rospy.Duration(1.0 / self.print_hz), self._tick)
        rospy.loginfo("voxel_reset_watcher  topic=%s  drop_frac=%.2f  min_prev=%d",
                      self.topic, self.drop_frac, self.min_prev)

    def _cb(self, msg):
        # width*height is fast; if your build sets these to (N,1) the count is N.
        # Otherwise we walk the cloud once.
        n = msg.width * msg.height
        if n == 0:
            try:
                n = sum(1 for _ in pc2.read_points(msg, skip_nans=True))
            except Exception:
                n = 0

        if (self.prev_count >= self.min_prev
                and n < self.prev_count * (1.0 - self.drop_frac)):
            self.reset_events += 1
            self.last_reset_t  = rospy.Time.now()
            rospy.logwarn(
                "═══ VOXEL RESET #%d ═══  prev=%d  now=%d  peak_so_far=%d  "
                "stamp=%.3f  →  check exploration_node logs at this time "
                "for a startup banner (respawn) or a clear/init call.",
                self.reset_events, self.prev_count, n,
                self.peak_count, self.last_reset_t.to_sec())

        self.last_count = n
        self.prev_count = n
        if n > self.peak_count:
            self.peak_count = n

    def _tick(self, _e):
        rospy.loginfo(
            "voxel hb  count=%d  peak=%d  resets=%d  last_reset=%s",
            self.last_count, self.peak_count, self.reset_events,
            "%.1fs ago" % (rospy.Time.now() - self.last_reset_t).to_sec()
            if self.last_reset_t is not None else "never")


if __name__ == "__main__":
    try:
        VoxelResetWatcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass