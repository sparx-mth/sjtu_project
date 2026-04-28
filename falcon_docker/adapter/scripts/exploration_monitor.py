#!/usr/bin/env python3
"""
exploration_monitor.py — live plots of FALCON exploration progress.

Two stacked plots, updated in real time:
  (1) Explored fraction (%) vs time
  (2) New voxels discovered per second vs time

It works by subscribing to FALCON's mapped-voxel PointCloud2 and
counting points each message. The point count is the number of
known voxels; its time derivative is the discovery rate.

Default topic is /sdf_map/occupancy_all (the colourful cells you
see in RViz). If your FALCON build publishes under a different
name, just override:
    rosrun falcon_adapter exploration_monitor.py _topic:=/voxel_mapping/occupancy_grid_occupied

Tip: while FALCON is running, find the right topic with
    rostopic list | grep -Ei 'occupancy|voxel|sdf'
and pick the one whose `rostopic hz` rises and whose point count
grows as the drone explores.

Percentage reference (100% = ?):
    By default, 100% = total voxels in the bounding box from
    hospital.yaml (40 x 40 x 2 m @ 0.15 m). Override either by
    setting ~total_voxels directly, or by tweaking ~box_x/y/z
    and ~resolution.
"""

import threading
from collections import deque

import numpy as np
import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import PointCloud2


class ExplorationMonitor:
    def __init__(self):
        rospy.init_node("exploration_monitor", anonymous=True)

        self.topic = rospy.get_param("~topic", "/sdf_map/occupancy_all")

        total = rospy.get_param("~total_voxels", 0)
        if total > 0:
            self.total_voxels = float(total)
        else:
            res = rospy.get_param("~resolution", 0.15)
            bx  = rospy.get_param("~box_x", 40.0)
            by  = rospy.get_param("~box_y", 40.0)
            bz  = rospy.get_param("~box_z",  2.0)
            self.total_voxels = (bx / res) * (by / res) * (bz / res)

        self.lock   = threading.Lock()
        self.t0     = None
        self.times  = deque(maxlen=20000)
        self.counts = deque(maxlen=20000)

        self.msg_count = 0
        rospy.Subscriber(self.topic, PointCloud2, self._cb, queue_size=2)
        rospy.loginfo("monitor: topic=%s  100%%=%.0f voxels",
                      self.topic, self.total_voxels)
        rospy.Timer(rospy.Duration(3.0), self._heartbeat)

    def _cb(self, msg):
        # PointCloud2 carries width*height points; that *is* the voxel count.
        n = msg.width * msg.height
        t = rospy.Time.now().to_sec()
        with self.lock:
            if self.t0 is None:
                self.t0 = t
                rospy.loginfo("monitor: first msg on %s, %d voxels",
                              self.topic, n)
            self.times.append(t - self.t0)
            self.counts.append(n)
            self.msg_count += 1

    def _heartbeat(self, _):
        with self.lock:
            n = self.msg_count
            last = self.counts[-1] if self.counts else 0
        if n == 0:
            rospy.logwarn("monitor: no msgs on %s yet — wrong topic? "
                          "try `rostopic list | grep -Ei occupancy\\|voxel\\|sdf`",
                          self.topic)
        else:
            rospy.loginfo_throttle(10.0,
                "monitor: %d msgs received, latest count=%d", n, last)

    def snapshot(self):
        with self.lock:
            return np.asarray(self.times), np.asarray(self.counts, dtype=float)


def main():
    mon = ExplorationMonitor()

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    fig.suptitle("FALCON exploration progress")

    (line_pct,)  = ax1.plot([], [], lw=2)
    ax1.set_ylabel("Explored (%)")
    ax1.set_ylim(0, 100)
    ax1.grid(True, alpha=0.3)

    (line_rate,) = ax2.plot([], [], lw=2, color="tab:green")
    ax2.set_ylabel("New voxels / sec")
    ax2.set_xlabel("Time (s)")
    ax2.grid(True, alpha=0.3)

    WIN = 5  # samples used for rate smoothing — bigger = smoother

    def update(_):
        t, c = mon.snapshot()
        if len(t) < 2:
            return line_pct, line_rate

        # (1) percentage curve
        line_pct.set_data(t, 100.0 * c / mon.total_voxels)

        # (2) discovery rate, smoothed by finite difference over WIN samples
        k = min(WIN, len(t) - 1)
        dt = t[k:] - t[:-k]
        dc = c[k:] - c[:-k]
        rate = np.where(dt > 0, dc / dt, 0.0)
        line_rate.set_data(t[k:], rate)

        ax1.set_xlim(0, max(t[-1], 1.0))
        ax2.relim(); ax2.autoscale_view(scaley=True, scalex=False)
        return line_pct, line_rate

    ani = FuncAnimation(fig, update, interval=500,
                        blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
    rospy.signal_shutdown("plot window closed")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass