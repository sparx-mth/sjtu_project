#!/usr/bin/env python3
"""
completion_watcher.py — touches a flag file when FALCON exploration completes.

Subscribes
──────────
    /planning/replan  (std_msgs/Int32)
        FALCON's traj_server publishes data=2 when there are no more
        frontiers, i.e. exploration is done. cmd_to_vel.py uses the
        same constant to enter its DONE state (see lines 248–251).

Params (private)
────────────────
    ~run_name      run subdirectory name; matches run_recorder ~run_name
    ~output_dir    parent dir;            matches run_recorder ~output_dir
    ~settle_sec    ignore /planning/replan messages received during the
                   first ~settle_sec seconds. Some FALCON variants
                   publish data=2 transiently at startup (before the
                   first frontier is found). Default: 30 s.

Output
──────
    On the first qualifying message, creates:
        <output_dir>/<run_name>/.exploration_done
    containing one line:  <wall_clock_unix>,<t_run_sec>

Why a flag file
───────────────
    Lets batch_runner.py be a plain subprocess driver — no rospy, no
    persistent ROS master across iterations. The orchestrator polls the
    filesystem; this node owns the topic.
"""

import os
import time

import rospy
from std_msgs.msg import Int32

EXPLORATION_DONE = 2  # /planning/replan value FALCON emits when finished


class Watcher:
    def __init__(self):
        rospy.init_node("completion_watcher")
        self.run_name   = rospy.get_param("~run_name", "unnamed")
        self.output_dir = rospy.get_param("~output_dir", "/home/falcon/runs")
        self.settle_sec = float(rospy.get_param("~settle_sec", 30.0))

        self.run_dir = os.path.join(self.output_dir, self.run_name)
        os.makedirs(self.run_dir, exist_ok=True)
        self.flag_path = os.path.join(self.run_dir, ".exploration_done")

        # Remove any stale flag from a previous attempt with the same name
        if os.path.exists(self.flag_path):
            try:
                os.remove(self.flag_path)
            except OSError:
                pass

        self.t0 = rospy.Time.now()
        self.done = False

        rospy.Subscriber("/planning/replan", Int32, self._cb, queue_size=10)
        rospy.loginfo("completion_watcher: run=%s flag=%s settle=%.1fs",
                      self.run_name, self.flag_path, self.settle_sec)

    def _cb(self, msg):
        if self.done:
            return
        elapsed = (rospy.Time.now() - self.t0).to_sec()
        if elapsed < self.settle_sec:
            return
        if msg.data != EXPLORATION_DONE:
            return

        self.done = True
        try:
            with open(self.flag_path, "w") as f:
                f.write("{:.6f},{:.3f}\n".format(time.time(), elapsed))
        except OSError as e:
            rospy.logerr("completion_watcher: failed to write flag: %s", e)
            return
        rospy.loginfo(
            "completion_watcher: exploration_done at t=%.1fs → %s",
            elapsed, self.flag_path)


if __name__ == "__main__":
    Watcher()
    rospy.spin()