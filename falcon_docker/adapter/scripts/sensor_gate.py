#!/usr/bin/env python3
"""
sensor_gate.py — Pose+depth pass-through that can be FROZEN.

Sits between Gazebo (or the ros1_bridge) and falcon_adapter.

   /<in_ns>/gt_pose                       ──┐
   /<in_ns>/front_depth/depth/image_raw   ──┤   →  /<out_ns>/...
   /<in_ns>/front_depth/depth/camera_info ──┘

Default mode: pass-through (each callback republishes immediately).

While `/sensor_gate/freeze` is True:
  - new incoming msgs are NOT republished and the cache is NOT updated
  - a `replay_hz` timer republishes the LAST msg seen before the freeze,
    with header.stamp refreshed for stamped types so downstream age
    checks don't fail.

Why: during in-place YAW rotations the depth camera is moving fast and
not pointing at meaningful new geometry. Letting those frames into
FALCON's voxel mapping erodes walls and adds noise. Freezing the gate
during YAW makes the voxel map effectively pause until forward flight
resumes; falcon_adapter never has to know.

Point falcon_adapter at out_ns by setting falcon_adapter's
~drone_ns parameter to whatever this gate's `out_ns` is.
"""
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo


class SensorGate:
    def __init__(self):
        rospy.init_node("sensor_gate")
        G = rospy.get_param

        self.in_ns     = G("~in_ns",   "/simple_drone")
        self.out_ns    = G("~out_ns",  "/gated_drone")
        self.replay_hz = float(G("~replay_hz", 30.0))

        self.frozen        = False
        self.last_pose     = None
        self.last_depth    = None
        self.last_caminfo  = None

        # Publishers — gated namespace
        self.pub_pose    = rospy.Publisher(
            self.out_ns + "/gt_pose", Pose, queue_size=1)
        self.pub_depth   = rospy.Publisher(
            self.out_ns + "/front_depth/depth/image_raw",
            Image, queue_size=2)
        self.pub_caminfo = rospy.Publisher(
            self.out_ns + "/front_depth/depth/camera_info",
            CameraInfo, queue_size=2)

        # Subscribers — live drone namespace
        rospy.Subscriber(self.in_ns + "/gt_pose", Pose,
                         self._pose_cb,    queue_size=10)
        rospy.Subscriber(self.in_ns + "/front_depth/depth/image_raw",
                         Image, self._depth_cb,   queue_size=2)
        rospy.Subscriber(self.in_ns + "/front_depth/depth/camera_info",
                         CameraInfo, self._caminfo_cb, queue_size=2)
        rospy.Subscriber("/sensor_gate/freeze", Bool,
                         self._freeze_cb, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / self.replay_hz), self._replay)

        rospy.loginfo("sensor_gate ready  in=%s  out=%s  replay=%.0fHz",
                      self.in_ns, self.out_ns, self.replay_hz)

    # ── Subscribers ──────────────────────────────────────────────
    def _freeze_cb(self, msg):
        new = bool(msg.data)
        if new != self.frozen:
            rospy.loginfo("sensor_gate: %s",
                          "FREEZE" if new else "unfreeze")
        self.frozen = new

    def _pose_cb(self, msg):
        # While frozen, we deliberately drop live updates so the cache
        # holds the snapshot from the moment freezing began.
        if self.frozen:
            return
        self.last_pose = msg
        self.pub_pose.publish(msg)

    def _depth_cb(self, msg):
        if self.frozen:
            return
        self.last_depth = msg
        self.pub_depth.publish(msg)

    def _caminfo_cb(self, msg):
        if self.frozen:
            return
        self.last_caminfo = msg
        self.pub_caminfo.publish(msg)

    # ── Replay during freeze ─────────────────────────────────────
    def _replay(self, _evt):
        if not self.frozen:
            return
        now = rospy.Time.now()
        if self.last_pose is not None:
            self.pub_pose.publish(self.last_pose)
        if self.last_depth is not None:
            self.last_depth.header.stamp = now
            self.pub_depth.publish(self.last_depth)
        if self.last_caminfo is not None:
            self.last_caminfo.header.stamp = now
            self.pub_caminfo.publish(self.last_caminfo)


if __name__ == "__main__":
    try:
        SensorGate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass