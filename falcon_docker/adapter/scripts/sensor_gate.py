#!/usr/bin/env python3
"""
sensor_gate.py — Pose+depth pass-through that can be FROZEN.

Sits between Gazebo (or the ros1_bridge, or a real drone) and
falcon_adapter.

   <pose topic>         ──┐
   <depth topic>        ──┤   →  /<out_ns>/gt_pose
   <camera_info topic>  ──┘                    /<out_ns>/front_depth/depth/image_raw
                                               /<out_ns>/front_depth/depth/camera_info
                                               (camera_info path only when
                                                ~bridge_camera_info=true)

By default the input topics are derived from `~in_ns` using the
sjtu_drone-style suffixes (/gt_pose, /front_depth/depth/image_raw,
/front_depth/depth/camera_info).

For a real drone whose topic names don't match that convention,
override them explicitly with these private params:
  ~pose_topic         (str)   default <in_ns>/gt_pose
  ~depth_topic        (str)   default <in_ns>/front_depth/depth/image_raw
  ~camera_info_topic  (str)   default <in_ns>/front_depth/depth/camera_info

CAMERA_INFO BRIDGING (~bridge_camera_info, bool, default False)
  When False (the new default), sensor_gate does NOT subscribe to
  camera_info and does NOT publish /<out_ns>/front_depth/depth/camera_info.
  FALCON's exploration_node reads camera intrinsics from rosparam
  (/uav_model/sensing_parameters/camera_intrinsics/*), not from this
  topic, so on a real-drone run where the intrinsics are static there
  is nothing to do here.
  Set True to restore the old pass-through behaviour.

The pose input MUST be geometry_msgs/Pose. If your real drone
publishes PoseStamped or Odometry, run pose_adapter.py upstream
of this node — that keeps every other consumer in the stack
unchanged.

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

        # When False (default), camera_info is neither subscribed nor
        # republished. Intrinsics come from rosparam instead.
        self.bridge_caminfo = bool(G("~bridge_camera_info", False))

        # Topic resolution: explicit private param wins, otherwise
        # fall back to the sjtu_drone-style ns/suffix default.
        self.in_pose_t    = G("~pose_topic",
                              self.in_ns + "/gt_pose")
        self.in_depth_t   = G("~depth_topic",
                              self.in_ns + "/front_depth/depth/image_raw")
        self.in_caminfo_t = G("~camera_info_topic",
                              self.in_ns + "/front_depth/depth/camera_info")

        self.frozen        = False
        self.last_pose     = None
        self.last_depth    = None
        self.last_caminfo  = None

        # Publishers — gated namespace (suffixes stay constant so
        # falcon_adapter sees the same paths as before)
        self.pub_pose    = rospy.Publisher(
            self.out_ns + "/gt_pose", Pose, queue_size=1)
        self.pub_depth   = rospy.Publisher(
            self.out_ns + "/front_depth/depth/image_raw",
            Image, queue_size=2)
        self.pub_caminfo = None
        if self.bridge_caminfo:
            self.pub_caminfo = rospy.Publisher(
                self.out_ns + "/front_depth/depth/camera_info",
                CameraInfo, queue_size=2)

        # Subscribers — live drone topics (resolved above)
        rospy.Subscriber(self.in_pose_t,    Pose,
                         self._pose_cb,    queue_size=10)
        rospy.Subscriber(self.in_depth_t,   Image,
                         self._depth_cb,   queue_size=2)
        if self.bridge_caminfo:
            rospy.Subscriber(self.in_caminfo_t, CameraInfo,
                             self._caminfo_cb, queue_size=2)
        rospy.Subscriber("/sensor_gate/freeze", Bool,
                         self._freeze_cb, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / self.replay_hz), self._replay)

        rospy.loginfo("=" * 64)
        rospy.loginfo("sensor_gate ready  in_ns=%s  out_ns=%s  replay=%.0fHz",
                      self.in_ns, self.out_ns, self.replay_hz)
        rospy.loginfo("  in pose        = %s", self.in_pose_t)
        rospy.loginfo("  in depth       = %s", self.in_depth_t)
        if self.bridge_caminfo:
            rospy.loginfo("  in camera_info = %s", self.in_caminfo_t)
        else:
            rospy.loginfo("  in camera_info = DISABLED "
                          "(bridge_camera_info=false; intrinsics from rosparam)")
        rospy.loginfo("=" * 64)

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
        # Only reached when bridge_camera_info=true (subscriber not
        # created otherwise).
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
        if self.pub_caminfo is not None and self.last_caminfo is not None:
            self.last_caminfo.header.stamp = now
            self.pub_caminfo.publish(self.last_caminfo)


if __name__ == "__main__":
    try:
        SensorGate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass