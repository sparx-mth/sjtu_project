#!/usr/bin/env python3
"""
pose_adapter.py — convert real-drone localization → bare geometry_msgs/Pose.

Real-drone localization sources almost never publish a bare Pose:
  - MAVROS (PX4 / ArduPilot):  /mavros/local_position/pose       PoseStamped
                               /mavros/local_position/odom       Odometry
  - VIO (VINS-Fusion, etc.):   /vins_estimator/odometry          Odometry
  - Visual SLAM (ORB-SLAM3):   /orb_slam3/camera_pose            PoseStamped
  - Mocap (Vicon/OptiTrack):   /vrpn_client_node/<obj>/pose      PoseStamped

But every consumer in this stack — sensor_gate, falcon_adapter,
astar_planner, waypoint_follower, bev_click_goal — was written
against Gazebo's sjtu_drone, which publishes a bare geometry_msgs/Pose
on /<drone_ns>/gt_pose.

This node subscribes to whatever your real drone publishes and
republishes a bare Pose on /<drone_ns>/gt_pose, so the rest of the
graph needs zero changes between Gazebo and real flight.

Params (all private):
  ~in_topic   (str)   topic to subscribe to                (default /mavros/local_position/pose)
  ~in_type    (str)   "pose_stamped" | "odometry" | "pose" (default pose_stamped)
  ~out_topic  (str)   topic to publish on                  (default /simple_drone/gt_pose)

Run standalone:
  rosrun falcon_adapter pose_adapter.py \
      _in_topic:=/mavros/local_position/pose \
      _in_type:=pose_stamped \
      _out_topic:=/simple_drone/gt_pose
"""
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry


class PoseAdapter:
    def __init__(self):
        rospy.init_node("pose_adapter")
        G = rospy.get_param

        self.in_topic  = G("~in_topic",  "/mavros/local_position/pose")
        self.in_type   = G("~in_type",   "pose_stamped").lower()
        self.out_topic = G("~out_topic", "/simple_drone/gt_pose")

        self.pub = rospy.Publisher(self.out_topic, Pose, queue_size=10)

        if self.in_type == "pose_stamped":
            rospy.Subscriber(self.in_topic, PoseStamped,
                             self._stamped_cb, queue_size=10)
        elif self.in_type == "odometry":
            rospy.Subscriber(self.in_topic, Odometry,
                             self._odom_cb, queue_size=10)
        elif self.in_type == "pose":
            # Pure pass-through. Useful only for renaming.
            rospy.Subscriber(self.in_topic, Pose,
                             self._pose_cb, queue_size=10)
        else:
            rospy.logfatal("pose_adapter: unknown ~in_type=%r  "
                           "(use pose_stamped | odometry | pose)",
                           self.in_type)
            raise RuntimeError("bad in_type")

        rospy.loginfo("=" * 64)
        rospy.loginfo("pose_adapter ready")
        rospy.loginfo("  in  = %s   (type=%s)", self.in_topic, self.in_type)
        rospy.loginfo("  out = %s   (geometry_msgs/Pose)", self.out_topic)
        rospy.loginfo("=" * 64)

    # ── Subscribers ──────────────────────────────────────────────
    def _stamped_cb(self, msg):
        self.pub.publish(msg.pose)

    def _odom_cb(self, msg):
        self.pub.publish(msg.pose.pose)

    def _pose_cb(self, msg):
        self.pub.publish(msg)


if __name__ == "__main__":
    try:
        PoseAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass