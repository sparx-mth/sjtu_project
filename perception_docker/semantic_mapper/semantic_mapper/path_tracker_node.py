#!/usr/bin/env python3
"""
path_tracker_node.py — Quadrotor Pure Pursuit on /planned_path.

Subscribes:
  /planned_path             nav_msgs/Path
  /odom_world               nav_msgs/Odometry        (bridged from FALCON)
  /path_tracker/enable      std_msgs/Bool            (optional kill switch)

Publishes:
  /simple_drone/cmd_vel     geometry_msgs/Twist      (body-frame, bridged to ROS1)
  /path_tracker/lookahead   visualization_msgs/Marker

Approach:
  * Holonomic in xy: pick a carrot at lookahead L along the path, rotate
    the world-frame error into body frame for linear.x / linear.y. Yaw is
    steered separately to face the carrot.
  * Monotonic progress index along the path prevents backtracking.
  * Altitude tracks the carrot's z (planner stamps z = current odom z).

Handoff behaviour (matters when FALCON resumes control after arrival):
  * On goal arrival, publish ONE zero Twist and latch a `_handed_off`
    flag. While latched, the tracker emits NOTHING on /cmd_vel — it
    completely steps off the channel so FALCON can drive the drone for
    in-room exploration.
  * The latch resets only when a fresh /planned_path arrives, at which
    point the tracker resumes normal operation.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


class PathTrackerNode(Node):
    def __init__(self):
        super().__init__('path_tracker')

        # ── Params ──
        self.declare_parameter('path_topic',     '/planned_path')
        self.declare_parameter('odom_topic',     '/odom_world')
        self.declare_parameter('cmd_vel_topic',  '/simple_drone/cmd_vel')
        self.declare_parameter('marker_topic',   '/path_tracker/lookahead')
        self.declare_parameter('frame_id',       'world')
        self.declare_parameter('lookahead_m',    0.8)
        self.declare_parameter('max_lin_vel',    0.2)   # match FALCON dynamics cap
        self.declare_parameter('max_ang_vel',    0.6)
        self.declare_parameter('goal_tol_m',     0.3)
        self.declare_parameter('kp_yaw',         1.2)
        self.declare_parameter('kp_z',           0.8)
        self.declare_parameter('control_hz',     20.0)

        gp = lambda k: self.get_parameter(k).value
        self.path_topic    = gp('path_topic')
        self.odom_topic    = gp('odom_topic')
        self.cmd_vel_topic = gp('cmd_vel_topic')
        self.marker_topic  = gp('marker_topic')
        self.frame_id      = gp('frame_id')
        self.L             = float(gp('lookahead_m'))
        self.v_max         = float(gp('max_lin_vel'))
        self.w_max         = float(gp('max_ang_vel'))
        self.goal_tol      = float(gp('goal_tol_m'))
        self.kp_yaw        = float(gp('kp_yaw'))
        self.kp_z          = float(gp('kp_z'))

        # ── State ──
        self.waypoints: np.ndarray | None = None  # (N, 3)
        self.progress = 0
        self._handed_off = False     # True after arrival; suppresses cmd_vel
        self.cur_xyz: np.ndarray | None = None
        self.cur_yaw = 0.0
        self.enabled = True
        self._stats = dict(ticks=0, cmds=0, idle=0, handed_off=0)

        # ── ROS ──
        self.cmd_pub    = self.create_publisher(Twist,  self.cmd_vel_topic, 1)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic,  1)
        self.create_subscription(Path,     self.path_topic,         self._path_cb,   1)
        self.create_subscription(Odometry, self.odom_topic,         self._odom_cb,  10)
        self.create_subscription(Bool,     '/path_tracker/enable',  self._enable_cb, 1)

        self.create_timer(1.0 / float(gp('control_hz')), self._tick)
        self.create_timer(2.0, self._heartbeat)

        self.get_logger().info(
            f"path_tracker ready  path={self.path_topic}  odom={self.odom_topic}  "
            f"cmd={self.cmd_vel_topic}  L={self.L:.2f}m  v_max={self.v_max:.2f}m/s")

    # ── Callbacks ──

    def _path_cb(self, msg: Path):
        if not msg.poses:
            self.waypoints = None
            self._publish_zero()
            return
        self.waypoints = np.array(
            [[p.pose.position.x, p.pose.position.y, p.pose.position.z]
             for p in msg.poses], dtype=np.float64)
        self.progress = 0
        # New path → take the channel back from FALCON.
        if self._handed_off:
            self.get_logger().info("new path received — taking back cmd_vel")
        self._handed_off = False
        self.get_logger().info(
            f"new path  {len(self.waypoints)} wp  "
            f"goal=({self.waypoints[-1,0]:.2f},{self.waypoints[-1,1]:.2f},"
            f"{self.waypoints[-1,2]:.2f})")

    def _odom_cb(self, msg: Odometry):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        self.cur_xyz = np.array([p.x, p.y, p.z])
        self.cur_yaw = _yaw_from_quat(o)

    def _enable_cb(self, msg: Bool):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self._publish_zero()

    # ── Control tick ──

    def _tick(self):
        self._stats['ticks'] += 1
        if not self.enabled or self.waypoints is None or self.cur_xyz is None:
            self._stats['idle'] += 1
            return

        # Once we've arrived and handed off, stay completely silent on
        # /cmd_vel until a fresh path arrives. FALCON owns the channel.
        if self._handed_off:
            self._stats['handed_off'] += 1
            return

        # Advance progress past segments we've crossed.
        wp = self.waypoints
        N = len(wp)
        while self.progress < N - 1:
            seg = wp[self.progress + 1] - wp[self.progress]
            seg_len2 = float(np.dot(seg, seg))
            if seg_len2 < 1e-9:
                self.progress += 1; continue
            t = float(np.dot(self.cur_xyz - wp[self.progress], seg) / seg_len2)
            if t >= 1.0:
                self.progress += 1
            else:
                break

        # Goal reached? Publish ONE final stop, then go silent.
        goal_xy = wp[-1, :2]
        dist_to_goal = float(np.linalg.norm(self.cur_xyz[:2] - goal_xy))
        if dist_to_goal < self.goal_tol:
            self._publish_zero()
            self._handed_off = True
            self.get_logger().info(
                f"goal reached  d={dist_to_goal:.2f}m — silent on cmd_vel "
                f"(FALCON has control)")
            return

        # Carrot at lookahead L.
        carrot = self._lookahead()

        # World-frame error → body-frame velocity (holonomic xy).
        dxw = float(carrot[0] - self.cur_xyz[0])
        dyw = float(carrot[1] - self.cur_xyz[1])
        d_xy = math.hypot(dxw, dyw)
        if d_xy < 1e-6:
            self._publish_zero()
            return

        speed = self.v_max * min(1.0, dist_to_goal / max(self.L, 1e-3))
        ux, uy = dxw / d_xy, dyw / d_xy
        c, s   = math.cos(-self.cur_yaw), math.sin(-self.cur_yaw)
        vx_b   = (c * ux - s * uy) * speed
        vy_b   = (s * ux + c * uy) * speed

        target_yaw = math.atan2(dyw, dxw)
        wz = _clamp(self.kp_yaw * _wrap(target_yaw - self.cur_yaw), self.w_max)
        vz = _clamp(self.kp_z * float(carrot[2] - self.cur_xyz[2]), self.v_max)

        cmd = Twist()
        cmd.linear.x  = vx_b
        cmd.linear.y  = vy_b
        cmd.linear.z  = vz
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)
        self._stats['cmds'] += 1
        self._publish_marker(carrot)

    # ── Lookahead ──

    def _lookahead(self) -> np.ndarray:
        wp = self.waypoints
        N = len(wp)
        p = self.cur_xyz[:2]
        L = self.L
        for i in range(self.progress, N - 1):
            a, b = wp[i], wp[i + 1]
            if np.linalg.norm(b[:2] - p) < L:
                continue
            d = b[:2] - a[:2]
            f = a[:2] - p
            A = float(np.dot(d, d))
            if A < 1e-9:
                return b
            B = 2.0 * float(np.dot(f, d))
            C = float(np.dot(f, f)) - L * L
            disc = B * B - 4.0 * A * C
            if disc < 0:
                return b
            t = max(0.0, min(1.0, (-B + math.sqrt(disc)) / (2.0 * A)))
            return a + t * (b - a)
        return wp[-1]

    # ── Output helpers ──

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _publish_marker(self, p: np.ndarray):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'lookahead', 0
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x = float(p[0])
        m.pose.position.y = float(p[1])
        m.pose.position.z = float(p[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        self.marker_pub.publish(m)

    def _heartbeat(self):
        s = self._stats
        N = len(self.waypoints) if self.waypoints is not None else 0
        ho = "  HANDED-OFF" if self._handed_off else ""
        self.get_logger().info(
            f"tracker hb  ticks={s['ticks']} cmds={s['cmds']} "
            f"idle={s['idle']} handed_off={s['handed_off']}  "
            f"path={N}wp idx={self.progress}{ho}")
        self._stats = dict(ticks=0, cmds=0, idle=0, handed_off=0)


def main():
    rclpy.init()
    node = PathTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Final zero on shutdown so the drone halts cleanly (only if we're
    # not already handed off — otherwise we don't want to step on FALCON).
    try:
        if not node._handed_off:
            node._publish_zero()
    except Exception:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()