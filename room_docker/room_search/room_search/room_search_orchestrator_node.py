#!/usr/bin/env python3
"""
room_search_orchestrator_node.py — go to a room, rotate to find a target,
                                   close in on it, land.

State machine
-------------
  WAIT_INIT          have we seen pose yet? have a goal+target? if so → NAV_TO_ROOM
  NAV_TO_ROOM        publish (room_center_x, room_center_y) to /waypoint_nav/goal,
                     watch drone pose, wait until within nav_arrival_radius_m.
                     Re-publish the goal periodically in case the path planner
                     hadn't started up yet on first publish.
                     → ROTATE_AND_SEARCH
  ROTATE_AND_SEARCH  take over /cmd_vel (publish external_ctrl=True so the
                     ROS1 waypoint_follower stops emitting zeros). Spin in
                     place at rotation_rate_rad_s while watching /target_seen.
                     - hit  → release ctrl, → APPROACH_TARGET
                     - timeout after max_rotation_revs full revolutions
                       → GIVE_UP
  APPROACH_TARGET    publish target XY to /waypoint_nav/goal and let the
                     existing A* + waypoint_follower stack drive there.
                     Continue listening to /perception/objects in case the
                     averaged XY refines, and republish if it shifts by more
                     than goal_refine_threshold_m. Wait for drone pose within
                     approach_radius_m of current target XY.
                     → LAND
  LAND               publish a short burst of /<drone_ns>/land (Empty).
                     The sjtu_drone landing controller owns the descent.
                     → DONE
  DONE               idle.
  GIVE_UP            idle, log a warning.

What we expect on the wire
--------------------------
  IN  (ROS2 native, from perception_docker/semantic_mapper):
    /target_seen          std_msgs/Bool        TRANSIENT_LOCAL
    /target_seen/info     std_msgs/String      TRANSIENT_LOCAL (JSON)
    /perception/objects   std_msgs/String      TRANSIENT_LOCAL (JSON, refined XY)

  IN  (must be bridged from ROS1):
    pose_topic            nav_msgs/Odometry  or  geometry_msgs/PoseStamped
                          - default: /odom_world (Odometry; already published
                            by falcon_adapter, easy to bridge)

  OUT (must be bridged to ROS1, consumed by falcon_adapter stack):
    /waypoint_nav/goal                   geometry_msgs/Point
    /<drone_ns>/cmd_vel                  geometry_msgs/Twist
    /<drone_ns>/land                     std_msgs/Empty
    /waypoint_follower/external_ctrl     std_msgs/Bool   (latched)

Bridge note
-----------
parameter_bridge requires every bridged topic to be listed in bridge.yaml.
The four "OUT" topics above are typically not in the default bridge config —
see room_docker/README.md for the snippet to add.
"""

import json
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry


class S:
    WAIT_INIT         = "WAIT_INIT"
    NAV_TO_ROOM       = "NAV_TO_ROOM"
    ROTATE_AND_SEARCH = "ROTATE_AND_SEARCH"
    APPROACH_TARGET   = "APPROACH_TARGET"
    LAND              = "LAND"
    DONE              = "DONE"
    GIVE_UP           = "GIVE_UP"


def _now_s(node: Node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


class RoomSearchOrchestrator(Node):
    def __init__(self):
        super().__init__("room_search_orchestrator")

        P = self.declare_parameter

        # ── Mission ───────────────────────────────────────────────
        # Room center to navigate to first (world frame, metres).
        P("room_center_x",        4.0)
        P("room_center_y",        5.0)
        # Target object label. The actual fuzzy matching to YOLO classes
        # is done by target_watcher_node — this node only reads
        # /target_seen and /target_seen/info. Keep this in sync with
        # target_watcher_node's target_object parameter (set both from
        # the launch file).
        P("target_object",        "keyboard")
        # drone_ns is the prefix for cmd_vel / land. Empty string ("")
        # works for the real-drone case where /cmd_vel and /land sit at
        # root. Set to "/simple_drone" for the Gazebo sim.
        P("drone_ns",             "/simple_drone")

        # ── Topic names (override per environment) ────────────────
        # Drone pose source. We accept Odometry (default; published by
        # falcon_adapter as /odom_world) OR PoseStamped OR bare Pose.
        # Set pose_type to "odometry" | "pose_stamped" | "pose".
        P("pose_topic",           "/odom_world")
        P("pose_type",             "odometry")

        # ── Phase radii / timings ────────────────────────────────
        # Acceptance circle for "we arrived at the room center"
        P("nav_arrival_radius_m",  0.50)
        # Max time to wait for nav arrival before logging a warning
        # (we keep waiting; this is just a heartbeat threshold).
        P("nav_arrival_warn_s",    60.0)
        # Period for re-publishing the nav goal while in NAV_TO_ROOM
        # (so a late-starting A* planner eventually sees it).
        P("nav_goal_republish_s",  3.0)

        # Rotation rate during ROTATE_AND_SEARCH (rad/s, positive = CCW)
        P("rotation_rate_rad_s",   0.5)
        # Cmd-vel publish rate during rotation
        P("rotation_ctrl_hz",     20.0)
        # Give up rotating after this many full revolutions (~720° default)
        P("max_rotation_revs",     2.0)

        # Acceptance circle for "we reached the target"
        P("approach_radius_m",     0.35)
        # Re-publish goal if averaged target XY shifts by more than this
        P("goal_refine_threshold_m", 0.30)
        # Stop refining + republishing once we're within this of the goal —
        # last-moment jitter shouldn't keep poking the path planner.
        P("approach_lock_radius_m", 0.60)
        # Same as nav_goal_republish_s but for the approach phase
        P("approach_goal_republish_s", 3.0)
        # Approach phase hard timeout
        P("approach_timeout_s",   90.0)

        # Land phase: how many /land Empty msgs to publish, at what rate
        P("land_burst_count",      10)
        P("land_burst_hz",        10.0)

        # Top-level tick rate for the state machine
        P("tick_hz",               5.0)

        g = lambda n: self.get_parameter(n).value
        self.room_xy: Tuple[float, float] = (
            float(g("room_center_x")), float(g("room_center_y")))
        self.target_label  = str(g("target_object"))
        self.drone_ns      = str(g("drone_ns"))
        self.pose_topic    = str(g("pose_topic"))
        self.pose_type     = str(g("pose_type")).lower()

        self.nav_arr_r      = float(g("nav_arrival_radius_m"))
        self.nav_warn_s     = float(g("nav_arrival_warn_s"))
        self.nav_republ_s   = float(g("nav_goal_republish_s"))

        self.rot_rate       = float(g("rotation_rate_rad_s"))
        self.rot_hz         = float(g("rotation_ctrl_hz"))
        self.max_revs       = float(g("max_rotation_revs"))

        self.approach_r     = float(g("approach_radius_m"))
        self.refine_thr     = float(g("goal_refine_threshold_m"))
        self.lock_r         = float(g("approach_lock_radius_m"))
        self.appr_republ_s  = float(g("approach_goal_republish_s"))
        self.appr_to_s      = float(g("approach_timeout_s"))

        self.land_n         = int(g("land_burst_count"))
        self.land_hz        = float(g("land_burst_hz"))
        self.tick_hz        = float(g("tick_hz"))

        # ── State ────────────────────────────────────────────────
        self.state        = S.WAIT_INIT
        self.t_state      = _now_s(self)
        self.cur_xy: Optional[Tuple[float, float]] = None
        self.target_xy: Optional[Tuple[float, float]] = None
        self.target_oid: Optional[int] = None
        self.target_seen  = False
        self.last_nav_pub = 0.0
        self.rot_yaw_acc  = 0.0      # cumulative rotation, rad (for max_revs)
        self._last_tick_t = _now_s(self)
        self._land_pubs_left = 0
        self._appr_locked = False    # stop refining goal once close

        # ── QoS ──────────────────────────────────────────────────
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Publishers (most are bridged to ROS1) ────────────────
        self.pub_goal = self.create_publisher(
            Point, "/waypoint_nav/goal", 1)
        self.pub_cmd  = self.create_publisher(
            Twist, self.drone_ns + "/cmd_vel", 1)
        self.pub_land = self.create_publisher(
            Empty, self.drone_ns + "/land", 1)
        # External-ctrl is latched so a late-arriving waypoint_follower
        # (or one that was restarted) immediately sees current state.
        self.pub_ext  = self.create_publisher(
            Bool, "/waypoint_follower/external_ctrl", latched)

        # Publish initial external_ctrl=False so the latched topic exists
        # before the first state transition reads/writes it.
        self.pub_ext.publish(Bool(data=False))
        self._ext_state = False

        # ── Subscribers ──────────────────────────────────────────
        # Drone pose source.
        if self.pose_type == "odometry":
            self.create_subscription(
                Odometry, self.pose_topic, self._odom_cb, sensor)
        elif self.pose_type == "pose_stamped":
            self.create_subscription(
                PoseStamped, self.pose_topic, self._ps_cb, sensor)
        elif self.pose_type == "pose":
            self.create_subscription(
                Pose, self.pose_topic, self._pose_cb, sensor)
        else:
            self.get_logger().fatal(
                f"pose_type={self.pose_type!r} not in "
                "{'odometry','pose_stamped','pose'}")
            raise RuntimeError("bad pose_type")

        # Target signals from target_watcher_node.
        self.create_subscription(
            Bool,   "/target_seen",        self._seen_cb,    latched)
        self.create_subscription(
            String, "/target_seen/info",   self._info_cb,    latched)
        # Live objects list (lets us refine target XY as more views average).
        self.create_subscription(
            String, "/perception/objects", self._objects_cb, latched)

        # ── Timers ───────────────────────────────────────────────
        self.create_timer(1.0 / self.tick_hz, self._tick)
        # Rotation control runs on a faster timer when active.
        self._rot_timer = self.create_timer(
            1.0 / self.rot_hz, self._rotate_tick)
        # Land burst timer is created lazily on entry to LAND.

        self.create_timer(5.0, self._hb)

        self.get_logger().info("=" * 64)
        self.get_logger().info("room_search_orchestrator ready")
        self.get_logger().info(
            f"  room_center = ({self.room_xy[0]:.2f}, {self.room_xy[1]:.2f}) m")
        self.get_logger().info(f"  target_object = {self.target_label!r}")
        self.get_logger().info(
            f"  drone_ns = {self.drone_ns!r}   "
            f"pose = {self.pose_topic} ({self.pose_type})")
        self.get_logger().info(
            f"  nav_arrival_radius={self.nav_arr_r:.2f}m  "
            f"approach_radius={self.approach_r:.2f}m  "
            f"rot_rate={self.rot_rate:.2f}rad/s  "
            f"max_revs={self.max_revs:.1f}")
        self.get_logger().info("=" * 64)

    # ─── State helpers ──────────────────────────────────────────────
    def _enter(self, new: str):
        if new == self.state:
            return
        self.get_logger().info(
            f"room_search: {self.state} -> {new}")
        self.state = new
        self.t_state = _now_s(self)
        # Per-state entry hooks
        if new == S.NAV_TO_ROOM:
            self._publish_nav_goal(self.room_xy)
            self.last_nav_pub = _now_s(self)
        elif new == S.ROTATE_AND_SEARCH:
            self._set_external_ctrl(True)
            self.rot_yaw_acc = 0.0
        elif new == S.APPROACH_TARGET:
            self._set_external_ctrl(False)
            if self.target_xy is not None:
                self._publish_nav_goal(self.target_xy)
                self.last_nav_pub = _now_s(self)
            self._appr_locked = False
        elif new == S.LAND:
            self._set_external_ctrl(False)
            self._land_pubs_left = max(1, int(self.land_n))
            period = 1.0 / max(self.land_hz, 1.0)
            self._land_timer = self.create_timer(period, self._land_tick)
        elif new in (S.DONE, S.GIVE_UP):
            self._set_external_ctrl(False)

    def _t_in(self) -> float:
        return _now_s(self) - self.t_state

    # ─── Outgoing primitives ────────────────────────────────────────
    def _publish_nav_goal(self, xy: Tuple[float, float]):
        m = Point()
        m.x = float(xy[0]); m.y = float(xy[1]); m.z = 0.0
        self.pub_goal.publish(m)
        self.get_logger().info(
            f"room_search: nav goal -> ({xy[0]:.2f}, {xy[1]:.2f})")

    def _set_external_ctrl(self, want: bool):
        if want == self._ext_state:
            return
        self.pub_ext.publish(Bool(data=bool(want)))
        self._ext_state = bool(want)
        self.get_logger().info(
            f"room_search: external_ctrl -> {want}")

    def _publish_cmd(self, vx: float, wz: float):
        m = Twist()
        m.linear.x  = float(vx)
        m.linear.y  = 0.0
        m.linear.z  = 0.0
        m.angular.x = 0.0
        m.angular.y = 0.0
        m.angular.z = float(wz)
        self.pub_cmd.publish(m)

    # ─── Pose callbacks ─────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.cur_xy = (float(p.x), float(p.y))

    def _ps_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.cur_xy = (float(p.x), float(p.y))

    def _pose_cb(self, msg: Pose):
        self.cur_xy = (float(msg.position.x), float(msg.position.y))

    # ─── Target callbacks ───────────────────────────────────────────
    def _seen_cb(self, msg: Bool):
        if bool(msg.data) and not self.target_seen:
            self.get_logger().info(
                "room_search: /target_seen=True received")
        self.target_seen = bool(msg.data) or self.target_seen

    def _info_cb(self, msg: String):
        # Authoritative source of the first matched object's XY + id.
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        xy = d.get("xy")
        if not (isinstance(xy, (list, tuple)) and len(xy) >= 2):
            return
        new_xy = (float(xy[0]), float(xy[1]))
        oid = int(d.get("object_id", -1))
        cname = str(d.get("matched_class", ""))
        self.target_oid = oid if oid >= 0 else self.target_oid
        # First-arrival logging.
        if self.target_xy is None:
            self.get_logger().info(
                f"room_search: target_seen/info  class={cname!r}  "
                f"id={oid}  xy=({new_xy[0]:.2f},{new_xy[1]:.2f})")
        self.target_xy = new_xy

    def _objects_cb(self, msg: String):
        # Refine XY by reading the averaged position of the matched object
        # from /perception/objects (object_mapper_node maintains a running
        # mean). Only relevant once we know which oid we're chasing.
        if self.target_oid is None:
            return
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        for o in (d.get("objects", []) or []):
            try:
                if int(o.get("id", -1)) != self.target_oid:
                    continue
                xy = o.get("xy")
                if not (isinstance(xy, (list, tuple)) and len(xy) >= 2):
                    return
                self.target_xy = (float(xy[0]), float(xy[1]))
                return
            except (TypeError, ValueError):
                continue

    # ─── Timer: rotation control ────────────────────────────────────
    def _rotate_tick(self):
        if self.state != S.ROTATE_AND_SEARCH:
            return
        # Accumulate "rotation done" assuming the platform achieves the
        # commanded rate (good enough as a give-up heuristic — we'd
        # rather over-rotate than miss the target).
        dt = 1.0 / max(self.rot_hz, 1.0)
        self.rot_yaw_acc += abs(self.rot_rate) * dt
        self._publish_cmd(0.0, self.rot_rate)

    # ─── Land burst ─────────────────────────────────────────────────
    def _land_tick(self):
        if self._land_pubs_left <= 0:
            try:
                self._land_timer.cancel()
            except Exception:
                pass
            self._enter(S.DONE)
            return
        self.pub_land.publish(Empty())
        self._land_pubs_left -= 1

    # ─── Main tick ──────────────────────────────────────────────────
    def _tick(self):
        if self.state == S.WAIT_INIT:
            if self.cur_xy is not None:
                self._enter(S.NAV_TO_ROOM)
            return

        if self.state == S.NAV_TO_ROOM:
            if self.cur_xy is None:
                return
            d = math.hypot(self.cur_xy[0] - self.room_xy[0],
                           self.cur_xy[1] - self.room_xy[1])
            if d < self.nav_arr_r:
                self.get_logger().info(
                    f"room_search: arrived at room center  d={d:.2f}m "
                    f"< {self.nav_arr_r:.2f}m")
                self._enter(S.ROTATE_AND_SEARCH)
                return
            # Periodic republish in case the planner missed the first one.
            if (_now_s(self) - self.last_nav_pub) > self.nav_republ_s:
                self._publish_nav_goal(self.room_xy)
                self.last_nav_pub = _now_s(self)
            return

        if self.state == S.ROTATE_AND_SEARCH:
            # Target hit?
            if self.target_seen and self.target_xy is not None:
                # Brake briefly so we don't keep yawing as we hand off.
                self._publish_cmd(0.0, 0.0)
                self._enter(S.APPROACH_TARGET)
                return
            # Watchdog: target_seen=True but no XY? Stay in rotation but
            # warn so the user can see something is off in the bridge.
            if self.target_seen and self.target_xy is None:
                self.get_logger().warn(
                    "room_search: /target_seen=True but no /target_seen/info "
                    "XY yet; still rotating",
                    throttle_duration_sec=2.0)
            # Give-up: exceeded budget.
            if self.rot_yaw_acc > (2.0 * math.pi * self.max_revs):
                self.get_logger().warn(
                    f"room_search: rotated ~{self.rot_yaw_acc:.1f} rad "
                    f"(>={self.max_revs:.1f} revs) without seeing "
                    f"{self.target_label!r} — giving up")
                self._publish_cmd(0.0, 0.0)
                self._enter(S.GIVE_UP)
            return

        if self.state == S.APPROACH_TARGET:
            if self.cur_xy is None or self.target_xy is None:
                return
            d = math.hypot(self.cur_xy[0] - self.target_xy[0],
                           self.cur_xy[1] - self.target_xy[1])
            if d < self.approach_r:
                self.get_logger().info(
                    f"room_search: at target  d={d:.2f}m "
                    f"< {self.approach_r:.2f}m  -> LAND")
                self._enter(S.LAND)
                return
            # Lock approach goal once close enough — last-moment XY jitter
            # shouldn't keep poking the planner.
            if d < self.lock_r and not self._appr_locked:
                self._appr_locked = True
                self.get_logger().info(
                    f"room_search: approach goal locked at "
                    f"({self.target_xy[0]:.2f}, {self.target_xy[1]:.2f})  "
                    f"d={d:.2f}m")
            # Refine goal if XY shifted significantly (and not yet locked).
            if not self._appr_locked:
                last = getattr(self, "_last_pub_xy", None)
                if last is None or math.hypot(self.target_xy[0] - last[0],
                                              self.target_xy[1] - last[1]) \
                                   > self.refine_thr:
                    self._publish_nav_goal(self.target_xy)
                    self.last_nav_pub = _now_s(self)
                    self._last_pub_xy = self.target_xy
            # Periodic republish even without refine, so a late planner
            # eventually picks it up.
            if (_now_s(self) - self.last_nav_pub) > self.appr_republ_s:
                self._publish_nav_goal(self.target_xy)
                self.last_nav_pub = _now_s(self)
            # Hard timeout.
            if self._t_in() > self.appr_to_s:
                self.get_logger().warn(
                    f"room_search: approach timed out after "
                    f"{self.appr_to_s:.0f}s  d={d:.2f}m  -> LAND anyway")
                self._enter(S.LAND)
            return

        # LAND has its own _land_tick that transitions to DONE.
        # DONE / GIVE_UP just sit.

    # ─── Heartbeat ──────────────────────────────────────────────────
    def _hb(self):
        bits = [f"state={self.state}",
                f"t_in={self._t_in():.1f}s",
                f"target_seen={self.target_seen}"]
        if self.cur_xy is not None:
            bits.append(f"pose=({self.cur_xy[0]:.2f},{self.cur_xy[1]:.2f})")
        else:
            bits.append("pose=None")
        if self.target_xy is not None:
            bits.append(
                f"target_xy=({self.target_xy[0]:.2f},{self.target_xy[1]:.2f})")
        if self.state == S.ROTATE_AND_SEARCH:
            bits.append(f"yaw_acc={self.rot_yaw_acc:.2f}rad")
        self.get_logger().info("room_search hb  " + "  ".join(bits))


def main():
    rclpy.init()
    node = RoomSearchOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort: hand control back before we die.
        try:
            node.pub_ext.publish(Bool(data=False))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
