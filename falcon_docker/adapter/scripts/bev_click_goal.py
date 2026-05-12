#!/usr/bin/env python3
"""
bev_click_goal.py — interactive 2D BEV viewer with click-to-navigate.

Opens a matplotlib window (NOT RViz) showing:
  • /falcon/bev_2d as a grayscale image
        gray   = unknown (-1)
        white  = free (0)
        black  = occupied (100)
  • Drone position + heading (red arrow), from /<drone_ns>/gt_pose
  • Current planned path (cyan polyline), from /path/waypoints
  • Last clicked goal (green star)

LEFT-CLICK anywhere on the map → publishes geometry_msgs/Point to
/waypoint_nav/goal. The existing astar_planner picks it up, replans,
waypoint_follower flies the new path. The GUI shows the new path
within one BEV update.

This node is purely a viewer + click adapter. It doesn't move the
drone, plan, or alter the map; it just sends one Point per click.

Run as a sidecar in a separate terminal of the falcon container:

    docker exec -it falcon bash
    source /catkin_ws/devel/setup.bash
    rosrun falcon_adapter bev_click_goal.py

Requires matplotlib. If missing inside the container:
    apt-get update && apt-get install -y python3-matplotlib
"""
import threading
import rospy
import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tft
from matplotlib.animation import FuncAnimation

from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid, Path


def quat_yaw(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class BEVClickGoal:
    def __init__(self):
        # disable_signals=True so matplotlib's main loop owns Ctrl+C
        rospy.init_node("bev_click_goal", disable_signals=True)
        G = rospy.get_param

        self.drone_ns   = G("~drone_ns",   "")
        self.bev_topic  = G("~bev_topic",  "/falcon/bev_2d")
        self.path_topic = G("~path_topic", "/path/waypoints")
        self.goal_topic = G("~goal_topic", "/waypoint_nav/goal")
        self.refresh_hz = float(G("~refresh_hz", 5.0))
        self.arrow_len  = float(G("~arrow_len_m", 0.5))

        # Latest data + lock for cross-thread access
        self._bev      = None
        self._path_xy  = []
        self._drone_p  = None        # (x, y, yaw)
        self._goal_xy  = None
        self._lock     = threading.Lock()

        # ROS plumbing
        self.goal_pub = rospy.Publisher(self.goal_topic, Point,
                                          queue_size=1, latch=True)
        rospy.Subscriber(self.bev_topic,  OccupancyGrid,
                         self._bev_cb,  queue_size=1)
        rospy.Subscriber(self.path_topic, Path,
                         self._path_cb, queue_size=1)
        rospy.Subscriber(self.drone_ns + "/gt_pose", Pose,
                         self._pose_cb, queue_size=10)

        # Matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect(
            'close_event',
            lambda _e: rospy.signal_shutdown("bev_click_goal window closed"))
        self.ax.set_aspect('equal')
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.grid(True, alpha=0.25)

        # Persistent artists (created lazily, updated in place)
        self._im           = None
        self._path_line    = None
        self._drone_dot    = None
        self._drone_arrow  = None
        self._goal_marker  = None

        rospy.loginfo("=" * 64)
        rospy.loginfo("bev_click_goal: ready")
        rospy.loginfo("  bev   in  = %s", self.bev_topic)
        rospy.loginfo("  path  in  = %s", self.path_topic)
        rospy.loginfo("  pose  in  = %s/gt_pose", self.drone_ns)
        rospy.loginfo("  goal  out = %s   (left-click to publish)",
                      self.goal_topic)
        rospy.loginfo("=" * 64)

    # ─── Subscribers ─────────────────────────────────────────────
    def _bev_cb(self, msg):
        with self._lock:
            self._bev = msg

    def _path_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self._path_xy = pts

    def _pose_cb(self, msg):
        yaw = quat_yaw(msg.orientation)
        with self._lock:
            self._drone_p = (msg.position.x, msg.position.y, yaw)

    # ─── Click handler ───────────────────────────────────────────
    def _on_click(self, event):
        if event.inaxes != self.ax:                  return
        if event.button != 1:                        return  # left only
        if event.xdata is None or event.ydata is None: return
        gx, gy = float(event.xdata), float(event.ydata)
        rospy.loginfo("bev_click_goal: click → goal (%.2f, %.2f)", gx, gy)
        with self._lock:
            self._goal_xy = (gx, gy)
            # Clear the displayed path so the stale cyan line disappears
            # immediately. astar_planner publishes the new path within
            # one BEV cycle (~0.5 s) and it'll be redrawn then.
            self._path_xy = []
        m = Point(); m.x = gx; m.y = gy; m.z = 0.0
        self.goal_pub.publish(m)

    # ─── Render (called by FuncAnimation on the main thread) ────
    def _render(self, _frame):
        with self._lock:
            bev   = self._bev
            path  = list(self._path_xy)
            drone = self._drone_p
            goal  = self._goal_xy

        if bev is None:
            self.ax.set_title("Waiting for %s ..." % self.bev_topic)
            return []

        info = bev.info
        W, H, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = np.frombuffer(np.array(bev.data, dtype=np.int8).tobytes(),
                              dtype=np.int8).reshape(H, W)

        # Tri-color RGB: unknown=gray, free=white, occupied=near-black
        rgb = np.full((H, W, 3), 180, dtype=np.uint8)
        rgb[data == 0]   = (255, 255, 255)
        rgb[data == 100] = (30, 30, 30)

        extent = (ox, ox + W * res, oy, oy + H * res)
        if self._im is None:
            self._im = self.ax.imshow(rgb, origin='lower',
                                       extent=extent, interpolation='nearest')
            self.ax.set_xlim(extent[0], extent[1])
            self.ax.set_ylim(extent[2], extent[3])
        else:
            self._im.set_data(rgb)
            self._im.set_extent(extent)

        # Title shows current goal status
        if goal is not None:
            self.ax.set_title(
                "Left-click to set goal   |   current goal: (%.2f, %.2f)"
                % goal)
        else:
            self.ax.set_title("Left-click anywhere to set navigation goal")

        # Path overlay
        if self._path_line is not None:
            self._path_line.remove(); self._path_line = None
        if len(path) >= 2:
            xs = [p[0] for p in path]; ys = [p[1] for p in path]
            self._path_line, = self.ax.plot(
                xs, ys, '-o', color='deepskyblue',
                linewidth=2.0, markersize=4, alpha=0.9, zorder=3)

        # Drone marker + heading arrow
        if self._drone_dot   is not None:
            self._drone_dot.remove();   self._drone_dot   = None
        if self._drone_arrow is not None:
            self._drone_arrow.remove(); self._drone_arrow = None
        if drone is not None:
            x, y, yaw = drone
            self._drone_dot, = self.ax.plot(
                [x], [y], 'o', color='red', markersize=8,
                markeredgecolor='black', zorder=5)
            dx = self.arrow_len * np.cos(yaw)
            dy = self.arrow_len * np.sin(yaw)
            self._drone_arrow = self.ax.annotate(
                '', xy=(x + dx, y + dy), xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color='red', lw=2),
                zorder=5)

        # Goal marker
        if self._goal_marker is not None:
            self._goal_marker.remove(); self._goal_marker = None
        if goal is not None:
            self._goal_marker, = self.ax.plot(
                [goal[0]], [goal[1]], '*', color='lime',
                markersize=20, markeredgecolor='black', zorder=4)

        return []

    # ─── Spin ────────────────────────────────────────────────────
    def spin(self):
        self.anim = FuncAnimation(
            self.fig, self._render,
            interval=int(1000.0 / self.refresh_hz),
            blit=False, cache_frame_data=False)
        try:
            plt.show()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    try:
        BEVClickGoal().spin()
    except rospy.ROSInterruptException:
        pass