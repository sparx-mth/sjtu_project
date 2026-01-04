#!/usr/bin/env python3
"""
route_map_viewer.py
-------------------
Live 2D occupancy-map viewer + RRT* route overlay with multiple smooth trajectories.

Features:
- Subscribes to drone pose: /simple_drone/gt_pose (geometry_msgs/Pose)
- Target selection:
    * Click on map
    * Press 't' to type target grid coords
- Calls planner service: /plan_path_rrt (autonomous_system/srv/PlanPath)
- Draws:
    * Raw waypoints from planner (blue dots)
    * Bezier trajectory (cyan curve) - heading-aware
    * CubicSpline trajectory (green curve)
    * MinSnap trajectory (magenta curve)
    * Velocity arrows (optional)

Controls:
- Click: Set target
- 't': Type target coordinates
- 'b': Toggle Bezier trajectory display
- 's': Toggle CubicSpline trajectory display
- 'm': Toggle MinSnap trajectory display
- 'v': Toggle velocity arrows
"""

import os
import time
from typing import Optional, Tuple, List

import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import simpledialog

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from autonomous_system.srv import PlanPath

# Try to import all smoothers
try:
    from autonomous_system.planning.bezier_trajectory_smoother import smooth_waypoints as bezier_smooth
    HAS_BEZIER = True
except ImportError:
    HAS_BEZIER = False
    bezier_smooth = None
    print("[WARN] Bezier smoother not available")

try:
    from autonomous_system.planning.minsnap_trajectory_smoother import smooth_waypoints as minsnap_smooth
    HAS_MINSNAP = True
except ImportError:
    HAS_MINSNAP = False
    minsnap_smooth = None
    print("[WARN] MinSnap smoother not available - install: pip install minsnap-trajectories")

try:
    from autonomous_system.planning.trajectory_smoother import smooth_waypoints as cubic_smooth
    HAS_CUBIC = True
except ImportError:
    HAS_CUBIC = False
    cubic_smooth = None
    print("[WARN] CubicSpline smoother not available")


class RouteMapViewer(Node):
    def __init__(self):
        super().__init__("route_map_viewer")

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter("map_yaml", "/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml")
        self.declare_parameter("pose_topic", "/simple_drone/gt_pose")
        self.declare_parameter("planner_service", "/plan_path_rrt")
        self.declare_parameter("planner_timeout_sec", 5.0)

        self.declare_parameter("target_size_px", 20)
        self.declare_parameter("replan_period_sec", 0.5)
        self.declare_parameter("replan_if_start_moved_m", 0.75)
        self.declare_parameter("show_velocity_arrows", True)
        self.declare_parameter("show_bezier_trajectory", True)
        self.declare_parameter("show_cubic_trajectory", True)
        self.declare_parameter("show_minsnap_trajectory", True)
        self.declare_parameter("velocity_arrow_stride", 4)
        self.declare_parameter("smooth_sample_spacing", 0.1)  # meters

        map_yaml_path = str(self.get_parameter("map_yaml").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        planner_service = str(self.get_parameter("planner_service").value)

        self.planner_timeout_sec = float(self.get_parameter("planner_timeout_sec").value)
        self.target_size = int(self.get_parameter("target_size_px").value)

        self.replan_period_sec = float(self.get_parameter("replan_period_sec").value)
        self.replan_if_start_moved_m = float(self.get_parameter("replan_if_start_moved_m").value)

        self.show_velocity_arrows = bool(self.get_parameter("show_velocity_arrows").value)
        self.show_bezier_trajectory = bool(self.get_parameter("show_bezier_trajectory").value) and HAS_BEZIER
        self.show_cubic_trajectory = bool(self.get_parameter("show_cubic_trajectory").value) and HAS_CUBIC
        self.show_minsnap_trajectory = bool(self.get_parameter("show_minsnap_trajectory").value) and HAS_MINSNAP
        self.velocity_arrow_stride = int(self.get_parameter("velocity_arrow_stride").value)
        self.smooth_sample_spacing = float(self.get_parameter("smooth_sample_spacing").value)

        # ----------------------------
        # Load map (YAML + image)
        # ----------------------------
        with open(map_yaml_path, "r") as f:
            info = yaml.safe_load(f)

        self.resolution = float(info["resolution"])
        self.origin = info["origin"]  # [ox, oy, yaw]

        map_image_path = info["image"]
        if not map_image_path.startswith("/"):
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)

        img = cv2.imread(map_image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(f"Failed to load map image: {map_image_path}")

        # Binary occupancy for display
        self.map_data = np.zeros_like(img, dtype=np.uint8)
        self.map_data[img < 50] = 1  # obstacles
        self.map_data = np.flipud(self.map_data)  # make y-axis match "origin=lower"

        self.h, self.w = self.map_data.shape[:2]
        self.get_logger().info(f"Map loaded: {map_image_path}, size=({self.w}x{self.h}), res={self.resolution}")

        # ----------------------------
        # State
        # ----------------------------
        self.drone_pose_world: Optional[Tuple[float, float]] = None

        # Default target in MAP coords (grid/pixel)
        self.target_map: Tuple[int, int] = (355, 593)

        # Raw waypoints from planner (map coords)
        self.route_map_x: List[float] = []
        self.route_map_y: List[float] = []
        self.route_vel_map_u: List[float] = []
        self.route_vel_map_v: List[float] = []

        # CubicSpline trajectory points (map coords)
        self.cubic_map_x: List[float] = []
        self.cubic_map_y: List[float] = []

        # Bezier trajectory points (map coords)
        self.bezier_map_x: List[float] = []
        self.bezier_map_y: List[float] = []

        # MinSnap trajectory points (map coords)
        self.minsnap_map_x: List[float] = []
        self.minsnap_map_y: List[float] = []

        # Planner request tracking
        self._plan_in_flight = False
        self._last_plan_start_world: Optional[Tuple[float, float]] = None
        self._last_plan_goal_world: Optional[Tuple[float, float]] = None
        self._need_replan = True

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self.pose_sub = self.create_subscription(Pose, pose_topic, self._pose_callback, 10)

        self.planner_client = self.create_client(PlanPath, planner_service)
        self.get_logger().info(f"Waiting for planner service: {planner_service} ...")
        while not self.planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("  Still waiting for planner...")

        # ----------------------------
        # Matplotlib UI
        # ----------------------------
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title("Route Map Viewer - [b]=Bezier [s]=CubicSpline [m]=MinSnap [v]=velocity")

        self.im = self.ax.imshow(self.map_data, cmap="gray", origin="lower")

        # Drone marker
        self.drone_point, = self.ax.plot([], [], "ro", markersize=8, label="drone")

        # Target rectangle
        self.target_rect = None
        self._draw_target_rectangle()

        # Raw waypoints - line + dots (blue)
        self.route_line, = self.ax.plot([], [], "b-", linewidth=1.5, alpha=0.7, label="RRT* waypoints")
        self.route_dots, = self.ax.plot([], [], "bo", markersize=6, alpha=0.9)

        # Bezier trajectory (cyan, thick)
        self.bezier_line, = self.ax.plot([], [], "c-", linewidth=3, alpha=0.8, label="Bezier")

        # CubicSpline trajectory (green, thick)
        self.cubic_line, = self.ax.plot([], [], "g-", linewidth=3, alpha=0.8, label="CubicSpline")

        # MinSnap trajectory (magenta, thick)
        self.minsnap_line, = self.ax.plot([], [], "m-", linewidth=3, alpha=0.8, label="MinSnap")

        # Velocity arrows (optional)
        self.vel_quiver = None

        # Status text (bottom-left)
        self.status_text = self.ax.text(
            0.02, 0.02, "",
            transform=self.ax.transAxes,
            fontsize=9,
            verticalalignment="bottom",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
        )

        self.ax.legend(loc="upper right")

        # Input handlers
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)

        plt.show(block=False)

        # Timers
        self.create_timer(0.1, self._update_display)
        self.create_timer(self.replan_period_sec, self._maybe_replan)

        self.get_logger().info("RouteMapViewer ready.")
        self.get_logger().info(f"  Bezier: {'available' if HAS_BEZIER else 'NOT available'}")
        self.get_logger().info(f"  CubicSpline: {'available' if HAS_CUBIC else 'NOT available'}")
        self.get_logger().info(f"  MinSnap: {'available' if HAS_MINSNAP else 'NOT available'}")
        self.get_logger().info("  Controls: click=target, t=type, b=bezier, s=cubic, m=minsnap, v=velocity")

    # ----------------------------
    # Coordinate conversions
    # ----------------------------
    def world_to_map_f(self, x_world: float, y_world: float) -> Tuple[float, float]:
        ox, oy, _ = self.origin
        return (x_world - ox) / self.resolution, (y_world - oy) / self.resolution

    def world_to_map_i(self, x_world: float, y_world: float) -> Tuple[int, int]:
        xf, yf = self.world_to_map_f(x_world, y_world)
        return int(round(xf)), int(round(yf))

    def map_to_world(self, x_map: float, y_map: float) -> Tuple[float, float]:
        ox, oy, _ = self.origin
        return x_map * self.resolution + ox, y_map * self.resolution + oy

    # ----------------------------
    # ROS callbacks
    # ----------------------------
    def _pose_callback(self, msg: Pose) -> None:
        self.drone_pose_world = (float(msg.position.x), float(msg.position.y))

    # ----------------------------
    # Planning
    # ----------------------------
    def _maybe_replan(self) -> None:
        if self.drone_pose_world is None:
            return
        if self._plan_in_flight:
            return

        start_world = self.drone_pose_world
        goal_world = self.map_to_world(float(self.target_map[0]), float(self.target_map[1]))

        # Check if we need to replan
        if not self._need_replan:
            if self._last_plan_start_world is not None:
                dist = ((start_world[0] - self._last_plan_start_world[0])**2 +
                        (start_world[1] - self._last_plan_start_world[1])**2)**0.5
                if dist < self.replan_if_start_moved_m:
                    return
            else:
                return

        self._need_replan = False
        self._plan_in_flight = True
        self._last_plan_start_world = start_world
        self._last_plan_goal_world = goal_world

        # Call planner
        req = PlanPath.Request()
        req.start_x, req.start_y = start_world
        req.goal_x, req.goal_y = goal_world

        future = self.planner_client.call_async(req)
        future.add_done_callback(self._plan_done_callback)

    def _plan_done_callback(self, future) -> None:
        self._plan_in_flight = False

        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self._clear_route()
            return

        if res is None or not res.success or len(res.waypoints_x) < 2:
            msg = res.message if res is not None else "None response"
            self.get_logger().warn(f"No path: {msg}")
            self._clear_route()
            return

        # Convert world waypoints -> map coords
        n = len(res.waypoints_x)
        mx, my = [], []
        wx_list, wy_list = [], []
        for i in range(n):
            wx_list.append(res.waypoints_x[i])
            wy_list.append(res.waypoints_y[i])
            xf, yf = self.world_to_map_f(res.waypoints_x[i], res.waypoints_y[i])
            mx.append(xf)
            my.append(yf)

        self.route_map_x = mx
        self.route_map_y = my

        # Velocity arrows
        self.route_vel_map_u, self.route_vel_map_v = [], []
        has_vel = (len(res.velocities_x) == n and len(res.velocities_y) == n)
        if self.show_velocity_arrows and has_vel and n > 0:
            scale = 1.0 / self.resolution
            for i in range(n):
                self.route_vel_map_u.append(res.velocities_x[i] * scale)
                self.route_vel_map_v.append(res.velocities_y[i] * scale)

        # Generate ALL smooth trajectories
        self._generate_bezier_trajectory(wx_list, wy_list)
        self._generate_cubic_trajectory(wx_list, wy_list)
        self._generate_minsnap_trajectory(wx_list, wy_list)

        self.get_logger().info(
            f"Route: {n} waypoints | "
            f"Bezier: {len(self.bezier_map_x)} pts | "
            f"Cubic: {len(self.cubic_map_x)} pts | "
            f"MinSnap: {len(self.minsnap_map_x)} pts"
        )

    def _generate_bezier_trajectory(self, wx_list: List[float], wy_list: List[float]) -> None:
        """Generate Bezier trajectory from world waypoints."""
        self.bezier_map_x, self.bezier_map_y = [], []

        if not HAS_BEZIER or len(wx_list) < 2:
            return

        try:
            trajectory = bezier_smooth(wx_list, wy_list)
            if trajectory is None:
                return

            points = trajectory.sample_trajectory(spacing=self.smooth_sample_spacing)
            for pt in points:
                mx, my = self.world_to_map_f(pt.x, pt.y)
                self.bezier_map_x.append(mx)
                self.bezier_map_y.append(my)
        except Exception as e:
            self.get_logger().warn(f"Bezier failed: {e}")

    def _generate_cubic_trajectory(self, wx_list: List[float], wy_list: List[float]) -> None:
        """Generate CubicSpline trajectory from world waypoints."""
        self.cubic_map_x, self.cubic_map_y = [], []

        if not HAS_CUBIC or len(wx_list) < 2:
            return

        try:
            trajectory = cubic_smooth(wx_list, wy_list)
            if trajectory is None:
                return

            points = trajectory.sample_trajectory(spacing=self.smooth_sample_spacing)
            for pt in points:
                mx, my = self.world_to_map_f(pt.x, pt.y)
                self.cubic_map_x.append(mx)
                self.cubic_map_y.append(my)
        except Exception as e:
            self.get_logger().warn(f"CubicSpline failed: {e}")

    def _generate_minsnap_trajectory(self, wx_list: List[float], wy_list: List[float]) -> None:
        """Generate MinSnap trajectory from world waypoints."""
        self.minsnap_map_x, self.minsnap_map_y = [], []

        if not HAS_MINSNAP or len(wx_list) < 2:
            return

        try:
            trajectory = minsnap_smooth(wx_list, wy_list)
            if trajectory is None:
                return

            points = trajectory.sample_trajectory(spacing=self.smooth_sample_spacing)
            for pt in points:
                mx, my = self.world_to_map_f(pt.x, pt.y)
                self.minsnap_map_x.append(mx)
                self.minsnap_map_y.append(my)
        except Exception as e:
            self.get_logger().warn(f"MinSnap failed: {e}")

    def _clear_route(self) -> None:
        """Clear all route data."""
        self.route_map_x, self.route_map_y = [], []
        self.route_vel_map_u, self.route_vel_map_v = [], []
        self.bezier_map_x, self.bezier_map_y = [], []
        self.cubic_map_x, self.cubic_map_y = [], []
        self.minsnap_map_x, self.minsnap_map_y = [], []

    # ----------------------------
    # Drawing helpers
    # ----------------------------
    def _draw_target_rectangle(self) -> None:
        if self.target_rect is not None:
            self.target_rect.remove()

        x_t, y_t = self.target_map
        s = self.target_size

        self.target_rect = plt.Rectangle(
            (x_t - s, y_t - s),
            2 * s, 2 * s,
            linewidth=2,
            edgecolor="lime",
            facecolor="none",
        )
        self.ax.add_patch(self.target_rect)
        self.fig.canvas.draw_idle()

    def _update_display(self) -> None:
        # Update drone marker
        if self.drone_pose_world is not None:
            dxw, dyw = self.drone_pose_world
            dxm, dym = self.world_to_map_f(dxw, dyw)
            self.drone_point.set_data([dxm], [dym])

        # Update raw waypoint line + dots (blue)
        if len(self.route_map_x) >= 2:
            self.route_line.set_data(self.route_map_x, self.route_map_y)
            self.route_dots.set_data(self.route_map_x, self.route_map_y)
        else:
            self.route_line.set_data([], [])
            self.route_dots.set_data([], [])

        # Update CubicSpline trajectory (green)
        if self.show_cubic_trajectory and len(self.cubic_map_x) >= 2:
            self.cubic_line.set_data(self.cubic_map_x, self.cubic_map_y)
            self.cubic_line.set_visible(True)
        else:
            self.cubic_line.set_data([], [])
            self.cubic_line.set_visible(False)

        # Update Bezier trajectory (cyan)
        if self.show_bezier_trajectory and len(self.bezier_map_x) >= 2:
            self.bezier_line.set_data(self.bezier_map_x, self.bezier_map_y)
            self.bezier_line.set_visible(True)
        else:
            self.bezier_line.set_data([], [])
            self.bezier_line.set_visible(False)

        # Update MinSnap trajectory (magenta)
        if self.show_minsnap_trajectory and len(self.minsnap_map_x) >= 2:
            self.minsnap_line.set_data(self.minsnap_map_x, self.minsnap_map_y)
            self.minsnap_line.set_visible(True)
        else:
            self.minsnap_line.set_data([], [])
            self.minsnap_line.set_visible(False)

        # Update velocity quiver
        if self.show_velocity_arrows and len(self.route_vel_map_u) > 0 and len(self.route_map_x) > 0:
            if self.vel_quiver is not None:
                self.vel_quiver.remove()
                self.vel_quiver = None

            stride = max(1, self.velocity_arrow_stride)
            xs = np.array(self.route_map_x[::stride], dtype=float)
            ys = np.array(self.route_map_y[::stride], dtype=float)
            us = np.array(self.route_vel_map_u[::stride], dtype=float)
            vs = np.array(self.route_vel_map_v[::stride], dtype=float)

            self.vel_quiver = self.ax.quiver(
                xs, ys, us, vs,
                angles="xy", scale_units="xy", scale=1.0,
                color="orange", alpha=0.7
            )
        else:
            if self.vel_quiver is not None:
                self.vel_quiver.remove()
                self.vel_quiver = None

        # Update status text
        tx, ty = self.target_map
        txw, tyw = self.map_to_world(float(tx), float(ty))

        bezier_str = f"ON ({len(self.bezier_map_x)})" if self.show_bezier_trajectory else "OFF"
        cubic_str = f"ON ({len(self.cubic_map_x)})" if self.show_cubic_trajectory else "OFF"
        minsnap_str = f"ON ({len(self.minsnap_map_x)})" if self.show_minsnap_trajectory else "OFF"
        vel_str = "ON" if self.show_velocity_arrows else "OFF"

        if self.drone_pose_world is None:
            self.status_text.set_text(
                f"Target: map=({tx},{ty}) world=({txw:.2f},{tyw:.2f})\n"
                f"Drone: waiting for pose...\n"
                f"[b] Bezier: {bezier_str} | [s] Cubic: {cubic_str} | [m] MinSnap: {minsnap_str} | [v] Vel: {vel_str}"
            )
        else:
            dxw, dyw = self.drone_pose_world
            dxm, dym = self.world_to_map_f(dxw, dyw)
            self.status_text.set_text(
                f"Drone: world=({dxw:.2f},{dyw:.2f}) map=({dxm:.1f},{dym:.1f})\n"
                f"Target: map=({tx},{ty}) world=({txw:.2f},{tyw:.2f})\n"
                f"RRT* waypoints: {len(self.route_map_x)}\n"
                f"[b] Bezier: {bezier_str} | [s] Cubic: {cubic_str} | [m] MinSnap: {minsnap_str} | [v] Vel: {vel_str}"
            )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    # ----------------------------
    # UI events
    # ----------------------------
    def _on_key(self, event) -> None:
        if event.key == "t":
            self._handle_type_target()
        elif event.key == "b":
            if HAS_BEZIER:
                self.show_bezier_trajectory = not self.show_bezier_trajectory
                state = "ON" if self.show_bezier_trajectory else "OFF"
                self.get_logger().info(f"Bezier trajectory: {state}")
            else:
                self.get_logger().warn("Bezier not available")
        elif event.key == "s":
            if HAS_CUBIC:
                self.show_cubic_trajectory = not self.show_cubic_trajectory
                state = "ON" if self.show_cubic_trajectory else "OFF"
                self.get_logger().info(f"CubicSpline trajectory: {state}")
            else:
                self.get_logger().warn("CubicSpline not available")
        elif event.key == "m":
            if HAS_MINSNAP:
                self.show_minsnap_trajectory = not self.show_minsnap_trajectory
                state = "ON" if self.show_minsnap_trajectory else "OFF"
                self.get_logger().info(f"MinSnap trajectory: {state}")
            else:
                self.get_logger().warn("MinSnap not available - install: pip install minsnap-trajectories")
        elif event.key == "v":
            self.show_velocity_arrows = not self.show_velocity_arrows
            state = "ON" if self.show_velocity_arrows else "OFF"
            self.get_logger().info(f"Velocity arrows: {state}")

    def _handle_type_target(self) -> None:
        root = tk.Tk()
        root.withdraw()
        try:
            x = simpledialog.askinteger("New Target X", "Enter target X (map pixel):", parent=root)
            if x is None:
                return
            y = simpledialog.askinteger("New Target Y", "Enter target Y (map pixel):", parent=root)
            if y is None:
                return

            x = int(np.clip(x, 0, self.w - 1))
            y = int(np.clip(y, 0, self.h - 1))

            self.target_map = (x, y)
            self._draw_target_rectangle()
            self._need_replan = True
        finally:
            root.destroy()

    def _on_click(self, event) -> None:
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return

        x = int(round(event.xdata))
        y = int(round(event.ydata))

        x = int(np.clip(x, 0, self.w - 1))
        y = int(np.clip(y, 0, self.h - 1))

        self.target_map = (x, y)
        self._draw_target_rectangle()
        self._need_replan = True

        print(f"[CLICK] New target (map) = ({x}, {y})")


def main():
    rclpy.init()
    node = RouteMapViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()