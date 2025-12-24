#!/usr/bin/env python3
"""
route_map_viewer.py
-------------------
Live 2D occupancy-map viewer + RRT* route overlay with smooth trajectory.

Features:
- Subscribes to drone pose: /simple_drone/gt_pose (geometry_msgs/Pose)
- Target selection:
    * Click on map
    * Press 't' to type target grid coords
- Calls planner service: /plan_path_rrt (autonomous_system/srv/PlanPath)
- Draws:
    * Raw waypoints from planner (blue dots)
    * Smooth spline trajectory (green curve)
    * Velocity arrows (optional)

Controls:
- Click: Set target
- 't': Type target coordinates
- 's': Toggle smooth trajectory display
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
from autonomous_system.planning.trajectory_smoother import smooth_waypoints


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
        self.declare_parameter("show_smooth_trajectory", True)
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
        self.show_smooth_trajectory = bool(self.get_parameter("show_smooth_trajectory").value)
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

        # Smooth trajectory points (map coords)
        self.smooth_map_x: List[float] = []
        self.smooth_map_y: List[float] = []

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
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_title("Route Map Viewer (s=smooth, v=velocity)")

        self.im = self.ax.imshow(self.map_data, cmap="gray", origin="lower")

        # Drone marker
        self.drone_point, = self.ax.plot([], [], "ro", markersize=6, label="drone")

        # Target rectangle
        self.target_rect = None
        self._draw_target_rectangle()

        # Raw waypoints - line + dots (blue)
        self.route_line, = self.ax.plot([], [], "b-", linewidth=1.5, alpha=0.7, label="RRT* path")
        self.route_dots, = self.ax.plot([], [], "bo", markersize=5, alpha=0.8)

        # Smooth trajectory (green, thicker)
        self.smooth_line, = self.ax.plot([], [], "g-", linewidth=3, label="smooth path")

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
        self.get_logger().info("  Click to set target, 't' to type, 's' toggle smooth, 'v' toggle velocity")

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

        goal_world = self.map_to_world(float(self.target_map[0]), float(self.target_map[1]))
        start_world = self.drone_pose_world

        if self._last_plan_start_world is None or self._last_plan_goal_world is None:
            self._need_replan = True
        else:
            gx0, gy0 = self._last_plan_goal_world
            gx1, gy1 = goal_world
            goal_changed = (abs(gx1 - gx0) > 1e-6) or (abs(gy1 - gy0) > 1e-6)

            sx0, sy0 = self._last_plan_start_world
            sx1, sy1 = start_world
            start_moved = ((sx1 - sx0) ** 2 + (sy1 - sy0) ** 2) ** 0.5 > self.replan_if_start_moved_m

            self._need_replan = self._need_replan or goal_changed or start_moved

        if not self._need_replan:
            return

        req = PlanPath.Request()
        req.start_x = float(start_world[0])
        req.start_y = float(start_world[1])
        req.goal_x = float(goal_world[0])
        req.goal_y = float(goal_world[1])

        self._plan_in_flight = True
        self._need_replan = False
        self._last_plan_start_world = start_world
        self._last_plan_goal_world = goal_world
        self._plan_sent_time = time.time()

        future = self.planner_client.call_async(req)
        future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future) -> None:
        self._plan_in_flight = False

        if hasattr(self, "_plan_sent_time") and (time.time() - self._plan_sent_time) > self.planner_timeout_sec:
            self.get_logger().warn("Planner response arrived after timeout; ignoring.")
            return

        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Planner call failed: {e}")
            self._clear_route()
            return

        if res is None or not res.success:
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

        # Generate smooth trajectory
        self._generate_smooth_trajectory(wx_list, wy_list)

        self.get_logger().info(f"Route: {n} waypoints, {len(self.smooth_map_x)} smooth points")

    def _generate_smooth_trajectory(self, wx_list: List[float], wy_list: List[float]) -> None:
        """Generate smooth spline trajectory from world waypoints."""
        self.smooth_map_x, self.smooth_map_y = [], []

        if len(wx_list) < 2:
            return

        trajectory = smooth_waypoints(wx_list, wy_list)
        if trajectory is None:
            return

        # Sample the smooth trajectory
        points = trajectory.sample_trajectory(spacing=self.smooth_sample_spacing)

        for pt in points:
            mx, my = self.world_to_map_f(pt.x, pt.y)
            self.smooth_map_x.append(mx)
            self.smooth_map_y.append(my)

    def _clear_route(self) -> None:
        """Clear all route data."""
        self.route_map_x, self.route_map_y = [], []
        self.route_vel_map_u, self.route_vel_map_v = [], []
        self.smooth_map_x, self.smooth_map_y = [], []

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

        # Update raw waypoint line + dots
        if len(self.route_map_x) >= 2:
            self.route_line.set_data(self.route_map_x, self.route_map_y)
            self.route_dots.set_data(self.route_map_x, self.route_map_y)
        else:
            self.route_line.set_data([], [])
            self.route_dots.set_data([], [])

        # Update smooth trajectory line
        if self.show_smooth_trajectory and len(self.smooth_map_x) >= 2:
            self.smooth_line.set_data(self.smooth_map_x, self.smooth_map_y)
            self.smooth_line.set_visible(True)
        else:
            self.smooth_line.set_data([], [])
            self.smooth_line.set_visible(False)

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

        smooth_str = "ON" if self.show_smooth_trajectory else "OFF"
        vel_str = "ON" if self.show_velocity_arrows else "OFF"

        if self.drone_pose_world is None:
            self.status_text.set_text(
                f"Target: map=({tx},{ty}) world=({txw:.2f},{tyw:.2f})\n"
                f"Drone: waiting for pose...\n"
                f"Smooth: {smooth_str} | Velocity: {vel_str}"
            )
        else:
            dxw, dyw = self.drone_pose_world
            dxm, dym = self.world_to_map_f(dxw, dyw)
            self.status_text.set_text(
                f"Drone: world=({dxw:.2f},{dyw:.2f}) map=({dxm:.1f},{dym:.1f})\n"
                f"Target: map=({tx},{ty}) world=({txw:.2f},{tyw:.2f})\n"
                f"Waypoints: {len(self.route_map_x)} | Smooth: {len(self.smooth_map_x)} pts\n"
                f"[s] Smooth: {smooth_str} | [v] Velocity: {vel_str}"
            )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    # ----------------------------
    # UI events
    # ----------------------------
    def _on_key(self, event) -> None:
        if event.key == "t":
            self._handle_type_target()
        elif event.key == "s":
            self.show_smooth_trajectory = not self.show_smooth_trajectory
            state = "ON" if self.show_smooth_trajectory else "OFF"
            self.get_logger().info(f"Smooth trajectory: {state}")
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