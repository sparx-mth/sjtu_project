#!/usr/bin/env python3
"""
route_map_viewer.py
-------------------
Live 2D occupancy-map viewer + RRT* route overlay.

Features:
- Subscribes to drone pose: /simple_drone/gt_pose (geometry_msgs/Pose)
- Target selection:
    * Click on map
    * Press 't' to type target grid coords
- Calls planner service: /plan_path_rrt (autonomous_system/srv/PlanPath)
- Draws returned path on the map (and optional velocity arrows)

Notes:
- Goal is selected in MAP GRID coords (pixels), converted to WORLD coords for planning.
- Planner returns WAYPOINTS in WORLD coords, converted back to MAP GRID for drawing.
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
        self.declare_parameter("velocity_arrow_stride", 4)  # draw 1 arrow every N waypoints

        map_yaml_path = str(self.get_parameter("map_yaml").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        planner_service = str(self.get_parameter("planner_service").value)

        self.planner_timeout_sec = float(self.get_parameter("planner_timeout_sec").value)
        self.target_size = int(self.get_parameter("target_size_px").value)

        self.replan_period_sec = float(self.get_parameter("replan_period_sec").value)
        self.replan_if_start_moved_m = float(self.get_parameter("replan_if_start_moved_m").value)

        self.show_velocity_arrows = bool(self.get_parameter("show_velocity_arrows").value)
        self.velocity_arrow_stride = int(self.get_parameter("velocity_arrow_stride").value)

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

        # Binary occupancy for display (same style as your original viewer)
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

        # Latest planned route (map coords for drawing)
        self.route_map_x: List[float] = []
        self.route_map_y: List[float] = []
        self.route_vel_map_u: List[float] = []  # velocity vector in map-units (pixels per second)
        self.route_vel_map_v: List[float] = []

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
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.set_title("Route Map Viewer")

        self.im = self.ax.imshow(self.map_data, cmap="gray", origin="lower")

        # Drone marker
        self.drone_point, = self.ax.plot([], [], "ro", markersize=5, label="drone")

        # Target rectangle
        self.target_rect = None
        self._draw_target_rectangle()

        # Route polyline
        self.route_line, = self.ax.plot([], [], "-", linewidth=2, label="route")

        # Velocity arrows (optional)
        self.vel_quiver = None  # created on-demand

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

        self.get_logger().info("RouteMapViewer ready. Click to set target, press 't' to type target.")

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

        # If we never planned yet, plan now
        if self._last_plan_start_world is None or self._last_plan_goal_world is None:
            self._need_replan = True
        else:
            # Replan if goal changed (target moved)
            gx0, gy0 = self._last_plan_goal_world
            gx1, gy1 = goal_world
            goal_changed = (abs(gx1 - gx0) > 1e-6) or (abs(gy1 - gy0) > 1e-6)

            # Replan if start moved enough
            sx0, sy0 = self._last_plan_start_world
            sx1, sy1 = start_world
            start_moved = ((sx1 - sx0) ** 2 + (sy1 - sy0) ** 2) ** 0.5 > self.replan_if_start_moved_m

            self._need_replan = self._need_replan or goal_changed or start_moved

        if not self._need_replan:
            return

        # Fire async service call
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

        # Timeout guard (service can still return late; we ignore if too late)
        if hasattr(self, "_plan_sent_time") and (time.time() - self._plan_sent_time) > self.planner_timeout_sec:
            self.get_logger().warn("Planner response arrived after timeout window; ignoring.")
            return

        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Planner call failed: {e}")
            self.route_map_x, self.route_map_y = [], []
            self.route_vel_map_u, self.route_vel_map_v = [], []
            return

        if res is None or not res.success:
            msg = res.message if res is not None else "None response"
            self.get_logger().warn(f"No path: {msg}")
            self.route_map_x, self.route_map_y = [], []
            self.route_vel_map_u, self.route_vel_map_v = [], []
            return

        # Convert world waypoints -> map coords for plotting
        n = len(res.waypoints_x)
        mx, my = [], []
        for i in range(n):
            xf, yf = self.world_to_map_f(res.waypoints_x[i], res.waypoints_y[i])
            mx.append(xf)
            my.append(yf)

        self.route_map_x = mx
        self.route_map_y = my

        # Optional velocity arrows
        self.route_vel_map_u, self.route_vel_map_v = [], []
        has_vel = (len(res.velocities_x) == n and len(res.velocities_y) == n)
        if self.show_velocity_arrows and has_vel and n > 0:
            # Convert world velocities (m/s) -> map velocities (pixels/s)
            # (pixels = meters / resolution)
            scale = 1.0 / self.resolution
            for i in range(n):
                self.route_vel_map_u.append(res.velocities_x[i] * scale)
                self.route_vel_map_v.append(res.velocities_y[i] * scale)

        self.get_logger().info(f"Planned route received: {n} waypoints")

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

        # Update route polyline
        if len(self.route_map_x) >= 2:
            self.route_line.set_data(self.route_map_x, self.route_map_y)
        else:
            self.route_line.set_data([], [])

        # Update velocity quiver (recreate for simplicity)
        if self.show_velocity_arrows and len(self.route_vel_map_u) > 0 and len(self.route_map_x) > 0:
            if self.vel_quiver is not None:
                self.vel_quiver.remove()
                self.vel_quiver = None

            stride = max(1, self.velocity_arrow_stride)
            xs = np.array(self.route_map_x[::stride], dtype=float)
            ys = np.array(self.route_map_y[::stride], dtype=float)
            us = np.array(self.route_vel_map_u[::stride], dtype=float)
            vs = np.array(self.route_vel_map_v[::stride], dtype=float)

            # arrows in "map pixel space"
            self.vel_quiver = self.ax.quiver(xs, ys, us, vs, angles="xy", scale_units="xy", scale=1.0)
        else:
            if self.vel_quiver is not None:
                self.vel_quiver.remove()
                self.vel_quiver = None

        # Update status text
        tx, ty = self.target_map
        txw, tyw = self.map_to_world(float(tx), float(ty))

        if self.drone_pose_world is None:
            self.status_text.set_text(
                f"Target map=({tx},{ty}) world=({txw:.2f},{tyw:.2f})\n"
                f"Drone: waiting for pose..."
            )
        else:
            dxw, dyw = self.drone_pose_world
            dxm, dym = self.world_to_map_f(dxw, dyw)
            self.status_text.set_text(
                f"Drone world=({dxw:.2f},{dyw:.2f}) map=({dxm:.1f},{dym:.1f})\n"
                f"Target map=({tx},{ty}) world=({txw:.2f},{tyw:.2f})\n"
                f"Route points: {len(self.route_map_x)}"
            )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    # ----------------------------
    # UI events
    # ----------------------------
    def _on_key(self, event) -> None:
        if event.key == "t":
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
