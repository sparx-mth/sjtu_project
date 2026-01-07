#!/usr/bin/env python3
"""
show_drone_trajectory.py
------------------------
Live 2D map viewer for a drone in Gazebo.
Subscribes to /simple_drone/gt_pose and draws the trajectory over an occupancy map.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
from collections import deque


class MapTrajectoryViewer(Node):
    def __init__(self):
        super().__init__('map_trajectory_viewer')
        # === Goal (world coordinates) ===
        self.declare_parameter("goal_x", -5.499710)
        self.declare_parameter("goal_y", 7.285320)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)

        # === Load map YAML (must contain image, resolution, origin) ===
        map_yaml_path = "/ros2_ws/maps/playground_map.yaml"
        with open(map_yaml_path, 'r') as f:
            info = yaml.safe_load(f)

        self.resolution = float(info['resolution'])
        self.origin = info['origin']  # [ox, oy, yaw]

        self.goal_map = self.world_to_map(self.goal_x, self.goal_y)

        map_image_path = info['image']
        if not map_image_path.startswith('/'):
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)

        img = cv2.imread(map_image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(f"Failed to load map image: {map_image_path}")

        # Build occupancy-like binary map for display (works for PGM)
        # PGM usually: 0=occupied (black), 254/255=free (white)
        img_u8 = img.astype(np.uint8)
        self.map_data = np.zeros_like(img_u8, dtype=np.uint8)
        self.map_data[img_u8 < 50] = 1  # obstacles -> 1, free -> 0 (just for visualization)
        self.map_data = np.flipud(self.map_data)  # align with origin='lower'

        self.get_logger().info(f"Map loaded: {map_image_path}, size={self.map_data.shape}, res={self.resolution}, origin={self.origin}")

        # === Drone pose + trajectory ===
        self.drone_pose_world = None
        self.last_added_map = None

        # Keep a rolling buffer (so it won't grow forever)
        self.traj = deque(maxlen=20000)  # adjust if you want more/less

        # Add a point only if moved at least N pixels (reduces noise + improves performance)
        self.min_step_px = 1

        self.pose_sub = self.create_subscription(
            Pose, '/simple_drone/gt_pose', self.pose_callback, 10
        )

        # === Matplotlib interactive window ===
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.imshow(self.map_data, cmap='gray', origin='lower')

        # === Goal marker ===
        gx, gy = self.goal_map
        (self.goal_point,) = self.ax.plot(
            [gx], [gy],
            marker='x',
            markersize=12,
            markeredgewidth=3,
            color='lime',
            label='GOAL'
        )

        self.ax.legend(loc="upper right")

        # trajectory line + current position marker
        (self.traj_line,) = self.ax.plot([], [], '-', linewidth=2)   # default color
        (self.curr_point,) = self.ax.plot([], [], 'o', markersize=5) # default color

        self.ax.set_title("Drone trajectory on occupancy map")
        plt.show(block=False)

        # Timer for UI updates
        self.create_timer(0.05, self.update_display)

        self.get_logger().info("Waiting for /simple_drone/gt_pose messages...")

    def pose_callback(self, msg: Pose):
        xw, yw = msg.position.x, msg.position.y
        xm, ym = self.world_to_map(xw, yw)

        h, w = self.map_data.shape[:2]
        self.get_logger().info(
            f"[DBG] world=({xw:.3f},{yw:.3f}) -> map_px=({xm},{ym}) | res={self.resolution} origin={self.origin} size=({w},{h})"
        )
        self.drone_pose_world = (msg.position.x, msg.position.y)

        # Convert to map pixels
        x_map, y_map = self.world_to_map(*self.drone_pose_world)

        # Add to trajectory only if moved enough
        if self.last_added_map is None:
            self.traj.append((x_map, y_map))
            self.last_added_map = (x_map, y_map)
            return

        dx = abs(x_map - self.last_added_map[0])
        dy = abs(y_map - self.last_added_map[1])
        if dx >= self.min_step_px or dy >= self.min_step_px:
            self.traj.append((x_map, y_map))
            self.last_added_map = (x_map, y_map)

    # === Conversions ===
    def world_to_map(self, x_world, y_world):
        ox, oy, _ = self.origin
        x = int(round((x_world - ox) / self.resolution))
        y = int(round((y_world - oy) / self.resolution))
        return x, y
    
    def map_to_world(self, x_map, y_map):
        ox, oy, _ = self.origin
        x_world = x_map * self.resolution + ox
        y_world = y_map * self.resolution + oy
        return x_world, y_world

    def update_display(self):
        if self.drone_pose_world is None:
            self.fig.canvas.flush_events()
            return

        # Update current point
        x_map, y_map = self.world_to_map(*self.drone_pose_world)
        self.curr_point.set_data([x_map], [y_map])

        # Update trajectory polyline
        if len(self.traj) >= 2:
            xs = [p[0] for p in self.traj]
            ys = [p[1] for p in self.traj]
            self.traj_line.set_data(xs, ys)

        xw, yw = self.drone_pose_world
        self.ax.set_title(
            f"Drone trajectory | "
            f"Drone=({xw:.2f},{yw:.2f})  "
            f"Goal=({self.goal_x:.2f},{self.goal_y:.2f})  "
            f"points={len(self.traj)}"
        )
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = MapTrajectoryViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
