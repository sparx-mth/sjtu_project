#!/usr/bin/env python3
"""
show_drone_map.py
-----------------
Live 2D map viewer for a drone in Gazebo.
Allows real-time updating of the target location:
 - Press 't' → GUI popup for new target
 - Click on the map → set the clicked pixel as new target
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import tkinter as tk
from tkinter import simpledialog


class MapPositionViewer(Node):
    def __init__(self):
        super().__init__('map_position_viewer')

        # === Load map ===
        map_yaml_path = "/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml"
        with open(map_yaml_path, 'r') as f:
            info = yaml.safe_load(f)

        self.resolution = info['resolution']
        self.origin = info['origin']

        map_image_path = info['image']
        if not map_image_path.startswith('/'):
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)

        img = cv2.imread(map_image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(f"Failed to load map image: {map_image_path}")

        self.map_data = np.zeros_like(img, dtype=np.uint8)
        self.map_data[img < 50] = 1
        self.map_data = np.flipud(self.map_data)

        self.get_logger().info(f"Map loaded: {map_image_path}, size={self.map_data.shape}")

        # === Default target (map pixel coords) ===
        self.target_map = (355, 593)
        self.target_size = 20
        self.target_rect = None

        # === Drone pose ===
        self.drone_pose = None
        self.pose_sub = self.create_subscription(
            Pose, '/simple_drone/gt_pose', self.pose_callback, 10
        )

        # === Matplotlib interactive window ===
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))

        self.im = self.ax.imshow(self.map_data, cmap='gray', origin='lower')
        self.point, = self.ax.plot([], [], 'ro', markersize=5)

        # connect keyboard + mouse handlers
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        self.draw_target_rectangle()

        self.ax.set_title("Drone position on occupancy map")
        plt.show(block=False)

    # === ROS pose callback ===
    def pose_callback(self, msg):
        self.drone_pose = (msg.position.x, msg.position.y)

    # === Conversions ===
    def world_to_map(self, x_world, y_world):
        ox, oy, _ = self.origin
        return int((x_world - ox) / self.resolution), int((y_world - oy) / self.resolution)

    # === Update target rectangle ===
    def draw_target_rectangle(self):
        if self.target_rect is not None:
            self.target_rect.remove()

        x_t, y_t = self.target_map
        s = self.target_size

        self.target_rect = plt.Rectangle(
            (x_t - s, y_t - s), 2 * s, 2 * s,
            linewidth=2, edgecolor='lime', facecolor='none'
        )
        self.ax.add_patch(self.target_rect)

        self.get_logger().info(f"Target updated → map=({x_t}, {y_t})")
        plt.draw()

    # === Update drone marker ===
    def update_display(self):
        if self.drone_pose is None:
            return

        x_map, y_map = self.world_to_map(*self.drone_pose)
        self.point.set_data([x_map], [y_map])
        self.ax.set_title(
            f"Drone Position: map=({x_map}, {y_map})  Target=({self.target_map[0]},{self.target_map[1]})"
        )
        plt.pause(0.05)

    # === Keyboard event ('t' to change target) ===
    def on_key(self, event):
        if event.key == 't':
            root = tk.Tk()
            root.withdraw()

            try:
                x = simpledialog.askinteger("New Target X", "Enter target X (map pixel):", parent=root)
                if x is None:
                    return
                y = simpledialog.askinteger("New Target Y", "Enter target Y (map pixel):", parent=root)
                if y is None:
                    return

                self.target_map = (x, y)
                self.draw_target_rectangle()

            finally:
                root.destroy()

    # === Mouse click event (pick coordinates) ===
    def on_click(self, event):
        if event.inaxes != self.ax:
            return

        x = int(event.xdata)
        y = int(event.ydata)

        self.target_map = (x, y)
        self.draw_target_rectangle()

        print(f"[CLICK] New target = ({x}, {y})")

    # === Loop ===
    def spin(self):
        self.get_logger().info("Waiting for /simple_drone/gt_pose messages...")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.update_display()


def main():
    rclpy.init()
    node = MapPositionViewer()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
