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

        # Text for coordinates display
        self.coord_text = self.ax.text(
            0.02, 0.02, '', transform=self.ax.transAxes,
            fontsize=9, verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
        )

        # connect keyboard + mouse handlers
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        self.draw_target_rectangle()

        self.ax.set_title("Drone position on occupancy map")
        plt.show(block=False)

        # === Timer for non-blocking updates ===
        self.create_timer(0.1, self.update_display)

        self.get_logger().info("Waiting for /simple_drone/gt_pose messages...")

    # === ROS pose callback ===
    def pose_callback(self, msg):
        self.drone_pose = (msg.position.x, msg.position.y)

    # === Conversions ===
    def world_to_map(self, x_world, y_world):
        ox, oy, _ = self.origin
        return int(round((x_world - ox) / self.resolution)), int(round((y_world - oy) / self.resolution))

    def map_to_world(self, x_map, y_map):
        ox, oy, _ = self.origin
        return x_map * self.resolution + ox, y_map * self.resolution + oy

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
        self.fig.canvas.draw_idle()

    # === Update drone marker ===
    def update_display(self):
        if self.drone_pose is None:
            self.fig.canvas.flush_events()
            return

        x_map, y_map = self.world_to_map(*self.drone_pose)
        x_world, y_world = self.drone_pose

        tx_map, ty_map = self.target_map
        tx_world, ty_world = self.map_to_world(tx_map, ty_map)

        self.point.set_data([x_map], [y_map])
        self.ax.set_title(f"Drone: grid=({x_map}, {y_map})  Target: grid=({tx_map}, {ty_map})")

        self.coord_text.set_text(
            f"Drone:  world=({x_world:.2f}, {y_world:.2f})\n"
            f"Target: world=({tx_world:.2f}, {ty_world:.2f})"
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

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

        x = int(round(event.xdata))
        y = int(round(event.ydata))

        self.target_map = (x, y)
        self.draw_target_rectangle()

        print(f"[CLICK] New target = ({x}, {y})")


def main():
    rclpy.init()
    node = MapPositionViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()