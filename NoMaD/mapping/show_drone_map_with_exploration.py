#!/usr/bin/env python3
"""
show_drone_map_with_exploration.py
-----------------------------------
Live 2D map viewer for a drone in Gazebo with FOG OF WAR exploration.

Publishes the observed map for navigation agents to use.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Circle
import os
import tkinter as tk
from tkinter import simpledialog
from datetime import datetime


class ExplorationMapViewer(Node):
    def __init__(self):
        super().__init__('exploration_map_viewer')

        self.declare_parameter('exploration_radius', 60)
        self.declare_parameter('exploration_radius_meters', -1.0)
        self.declare_parameter('map_yaml', '/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml')
        self.declare_parameter('publish_rate', 15.0)  # Hz - faster for navigation
        self.declare_parameter('num_rays', 360)

        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.exploration_radius_meters = self.get_parameter('exploration_radius_meters').value
        map_yaml_path = self.get_parameter('map_yaml').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.num_rays = self.get_parameter('num_rays').value

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

        # Ground truth: 0 = free, 1 = occupied (wall)
        self.ground_truth_map = np.zeros_like(img, dtype=np.uint8)
        self.ground_truth_map[img < 50] = 1
        self.ground_truth_map = np.flipud(self.ground_truth_map)

        self.map_height, self.map_width = self.ground_truth_map.shape
        self.get_logger().info(f"Map loaded: {map_image_path}, size={self.ground_truth_map.shape}")

        if self.exploration_radius_meters > 0:
            self.exploration_radius = int(self.exploration_radius_meters / self.resolution)

        self.exploration_mask = np.zeros((self.map_height, self.map_width), dtype=np.uint8)

        # Display map: -1 = unexplored (gray), 0 = free (white), 1 = occupied (black)
        self.display_map = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        self.target_map = (355, 593)
        self.target_size = 20
        self.target_rect = None

        self.drone_pose = None
        self.prev_drone_grid = None
        self.pose_sub = self.create_subscription(
            Pose, '/simple_drone/gt_pose', self.pose_callback, 10
        )

        # Publisher for observed map (Int8MultiArray with metadata)
        self.observed_map_pub = self.create_publisher(
            Int8MultiArray, '/exploration/observed_map', 10
        )

        # Faster publish timer for navigation
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_observed_map
        )

        self.total_cells = self.map_height * self.map_width
        self.explored_cells = 0
        self.exploration_percentage = 0.0

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))

        cmap = mcolors.ListedColormap(['#808080', 'white', 'black'])
        bounds = [-1.5, -0.5, 0.5, 1.5]
        norm = mcolors.BoundaryNorm(bounds, cmap.N)

        self.im = self.ax.imshow(self.display_map, cmap=cmap, norm=norm, origin='lower')
        self.drone_point, = self.ax.plot([], [], 'ro', markersize=8, label='Drone')

        self.radius_circle = Circle((0, 0), self.exploration_radius,
                                    fill=False, color='cyan', linestyle='--', linewidth=1.5, alpha=0.7)
        self.ax.add_patch(self.radius_circle)

        self.coord_text = self.ax.text(
            0.02, 0.02, '', transform=self.ax.transAxes,
            fontsize=9, verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
        )

        self.stats_text = self.ax.text(
            0.98, 0.98, '', transform=self.ax.transAxes,
            fontsize=9, verticalalignment='top', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8)
        )

        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        self.draw_target_rectangle()

        self.ax.set_title("Drone Exploration Map (Fog of War + Line of Sight)")
        self.ax.legend(loc='upper left')
        plt.show(block=False)

        self.create_timer(0.1, self.update_display)

        self.get_logger().info("Exploration Map Viewer ready.")
        self.get_logger().info(f"Publishing observed map at {self.publish_rate} Hz")

    def pose_callback(self, msg):
        self.drone_pose = (msg.position.x, msg.position.y)

    def world_to_map(self, x_world, y_world):
        ox, oy, _ = self.origin
        return int(round((x_world - ox) / self.resolution)), int(round((y_world - oy) / self.resolution))

    def map_to_world(self, x_map, y_map):
        ox, oy, _ = self.origin
        return x_map * self.resolution + ox, y_map * self.resolution + oy

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for raycasting."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        if dx > dy:
            err = dx / 2
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        cells.append((x1, y1))
        return cells

    def raycast_reveal(self, center_x, center_y):
        """Reveal visible cells using raycasting."""
        new_cells_revealed = 0
        radius = self.exploration_radius

        for i in range(self.num_rays):
            angle = 2 * np.pi * i / self.num_rays
            end_x = int(center_x + radius * np.cos(angle))
            end_y = int(center_y + radius * np.sin(angle))
            ray_cells = self.bresenham_line(center_x, center_y, end_x, end_y)

            for px, py in ray_cells:
                if not (0 <= px < self.map_width and 0 <= py < self.map_height):
                    break

                dist_sq = (px - center_x) ** 2 + (py - center_y) ** 2
                if dist_sq > radius * radius:
                    break

                if self.exploration_mask[py, px] == 0:
                    self.exploration_mask[py, px] = 1
                    self.display_map[py, px] = self.ground_truth_map[py, px]
                    new_cells_revealed += 1

                if self.ground_truth_map[py, px] == 1:
                    break

        if new_cells_revealed > 0:
            self.explored_cells += new_cells_revealed
            self.exploration_percentage = (self.explored_cells / self.total_cells) * 100

    def publish_observed_map(self):
        """Publish the current observed map for navigation agents."""
        msg = Int8MultiArray()

        # Store metadata in dimension labels: "resolution,origin_x,origin_y"
        dim0 = MultiArrayDimension()
        dim0.label = f"{self.resolution},{self.origin[0]},{self.origin[1]}"
        dim0.size = self.map_height
        dim0.stride = self.map_height * self.map_width

        dim1 = MultiArrayDimension()
        dim1.label = "width"
        dim1.size = self.map_width
        dim1.stride = self.map_width

        msg.layout.dim = [dim0, dim1]
        msg.layout.data_offset = 0

        # Flatten the display_map (-1=unknown, 0=free, 1=wall)
        msg.data = self.display_map.flatten().tolist()

        self.observed_map_pub.publish(msg)

    def reset_exploration(self):
        self.exploration_mask.fill(0)
        self.display_map.fill(-1)
        self.explored_cells = 0
        self.exploration_percentage = 0.0
        self.prev_drone_grid = None
        self.get_logger().info("Exploration reset!")

    def save_exploration_map(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        mask_path = f"/tmp/exploration_mask_{timestamp}.png"
        cv2.imwrite(mask_path, (self.exploration_mask * 255).astype(np.uint8))

        display_path = f"/tmp/exploration_display_{timestamp}.png"
        display_img = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
        display_img[self.display_map == -1] = 128
        display_img[self.display_map == 0] = 255
        display_img[self.display_map == 1] = 0
        cv2.imwrite(display_path, np.flipud(display_img))

        self.get_logger().info(f"Saved exploration maps to /tmp/")

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
        self.fig.canvas.draw_idle()

    def update_display(self):
        if self.drone_pose is None:
            self.fig.canvas.flush_events()
            return

        x_map, y_map = self.world_to_map(*self.drone_pose)
        x_world, y_world = self.drone_pose

        if self.prev_drone_grid is None or \
                abs(x_map - self.prev_drone_grid[0]) > 0 or \
                abs(y_map - self.prev_drone_grid[1]) > 0:
            self.raycast_reveal(x_map, y_map)
            self.prev_drone_grid = (x_map, y_map)
            self.im.set_data(self.display_map)

        self.drone_point.set_data([x_map], [y_map])
        self.radius_circle.center = (x_map, y_map)
        self.radius_circle.radius = self.exploration_radius

        tx_map, ty_map = self.target_map
        tx_world, ty_world = self.map_to_world(tx_map, ty_map)

        self.ax.set_title(f"Drone Exploration | Radius: {self.exploration_radius}px")

        self.coord_text.set_text(
            f"Drone:  world=({x_world:.2f}, {y_world:.2f}) grid=({x_map}, {y_map})\n"
            f"Target: world=({tx_world:.2f}, {ty_world:.2f}) grid=({tx_map}, {ty_map})"
        )

        self.stats_text.set_text(
            f"Explored: {self.exploration_percentage:.1f}%\n"
            f"Cells: {self.explored_cells:,} / {self.total_cells:,}"
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

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

        elif event.key == '+' or event.key == '=':
            self.exploration_radius = min(self.exploration_radius + 5, 200)

        elif event.key == '-':
            self.exploration_radius = max(self.exploration_radius - 5, 5)

        elif event.key == 'r':
            self.reset_exploration()

        elif event.key == 's':
            self.save_exploration_map()

    def on_click(self, event):
        if event.inaxes != self.ax:
            return
        x = int(round(event.xdata))
        y = int(round(event.ydata))
        self.target_map = (x, y)
        self.draw_target_rectangle()

    def get_observed_map(self):
        return self.display_map.copy()


def main():
    rclpy.init()
    node = ExplorationMapViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()