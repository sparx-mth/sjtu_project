#!/usr/bin/env python3
"""
Frontier Exploration Agent (Service-based)
-------------------------------------------
Explores using frontiers from the observed map.
Gets map from exploration node (Int8MultiArray format).

Map values:
    -1 = Unknown
     0 = Known free
     1 = Known wall
"""

import math
import time
import threading
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8MultiArray
from autonomous_system.srv import NavigateToPose

GridPoint = Tuple[int, int]


class FrontierExplorationService(Node):
    """
    Frontier-based exploration using map from exploration node.

    Subscribes to /exploration/observed_map (Int8MultiArray).
    Uses /navigate_to_pose service for navigation.
    """

    def __init__(self):
        super().__init__("frontier_exploration_service")

        # Parameters
        self.declare_parameter("cruise_altitude", 1.5)
        self.declare_parameter("exploration_timeout", 300.0)
        self.declare_parameter("min_frontier_distance", 1.0)  # meters

        self.default_altitude = float(self.get_parameter("cruise_altitude").value)
        self.exploration_timeout = float(self.get_parameter("exploration_timeout").value)
        self.min_frontier_distance = float(self.get_parameter("min_frontier_distance").value)

        # Map from exploration node
        self.map_cb_group = ReentrantCallbackGroup()
        self.map_lock = threading.Lock()
        self.map_data: Optional[np.ndarray] = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.05
        self.origin = (0.0, 0.0, 0.0)
        self.map_received = False

        # Subscribe to Int8MultiArray map (same as navigation agent)
        self.map_sub = self.create_subscription(
            Int8MultiArray,
            "/exploration/observed_map",
            self._map_callback,
            10,
            callback_group=self.map_cb_group
        )

        # Pose
        self.pose_cb_group = ReentrantCallbackGroup()
        self.pose: Optional[Pose] = None
        self.pose_sub = self.create_subscription(
            Pose, "/simple_drone/gt_pose", self._pose_cb, 10,
            callback_group=self.pose_cb_group
        )

        # Navigation client
        self.nav_client = self.create_client(NavigateToPose, "/navigate_to_pose")
        self.get_logger().info("Waiting for /navigate_to_pose service...")
        self.nav_client.wait_for_service()

        # Service
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(
            NavigateToPose,
            "/explore_frontiers",
            self._handle_explore_request,
            callback_group=self.service_cb_group
        )

        self.busy = False
        self.get_logger().info(f"FrontierExplorationService ready. Timeout: {self.exploration_timeout}s")
        self.get_logger().info("Waiting for exploration map on /exploration/observed_map...")

    def _map_callback(self, msg: Int8MultiArray):
        """Handle map updates from Int8MultiArray format."""
        if len(msg.layout.dim) < 2:
            return

        height = msg.layout.dim[0].size
        width = msg.layout.dim[1].size

        # Get metadata from dimension label (same format as navigation agent)
        # Format: "resolution,origin_x,origin_y"
        try:
            parts = msg.layout.dim[0].label.split(",")
            resolution = float(parts[0])
            origin_x = float(parts[1])
            origin_y = float(parts[2])
        except (IndexError, ValueError):
            resolution = 0.05
            origin_x, origin_y = -25.0, -30.0

        with self.map_lock:
            self.map_width = width
            self.map_height = height
            self.resolution = resolution
            self.origin = (origin_x, origin_y, 0.0)
            self.map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        if not self.map_received:
            self.map_received = True
            self.get_logger().info(f"First map received: {width}x{height}, res={resolution}")

    def _pose_cb(self, msg: Pose):
        self.pose = msg

    def world_to_grid(self, wx: float, wy: float) -> GridPoint:
        ox, oy, _ = self.origin
        gx = int(round((wx - ox) / self.resolution))
        gy = int(round((wy - oy) / self.resolution))
        return (gx, gy)

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        ox, oy, _ = self.origin
        wx = gx * self.resolution + ox
        wy = gy * self.resolution + oy
        return (wx, wy)

    def find_frontiers(self) -> List[GridPoint]:
        """
        Find frontiers: explored free cells (0) adjacent to unknown (-1).
        """
        with self.map_lock:
            if self.map_data is None:
                return []

            frontiers = []
            for gy in range(1, self.map_height - 1):
                for gx in range(1, self.map_width - 1):
                    # Must be explored free (value 0)
                    if self.map_data[gy, gx] != 0:
                        continue

                    # Check if adjacent to unknown (-1)
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nx, ny = gx + dx, gy + dy
                        if self.map_data[ny, nx] == -1:
                            frontiers.append((gx, gy))
                            break

            return frontiers

    def cluster_frontiers(self, frontiers: List[GridPoint], min_dist_cells: int = 10) -> List[GridPoint]:
        """
        Cluster nearby frontiers and return representative points.
        This reduces the number of targets and picks better exploration points.
        """
        if not frontiers:
            return []

        # Simple clustering: pick frontiers that are far enough apart
        clustered = []
        for f in frontiers:
            is_new_cluster = True
            for c in clustered:
                dist = math.hypot(f[0] - c[0], f[1] - c[1])
                if dist < min_dist_cells:
                    is_new_cluster = False
                    break
            if is_new_cluster:
                clustered.append(f)

        return clustered

    def find_best_frontier(self, frontiers: List[GridPoint]) -> Optional[GridPoint]:
        """
        Find the best frontier to explore.
        Prioritizes frontiers that are not too close but also not too far.
        """
        if not frontiers or self.pose is None:
            return None

        gx, gy = self.world_to_grid(self.pose.position.x, self.pose.position.y)
        min_dist_cells = int(self.min_frontier_distance / self.resolution)

        # Filter out frontiers that are too close
        valid_frontiers = [
            f for f in frontiers
            if math.hypot(f[0] - gx, f[1] - gy) > min_dist_cells
        ]

        if not valid_frontiers:
            # If all are too close, just pick the closest one
            return min(frontiers, key=lambda f: math.hypot(f[0] - gx, f[1] - gy))

        # Pick the closest valid frontier
        return min(valid_frontiers, key=lambda f: math.hypot(f[0] - gx, f[1] - gy))

    def navigate_to(self, wx: float, wy: float, wz: float) -> bool:
        """Navigate to a world coordinate using the navigation service."""
        req = NavigateToPose.Request()
        req.x, req.y, req.z = wx, wy, wz
        future = self.nav_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return False
        return future.result().success

    def get_exploration_progress(self) -> float:
        """Get exploration progress as percentage."""
        with self.map_lock:
            if self.map_data is None:
                return 0.0
            total = self.map_data.size
            explored = np.sum(self.map_data != -1)
            return (explored / total) * 100.0

    def _handle_explore_request(
            self,
            request: NavigateToPose.Request,
            response: NavigateToPose.Response
    ) -> NavigateToPose.Response:

        if self.busy:
            response.success = False
            response.message = "Exploration agent is busy."
            return response

        self.busy = True
        cruise_altitude = request.z if request.z > 0 else self.default_altitude
        start_time = time.time()

        self.get_logger().info(f"Starting frontier exploration (timeout: {self.exploration_timeout}s)")

        # Wait for map and pose
        wait_start = time.time()
        while (self.map_data is None or self.pose is None) and time.time() - wait_start < 10.0:
            time.sleep(0.1)

        if self.map_data is None:
            response.success = False
            response.message = "No map received."
            self.busy = False
            return response

        if self.pose is None:
            response.success = False
            response.message = "No pose received."
            self.busy = False
            return response

        iteration = 0
        frontiers_visited = 0
        failed_attempts = 0
        max_failed_attempts = 5

        while rclpy.ok():
            iteration += 1
            elapsed = time.time() - start_time

            if elapsed >= self.exploration_timeout:
                progress = self.get_exploration_progress()
                response.success = False
                response.message = f"Timeout after {elapsed:.1f}s. Progress: {progress:.1f}%. Visited {frontiers_visited} frontiers."
                self.get_logger().info(response.message)
                self.busy = False
                return response

            progress = self.get_exploration_progress()
            frontiers = self.find_frontiers()

            if not frontiers:
                response.success = True
                response.message = f"Exploration complete! {progress:.1f}% explored in {elapsed:.1f}s. Visited {frontiers_visited} frontiers."
                self.get_logger().info(response.message)
                self.busy = False
                return response

            # Cluster frontiers to reduce targets
            clustered = self.cluster_frontiers(frontiers)
            self.get_logger().info(
                f"[{iteration}] {progress:.1f}% explored, "
                f"{len(frontiers)} frontier cells, {len(clustered)} clusters"
            )

            target = self.find_best_frontier(clustered)
            if target is None:
                failed_attempts += 1
                if failed_attempts >= max_failed_attempts:
                    response.success = False
                    response.message = f"No valid frontiers found after {failed_attempts} attempts."
                    self.busy = False
                    return response
                time.sleep(0.5)
                continue

            wx, wy = self.grid_to_world(target[0], target[1])
            self.get_logger().info(f"  -> Navigating to frontier at ({wx:.2f}, {wy:.2f})")

            if self.navigate_to(wx, wy, cruise_altitude):
                frontiers_visited += 1
                failed_attempts = 0
                self.get_logger().info(f"  -> Reached frontier #{frontiers_visited}")
            else:
                failed_attempts += 1
                self.get_logger().warn(f"  -> Failed to reach frontier (attempt {failed_attempts})")

            if failed_attempts >= max_failed_attempts:
                response.success = False
                response.message = f"Too many navigation failures. Progress: {progress:.1f}%."
                self.busy = False
                return response

            time.sleep(0.3)

        response.success = False
        response.message = "Interrupted."
        self.busy = False
        return response


def main():
    rclpy.init()
    node = FrontierExplorationService()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()