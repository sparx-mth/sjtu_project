#!/usr/bin/env python3
"""
autonomous_path_runner.py
-------------------------
1. Get current position from /simple_drone/gt_pose
2. Ask user for target grid coordinates (X,Y)
3. Plan path using A* on occupancy grid
4. Simplify path (RDP) and extract turn-based waypoints with spacing
5. Convert to world coords and navigate autonomously
"""

import rclpy, heapq, math, cv2, yaml, numpy as np, os, time
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist


# === Utilities ===
def _point_line_distance(p, a, b):
    """Perpendicular distance from point p to line segment a-b (in grid cells)."""
    (px, py), (ax, ay), (bx, by) = p, a, b
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    c1 = vx * wx + vy * wy
    if c1 <= 0:
        return math.hypot(px - ax, py - ay)
    c2 = vx * vx + vy * vy
    if c2 <= c1:
        return math.hypot(px - bx, py - by)
    t = c1 / c2
    projx, projy = ax + t * vx, ay + t * vy
    return math.hypot(px - projx, py - projy)

def rdp_simplify(path, eps=1.5):
    """Ramer–Douglas–Peucker simplification on integer grid path."""
    if len(path) <= 2:
        return path[:]
    a, b = path[0], path[-1]
    idx, dmax = -1, -1.0
    for i in range(1, len(path) - 1):
        d = _point_line_distance(path[i], a, b)
        if d > dmax:
            idx, dmax = i, d
    if dmax > eps:
        left = rdp_simplify(path[: idx + 1], eps)
        right = rdp_simplify(path[idx:], eps)
        return left[:-1] + right
    else:
        return [a, b]


# === A* Planner ===
class AStarPlanner:
    def __init__(self, map_yaml_path):
        with open(map_yaml_path, 'r') as f:
            info = yaml.safe_load(f)
        self.resolution = info['resolution']
        self.origin = info['origin']
        img_path = info['image']
        if not img_path.startswith('/'):
            img_path = os.path.join(os.path.dirname(map_yaml_path), img_path)
        img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        assert img is not None, f"Cannot load map {img_path}"

        self.map_data = np.zeros_like(img, dtype=np.uint8)
        self.map_data[img < 50] = 1  # 1 = occupied
        self.map_data = np.flipud(self.map_data)

        # Inflate obstacles by ~10 cells
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
        self.map_data = cv2.dilate(self.map_data, kernel)

        print(f"Map loaded: {img_path}, shape={self.map_data.shape}")

    def is_free(self, x, y):
        return 0 <= y < self.map_data.shape[0] and 0 <= x < self.map_data.shape[1] and self.map_data[y, x] == 0

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def neighbors(self, node):
        x, y = node
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connectivity (safe with inflation)
        return [(x+dx, y+dy) for dx, dy in moves if self.is_free(x+dx, y+dy)]

    def plan(self, start, goal):
        open_set = [(0, start)]
        came_from, g, f = {}, {start: 0}, {start: self.heuristic(start, goal)}
        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                return self._reconstruct(came_from, cur)
            for n in self.neighbors(cur):
                ng = g[cur] + 1
                if n not in g or ng < g[n]:
                    came_from[n], g[n] = cur, ng
                    f[n] = ng + self.heuristic(n, goal)
                    heapq.heappush(open_set, (f[n], n))
        print("No path found!")
        return []

    def _reconstruct(self, came_from, cur):
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        return path[::-1]

    def extract_turn_points(self, path, min_dist=6):
        """
        Keep major turns and ensure spacing >= min_dist grid cells.
        Apply after RDP simplification for best effect.
        """
        if len(path) < 3:
            return path[:]
        wp = [path[0]]
        prev_dir = (path[1][0]-path[0][0], path[1][1]-path[0][1])
        last_saved = path[0]

        for i in range(2, len(path)):
            cur_dir = (path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])
            dist_from_last = math.hypot(path[i-1][0]-last_saved[0], path[i-1][1]-last_saved[1])

            # save on real direction change OR spacing threshold
            if cur_dir != prev_dir or dist_from_last >= min_dist:
                wp.append(path[i-1])
                last_saved = path[i-1]
            prev_dir = cur_dir

        if wp[-1] != path[-1]:
            wp.append(path[-1])
        return wp

    def map_to_world(self, x, y):
        ox, oy, _ = self.origin
        return x * self.resolution + ox, y * self.resolution + oy


# === Drone Navigator ===
class WaypointNavigator(Node):
    def __init__(self, waypoints_world):
        super().__init__('waypoint_navigator')
        self.pose = Pose()
        self.pose_sub = self.create_subscription(Pose, '/simple_drone/gt_pose', self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.z_fixed, self.kp, self.tol = 1.5, 0.8, 0.05
        self.waypoints = waypoints_world
        self.get_logger().info("Waiting for pose...")
        while rclpy.ok() and self.pose.position.x == 0.0 and self.pose.position.y == 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        self.run_mission()

    def pose_cb(self, msg): self.pose = msg

    def move_to(self, tx, ty, tz):
        stable = 0
        while rclpy.ok():
            rclpy.spin_once(self)
            x, y, z = self.pose.position.x, self.pose.position.y, self.pose.position.z
            dx, dy, dz = tx - x, ty - y, tz - z
            dist = math.hypot(dx, dy)
            if dist < self.tol:
                stable += 1
                if stable > 10:
                    self.stop()
                    break
            else:
                stable = 0
                twist = Twist()
                twist.linear.x = max(min(self.kp * dx, 0.3), -0.3)
                twist.linear.y = max(min(self.kp * dy, 0.3), -0.3)
                twist.linear.z = max(min(self.kp * dz, 0.3), -0.3)
                self.cmd_pub.publish(twist)
            time.sleep(0.02)

    def stop(self):
        twist = Twist()
        for _ in range(10):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

    def run_mission(self):
        self.get_logger().info(f"Starting mission with {len(self.waypoints)} waypoints...")
        for i, (tx, ty) in enumerate(self.waypoints):
            self.get_logger().info(f"→ Waypoint {i+1}/{len(self.waypoints)}: ({tx:.2f}, {ty:.2f})")
            self.move_to(tx, ty, self.z_fixed)
            time.sleep(1.0)
        self.get_logger().info("Mission completed ✓")


# === Main orchestrator ===
def main():
    map_yaml = "/root/drone_workspace/sjtu_drone/maps/hospital_map_cropped.yaml"
    planner = AStarPlanner(map_yaml)

    # Get current pose and target (grid coords)
    start_x = int(input("Enter current grid X: "))
    start_y = int(input("Enter current grid Y: "))
    goal_x = int(input("Enter target grid X: "))
    goal_y = int(input("Enter target grid Y: "))
    start, goal = (start_x, start_y), (goal_x, goal_y)

    # Plan A*
    path = planner.plan(start, goal)
    if not path:
        print("No path found.")
        return
    print(f"Full path length: {len(path)}")

    # Simplify path + extract spaced turn points
    path_simpl = rdp_simplify(path, eps=1.5)              # reduce zigzags
    waypoints_grid = planner.extract_turn_points(path_simpl, min_dist=6)

    # Convert to world
    waypoints_world = [planner.map_to_world(*p) for p in waypoints_grid]

    # Print both grid and world points
    print(f"Reduced waypoints ({len(waypoints_grid)}):")
    for g, w in zip(waypoints_grid, waypoints_world):
        print(f"  grid={g} -> world=({w[0]:.2f}, {w[1]:.2f})")

    # Run navigation
    rclpy.init()
    WaypointNavigator(waypoints_world)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
