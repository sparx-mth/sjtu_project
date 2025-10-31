#!/usr/bin/env python3
"""
planner_a_star.py
-----------------
Perform A* path planning on an occupancy grid map,
and extract only key waypoints (start + turns + goal).
"""

import heapq, math, cv2, yaml, numpy as np, os


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
        assert img is not None, f"Cannot load map image {img_path}"

        self.map_data = np.zeros_like(img, dtype=np.uint8)
        self.map_data[img < 50] = 1
        self.map_data = np.flipud(self.map_data)

        # Inflate obstacles by 10 cells
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
        self.map_data = cv2.dilate(self.map_data, kernel)

        print(f"Map loaded: {img_path}, shape={self.map_data.shape}")

    def is_free(self, x, y):
        if 0 <= y < self.map_data.shape[0] and 0 <= x < self.map_data.shape[1]:
            return self.map_data[y, x] == 0
        return False

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def neighbors(self, node):
        x, y = node
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        return [(x+dx, y+dy) for dx, dy in moves if self.is_free(x+dx, y+dy)]

    def plan(self, start, goal):
        open_set = [(0, start)]
        came_from, g, f = {}, {start: 0}, {start: self.heuristic(start, goal)}
        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                return self.reconstruct_path(came_from, cur)
            for n in self.neighbors(cur):
                ng = g[cur] + 1
                if n not in g or ng < g[n]:
                    came_from[n], g[n] = cur, ng
                    f[n] = ng + self.heuristic(n, goal)
                    heapq.heappush(open_set, (f[n], n))
        print("No path found!")
        return []

    def reconstruct_path(self, came_from, cur):
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        return path[::-1]

    def extract_turn_points(self, path):
        if len(path) < 3:
            return path
        wp = [path[0]]
        prev = (path[1][0]-path[0][0], path[1][1]-path[0][1])
        for i in range(2, len(path)):
            cur = (path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])
            if cur != prev:
                wp.append(path[i-1])
            prev = cur
        wp.append(path[-1])
        return wp

    def map_to_world(self, x, y):
        ox, oy, _ = self.origin
        return x * self.resolution + ox, y * self.resolution + oy


def main():
    map_yaml = "/root/drone_workspace/sjtu_drone/maps/hospital_map_cropped.yaml"
    planner = AStarPlanner(map_yaml)
    start, goal = (346, 900), (355, 593)

    path = planner.plan(start, goal)
    print(f"Full path length: {len(path)}")
    waypoints = planner.extract_turn_points(path)
    print(f"Reduced waypoints ({len(waypoints)}):")
    for w in waypoints:
        wx, wy = planner.map_to_world(*w)
        print(f"  grid={w}, world=({wx:.2f},{wy:.2f})")


if __name__ == "__main__":
    main()
