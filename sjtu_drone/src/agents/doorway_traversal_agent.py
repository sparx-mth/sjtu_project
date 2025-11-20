#!/usr/bin/env python3
"""
complete_door_navigation.py
--------------------------------
1. Load map (PGM + YAML)
2. Hold static door list (grid coords)
3. Receive robot grid position
4. Find nearest door
5. Compute pass-through target (behind the door)
6. Run A* to that target
7. Produce world-space waypoints
"""

import numpy as np
import imageio.v2 as imageio
import yaml
import math
import cv2
import os
import heapq
import matplotlib.pyplot as plt

# ============================================================
#   MAP LOADER
# ============================================================

class MapLoader:
    """Load occupancy map and convert grid<->world coordinates."""

    def __init__(self, grid, resolution, origin):
        self.grid = grid
        self.resolution = resolution
        self.origin = origin
        self.height, self.width = grid.shape

    @classmethod
    def load(cls, pgm_path, yaml_path):
        grid = imageio.imread(pgm_path)

        with open(yaml_path, "r") as f:
            cfg = yaml.safe_load(f)

        resolution = float(cfg["resolution"])
        origin = tuple(cfg["origin"])

        return cls(grid, resolution, origin)

    def grid_to_world(self, x, y):
        ox, oy, _ = self.origin
        return (
            ox + (x + 0.5) * self.resolution,
            oy + (y + 0.5) * self.resolution
        )


# ============================================================
#   A* PLANNER
# ============================================================

class AStarPlanner:
    def __init__(self, yaml_path):
        with open(yaml_path, 'r') as f:
            info = yaml.safe_load(f)

        self.resolution = info["resolution"]
        self.origin = info["origin"]

        img_path = info["image"]
        if not img_path.startswith("/"):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)

        img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        assert img is not None, f"Cannot load map: {img_path}"

        # Binary map: 0 = free, 1 = occupied
        self.map = np.zeros_like(img, dtype=np.uint8)
        self.map[img < 50] = 1
        self.map = np.flipud(self.map)

        # Inflate obstacles for safety
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
        self.map = cv2.dilate(self.map, kernel)

        self.h, self.w = self.map.shape

    def is_free(self, x, y):
        return (
            0 <= x < self.w and
            0 <= y < self.h and
            self.map[y, x] == 0
        )

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def neighbors(self, n):
        x, y = n
        moves = [(-1,0),(1,0),(0,-1),(0,1)]
        for dx, dy in moves:
            nx, ny = x+dx, y+dy
            if self.is_free(nx, ny):
                yield (nx, ny)

    def plan(self, start, goal):
        open_set = [(0, start)]
        came = {}
        g = {start: 0}
        f = {start: self.heuristic(start, goal)}

        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                return self.reconstruct(came, cur)

            for nb in self.neighbors(cur):
                cost = g[cur] + 1
                if nb not in g or cost < g[nb]:
                    came[nb] = cur
                    g[nb] = cost
                    f_nb = cost + self.heuristic(nb, goal)
                    f[nb] = f_nb
                    heapq.heappush(open_set, (f_nb, nb))

        return []  # no path

    def reconstruct(self, came, cur):
        path = [cur]
        while cur in came:
            cur = came[cur]
            path.append(cur)
        return path[::-1]

    def map_to_world(self, x, y):
        ox, oy, _ = self.origin
        return x * self.resolution + ox, y * self.resolution + oy


# ============================================================
#   DOOR AGENT
# ============================================================

class DoorAgent:
    """
    1. Find nearest door to robot
    2. Compute 'exit point' behind the door
    """

    def __init__(self, doors, resolution):
        self.doors = doors
        self.resolution = resolution

    def nearest_door(self, robot):
        rx, ry = robot
        return min(self.doors, key=lambda d: math.hypot(d[0]-rx, d[1]-ry))

    def exit_point(self, robot, door, dist_m=1.0):
        rx, ry = robot
        dx, dy = door

        vx = dx - rx
        vy = dy - ry
        L = math.hypot(vx, vy)
        if L < 1e-6: L = 1e-6

        vx /= L
        vy /= L

        cells = int(dist_m / self.resolution)
        tx = int(dx + vx * cells)
        ty = int(dy + vy * cells)
        return (tx, ty)


# ============================================================
#   VISUALIZATION
# ============================================================

def show_map_with_doors(grid, doors):
    plt.figure(figsize=(10, 10))
    plt.imshow(grid, cmap='gray')
    xs = [d[0] for d in doors]
    ys = [d[1] for d in doors]
    plt.scatter(xs, ys, c='red', marker='x')
    plt.title("Doors on map")
    plt.show()


# ============================================================
#   MAIN
# ============================================================

if __name__ == "__main__":

    pgm = "/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.pgm"
    yaml_path = "/root/sjtu_project/sjtu_drone/maps/hospital_map_cropped.yaml"

    loader = MapLoader.load(pgm, yaml_path)
    planner = AStarPlanner(yaml_path)

    # Doors in GRID coords
    doors = [
        (133, 75), (256, 75), (157, 252), (457, 225), (157, 299),
        (157, 649), (482, 298), (249, 475), (390, 475), (249, 624),
        (390, 624), (482, 649), (390, 875), (249, 875), (140, 862),
        (500, 862), (93, 996), (93, 1156), (150, 1175), (188, 1250),
        (545, 996), (545, 1156), (488, 1175), (448, 1250)
    ]

    agent = DoorAgent(doors, resolution=loader.resolution)

    # Robot's current grid pose (you can replace this with real-time code)
    rx = int(input("Robot X (grid): "))
    ry = int(input("Robot Y (grid): "))

    robot = (rx, ry)

    # 1. Find nearest door
    nearest = agent.nearest_door(robot)
    print("Nearest door:", nearest)

    # 2. Compute point behind the door
    target = agent.exit_point(robot, nearest, dist_m=1.0)
    print("Exit point:", target)

    # 3. Plan to that point
    path = planner.plan(robot, target)
    print("Path length:", len(path))

    # 4. Convert to world
    waypoints_world = [planner.map_to_world(*p) for p in path]

    print("\nWorld waypoints:")
    for w in waypoints_world:
        print(f"{w}")

    # 5. Show map for debugging
    show_map_with_doors(loader.grid, doors)
