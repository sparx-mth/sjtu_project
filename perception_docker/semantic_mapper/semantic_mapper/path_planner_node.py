#!/usr/bin/env python3
"""
path_planner_node.py — A* 2D path planning on FALCON's BEV map.

Why A* and not RRT*/BIT* anymore: the BEV is already a discretized
occupancy grid (~160×340 cells at 0.15 m). On grids, A* is faster,
deterministic, and optimal — sampling-based planners only make sense
when the search space is continuous and high-dimensional. We were
fighting OMPL's pybind11/nanobind State construction quirks for no
gain. This version has zero external planning deps.

Inputs (all bridged from the ROS1 side):
  /falcon/bev_2d           nav_msgs/OccupancyGrid
  /odom_world              nav_msgs/Odometry
  /move_base_simple/goal   geometry_msgs/PoseStamped

Output:
  /planned_path            nav_msgs/Path  (visualize in RViz)

Pipeline per goal:
  1. Inflate the occupancy grid by drone radius.
  2. A* on 8-connected cells; octile heuristic; no diagonal corner-cutting.
  3. Greedy line-of-sight smoothing collapses long runs of cells.
  4. Convert to world coords, publish as nav_msgs/Path at flight altitude.
"""

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped


# ─────────────────────────────────────────────────────────────────────
#  Grid wrapper
# ─────────────────────────────────────────────────────────────────────
class Grid2D:
    """OccupancyGrid → inflated boolean obstacle mask."""

    def __init__(self, msg: OccupancyGrid, infl_m: float, unknown_is_free: bool):
        self.res = msg.info.resolution
        self.W = msg.info.width
        self.H = msg.info.height
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y

        g = np.array(msg.data, dtype=np.int8).reshape(self.H, self.W)
        blocked = (g == 100)
        if not unknown_is_free:
            blocked |= (g == -1)

        # 4-connected dilation by inflation radius.
        for _ in range(max(0, int(round(infl_m / self.res)))):
            out = blocked.copy()
            out[1:,  :] |= blocked[:-1, :]
            out[:-1, :] |= blocked[1:,  :]
            out[:, 1:]  |= blocked[:, :-1]
            out[:, :-1] |= blocked[:, 1:]
            blocked = out
        self.blocked = blocked

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int((x - self.ox) / self.res),
                int((y - self.oy) / self.res))

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        # Cell centers, not corners.
        return (self.ox + (cx + 0.5) * self.res,
                self.oy + (cy + 0.5) * self.res)

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.W and 0 <= cy < self.H

    def is_free_cell(self, cx: int, cy: int) -> bool:
        return self.in_bounds(cx, cy) and not bool(self.blocked[cy, cx])

    def is_free(self, x: float, y: float) -> bool:
        return self.is_free_cell(*self.world_to_cell(x, y))


# ─────────────────────────────────────────────────────────────────────
#  A* on the grid (8-connected, no diagonal corner-cutting)
# ─────────────────────────────────────────────────────────────────────
_SQRT2 = math.sqrt(2.0)
# (dx, dy, cost). Cardinals first so equal-cost ties prefer them.
_MOVES = (
    ( 1,  0, 1.0), (-1,  0, 1.0), (0,  1, 1.0), (0, -1, 1.0),
    ( 1,  1, _SQRT2), (1, -1, _SQRT2),
    (-1,  1, _SQRT2), (-1, -1, _SQRT2),
)


def _astar(blocked: np.ndarray, start: Tuple[int, int],
           goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    H, W = blocked.shape
    sx, sy = start
    gx, gy = goal
    if blocked[sy, sx] or blocked[gy, gx]:
        return None
    if start == goal:
        return [start]

    # Octile heuristic (admissible & consistent for 8-connected with sqrt2 cost).
    def h(cx, cy):
        dx = abs(cx - gx); dy = abs(cy - gy)
        return (dx + dy) + (_SQRT2 - 2.0) * min(dx, dy)

    open_h: List[Tuple[float, float, Tuple[int, int]]] = []
    heapq.heappush(open_h, (h(sx, sy), 0.0, start))
    came_from = {}
    g_score = {start: 0.0}

    while open_h:
        _, g, (cx, cy) = heapq.heappop(open_h)
        if (cx, cy) == goal:
            path = [(cx, cy)]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            return path[::-1]
        if g > g_score.get((cx, cy), float('inf')):
            continue
        for dx, dy, cost in _MOVES:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            if blocked[ny, nx]:
                continue
            # Don't allow diagonals through a corner of two adjacent obstacles.
            if dx and dy and (blocked[cy, nx] or blocked[ny, cx]):
                continue
            ng = g + cost
            if ng < g_score.get((nx, ny), float('inf')):
                g_score[(nx, ny)] = ng
                came_from[(nx, ny)] = (cx, cy)
                heapq.heappush(open_h, (ng + h(nx, ny), ng, (nx, ny)))
    return None


def _line_of_sight(blocked: np.ndarray,
                   a: Tuple[int, int], b: Tuple[int, int]) -> bool:
    """Bresenham. True iff every traversed cell is free."""
    x0, y0 = a; x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    cx, cy = x0, y0
    while True:
        if blocked[cy, cx]:
            return False
        if (cx, cy) == (x1, y1):
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy; cx += sx
        if e2 <  dx:
            err += dx; cy += sy


def _smooth(blocked: np.ndarray,
            cells: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """Greedy LOS shortcut. Replaces long staircase runs with straight lines."""
    if len(cells) < 3:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not _line_of_sight(blocked, cells[i], cells[j]):
            j -= 1
        out.append(cells[j])
        i = j
    return out


# ─────────────────────────────────────────────────────────────────────
#  Node
# ─────────────────────────────────────────────────────────────────────
class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.declare_parameter('bev_topic',          '/falcon/bev_2d')
        self.declare_parameter('odom_topic',         '/odom_world')
        self.declare_parameter('goal_topic',         '/move_base_simple/goal')
        self.declare_parameter('path_topic',         '/planned_path')
        self.declare_parameter('frame_id',           'world')
        self.declare_parameter('inflation_radius_m', 0.35)
        self.declare_parameter('unknown_is_free',    False)
        self.declare_parameter('plan_timeout_s',     3.0)   # logged-only safety check

        gp = lambda k: self.get_parameter(k).value
        self.bev_topic   = gp('bev_topic')
        self.odom_topic  = gp('odom_topic')
        self.goal_topic  = gp('goal_topic')
        self.path_topic  = gp('path_topic')
        self.frame_id    = gp('frame_id')
        self.inflation_m = float(gp('inflation_radius_m'))
        self.unk_free    = bool(gp('unknown_is_free'))
        self.timeout_s   = float(gp('plan_timeout_s'))

        self.grid: Optional[Grid2D] = None
        self.cur_xyz: Optional[Tuple[float, float, float]] = None

        self.path_pub = self.create_publisher(Path, self.path_topic, 1)
        self.create_subscription(OccupancyGrid, self.bev_topic,  self._bev_cb,  1)
        self.create_subscription(Odometry,      self.odom_topic, self._odom_cb, 10)
        self.create_subscription(PoseStamped,   self.goal_topic, self._goal_cb, 1)

        self.get_logger().info(
            f"path_planner ready (A*)  bev={self.bev_topic}  "
            f"odom={self.odom_topic}  goal={self.goal_topic}  "
            f"out={self.path_topic}  inflation={self.inflation_m:.2f}m")

    # ── Subs ──────────────────────────────────────────────────────
    def _bev_cb(self, msg: OccupancyGrid):
        self.grid = Grid2D(msg, self.inflation_m, self.unk_free)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.cur_xyz = (p.x, p.y, p.z)

    # ── Goal handler ──────────────────────────────────────────────
    def _goal_cb(self, msg: PoseStamped):
        if self.grid is None:
            self.get_logger().warn(f"no map yet on {self.bev_topic}");  return
        if self.cur_xyz is None:
            self.get_logger().warn(f"no odom yet on {self.odom_topic}"); return

        sx, sy, sz = self.cur_xyz
        gx, gy = msg.pose.position.x, msg.pose.position.y
        self.get_logger().info(f"plan ({sx:.2f},{sy:.2f}) -> ({gx:.2f},{gy:.2f})")

        if not self.grid.is_free(sx, sy):
            self.get_logger().warn(f"start ({sx:.2f},{sy:.2f}) blocked — abort"); return
        if not self.grid.is_free(gx, gy):
            self.get_logger().warn(f"goal  ({gx:.2f},{gy:.2f}) blocked — abort"); return

        cs = self.grid.world_to_cell(sx, sy)
        cg = self.grid.world_to_cell(gx, gy)

        t0 = self.get_clock().now()
        cells = _astar(self.grid.blocked, cs, cg)
        dt = (self.get_clock().now() - t0).nanoseconds * 1e-9

        if cells is None:
            self.get_logger().warn(f"A* found no path  ({dt*1000:.0f} ms)")
            return
        if dt > self.timeout_s:
            self.get_logger().warn(
                f"A* exceeded plan_timeout_s ({dt:.2f}s); using result anyway")

        smoothed = _smooth(self.grid.blocked, cells)
        # World coords, snapped to cell centers.
        pts = [self.grid.cell_to_world(*c) for c in smoothed]
        # Force the very first/last waypoint to be the actual start/goal so
        # the tracker doesn't get a sub-cell jump at endpoints.
        if pts:
            pts[0]  = (sx, sy)
            pts[-1] = (gx, gy)

        self._publish_path(pts, sz, dt_ms=dt * 1000.0,
                           raw=len(cells), final=len(pts))

    # ── Output ────────────────────────────────────────────────────
    def _publish_path(self, pts, z, dt_ms, raw, final):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.get_logger().info(
            f"published path  {raw} cells -> {final} waypoints  "
            f"({dt_ms:.0f} ms)")


def main():
    rclpy.init()
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()