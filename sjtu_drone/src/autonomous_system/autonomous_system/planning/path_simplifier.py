#!/usr/bin/env python3
"""
Path Simplification Utilities
-----------------------------
Provides:
 - Ramer–Douglas–Peucker (RDP) simplification for noisy grid paths
 - Turn-based waypoint extraction with minimum spacing
"""

import math
from typing import List, Tuple

GridPoint = Tuple[int, int]


def point_line_distance(p: GridPoint, a: GridPoint, b: GridPoint) -> float:
    """Perpendicular distance from point p to line segment a-b in grid coordinates."""
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
    projx = ax + t * vx
    projy = ay + t * vy
    return math.hypot(px - projx, py - projy)


def rdp_simplify(path: List[GridPoint], eps: float = 1.5) -> List[GridPoint]:
    """
    RDP simplification for integer grid paths.

    Args:
        path: List of (x,y) grid cells.
        eps: Distance threshold (larger = more aggressive simplification).

    Returns:
        Simplified list of (x,y) grid cells.
    """
    if len(path) <= 2:
        return path[:]

    a, b = path[0], path[-1]
    idx = -1
    dmax = -1.0

    for i in range(1, len(path) - 1):
        d = point_line_distance(path[i], a, b)
        if d > dmax:
            idx, dmax = i, d

    if dmax > eps:
        left = rdp_simplify(path[: idx + 1], eps)
        right = rdp_simplify(path[idx:], eps)
        return left[:-1] + right
    else:
        return [a, b]


def extract_turn_points(path: List[GridPoint], min_dist: float = 6.0) -> List[GridPoint]:
    """
    Extract a minimal set of turning waypoints from a simplified path.

    Keeps points where:
        - The direction changes between steps, OR
        - The distance from the last saved waypoint exceeds min_dist.
    """
    if len(path) < 3:
        return path[:]

    waypoints = [path[0]]
    prev_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])
    last_saved = path[0]

    for i in range(2, len(path)):
        cur_dir = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        dist_from_last = math.hypot(
            path[i - 1][0] - last_saved[0],
            path[i - 1][1] - last_saved[1],
        )

        # Save waypoint if direction changed or spacing exceeded
        if cur_dir != prev_dir or dist_from_last >= min_dist:
            waypoints.append(path[i - 1])
            last_saved = path[i - 1]

        prev_dir = cur_dir

    if waypoints[-1] != path[-1]:
        waypoints.append(path[-1])

    return waypoints
