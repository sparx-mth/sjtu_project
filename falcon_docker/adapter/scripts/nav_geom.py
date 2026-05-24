#!/usr/bin/env python3
"""
nav_geom.py — pure geometry for the VISUAL_SERVOING mode.

NO ROS imports here on purpose: every function is a plain numpy/math
helper so it can be unit-tested off-robot. The visual_servoing_controller
node is the only place that touches ROS; it calls into this module.

Frames
------
world  : the frame /<drone_ns>/gt_pose and /falcon/bev_2d live in.
body   : drone frame. +x forward, +y left, +z up. Drone is origin.
camera : optical frame. +z forward, +x right, +y down.

The body<->camera extrinsic is the SAME one falcon_adapter.py uses
(T_b_c), so pixels we compute here line up with what FALCON sees.
"""
import math
import numpy as np


# ── angles ──────────────────────────────────────────────────────────
def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(qx, qy, qz, qw):
    """Yaw (rad) from a quaternion. Robust to small roll/pitch."""
    s = 2.0 * (qw * qz + qx * qy)
    c = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(s, c)


# ── world <-> body SE(2) ────────────────────────────────────────────
def world_to_body(px, py, ox, oy, oyaw):
    """Express world point (px,py) in the body frame of a drone whose
    world pose is (ox, oy, oyaw). Returns (dx, dy): the point as seen
    from a drone that is, by definition, at (0,0) facing +x."""
    rx, ry = px - ox, py - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    return (c * rx - s * ry, s * rx + c * ry)


def body_to_world(dx, dy, ox, oy, oyaw):
    """Inverse of world_to_body. Anchor a body-frame point back into
    the world using the drone pose (ox, oy, oyaw)."""
    c, s = math.cos(oyaw), math.sin(oyaw)
    return (ox + c * dx - s * dy, oy + s * dx + c * dy)


# ── camera extrinsic (identical to falcon_adapter.T_b_c) ────────────
def make_T_b_c(cam_x=0.2, cam_y=0.0, cam_z=0.0):
    """4x4 camera->body transform. p_body = T_b_c @ p_cam."""
    return np.array([
        [0.0, 0.0, 1.0, cam_x],
        [-1.0, 0.0, 0.0, cam_y],
        [0.0, -1.0, 0.0, cam_z],
        [0.0, 0.0, 0.0, 1.0],
    ])


def point_body_to_pixel(dx, dy, dz, K, T_b_c):
    """Project a body-frame point to a pixel (u, v).
    Returns (u, v, depth_along_optical_axis) or None if behind camera.
    K = (fx, fy, cx, cy)."""
    T_c_b = np.linalg.inv(T_b_c)
    p_c = T_c_b @ np.array([dx, dy, dz, 1.0])
    Xc, Yc, Zc = p_c[0], p_c[1], p_c[2]
    if Zc <= 1e-3:
        return None
    fx, fy, cx, cy = K
    return (fx * Xc / Zc + cx, fy * Yc / Zc + cy, Zc)


def pixel_to_point_body(u, v, depth, K, T_b_c):
    """Back-project pixel (u,v) at metric `depth` (optical Z, metres)
    into the body frame. Returns (dx, dy, dz)."""
    fx, fy, cx, cy = K
    Xc = (u - cx) / fx * depth
    Yc = (v - cy) / fy * depth
    Zc = depth
    p_b = T_b_c @ np.array([Xc, Yc, Zc, 1.0])
    return (float(p_b[0]), float(p_b[1]), float(p_b[2]))


# ── FOV + occlusion against an OccupancyGrid ────────────────────────
class OccGrid:
    """Thin read-only view over a nav_msgs/OccupancyGrid for the
    occlusion ray-cast. Pass the .info fields + flat data list."""

    OCC = 100

    def __init__(self, width, height, res, origin_x, origin_y, data):
        self.w = int(width)
        self.h = int(height)
        self.res = float(res)
        self.ox = float(origin_x)
        self.oy = float(origin_y)
        self.data = np.asarray(data, dtype=np.int16).reshape(self.h, self.w)

    def value_at(self, wx, wy):
        cx = int((wx - self.ox) / self.res)
        cy = int((wy - self.oy) / self.res)
        if 0 <= cx < self.w and 0 <= cy < self.h:
            return int(self.data[cy, cx])
        return -1  # out of bounds → treat as unknown

    def is_occ(self, wx, wy):
        return self.value_at(wx, wy) == self.OCC


def in_fov(dx_body, dy_body, half_fov_rad, max_range_m, min_range_m=0.0):
    """True if a body-frame point lies inside the horizontal camera FOV
    cone and within the usable depth range."""
    r = math.hypot(dx_body, dy_body)
    if r < min_range_m or r > max_range_m:
        return False
    if dx_body <= 0.0:                       # behind the camera
        return False
    return abs(math.atan2(dy_body, dx_body)) <= half_fov_rad


def ray_is_clear(wx0, wy0, wx1, wy1, grid, step=None):
    """March a ray from (wx0,wy0) to (wx1,wy1) over `grid`. Returns
    False if any cell on the way is OCC (a known wall). UNKNOWN and
    FREE cells do NOT block — we only refuse *known* walls, matching
    the user's "not hidden behind a wall" requirement."""
    if grid is None:
        return True
    if step is None:
        step = max(grid.res * 0.5, 1e-3)
    dist = math.hypot(wx1 - wx0, wy1 - wy0)
    n = max(int(dist / step), 1)
    for i in range(1, n):          # skip i=0 (drone cell itself)
        t = i / float(n)
        if grid.is_occ(wx0 + t * (wx1 - wx0), wy0 + t * (wy1 - wy0)):
            return False
    return True


def furthest_visible_waypoint(path_xy, drone_xyz_yaw, grid,
                              half_fov_rad, max_range_m,
                              min_range_m=0.30):
    """Pick the FURTHEST waypoint on `path_xy` (world) that is both
    inside the camera FOV and not occluded by a known wall, as seen
    from the drone pose (x, y, yaw).

    Returns (index, (wx, wy), (dx_body, dy_body)) or None if nothing
    on the path is currently visible.
    """
    ox, oy, oyaw = drone_xyz_yaw
    for i in range(len(path_xy) - 1, -1, -1):
        wx, wy = path_xy[i]
        dx, dy = world_to_body(wx, wy, ox, oy, oyaw)
        if not in_fov(dx, dy, half_fov_rad, max_range_m, min_range_m):
            continue
        if not ray_is_clear(ox, oy, wx, wy, grid):
            continue
        return (i, (wx, wy), (dx, dy))
    return None