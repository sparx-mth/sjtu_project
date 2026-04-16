#!/usr/bin/env python3
"""
semantic_mapper_node.py — MORE-style topological mapper.

Doors are known a priori (Gazebo world coordinates), so we don't try to
auto-detect them. Each tick:

  1. depth → BEV log-odds (unchanged).
  2. Paint a short wall at EVERY hardcoded door so the free mask is cut
     into separate rooms the moment the drone starts exploring across a
     doorway. No "discovery" — a door is a door whether the drone has
     visited it or not; the cut only has visible effect once there's
     free space on at least one side.
  3. Connected-components label the cut free mask → one CC per room.
  4. PERSISTENT ROOM REGISTRY: IoU-match new CCs to the previous tick's
     rooms. Each room keeps its ID (and its colour) for the lifetime of
     the node.
  5. For the topological/MORE visualisation layer, compute the medial
     axis (Voronoi graph) of the free mask and publish it as a blue
     spine on the floor.
  6. For each door, sample an annulus just outside the wall and record
     which room IDs it touches. Those are its adjacent rooms, and the
     pair forms a topological edge.
  7. Publish:
       /scene_graph/bev        OccupancyGrid
       /scene_graph/markers    MarkerArray (room fills, centroids,
                                 labels, doors, skeleton, room↔room
                                 edges)
       /scene_graph            JSON String: rooms + doors + edges,
                                 stable IDs across ticks.

QoS note: depth and camera_info use rclpy's built-in
qos_profile_sensor_data (BEST_EFFORT, depth=5). The previous custom
profile with depth=1 was too aggressive for CycloneDDS and occasionally
lost the subscription match after a sim hiccup, freezing the BEV.
"""

import math, json
import numpy as np
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA, String
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
import transforms3d
from scipy.ndimage import (distance_transform_edt, label as cc_label,
                           binary_closing, binary_opening)

try:
    from skimage.morphology import medial_axis
    _HAS_SKIMAGE = True
except ImportError:
    _HAS_SKIMAGE = False


# ──────────────────────────────────────────────────────────────────────
#  Hardcoded door world coordinates (metres), 24 doors in Gazebo world.
#  Stored as a FLAT list [x0,y0, x1,y1, ...] because ROS 2 parameters
#  don't accept list-of-lists.
# ──────────────────────────────────────────────────────────────────────
DEFAULT_DOOR_XY_FLAT = [
    -7.53, -32.00,   -2.61, -32.00,   -6.57, -24.92,    5.43, -26.00,
    -6.57, -23.04,   -6.57,  -9.04,    6.43, -23.08,   -2.89, -16.00,
     2.75, -16.00,   -2.89, -10.04,    2.75, -10.04,    6.43,  -9.04,
     2.75,   0.00,   -2.89,   0.00,   -7.25,  -0.52,    7.15,  -0.52,
    -9.13,   4.84,   -9.13,  11.24,   -6.85,  12.00,   -5.33,  15.00,
     8.95,   4.84,    8.95,  11.24,    6.67,  12.00,    5.07,  15.00,
]


# ──────────────────────────────────────────────────────────────────────
#  BEV: log-odds occupancy grid
# ──────────────────────────────────────────────────────────────────────
class BEV:
    L_HIT, L_MISS = 0.85, -0.40
    L_MIN, L_MAX  = -5.0, 5.0

    def __init__(self, xmin, ymin, xmax, ymax, res):
        self.xmin, self.ymin = xmin, ymin
        self.xmax, self.ymax = xmax, ymax
        self.res = res
        self.W = int(round((xmax - xmin) / res))
        self.H = int(round((ymax - ymin) / res))
        self.logodds = np.zeros((self.H, self.W), dtype=np.float32)

    def w2c(self, x, y):
        return (int((x - self.xmin) / self.res),
                int((y - self.ymin) / self.res))

    def c2w(self, cx, cy):
        return (self.xmin + (cx + 0.5) * self.res,
                self.ymin + (cy + 0.5) * self.res)

    def in_bounds(self, cx, cy):
        return 0 <= cx < self.W and 0 <= cy < self.H

    def integrate_rays(self, origin_xy, hits):
        ox, oy = self.w2c(*origin_xy)
        if not self.in_bounds(ox, oy):
            return
        for hxw, hyw in hits:
            hx, hy = self.w2c(hxw, hyw)
            N = max(abs(hx - ox), abs(hy - oy))
            if N == 0:
                continue
            for t in np.linspace(0, 1, N, endpoint=False):
                fx = int(ox + t * (hx - ox))
                fy = int(oy + t * (hy - oy))
                if self.in_bounds(fx, fy):
                    self.logodds[fy, fx] = np.clip(
                        self.logodds[fy, fx] + self.L_MISS,
                        self.L_MIN, self.L_MAX)
            if self.in_bounds(hx, hy):
                self.logodds[hy, hx] = np.clip(
                    self.logodds[hy, hx] + self.L_HIT,
                    self.L_MIN, self.L_MAX)

    @property
    def free_mask(self):   return self.logodds < -0.1
    @property
    def occ_mask(self):    return self.logodds >  0.4
    @property
    def known_mask(self):  return np.abs(self.logodds) > 1e-6


# ──────────────────────────────────────────────────────────────────────
#  Persistent room ID registry — IoU matching across ticks.
# ──────────────────────────────────────────────────────────────────────
class RoomRegistry:
    """
    A room, once seen, keeps its ID (and colour) for the lifetime of the
    node. Each new connected component is matched to the previous tick's
    room with the highest mask-IoU ≥ threshold; unmatched components get
    a fresh ID.
    """
    def __init__(self, iou_threshold: float = 0.25):
        self.iou_threshold = iou_threshold
        self.rooms = OrderedDict()   # id -> dict
        self._next_id = 0

    def update(self, masks, stats, bev):
        pairs = []
        new_sums = [int(m.sum()) for m in masks]
        for i, m in enumerate(masks):
            for pid, prev in self.rooms.items():
                pm = prev['mask']
                if pm.shape != m.shape:
                    continue
                inter = int(np.logical_and(m, pm).sum())
                if inter == 0:
                    continue
                union = new_sums[i] + int(pm.sum()) - inter
                iou = inter / max(1, union)
                if iou >= self.iou_threshold:
                    pairs.append((iou, i, pid))

        pairs.sort(reverse=True)
        i_to_id, used = {}, set()
        for _, i, pid in pairs:
            if i in i_to_id or pid in used:
                continue
            i_to_id[i] = pid
            used.add(pid)

        for i in range(len(masks)):
            if i not in i_to_id:
                i_to_id[i] = self._next_id
                self._next_id += 1

        new_reg = OrderedDict()
        for i in range(len(masks)):
            pid = i_to_id[i]
            cx, cy = stats[i]['centroid_cells']
            wx, wy = bev.c2w(cx, cy)
            new_reg[pid] = {
                'id':                 pid,
                'mask':               masks[i],
                'centroid':           (wx, wy),
                'n_cells':            stats[i]['n_cells'],
                'median_clearance_m': stats[i]['median_clearance_m'],
                'kind':               stats[i]['kind'],
            }
        self.rooms = new_reg
        return self.rooms


# ──────────────────────────────────────────────────────────────────────
#  Segmentation: cut free mask at hardcoded doors → CC → rooms.
# ──────────────────────────────────────────────────────────────────────
def _ridge_skeleton(dt_px: np.ndarray) -> np.ndarray:
    """Fallback medial axis when scikit-image isn't installed."""
    skel = np.zeros_like(dt_px, dtype=bool)
    c = dt_px[1:-1, 1:-1]
    ridge_x = (c >= dt_px[1:-1, :-2]) & (c >= dt_px[1:-1, 2:])
    ridge_y = (c >= dt_px[:-2, 1:-1]) & (c >= dt_px[2:,  1:-1])
    skel[1:-1, 1:-1] = (c > 0.5) & (ridge_x | ridge_y)
    return skel


def segment_rooms(free_mask: np.ndarray,
                  door_cells,
                  res: float,
                  door_wall_cells: int,
                  min_room_cells: int,
                  corridor_thresh_m: float):
    """
    Returns (room_masks, room_stats, skeleton_mask, dt_m).
    """
    H, W = free_mask.shape
    empty_skel = np.zeros_like(free_mask, dtype=bool)
    empty_dt   = np.zeros(free_mask.shape, dtype=np.float32)
    if not np.any(free_mask):
        return [], [], empty_skel, empty_dt

    fm = binary_closing(free_mask, iterations=1)
    fm = binary_opening(fm, iterations=1)
    if not np.any(fm):
        return [], [], empty_skel, empty_dt

    dt_px = distance_transform_edt(fm).astype(np.float32)
    dt_m  = dt_px * res

    if _HAS_SKIMAGE:
        skel = medial_axis(fm).astype(bool)
    else:
        skel = _ridge_skeleton(dt_px)

    # Paint a small wall disc at every hardcoded door — "take down the
    # door arch" so the free mask splits on either side of every doorway.
    cut = fm.copy()
    r = max(1, int(door_wall_cells))
    yy, xx = np.ogrid[-r:r+1, -r:r+1]
    disk = (xx*xx + yy*yy) <= r*r
    for dcx, dcy in door_cells:
        if not (0 <= dcx < W and 0 <= dcy < H):
            continue
        y0, y1 = max(0, dcy-r), min(H, dcy+r+1)
        x0, x1 = max(0, dcx-r), min(W, dcx+r+1)
        dy0, dy1 = y0 - (dcy-r), y1 - (dcy-r)
        dx0, dx1 = x0 - (dcx-r), x1 - (dcx-r)
        cut[y0:y1, x0:x1] &= ~disk[dy0:dy1, dx0:dx1]

    labels, n_labels = cc_label(cut)

    masks, stats = [], []
    for i in range(1, n_labels + 1):
        m = (labels == i)
        n = int(m.sum())
        if n < min_room_cells:
            continue
        ys, xs = np.where(m)
        clear = float(np.median(dt_m[m]))
        kind = 'corridor' if clear < corridor_thresh_m else 'room'
        masks.append(m)
        stats.append({
            'centroid_cells':     (float(xs.mean()), float(ys.mean())),
            'n_cells':            n,
            'median_clearance_m': clear,
            'kind':               kind,
        })
    return masks, stats, skel, dt_m


# ──────────────────────────────────────────────────────────────────────
#  ROS 2 node
# ──────────────────────────────────────────────────────────────────────
class SemanticMapperNode(Node):
    def __init__(self):
        super().__init__('semantic_mapper')

        # ── Topics / frame ──
        self.declare_parameter('depth_topic',
                               '/simple_drone/front_depth/depth/image_raw')
        self.declare_parameter('cam_info_topic',
                               '/simple_drone/front_depth/depth/camera_info')
        self.declare_parameter('pose_topic',    '/simple_drone/gt_pose')
        self.declare_parameter('world_frame',   'world')

        # Doors: flat [x0, y0, x1, y1, ...].
        self.declare_parameter('door_xy', DEFAULT_DOOR_XY_FLAT)

        # ── BEV bounds / resolution ──
        self.declare_parameter('bbox_xmin', -12.0)
        self.declare_parameter('bbox_ymin', -34.0)
        self.declare_parameter('bbox_xmax',  12.0)
        self.declare_parameter('bbox_ymax',  17.0)
        self.declare_parameter('bev_resolution', 0.15)

        # ── Depth integration ──
        self.declare_parameter('z_slab_min',    0.30)
        self.declare_parameter('z_slab_max',    1.80)
        self.declare_parameter('max_depth',     5.00)
        self.declare_parameter('depth_stride',  8)
        self.declare_parameter('depth_min_dt',  0.15)

        # ── Segmentation / rooms ──
        self.declare_parameter('door_wall_m',        0.60)
        self.declare_parameter('min_room_cells',     80)
        self.declare_parameter('corridor_thresh_m',  1.20)
        self.declare_parameter('room_iou_threshold', 0.25)
        self.declare_parameter('tick_rate',          2.0)

        # ── Camera offset body→optical ──
        self.declare_parameter('cam_offset_x', 0.2)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_offset_z', 0.0)

        # ── Viz ──
        self.declare_parameter('viz_sphere_r',   0.30)
        self.declare_parameter('viz_text_h',     0.60)
        self.declare_parameter('viz_edge_w',     0.20)
        self.declare_parameter('viz_door_r',     0.25)
        self.declare_parameter('viz_door_h',     1.40)
        self.declare_parameter('viz_fill_alpha', 0.32)
        self.declare_parameter('viz_skeleton',   True)

        g = lambda n: self.get_parameter(n).value

        # ── Resolve scalars ──
        self.res = float(g('bev_resolution'))
        self.bev = BEV(float(g('bbox_xmin')), float(g('bbox_ymin')),
                       float(g('bbox_xmax')), float(g('bbox_ymax')),
                       self.res)
        self.world_frame = str(g('world_frame'))

        self.z_slab_min   = float(g('z_slab_min'))
        self.z_slab_max   = float(g('z_slab_max'))
        self.max_depth    = float(g('max_depth'))
        self.depth_stride = int(g('depth_stride'))
        self.depth_min_dt = float(g('depth_min_dt'))

        self.door_wall_cells = max(
            1, int(round(float(g('door_wall_m')) / self.res)))
        self.min_room_cells    = int(g('min_room_cells'))
        self.corridor_thresh_m = float(g('corridor_thresh_m'))
        self.tick_rate         = float(g('tick_rate'))
        self.viz_skeleton      = bool(g('viz_skeleton'))

        cx_, cy_, cz_ = (float(g('cam_offset_x')),
                         float(g('cam_offset_y')),
                         float(g('cam_offset_z')))
        self.T_b_c = np.array([
            [ 0.0,  0.0, 1.0, cx_],
            [-1.0,  0.0, 0.0, cy_],
            [ 0.0, -1.0, 0.0, cz_],
            [ 0.0,  0.0, 0.0, 1.0],
        ])

        self.viz_sphere_r   = float(g('viz_sphere_r'))
        self.viz_text_h     = float(g('viz_text_h'))
        self.viz_edge_w     = float(g('viz_edge_w'))
        self.viz_door_r     = float(g('viz_door_r'))
        self.viz_door_h     = float(g('viz_door_h'))
        self.viz_fill_alpha = float(g('viz_fill_alpha'))

        # ── Doors: parse flat list, precompute cell coordinates ──
        flat = [float(v) for v in g('door_xy')]
        if len(flat) % 2 != 0:
            raise ValueError(
                f"door_xy must have an even number of entries "
                f"(got {len(flat)}); it is a flat [x0,y0,x1,y1,...] list.")
        self.doors = []
        for i in range(len(flat) // 2):
            wx, wy = flat[2*i], flat[2*i + 1]
            cx, cy = self.bev.w2c(wx, wy)
            self.doors.append({
                'id':    i,
                'xy':    (wx, wy),
                'cell':  (cx, cy),
                'rooms': [],
            })
        self.get_logger().info(
            f"loaded {len(self.doors)} hardcoded doors "
            f"(x∈[{min(d['xy'][0] for d in self.doors):.1f},"
            f"{max(d['xy'][0] for d in self.doors):.1f}] "
            f"y∈[{min(d['xy'][1] for d in self.doors):.1f},"
            f"{max(d['xy'][1] for d in self.doors):.1f}])")

        # ── State ──
        self.bridge = CvBridge()
        self.cur_pose = None
        self.K = None
        self.last_depth_t = None

        self.room_registry = RoomRegistry(
            iou_threshold=float(g('room_iou_threshold')))
        self.last_skeleton = None
        self.last_dt_m     = None

        self._hb_pose  = 0
        self._hb_depth = 0
        self._hb_tick  = 0

        # ── ROS glue ──
        # Pose is RELIABLE by default (depth=10).
        # Depth + camera_info: use rclpy's built-in sensor-data profile
        # (BEST_EFFORT, depth=5). The previous custom profile with
        # depth=1 was too aggressive under CycloneDDS and could lose the
        # subscription match after a sim hiccup — heartbeat showed
        # depth=0 forever afterwards.
        # BEV + scene-graph stay TRANSIENT_LOCAL so late-joining RViz
        # gets the last map.
        latched_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Pose,  str(g('pose_topic')),
                                 self._pose_cb, 10)
        self.create_subscription(Image, str(g('depth_topic')),
                                 self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(g('cam_info_topic')),
                                 self._cam_info_cb, qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/scene_graph/markers', 1)
        self.bev_pub = self.create_publisher(
            OccupancyGrid, '/scene_graph/bev', latched_qos)
        self.sg_pub = self.create_publisher(String, '/scene_graph', latched_qos)

        self.create_timer(1.0 / self.tick_rate, self._tick)
        self.create_timer(5.0, self._heartbeat)

        if not _HAS_SKIMAGE:
            self.get_logger().warning(
                "scikit-image not found — falling back to a DT-ridge "
                "skeleton. `pip install scikit-image` for a proper "
                "medial axis.")
        self.get_logger().info(
            "semantic_mapper (MORE-style, hardcoded doors) ready")

    # ── Callbacks ─────────────────────────────────────────────────────
    def _pose_cb(self, msg):
        self.cur_pose = msg
        self._hb_pose += 1

    def _cam_info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f"cam intrinsics: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} "
                f"cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}")

    def _depth_cb(self, msg: Image):
        self._hb_depth += 1
        now = self.get_clock().now()
        if self.last_depth_t is not None:
            dt = (now - self.last_depth_t).nanoseconds * 1e-9
            if dt < self.depth_min_dt:
                return
        self.last_depth_t = now
        if self.cur_pose is None or self.K is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception:
            return
        self._integrate_depth(depth)

    # ── Geometry ──────────────────────────────────────────────────────
    def _T_world_cam(self):
        p = self.cur_pose
        q = [p.orientation.w, p.orientation.x,
             p.orientation.y, p.orientation.z]
        T = np.eye(4)
        T[:3, :3] = transforms3d.quaternions.quat2mat(q)
        T[:3, 3]  = [p.position.x, p.position.y, p.position.z]
        return T @ self.T_b_c

    def _integrate_depth(self, depth):
        H, W = depth.shape
        s = self.depth_stride
        us = np.arange(0, W, s); vs = np.arange(0, H, s)
        uu, vv = np.meshgrid(us, vs)
        d = depth[::s, ::s]
        valid = np.isfinite(d) & (d > 0.1) & (d < self.max_depth)
        if not np.any(valid):
            return
        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        zz = d[valid].astype(np.float32)
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        Xc = (uu - cx) * zz / fx
        Yc = (vv - cy) * zz / fy
        pts_c = np.stack([Xc, Yc, zz, np.ones_like(zz)], axis=0)
        pts_w = (self._T_world_cam() @ pts_c)[:3].T
        in_slab = ((pts_w[:, 2] >= self.z_slab_min) &
                   (pts_w[:, 2] <= self.z_slab_max))
        pts_w = pts_w[in_slab]
        if pts_w.shape[0] == 0:
            return
        p = self.cur_pose.position
        if pts_w.shape[0] > 1500:
            pts_w = pts_w[np.random.choice(pts_w.shape[0], 1500, replace=False)]
        self.bev.integrate_rays(
            (p.x, p.y),
            [(float(x), float(y)) for x, y, _ in pts_w])

    # ── Segmentation + registry + door↔room linking ───────────────────
    def _segment(self):
        door_cells = [d['cell'] for d in self.doors]
        masks, stats, skel, dt_m = segment_rooms(
            self.bev.free_mask,
            door_cells,
            self.res,
            door_wall_cells=self.door_wall_cells,
            min_room_cells=self.min_room_cells,
            corridor_thresh_m=self.corridor_thresh_m,
        )
        rooms = self.room_registry.update(masks, stats, self.bev)

        H, W = self.bev.free_mask.shape
        r  = self.door_wall_cells
        R2 = r + 3
        R1_sq, R2_sq = r * r, R2 * R2
        for door in self.doors:
            dcx, dcy = door['cell']
            if not (0 <= dcx < W and 0 <= dcy < H):
                door['rooms'] = []
                continue
            y0, y1 = max(0, dcy-R2), min(H, dcy+R2+1)
            x0, x1 = max(0, dcx-R2), min(W, dcx+R2+1)
            touched = set()
            for yy2 in range(y0, y1):
                for xx2 in range(x0, x1):
                    dd = (yy2-dcy)**2 + (xx2-dcx)**2
                    if R1_sq < dd <= R2_sq:
                        for rid, room in rooms.items():
                            if room['mask'][yy2, xx2]:
                                touched.add(rid)
                                break
            door['rooms'] = sorted(touched)

        self.last_skeleton = skel
        self.last_dt_m     = dt_m

    # ── Publishing ────────────────────────────────────────────────────
    def _publish_bev(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.info.resolution = self.res
        msg.info.width  = self.bev.W
        msg.info.height = self.bev.H
        msg.info.origin.position.x = self.bev.xmin
        msg.info.origin.position.y = self.bev.ymin
        msg.info.origin.orientation.w = 1.0
        occ = np.full(self.bev.logodds.shape, -1, dtype=np.int8)
        occ[self.bev.free_mask] = 0
        occ[self.bev.occ_mask]  = 100
        msg.data = occ.flatten().tolist()
        self.bev_pub.publish(msg)

    def _room_color(self, i):
        import colorsys
        h = (i * 0.6180339887) % 1.0
        return colorsys.hsv_to_rgb(h, 0.85, 0.95)

    def _publish_markers(self):
        arr = MarkerArray()
        mid = 0
        stamp = self.get_clock().now().to_msg()
        arr.markers.append(Marker(action=Marker.DELETEALL))

        # Room fills — colour from persistent ID.
        for room in self.room_registry.rooms.values():
            r, g, b = self._room_color(room['id'])
            m = Marker(); m.header.frame_id = self.world_frame
            m.header.stamp = stamp
            m.ns = 'room_fill'; m.id = mid; mid += 1
            m.type = Marker.CUBE_LIST; m.action = Marker.ADD
            m.scale.x = self.res; m.scale.y = self.res; m.scale.z = 0.02
            m.pose.orientation.w = 1.0
            ys, xs = np.where(room['mask'])
            if len(xs) > 6000:
                idx = np.random.choice(len(xs), 6000, replace=False)
                xs, ys = xs[idx], ys[idx]
            for cx, cy in zip(xs, ys):
                wx, wy = self.bev.c2w(int(cx), int(cy))
                m.points.append(Point(x=wx, y=wy, z=0.05))
            m.color = ColorRGBA(r=r, g=g, b=b, a=self.viz_fill_alpha)
            arr.markers.append(m)

        # Centroid sphere + label.
        for room in self.room_registry.rooms.values():
            r, g, b = self._room_color(room['id'])
            s = Marker(); s.header.frame_id = self.world_frame
            s.header.stamp = stamp
            s.ns = 'room_centroids'; s.id = mid; mid += 1
            s.type = Marker.SPHERE; s.action = Marker.ADD
            cxw, cyw = room['centroid']
            s.pose.position = Point(x=cxw, y=cyw, z=2.0)
            s.pose.orientation.w = 1.0
            d = self.viz_sphere_r * 2.0
            s.scale.x = s.scale.y = s.scale.z = d
            s.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            arr.markers.append(s)

            t = Marker(); t.header.frame_id = self.world_frame
            t.header.stamp = stamp
            t.ns = 'room_labels'; t.id = mid; mid += 1
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position = Point(x=cxw, y=cyw, z=3.0)
            t.pose.orientation.w = 1.0
            t.scale.z = self.viz_text_h
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            tag = 'C' if room['kind'] == 'corridor' else 'R'
            t.text = (f"{tag}{room['id']}  ({room['n_cells']} cells, "
                      f"{room['median_clearance_m']:.1f} m)")
            arr.markers.append(t)

        # Doors.
        for door in self.doors:
            dx, dy = door['xy']
            m = Marker(); m.header.frame_id = self.world_frame
            m.header.stamp = stamp
            m.ns = 'doors'; m.id = mid; mid += 1
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position = Point(x=dx, y=dy, z=self.viz_door_h * 0.5)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = self.viz_door_r * 2.0
            m.scale.z = self.viz_door_h
            m.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.95)
            arr.markers.append(m)

            t = Marker(); t.header.frame_id = self.world_frame
            t.header.stamp = stamp
            t.ns = 'door_labels'; t.id = mid; mid += 1
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position = Point(x=dx, y=dy, z=self.viz_door_h + 0.4)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.35
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            t.text = f"d{door['id']}"
            arr.markers.append(t)

        # Voronoi skeleton — MORE-style spine.
        if self.viz_skeleton and self.last_skeleton is not None \
           and np.any(self.last_skeleton):
            ys, xs = np.where(self.last_skeleton)
            if len(xs) > 4000:
                idx = np.random.choice(len(xs), 4000, replace=False)
                xs, ys = xs[idx], ys[idx]
            m = Marker(); m.header.frame_id = self.world_frame
            m.header.stamp = stamp
            m.ns = 'skeleton'; m.id = mid; mid += 1
            m.type = Marker.CUBE_LIST; m.action = Marker.ADD
            m.scale.x = m.scale.y = self.res * 0.8
            m.scale.z = 0.05
            m.pose.orientation.w = 1.0
            for cx, cy in zip(xs, ys):
                wx, wy = self.bev.c2w(int(cx), int(cy))
                m.points.append(Point(x=wx, y=wy, z=0.25))
            m.color = ColorRGBA(r=0.25, g=0.6, b=1.0, a=0.9)
            arr.markers.append(m)

        # Topological edges: room → door → room.
        pos = {r['id']: r['centroid']
               for r in self.room_registry.rooms.values()}
        for door in self.doors:
            if len(door['rooms']) < 2:
                continue
            for i in range(len(door['rooms'])):
                for j in range(i + 1, len(door['rooms'])):
                    a, b = door['rooms'][i], door['rooms'][j]
                    if a not in pos or b not in pos:
                        continue
                    m = Marker(); m.header.frame_id = self.world_frame
                    m.header.stamp = stamp
                    m.ns = 'room_room_edges'; m.id = mid; mid += 1
                    m.type = Marker.LINE_LIST; m.action = Marker.ADD
                    m.scale.x = self.viz_edge_w
                    m.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=1.0)
                    ax, ay = pos[a]; bx, by = pos[b]
                    dx, dy = door['xy']
                    for p in [(ax, ay), (dx, dy), (dx, dy), (bx, by)]:
                        m.points.append(Point(x=p[0], y=p[1], z=2.5))
                    m.pose.orientation.w = 1.0
                    arr.markers.append(m)

        self.marker_pub.publish(arr)

    def _publish_scene_graph(self):
        sg = {
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
            'rooms': [{'id':                 r['id'],
                       'centroid':           list(r['centroid']),
                       'n_cells':            r['n_cells'],
                       'median_clearance_m': r['median_clearance_m'],
                       'kind':               r['kind']}
                      for r in self.room_registry.rooms.values()],
            'doors': [{'id':    d['id'],
                       'xy':    list(d['xy']),
                       'rooms': d['rooms']}
                      for d in self.doors],
            'edges': [],
        }
        seen = set()
        for d in self.doors:
            if len(d['rooms']) < 2:
                continue
            for i in range(len(d['rooms'])):
                for j in range(i + 1, len(d['rooms'])):
                    a, b = sorted([d['rooms'][i], d['rooms'][j]])
                    k = (a, b)
                    if k in seen:
                        continue
                    seen.add(k)
                    sg['edges'].append({'a': a, 'b': b,
                                        'via_door': d['id']})
        self.sg_pub.publish(String(data=json.dumps(sg)))

    # ── Diagnostics ───────────────────────────────────────────────────
    def _heartbeat(self):
        free = int(np.count_nonzero(self.bev.free_mask))
        occ  = int(np.count_nonzero(self.bev.occ_mask))
        depth_silent = (self._hb_depth == 0)
        msg = (
            f"hb: pose={self._hb_pose} depth={self._hb_depth} "
            f"tick={self._hb_tick} "
            f"K={'ok' if self.K is not None else 'MISSING'} "
            f"BEV free/occ = {free}/{occ} "
            f"rooms={len(self.room_registry.rooms)}"
            + ("  ⚠ DEPTH STREAM SILENT" if depth_silent else ""))
        if depth_silent:
            self.get_logger().warning(msg)
        else:
            self.get_logger().info(msg)
        self._hb_pose = self._hb_depth = self._hb_tick = 0

    # ── Main tick ─────────────────────────────────────────────────────
    def _tick(self):
        self._hb_tick += 1
        if self.cur_pose is None:
            return

        # BEV first, ALWAYS. Independent of segmentation.
        try:
            self._publish_bev()
        except Exception as e:
            self.get_logger().error(f"_publish_bev failed: {e}")

        try:
            self._segment()
            self._publish_markers()
            self._publish_scene_graph()
        except Exception as e:
            self.get_logger().error(
                f"segmentation pipeline failed: {e}", exc_info=True)


def main():
    rclpy.init()
    node = SemanticMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()