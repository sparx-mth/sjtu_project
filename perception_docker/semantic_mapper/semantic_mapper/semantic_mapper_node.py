#!/usr/bin/env python3
"""
semantic_mapper_node.py — FALCON BEV → Voronoi → rooms → scene graph.

Subscribes to a single nav_msgs/OccupancyGrid published by the
FALCON-side `bev_publisher` node (value encoding: -1 unknown, 0 free,
>=50 occupied). Every tick:

    OccupancyGrid  ─► medial_axis Voronoi
                     ─► punch disk at each DISCOVERED door
                     ─► 8-connected CC
                     ─► nearest-skeleton paint
                     ─► rooms (IoU-matched across ticks → stable IDs)
                     ─► scene graph (rooms, doors, edges) as JSON

No PointCloud2, no z-slab, no BEV rebuild, no bbox parameters — the
grid header already says where the map is and how big it is. The
FALCON container handles all 3D-to-2D conversion.
"""

import json
import colorsys
from collections import OrderedDict

import numpy as np
from scipy.ndimage import (binary_closing, binary_opening,
                           distance_transform_edt, label as cc_label)

try:
    from skimage.morphology import medial_axis
    _HAS_SKIMAGE = True
except ImportError:
    _HAS_SKIMAGE = False

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


# ──────────────────────────────────────────────────────────────────────
#  Hardcoded door positions (world coords, metres) — 24 in the hospital.
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
#  Voronoi → cut at doors → rooms.  Same algorithm as before; it's
#  agnostic about where the free mask came from.
# ──────────────────────────────────────────────────────────────────────
def compute_rooms(free_mask, door_cells, door_cut_cells, min_room_cells):
    """
    Returns
    -------
    room_lbl : (H,W) int32        0 = not-a-room, 1..N = rooms (fresh labels)
    skeleton : (H,W) bool         Voronoi spine AFTER cutting at discovered doors
    stats    : list[dict]         one per room in label order
    """
    H, W = free_mask.shape
    empty_lbl = np.zeros((H, W), np.int32)
    empty_sk  = np.zeros((H, W), bool)
    if not free_mask.any():
        return empty_lbl, empty_sk, []

    # Heal pinhole noise so the skeleton doesn't fork unnecessarily.
    fm = binary_closing(free_mask, iterations=1)
    fm = binary_opening(fm, iterations=1)
    if not fm.any():
        return empty_lbl, empty_sk, []

    # 1. Medial-axis Voronoi skeleton of free space.
    skel = (medial_axis(fm).astype(bool) if _HAS_SKIMAGE
            else _ridge_fallback(fm))

    # 2. Punch a disk through the skeleton at every discovered door.
    r = max(1, int(door_cut_cells))
    yy, xx = np.ogrid[-r:r+1, -r:r+1]
    disk = (xx*xx + yy*yy) <= r*r
    sk_cut = skel.copy()
    for dcx, dcy in door_cells:
        if not (0 <= dcx < W and 0 <= dcy < H):
            continue
        y0, y1 = max(0, dcy-r), min(H, dcy+r+1)
        x0, x1 = max(0, dcx-r), min(W, dcx+r+1)
        dy0, dx0 = y0 - (dcy-r), x0 - (dcx-r)
        sk_cut[y0:y1, x0:x1] &= ~disk[dy0:dy0+(y1-y0), dx0:dx0+(x1-x0)]

    # 3. 8-connected CC on the cut skeleton. (4-conn would fragment the
    #    diagonal medial-axis ridges into hundreds of bogus components.)
    sk_lbl, n_lbl = cc_label(sk_cut, structure=np.ones((3, 3), np.uint8))
    if n_lbl == 0:
        return empty_lbl, sk_cut, []

    # 4. Paint each free cell with the label of its nearest skeleton
    #    pixel. That's the Voronoi diagram of the cut skeleton clipped
    #    to free space.
    _, (iy, ix) = distance_transform_edt(~sk_cut, return_indices=True)
    room_lbl = np.where(fm, sk_lbl[iy, ix], 0).astype(np.int32)

    # 5. Drop rooms smaller than the threshold; remap surviving IDs to
    #    a contiguous 1..N for downstream.
    out = np.zeros_like(room_lbl)
    stats = []
    for k in range(1, n_lbl + 1):
        m = (room_lbl == k)
        n = int(m.sum())
        if n < min_room_cells:
            continue
        nid = len(stats) + 1
        out[m] = nid
        ys, xs = np.where(m)
        stats.append({
            'id': nid, 'mask': m, 'n_cells': n,
            'centroid_cells': (float(xs.mean()), float(ys.mean())),
        })

    keep_sk = np.zeros_like(sk_cut)
    for s in stats:
        keep_sk |= sk_cut & s['mask']
    return out, keep_sk, stats


def _ridge_fallback(fm):
    """DT-ridge skeleton fallback when scikit-image isn't available."""
    dt = distance_transform_edt(fm)
    sk = np.zeros_like(fm, bool)
    c = dt[1:-1, 1:-1]
    rx = (c >= dt[1:-1, :-2]) & (c >= dt[1:-1, 2:])
    ry = (c >= dt[:-2, 1:-1]) & (c >= dt[2:,  1:-1])
    sk[1:-1, 1:-1] = (c > 0.5) & (rx | ry)
    return sk


# ──────────────────────────────────────────────────────────────────────
#  Persistent IoU-based room ID registry.
# ──────────────────────────────────────────────────────────────────────
class RoomRegistry:
    def __init__(self, iou_threshold=0.25):
        self.iou = iou_threshold
        self.rooms = OrderedDict()
        self._next = 0

    def update(self, stats, c2w):
        pairs = []
        for i, s in enumerate(stats):
            for pid, prev in self.rooms.items():
                if prev['mask'].shape != s['mask'].shape:
                    continue
                inter = int(np.logical_and(s['mask'], prev['mask']).sum())
                if inter == 0:
                    continue
                union = s['n_cells'] + int(prev['mask'].sum()) - inter
                iou = inter / max(1, union)
                if iou >= self.iou:
                    pairs.append((iou, i, pid))

        pairs.sort(reverse=True)
        i2id, used = {}, set()
        for _, i, pid in pairs:
            if i in i2id or pid in used:
                continue
            i2id[i] = pid
            used.add(pid)
        for i in range(len(stats)):
            if i not in i2id:
                i2id[i] = self._next
                self._next += 1

        new = OrderedDict()
        for i, s in enumerate(stats):
            sid = i2id[i]
            wx, wy = c2w(*s['centroid_cells'])
            new[sid] = {'id': sid, 'mask': s['mask'],
                        'n_cells': s['n_cells'], 'centroid': (wx, wy)}
        self.rooms = new
        return self.rooms


# ──────────────────────────────────────────────────────────────────────
#  The node.
# ──────────────────────────────────────────────────────────────────────
class SemanticMapperNode(Node):
    # OccupancyGrid value semantics (nav_msgs standard)
    UNK, FREE_MAX, OCC_MIN = -1, 49, 50

    def __init__(self):
        super().__init__('semantic_mapper')

        # ── Parameters ──
        P = self.declare_parameter
        P('bev_topic',   '/falcon/bev_2d')
        P('world_frame', 'world')
        P('door_xy',     DEFAULT_DOOR_XY_FLAT)

        P('door_cut_m',          0.60)
        P('door_match_radius_m', 0.90)
        P('door_discover_m',     0.30)
        # Rooms smaller than this many cells are discarded. At 0.15 m
        # resolution, 40 cells ≈ 0.9 m² — small storage closets qualify.
        P('min_room_cells',      40)
        # IoU threshold for matching a fresh room to a previously-seen
        # one. Lower = more tolerant to mask drift as the drone explores
        # and the room's shape grows / reshapes slightly each tick.
        P('room_iou_threshold',  0.15)
        P('tick_rate',           2.0)

        P('viz_fill_alpha', 0.32)
        P('viz_door_r',     0.25)
        P('viz_door_h',     1.40)
        P('viz_text_h',     0.55)
        P('viz_edge_w',     0.18)
        P('viz_cut_disk',   True)

        g = lambda n: self.get_parameter(n).value
        self.world_frame = str(g('world_frame'))
        self.door_cut_m      = float(g('door_cut_m'))
        self.door_match_r_m  = float(g('door_match_radius_m'))
        self.door_discover_m = float(g('door_discover_m'))
        self.min_room_cells  = int(g('min_room_cells'))
        self.tick_rate       = float(g('tick_rate'))
        self.viz = {k: g('viz_' + k) for k in
                    ('fill_alpha', 'door_r', 'door_h', 'text_h',
                     'edge_w', 'cut_disk')}

        # Doors: world-frame XY only; cell coords are derived once we
        # see the first OccupancyGrid and learn its origin/resolution.
        flat = [float(v) for v in g('door_xy')]
        if len(flat) % 2:
            raise ValueError("door_xy must have an even length.")
        self.doors = []
        for i in range(len(flat) // 2):
            wx, wy = flat[2*i], flat[2*i + 1]
            self.doors.append({'id': i, 'xy': (wx, wy),
                               'cell': None, 'rooms': [],
                               'discovered': False})

        self.registry = RoomRegistry(float(g('room_iou_threshold')))

        # ── State (populated on first OccupancyGrid) ──
        self.grid = None     # (H, W) int8, -1 unknown / 0 free / 100 occ
        self.res  = None
        self.xmin = None
        self.ymin = None
        self._skel = None

        # ── ROS glue ──
        # Our outbound /scene_graph uses TRANSIENT_LOCAL so late-joining
        # subscribers get the last scene graph. But the inbound
        # /falcon/bev_2d comes through ros1_bridge, which maps a ROS 1
        # latched publisher to ROS 2 VOLATILE. Asking for TRANSIENT_LOCAL
        # on the subscriber yields an "incompatible QoS: DURABILITY"
        # warning and no messages arrive.
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        bev_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(OccupancyGrid, str(g('bev_topic')),
                                 self._grid_cb, bev_qos)

        self.pub_markers = self.create_publisher(
            MarkerArray, '/scene_graph/markers', 1)
        self.pub_sg = self.create_publisher(String, '/scene_graph', latched)

        self.create_timer(1.0 / self.tick_rate, self._tick)
        self.create_timer(5.0, self._heartbeat)
        self._hb = dict(grid=0, tick=0)

        self.get_logger().info(
            f"semantic_mapper ready. Subscribed to {g('bev_topic')} "
            f"(nav_msgs/OccupancyGrid). {len(self.doors)} doors loaded.")

    # ── Geometry helpers (grid-dependent) ──
    def w2c(self, x, y):
        return (int((x - self.xmin) / self.res),
                int((y - self.ymin) / self.res))

    def c2w(self, cx, cy):
        return (self.xmin + (cx + 0.5) * self.res,
                self.ymin + (cy + 0.5) * self.res)

    @property
    def door_cut_cells(self):
        return max(1, int(round(self.door_cut_m / self.res)))

    @property
    def door_discover_cells(self):
        return max(1, int(round(self.door_discover_m / self.res)))

    @property
    def door_match_cells(self):
        return max(self.door_cut_cells + 2,
                   int(round(self.door_match_r_m / self.res)))

    # ── OccupancyGrid callback — the only data input ──
    def _grid_cb(self, msg: OccupancyGrid):
        W, H = msg.info.width, msg.info.height
        new_res  = float(msg.info.resolution)
        new_xmin = float(msg.info.origin.position.x)
        new_ymin = float(msg.info.origin.position.y)

        # Detect geometry change (first message, or upstream reconfig).
        geom_changed = (self.res is None
                        or abs(new_res - self.res) > 1e-6
                        or abs(new_xmin - self.xmin) > 1e-6
                        or abs(new_ymin - self.ymin) > 1e-6
                        or self.grid is None
                        or self.grid.shape != (H, W))

        self.grid = np.asarray(msg.data, dtype=np.int8).reshape(H, W)

        if geom_changed:
            self.res, self.xmin, self.ymin = new_res, new_xmin, new_ymin
            # (Re)compute each door's cell coords in this grid.
            for d in self.doors:
                d['cell'] = self.w2c(*d['xy'])
            # Shape changed → IoU matching across ticks no longer works,
            # reset the registry so room IDs restart cleanly.
            self.registry = RoomRegistry(self.registry.iou)
            self.get_logger().info(
                f"grid geometry: {W}×{H} @ {self.res:.3f} m, "
                f"origin ({self.xmin:.2f}, {self.ymin:.2f}); "
                f"door_cut={self.door_cut_cells}c, "
                f"discover={self.door_discover_cells}c")

        self._hb['grid'] += 1

    # ── Door discovery (FALCON has seen the doorway?) ──
    def _update_discovered(self):
        H, W = self.grid.shape
        r = self.door_discover_cells
        for d in self.doors:
            if d['discovered']:
                continue
            dcx, dcy = d['cell']
            if not (0 <= dcx < W and 0 <= dcy < H):
                continue
            y0, y1 = max(0, dcy-r), min(H, dcy+r+1)
            x0, x1 = max(0, dcx-r), min(W, dcx+r+1)
            patch = self.grid[y0:y1, x0:x1]
            if (patch != self.UNK).any():
                d['discovered'] = True
                self.get_logger().info(
                    f"door d{d['id']} discovered at "
                    f"({d['xy'][0]:.2f}, {d['xy'][1]:.2f})")

    # ── Door ↔ room association (annulus just outside the cut disk) ──
    def _link_doors(self, rooms):
        H, W = self.grid.shape
        r_in  = self.door_cut_cells
        r_out = self.door_match_cells
        for d in self.doors:
            d['rooms'] = []
            if not d['discovered']:
                continue
            dcx, dcy = d['cell']
            if not (0 <= dcx < W and 0 <= dcy < H):
                continue
            y0, y1 = max(0, dcy-r_out), min(H, dcy+r_out+1)
            x0, x1 = max(0, dcx-r_out), min(W, dcx+r_out+1)
            ys = np.arange(y0, y1) - dcy
            xs = np.arange(x0, x1) - dcx
            dd = ys[:, None]**2 + xs[None, :]**2
            annulus = (dd > r_in*r_in) & (dd <= r_out*r_out)
            touched = sorted({rid for rid, r in rooms.items()
                              if (r['mask'][y0:y1, x0:x1] & annulus).any()})
            d['rooms'] = touched

    # ── Main tick ──
    def _tick(self):
        self._hb['tick'] += 1
        if self.grid is None:
            return

        free_mask = (self.grid >= 0) & (self.grid <= self.FREE_MAX)

        self._update_discovered()

        cut_cells = [d['cell'] for d in self.doors if d['discovered']]
        _, self._skel, stats = compute_rooms(
            free_mask, cut_cells, self.door_cut_cells, self.min_room_cells)
        rooms = self.registry.update(stats, self.c2w)
        self._link_doors(rooms)

        self._publish_markers(rooms)
        self._publish_scene_graph(rooms)

    # ── Visualization ──
    @staticmethod
    def _rgb(i):
        h = (i * 0.6180339887) % 1.0
        return colorsys.hsv_to_rgb(h, 0.85, 0.95)

    def _publish_markers(self, rooms):
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        stamp = self.get_clock().now().to_msg()
        mid = [0]

        def mk(ns, typ, scale, color, alpha=1.0):
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = stamp
            m.ns, m.id = ns, mid[0]; mid[0] += 1
            m.type, m.action = typ, Marker.ADD
            m.pose.orientation.w = 1.0
            if isinstance(scale, (tuple, list)):
                m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            else:
                m.scale.x = m.scale.y = m.scale.z = float(scale)
            r, g, b = color
            m.color = ColorRGBA(r=float(r), g=float(g),
                                b=float(b), a=float(alpha))
            return m

        def points_from_mask(mask, z, limit):
            ys, xs = np.where(mask)
            if len(xs) > limit:
                k = np.random.choice(len(xs), limit, replace=False)
                xs, ys = xs[k], ys[k]
            return [Point(x=self.c2w(int(cx), int(cy))[0],
                          y=self.c2w(int(cx), int(cy))[1], z=z)
                    for cx, cy in zip(xs, ys)]

        # 1. Room fills — one colour per room.
        for room in rooms.values():
            col = self._rgb(room['id'])
            m = mk('room_fill', Marker.CUBE_LIST,
                   (self.res, self.res, 0.02),
                   col, self.viz['fill_alpha'])
            m.points = points_from_mask(room['mask'], 0.05, 6000)
            arr.markers.append(m)

        # 2. Voronoi skeleton — SAME colour as its room.
        for room in rooms.values():
            if self._skel is None:
                continue
            sk_in_room = self._skel & room['mask']
            if not sk_in_room.any():
                continue
            col = self._rgb(room['id'])
            m = mk('skeleton', Marker.CUBE_LIST,
                   (self.res * 1.2, self.res * 1.2, 0.05),
                   col, 1.0)
            m.points = points_from_mask(sk_in_room, 0.25, 1500)
            arr.markers.append(m)

        # 3. Room centroids + labels.
        for room in rooms.values():
            col = self._rgb(room['id'])
            s = mk('rooms', Marker.SPHERE, 0.6, col)
            s.pose.position = Point(x=room['centroid'][0],
                                    y=room['centroid'][1], z=2.0)
            arr.markers.append(s)
            t = mk('room_labels', Marker.TEXT_VIEW_FACING,
                   (0, 0, float(self.viz['text_h'])), (1, 1, 1))
            t.pose.position = Point(x=room['centroid'][0],
                                    y=room['centroid'][1], z=3.0)
            t.text = f"R{room['id']}  ({room['n_cells']})"
            arr.markers.append(t)

        # 4. Doors — bright if discovered, grey ghost if not.
        for d in self.doors:
            dx, dy = d['xy']
            if d['discovered']:
                ns_c, ns_l = 'doors_discovered', 'door_labels'
                col_c, a_c = (1.0, 0.55, 0.0), 0.95
                col_l, a_l = (1.0, 1.0, 1.0), 1.0
            else:
                ns_c, ns_l = 'doors_pending', 'door_labels_pending'
                col_c, a_c = (0.55, 0.58, 0.65), 0.35
                col_l, a_l = (0.75, 0.78, 0.82), 0.55
            cyl = mk(ns_c, Marker.CYLINDER,
                     (float(self.viz['door_r']) * 2,
                      float(self.viz['door_r']) * 2,
                      float(self.viz['door_h'])), col_c, a_c)
            cyl.pose.position = Point(x=dx, y=dy,
                                      z=float(self.viz['door_h']) * 0.5)
            arr.markers.append(cyl)
            lbl = mk(ns_l, Marker.TEXT_VIEW_FACING,
                     (0, 0, 0.35), col_l, a_l)
            lbl.pose.position = Point(x=dx, y=dy,
                                      z=float(self.viz['door_h']) + 0.35)
            lbl.text = f"d{d['id']}"
            arr.markers.append(lbl)
            if d['discovered'] and self.viz['cut_disk']:
                disk = mk('cut_disks', Marker.CYLINDER,
                          (self.door_cut_cells * self.res * 2,
                           self.door_cut_cells * self.res * 2, 0.03),
                          (0.7, 0.05, 0.05), 0.55)
                disk.pose.position = Point(x=dx, y=dy, z=0.15)
                arr.markers.append(disk)

        # 5. Topological edges room → door → room.
        pos = {r['id']: r['centroid'] for r in rooms.values()}
        for d in self.doors:
            if not d['discovered']:
                continue
            rs = [r for r in d['rooms'] if r in pos]
            for i in range(len(rs)):
                for j in range(i + 1, len(rs)):
                    m = mk('room_room_edges', Marker.LINE_LIST,
                           (float(self.viz['edge_w']), 0, 0),
                           (1.0, 0.85, 0.0))
                    ax, ay = pos[rs[i]]; bx, by = pos[rs[j]]
                    dx, dy = d['xy']
                    for p in [(ax, ay), (dx, dy), (dx, dy), (bx, by)]:
                        m.points.append(Point(x=p[0], y=p[1], z=2.5))
                    arr.markers.append(m)

        self.pub_markers.publish(arr)

    def _publish_scene_graph(self, rooms):
        sg = {
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
            'rooms': [{'id': r['id'],
                       'centroid': list(r['centroid']),
                       'n_cells': r['n_cells']}
                      for r in rooms.values()],
            'doors': [{'id': d['id'],
                       'xy': list(d['xy']),
                       'rooms': d['rooms'],
                       'discovered': d['discovered']}
                      for d in self.doors],
            'edges': [],
        }
        seen = set()
        for d in self.doors:
            if not d['discovered']:
                continue
            for i in range(len(d['rooms'])):
                for j in range(i + 1, len(d['rooms'])):
                    a, b = sorted([d['rooms'][i], d['rooms'][j]])
                    if (a, b) in seen:
                        continue
                    seen.add((a, b))
                    sg['edges'].append({'a': a, 'b': b, 'via_door': d['id']})
        self.pub_sg.publish(String(data=json.dumps(sg)))

    def _heartbeat(self):
        hb = self._hb
        n_disc = sum(1 for d in self.doors if d['discovered'])
        if self.grid is None:
            self.get_logger().warning(
                f"hb  tick={hb['tick']} grid=0  "
                f"(no OccupancyGrid yet — is FALCON bev_publisher running?)")
        else:
            free = int(((self.grid >= 0) & (self.grid <= self.FREE_MAX)).sum())
            occ  = int((self.grid >= self.OCC_MIN).sum())
            self.get_logger().info(
                f"hb  grid={hb['grid']} tick={hb['tick']}  "
                f"free={free} occ={occ}  "
                f"rooms={len(self.registry.rooms)}  "
                f"doors={n_disc}/{len(self.doors)}")
        self._hb = dict(grid=0, tick=0)


def main():
    rclpy.init()
    node = SemanticMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Humble's signal handler may have already shut down the
        # context. Guard to avoid a noisy double-shutdown traceback.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()