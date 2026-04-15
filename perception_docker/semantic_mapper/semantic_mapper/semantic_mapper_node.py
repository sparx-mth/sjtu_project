#!/usr/bin/env python3
"""
semantic_mapper_node.py  —  MORE-style hierarchical scene graph (ROS2 Humble).

This is a ground-up rewrite of the room-segmentation logic. The previous
version cut the free-space pixel mask with axis-aligned squares around door
detections and ran connected-components labeling on the result. That is much
cruder than what MORE actually does and tends to either fracture rooms (when
door_radius is too small) or merge them (when too large).

What this file does instead, following MORE (Mohammadi et al., 2025,
arXiv:2505.03035, §III-B + supplementary §S.3-S.4):

  1. Build a 2D log-odds occupancy BEV from depth + pose (unchanged).
  2. From the free mask, extract a navigation skeleton via medial-axis
     transform — a discrete approximation of the Generalized Voronoi
     Diagram MORE uses.
  3. Build a graph where skeleton branch-points and endpoints are nodes
     and degree-2 chains between them are edges (each edge stores the
     polyline of pixels it traces).
  4. Sparsify by collapsing short degree-2 chains (MORE Alg. 1).
  5. For every detected door, evaluate a 2D Gaussian kernel along each
     graph edge and *delete* edges whose max-along-edge value exceeds a
     threshold. This is the key idea: cut the GRAPH, not the pixels.
  6. Connected components of the cut graph = rooms / regions.
  7. Each free cell is assigned to the room of its nearest skeleton node
     (one EDT call with return_indices).
  8. A region is reclassified as `open_space` if it is much larger than
     the median region or if its boundary is mostly frontier (not wall).
  9. For each door, look up the rooms within a few cells. Pairs of
     distinct rooms become room↔room adjacency edges (carried in the
     scene-graph JSON as `edges` and as `rooms[].neighbors`).
 10. Object → room assignment uses MORE's distance formula in spirit,
     min dV(v,u) + dE(O,v)^λ + dE(vp,u), simplified to operate on cell
     coordinates. The viewpoint is the camera position at first detection.
 11. Persistent room IDs across ticks via mask-IoU matching (replaces
     the old centroid-nearest matching, which flickers).

FALCON's connectivity graph is intentionally NOT consumed here — the two
serve different purposes (FALCON: low-level coverage path; this node:
semantic hierarchy for the LLM planner). They are complementary, not
competing.
"""

import json
import math
import time
from collections import defaultdict, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

import transforms3d

from .yolo_detector import DET_DOOR_PREFIX, DET_TARGET_PREFIX


# =====================================================================
#  BEV occupancy grid (log-odds) — unchanged from previous version
# =====================================================================
class BEVGrid:
    L_HIT     = +0.85
    L_MISS    = -0.40
    L_MIN     = -5.0
    L_MAX     =  5.0
    L_OCC_TH  =  0.85
    L_FREE_TH = -0.40

    def __init__(self, xmin, ymin, xmax, ymax, res):
        self.xmin, self.ymin, self.res = xmin, ymin, res
        self.W = int(math.ceil((xmax - xmin) / res))
        self.H = int(math.ceil((ymax - ymin) / res))
        self.logodds = np.zeros((self.H, self.W), dtype=np.float32)

    def w2c(self, x, y):
        return int((x - self.xmin) / self.res), int((y - self.ymin) / self.res)

    def c2w(self, cx, cy):
        return self.xmin + (cx + 0.5) * self.res, self.ymin + (cy + 0.5) * self.res

    def in_bounds(self, cx, cy):
        return 0 <= cx < self.W and 0 <= cy < self.H

    def integrate_rays(self, origin_xy, hits_xy):
        ox, oy = self.w2c(*origin_xy)
        if not self.in_bounds(ox, oy):
            return
        for (x, y) in hits_xy:
            hx, hy = self.w2c(x, y)
            if not self.in_bounds(hx, hy):
                continue
            n = max(1, int(math.hypot(hx - ox, hy - oy)))
            ts = np.linspace(0.0, 1.0, min(n, 12), endpoint=False)[1:]
            for t in ts:
                fx = int(ox + t * (hx - ox))
                fy = int(oy + t * (hy - oy))
                if self.in_bounds(fx, fy):
                    self.logodds[fy, fx] = np.clip(
                        self.logodds[fy, fx] + self.L_MISS, self.L_MIN, self.L_MAX)
            self.logodds[hy, hx] = np.clip(
                self.logodds[hy, hx] + self.L_HIT, self.L_MIN, self.L_MAX)

    def occ_mask(self):  return self.logodds >  self.L_OCC_TH
    def free_mask(self): return self.logodds <  self.L_FREE_TH
    def unk_mask(self):
        return (self.logodds >= self.L_FREE_TH) & (self.logodds <= self.L_OCC_TH)

    def frontier_mask(self):
        from scipy.ndimage import binary_dilation
        return self.free_mask() & binary_dilation(self.unk_mask(), iterations=1)


# =====================================================================
#  Voronoi / medial-axis graph
# =====================================================================
class VoronoiGraph:
    """
    Sparsified medial-axis graph over the BEV free mask.

    Nodes = skeleton pixels with degree != 2 (branch points + endpoints).
    Edges = chains of degree-2 skeleton pixels between two nodes; each
            edge stores the full pixel polyline so we can integrate
            Gaussian door kernels along it.

    Sparsification follows MORE Alg. 1: a degree-2 node x with neighbors
    (u,v) is removed and replaced by a single edge (u,v) iff
    dist(u,x)+dist(x,v) < c. We iterate until no such x remains.

    After cut_at_doors(), the graph is partitioned into connected
    components by union-find; each component is a "region" candidate.
    """

    NB8 = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]

    def __init__(self):
        self.skeleton = None        # bool HxW
        self.dist = None            # float HxW (distance to nearest obstacle, in cells)
        self.node_of = None         # int HxW: node id at this pixel, -1 elsewhere
        self.nodes = {}             # node_id -> (cx, cy)
        self.adj = defaultdict(dict)  # node_id -> {other_id: polyline list[(cx,cy)]}
        self.next_id = 0

    # ------------------------------------------------------------------
    def build(self, free_mask, sparsify_dist_cells=8):
        from skimage.morphology import medial_axis
        self.skeleton, self.dist = medial_axis(free_mask, return_distance=True)
        if not np.any(self.skeleton):
            self.node_of = -np.ones_like(free_mask, dtype=np.int32)
            return

        # Per-pixel skeleton degree (count of 8-neighbors that are also skeleton)
        deg = self._compute_degree(self.skeleton)

        # Initial nodes: every skeleton pixel with degree != 2
        ys, xs = np.where(self.skeleton & (deg != 2))
        self.node_of = -np.ones(self.skeleton.shape, dtype=np.int32)
        for x, y in zip(xs, ys):
            self.nodes[self.next_id] = (int(x), int(y))
            self.node_of[y, x] = self.next_id
            self.next_id += 1

        # Trace degree-2 chains between nodes to build edges
        self._trace_edges(deg)

        # MORE-style sparsification (Alg. 1)
        self._sparsify(sparsify_dist_cells)

    # ------------------------------------------------------------------
    @staticmethod
    def _compute_degree(skeleton):
        from scipy.ndimage import convolve
        kernel = np.array([[1,1,1],[1,0,1],[1,1,1]], dtype=np.uint8)
        deg = convolve(skeleton.astype(np.uint8), kernel, mode='constant', cval=0)
        deg[~skeleton] = 0
        return deg

    # ------------------------------------------------------------------
    def _trace_edges(self, deg):
        """Walk from each node along degree-2 pixels to a neighboring node."""
        H, W = self.skeleton.shape
        visited = np.zeros_like(self.skeleton, dtype=bool)
        for nid, (sx, sy) in list(self.nodes.items()):
            for dx, dy in self.NB8:
                x, y = sx + dx, sy + dy
                if not (0 <= x < W and 0 <= y < H): continue
                if not self.skeleton[y, x]: continue
                if visited[y, x]: continue
                # Walk along the chain
                polyline = [(sx, sy), (x, y)]
                px, py = sx, sy   # previous
                cx, cy = x, y
                visited[cy, cx] = True
                while self.node_of[cy, cx] < 0:
                    nxt = None
                    for ddx, ddy in self.NB8:
                        nx_, ny_ = cx + ddx, cy + ddy
                        if not (0 <= nx_ < W and 0 <= ny_ < H): continue
                        if not self.skeleton[ny_, nx_]: continue
                        if (nx_, ny_) == (px, py): continue
                        # If we hit another node, stop
                        if self.node_of[ny_, nx_] >= 0:
                            nxt = (nx_, ny_)
                            break
                        # Otherwise must be a degree-2 neighbor we haven't seen
                        if not visited[ny_, nx_]:
                            nxt = (nx_, ny_)
                            break
                    if nxt is None:
                        break
                    px, py = cx, cy
                    cx, cy = nxt
                    polyline.append((cx, cy))
                    if self.node_of[cy, cx] < 0:
                        visited[cy, cx] = True
                # If we landed on a node, register the edge
                end_id = self.node_of[cy, cx]
                if end_id >= 0 and end_id != nid:
                    # Length = sum of segment lengths
                    length = sum(math.hypot(polyline[i+1][0]-polyline[i][0],
                                            polyline[i+1][1]-polyline[i][1])
                                 for i in range(len(polyline)-1))
                    # Keep the shorter one if there's already an edge between these
                    existing = self.adj[nid].get(end_id)
                    if existing is None or self._polyline_len(existing) > length:
                        self.adj[nid][end_id] = polyline
                        self.adj[end_id][nid] = list(reversed(polyline))

    # ------------------------------------------------------------------
    @staticmethod
    def _polyline_len(poly):
        return sum(math.hypot(poly[i+1][0]-poly[i][0], poly[i+1][1]-poly[i][1])
                   for i in range(len(poly)-1))

    # ------------------------------------------------------------------
    def _sparsify(self, c):
        """MORE Alg. 1: collapse degree-2 nodes whose neighbors are within c."""
        changed = True
        while changed:
            changed = False
            for nid in list(self.nodes.keys()):
                if nid not in self.adj: continue
                neigh = list(self.adj[nid].keys())
                if len(neigh) != 2: continue
                u, v = neigh
                pu = self.adj[nid][u]
                pv = self.adj[nid][v]
                lu = self._polyline_len(pu)
                lv = self._polyline_len(pv)
                if lu + lv >= c: continue
                # Remove nid; new edge (u,v) is pu reversed + pv (skip duplicate node pixel)
                new_poly = list(reversed(pu)) + pv[1:]
                # Drop nid
                del self.adj[nid][u]; del self.adj[nid][v]
                if not self.adj[nid]: del self.adj[nid]
                self.adj[u].pop(nid, None)
                self.adj[v].pop(nid, None)
                # Erase from node_of so cell→node lookups skip it
                cx, cy = self.nodes[nid]
                self.node_of[cy, cx] = -1
                del self.nodes[nid]
                # Add (or keep shorter) edge u-v
                existing = self.adj[u].get(v)
                if existing is None or self._polyline_len(existing) > self._polyline_len(new_poly):
                    self.adj[u][v] = new_poly
                    self.adj[v][u] = list(reversed(new_poly))
                changed = True

    # ------------------------------------------------------------------
    def cut_at_doors(self, door_centers_cells, sigma_cells, threshold):
        """
        Drop edges that pass through a door's Gaussian footprint.

        For each edge polyline, evaluate a 2D Gaussian centered at each door
        sampled at every polyline pixel; if the max value (across all doors,
        across all pixels) exceeds `threshold`, delete the edge.

        v1 uses an isotropic Gaussian. To upgrade to MORE's oriented kernel:
        forward door bounding-box width/orientation from the detector,
        store per-door (sigma_along, sigma_across, theta), and replace the
        isotropic exp(-r^2/(2 sigma^2)) below with the rotated 2D form.
        """
        if not door_centers_cells or sigma_cells <= 0:
            return
        two_s2 = 2.0 * sigma_cells * sigma_cells
        doors = np.asarray(door_centers_cells, dtype=np.float32)  # (D,2)

        to_drop = []  # list of (a, b)
        for a, neigh in self.adj.items():
            for b, poly in neigh.items():
                if a >= b: continue  # process each undirected edge once
                pts = np.asarray(poly, dtype=np.float32)  # (P,2)
                # Pairwise squared distances point-to-door, take min over points
                # then convert to Gaussian and take max over doors.
                # Shape: (D, P)
                dx = doors[:, 0:1] - pts[:, 0:1].T  # (D, P)
                dy = doors[:, 1:2] - pts[:, 1:2].T
                d2 = dx * dx + dy * dy
                g = np.exp(-d2 / two_s2)         # (D, P)
                # An edge is "blocked" if at any pixel any door's Gaussian is high.
                if g.max() > threshold:
                    to_drop.append((a, b))

        for a, b in to_drop:
            self.adj[a].pop(b, None)
            self.adj[b].pop(a, None)

    # ------------------------------------------------------------------
    def components(self):
        """
        Returns:
            comp_of_node: dict node_id -> comp_id
            comp_count:   int
        """
        seen = set()
        comp_of = {}
        cid = 0
        for nid in self.nodes.keys():
            if nid in seen: continue
            stack = [nid]
            while stack:
                x = stack.pop()
                if x in seen: continue
                seen.add(x)
                comp_of[x] = cid
                stack.extend(self.adj.get(x, {}).keys())
            cid += 1
        return comp_of, cid


# =====================================================================
#  Node
# =====================================================================
class SemanticMapper(Node):
    def __init__(self):
        super().__init__("semantic_mapper")

        # ── Params ──
        self.declare_parameters("", [
            ("world_frame",     "world"),
            ("tick_rate",       2.0),
            ("bbox_xmin",     -20.0),
            ("bbox_ymin",     -20.0),
            ("bbox_xmax",      20.0),
            ("bbox_ymax",      20.0),
            ("bev_resolution",  0.15),
            ("z_slab_min",      0.3),
            ("z_slab_max",      1.8),
            ("max_depth",       5.0),
            ("depth_stride",    8),
            ("depth_min_dt",    0.15),

            # ── door cut ──
            # σ of the Gaussian (in metres). Set ≈ door half-width.
            ("door_sigma_m",    0.45),
            # threshold on max-along-edge Gaussian value to declare an edge cut.
            # 0.6 is a sensible middle ground; raise to keep more edges
            # (rooms more likely to merge), lower to cut more (rooms split more).
            ("door_cut_thresh", 0.6),

            # ── voronoi ──
            # MORE Alg. 1 sparsification length (in cells). Larger = sparser.
            ("voronoi_sparsify_cells", 8),

            # ── room post-processing ──
            ("min_room_cells",  60),
            # A region whose area > open_space_area_ratio * median(area) is
            # tagged as kind="open_space".
            ("open_space_area_ratio", 4.0),
            # ...or whose perimeter has more than this fraction touching frontier.
            ("open_space_frontier_frac", 0.5),

            # ── object assignment ──
            ("obj_merge_dist",  0.5),
            # MORE's exponent λ in min dV(v,u) + dE(O,v)^λ + dE(vp,u). Default 1.3.
            ("object_lambda",   1.3),

            # ── persistence ──
            # IoU threshold above which a new room inherits an old room's id.
            ("room_iou_match",  0.30),

            # ── topics ──
            ("drone_ns",       "/simple_drone"),
            ("depth_topic",    "/simple_drone/front_depth/depth/image_raw"),
            ("cam_info_topic", "/simple_drone/front_depth/depth/camera_info"),
            ("pose_topic",     "/simple_drone/gt_pose"),
            ("cam_offset_x",   0.2),
            ("cam_offset_y",   0.0),
            ("cam_offset_z",   0.0),

            # ── viz ──
            ("publish_voronoi_mesh", False),
            # Marker scales — defaults tuned for ~40–50 m maps (e.g. hospital).
            # Increase for very large outdoor scenes; decrease for small rooms.
            ("viz_room_sphere_radius_m", 0.8),
            ("viz_room_text_size_m",     1.2),
            ("viz_room_outline_width_m", 0.15),
            ("viz_room_edge_width_m",    0.25),
            ("viz_object_size_m",        0.4),
            ("viz_object_text_size_m",   0.5),
            ("viz_object_edge_width_m",  0.05),
            ("viz_door_radius_m",        0.4),
            ("viz_door_text_size_m",     0.6),
            ("viz_door_height_m",        2.5),
            # Fill rooms with translucent colored cells (much more visible
            # than outline-only). Subsamples to viz_max_fill_cells for big rooms.
            ("viz_fill_rooms",           True),
            ("viz_fill_open_space",      True),
            ("viz_fill_alpha",           0.45),
            ("viz_open_space_alpha",     0.18),
            ("viz_max_fill_cells",       8000),
        ])

        def g(name): return self.get_parameter(name).value
        self.world_frame    = g("world_frame")
        self.tick_rate      = float(g("tick_rate"))
        self.xmin, self.ymin = float(g("bbox_xmin")), float(g("bbox_ymin"))
        self.xmax, self.ymax = float(g("bbox_xmax")), float(g("bbox_ymax"))
        self.res            = float(g("bev_resolution"))
        self.z_slab_min     = float(g("z_slab_min"))
        self.z_slab_max     = float(g("z_slab_max"))
        self.max_depth      = float(g("max_depth"))
        self.depth_stride   = int(g("depth_stride"))
        self.depth_min_dt   = float(g("depth_min_dt"))

        self.door_sigma_m       = float(g("door_sigma_m"))
        self.door_cut_thresh    = float(g("door_cut_thresh"))
        self.voronoi_sparsify_c = int(g("voronoi_sparsify_cells"))
        self.min_room_cells     = int(g("min_room_cells"))
        self.open_space_area_ratio    = float(g("open_space_area_ratio"))
        self.open_space_frontier_frac = float(g("open_space_frontier_frac"))
        self.obj_merge_dist     = float(g("obj_merge_dist"))
        self.object_lambda      = float(g("object_lambda"))
        self.room_iou_match     = float(g("room_iou_match"))

        self.cam_x = float(g("cam_offset_x"))
        self.cam_y = float(g("cam_offset_y"))
        self.cam_z = float(g("cam_offset_z"))
        self.publish_voronoi_mesh = bool(g("publish_voronoi_mesh"))

        # Viz scales
        self.viz_room_sphere_r  = float(g("viz_room_sphere_radius_m"))
        self.viz_room_text_h    = float(g("viz_room_text_size_m"))
        self.viz_room_outline_w = float(g("viz_room_outline_width_m"))
        self.viz_room_edge_w    = float(g("viz_room_edge_width_m"))
        self.viz_object_s       = float(g("viz_object_size_m"))
        self.viz_object_text_h  = float(g("viz_object_text_size_m"))
        self.viz_obj_edge_w     = float(g("viz_object_edge_width_m"))
        self.viz_door_r         = float(g("viz_door_radius_m"))
        self.viz_door_text_h    = float(g("viz_door_text_size_m"))
        self.viz_door_h         = float(g("viz_door_height_m"))
        self.viz_fill_rooms     = bool(g("viz_fill_rooms"))
        self.viz_fill_open      = bool(g("viz_fill_open_space"))
        self.viz_fill_alpha     = float(g("viz_fill_alpha"))
        self.viz_open_alpha     = float(g("viz_open_space_alpha"))
        self.viz_max_fill_cells = int(g("viz_max_fill_cells"))

        depth_topic     = g("depth_topic")
        cam_info_topic  = g("cam_info_topic")
        pose_topic      = g("pose_topic")

        # ── Body→camera transform (must match URDF / adapter) ──
        self.T_b_c = np.array([
            [ 0.0,  0.0, 1.0, self.cam_x],
            [-1.0,  0.0, 0.0, self.cam_y],
            [ 0.0, -1.0, 0.0, self.cam_z],
            [ 0.0,  0.0, 0.0, 1.0 ],
        ])

        # ── State ──
        self.bev = BEVGrid(self.xmin, self.ymin, self.xmax, self.ymax, self.res)
        self.bridge = CvBridge()

        self.cur_pose = None
        self.K = None
        self.width = self.height = None
        self.last_depth_t = None
        self.last_depth_img = None

        self.last_dets = []
        self.doors   = []
        self.objects = []
        self._next_door_id = 0
        self._next_obj_id  = 0

        # Persistent room state. Each room is a dict
        #   { id, label, kind ("room"|"open_space"), centroid, mask (HxW bool),
        #     n_cells, tau, F, object_ids, neighbors (list of {room, via_door}) }
        self.rooms = []
        self.cell_to_room = None     # int HxW, -1 outside any room
        self._next_room_id = 0
        self._cur_room_id = None
        self._cur_room_t0 = None

        # Cached graph for object-assignment lookups
        self._voronoi = None
        self._comp_of_node = {}     # nid -> comp_id (post-cut)
        self._comp_to_room = {}     # comp_id -> room_id (this tick)

        # ── QoS ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── Pubs ──
        self.sg_pub     = self.create_publisher(String, "/scene_graph", latched_qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/scene_graph/markers", 1)
        self.bev_pub    = self.create_publisher(OccupancyGrid, "/scene_graph/bev",
                                                latched_qos)

        # ── Subs ──
        self.create_subscription(Pose, pose_topic, self._pose_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, sensor_qos)
        self.create_subscription(CameraInfo, cam_info_topic, self._cam_info_cb,
                                 sensor_qos)
        self.create_subscription(Detection2DArray, "/perception/detections",
                                 self._det_cb, 10)

        self.create_timer(1.0 / self.tick_rate, self._tick)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  semantic_mapper (MORE-style, ROS2)")
        self.get_logger().info(f"  bbox [{self.xmin:.1f},{self.ymin:.1f}] -> "
                               f"[{self.xmax:.1f},{self.ymax:.1f}]  res={self.res:.2f}m")
        self.get_logger().info(
            f"  BEV  {self.bev.W}x{self.bev.H}  z-slab [{self.z_slab_min:.1f},"
            f"{self.z_slab_max:.1f}]")
        self.get_logger().info(f"  door σ={self.door_sigma_m:.2f}m  "
                               f"cut_thresh={self.door_cut_thresh:.2f}")
        self.get_logger().info(f"  λ={self.object_lambda}  IoU match={self.room_iou_match}")
        self.get_logger().info("=" * 60)

    # ────────────────────────────────────────────────────────── callbacks
    def _pose_cb(self, msg):
        self.cur_pose = msg

    def _cam_info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.width, self.height = msg.width, msg.height
            self.get_logger().info(
                f"cam intrinsics: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} "
                f"cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}  {msg.width}x{msg.height}"
            )

    def _depth_cb(self, msg):
        now = self.get_clock().now()
        if self.last_depth_t is not None:
            dt = (now - self.last_depth_t).nanoseconds * 1e-9
            if dt < self.depth_min_dt:
                return
        self.last_depth_t = now
        if self.cur_pose is None or self.K is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception:
            return
        self.last_depth_img = depth
        self._integrate_depth_into_bev(depth)

    def _det_cb(self, msg: Detection2DArray):
        dets = []
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            tagged = hyp.class_id
            is_door = tagged.startswith(DET_DOOR_PREFIX)
            is_target = tagged.startswith(DET_TARGET_PREFIX)
            cls = tagged.split(":", 1)[1] if ":" in tagged else tagged
            dets.append({
                "cls":      cls,
                "conf":     hyp.score,
                "uv":       (det.bbox.center.position.x,
                             det.bbox.center.position.y),
                "is_door":  is_door,
                "is_target":is_target,
            })
        self.last_dets = dets

    # ────────────────────────────────────────────────────────── geometry
    def _T_world_cam(self):
        p = self.cur_pose
        q_wxyz = [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
        T = np.eye(4)
        T[:3, :3] = transforms3d.quaternions.quat2mat(q_wxyz)
        T[:3, 3] = [p.position.x, p.position.y, p.position.z]
        return T @ self.T_b_c

    def _pixel_to_world(self, u, v, z_c):
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        Xc = (u - cx) * z_c / fx
        Yc = (v - cy) * z_c / fy
        Zc = z_c
        return (self._T_world_cam() @ np.array([Xc, Yc, Zc, 1.0]))[:3]

    # ────────────────────────────────────────────────────────────── BEV
    def _integrate_depth_into_bev(self, depth):
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

        in_slab = (pts_w[:, 2] >= self.z_slab_min) & (pts_w[:, 2] <= self.z_slab_max)
        pts_w = pts_w[in_slab]
        if pts_w.shape[0] == 0:
            return

        p = self.cur_pose.position
        origin = (p.x, p.y)
        if pts_w.shape[0] > 1500:
            idx = np.random.choice(pts_w.shape[0], 1500, replace=False)
            pts_w = pts_w[idx]
        self.bev.integrate_rays(origin, [(float(x), float(y)) for x, y, _ in pts_w])

    # ──────────────────────────────────────────────── detection lifting
    def _lift_detections(self):
        if self.last_depth_img is None or self.cur_pose is None or self.K is None:
            return
        if not self.last_dets:
            return
        depth = self.last_depth_img
        H, W = depth.shape
        vp = (self.cur_pose.position.x, self.cur_pose.position.y)
        for d in self.last_dets:
            u, v = d["uv"]
            ui, vi = int(round(u)), int(round(v))
            if not (0 <= ui < W and 0 <= vi < H):
                continue
            r = 2
            patch = depth[max(0, vi-r):vi+r+1, max(0, ui-r):ui+r+1]
            patch = patch[np.isfinite(patch) & (patch > 0.1) & (patch < self.max_depth)]
            if patch.size == 0:
                continue
            z_c = float(np.median(patch))
            p_w = self._pixel_to_world(u, v, z_c)
            if d["is_door"]:
                self._upsert_door(p_w[0], p_w[1], d["conf"])
            else:
                self._upsert_object(d["cls"], d["conf"], p_w[0], p_w[1], p_w[2], vp)
        self.last_dets = []

    def _upsert_door(self, x, y, conf):
        for door in self.doors:
            if math.hypot(door["xy"][0]-x, door["xy"][1]-y) < self.obj_merge_dist:
                door["conf"] = max(door["conf"], conf)
                return
        self.doors.append({
            "id": self._next_door_id, "xy": (x, y), "conf": conf,
            "rooms": [],   # filled by _build_rooms
        })
        self._next_door_id += 1

    def _upsert_object(self, cls, conf, x, y, z, viewpoint_xy):
        for obj in self.objects:
            if obj["cls"] == cls and \
               math.hypot(obj["xy"][0]-x, obj["xy"][1]-y) < self.obj_merge_dist:
                if conf > obj["conf"]:
                    obj["conf"] = conf; obj["xy"] = (x, y); obj["z"] = z
                    obj["vp"] = viewpoint_xy
                return
        self.objects.append({
            "id": self._next_obj_id, "cls": cls, "conf": conf,
            "xy": (x, y), "z": z, "vp": viewpoint_xy, "room": None,
        })
        self._next_obj_id += 1

    # ─────────────────────────────────────────────────────────── rooms
    def _build_rooms(self):
        """
        End-to-end MORE-style room construction:
           free_mask -> Voronoi -> cut_at_doors -> components ->
           cells assigned to nearest skeleton-component -> persistent ids
           -> open_space classification -> object assignment ->
           door/room adjacency.
        """
        from scipy.ndimage import distance_transform_edt

        free = self.bev.free_mask()
        if not np.any(free):
            self.rooms = []
            self.cell_to_room = -np.ones_like(free, dtype=np.int32)
            self._voronoi = None
            return

        # 1) Voronoi / medial axis
        vg = VoronoiGraph()
        vg.build(free, sparsify_dist_cells=self.voronoi_sparsify_c)
        self._voronoi = vg

        # 2) Cut edges at doors
        door_cells = []
        for door in self.doors:
            cx, cy = self.bev.w2c(*door["xy"])
            if 0 <= cx < self.bev.W and 0 <= cy < self.bev.H:
                door_cells.append((cx, cy))
        sigma_cells = max(0.5, self.door_sigma_m / self.res)
        vg.cut_at_doors(door_cells, sigma_cells, self.door_cut_thresh)

        # 3) Components of cut graph
        comp_of_node, n_comp = vg.components()
        self._comp_of_node = comp_of_node

        # 4) Assign each free cell to the component of its nearest skeleton node.
        #    distance_transform_edt with return_indices on ~node_mask gives us,
        #    for every pixel, the (y,x) of the nearest node-pixel.
        node_mask = (vg.node_of >= 0)
        if not np.any(node_mask):
            self.rooms = []
            self.cell_to_room = -np.ones_like(free, dtype=np.int32)
            return
        # Seed each node-pixel with its component id (+1 so 0 means "no seed")
        comp_field = np.zeros_like(vg.node_of, dtype=np.int32)
        for nid, cid in comp_of_node.items():
            cx, cy = vg.nodes[nid]
            comp_field[cy, cx] = cid + 1
        # EDT from the inverse of node_mask
        _, indices = distance_transform_edt(~node_mask, return_indices=True)
        nearest_y, nearest_x = indices  # each HxW
        comp_per_cell = comp_field[nearest_y, nearest_x] - 1   # -1 where seed was 0
        # Restrict to free cells; everything else is -1
        cell_to_comp = np.where(free, comp_per_cell, -1).astype(np.int32)

        # 5) Build candidate room dicts; drop tiny components
        candidates = []
        for cid in range(n_comp):
            mask = (cell_to_comp == cid)
            n_cells = int(mask.sum())
            if n_cells < self.min_room_cells:
                cell_to_comp[mask] = -1
                continue
            ys, xs = np.where(mask)
            wx = self.xmin + (xs + 0.5) * self.res
            wy = self.ymin + (ys + 0.5) * self.res
            centroid = (float(wx.mean()), float(wy.mean()))
            candidates.append({
                "comp_id": cid, "mask": mask, "n_cells": n_cells,
                "centroid": centroid,
            })

        # 6) Persistent ID assignment via mask IoU vs. previous tick
        prev_rooms = self.rooms
        new_rooms = []
        prev_mask_shape = prev_rooms[0]["mask"].shape if prev_rooms else None
        if prev_mask_shape is not None and prev_mask_shape != free.shape:
            prev_rooms = []   # bbox/res changed; can't compare
        used_prev = set()
        for cand in candidates:
            best_id, best_iou = None, 0.0
            for prev in prev_rooms:
                if prev["id"] in used_prev: continue
                inter = np.logical_and(cand["mask"], prev["mask"]).sum()
                if inter == 0: continue
                union = np.logical_or(cand["mask"], prev["mask"]).sum()
                iou = inter / union if union else 0.0
                if iou > best_iou:
                    best_iou = iou; best_id = prev["id"]
            if best_id is not None and best_iou >= self.room_iou_match:
                # Inherit id and rolling state
                old = next(p for p in prev_rooms if p["id"] == best_id)
                used_prev.add(best_id)
                new_rooms.append({
                    "id": best_id, "label": old["label"], "kind": old["kind"],
                    "tau": old["tau"], "centroid": cand["centroid"],
                    "mask": cand["mask"], "n_cells": cand["n_cells"],
                    "comp_id": cand["comp_id"],
                    "F": 0, "object_ids": [], "neighbors": [],
                })
            else:
                rid = self._next_room_id; self._next_room_id += 1
                new_rooms.append({
                    "id": rid, "label": "unknown", "kind": "room",
                    "tau": 0.0, "centroid": cand["centroid"],
                    "mask": cand["mask"], "n_cells": cand["n_cells"],
                    "comp_id": cand["comp_id"],
                    "F": 0, "object_ids": [], "neighbors": [],
                })

        self.rooms = new_rooms
        self.cell_to_room = -np.ones_like(free, dtype=np.int32)
        for r in self.rooms:
            self.cell_to_room[r["mask"]] = r["id"]
        # Map skeleton-component id → room id (for object assignment / adjacency)
        self._comp_to_room = {r["comp_id"]: r["id"] for r in self.rooms}

        # 7) open_space classification
        if self.rooms:
            areas = sorted(r["n_cells"] for r in self.rooms)
            median = areas[len(areas)//2]
            fmask = self.bev.frontier_mask()
            occ = self.bev.occ_mask()
            for r in self.rooms:
                from scipy.ndimage import binary_dilation
                edge = np.logical_and(r["mask"],
                                      binary_dilation(~r["mask"], iterations=1))
                edge_n = int(edge.sum())
                if edge_n == 0:
                    continue
                front_touch = int(np.logical_and(edge,
                                  binary_dilation(fmask, iterations=1)).sum())
                wall_touch  = int(np.logical_and(edge,
                                  binary_dilation(occ, iterations=1)).sum())
                front_frac = front_touch / edge_n
                wall_frac  = wall_touch  / edge_n
                # Open-space heuristic (either signal triggers it)
                if (r["n_cells"] > self.open_space_area_ratio * median and
                        front_frac > 0.2 and wall_frac < 0.5) or \
                   (front_frac > self.open_space_frontier_frac and
                        r["n_cells"] > median):
                    r["kind"] = "open_space"
                    if r["label"] == "unknown":
                        r["label"] = "open_space"
                # Frontier-cell count for the LLM (unchanged semantics)
                r["F"] = int(np.logical_and(fmask, r["mask"]).sum())

        # 8) Object → room assignment (MORE formula, simplified)
        for obj in self.objects:
            obj["room"] = self._assign_object_to_room(obj)
        for r in self.rooms:
            r["object_ids"] = [o["id"] for o in self.objects if o["room"] == r["id"]]

        # 9) Door → adjacency
        r_cells_for_door = max(2, int(round(self.door_sigma_m * 2 / self.res)))
        edges = []  # list of {a, b, via_door}
        seen_edges = set()
        for door in self.doors:
            cx, cy = self.bev.w2c(*door["xy"])
            x0 = max(0, cx - r_cells_for_door); x1 = min(self.bev.W, cx + r_cells_for_door + 1)
            y0 = max(0, cy - r_cells_for_door); y1 = min(self.bev.H, cy + r_cells_for_door + 1)
            sub = self.cell_to_room[y0:y1, x0:x1]
            ids = sorted(int(v) for v in np.unique(sub) if v >= 0)
            door["rooms"] = ids
            # Make pairwise edges from this door
            for i in range(len(ids)):
                for j in range(i+1, len(ids)):
                    a, b = ids[i], ids[j]
                    key = (a, b)
                    if key in seen_edges: continue
                    seen_edges.add(key)
                    edges.append({"a": a, "b": b, "via_door": door["id"]})
                    self._lookup_room(a)["neighbors"].append({"room": b, "via_door": door["id"]})
                    self._lookup_room(b)["neighbors"].append({"room": a, "via_door": door["id"]})
        # Open_space implicit edges: any room sharing a Voronoi-graph component-edge
        # with an open_space gets a no-door neighbor relation. We approximate this
        # by checking, for each pair (room, open_space) that share door-less border
        # cells (touch in cell-space without an obstacle between them).
        from scipy.ndimage import binary_dilation
        open_rooms = [r for r in self.rooms if r["kind"] == "open_space"]
        other_rooms = [r for r in self.rooms if r["kind"] != "open_space"]
        for o in open_rooms:
            o_dilated = binary_dilation(o["mask"], iterations=1)
            for r in other_rooms:
                if np.any(o_dilated & r["mask"]):
                    a, b = sorted([o["id"], r["id"]])
                    key = (a, b)
                    if key in seen_edges: continue
                    seen_edges.add(key)
                    edges.append({"a": a, "b": b, "via_door": None})
                    o["neighbors"].append({"room": r["id"], "via_door": None})
                    r["neighbors"].append({"room": o["id"], "via_door": None})

        self._edges = edges

        # 10) Update τ for the room currently containing the robot
        if self.cur_pose is not None and self.rooms:
            pcx, pcy = self.bev.w2c(self.cur_pose.position.x,
                                    self.cur_pose.position.y)
            rid = self._nearest_room_id(pcx, pcy)
            now = time.time()
            if rid is not None:
                if self._cur_room_id == rid and self._cur_room_t0 is not None:
                    dt = now - self._cur_room_t0
                    self._lookup_room(rid)["tau"] += dt
                self._cur_room_id = rid
                self._cur_room_t0 = now

    # ------------------------------------------------------------------
    def _lookup_room(self, rid):
        for r in self.rooms:
            if r["id"] == rid: return r
        return None

    def _nearest_room_id(self, cx, cy, max_search=30):
        if self.cell_to_room is None: return None
        if 0 <= cx < self.bev.W and 0 <= cy < self.bev.H:
            v = self.cell_to_room[cy, cx]
            if v >= 0: return int(v)
        H, W = self.cell_to_room.shape
        seen = np.zeros((H, W), dtype=bool)
        q = deque([(cx, cy, 0)])
        while q:
            x, y, d = q.popleft()
            if d > max_search: continue
            if not (0 <= x < W and 0 <= y < H): continue
            if seen[y, x]: continue
            seen[y, x] = True
            v = self.cell_to_room[y, x]
            if v >= 0: return int(v)
            q.extend([(x+1,y,d+1),(x-1,y,d+1),(x,y+1,d+1),(x,y-1,d+1)])
        return None

    # ------------------------------------------------------------------
    def _assign_object_to_room(self, obj):
        """
        MORE-style assignment:
            min over rooms r of
                dV(node_near_obj_in_r, node_near_vp_in_r)
              + dE(obj, node_near_obj_in_r)^λ
              + dE(vp, node_near_vp_in_r)
        We compute it cheaply: for each room, take the node closest to the
        object (in cell-space, restricted to that room's component) and the
        node closest to the viewpoint (same), and use Euclidean distance
        for dV (since the graph is small, exact dV would be over-engineering
        at this stage; comment below shows how to upgrade).
        """
        if not self.rooms or self._voronoi is None:
            # Fall back: nearest reachable room from the object cell
            ox, oy = self.bev.w2c(*obj["xy"])
            return self._nearest_room_id(ox, oy)

        ocx, ocy = self.bev.w2c(*obj["xy"])
        vp = obj.get("vp") or (obj["xy"][0], obj["xy"][1])
        vcx, vcy = self.bev.w2c(*vp)

        best_room, best_cost = None, float("inf")
        for r in self.rooms:
            cid = r["comp_id"]
            # Candidate nodes that belong to this component
            cand = [(nid, self._voronoi.nodes[nid])
                    for nid, c in self._comp_of_node.items() if c == cid]
            if not cand:
                continue
            # Nearest node to object
            nO_id, (nOx, nOy) = min(cand, key=lambda kv:
                                    (kv[1][0]-ocx)**2 + (kv[1][1]-ocy)**2)
            # Nearest node to viewpoint
            nV_id, (nVx, nVy) = min(cand, key=lambda kv:
                                    (kv[1][0]-vcx)**2 + (kv[1][1]-vcy)**2)
            dE_obj = math.hypot(ocx - nOx, ocy - nOy) * self.res
            dE_vp  = math.hypot(vcx - nVx, vcy - nVy) * self.res
            # Approximate dV by Euclidean between the two graph nodes (in metres).
            # To use the exact graph distance, run nx.shortest_path_length on the
            # cut graph weighted by polyline-length-in-metres.
            dV = math.hypot(nOx - nVx, nOy - nVy) * self.res
            cost = dV + dE_obj ** self.object_lambda + dE_vp
            if cost < best_cost:
                best_cost = cost; best_room = r["id"]
        return best_room

    # ──────────────────────────────────────────────────── publish: JSON
    def _serialize_scene_graph(self):
        cur_room = None
        if self.cur_pose is not None:
            cx, cy = self.bev.w2c(self.cur_pose.position.x,
                                  self.cur_pose.position.y)
            cur_room = self._nearest_room_id(cx, cy)
        return {
            "stamp": self.get_clock().now().nanoseconds * 1e-9,
            "current_room": cur_room,
            "rooms": [{
                "id": r["id"], "label": r["label"], "kind": r["kind"],
                "centroid": list(r["centroid"]),
                "n_cells": r["n_cells"], "tau": r["tau"], "F": r["F"],
                "objects": r["object_ids"],
                "neighbors": r["neighbors"],
            } for r in self.rooms],
            "objects": [{
                "id": o["id"], "cls": o["cls"], "conf": o["conf"],
                "xy": list(o["xy"]), "z": o["z"], "room": o["room"],
            } for o in self.objects],
            "doors": [{
                "id": d["id"], "xy": list(d["xy"]),
                "conf": d["conf"], "rooms": d.get("rooms", []),
            } for d in self.doors],
            "edges": getattr(self, "_edges", []),
        }

    def _publish_bev(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.world_frame
        grid.info.resolution = self.res
        grid.info.width  = self.bev.W
        grid.info.height = self.bev.H
        grid.info.origin.position.x = self.xmin
        grid.info.origin.position.y = self.ymin
        grid.info.origin.orientation.w = 1.0
        out = -np.ones((self.bev.H, self.bev.W), dtype=np.int8)
        out[self.bev.free_mask()] = 0
        out[self.bev.occ_mask()]  = 100
        grid.data = out.flatten().tolist()
        self.bev_pub.publish(grid)

    # ──────────────────────────────────────────────── publish: markers
    def _room_color(self, r):
        """Stable per-room color. Open spaces get a soft slate so they don't
        compete with the colorful rooms."""
        import colorsys
        if r["kind"] == "open_space":
            return (0.55, 0.62, 0.70)   # slate
        # Golden-ratio hue stride for maximum perceptual separation between
        # neighboring IDs.
        h = (r["id"] * 0.6180339887) % 1.0
        rgb = colorsys.hsv_to_rgb(h, 0.85, 0.95)
        return (rgb[0], rgb[1], rgb[2])

    # ──────────────────────────────────────────────────────────────────
    # Z-stacking plan (so nothing visually clobbers anything else)
    #   0.00     BEV occupancy (separate /scene_graph/bev topic)
    #   0.05     Filled room cells (CUBE_LIST per room, 2 cm thick)
    #   0.20     Room polygon outlines (thick LINE_STRIP)
    #   varies   Object cubes (lifted z from depth)
    #   varies   Doors: cylinders centered at door_h / 2
    #   2.00     Room centroid spheres
    #   2.50     Room↔room edges (above doors so they read clearly)
    #   3.50     Room labels
    # ──────────────────────────────────────────────────────────────────
    Z_FILL        = 0.05
    Z_OUTLINE     = 0.20
    Z_ROOM_SPHERE = 2.00
    Z_EDGE        = 2.50
    Z_ROOM_LABEL  = 3.50
    Z_DOOR_LABEL  = 3.20

    def _publish_markers(self, sg):
        from skimage.measure import find_contours
        arr = MarkerArray()
        mid = 0
        stamp = self.get_clock().now().to_msg()

        # Full clear so deleted rooms / objects actually vanish in RViz
        clear = Marker(); clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        room_pos = {r["id"]: r["centroid"] for r in sg["rooms"]}

        # ── 1) Filled room cells (the headline visual) ───────────────
        # CUBE_LIST is GPU-instanced in RViz, so even ~5–8k cubes per room
        # render fine. We subsample huge rooms to keep it snappy.
        if self.viz_fill_rooms:
            for r in self.rooms:
                if r["kind"] == "open_space" and not self.viz_fill_open:
                    continue
                color = self._room_color(r)
                m = Marker()
                m.header.frame_id = self.world_frame; m.header.stamp = stamp
                m.ns = "room_fill"; m.id = mid; mid += 1
                m.type = Marker.CUBE_LIST; m.action = Marker.ADD
                m.scale.x = self.res
                m.scale.y = self.res
                m.scale.z = 0.02
                m.pose.orientation.w = 1.0
                ys, xs = np.where(r["mask"])
                if len(xs) > self.viz_max_fill_cells:
                    idx = np.random.choice(len(xs), self.viz_max_fill_cells,
                                           replace=False)
                    xs = xs[idx]; ys = ys[idx]
                for cx, cy in zip(xs, ys):
                    wx, wy = self.bev.c2w(int(cx), int(cy))
                    m.points.append(Point(x=wx, y=wy, z=self.Z_FILL))
                alpha = (self.viz_open_alpha
                         if r["kind"] == "open_space" else self.viz_fill_alpha)
                m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)
                arr.markers.append(m)

        # ── 2) Room polygon outlines (thick) ─────────────────────────
        for r in self.rooms:
            color = self._room_color(r)
            contours = find_contours(r["mask"].astype(np.float32), 0.5)
            for contour in contours:
                m = Marker()
                m.header.frame_id = self.world_frame; m.header.stamp = stamp
                m.ns = "room_polygons"; m.id = mid; mid += 1
                m.type = Marker.LINE_STRIP; m.action = Marker.ADD
                m.scale.x = self.viz_room_outline_w
                m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
                for cy, cx in contour:
                    wx, wy = self.bev.c2w(cx, cy)
                    m.points.append(Point(x=wx, y=wy, z=self.Z_OUTLINE))
                if m.points:
                    m.points.append(m.points[0])
                m.pose.orientation.w = 1.0
                arr.markers.append(m)

        # ── 3) Room centroid spheres ─────────────────────────────────
        for r in sg["rooms"]:
            color = self._room_color(self._lookup_room(r["id"]))
            m = Marker()
            m.header.frame_id = self.world_frame; m.header.stamp = stamp
            m.ns = "rooms"; m.id = mid; mid += 1
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position = Point(x=r["centroid"][0], y=r["centroid"][1],
                                    z=self.Z_ROOM_SPHERE)
            m.pose.orientation.w = 1.0
            s = self.viz_room_sphere_r * 2.0   # SPHERE.scale = diameter
            m.scale.x = m.scale.y = m.scale.z = s
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            arr.markers.append(m)

        # ── 4) Room labels (large, two-line) ─────────────────────────
        for r in sg["rooms"]:
            t = Marker()
            t.header.frame_id = self.world_frame; t.header.stamp = stamp
            t.ns = "room_labels"; t.id = mid; mid += 1
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position = Point(x=r["centroid"][0], y=r["centroid"][1],
                                    z=self.Z_ROOM_LABEL)
            t.pose.orientation.w = 1.0
            t.scale.z = self.viz_room_text_h
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            kind_tag = "" if r["kind"] == "room" else f"  [{r['kind']}]"
            t.text = (f'{r["label"].upper()} #{r["id"]}{kind_tag}\n'
                      f'objs={len(r["objects"])}  '
                      f'τ={r["tau"]:.0f}s  F={r["F"]}')
            arr.markers.append(t)

        # ── 5) Room ↔ Room edges (routed through doors) ──────────────
        for e in sg["edges"]:
            if e["a"] not in room_pos or e["b"] not in room_pos: continue
            m = Marker()
            m.header.frame_id = self.world_frame; m.header.stamp = stamp
            m.ns = "room_room_edges"; m.id = mid; mid += 1
            m.type = Marker.LINE_LIST; m.action = Marker.ADD
            m.scale.x = self.viz_room_edge_w
            if e["via_door"] is None:
                # Open-space adjacency — softer color
                m.color = ColorRGBA(r=0.65, g=0.65, b=0.65, a=0.95)
            else:
                # Door adjacency — bright yellow
                m.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=1.0)
            ax, ay = room_pos[e["a"]]
            bx, by = room_pos[e["b"]]
            if e["via_door"] is not None:
                door = next((d for d in sg["doors"]
                             if d["id"] == e["via_door"]), None)
                if door is not None:
                    dx, dy = door["xy"]
                    m.points.append(Point(x=ax, y=ay, z=self.Z_EDGE))
                    m.points.append(Point(x=dx, y=dy, z=self.Z_EDGE))
                    m.points.append(Point(x=dx, y=dy, z=self.Z_EDGE))
                    m.points.append(Point(x=bx, y=by, z=self.Z_EDGE))
                else:
                    m.points.append(Point(x=ax, y=ay, z=self.Z_EDGE))
                    m.points.append(Point(x=bx, y=by, z=self.Z_EDGE))
            else:
                m.points.append(Point(x=ax, y=ay, z=self.Z_EDGE))
                m.points.append(Point(x=bx, y=by, z=self.Z_EDGE))
            m.pose.orientation.w = 1.0
            arr.markers.append(m)

        # ── 6) Objects + parent-room edges ───────────────────────────
        s_obj = self.viz_object_s
        for o in sg["objects"]:
            m = Marker()
            m.header.frame_id = self.world_frame; m.header.stamp = stamp
            m.ns = "objects"; m.id = mid; mid += 1
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position = Point(x=o["xy"][0], y=o["xy"][1], z=o["z"])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = s_obj
            m.color = ColorRGBA(r=0.1, g=0.95, b=0.1, a=1.0)
            arr.markers.append(m)

            t = Marker()
            t.header.frame_id = self.world_frame; t.header.stamp = stamp
            t.ns = "object_labels"; t.id = mid; mid += 1
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position = Point(x=o["xy"][0], y=o["xy"][1],
                                    z=o["z"] + s_obj + 0.3)
            t.pose.orientation.w = 1.0
            t.scale.z = self.viz_object_text_h
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = o["cls"]
            arr.markers.append(t)

            if o["room"] is not None and o["room"] in room_pos:
                e = Marker()
                e.header.frame_id = self.world_frame; e.header.stamp = stamp
                e.ns = "room_object_edges"; e.id = mid; mid += 1
                e.type = Marker.LINE_LIST; e.action = Marker.ADD
                e.scale.x = self.viz_obj_edge_w
                e.color = ColorRGBA(r=0.2, g=0.7, b=0.2, a=0.55)
                rx, ry = room_pos[o["room"]]
                e.points.append(Point(x=o["xy"][0], y=o["xy"][1], z=o["z"]))
                e.points.append(Point(x=rx, y=ry, z=self.Z_ROOM_SPHERE))
                e.pose.orientation.w = 1.0
                arr.markers.append(e)

        # ── 7) Doors (taller, with id labels) ────────────────────────
        for d in sg["doors"]:
            m = Marker()
            m.header.frame_id = self.world_frame; m.header.stamp = stamp
            m.ns = "doors"; m.id = mid; mid += 1
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position = Point(x=d["xy"][0], y=d["xy"][1],
                                    z=self.viz_door_h * 0.5)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = self.viz_door_r * 2.0
            m.scale.z = self.viz_door_h
            m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
            arr.markers.append(m)

            t = Marker()
            t.header.frame_id = self.world_frame; t.header.stamp = stamp
            t.ns = "door_labels"; t.id = mid; mid += 1
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position = Point(x=d["xy"][0], y=d["xy"][1],
                                    z=self.Z_DOOR_LABEL)
            t.pose.orientation.w = 1.0
            t.scale.z = self.viz_door_text_h
            t.color = ColorRGBA(r=1.0, g=0.65, b=0.1, a=1.0)
            t.text = f"door#{d['id']}"
            arr.markers.append(t)

        # ── 8) Voronoi mesh (debug; OFF by default) ──────────────────
        if self.publish_voronoi_mesh and self._voronoi is not None:
            m = Marker()
            m.header.frame_id = self.world_frame; m.header.stamp = stamp
            m.ns = "voronoi_mesh"; m.id = mid; mid += 1
            m.type = Marker.LINE_LIST; m.action = Marker.ADD
            m.scale.x = 0.04
            m.color = ColorRGBA(r=0.4, g=0.6, b=1.0, a=0.6)
            seen = set()
            for a, neigh in self._voronoi.adj.items():
                for b, poly in neigh.items():
                    if (b, a) in seen: continue
                    seen.add((a, b))
                    for i in range(len(poly) - 1):
                        wx0, wy0 = self.bev.c2w(poly[i][0], poly[i][1])
                        wx1, wy1 = self.bev.c2w(poly[i+1][0], poly[i+1][1])
                        m.points.append(Point(x=wx0, y=wy0, z=0.15))
                        m.points.append(Point(x=wx1, y=wy1, z=0.15))
            m.pose.orientation.w = 1.0
            arr.markers.append(m)

        self.marker_pub.publish(arr)

    # ──────────────────────────────────────────────────────────────────
    def _tick(self):
        if self.cur_pose is None:
            return
        self._lift_detections()
        self._build_rooms()
        sg = self._serialize_scene_graph()
        self.sg_pub.publish(String(data=json.dumps(sg)))
        self._publish_bev()
        self._publish_markers(sg)


def main():
    rclpy.init()
    node = SemanticMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()