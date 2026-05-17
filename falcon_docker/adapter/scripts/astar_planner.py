#!/usr/bin/env python3
"""
astar_planner.py — A* on the 2D BEV → smoothed, corner-preserving waypoints.

v4 — speed + waypoint geometry fixes
  vs v3:
  • Bounding-box restricted A*: the search domain is the bbox of
    start∪goal expanded by `search_margin_m`. On a 160×160 BEV with a
    near-by goal this typically cuts expansions 5–20×.
  • Octile heuristic instead of Euclidean. Tighter admissible bound for
    8-connected motion → fewer node expansions, same optimality.
  • Cost-map cache keyed on the bev message identity. The collision
    re-check and the subsequent replan no longer dilate the occupancy
    mask twice for the same BEV.
  • Faster BEV decode via `np.frombuffer(bytes(...))`.
  • New `_split_long` resampler: keeps every LOS-smoothed corner exactly
    where A* placed it and only inserts intermediate points on segments
    longer than `waypoint_spacing_m`. So a 6 m straight produces 2
    waypoints (start, end), not 7. Drone yaws only at real corners.
  • `start_skip_m`: leading waypoints within this distance of the start
    pose are dropped from the published path. Stops the follower from
    being pointed at a waypoint that's effectively behind it whenever a
    replan happens mid-flight (the source of "drone went back to a
    point it just visited").

Inputs / outputs / topics: unchanged from v3.
"""
import heapq
import math
import numpy as np
import rospy

from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path


SQRT2 = math.sqrt(2.0)


class AStarPlanner:
    def __init__(self):
        rospy.init_node("astar_planner")
        G = rospy.get_param

        self.drone_ns           = G("~drone_ns",        "/simple_drone")
        self.bev_topic          = G("~bev_topic",       "/falcon/bev_2d")
        self.path_topic         = G("~path_topic",      "/path/waypoints")
        self.goal_topic         = G("~goal_topic",      "/waypoint_nav/goal")
        self.frame_id           = G("~frame_id",        "world")
        # NOTE: with the new corner-preserving resampler, this acts as
        # "max segment length" — corners are kept exactly. Bump it to
        # 3–5 m for long, direct flight legs.
        self.waypoint_spacing_m = float(G("~waypoint_spacing_m", 3.0))
        self.inflate_radius_m   = float(G("~inflate_radius_m",   0.4))
        self.unknown_blocked    = bool (G("~unknown_blocked",    False))
        self.unknown_cost       = float(G("~unknown_cost",       1.0))
        self.los_smoothing      = bool (G("~los_smoothing",      True))
        self.turn_penalty       = float(G("~turn_penalty",       0.0))
        self.replan_on_collision = bool(G("~replan_on_collision", True))
        self.replan_on_bev      = bool (G("~replan_on_bev",      False))
        self.replan_period_s    = float(G("~replan_period_s",    0.0))
        self.snap_radius_m      = float(G("~goal_snap_radius_m", 2.0))
        # NEW
        self.search_margin_m    = float(G("~search_margin_m",    3.0))
        self.start_skip_m       = float(G("~start_skip_m",       0.4))
        # ── Map-warmup gate (bug-1 fix) ──────────────────────────
        # On startup bev_publisher latches a BEV that is all-UNK plus
        # the simulated office walls (OCC). With unknown_blocked=False
        # an all-UNK map looks like wide-open free space, so A* finds
        # a straight path to the goal and the drone flies before FALCON
        # has integrated a single real depth frame. We refuse to
        # publish ANY path until the BEV holds at least this many genuine
        # FREE cells — FREE only comes from FALCON's real
        # occupancy_grid_free cloud, never from the simulated walls or
        # from UNK, so it's a true "the map has warmed up" signal.
        # Set 0 to disable (restores old behaviour). A goal click does
        # NOT bypass this gate — flying into an unmapped world on a
        # click is just as unsafe as doing it on the init goal.
        self.min_free_cells = int(G("~min_free_cells_to_plan", 80))
        self._warmed_up     = (self.min_free_cells <= 0)

        gx = G("~goal_x", None); gy = G("~goal_y", None)
        self.goal_xy = ((float(gx), float(gy))
                        if gx is not None and gy is not None else None)

        self.pose_xy   = None
        self.bev       = None
        self.has_plan  = False
        self.fail_reason = "(not tried yet)"
        self.last_cells = []

        # Cost-map cache: keyed on the bev message object identity.
        # Same bev → same cost/occ → no second dilation pass.
        self._cost_cache_for = None
        self._cost_cache     = None

        self.pub_path = rospy.Publisher(self.path_topic, Path,
                                         queue_size=1, latch=True)
        rospy.Subscriber(self.bev_topic, OccupancyGrid,
                         self._bev_cb,  queue_size=1)
        rospy.Subscriber(self.drone_ns + "/gt_pose", Pose,
                         self._pose_cb, queue_size=10)
        rospy.Subscriber(self.goal_topic, Point,
                         self._goal_cb, queue_size=1)

        if self.replan_period_s > 0:
            rospy.Timer(rospy.Duration(self.replan_period_s),
                        lambda _e: self._try_plan())

        rospy.Timer(rospy.Duration(2.0), self._status)

        rospy.loginfo("=" * 64)
        rospy.loginfo("astar_planner v4 ready")
        rospy.loginfo("  bev   in  = %s", self.bev_topic)
        rospy.loginfo("  pose  in  = %s/gt_pose", self.drone_ns)
        rospy.loginfo("  goal  in  = %s", self.goal_topic)
        rospy.loginfo("  path  out = %s", self.path_topic)
        rospy.loginfo("  goal init = %s",
                      "(%.2f,%.2f)" % self.goal_xy if self.goal_xy else "none")
        rospy.loginfo("  max_seg=%.2fm  inflate=%.2fm  unknown=%s  "
                      "los_smooth=%s  turn_pen=%.2f  collision_replan=%s",
                      self.waypoint_spacing_m, self.inflate_radius_m,
                      "blocked" if self.unknown_blocked
                                else "free×%.1f" % self.unknown_cost,
                      self.los_smoothing, self.turn_penalty,
                      self.replan_on_collision)
        rospy.loginfo("  search_margin=%.1fm  start_skip=%.2fm",
                      self.search_margin_m, self.start_skip_m)
        rospy.loginfo("=" * 64)

    # ─── Callbacks ───────────────────────────────────────────────
    def _pose_cb(self, msg):
        first = self.pose_xy is None
        self.pose_xy = (float(msg.position.x), float(msg.position.y))
        if first:
            rospy.loginfo("astar_planner: first pose  start=(%.2f,%.2f)",
                          *self.pose_xy)

    def _goal_cb(self, msg):
        new = (float(msg.x), float(msg.y))
        # Always log receipt so a click is visible in the log even if
        # the goal didn't change or planning later fails. Use a small
        # epsilon instead of exact float equality.
        same = (self.goal_xy is not None
                and abs(new[0] - self.goal_xy[0]) < 1e-3
                and abs(new[1] - self.goal_xy[1]) < 1e-3)
        rospy.loginfo("astar_planner: GOAL RECEIVED (%.2f, %.2f)%s",
                      new[0], new[1], "  (== current goal)" if same else "")
        # A new click is an explicit user intent: replan even if the
        # numbers match (the world may have changed) and force-clear
        # the plan + cost cache so the next BEV is treated as fresh.
        self.goal_xy = new
        self.has_plan = False
        self._cost_cache_for = None
        ok = self._try_plan()
        if not ok:
            rospy.logwarn("astar_planner: click goal (%.2f, %.2f) accepted "
                          "but no path yet — reason=%s "
                          "(will retry on next BEV)",
                          new[0], new[1], self.fail_reason)

    def _bev_cb(self, msg):
        first = self.bev is None
        self.bev = msg
        # Invalidate the cost cache; the bev object identity changed.
        self._cost_cache_for = None
        if first:
            i = msg.info
            rospy.loginfo("astar_planner: first BEV  W=%d H=%d res=%.2f  "
                          "origin=(%.1f,%.1f)",
                          i.width, i.height, i.resolution,
                          i.origin.position.x, i.origin.position.y)
        if not self.has_plan:
            self._try_plan()
        elif self.replan_on_bev:
            self._try_plan()
        elif self.replan_on_collision and self._path_collides():
            rospy.logwarn("astar_planner: published path now crosses an "
                          "occupied cell — replanning")
            self.has_plan = False
            self._try_plan()

    # ─── Cost map (cached per BEV) ────────────────────────────────
    def _build_cost(self):
        if self._cost_cache_for is self.bev and self._cost_cache is not None:
            return self._cost_cache

        info = self.bev.info
        W, H, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        # Faster decode: rospy gives a Python tuple of ints; bytes() over
        # a tuple of small ints is faster than np.array(tuple, ...) for
        # large arrays. (Both are in C; bytes path skips the per-element
        # PyLong unboxing that np.array does.)
        try:
            buf = bytes(bytearray(self.bev.data))
            data = np.frombuffer(buf, dtype=np.int8).reshape(H, W)
        except Exception:
            data = np.array(self.bev.data, dtype=np.int8).reshape(H, W)

        occ = (data == 100)
        n = max(0, int(round(self.inflate_radius_m / res)))
        if n > 0:
            occ = self._dilate(occ, n)
        cost = np.full((H, W), 1.0, dtype=np.float32)
        cost[occ] = np.inf
        unk = (data == -1) & ~occ
        cost[unk] = (np.inf if self.unknown_blocked
                     else float(self.unknown_cost))

        out = (cost, occ, (W, H, res, ox, oy))
        self._cost_cache_for = self.bev
        self._cost_cache     = out
        return out

    # ─── Planning ────────────────────────────────────────────────
    def _try_plan(self):
        if self.bev is None:
            self.fail_reason = "no BEV yet"; return False
        if self.goal_xy is None:
            self.fail_reason = "no goal set"; return False
        if self.pose_xy is None:
            self.fail_reason = "no pose yet"; return False

        # ── Map-warmup gate ──────────────────────────────────────
        # Count genuine FREE cells in the current BEV. Until the map
        # has warmed up we refuse to plan, so the follower stays in
        # WAIT_PATH and the drone holds position instead of cruising
        # through an all-unknown (== looks-free) map.
        if not self._warmed_up:
            try:
                buf = bytes(bytearray(self.bev.data))
                d = np.frombuffer(buf, dtype=np.int8)
            except Exception:
                d = np.array(self.bev.data, dtype=np.int8)
            n_free = int((d == 0).sum())
            if n_free < self.min_free_cells:
                self.fail_reason = ("map warming up: %d/%d FREE cells"
                                    % (n_free, self.min_free_cells))
                rospy.loginfo_throttle(
                    2.0, "astar_planner: %s — holding (no path "
                    "published yet)", self.fail_reason)
                return False
            self._warmed_up = True
            rospy.loginfo("astar_planner: map warmed up "
                          "(%d FREE cells \u2265 %d) — planning enabled",
                          n_free, self.min_free_cells)

        t0 = rospy.Time.now()
        result = self._plan(self.pose_xy, self.goal_xy)
        if isinstance(result, str):
            self.fail_reason = result
            rospy.logwarn_throttle(5.0,
                "astar_planner: PLAN FAILED  start=(%.2f,%.2f) "
                "goal=(%.2f,%.2f)  reason=%s",
                self.pose_xy[0], self.pose_xy[1],
                self.goal_xy[0], self.goal_xy[1], result)
            return False
        cells, world_pts = result
        self.last_cells = cells
        self._publish(world_pts, (rospy.Time.now() - t0).to_sec())
        self.has_plan = True
        self.fail_reason = "(success)"
        return True

    def _plan(self, start_xy, goal_xy):
        cost, occ, (W, H, res, ox, oy) = self._build_cost()
        def w2c(x, y): return int((x - ox) / res), int((y - oy) / res)
        def c2w(cx, cy): return ox + (cx + 0.5) * res, oy + (cy + 0.5) * res

        sx, sy = w2c(*start_xy)
        gx, gy = w2c(*goal_xy)
        if not (0 <= sx < W and 0 <= sy < H):
            return "start cell (%d,%d) outside BEV %dx%d" % (sx, sy, W, H)
        if not (0 <= gx < W and 0 <= gy < H):
            return "goal cell (%d,%d) outside BEV %dx%d" % (gx, gy, W, H)

        if not math.isfinite(cost[gy, gx]):
            sn = self._snap(cost, gx, gy, int(self.snap_radius_m / res))
            if sn is None:
                return ("goal blocked, no free cell within %.1fm"
                        % self.snap_radius_m)
            gx, gy = sn

        cost[sy, sx] = 1.0   # start always passable

        # Bounding-box search domain. Cells outside the box are not
        # expanded. This is the single biggest A* speed win.
        margin = max(1, int(round(self.search_margin_m / res)))
        xmin = max(0, min(sx, gx) - margin)
        xmax = min(W, max(sx, gx) + margin + 1)
        ymin = max(0, min(sy, gy) - margin)
        ymax = min(H, max(sy, gy) + margin + 1)

        cells = self._astar(cost, (sx, sy), (gx, gy),
                            (xmin, xmax, ymin, ymax),
                            self.turn_penalty)
        if cells is None:
            return "A* unreachable through current cost map"

        if self.los_smoothing and len(cells) > 2:
            cells = self._los_smooth(cells, occ)

        # Corner-preserving resample: keep every LOS corner, only split
        # segments longer than `waypoint_spacing_m`.
        world_pts = self._split_long(
            [c2w(cx, cy) for (cx, cy) in cells],
            self.waypoint_spacing_m)

        # Drop leading waypoints that are within `start_skip_m` of the
        # actual drone pose. Prevents the follower from yawing toward a
        # point that is effectively where it already is.
        sxw, syw = start_xy
        while (len(world_pts) > 1
               and math.hypot(world_pts[0][0] - sxw,
                              world_pts[0][1] - syw) < self.start_skip_m):
            world_pts.pop(0)

        return cells, world_pts

    # ─── A* (bbox-restricted, octile h, optional turn penalty) ────
    @staticmethod
    def _astar(cost, start, goal, bbox, turn_penalty=0.0):
        H, W = cost.shape
        sx, sy = start; gx, gy = goal
        xmin, xmax, ymin, ymax = bbox

        # Octile heuristic: tightest admissible h for 8-connected moves
        # with (1.0, sqrt(2)) step costs. Reduces expansions vs Euclidean.
        def h(x, y):
            dx = abs(x - gx); dy = abs(y - gy)
            return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)

        N = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, SQRT2), (-1, 1, SQRT2),
             (1, -1, SQRT2),  (1, 1, SQRT2))

        g = np.full((H, W), np.inf, dtype=np.float32)
        g[sy, sx] = 0.0
        closed = np.zeros((H, W), dtype=bool)
        came = {}
        pq = [(h(sx, sy), 0.0, sx, sy)]
        while pq:
            _f, gc, x, y = heapq.heappop(pq)
            if closed[y, x]:
                continue
            closed[y, x] = True
            if (x, y) == (gx, gy):
                path = [(x, y)]
                while (x, y) in came:
                    x, y = came[(x, y)]
                    path.append((x, y))
                return path[::-1]

            prev = None
            if turn_penalty > 0.0 and (x, y) in came:
                px, py = came[(x, y)]
                prev = (x - px, y - py)
            for dx, dy, step in N:
                nx, ny = x + dx, y + dy
                # Bbox + grid bounds in one check
                if not (xmin <= nx < xmax and ymin <= ny < ymax):
                    continue
                if closed[ny, nx]:
                    continue
                c = cost[ny, nx]
                if not math.isfinite(c):
                    continue
                turn = (turn_penalty if (prev is not None
                                         and prev != (dx, dy)) else 0.0)
                ng = gc + step * c + turn
                if ng < g[ny, nx]:
                    g[ny, nx] = ng
                    came[(nx, ny)] = (x, y)
                    heapq.heappush(pq, (ng + h(nx, ny), ng, nx, ny))
        return None

    # ─── LOS smoothing (any-angle post-pass) ────────────────────
    @staticmethod
    def _line_clear(occ, x0, y0, x1, y1):
        """Bresenham line; True if no cell along the line is occupied."""
        dx = abs(x1 - x0); dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if occ[y, x]:
                return False
            if x == x1 and y == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy; x += sx
            if e2 < dx:
                err += dx; y += sy

    @classmethod
    def _los_smooth(cls, cells, occ):
        """Greedy: from each kept cell, jump to the farthest visible cell.
        Removes A*'s staircase, leaving only the necessary corners."""
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1:
                if cls._line_clear(occ, cells[i][0], cells[i][1],
                                        cells[j][0], cells[j][1]):
                    break
                j -= 1
            out.append(cells[j])
            i = j
        return out

    # ─── Path collision check (for lazy replanning) ─────────────
    def _path_collides(self):
        if len(self.last_cells) < 2:
            return False
        _, occ, _ = self._build_cost()   # cached on same BEV
        H, W = occ.shape
        for (x0, y0), (x1, y1) in zip(self.last_cells[:-1],
                                       self.last_cells[1:]):
            if not (0 <= x0 < W and 0 <= y0 < H
                    and 0 <= x1 < W and 0 <= y1 < H):
                continue
            if not self._line_clear(occ, x0, y0, x1, y1):
                return True
        return False

    # ─── Helpers ─────────────────────────────────────────────────
    @staticmethod
    def _snap(cost, x, y, max_r):
        H, W = cost.shape
        for r in range(1, max_r + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r: continue
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < W and 0 <= ny < H
                            and math.isfinite(cost[ny, nx])):
                        return nx, ny
        return None

    @staticmethod
    def _dilate(mask, iters):
        m = mask.copy()
        for _ in range(iters):
            o = m.copy()
            o[1:, :]  |= m[:-1, :]
            o[:-1, :] |= m[1:, :]
            o[:, 1:]  |= m[:, :-1]
            o[:, :-1] |= m[:, 1:]
            m = o
        return m

    @staticmethod
    def _split_long(pts, max_seg):
        """Keep every input vertex (the LOS corners) exactly. Insert
        evenly-spaced intermediate points only where a segment exceeds
        `max_seg`. Output starts at pts[0] and ends at pts[-1].

        Behaviour:
          start → corner_a (3.2 m, max_seg=3.0) → corner_b (8.5 m) → goal
        becomes
          start, mid, corner_a, mid1, mid2, corner_b, mid3, goal
        i.e. the corner_b → goal leg gets split into 3 sub-legs of
        ~2.83 m (≤ 3.0), but corner_a and corner_b are kept.
        """
        if len(pts) < 2 or max_seg <= 0:
            return list(pts)
        out = [pts[0]]
        for i in range(1, len(pts)):
            ax, ay = pts[i - 1]
            bx, by = pts[i]
            d = math.hypot(bx - ax, by - ay)
            if d <= max_seg:
                out.append((bx, by))
                continue
            n = int(math.ceil(d / max_seg))   # n sub-segments → n−1 inserts
            for k in range(1, n):
                t = k / n
                out.append((ax + t * (bx - ax), ay + t * (by - ay)))
            out.append((bx, by))
        return out

    def _publish(self, pts, plan_dt_s):
        m = Path()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self.frame_id
        for x, y in pts:
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            m.poses.append(ps)
        self.pub_path.publish(m)
        L = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                for a, b in zip(pts[:-1], pts[1:])) if len(pts) >= 2 else 0.0
        rospy.loginfo("astar_planner: PATH PUBLISHED  %d wp  %.2fm  "
                      "plan=%.0fms  first=(%.2f,%.2f) last=(%.2f,%.2f)",
                      len(pts), L, 1000.0 * plan_dt_s,
                      pts[0][0], pts[0][1], pts[-1][0], pts[-1][1])

    def _status(self, _e):
        if self.has_plan: return
        rospy.loginfo("astar_planner waiting:  bev=%s  pose=%s  goal=%s  "
                      "last_reason=%s",
                      "yes" if self.bev is not None else "NO",
                      "yes" if self.pose_xy is not None else "NO",
                      ("(%.2f,%.2f)" % self.goal_xy)
                          if self.goal_xy else "NO",
                      self.fail_reason)


if __name__ == "__main__":
    try:
        AStarPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass