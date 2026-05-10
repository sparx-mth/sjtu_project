#!/usr/bin/env python3
"""
astar_planner.py — A* on the 2D BEV → smoothed, evenly-spaced waypoints.

Inputs
  /falcon/bev_2d           (nav_msgs/OccupancyGrid, latched)
  /<drone_ns>/gt_pose      (geometry_msgs/Pose)         — start
  /waypoint_nav/goal       (geometry_msgs/Point)        — runtime goal

Output
  /path/waypoints          (nav_msgs/Path, latched)

v3 — straight-line bias and lazy collision replanning
  • unknown_cost=1.0 by default — unknown cells planned through as if
    free. As we explore and a cell turns occupied, we replan.
  • Line-of-sight (LOS) smoothing post-pass collapses A*'s grid
    staircase into long straight segments. Drone yaws only at the
    actual corners, then advances. (Equivalent to "any-angle" planning.)
  • Lazy collision replan: on every BEV update, walk the published
    path's segments and re-check against the new occupied mask.
    Replan only if a segment now crosses occupancy. No oscillation,
    no wasted work when the world is consistent with the plan.
  • Optional turn_penalty (>0) adds a small extra cost to direction
    changes during A* itself. Off by default — LOS smoothing already
    handles it. Turn it on if you want even fewer initial branches.
"""
import heapq
import math
import numpy as np
import rospy

from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path


class AStarPlanner:
    def __init__(self):
        rospy.init_node("astar_planner")
        G = rospy.get_param

        self.drone_ns           = G("~drone_ns",        "/simple_drone")
        self.bev_topic          = G("~bev_topic",       "/falcon/bev_2d")
        self.path_topic         = G("~path_topic",      "/path/waypoints")
        self.goal_topic         = G("~goal_topic",      "/waypoint_nav/goal")
        self.frame_id           = G("~frame_id",        "world")
        self.waypoint_spacing_m = float(G("~waypoint_spacing_m", 1.0))
        self.inflate_radius_m   = float(G("~inflate_radius_m",   0.4))
        self.unknown_blocked    = bool (G("~unknown_blocked",    False))
        self.unknown_cost       = float(G("~unknown_cost",       1.0))   # unknown=free
        self.los_smoothing      = bool (G("~los_smoothing",      True))
        self.turn_penalty       = float(G("~turn_penalty",       0.0))
        self.replan_on_collision = bool(G("~replan_on_collision", True))
        self.replan_on_bev      = bool (G("~replan_on_bev",      False))
        self.replan_period_s    = float(G("~replan_period_s",    0.0))
        self.snap_radius_m      = float(G("~goal_snap_radius_m", 2.0))

        gx = G("~goal_x", None); gy = G("~goal_y", None)
        self.goal_xy = ((float(gx), float(gy))
                        if gx is not None and gy is not None else None)

        self.pose_xy   = None
        self.bev       = None
        self.has_plan  = False
        self.fail_reason = "(not tried yet)"
        self.last_cells = []   # smoothed cell-coord path; kept for collision recheck

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
        rospy.loginfo("astar_planner ready")
        rospy.loginfo("  bev   in  = %s", self.bev_topic)
        rospy.loginfo("  pose  in  = %s/gt_pose", self.drone_ns)
        rospy.loginfo("  goal  in  = %s", self.goal_topic)
        rospy.loginfo("  path  out = %s", self.path_topic)
        rospy.loginfo("  goal init = %s",
                      "(%.2f,%.2f)" % self.goal_xy if self.goal_xy else "none")
        rospy.loginfo("  spacing=%.2fm  inflate=%.2fm  unknown=%s  "
                      "los_smooth=%s  turn_pen=%.2f  collision_replan=%s",
                      self.waypoint_spacing_m, self.inflate_radius_m,
                      "blocked" if self.unknown_blocked
                                else "free×%.1f" % self.unknown_cost,
                      self.los_smoothing, self.turn_penalty,
                      self.replan_on_collision)
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
        if new != self.goal_xy:
            rospy.loginfo("astar_planner: goal → (%.2f, %.2f)", *new)
            self.goal_xy = new
            self.has_plan = False
            self._try_plan()

    def _bev_cb(self, msg):
        first = self.bev is None
        self.bev = msg
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

    # ─── Cost map construction (used by plan AND collision check) ──
    def _build_cost(self):
        info = self.bev.info
        W, H, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
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
        return cost, occ, (W, H, res, ox, oy)

    # ─── Planning ────────────────────────────────────────────────
    def _try_plan(self):
        if self.bev is None:
            self.fail_reason = "no BEV yet"; return
        if self.goal_xy is None:
            self.fail_reason = "no goal set"; return
        if self.pose_xy is None:
            self.fail_reason = "no pose yet"; return
        result = self._plan(self.pose_xy, self.goal_xy)
        if isinstance(result, str):
            self.fail_reason = result
            rospy.logwarn_throttle(5.0,
                "astar_planner: PLAN FAILED  start=(%.2f,%.2f) "
                "goal=(%.2f,%.2f)  reason=%s",
                self.pose_xy[0], self.pose_xy[1],
                self.goal_xy[0], self.goal_xy[1], result)
            return
        cells, world_pts = result
        self.last_cells = cells
        self._publish(world_pts)
        self.has_plan = True
        self.fail_reason = "(success)"

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

        cells = self._astar(cost, (sx, sy), (gx, gy), self.turn_penalty)
        if cells is None:
            return "A* unreachable through current cost map"

        if self.los_smoothing and len(cells) > 2:
            cells = self._los_smooth(cells, occ)

        world_pts = self._resample([c2w(cx, cy) for (cx, cy) in cells],
                                   self.waypoint_spacing_m)
        return cells, world_pts

    # ─── A* (with optional turn penalty) ────────────────────────
    @staticmethod
    def _astar(cost, start, goal, turn_penalty=0.0):
        H, W = cost.shape
        sx, sy = start; gx, gy = goal
        def h(x, y): return math.hypot(x - gx, y - gy)
        N = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.4142), (-1, 1, 1.4142),
             (1, -1, 1.4142),  (1, 1, 1.4142))
        g = np.full((H, W), np.inf, dtype=np.float32)
        g[sy, sx] = 0.0
        came = {}
        pq = [(h(sx, sy), 0.0, sx, sy)]
        while pq:
            _f, gc, x, y = heapq.heappop(pq)
            if (x, y) == (gx, gy):
                path = [(x, y)]
                while (x, y) in came:
                    x, y = came[(x, y)]
                    path.append((x, y))
                return path[::-1]
            if gc > g[y, x]:
                continue
            # Direction we entered (x,y) from, if any — used for turn cost
            prev = None
            if turn_penalty > 0.0 and (x, y) in came:
                px, py = came[(x, y)]
                prev = (x - px, y - py)
            for dx, dy, step in N:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < W and 0 <= ny < H):
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
        _, occ, _ = self._build_cost()
        H, W = occ.shape
        for (x0, y0), (x1, y1) in zip(self.last_cells[:-1],
                                       self.last_cells[1:]):
            if not (0 <= x0 < W and 0 <= y0 < H
                    and 0 <= x1 < W and 0 <= y1 < H):
                continue   # out-of-grid points: don't trigger replan
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
    def _resample(pts, spacing):
        if len(pts) <= 1: return list(pts)
        out = [pts[0]]; last = pts[0]; accum = 0.0
        for cur in pts[1:]:
            seg = math.hypot(cur[0] - last[0], cur[1] - last[1])
            while seg > 0 and accum + seg >= spacing:
                t = (spacing - accum) / seg
                npt = (last[0] + t * (cur[0] - last[0]),
                       last[1] + t * (cur[1] - last[1]))
                out.append(npt); last = npt
                seg = math.hypot(cur[0] - last[0], cur[1] - last[1])
                accum = 0.0
            accum += seg; last = cur
        if math.hypot(out[-1][0] - pts[-1][0],
                      out[-1][1] - pts[-1][1]) > 1e-3:
            out.append(pts[-1])
        return out

    def _publish(self, pts):
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
                for a, b in zip(pts[:-1], pts[1:]))
        rospy.loginfo("astar_planner: PATH PUBLISHED  %d waypoints, %.2fm  "
                      "first=(%.2f,%.2f) last=(%.2f,%.2f)",
                      len(pts), L, pts[0][0], pts[0][1],
                      pts[-1][0], pts[-1][1])

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