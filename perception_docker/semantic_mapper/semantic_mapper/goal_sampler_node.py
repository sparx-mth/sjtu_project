#!/usr/bin/env python3
"""
goal_sampler_node.py — Sample a room from the LLM oracle's distribution
and publish its centroid as the next planning goal.

State machine:
  IDLE      → weighted-sample one room from /llm_oracle/probabilities,
              publish its centroid to /move_base_simple/goal,
              transition to PURSUING.
  PURSUING  → wait for one of:
                * drone within `arrival_tol_m` of goal       → DWELL
                * planner produced no Path within plan_grace_s → IDLE
                  (centroid unreachable; re-sample immediately)
                * `max_pursue_s` elapsed                     → IDLE
                  (stuck; re-sample immediately)
  DWELL     → publish nothing for `dwell_after_arrival_s` seconds so
              FALCON can take over and explore the room. After the
              timer expires, transition to IDLE and re-sample.
  DONE      → /target_seen=True ; sampler sits silent.

Inputs
------
  /llm_oracle/probabilities  std_msgs/String   (JSON, see llm_oracle_node)
  /scene_graph               std_msgs/String   (JSON; provides centroids)
  /odom_world                nav_msgs/Odometry
  /target_seen               std_msgs/Bool     (latched)
  /planned_path              nav_msgs/Path     (only used to confirm the
                                                planner accepted the goal)

Output
------
  /move_base_simple/goal     geometry_msgs/PoseStamped
  /goal_sampler/info         std_msgs/String   (JSON debug payload)
"""

import json
import math
import random
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class GoalSamplerNode(Node):
    def __init__(self):
        super().__init__('goal_sampler')

        # ── Params ──
        P = self.declare_parameter
        P('goal_topic',             '/move_base_simple/goal')
        P('frame_id',               'world')
        P('tick_hz',                1.0)
        P('arrival_tol_m',          0.6)
        P('plan_grace_s',           5.0)
        P('max_pursue_s',           60.0)
        P('dwell_after_arrival_s',  15.0)   # FALCON exploration window
        P('min_prob',               0.01)
        P('seed',                   -1)

        gp = lambda k: self.get_parameter(k).value
        self.goal_topic   = str(gp('goal_topic'))
        self.frame_id     = str(gp('frame_id'))
        self.tick_dt      = 1.0 / max(0.1, float(gp('tick_hz')))
        self.arrival_tol  = float(gp('arrival_tol_m'))
        self.plan_grace   = float(gp('plan_grace_s'))
        self.max_pursue_s = float(gp('max_pursue_s'))
        self.dwell_s      = float(gp('dwell_after_arrival_s'))
        self.min_prob     = float(gp('min_prob'))
        seed = int(gp('seed'))
        self.rng = random.Random(seed if seed >= 0 else None)

        # ── State ──
        self._probs: Optional[List[Dict]] = None
        self._centroids: Dict[int, Tuple[float, float]] = {}
        self._cur_xyz: Optional[Tuple[float, float, float]] = None
        self._target_seen = False
        self._last_path_t: Optional[float] = None

        self._state = 'IDLE'                     # IDLE | PURSUING | DWELL
        self._goal_xy: Optional[Tuple[float, float]] = None
        self._goal_room_id: Optional[int] = None
        self._goal_t: Optional[float] = None
        self._path_t_at_goal: Optional[float] = None
        self._dwell_end_t: Optional[float] = None

        # ── ROS ──
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 1)
        self.info_pub = self.create_publisher(String, '/goal_sampler/info', latched)

        self.create_subscription(String,   '/llm_oracle/probabilities',
                                 self._probs_cb, latched)
        self.create_subscription(String,   '/scene_graph',
                                 self._sg_cb,    latched)
        self.create_subscription(Odometry, '/odom_world',
                                 self._odom_cb,  10)
        self.create_subscription(Bool,     '/target_seen',
                                 self._seen_cb,  latched)
        self.create_subscription(Path,     '/planned_path',
                                 self._path_cb,  1)

        self.create_timer(self.tick_dt, self._tick)
        self.create_timer(10.0, self._hb)
        self._stats = dict(samples=0, arrivals=0, plan_fails=0,
                           timeouts=0, dwell_completes=0)

        self.get_logger().info(
            f"goal_sampler ready  goal={self.goal_topic}  "
            f"arrival_tol={self.arrival_tol:.2f}m  "
            f"plan_grace={self.plan_grace:.0f}s  "
            f"max_pursue={self.max_pursue_s:.0f}s  "
            f"dwell={self.dwell_s:.1f}s")

    # ── Callbacks ──

    def _probs_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        rooms = d.get('rooms')
        if isinstance(rooms, list) and rooms:
            self._probs = rooms

    def _sg_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        cents = {}
        for r in d.get('rooms', []) or []:
            try:
                rid = int(r['id'])
                cx, cy = r['centroid']
                cents[rid] = (float(cx), float(cy))
            except (KeyError, TypeError, ValueError):
                continue
        if cents:
            self._centroids = cents

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._cur_xyz = (p.x, p.y, p.z)

    def _seen_cb(self, msg: Bool):
        if msg.data and not self._target_seen:
            self.get_logger().info("target seen — goal_sampler stopping")
        self._target_seen = bool(msg.data)

    def _path_cb(self, _msg: Path):
        self._last_path_t = self._now()

    # ── Main tick ──

    def _tick(self):
        if self._target_seen or self._cur_xyz is None:
            return
        if self._state == 'IDLE':
            self._sample_and_publish()
        elif self._state == 'PURSUING':
            self._check_pursuit()
        elif self._state == 'DWELL':
            self._check_dwell()

    def _sample_and_publish(self):
        if not self._probs or not self._centroids:
            return

        cand = []
        for r in self._probs:
            try:
                rid = int(r['id']); p = float(r.get('prob', 0.0))
            except (KeyError, TypeError, ValueError):
                continue
            if p < self.min_prob or rid not in self._centroids:
                continue
            cx, cy = self._centroids[rid]
            cand.append((rid, p, cx, cy, str(r.get('label', '?'))))
        if not cand:
            return

        total = sum(c[1] for c in cand)
        weights = [c[1] / total for c in cand]
        idx = self._weighted_pick(weights)
        rid, p_orig, cx, cy, label = cand[idx]

        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = cx
        ps.pose.position.y = cy
        ps.pose.position.z = self._cur_xyz[2]
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)

        self._goal_xy        = (cx, cy)
        self._goal_room_id   = rid
        self._goal_t         = self._now()
        self._path_t_at_goal = self._last_path_t
        self._state = 'PURSUING'
        self._stats['samples'] += 1

        self.get_logger().info(
            f"sample R{rid} ({label})  prob={p_orig:.2f}  "
            f"goal=({cx:.2f},{cy:.2f})  "
            f"from {len(cand)} candidates")

        self.info_pub.publish(String(data=json.dumps({
            'stamp':   self._goal_t,
            'room_id': rid, 'label': label, 'prob': p_orig,
            'goal':    [cx, cy],
            'candidates': [{'id': c[0], 'prob_renorm': w, 'label': c[4]}
                           for c, w in zip(cand, weights)],
        })))

    def _check_pursuit(self):
        gx, gy = self._goal_xy
        cx, cy, _ = self._cur_xyz
        d = math.hypot(gx - cx, gy - cy)
        elapsed = self._now() - self._goal_t

        if d < self.arrival_tol:
            self._state = 'DWELL'
            self._dwell_end_t = self._now() + self.dwell_s
            self._stats['arrivals'] += 1
            self.get_logger().info(
                f"arrived R{self._goal_room_id}  d={d:.2f}m "
                f"after {elapsed:.1f}s — dwelling {self.dwell_s:.0f}s "
                f"for FALCON exploration")
            return

        new_path = (self._last_path_t is not None and
                    (self._path_t_at_goal is None or
                     self._last_path_t > self._path_t_at_goal))
        if elapsed > self.plan_grace and not new_path:
            self.get_logger().warn(
                f"R{self._goal_room_id}: no path in {self.plan_grace:.0f}s "
                f"(centroid likely blocked) — re-sampling")
            self._stats['plan_fails'] += 1
            self._reset()
            return

        if elapsed > self.max_pursue_s:
            self.get_logger().warn(
                f"R{self._goal_room_id}: pursue timeout "
                f"({self.max_pursue_s:.0f}s) — re-sampling")
            self._stats['timeouts'] += 1
            self._reset()

    def _check_dwell(self):
        if self._dwell_end_t is None or self._now() >= self._dwell_end_t:
            self._stats['dwell_completes'] += 1
            self.get_logger().info(
                f"dwell complete ({self.dwell_s:.0f}s) — re-sampling")
            self._reset()

    def _reset(self):
        self._state = 'IDLE'
        self._goal_xy = self._goal_room_id = self._goal_t = None
        self._path_t_at_goal = None
        self._dwell_end_t = None

    # ── Utils ──

    def _weighted_pick(self, weights: List[float]) -> int:
        r = self.rng.random(); cum = 0.0
        for i, w in enumerate(weights):
            cum += w
            if r <= cum:
                return i
        return len(weights) - 1

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _hb(self):
        s = self._stats
        extra = ""
        if self._state == 'DWELL' and self._dwell_end_t is not None:
            rem = max(0.0, self._dwell_end_t - self._now())
            extra = f"  dwell_left={rem:.1f}s"
        self.get_logger().info(
            f"sampler hb  state={self._state}{extra} "
            f"goal_room={self._goal_room_id}  "
            f"samples={s['samples']} arrivals={s['arrivals']} "
            f"dwell_done={s['dwell_completes']} "
            f"plan_fails={s['plan_fails']} timeouts={s['timeouts']}  "
            f"probs={self._probs is not None} centroids={len(self._centroids)}")
        self._stats = dict(samples=0, arrivals=0, plan_fails=0,
                           timeouts=0, dwell_completes=0)


def main():
    rclpy.init()
    node = GoalSamplerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()