#!/usr/bin/env python3
"""
target_watcher_node.py — watches confirmed objects, fires /target_seen on match.

Subscribes
----------
/perception/objects  (std_msgs/String, JSON from object_mapper_node)
    Objects already filtered to count >= min_observations.

Publishes
---------
/target_seen           (std_msgs/Bool, latched TRANSIENT_LOCAL)
    False at startup. Flips to True permanently when a confirmed
    object matches the target. The LLM oracle and room classifier
    subscribe to this and stop ticking when it's True.
/target_seen/info      (std_msgs/String, JSON, latched)
    Emitted once when /target_seen flips. Carries target, matched
    class, object id, world XY, count, and LLM reason.
/simple_drone/cmd_vel  (geometry_msgs/Twist)
    Burst of zero-Twist messages to halt the drone. Best-effort — it
    races FALCON's outbound trajectory stream. Canonical signal is
    still /target_seen; halt_duration_s=0 disables the burst.

Fuzzy match
-----------
The YOLO class name and the natural-language target rarely match
character-for-character ("car keys" vs "key", "toilet seat" vs
"toilet"). We ask the LLM "does class X match target Y?" and cache
the answer by (target, class). Same-class subsequent detections
hit the cache and never re-query the LLM. If the LLM is unreachable
we fall back to lowercased substring match, so the watcher still
fires on exact names even offline.
"""

import json
from typing import Dict, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

from semantic_mapper.llm_client import LLMClient


MATCH_SYSTEM = """You decide whether a detected object should count as a hit \
for a robot's search target.

Inputs:
  TARGET: the object the user asked the robot to find.
  CLASS:  a word or short phrase from the object detector's vocabulary.

Answer TRUE if a real-world object detected as CLASS would reasonably be \
accepted as the TARGET. Answer FALSE otherwise.

Guidelines:
- Same word, same meaning -> TRUE. (target="toilet", class="toilet" -> TRUE.)
- CLASS is a more specific kind of TARGET -> TRUE. \
(target="keys", class="car key" -> TRUE.)
- CLASS and TARGET are synonyms or near-synonyms -> TRUE. \
(target="couch", class="sofa" -> TRUE; target="mug", class="cup" -> TRUE.)
- TARGET names a specific thing and CLASS is the usual label for its \
main object -> TRUE. (target="toilet seat", class="toilet" -> TRUE; \
target="car keys", class="key" -> TRUE.)
- CLASS is a broader category, a different object, or only an accessory \
of TARGET -> FALSE. (target="apple", class="fruit" -> FALSE; \
target="car keys", class="car" -> FALSE; \
target="laptop", class="monitor" -> FALSE.)
- CLASS is a person or animal and TARGET isn't -> FALSE.

Reply ONLY with a JSON object:
{"match": <true|false>, "reason": "<one short sentence>"}"""


MATCH_USER_TEMPLATE = """TARGET: {target!r}
CLASS:  {cname!r}

Match?"""


def _fallback_match(target: str, cname: str) -> bool:
    """Last-resort match when the LLM is unreachable."""
    t = target.strip().lower()
    c = cname.strip().lower()
    if not t or not c:
        return False
    if t == c:
        return True
    # Token overlap (handles "car keys" <-> "key"/"keys").
    t_tokens = set(t.replace("_", " ").split())
    c_tokens = set(c.replace("_", " ").split())
    if t_tokens & c_tokens:
        return True
    return t in c or c in t


class TargetWatcherNode(Node):
    def __init__(self):
        super().__init__("target_watcher")

        P = self.declare_parameter
        P("target_object",      "car keys")
        P("objects_topic",      "/perception/objects")
        P("drone_cmd_topic",    "/simple_drone/cmd_vel")
        P("halt_duration_s",    3.0)
        P("halt_rate_hz",       50.0)     # must exceed FALCON's traj rate
        P("use_llm",            True)

        g = lambda n: self.get_parameter(n).value
        self.target        = str(g("target_object"))
        self.objects_topic = str(g("objects_topic"))
        self.cmd_topic     = str(g("drone_cmd_topic"))
        self.halt_dur      = float(g("halt_duration_s"))
        self.halt_rate     = max(1.0, float(g("halt_rate_hz")))
        self.use_llm       = bool(g("use_llm"))

        self.add_on_set_parameters_callback(self._on_param_set)

        # LLM — optional. Watcher must work without one.
        self.llm: Optional[LLMClient] = None
        if self.use_llm:
            self.llm = LLMClient.from_env()
            if not self.llm.ping():
                self.get_logger().warn(
                    f"LLM at {self.llm.cfg.base_url} unreachable; "
                    f"will fall back to token/substring match.")

        # State.
        self._seen: bool = False
        self._match_cache: Dict[Tuple[str, str], bool] = {}
        self._checked_obj_ids: Set[int] = set()
        self._halt_timer = None
        self._halt_remaining = 0

        # QoS.
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        # Pubs.
        self.pub_seen = self.create_publisher(Bool, "/target_seen", latched)
        self.pub_info = self.create_publisher(
            String, "/target_seen/info", latched)
        self.pub_cmd  = self.create_publisher(Twist, self.cmd_topic, 10)

        # Initial "not seen" message so late subscribers get deterministic state.
        self.pub_seen.publish(Bool(data=False))

        # Subs.
        self.create_subscription(String, self.objects_topic,
                                 self._objects_cb, latched)

        self.create_timer(10.0, self._hb)
        self._n = dict(objs_seen=0, checks=0, llm_calls=0,
                       cache_hits=0, fb_hits=0)

        self.get_logger().info(
            f"target_watcher ready  target={self.target!r}  "
            f"llm={'on' if self.use_llm else 'off'}  "
            f"halt={self.halt_dur:.1f}s @ {self.halt_rate:.0f}Hz")

    # ── Runtime param changes ─────────────────────────────────────
    def _on_param_set(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "target_object":
                new_t = str(p.value)
                if new_t != self.target:
                    self.get_logger().info(
                        f"target_object: {self.target!r} -> {new_t!r}  "
                        f"(re-checking objects; not un-seeing)")
                    self.target = new_t
                    # Allow re-checking existing objects against the
                    # new target. The seen latch does NOT reset —
                    # once latched, stays latched for this process.
                    self._checked_obj_ids.clear()
        return SetParametersResult(successful=True)

    # ── Objects subscription ──────────────────────────────────────
    def _objects_cb(self, msg: String):
        if self._seen:
            return    # idempotent — first match wins
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        objs = data.get("objects", []) or []
        self._n["objs_seen"] = len(objs)
        for o in objs:
            try:
                oid   = int(o.get("id", -1))
                cname = str(o.get("class", "")).strip().lower()
            except (TypeError, ValueError):
                continue
            if oid < 0 or not cname:
                continue
            if oid in self._checked_obj_ids:
                continue
            self._checked_obj_ids.add(oid)
            self._n["checks"] += 1

            if self._is_match(self.target, cname):
                self._on_match(o)
                return

    # ── Matching ──────────────────────────────────────────────────
    def _is_match(self, target: str, cname: str) -> bool:
        t = target.strip().lower()
        c = cname.strip().lower()

        # Exact match is an exact match. Never ask the LLM about this —
        # it's wasteful, and small models occasionally get it wrong
        # (observed: llama3.2:3b replying False for target='toilet'
        # vs class='toilet').
        if t == c:
            self.get_logger().info(
                f"match check  target={target!r}  class={cname!r}  "
                f"→ True   (exact match)")
            self._match_cache[(t, c)] = True
            return True

        key = (t, c)
        if key in self._match_cache:
            self._n["cache_hits"] += 1
            return self._match_cache[key]

        if self.llm is not None:
            verdict = self._ask_llm_match(target, cname)
            if verdict is not None:
                self._match_cache[key] = verdict
                return verdict
            # LLM error → fall through to fallback.

        verdict = _fallback_match(target, cname)
        self._match_cache[key] = verdict
        self._n["fb_hits"] += 1
        return verdict

    def _ask_llm_match(self, target: str, cname: str) -> Optional[bool]:
        user = MATCH_USER_TEMPLATE.format(target=target, cname=cname)
        try:
            self._n["llm_calls"] += 1
            reply = self.llm.chat_json(MATCH_SYSTEM, user)
        except Exception as e:
            self.get_logger().warn(f"LLM match failed ({cname!r}): {e}",
                                   throttle_duration_sec=5.0)
            return None
        verdict = bool(reply.get("match", False))
        reason = str(reply.get("reason", ""))[:160]
        self.get_logger().info(
            f"match check  target={target!r}  class={cname!r}  "
            f"→ {verdict}   ({reason})")
        # Stash the reason for later /target_seen/info payload use.
        self._match_cache_reasons = getattr(self, '_match_cache_reasons', {})
        self._match_cache_reasons[(target.strip().lower(), cname)] = reason
        return verdict

    # ── Match handler ─────────────────────────────────────────────
    def _on_match(self, obj: dict):
        self._seen = True

        oid   = int(obj.get("id", -1))
        cname = str(obj.get("class", "")).strip().lower()
        xy    = obj.get("xy", [0.0, 0.0])
        count = int(obj.get("count", 0))
        reason = getattr(self, '_match_cache_reasons', {}).get(
            (self.target.strip().lower(), cname), "")

        bar = "=" * 64
        self.get_logger().info(bar)
        self.get_logger().info(
            f"  TARGET FOUND   target={self.target!r}  "
            f"matched class={cname!r}  obj_id={oid}")
        self.get_logger().info(
            f"  world XY = ({float(xy[0]):.2f}, {float(xy[1]):.2f})   "
            f"confirmations={count}")
        if reason:
            self.get_logger().info(f"  LLM reason: {reason}")
        self.get_logger().info(
            f"  -> publishing /target_seen=True and halting drone "
            f"for {self.halt_dur:.1f}s")
        self.get_logger().info(bar)

        # Canonical latched signal.
        self.pub_seen.publish(Bool(data=True))
        info = {
            "stamp":         self.get_clock().now().nanoseconds * 1e-9,
            "target":        self.target,
            "matched_class": cname,
            "object_id":     oid,
            "xy":            [float(xy[0]), float(xy[1])],
            "count":         count,
            "reason":        reason,
        }
        self.pub_info.publish(String(data=json.dumps(info)))

        # Halt burst.
        if self.halt_dur > 0.0:
            self._halt_remaining = int(round(self.halt_dur * self.halt_rate))
            period = 1.0 / self.halt_rate
            self._halt_timer = self.create_timer(period, self._halt_tick)

    def _halt_tick(self):
        # Publish zero Twist until the budget runs out, then cancel.
        self.pub_cmd.publish(Twist())
        self._halt_remaining -= 1
        if self._halt_remaining <= 0 and self._halt_timer is not None:
            self._halt_timer.cancel()
            self._halt_timer = None
            self.get_logger().info(
                "halt burst finished; drone should be stopped. "
                "(Ctrl-C FALCON if you want the full pipeline to exit.)")

    # ── Heartbeat ─────────────────────────────────────────────────
    def _hb(self):
        status = "SEEN" if self._seen else "watching"
        self.get_logger().info(
            f"target_watcher hb  {status}  target={self.target!r}  "
            f"objs={self._n['objs_seen']} checks={self._n['checks']} "
            f"llm={self._n['llm_calls']} cache={self._n['cache_hits']} "
            f"fallback={self._n['fb_hits']}  cache_size={len(self._match_cache)}")
        self._n = dict(objs_seen=self._n['objs_seen'], checks=0,
                       llm_calls=0, cache_hits=0, fb_hits=0)


def main():
    rclpy.init()
    node = TargetWatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()