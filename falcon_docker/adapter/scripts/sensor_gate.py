#!/usr/bin/env python3
"""
sensor_gate.py — Pose+depth pass-through that can be FROZEN.

Sits between Gazebo (or the ros1_bridge, or a real drone) and
falcon_adapter.

   <pose topic>         ──┐
   <depth topic>        ──┤   →  /<out_ns>/gt_pose
   <camera_info topic>  ──┘                    /<out_ns>/front_depth/depth/image_raw
                                               /<out_ns>/front_depth/depth/camera_info
                                               (camera_info path only when
                                                ~bridge_camera_info=true)

FREEZE LOGIC (mode-authoritative)
  Two inputs feed the freeze decision:

    1. /xtend/demo_mode (std_msgs/String) — the system-wide DemoMode.
       This is the AUTHORITATIVE signal for whether the system is
       actually in turning mode. When mode-based freezing is enabled
       (~freeze_on_turning_mode=true, the default) and we have received
       at least one message on this topic, the gate is frozen IFF
       msg.data == "turning".

    2. /sensor_gate/freeze (std_msgs/Bool) — waypoint_follower's
       per-state intent. Tracked for diagnostics, used as a FALLBACK
       only when mode-based freezing is disabled or no mode message
       has been received yet.

  Why mode-authoritative, not "OR" of the two inputs?
    Earlier the gate freezed whenever EITHER input said so. That fails
    in this case: waypoint_follower enters YAW_ALIGN, the mode handshake
    confirms "turning", waypoint_follower publishes /sensor_gate/freeze=
    True. Later the external state machine drops the mode back to
    "fly_straight" (timeout, external command, anything) BEFORE
    waypoint_follower has a chance to publish /sensor_gate/freeze=False.
    waypoint_follower then tries to start another rotation, its
    _ensure_mode(TURNING) call fails because the system mode is now
    "fly_straight", and its YAW_ALIGN handler `return`s early — never
    reaching _set_freeze(False). The stale True on /sensor_gate/freeze
    plus an "OR" gate kept the map frozen indefinitely while the drone
    was actually sitting still in fly_straight mode.

    Mode-authoritative removes this class of stuck state. The system
    mode reflects what the system is REALLY doing; if it says we're
    not turning, we trust that and unfreeze, regardless of whatever
    waypoint_follower last published. waypoint_follower's intent
    becomes redundant when the mode signal is present — which is the
    correct architectural relationship: the system mode is downstream
    of everyone's requests and is the only signal that knows what was
    actually granted.

  Mode-based freezing can be disabled with ~freeze_on_turning_mode:=
  false, in which case behaviour reverts exactly to the original
  "only /sensor_gate/freeze controls the gate" semantics.

DIAGNOSTICS
  The gate logs a heartbeat every ~heartbeat_period_sec showing the
  current source-of-truth and both raw inputs, so it is always obvious
  which signal is driving the freeze.

MANUAL RECOVERY
  If the mode topic gets stuck on "turning" (publisher died after
  publishing once, etc.), force-unfreeze with:

    rostopic pub --once /sensor_gate/reset_mode_freeze std_msgs/Bool \\
        "data: true"

  This clears _mode_says_freeze to False. The next genuine "turning"
  message re-sets it normally, so the reset is non-destructive.

While the gate is frozen:
  - new incoming msgs are NOT republished and the cache is NOT updated
  - a `replay_hz` timer republishes the LAST msg seen before the freeze,
    with header.stamp refreshed for stamped types so downstream age
    checks don't fail.

Point falcon_adapter at out_ns by setting falcon_adapter's
~drone_ns parameter to whatever this gate's `out_ns` is.
"""
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo


# Names for the three possible "who's driving the freeze right now"
# values. Pure documentation — used only in the heartbeat string.
SRC_MODE_AUTH       = "mode_auth"          # mode topic is authoritative
SRC_EXPLICIT_FBACK  = "explicit_fallback"  # mode topic enabled but no msgs yet
SRC_EXPLICIT_ONLY   = "explicit_only"      # mode-based freezing disabled


class SensorGate:
    def __init__(self):
        rospy.init_node("sensor_gate")
        G = rospy.get_param

        self.in_ns     = G("~in_ns",   "/simple_drone")
        self.out_ns    = G("~out_ns",  "/gated_drone")
        self.replay_hz = float(G("~replay_hz", 30.0))

        # When False (default), camera_info is neither subscribed nor
        # republished. Intrinsics come from rosparam instead.
        self.bridge_caminfo = bool(G("~bridge_camera_info", False))

        # Mode-based freezing. See module docstring for the semantics.
        self.freeze_on_turning_mode = bool(
            G("~freeze_on_turning_mode", True))
        self.demo_mode_topic   = G("~demo_mode_topic",   "/xtend/demo_mode")
        self.turning_mode_name = str(G("~turning_mode_name", "turning")
                                     ).strip().lower()
        # Heartbeat log period. Set to 0 to disable.
        self.hb_period         = float(G("~heartbeat_period_sec", 2.0))

        # Topic resolution: explicit private param wins, otherwise
        # fall back to the sjtu_drone-style ns/suffix default.
        self.in_pose_t    = G("~pose_topic",
                              self.in_ns + "/gt_pose")
        self.in_depth_t   = G("~depth_topic",
                              self.in_ns + "/front_depth/depth/image_raw")
        self.in_caminfo_t = G("~camera_info_topic",
                              self.in_ns + "/front_depth/depth/camera_info")

        # ── State ─────────────────────────────────────────────────
        # Raw inputs. self.frozen is the EFFECTIVE state computed from
        # them by _compute_freeze() according to the rules in the
        # docstring (mode-authoritative when available).
        self._explicit_freeze   = False
        self._mode_says_freeze  = False
        self.frozen             = False
        self._freeze_source     = SRC_EXPLICIT_FBACK  # set properly on first recompute

        self.last_pose     = None
        self.last_depth    = None
        self.last_caminfo  = None

        # ── Diagnostics ───────────────────────────────────────────
        self._n_explicit_msgs       = 0
        self._n_mode_msgs           = 0
        self._last_mode_str         = "(no msgs yet)"
        self._last_explicit_str     = "(no msgs yet)"
        self._last_state_change_t   = rospy.Time.now()

        # ── Publishers ────────────────────────────────────────────
        self.pub_pose    = rospy.Publisher(
            self.out_ns + "/gt_pose", Pose, queue_size=1)
        self.pub_depth   = rospy.Publisher(
            self.out_ns + "/front_depth/depth/image_raw",
            Image, queue_size=2)
        self.pub_caminfo = None
        if self.bridge_caminfo:
            self.pub_caminfo = rospy.Publisher(
                self.out_ns + "/front_depth/depth/camera_info",
                CameraInfo, queue_size=2)

        # ── Subscribers ───────────────────────────────────────────
        rospy.Subscriber(self.in_pose_t,    Pose,
                         self._pose_cb,    queue_size=10)
        rospy.Subscriber(self.in_depth_t,   Image,
                         self._depth_cb,   queue_size=2)
        if self.bridge_caminfo:
            rospy.Subscriber(self.in_caminfo_t, CameraInfo,
                             self._caminfo_cb, queue_size=2)
        rospy.Subscriber("/sensor_gate/freeze", Bool,
                         self._freeze_cb, queue_size=1)
        if self.freeze_on_turning_mode:
            rospy.Subscriber(self.demo_mode_topic, String,
                             self._demo_mode_cb, queue_size=10)
        # Manual stuck-mode recovery (see docstring).
        rospy.Subscriber("/sensor_gate/reset_mode_freeze", Bool,
                         self._reset_mode_freeze_cb, queue_size=1)

        # ── Timers ────────────────────────────────────────────────
        rospy.Timer(rospy.Duration(1.0 / self.replay_hz), self._replay)
        if self.hb_period > 0.0:
            rospy.Timer(rospy.Duration(self.hb_period), self._heartbeat)

        # ── Startup banner ────────────────────────────────────────
        rospy.loginfo("=" * 64)
        rospy.loginfo("sensor_gate ready  in_ns=%s  out_ns=%s  replay=%.0fHz",
                      self.in_ns, self.out_ns, self.replay_hz)
        rospy.loginfo("  in pose        = %s", self.in_pose_t)
        rospy.loginfo("  in depth       = %s", self.in_depth_t)
        if self.bridge_caminfo:
            rospy.loginfo("  in camera_info = %s", self.in_caminfo_t)
        else:
            rospy.loginfo("  in camera_info = DISABLED "
                          "(bridge_camera_info=false; intrinsics from rosparam)")
        if self.freeze_on_turning_mode:
            rospy.loginfo("  freeze logic   = MODE-AUTHORITATIVE")
            rospy.loginfo("                   frozen <- (%s == %r)",
                          self.demo_mode_topic, self.turning_mode_name)
            rospy.loginfo("                   /sensor_gate/freeze used only as")
            rospy.loginfo("                   fallback before first mode msg")
        else:
            rospy.loginfo("  freeze logic   = EXPLICIT-ONLY")
            rospy.loginfo("                   frozen <- /sensor_gate/freeze")
        rospy.loginfo("  heartbeat      = %s",
                      ("every %.1fs" % self.hb_period)
                      if self.hb_period > 0 else "disabled")
        rospy.loginfo("  manual override: rostopic pub --once "
                      "/sensor_gate/reset_mode_freeze std_msgs/Bool 'data: true'")
        rospy.loginfo("=" * 64)

    # ── Freeze computation ───────────────────────────────────────
    def _compute_freeze(self):
        """Decide the effective freeze given the raw inputs.

        Returns (frozen, source_label). The source label is used by the
        heartbeat to make it obvious which input is driving the freeze.
        """
        if not self.freeze_on_turning_mode:
            return self._explicit_freeze, SRC_EXPLICIT_ONLY
        if self._n_mode_msgs == 0:
            # Mode-based freezing enabled but the mode topic hasn't
            # spoken yet. Stay compatible with explicit-only behaviour
            # during the startup window — without this fallback, a
            # missing mode topic would silently turn freezing off
            # entirely.
            return self._explicit_freeze, SRC_EXPLICIT_FBACK
        return self._mode_says_freeze, SRC_MODE_AUTH

    def _recompute_freeze(self, trigger):
        new, source = self._compute_freeze()
        if new != self.frozen or source != self._freeze_source:
            # Log on either the frozen flag flipping OR the source-of-
            # truth changing (e.g. first mode message arriving promotes
            # us from explicit_fallback to mode_auth — worth noting
            # even if `frozen` didn't change).
            if new != self.frozen:
                self._last_state_change_t = rospy.Time.now()
            rospy.loginfo(
                "sensor_gate: %s  src=%s  (explicit=%s mode_turning=%s last_mode=%r) trigger=%s",
                "FREEZE" if new else "UNFREEZE",
                source,
                self._explicit_freeze, self._mode_says_freeze,
                self._last_mode_str, trigger)
        self.frozen = new
        self._freeze_source = source

    # ── Subscribers ──────────────────────────────────────────────
    def _freeze_cb(self, msg):
        self._n_explicit_msgs += 1
        new = bool(msg.data)
        self._last_explicit_str = "True" if new else "False"
        if new != self._explicit_freeze:
            self._explicit_freeze = new
            self._recompute_freeze("/sensor_gate/freeze=%s" % new)

    def _demo_mode_cb(self, msg):
        # Match what waypoint_follower does on the same topic so
        # trailing whitespace or casing across the bridge can't
        # desynchronize the two consumers.
        self._n_mode_msgs += 1
        mode = (msg.data or "").strip().lower()
        self._last_mode_str = mode if mode else "(empty)"
        new = (mode == self.turning_mode_name)
        # Always recompute on the first message (it may demote us from
        # explicit_fallback to mode_auth even if the bool value matches).
        if new != self._mode_says_freeze or self._n_mode_msgs == 1:
            self._mode_says_freeze = new
            self._recompute_freeze("demo_mode=%r" % mode)

    def _reset_mode_freeze_cb(self, msg):
        """Force-clear the mode-based freeze (see docstring).

        Use when /xtend/demo_mode is stuck on "turning" and you need
        to recover without restarting the node. The flag will be
        re-set automatically the next time a fresh "turning" message
        arrives, so this is non-destructive.
        """
        if self._mode_says_freeze:
            rospy.logwarn("sensor_gate: MANUAL reset of mode-based freeze  "
                          "(last_mode=%r, msgs=%d)",
                          self._last_mode_str, self._n_mode_msgs)
            self._mode_says_freeze = False
            self._recompute_freeze("manual /sensor_gate/reset_mode_freeze")

    # ── Diagnostic heartbeat ─────────────────────────────────────
    def _heartbeat(self, _evt):
        age = (rospy.Time.now() - self._last_state_change_t).to_sec()
        rospy.loginfo(
            "sensor_gate hb: frozen=%s src=%s  "
            "[explicit=%s mode_turning=%s]  last_mode=%r last_explicit=%s  "
            "msgs: mode=%d explicit=%d  age=%.1fs",
            self.frozen, self._freeze_source,
            self._explicit_freeze, self._mode_says_freeze,
            self._last_mode_str, self._last_explicit_str,
            self._n_mode_msgs, self._n_explicit_msgs, age)

    # ── Pass-through callbacks ───────────────────────────────────
    def _pose_cb(self, msg):
        # While frozen, we deliberately drop live updates so the cache
        # holds the snapshot from the moment freezing began.
        if self.frozen:
            return
        self.last_pose = msg
        self.pub_pose.publish(msg)

    def _depth_cb(self, msg):
        if self.frozen:
            return
        self.last_depth = msg
        self.pub_depth.publish(msg)

    def _caminfo_cb(self, msg):
        # Only reached when bridge_camera_info=true (subscriber not
        # created otherwise).
        if self.frozen:
            return
        self.last_caminfo = msg
        self.pub_caminfo.publish(msg)

    # ── Replay during freeze ─────────────────────────────────────
    def _replay(self, _evt):
        if not self.frozen:
            return
        now = rospy.Time.now()
        if self.last_pose is not None:
            self.pub_pose.publish(self.last_pose)
        if self.last_depth is not None:
            self.last_depth.header.stamp = now
            self.pub_depth.publish(self.last_depth)
        if self.pub_caminfo is not None and self.last_caminfo is not None:
            self.last_caminfo.header.stamp = now
            self.pub_caminfo.publish(self.last_caminfo)


if __name__ == "__main__":
    try:
        SensorGate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass