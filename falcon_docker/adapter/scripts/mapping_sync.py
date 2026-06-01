#!/usr/bin/env python3
"""
mapping_sync.py — timestamp-exact depth <-> localization pairing + a hard
localization gate for FALCON's voxel mapping.  (v3: multi-source localization)

WHAT IT GUARANTEES
==================
Every depth frame FALCON fuses is paired with the localization carrying the
SAME header.stamp (the RGB/depth capture time); a depth frame with no
co-temporal localization is DROPPED -- "no location => no voxels". Emitted pose
and depth share an IDENTICAL stamp, so FALCON's Transformer (tight
/transformer/timestamp_tolerance, e.g. 0.02 s) resolves an exact match.

ORDERING-AGNOSTIC
=================
Buffers BOTH streams and matches when the SECOND half of a pair arrives,
whichever comes first:
  * depth arrives, pose already present  -> emit now            (pose-leads, e.g. AprilTag@10Hz from RGB)
  * depth arrives, pose not yet present  -> hold depth, emit when its pose comes (pose-trails, e.g. flow_depth@3Hz from depth)
  * pose never comes for that stamp      -> held depth ages out after ~match_hold_sec and is DROPPED (gate)
The wait is NON-BLOCKING. So the node is correct for a pose-leads OR a
pose-trails source with no code change; only ~match_hold_sec matters and its
default covers both.

MULTI-SOURCE (this is the v3 change -- read this)
=================================================
~pose_stamped_topics may name ONE topic or SEVERAL (comma-separated string, or
a YAML list via rosparam). Order = PRIORITY (first wins). For each depth stamp
the node tries the sources in order and uses the first that has a co-temporal
pose; lower-priority sources fill the gaps. Examples:

    /flow_depth/pose_est                       # single source (pose-trails)
    /xtend/april_tag_pose                       # single source (pose-leads)
    /xtend/april_tag_pose,/flow_depth/pose_est  # AprilTag preferred, flow_depth fallback

Subscribing to a source that isn't publishing is harmless -- its buffer stays
empty and the node simply falls through to the next source. So you can leave
both configured permanently and it will transparently use whichever is alive,
preferring the higher-priority one.

  *** SAFETY: SAME WORLD FRAME REQUIRED WHEN USING >1 SOURCE ***
  Per-frame fallback between two estimators is only valid if they publish in
  the SAME world frame and agree on scale/origin. An absolute, drift-free
  source (AprilTag, from a known tag map) and a relative/drifting source
  (visual-inertial or flow odometry whose origin is the start pose) are
  generally NOT in the same frame -- switching between them per frame would
  teleport the map at every handover and corrupt the voxels. If you are not
  certain both are in one consistent world frame, configure ONE source.
  The node measures cross-source position disagreement on co-temporal frames
  and warns ("sources disagree ...") so a frame mismatch is loud, not silent.

FREQUENCY IS NOT THE ISSUE
==========================
Matching is by timestamp, so a 1-3 Hz pair matches as well as a 10 Hz one --
provided both messages carry the same capture stamp. Raising ~sync_tolerance
does NOT recover frames with no co-temporal pose (a real dropout); it only
risks mis-pairing. Keep it small.

FEED IT THE DEPTH THAT CARRIES THE TRUE CAPTURE STAMP
=====================================================
Point ~depth_topic at the SAME depth the localization consumed (raw DA3 output,
e.g. /xtend/depth_m). Do NOT feed a depth re-stamped to wall-clock by a
freeze/replay gate -- that stamp no longer equals the pose's capture stamp, so
nothing pairs and you get zero voxels. The heartbeat distinguishes that case
("clockmismatch") from a real dropout ("dropout").

FRAME HANDLING
==============
By default the incoming pose is treated as a body(FLU) pose and right-
multiplied by T_b_c (+ cam_offset), reproducing falcon_adapter so this node
changes ONLY timing/gating/source-selection. If your localization already
publishes the CAMERA pose in world, set ~pose_is_camera_frame:=true and
~cam_offset_x:=0.0 in BOTH this node and falcon_adapter, and validate
separately. (This applies to ALL sources -- they must share the convention.)

PARAMS (all private ~)
  ~pose_stamped_topics  localization input(s): comma string or list   default /flow_depth/pose_est
                        (priority = order; ~pose_stamped_topic singular still honored)
  ~depth_topic          depth input (Image)                           default /xtend/depth_m
  ~out_pose_topic       camera-in-world out (PoseStamped)             default /map_ros/pose
  ~out_depth_topic      depth out (Image)                             default /map_ros/depth
  ~world_frame          frame_id on the out pose                      default world

  ~sync_tolerance       s; nearest pose within this = match           default 0.05
  ~max_interp_gap       s; SLERP if bracketed within this (per source) default 0.12 (0=off)
  ~match_hold_sec       s; non-blocking wait for a late pose          default 0.5  (0=v1 behavior)
  ~pose_buffer_sec      s of per-source pose history                  default 5.0
  ~depth_min_dt         s; throttle on INTAKE depth rate              default 0.0 (off)
  ~clock_warn_sec       s; dropped depth's nearest pose farther than  default 0.5
                        this -> warn "different clocks"
  ~disagree_warn_m      m; co-temporal cross-source position gap over default 0.30
                        this -> warn "sources disagree" (frame check)

  Frame (mirror falcon_adapter):
  ~pose_is_camera_frame default False
  ~cam_offset_x/y/z     default 0.2 / 0.0 / 0.0
"""

import threading
from bisect import bisect_left

import numpy as np
import rospy
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image


def _pose_to_T(pose):
    q = pose.orientation
    T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    return T


def _fill_pose_from_T(ps_pose, T):
    ps_pose.position.x = float(T[0, 3])
    ps_pose.position.y = float(T[1, 3])
    ps_pose.position.z = float(T[2, 3])
    q = tft.quaternion_from_matrix(T)
    ps_pose.orientation.x = float(q[0])
    ps_pose.orientation.y = float(q[1])
    ps_pose.orientation.z = float(q[2])
    ps_pose.orientation.w = float(q[3])


def _parse_topics(raw, fallback_single):
    if raw is None:
        raw = fallback_single
    if isinstance(raw, (list, tuple)):
        items = [str(s).strip() for s in raw]
    else:
        items = [s.strip() for s in str(raw).split(",")]
    items = [s for s in items if s]
    return items if items else [fallback_single]


class _PendingDepth:
    __slots__ = ("stamp", "msg", "wall")

    def __init__(self, stamp, msg, wall):
        self.stamp = stamp        # capture time (sec) from the depth header
        self.msg = msg            # original Image msg, forwarded unchanged
        self.wall = wall          # rospy.Time received (for the hold)


class MappingSync:
    def __init__(self):
        rospy.init_node("mapping_sync")
        G = rospy.get_param

        self.topics = _parse_topics(G("~pose_stamped_topics", None),
                                    G("~pose_stamped_topic", "/flow_depth/pose_est"))
        self.nsrc = len(self.topics)

        self.depth_topic     = G("~depth_topic", "/xtend/depth_m")
        self.out_pose_topic  = G("~out_pose_topic", "/map_ros/pose")
        self.out_depth_topic = G("~out_depth_topic", "/map_ros/depth")
        self.world_frame     = G("~world_frame", "world")

        self.sync_tol        = float(G("~sync_tolerance", 0.05))
        self.max_interp_gap  = float(G("~max_interp_gap", 0.12))
        self.match_hold_sec  = float(G("~match_hold_sec", 0.5))
        self.buffer_sec      = float(G("~pose_buffer_sec", 5.0))
        self.depth_min_dt    = float(G("~depth_min_dt", 0.0))
        self.clock_warn_sec  = float(G("~clock_warn_sec", 0.5))
        self.disagree_warn_m = float(G("~disagree_warn_m", 0.30))

        self.pose_is_camera_frame = bool(G("~pose_is_camera_frame", False))
        cam_x = float(G("~cam_offset_x", 0.2))
        cam_y = float(G("~cam_offset_y", 0.0))
        cam_z = float(G("~cam_offset_z", 0.0))
        self.T_b_c = np.array([
            [ 0.0,  0.0, 1.0, cam_x],
            [-1.0,  0.0, 0.0, cam_y],
            [ 0.0, -1.0, 0.0, cam_z],
            [ 0.0,  0.0, 0.0, 1.0],
        ])

        # Shared state (guarded by _lock). One pose buffer PER source.
        self._lock = threading.Lock()
        self._buf = [{"stamps": [], "poses": []} for _ in range(self.nsrc)]
        self._pending = []                # _PendingDepth, stamp-sorted
        self._prev_intake_t = None
        self._last_pose_wall = None

        # diagnostics
        self._n_pose = [0] * self.nsrc
        self._n_emit_src = [0] * self.nsrc
        self._n_depth = 0
        self._n_exact = 0
        self._n_nearest = 0
        self._n_interp = 0
        self._n_held = 0
        self._n_drop_throttle = 0
        self._n_drop_empty = 0            # dropped, no poses near -> real dropout
        self._n_drop_far = 0              # dropped, poses present but FAR -> clock mismatch
        self._n_disagree = 0
        self._last_far = 0.0
        self._last_disagree = 0.0

        self.pub_pose = rospy.Publisher(self.out_pose_topic, PoseStamped, queue_size=10)
        self.pub_depth = rospy.Publisher(self.out_depth_topic, Image, queue_size=8)

        self._subs = []
        for idx, tp in enumerate(self.topics):
            self._subs.append(
                rospy.Subscriber(tp, PoseStamped, self._pose_cb,
                                 callback_args=idx, queue_size=200))
        rospy.Subscriber(self.depth_topic, Image, self._depth_cb, queue_size=16)

        rospy.Timer(rospy.Duration(0.05), self._sweep_timer)
        rospy.Timer(rospy.Duration(2.0), self._heartbeat)

        rospy.loginfo("=" * 72)
        rospy.loginfo("mapping_sync v3 -- multi-source pairing + localization gate")
        for i, tp in enumerate(self.topics):
            rospy.loginfo("  pose  in[%d] = %s   (priority %d%s)",
                          i, tp, i, " = HIGHEST" if i == 0 else "")
        if self.nsrc > 1:
            rospy.loginfo("  >>> %d sources: first with a co-temporal pose wins; "
                          "ensure ALL share one world frame (disagree_warn=%.2fm)",
                          self.nsrc, self.disagree_warn_m)
        rospy.loginfo("  depth in  = %s   (stamp == capture time; use the RAW depth)", self.depth_topic)
        rospy.loginfo("  pose  out = %s", self.out_pose_topic)
        rospy.loginfo("  depth out = %s", self.out_depth_topic)
        rospy.loginfo("  sync_tol=%.3fs  interp_gap=%.3fs%s  match_hold=%.3fs  buffer=%.1fs",
                      self.sync_tol, self.max_interp_gap,
                      " (interp OFF)" if self.max_interp_gap <= 0 else "",
                      self.match_hold_sec, self.buffer_sec)
        rospy.loginfo("  pose_is_camera_frame=%s  cam_offset=(%.3f,%.3f,%.3f)",
                      self.pose_is_camera_frame, cam_x, cam_y, cam_z)
        rospy.loginfo("=" * 72)

    # -- localization in (one callback per source via callback_args) -----
    def _pose_cb(self, msg, src):
        t = msg.header.stamp.to_sec()
        if t <= 0.0:
            rospy.logwarn_throttle(5.0, "mapping_sync: pose on %s has stamp %.6f <= 0 -- "
                                        "is it stamped with the capture time?",
                                   self.topics[src], t)
        T = _pose_to_T(msg.pose)
        emits = []
        with self._lock:
            self._n_pose[src] += 1
            self._last_pose_wall = rospy.Time.now()
            b = self._buf[src]
            stamps, poses = b["stamps"], b["poses"]
            i = bisect_left(stamps, t)
            if i < len(stamps) and stamps[i] == t:
                poses[i] = T
            else:
                stamps.insert(i, t)
                poses.insert(i, T)
            cutoff = stamps[-1] - self.buffer_sec
            d = 0
            while d < len(stamps) and stamps[d] < cutoff:
                d += 1
            if d:
                del stamps[:d]
                del poses[:d]
            # Frame-consistency check: compare this just-arrived pose against any
            # OTHER source that has a co-temporal pose. Runs on every arrival, so
            # it catches the common case where sources are staggered in time
            # (e.g. flow_depth trails AprilTag) and are never both present at a
            # single match instant.
            if self.nsrc > 1:
                self._check_disagreement_locked(src, t, T)
            emits = self._match_pending_locked()
        self._emit(emits)

    def _check_disagreement_locked(self, src, t, T):
        for s in range(self.nsrc):
            if s == src:
                continue
            To, _kind, _dt = self._lookup_one(self._buf[s], t, self.sync_tol, 0.0)
            if To is None:
                continue
            dist = float(np.linalg.norm(To[:3, 3] - T[:3, 3]))
            if dist > self.disagree_warn_m:
                self._n_disagree += 1
                self._last_disagree = dist

    # -- depth in --------------------------------------------------------
    def _depth_cb(self, msg):
        td = msg.header.stamp.to_sec()
        self._n_depth += 1

        if self.depth_min_dt > 0.0 and self._prev_intake_t is not None:
            if (td - self._prev_intake_t) < self.depth_min_dt:
                self._n_drop_throttle += 1
                return
        self._prev_intake_t = td

        emits = []
        with self._lock:
            T_body, kind, src = self._lookup_locked(td)
            if T_body is not None:
                emits.append(self._build_emit_locked(msg, T_body, kind, src))
            else:
                p = _PendingDepth(td, msg, rospy.Time.now())
                idx = bisect_left([q.stamp for q in self._pending], td)
                self._pending.insert(idx, p)
                self._n_held += 1
            self._sweep_locked(rospy.Time.now())
        self._emit(emits)

    # -- matching helpers (lock held) ------------------------------------
    @staticmethod
    def _lookup_one(b, t, tol, gap):
        """Match within ONE source's buffer. Returns (T, kind, dt) or (None,_,None)."""
        stamps, poses = b["stamps"], b["poses"]
        n = len(stamps)
        if n == 0:
            return None, "empty", None
        i = bisect_left(stamps, t)
        bj, bd = None, None
        for j in (i - 1, i):
            if 0 <= j < n:
                d = abs(stamps[j] - t)
                if bd is None or d < bd:
                    bd, bj = d, j
        if bj is not None and bd <= tol:
            return poses[bj].copy(), ("exact" if bd == 0.0 else "nearest"), bd
        if gap > 0.0 and 0 < i < n:
            t0, t1 = stamps[i - 1], stamps[i]
            if (t1 - t0) <= gap and t0 <= t <= t1:
                return MappingSync._interp(poses[i - 1], t0, poses[i], t1, t), "interp", 0.0
        return None, "no_match", None

    def _lookup_locked(self, t):
        """Try sources in priority order; first with a co-temporal pose wins.
        (Cross-source frame-mismatch detection runs in _check_disagreement_locked
        on every pose arrival, which is when staggered sources actually overlap.)"""
        win = None  # (T, kind, src)
        for s in range(self.nsrc):
            T, kind, _dt = self._lookup_one(self._buf[s], t, self.sync_tol, self.max_interp_gap)
            if T is None:
                continue
            if win is None:
                win = (T, kind, s)
                break  # priority order: first source with a co-temporal pose wins
        if win is None:
            return None, "no_match", -1
        return win

    def _match_pending_locked(self):
        if not self._pending:
            return []
        emits, keep = [], []
        for p in self._pending:
            T_body, kind, src = self._lookup_locked(p.stamp)
            if T_body is not None:
                emits.append(self._build_emit_locked(p.msg, T_body, kind, src))
            else:
                keep.append(p)
        self._pending = keep
        return emits

    def _sweep_locked(self, now_wall):
        if not self._pending:
            return
        keep = []
        for p in self._pending:
            if (now_wall - p.wall).to_sec() > self.match_hold_sec:
                self._record_drop_locked(p.stamp)      # gate: no location => dropped
            else:
                keep.append(p)
        self._pending = keep

    def _record_drop_locked(self, td):
        # nearest pose across ALL sources, to classify dropout vs clock-mismatch
        best_d = None
        for s in range(self.nsrc):
            stamps = self._buf[s]["stamps"]
            n = len(stamps)
            if n == 0:
                continue
            i = bisect_left(stamps, td)
            for j in (i - 1, i):
                if 0 <= j < n:
                    d = abs(stamps[j] - td)
                    best_d = d if best_d is None else min(best_d, d)
        if best_d is None:
            self._n_drop_empty += 1
        elif best_d > self.clock_warn_sec:
            self._n_drop_far += 1
            self._last_far = best_d
        else:
            self._n_drop_empty += 1

    def _build_emit_locked(self, depth_msg, T_body, kind, src):
        if kind == "exact":
            self._n_exact += 1
        elif kind == "nearest":
            self._n_nearest += 1
        else:
            self._n_interp += 1
        if 0 <= src < self.nsrc:
            self._n_emit_src[src] += 1
        T_w_c = T_body if self.pose_is_camera_frame else (T_body @ self.T_b_c)
        ps = PoseStamped()
        ps.header.stamp = depth_msg.header.stamp        # IDENTICAL to the depth stamp
        ps.header.frame_id = self.world_frame
        _fill_pose_from_T(ps.pose, T_w_c)
        return (ps, depth_msg)

    @staticmethod
    def _interp(T0, t0, T1, t1, t):
        r = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        p = (1.0 - r) * T0[:3, 3] + r * T1[:3, 3]
        q = tft.quaternion_slerp(tft.quaternion_from_matrix(T0),
                                 tft.quaternion_from_matrix(T1), r)
        T = tft.quaternion_matrix(q)
        T[:3, 3] = p
        return T

    # -- emit (lock NOT held) --------------------------------------------
    def _emit(self, emits):
        for pose_msg, depth_msg in emits:
            self.pub_pose.publish(pose_msg)     # pose first
            self.pub_depth.publish(depth_msg)

    # -- timers ----------------------------------------------------------
    def _sweep_timer(self, _evt):
        with self._lock:
            self._sweep_locked(rospy.Time.now())

    def _heartbeat(self, _evt):
        with self._lock:
            bufs = [len(b["stamps"]) for b in self._buf]
            pend = len(self._pending)
            n_pose = list(self._n_pose)
            emit_src = list(self._n_emit_src)
            n_depth = self._n_depth
            ex, ne, it = self._n_exact, self._n_nearest, self._n_interp
            d_empty, d_far, d_thr = self._n_drop_empty, self._n_drop_far, self._n_drop_throttle
            ndis, last_far, last_dis = self._n_disagree, self._last_far, self._last_disagree
        emit = ex + ne + it
        pose_age = (rospy.Time.now() - self._last_pose_wall).to_sec() if self._last_pose_wall else -1.0
        src_str = "  ".join("[%d]%s pose=%d buf=%d emit=%d"
                            % (i, self.topics[i].split("/")[-1], n_pose[i], bufs[i], emit_src[i])
                            for i in range(self.nsrc))
        rospy.loginfo(
            "mapping_sync hb | %s | depth=%d -> emit=%d [exact=%d near=%d interp=%d] "
            "held_now=%d drop[dropout=%d clockmismatch=%d throttle=%d] | last_pose=%.1fs ago",
            src_str, n_depth, emit, ex, ne, it, pend, d_empty, d_far, d_thr, pose_age)
        if self.nsrc > 1 and ndis > 0:
            rospy.logwarn_throttle(
                5.0, "mapping_sync: %d co-temporal frames where sources disagree by up to "
                     "%.2fm -- they may be in DIFFERENT world frames. If so, use ONE source.",
                ndis, last_dis)
        if n_depth > 10 and emit == 0:
            if d_far > 0:
                rospy.logwarn_throttle(
                    5.0, "mapping_sync: NOT pairing; nearest pose to dropped depth ~%.2fs away "
                         "-- depth and pose look like DIFFERENT clocks. Feed ~depth_topic the "
                         "RAW depth the localization used (not a re-stamped topic).", last_far)
            else:
                rospy.logwarn_throttle(
                    5.0, "mapping_sync: depth flowing but ZERO paired and no poses near them -- "
                         "is any localization publishing a PoseStamped on %s?", self.topics)


if __name__ == "__main__":
    try:
        MappingSync()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass