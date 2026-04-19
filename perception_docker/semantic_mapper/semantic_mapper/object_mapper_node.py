#!/usr/bin/env python3
"""
object_mapper_node.py — YOLO detections -> 2D world XY -> dedupe -> markers + JSON.

Joins /perception/detections with depth + pose to place objects on the BEV map.
One colour per class (deterministic from the class name — every chair the same
green, every bed the same blue, etc.). Each object must be observed at least
`min_observations` times before it is published (false-positive guard).

Defensive defaults
------------------
* `rgb_intrin_fallback` and `depth_intrin_fallback` params are used if the
  bridge doesn't forward the matching /camera_info (a common ros1_bridge
  quirk). They're computed from the xacro values so the math is right.
  If the real CameraInfo arrives later, it overrides the fallback silently.
* pose/detection stamp matching falls back to the freshest cached pose when
  stamps are wildly off — happens when RGB is on sim time but pose is on
  wall time. Drone is slow, so the staleness cost is negligible.

Heartbeat
---------
Every 5 s prints  MISSING[...] listing whichever inputs haven't arrived,
or OK. Also tells you whether rgb_intr / depth_intr came from the bridge
or from the fallback.
"""

import colorsys
import json
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray


def _q2R(q):
    """[x, y, z, w] unit quaternion -> 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),         2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x * x + z * z),         2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


class ObjectMapper(Node):
    SKIP_PREFIXES = ("door:",)

    def __init__(self):
        super().__init__("object_mapper")

        P = self.declare_parameter
        P("world_frame",       "world")
        P("min_depth_m",       0.30)
        P("max_depth_m",       5.00)
        P("depth_shrink",      0.50)
        P("depth_percentile",  30.0)
        P("min_valid_pixels",  20)
        P("dedup_radius_m",    0.70)
        P("min_observations",  2)
        P("max_stamp_gap_s",   0.25)
        P("min_conf",          0.25)
        P("cell_size_m",       0.15)
        P("cell_patch",        2)
        P("legend_xy",         [-11.0, 16.0])

        # Fallback intrinsics (fx, fy, cx, cy, W, H) from the xacros:
        #   RGB   640x360, hfov=2.09 rad -> fx = 640 / (2 tan(1.045)) ~= 186
        #   Depth 640x480, hfov=pi/2     -> fx = 320
        P("rgb_intrin_fallback",   [186.0, 186.0, 320.0, 180.0, 640.0, 360.0])
        P("depth_intrin_fallback", [320.0, 320.0, 320.0, 240.0, 640.0, 480.0])

        g = lambda n: self.get_parameter(n).value
        self.frame    = str(g("world_frame"))
        self.min_d    = float(g("min_depth_m"))
        self.max_d    = float(g("max_depth_m"))
        self.shrink   = float(g("depth_shrink"))
        self.pctl     = float(g("depth_percentile"))
        self.min_px   = int(g("min_valid_pixels"))
        self.dedup_r2 = float(g("dedup_radius_m")) ** 2
        self.min_obs  = int(g("min_observations"))
        self.max_gap  = float(g("max_stamp_gap_s"))
        self.min_conf = float(g("min_conf"))
        self.cell     = float(g("cell_size_m"))
        self.pn       = int(g("cell_patch"))
        self.legend   = [float(v) for v in g("legend_xy")]

        # Start with fallbacks so the node can project from the very first
        # detection even if CameraInfo topics aren't bridged. The real
        # CameraInfo overrides these if/when it arrives.
        rf = [float(v) for v in g("rgb_intrin_fallback")]
        df = [float(v) for v in g("depth_intrin_fallback")]
        self.r_intr = (rf[0], rf[1], rf[2], rf[3], int(rf[4]), int(rf[5]))
        self.d_intr = (df[0], df[1], df[2], df[3], int(df[4]), int(df[5]))
        self.r_intr_bridged = False
        self.d_intr_bridged = False

        # State
        self.depth    = None
        self.poses    = deque(maxlen=64)
        self.objs     = {}
        self.next_id  = 0
        self.colors   = {}
        self._warned_clock = False

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Image,       "/map_ros/depth",
                                 self._depth_cb, sensor_qos)
        self.create_subscription(CameraInfo,  "/map_ros/depth/camera_info",
                                 self._dinfo_cb, 1)
        self.create_subscription(CameraInfo,  "/simple_drone/front/camera_info",
                                 self._rinfo_cb, 1)
        self.create_subscription(PoseStamped, "/map_ros/pose",
                                 self._pose_cb, 20)
        self.create_subscription(Detection2DArray, "/perception/detections",
                                 self._det_cb, 2)

        self.pub_obj = self.create_publisher(String, "/perception/objects", latched)
        self.pub_mk  = self.create_publisher(MarkerArray,
                                             "/perception/object_markers", 1)

        # Keep the legend visible even before any object is confirmed.
        self.create_timer(1.0, self._publish_markers)
        self.create_timer(5.0, self._hb)
        self._n = dict(dets=0, proj=0, new=0, skipped_oof=0)
        self.get_logger().info(
            f"object_mapper ready  dedup={g('dedup_radius_m'):.2f}m  "
            f"min_obs={self.min_obs}  conf>={self.min_conf:.2f}")
        self.get_logger().info(
            f"rgb fallback intrinsics: fx={rf[0]:.1f} cx={rf[2]:.1f} "
            f"{int(rf[4])}x{int(rf[5])}")

    # ------------------------------------------------------------------
    def _dinfo_cb(self, m):
        self.d_intr = (m.k[0], m.k[4], m.k[2], m.k[5], m.width, m.height)
        if not self.d_intr_bridged:
            self.d_intr_bridged = True
            self.get_logger().info(
                f"depth CameraInfo received: fx={m.k[0]:.1f} "
                f"{m.width}x{m.height}")

    def _rinfo_cb(self, m):
        self.r_intr = (m.k[0], m.k[4], m.k[2], m.k[5], m.width, m.height)
        if not self.r_intr_bridged:
            self.r_intr_bridged = True
            self.get_logger().info(
                f"RGB CameraInfo received: fx={m.k[0]:.1f} "
                f"{m.width}x{m.height}")

    def _depth_cb(self, m):
        if m.encoding != "32FC1":
            return
        self.depth = np.frombuffer(m.data, dtype=np.float32).reshape(
            m.height, m.width).copy()

    def _pose_cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.position, m.pose.orientation
        self.poses.append((t,
                           np.array([p.x, p.y, p.z]),
                           _q2R([q.x, q.y, q.z, q.w])))

    def _pose_at(self, t):
        """Closest pose in time; if none is within max_gap, fall back to the
        freshest cached pose. The drone is slow, so staleness is cheap; being
        robust to sim-vs-wall-clock mismatches between RGB and pose is worth
        more than strict stamp matching."""
        if not self.poses:
            return None
        best = min(self.poses, key=lambda pb: abs(pb[0] - t))
        if abs(best[0] - t) > self.max_gap:
            if not self._warned_clock:
                self.get_logger().warn(
                    f"pose/detection stamp gap = {abs(best[0] - t):.2f}s "
                    f"(> {self.max_gap}s); using latest pose. "
                    "Likely cause: RGB uses sim time, pose uses wall time.")
                self._warned_clock = True
            return (self.poses[-1][1], self.poses[-1][2])
        return (best[1], best[2])

    # ------------------------------------------------------------------
    def _det_cb(self, msg: Detection2DArray):
        self._n["dets"] += 1
        if self.depth is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self._pose_at(t)
        if pose is None:
            return
        cam_p, cam_R = pose

        fx_d, fy_d, cx_d, cy_d, dW, dH = self.d_intr
        fx_r, fy_r, cx_r, cy_r, _,  _  = self.r_intr
        kx = fx_d / fx_r
        ky = fy_d / fy_r

        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            raw = str(hyp.class_id)
            if raw.startswith(self.SKIP_PREFIXES):
                continue
            if float(hyp.score) < self.min_conf:
                continue
            cname = raw.split(":", 1)[-1] if ":" in raw else raw

            u_r = det.bbox.center.position.x
            v_r = det.bbox.center.position.y
            u_d = (u_r - cx_r) * kx + cx_d
            v_d = (v_r - cy_r) * ky + cy_d
            hw  = det.bbox.size_x * 0.5 * kx
            hh  = det.bbox.size_y * 0.5 * ky

            depth = self._robust_depth(u_d, v_d, hw, hh, dW, dH)
            if depth is None:
                self._n["skipped_oof"] += 1
                continue

            Xc = (u_d - cx_d) / fx_d * depth
            Yc = (v_d - cy_d) / fy_d * depth
            Pw = cam_R @ np.array([Xc, Yc, depth]) + cam_p
            self._n["proj"] += 1
            self._add(cname, float(Pw[0]), float(Pw[1]))

        if msg.detections:
            self._publish_objects()
            # markers also published periodically by timer; this one keeps
            # the display tight-synced to new detections.
            self._publish_markers()

    def _robust_depth(self, u, v, hw, hh, W, H):
        s = max(0.05, min(1.0, self.shrink))
        x0, x1 = int(max(0, u - hw * s)), int(min(W, u + hw * s))
        y0, y1 = int(max(0, v - hh * s)), int(min(H, v + hh * s))
        if x1 <= x0 or y1 <= y0:
            return None
        patch = self.depth[y0:y1, x0:x1]
        valid = np.isfinite(patch) & (patch >= self.min_d) & (patch <= self.max_d)
        if int(valid.sum()) < self.min_px:
            return None
        return float(np.percentile(patch[valid], self.pctl))

    def _add(self, cname, wx, wy):
        for o in self.objs.values():
            if o["class"] != cname:
                continue
            ox, oy = o["xy"]
            if (ox - wx) ** 2 + (oy - wy) ** 2 <= self.dedup_r2:
                n = o["count"]
                o["xy"] = ((ox * n + wx) / (n + 1),
                           (oy * n + wy) / (n + 1))
                o["count"] = n + 1
                return
        oid = self.next_id; self.next_id += 1
        self.objs[oid] = {"id": oid, "class": cname,
                          "xy": (wx, wy), "count": 1}
        self._n["new"] += 1
        self._color_for(cname)

    def _color_for(self, cname):
        if cname not in self.colors:
            h = (abs(hash(cname)) % 997) / 997.0
            self.colors[cname] = colorsys.hsv_to_rgb(h, 0.80, 1.0)
        return self.colors[cname]

    def _confirmed(self):
        return [o for o in self.objs.values() if o["count"] >= self.min_obs]

    # ------------------------------------------------------------------
    def _publish_objects(self):
        self.pub_obj.publish(String(data=json.dumps({
            "stamp":   self.get_clock().now().nanoseconds * 1e-9,
            "objects": [{"id": o["id"], "class": o["class"],
                         "xy": list(o["xy"]), "count": o["count"]}
                        for o in self._confirmed()],
        })))

    def _publish_markers(self):
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        stamp = self.get_clock().now().to_msg()
        nid = [0]

        def mk(ns, typ, scale, color, alpha=1.0):
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = stamp
            m.ns, m.id = ns, nid[0]; nid[0] += 1
            m.type, m.action = typ, Marker.ADD
            m.pose.orientation.w = 1.0
            if isinstance(scale, (tuple, list)):
                m.scale.x, m.scale.y, m.scale.z = (float(s) for s in scale)
            else:
                m.scale.x = m.scale.y = m.scale.z = float(scale)
            r, g, b = color
            m.color = ColorRGBA(r=float(r), g=float(g),
                                b=float(b), a=float(alpha))
            return m

        c   = self.cell
        off = (self.pn - 1) * 0.5
        confirmed = self._confirmed()

        for o in confirmed:
            m = mk(f'obj:{o["class"]}', Marker.CUBE_LIST,
                   (c, c, 0.06), self._color_for(o["class"]), 1.0)
            ox, oy = o["xy"]
            for i in range(self.pn):
                for j in range(self.pn):
                    m.points.append(Point(
                        x=ox + (j - off) * c,
                        y=oy + (i - off) * c,
                        z=0.10))
            arr.markers.append(m)

        for o in confirmed:
            t = mk("obj_labels", Marker.TEXT_VIEW_FACING,
                   (0, 0, 0.28), (1.0, 1.0, 1.0))
            t.pose.position = Point(x=o["xy"][0], y=o["xy"][1], z=1.0)
            t.text = o["class"]
            arr.markers.append(t)

        # Legend always present so you can tell the topic is alive.
        lx, ly = self.legend
        classes = sorted({o["class"] for o in confirmed})
        th = mk("legend", Marker.TEXT_VIEW_FACING,
                (0, 0, 0.55), (1.0, 1.0, 1.0))
        th.pose.position = Point(x=lx, y=ly, z=0.15)
        th.text = "OBJECTS" if classes else "OBJECTS (none yet)"
        arr.markers.append(th)
        for i, cname in enumerate(classes):
            yy = ly - (i + 1) * 0.6
            sw = mk("legend_sw", Marker.CUBE,
                    (c * 2, c * 2, 0.06), self._color_for(cname), 1.0)
            sw.pose.position = Point(x=lx, y=yy, z=0.15)
            arr.markers.append(sw)
            n = sum(1 for o in confirmed if o["class"] == cname)
            tx = mk("legend", Marker.TEXT_VIEW_FACING,
                    (0, 0, 0.40), (1.0, 1.0, 1.0))
            tx.pose.position = Point(x=lx + 0.8, y=yy, z=0.15)
            tx.text = f"{cname}  x{n}"
            arr.markers.append(tx)

        self.pub_mk.publish(arr)

    def _hb(self):
        missing = []
        if self.depth is None:  missing.append("depth")
        if not self.poses:      missing.append("pose")
        status = f"MISSING[{','.join(missing)}]" if missing else "OK"
        self.get_logger().info(
            f"obj hb  dets={self._n['dets']} proj={self._n['proj']} "
            f"new={self._n['new']} oof={self._n['skipped_oof']}  "
            f"total={len(self.objs)} "
            f"confirmed={len(self._confirmed())} classes={len(self.colors)}  "
            f"rgb_intr={'bridged' if self.r_intr_bridged else 'FALLBACK'}  "
            f"depth_intr={'bridged' if self.d_intr_bridged else 'FALLBACK'}  "
            f"{status}")
        self._n = dict(dets=0, proj=0, new=0, skipped_oof=0)


def main():
    rclpy.init()
    node = ObjectMapper()
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