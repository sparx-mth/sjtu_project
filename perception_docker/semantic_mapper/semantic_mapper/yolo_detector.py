#!/usr/bin/env python3
"""
yolo_detector.py  —  P3 of the pipeline (ROS2 Humble, YOLO-World edition).

RGB  ─► YOLO-World ─► /perception/detections   (vision_msgs/Detection2DArray)
                    ├► /perception/target_seen (std_msgs/Bool, transient-local)
                    └► /perception/debug_image (sensor_msgs/Image)

YOLO-World is OPEN VOCABULARY: you pass any list of strings as classes
at runtime. No fixed COCO list, no retraining.

Detections are tagged in `class_id`:
    "door:<cname>"    → opening (door / window / doorway) — used to cut rooms
    "target:<cname>"  → matches target_text (preempts main loop)
    "<cname>"         → regular object
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import (Detection2DArray, Detection2D,
                             ObjectHypothesisWithPose, BoundingBox2D)

from cv_bridge import CvBridge


DET_DOOR_PREFIX   = "door:"
DET_TARGET_PREFIX = "target:"


# ──────────────────────────────────────────────────────────────────────
#  Default open-vocabulary list for the hospital world.
#  YOLO-World accepts any string — add or remove as you like.
#
#  Opening prompts are intentionally varied. YOLO-World benefits from
#  multiple phrasings for the same concept; "open doorway" is detected
#  more reliably than "doorway" alone, and "window frame" works on
#  windows that are recessed into walls without obvious panes.
# ──────────────────────────────────────────────────────────────────────
DEFAULT_VOCABULARY = [
    # ── architectural openings (priority, used for room cutting) ──
    "door",
    "open door",
    "closed door",
    "doorway",
    "open doorway",
    "door frame",
    "window",
    "window frame",
    "open window",
    "glass window",

    # ── people ──
    "person", "patient", "nurse", "doctor",

    # ── furniture ──
    "chair", "wheelchair", "office chair", "armchair",
    "bed", "hospital bed", "stretcher",
    "couch", "sofa",
    "table", "bedside table", "coffee table", "desk",
    "cabinet", "drawer", "shelf",

    # ── bathroom / kitchen ──
    "toilet", "sink", "shower", "bathtub",
    "refrigerator", "microwave", "oven",

    # ── medical ──
    "iv stand", "medical cart", "monitor", "x-ray machine",
    "surgical trolley",

    # ── appliances / decor ──
    "tv", "television", "clock", "potted plant",
    "lamp", "trash can", "vending machine",
]


# Substrings that mark a class as an "opening" for room-cutting purposes.
DEFAULT_OPENING_KEYWORDS = ["door", "doorway", "window"]


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        # ── Params ──
        self.declare_parameter("rgb_topic", "/simple_drone/front/image_raw")
        self.declare_parameter("model_path", "yolov8s-world.pt")  # YOLO-World small
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("conf_thresh", 0.10)               # World needs lower thr
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("min_dt", 1.0)                     # 1 Hz default
        self.declare_parameter("target_text", "apple")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("vocabulary", DEFAULT_VOCABULARY)
        self.declare_parameter("opening_keywords", DEFAULT_OPENING_KEYWORDS)

        self.rgb_topic   = self.get_parameter("rgb_topic").value
        self.model_path  = self.get_parameter("model_path").value
        self.device      = self.get_parameter("device").value
        self.conf_thresh = float(self.get_parameter("conf_thresh").value)
        self.imgsz       = int(self.get_parameter("imgsz").value)
        self.min_dt      = float(self.get_parameter("min_dt").value)
        self.target_text = str(self.get_parameter("target_text").value).lower()
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)

        self.vocabulary = [str(s) for s in self.get_parameter("vocabulary").value]
        self.opening_keywords = [str(s).lower() for s in
                                 self.get_parameter("opening_keywords").value]

        # ── Model: YOLO-World ──
        from ultralytics import YOLOWorld
        self.get_logger().info(f"loading YOLO-World {self.model_path} on {self.device}")
        self.model = YOLOWorld(self.model_path)
        self.model.to(self.device)
        # Open-vocab call — give it any list of strings.
        self.model.set_classes(self.vocabulary)

        self.bridge = CvBridge()
        self.last_t = None
        self.target_seen = False

        # ── QoS ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── Pubs ──
        self.det_pub  = self.create_publisher(Detection2DArray,
                                              "/perception/detections", 10)
        self.seen_pub = self.create_publisher(Bool,
                                              "/perception/target_seen", latched_qos)
        self.seen_pub.publish(Bool(data=False))
        if self.publish_debug:
            self.dbg_pub = self.create_publisher(Image,
                                                 "/perception/debug_image", 2)

        # ── Sub ──
        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, sensor_qos)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  YOLO-World detector (open vocabulary)")
        self.get_logger().info(f"  RGB             : {self.rgb_topic}")
        self.get_logger().info(f"  model           : {self.model_path}  ({self.device})")
        self.get_logger().info(f"  rate            : {1.0/self.min_dt:.2f} Hz "
                               f"(min_dt={self.min_dt:.2f}s)")
        self.get_logger().info(f"  conf_thresh     : {self.conf_thresh}")
        self.get_logger().info(f"  target          : '{self.target_text}'")
        self.get_logger().info(f"  opening keywords: {self.opening_keywords}")
        self.get_logger().info(f"  vocab size      : {len(self.vocabulary)} prompts")
        self.get_logger().info("=" * 60)

    # ------------------------------------------------------------------
    def _is_opening(self, cname: str) -> bool:
        return any(kw in cname for kw in self.opening_keywords)

    # ------------------------------------------------------------------
    def rgb_cb(self, msg: Image):
        now = self.get_clock().now()
        if self.last_t is not None:
            dt = (now - self.last_t).nanoseconds * 1e-9
            if dt < self.min_dt:
                return
        self.last_t = now

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}", throttle_duration_sec=5.0)
            return

        results = self.model.predict(
            img, imgsz=self.imgsz, conf=self.conf_thresh,
            device=self.device, verbose=False,
        )[0]

        out = Detection2DArray()
        out.header = msg.header

        dbg_items = []
        saw_target = False
        if results.boxes is not None and len(results.boxes) > 0:
            xyxy = results.boxes.xyxy.cpu().numpy()
            conf = results.boxes.conf.cpu().numpy()
            cls  = results.boxes.cls.cpu().numpy().astype(int)
            names = results.names
            for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
                cname = str(names[int(k)]).lower()
                is_door   = self._is_opening(cname)
                is_target = self.target_text in cname

                tagged = cname
                if is_door:
                    tagged = DET_DOOR_PREFIX + cname
                elif is_target:
                    tagged = DET_TARGET_PREFIX + cname
                    saw_target = True

                det = Detection2D()
                det.header = msg.header
                bb = BoundingBox2D()
                bb.center.position.x = float(0.5 * (x1 + x2))
                bb.center.position.y = float(0.5 * (y1 + y2))
                bb.size_x = float(x2 - x1)
                bb.size_y = float(y2 - y1)
                det.bbox = bb

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = tagged
                hyp.hypothesis.score = float(c)
                det.results.append(hyp)
                out.detections.append(det)

                dbg_items.append((int(x1), int(y1), int(x2), int(y2),
                                  tagged, float(c), is_door, is_target))

        self.det_pub.publish(out)

        if (not self.target_seen) and saw_target:
            self.target_seen = True
            self.seen_pub.publish(Bool(data=True))
            self.get_logger().warn(f"=== TARGET SEEN: '{self.target_text}' ===")

        if self.publish_debug:
            self._publish_debug(img, dbg_items, msg.header)

    # ------------------------------------------------------------------
    def _publish_debug(self, img, items, header):
        import cv2
        out = img.copy()
        for x1, y1, x2, y2, label, score, is_door, is_target in items:
            color = ((0, 0, 255) if is_target           # red   = target
                     else (0, 255, 255) if is_door      # yellow= opening
                     else (0, 200, 0))                  # green = other
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            cv2.putText(out, f'{label} {score:.2f}', (x1, max(y1 - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        try:
            m = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")
            m.header = header
            self.dbg_pub.publish(m)
        except Exception:
            pass


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()