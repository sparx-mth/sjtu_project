#!/usr/bin/env python3
"""
yolo_detector.py — RGB -> YOLO-World -> Detection2DArray @ 1 Hz.

Open-vocabulary detection: pass any list of strings as classes via the
`vocabulary` parameter at launch, no retraining needed. Downstream
(object_mapper_node) does depth sampling and world-frame projection.

Publishes
---------
  /perception/detections   vision_msgs/Detection2DArray
  /perception/debug_image  sensor_msgs/Image   (boxes drawn on RGB, optional)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import (Detection2DArray, Detection2D,
                             ObjectHypothesisWithPose, BoundingBox2D)
from cv_bridge import CvBridge


# Small, focused vocabulary: YOLO-World is more accurate with fewer,
# well-separated prompts. Add/remove per scene as needed.
DEFAULT_VOCABULARY = [
    "person",
    "chair", "bed", "couch", "table", "desk",
    "toilet", "sink", "refrigerator",
    "tv", "potted plant", "trash can",
]


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        P = self.declare_parameter
        P("rgb_topic",           "/simple_drone/front/image_raw")
        P("model_path",          "yolov8s-world.pt")
        P("device",              "cuda:0")
        P("conf_thresh",         0.70)       # low: object_mapper filters again
        P("imgsz",               640)
        P("min_dt",              1.0)        # 1 Hz
        P("vocabulary",          DEFAULT_VOCABULARY)
        P("publish_debug_image", True)

        g = lambda n: self.get_parameter(n).value
        self.conf_thresh   = float(g("conf_thresh"))
        self.imgsz         = int(g("imgsz"))
        self.min_dt        = float(g("min_dt"))
        self.publish_debug = bool(g("publish_debug_image"))
        self.device        = str(g("device"))
        vocabulary         = [str(s) for s in g("vocabulary")]

        from ultralytics import YOLOWorld
        self.get_logger().info(
            f"loading YOLO-World {g('model_path')} on {self.device}")
        self.model = YOLOWorld(str(g("model_path")))
        self.model.to(self.device)
        self.model.set_classes(vocabulary)

        self.bridge = CvBridge()
        self.last_t = None

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        self.det_pub = self.create_publisher(
            Detection2DArray, "/perception/detections", 10)
        if self.publish_debug:
            self.dbg_pub = self.create_publisher(
                Image, "/perception/debug_image", 2)

        self.create_subscription(Image, str(g("rgb_topic")),
                                 self._rgb_cb, sensor_qos)

        self.get_logger().info(
            f"yolo_detector ready  rate={1.0/self.min_dt:.1f} Hz  "
            f"conf>={self.conf_thresh}  vocab={len(vocabulary)}")

    def _rgb_cb(self, msg: Image):
        now = self.get_clock().now()
        if (self.last_t is not None
                and (now - self.last_t).nanoseconds * 1e-9 < self.min_dt):
            return
        self.last_t = now

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}",
                                   throttle_duration_sec=5.0)
            return

        r = self.model.predict(img, imgsz=self.imgsz,
                               conf=self.conf_thresh,
                               device=self.device, verbose=False)[0]

        out = Detection2DArray()
        out.header = msg.header
        dbg = []

        if r.boxes is not None and len(r.boxes) > 0:
            xyxy  = r.boxes.xyxy.cpu().numpy()
            conf  = r.boxes.conf.cpu().numpy()
            cls   = r.boxes.cls.cpu().numpy().astype(int)
            names = r.names
            for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
                cname = str(names[int(k)]).lower()

                det = Detection2D()
                det.header = msg.header
                bb = BoundingBox2D()
                bb.center.position.x = float(0.5 * (x1 + x2))
                bb.center.position.y = float(0.5 * (y1 + y2))
                bb.size_x = float(x2 - x1)
                bb.size_y = float(y2 - y1)
                det.bbox = bb

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cname
                hyp.hypothesis.score = float(c)
                det.results.append(hyp)
                out.detections.append(det)

                dbg.append((int(x1), int(y1), int(x2), int(y2),
                            cname, float(c)))

        self.det_pub.publish(out)
        if self.publish_debug:
            self._publish_debug(img, dbg, msg.header)

    def _publish_debug(self, img, items, header):
        import cv2
        out = img.copy()
        for x1, y1, x2, y2, label, score in items:
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 200, 0), 2)
            cv2.putText(out, f"{label} {score:.2f}",
                        (x1, max(y1 - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 200, 0), 1, cv2.LINE_AA)
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()