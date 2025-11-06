#!/usr/bin/env bash
set -euo pipefail

#####################################
# === CONFIG (edit as needed) ===
#####################################
CAM_LINK="simple_drone/front_cam_link"           # Mechanical camera link frame (from URDF)
CAM_OPTICAL="simple_drone/front_cam_optical"     # Optical frame we will publish (Z forward, X right, Y down)
WORLD_FRAME="world"                               # World frame name

# Camera topics (from your setup)
IMAGE_TOPIC="/simple_drone/front/image_raw"
CAMINFO_TOPIC="/simple_drone/front/camera_info"

# AprilTag frames
TAG_EST_FRAME="tag36h11:14"        # Estimated frame published by apriltag_ros
TAG_GT_FRAME="tag36h11:14_gt"      # Ground-truth frame we will publish as a static TF

# Ground-truth pose of the tag in the world (from Gazebo UI)
TAG_X="11.195"
TAG_Y="0.000"
TAG_Z="1.500"

# Quaternion (from RPY roll=0, pitch=-1.5708, yaw=0)
TAG_QX=0.000000
TAG_QY=-0.707108
TAG_QZ=0.000000
TAG_QW=0.707105

# --- Tag #2 (ID=15) ---
TAG2_EST_FRAME="tag36h11:15"
TAG2_GT_FRAME="tag36h11:15_gt"

# Pose of tag #2 in world (from Gazebo)
TAG2_X="11.495"
TAG2_Y="1.000"
TAG2_Z="1.500"

# Same orientation as tag #1 (roll=0, pitch=-90°, yaw=0)
TAG2_QX=0.000000
TAG2_QY=-0.707108
TAG2_QZ=0.000000
TAG2_QW=0.707105

# --- [2.5/4] OPTIONAL: extra 90-deg alignment for GT axes ---
# Pick one: none | Rx90 | Rx-90 | Ry90 | Ry-90 | Rz90 | Rz-90
TAG_EXTRA_ROT="${TAG_EXTRA_ROT:-Rz90}"   # <-- נסי להתחיל מ-Rz90; אם הזווית עדיין גדולה, נסי אחד אחר

# apriltag_ros parameters
APRILTAG_FAMILY="36h11"            # Tag family
TAG_SIZE="0.348"                   # Tag side length in meters (must match Gazebo model)
PUBLISH_TF="true"                  # Publish TF for detections


EX_QX=-0.5; EX_QY=-0.5; EX_QZ=0.5; EX_QW=0.5

#####################################

# Keep child PIDs for clean shutdown
pids=()
cleanup() {
  echo ""
  echo "Stopping all processes..."
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait || true
}
trap cleanup INT TERM

# Ensure ROS 2 is present
command -v ros2 >/dev/null || { echo "ros2 not found in PATH"; exit 1; }

# Create a small Python script that logs accuracy over time
EVAL_PY="$(mktemp /tmp/apriltag_eval.XXXXXX.py)"
cat > "$EVAL_PY" << 'PY'
import math, numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

def q_to_np(q): return np.array([q.x, q.y, q.z, q.w], float)

def angle_deg(q1, q2):
    """Smallest rotation angle (deg) between two orientations (quaternions)."""
    dot = abs(float(np.dot(q1, q2)))
    dot = max(min(dot, 1.0), 0.0)
    return math.degrees(2.0 * math.acos(dot))

class Eval(Node):
    """
    Periodically fetches:
      - cam_optical -> tag (estimation from apriltag_ros)
      - cam_optical -> tag_gt (static GT TF)
    and prints instantaneous + running average errors (translation [m], rotation [deg]).
    """
    def __init__(self):
        super().__init__('apriltag_eval_allinone')
        import os
        self.cam = os.environ.get('CAM_OPTICAL')
        self.tag_est = os.environ.get('TAG_EST_FRAME')
        self.tag_gt  = os.environ.get('TAG_GT_FRAME')
        self.buf = Buffer()
        self.lst = TransformListener(self.buf, self)
        self.et, self.er = [], []
        self.timer = self.create_timer(0.3, self.tick)  # 3–4 Hz

    def tick(self):
        try:
            t_est = self.buf.lookup_transform(self.cam, self.tag_est, rclpy.time.Time())
            t_gt  = self.buf.lookup_transform(self.cam, self.tag_gt , rclpy.time.Time())
        except Exception:
            return

        te, qe = t_est.transform.translation, t_est.transform.rotation
        tg, qg = t_gt.transform.translation , t_gt.transform.rotation
        ve = np.array([te.x, te.y, te.z])
        vg = np.array([tg.x, tg.y, tg.z])

        et = float(np.linalg.norm(ve - vg))
        er = angle_deg(q_to_np(qe), q_to_np(qg))

        self.et.append(et); self.er.append(er)
        n = len(self.et)
        if n % 10 == 0:
            self.get_logger().info(
                f'inst: {et:.3f} m, {er:.2f} deg | '
                f'avg: {np.mean(self.et):.3f} m, {np.mean(self.er):.2f} deg (n={n})'
            )

def main():
    rclpy.init()
    n = Eval()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PY

echo "[1/4] Publishing static TF (link -> optical)..."
TAG1_GT_ALIGNED="${TAG_GT_FRAME}_aligned"
ros2 run tf2_ros static_transform_publisher \
  0 0 0  $EX_QX $EX_QY $EX_QZ $EX_QW \
  "$TAG_GT_FRAME" "$TAG1_GT_ALIGNED" &
pids+=($!)

TAG2_GT_ALIGNED="${TAG2_GT_FRAME}_aligned"
ros2 run tf2_ros static_transform_publisher \
  0 0 0  $EX_QX $EX_QY $EX_QZ $EX_QW \
  "$TAG2_GT_FRAME" "$TAG2_GT_ALIGNED" &
pids+=($!)


echo "[2/4] Publishing static TF (world -> $TAG_GT_FRAME)..."
# Ground-truth pose of the AprilTag model in the world
ros2 run tf2_ros static_transform_publisher \
  "$TAG_X" "$TAG_Y" "$TAG_Z" "$TAG_QX" "$TAG_QY" "$TAG_QZ" "$TAG_QW" \
  "$WORLD_FRAME" "$TAG_GT_FRAME" &
pids+=($!)
       
echo "[2/4b] Publishing static TF (world -> $TAG2_GT_FRAME)..."
ros2 run tf2_ros static_transform_publisher \
  "$TAG2_X" "$TAG2_Y" "$TAG2_Z" "$TAG2_QX" "$TAG2_QY" "$TAG2_QZ" "$TAG2_QW" \
  "$WORLD_FRAME" "$TAG2_GT_FRAME" &
pids+=($!)


export TAG1_EST_FRAME="$TAG_EST_FRAME"
export TAG1_GT_FRAME="$TAG1_GT_ALIGNED"

export TAG2_EST_FRAME="$TAG2_EST_FRAME"
export TAG2_GT_FRAME="$TAG2_GT_ALIGNED"

echo "[3/4] Starting apriltag_ros (with remaps and params)..."
# Use optical frame for correct camera convention (REP-103)
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:="$IMAGE_TOPIC" \
  -r camera_info:="$CAMINFO_TOPIC" \
  -p family:="$APRILTAG_FAMILY" \
  -p size:="$TAG_SIZE" \
  -p camera_frame:="$CAM_OPTICAL" \
  -p publish_tf:="$PUBLISH_TF" &
pids+=($!)

echo "[4/4] Starting accuracy logger..."
export CAM_OPTICAL TAG_EST_FRAME TAG1_GT_ALIGNED
python3 "$EVAL_PY" &
pids+=($!)

echo ""
echo "=== All set. Press Ctrl+C to stop. ==="
wait
