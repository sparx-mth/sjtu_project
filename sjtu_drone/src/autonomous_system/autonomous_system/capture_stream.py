"""
capture_stream.py — Run INSIDE the Docker container.
Saves a continuous stream of RGB+depth frames while you fly the drone.

Usage:
    python3 capture_stream.py --fps 5 --duration 20

    This captures 5 frames/sec for 20 seconds = 100 frames.
    Fly the drone during capture to build a proper motion sequence.

Output saved to /tmp/navdp_stream/
Copy out with:
    docker cp sjtu_drone_hospital:/tmp/navdp_stream ~/navdp_stream
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import os
import argparse
import time
import threading


class StreamCapture(Node):
    def __init__(self, save_dir, fps, duration):
        super().__init__('navdp_stream_capture')
        self.save_dir = save_dir
        self.fps = fps
        self.duration = duration
        self.total_frames = int(fps * duration)
        self.interval = 1.0 / fps
        os.makedirs(save_dir, exist_ok=True)

        self.latest_rgb = None
        self.latest_depth = None
        self.rgb_shape = None
        self.depth_shape = None
        self.frame_count = 0
        self.capturing = False

        # Subscribe to depth camera topics (RGB aligned with depth)
        self.rgb_sub = self.create_subscription(
            Image, '/simple_drone/front_depth/image_raw', self.rgb_cb, 10)
        self.depth_sub = self.create_subscription(
            Image, '/simple_drone/front_depth/depth/image_raw', self.depth_cb, 10)

        self.get_logger().info(f'Waiting for camera topics...')
        self.get_logger().info(f'Will capture {self.total_frames} frames at {fps} fps ({duration}s)')
        self.get_logger().info(f'Press Enter in this terminal to START capture, then fly the drone!')

        # Start input thread
        self.input_thread = threading.Thread(target=self.wait_for_start, daemon=True)
        self.input_thread.start()

    def rgb_cb(self, msg):
        if msg.encoding == 'rgb8':
            self.latest_rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        elif msg.encoding == 'bgr8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            self.latest_rgb = arr[:, :, ::-1].copy()
        self.rgb_shape = (msg.height, msg.width)

    def depth_cb(self, msg):
        if msg.encoding == '32FC1':
            self.latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width).copy()
        elif msg.encoding == '16UC1':
            self.latest_depth = (np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width).astype(np.float32) / 1000.0).copy()
        self.depth_shape = (msg.height, msg.width)

    def wait_for_start(self):
        """Wait for user to press Enter, then start capture timer."""
        while self.latest_rgb is None or self.latest_depth is None:
            time.sleep(0.1)
        print(f"\nReceiving: RGB {self.rgb_shape}, Depth {self.depth_shape}")
        input("\n>>> Press ENTER to start capture, then fly the drone! <<<\n")
        self.capturing = True
        self.capture_start_time = time.time()
        print(f"CAPTURING... fly the drone! ({self.duration}s)")

    def save_frame(self):
        """Called by timer to save current frame."""
        if not self.capturing:
            return
        if self.frame_count >= self.total_frames:
            elapsed = time.time() - self.capture_start_time
            print(f'\nDone! Saved {self.frame_count} frames in {elapsed:.1f}s to {self.save_dir}')
            print(f'Now exit and run: docker cp sjtu_drone_hospital:{self.save_dir} ~/navdp_stream')
            self.capturing = False
            raise SystemExit()

        if self.latest_rgb is None or self.latest_depth is None:
            return

        # Save RGB as raw numpy (faster than PNG, and preserves exact values)
        rgb_path = os.path.join(self.save_dir, f'frame_{self.frame_count:04d}_rgb.npy')
        np.save(rgb_path, self.latest_rgb)

        # Save depth as raw numpy (float32 meters)
        depth_path = os.path.join(self.save_dir, f'frame_{self.frame_count:04d}_depth.npy')
        np.save(depth_path, self.latest_depth)

        if self.frame_count % 10 == 0:
            print(f'  Frame {self.frame_count}/{self.total_frames} '
                  f'depth=[{self.latest_depth.min():.1f}-{self.latest_depth.max():.1f}m]')

        self.frame_count += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fps', type=float, default=5,
                        help='Frames per second to capture')
    parser.add_argument('--duration', type=float, default=20,
                        help='Duration in seconds')
    parser.add_argument('--save_dir', type=str, default='/tmp/navdp_stream')
    args = parser.parse_args()

    rclpy.init()
    node = StreamCapture(args.save_dir, args.fps, args.duration)

    # Create a timer at the desired FPS
    node.create_timer(node.interval, node.save_frame)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()