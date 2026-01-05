#!/usr/bin/env python3
import os, csv, json, math, time
from dataclasses import dataclass
import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

@dataclass
class Goal:
    x: float
    y: float
    z: float = 0.0

class NomadRunLogger(Node):
    def __init__(self, out_dir: str, goal: Goal,
                 dist_thresh: float = 0.25,
                 consecutive_hits: int = 10,
                 timeout_sec: float = 120.0):
        super().__init__("nomad_run_logger")

        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)

        self.goal = goal
        self.dist_thresh = dist_thresh
        self.consecutive_hits_required = consecutive_hits
        self.timeout_sec = timeout_sec

        self.clock_time_sec = None
        self.start_wall = time.time()

        # last poses
        self.last_odom = None
        self.last_gt = None

        # reached logic
        self.hit_count = 0
        self.min_dist = float("inf")
        self.reached = False
        self.end_reason = "timeout"

        # open CSV writers
        self.gt_csv_path = os.path.join(out_dir, "traj_gt.csv")
        self.pred_csv_path = os.path.join(out_dir, "traj_pred.csv")
        self.gt_f = open(self.gt_csv_path, "w", newline="")
        self.pred_f = open(self.pred_csv_path, "w", newline="")
        self.gt_w = csv.writer(self.gt_f)
        self.pred_w = csv.writer(self.pred_f)

        # headers
        self.gt_w.writerow(["t_sec","x","y","z","qx","qy","qz","qw"])
        self.pred_w.writerow(["t_sec","x","y","z","qx","qy","qz","qw"])

        # subs
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(Pose, "/simple_drone/gt_pose", self.on_gt, 50)
        self.create_subscription(Odometry, "/simple_drone/odom", self.on_odom, 50)

        # timer to check timeout and finalize
        self.create_timer(0.1, self.on_timer)

        self.get_logger().info(f"Logging to: {out_dir}")
        self.get_logger().info(f"Goal fixed: x={goal.x}, y={goal.y}, z={goal.z}")

    def now_t(self):
        # prefer /clock time if available
        if self.clock_time_sec is not None:
            return self.clock_time_sec
        # fallback to wall since node start
        return time.time() - self.start_wall

    def on_clock(self, msg: Clock):
        # msg.clock is builtin_interfaces/Time
        self.clock_time_sec = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_gt(self, msg: Pose):
        self.last_gt = msg
        t = self.now_t()
        self.gt_w.writerow([
            f"{t:.6f}",
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])

    def on_odom(self, msg: Odometry):
        self.last_odom = msg
        t = self.now_t()
        p = msg.pose.pose
        self.pred_w.writerow([
            f"{t:.6f}",
            p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        ])

        # distance-to-goal in XY
        dx = p.position.x - self.goal.x
        dy = p.position.y - self.goal.y
        dist = math.sqrt(dx*dx + dy*dy)
        self.min_dist = min(self.min_dist, dist)

        if dist < self.dist_thresh:
            self.hit_count += 1
            if self.hit_count >= self.consecutive_hits_required:
                self.reached = True
                self.end_reason = "reached"
        else:
            self.hit_count = 0

    def on_timer(self):
        if self.reached:
            self.finalize_and_shutdown()
            return

        if (time.time() - self.start_wall) > self.timeout_sec:
            self.reached = False
            self.end_reason = "timeout"
            self.finalize_and_shutdown()

    def finalize_and_shutdown(self):
        # close CSV
        try:
            self.gt_f.flush(); self.pred_f.flush()
            self.gt_f.close(); self.pred_f.close()
        except Exception:
            pass

        # summary
        summary = {
            "goal": {"x": self.goal.x, "y": self.goal.y, "z": self.goal.z},
            "reached": bool(self.end_reason == "reached"),
            "end_reason": self.end_reason,
            "timeout_sec": self.timeout_sec,
            "dist_thresh": self.dist_thresh,
            "consecutive_hits": self.consecutive_hits_required,
            "min_dist_xy": self.min_dist,
        }

        if self.last_odom is not None:
            p = self.last_odom.pose.pose
            summary["final_pred"] = {
                "x": p.position.x, "y": p.position.y, "z": p.position.z,
                "qx": p.orientation.x, "qy": p.orientation.y, "qz": p.orientation.z, "qw": p.orientation.w
            }
        if self.last_gt is not None:
            g = self.last_gt
            summary["final_gt"] = {
                "x": g.position.x, "y": g.position.y, "z": g.position.z,
                "qx": g.orientation.x, "qy": g.orientation.y, "qz": g.orientation.z, "qw": g.orientation.w
            }

        with open(os.path.join(self.out_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info(f"Finished: {self.end_reason}. min_dist_xy={self.min_dist:.3f}")
        rclpy.shutdown()

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="output dir for this run (e.g., runs/run_000)")
    parser.add_argument("--goal_x", type=float, default=-8.810700)
    parser.add_argument("--goal_y", type=float, default=1.437040)
    parser.add_argument("--goal_z", type=float, default=0.0)
    parser.add_argument("--dist", type=float, default=0.25)
    parser.add_argument("--hits", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = NomadRunLogger(
        out_dir=args.out,
        goal=Goal(args.goal_x, args.goal_y, args.goal_z),
        dist_thresh=args.dist,
        consecutive_hits=args.hits,
        timeout_sec=args.timeout
    )
    rclpy.spin(node)

if __name__ == "__main__":
    main()
