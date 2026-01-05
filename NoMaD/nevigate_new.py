#!/usr/bin/env python3
"""
navigate_new.py

A drop-in variant of navigate.py that keeps the original navigation logic intact,
and only ADDS:
- trajectory logging (ODOM + GT)
- per-step logging (closest node, topomap distance, chosen waypoint, etc.)
- verification when NoMaD reports "Reached goal! Stopping..." by measuring the
  final ODOM distance to the fixed goal (x,y) and classifying success by a radius
- writing all artifacts to disk (JSON + CSV)

Notes:
- "Reached goal" is still determined EXACTLY the same way as the original file:
  closest_node == goal_node.
- The printed "distance" in the logs is the model/topomap distance, NOT meters.
"""

import argparse
import os
import time
import json
import csv
import math
from pathlib import Path

import numpy as np
import rclpy
import torch
import yaml
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from rclpy.node import Node
from PIL import Image as PILImage

# ROS
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

# UTILS
from visualnav_transformer.deployment.src.topic_names import (
    IMAGE_TOPIC,
    SAMPLED_ACTIONS_TOPIC,
    WAYPOINT_TOPIC,
)
from visualnav_transformer.deployment.src.utils import (
    load_model,
    msg_to_pil,
    to_numpy,
    transform_images,
)
from visualnav_transformer.train.vint_train.training.train_utils import get_action

# CONSTANTS
MODEL_WEIGHTS_PATH = "model_weights"
ROBOT_CONFIG_PATH = "config/robot.yaml"
MODEL_CONFIG_PATH = "config/models.yaml"
TOPOMAP_IMAGES_DIR = "topomaps/images"

with open(ROBOT_CONFIG_PATH, "r") as f:
    robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
RATE = robot_config["frame_rate"]

# GLOBALS
context_queue = []
context_size = None

# Load the model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)


def callback_obs(msg):
    print("Received image.")
    obs_img = msg_to_pil(msg)
    if context_size is not None:
        if len(context_queue) < context_size + 1:
            context_queue.append(obs_img)
        else:
            context_queue.pop(0)
            context_queue.append(obs_img)


class RunLogger:
    """Handles writing traj + summary to disk."""

    def __init__(self, out_dir: Path, goal_x: float, goal_y: float, success_radius: float):
        self.out_dir = out_dir
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.success_radius = float(success_radius)

        self.out_dir.mkdir(parents=True, exist_ok=True)

        # CSVs
        self.odom_csv_path = self.out_dir / "traj_odom.csv"
        self.gt_csv_path = self.out_dir / "traj_gt.csv"
        self.steps_csv_path = self.out_dir / "steps.csv"
        self.summary_path = self.out_dir / "summary.json"

        self._odom_f = open(self.odom_csv_path, "w", newline="")
        self._gt_f = open(self.gt_csv_path, "w", newline="")
        self._steps_f = open(self.steps_csv_path, "w", newline="")

        self.odom_w = csv.writer(self._odom_f)
        self.gt_w = csv.writer(self._gt_f)
        self.steps_w = csv.writer(self._steps_f)

        self.odom_w.writerow(["t_sec", "t_nanosec", "x", "y", "z", "qx", "qy", "qz", "qw",
                              "vx", "vy", "vz", "wx", "wy", "wz"])
        self.gt_w.writerow(["t_sec", "t_nanosec", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self.steps_w.writerow(["t_wall_sec", "closest_node", "goal_node", "topomap_dist",
                               "start_node", "end_node", "min_idx", "sg_idx",
                               "waypoint_x", "waypoint_y"])

        self.start_wall = time.time()

        self.final_odom = None
        self.final_gt = None
        self.reached_goal_topological = False
        self.dist_to_goal_xy_m = None
        self.success = None

    def close(self):
        for f in (self._odom_f, self._gt_f, self._steps_f):
            try:
                f.flush()
                f.close()
            except Exception:
                pass

    def log_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.header.stamp
        tw = msg.twist.twist
        self.odom_w.writerow([
            int(t.sec), int(t.nanosec),
            float(p.x), float(p.y), float(p.z),
            float(q.x), float(q.y), float(q.z), float(q.w),
            float(tw.linear.x), float(tw.linear.y), float(tw.linear.z),
            float(tw.angular.x), float(tw.angular.y), float(tw.angular.z),
        ])
        self.final_odom = {
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
        }

    def log_gt(self, msg: Pose, now_msg):
        p = msg.position
        q = msg.orientation
        self.gt_w.writerow([
            int(now_msg.sec), int(now_msg.nanosec),
            float(p.x), float(p.y), float(p.z),
            float(q.x), float(q.y), float(q.z), float(q.w),
        ])
        self.final_gt = {
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
        }

    def log_step(self, closest_node: int, goal_node: int, topomap_dist: float,
                 start_node: int, end_node: int, min_idx: int, sg_idx: int,
                 waypoint_xy):
        self.steps_w.writerow([
            time.time(), int(closest_node), int(goal_node), float(topomap_dist),
            int(start_node), int(end_node), int(min_idx), int(sg_idx),
            float(waypoint_xy[0]), float(waypoint_xy[1]),
        ])

    def verify_on_reached_goal(self):
        """Called exactly when the original logic reaches 'Reached goal! Stopping...'."""
        self.reached_goal_topological = True
        if self.final_odom is None:
            self.dist_to_goal_xy_m = None
            self.success = False
            return

        dx = self.final_odom["x"] - self.goal_x
        dy = self.final_odom["y"] - self.goal_y
        self.dist_to_goal_xy_m = float(math.sqrt(dx * dx + dy * dy))
        self.success = bool(self.dist_to_goal_xy_m <= self.success_radius)

    def write_summary(self, args: argparse.Namespace):
        payload = {
            "run_id": str(getattr(args, "run_id", "")),
            "timestamp_unix": int(time.time()),
            "model": str(args.model),
            "topomap_dir": str(args.dir),
            "goal_node": int(args.goal_node),
            "goal_node_resolved": int(getattr(args, "_goal_node_resolved", args.goal_node)),
            "goal_xy": {"x": self.goal_x, "y": self.goal_y},
            "success_radius_m": self.success_radius,
            "reached_goal_topological": bool(self.reached_goal_topological),
            "dist_to_goal_xy_m": self.dist_to_goal_xy_m,
            "success": bool(self.success) if self.success is not None else None,
            "final_odom": self.final_odom,
            "final_gt": self.final_gt,
            "artifacts": {
                "traj_odom_csv": str(self.odom_csv_path),
                "traj_gt_csv": str(self.gt_csv_path),
                "steps_csv": str(self.steps_csv_path),
            },
        }
        self.summary_path.write_text(json.dumps(payload, indent=2))


class NavigationNode(Node):
    def __init__(self, logger: RunLogger | None = None):
        super().__init__("navigation_node")

        self.create_subscription(Image, IMAGE_TOPIC, callback_obs, 10)

        self.waypoint_pub = self.create_publisher(Float32MultiArray, WAYPOINT_TOPIC, 1)
        self.sampled_actions_pub = self.create_publisher(Float32MultiArray, SAMPLED_ACTIONS_TOPIC, 1)

        self.logger = logger
        self.last_odom = None
        self.last_gt = None

        # Additions: subscribe to ODOM + GT for logging only (does not affect navigation logic)
        self.create_subscription(Odometry, "/simple_drone/odom", self._odom_cb, 50)
        self.create_subscription(Pose, "/simple_drone/gt_pose", self._gt_cb, 50)

    def _odom_cb(self, msg: Odometry):
        self.last_odom = msg
        if self.logger is not None:
            self.logger.log_odom(msg)

    def _gt_cb(self, msg: Pose):
        self.last_gt = msg
        if self.logger is not None:
            now_msg = self.get_clock().now().to_msg()
            self.logger.log_gt(msg, now_msg)


def main(args: argparse.Namespace):
    global context_size

    # load model parameters
    with open(MODEL_CONFIG_PATH, "r") as f:
        model_paths = yaml.safe_load(f)

    model_config_path = model_paths[args.model]["config_path"]
    with open(model_config_path, "r") as f:
        model_params = yaml.safe_load(f)

    context_size = model_params["context_size"]

    # load model weights
    ckpth_path = model_paths[args.model]["ckpt_path"]
    if os.path.exists(ckpth_path):
        print(f"Loading model from {ckpth_path}")
    else:
        raise FileNotFoundError(f"Model weights not found at {ckpth_path}")

    model = load_model(ckpth_path, model_params, device)
    model = model.to(device)
    model.eval()

    num_diffusion_iters = model_params["num_diffusion_iters"]
    noise_scheduler = DDPMScheduler(
        num_train_timesteps=model_params["num_diffusion_iters"],
        beta_schedule="squaredcos_cap_v2",
        clip_sample=True,
        prediction_type="epsilon",
    )

    # load topomap (same as original)
    topomap_filenames = sorted(
        os.listdir(os.path.join(TOPOMAP_IMAGES_DIR, args.dir)),
        key=lambda x: int(x.split(".")[0]),
    )
    topomap_dir = f"{TOPOMAP_IMAGES_DIR}/{args.dir}"
    num_nodes = len(os.listdir(topomap_dir))
    topomap = []
    for i in range(num_nodes):
        image_path = os.path.join(topomap_dir, topomap_filenames[i])
        topomap.append(PILImage.open(image_path))

    closest_node = 0
    assert -1 <= args.goal_node < len(topomap), "Invalid goal index"

    # resolve goal node exactly like original
    if args.goal_node == -1:
        goal_node = len(topomap) - 1
    else:
        goal_node = args.goal_node
    args._goal_node_resolved = goal_node

    reached_goal = False

    # --- NEW: output path + logger ---
    out_root = Path(args.out)
    run_id = args.run_id or f"run_{int(time.time())}"
    out_dir = out_root / run_id
    logger = RunLogger(out_dir=out_dir, goal_x=args.goal_x, goal_y=args.goal_y, success_radius=args.success_radius)

    # ROS
    rclpy.init()
    node = NavigationNode(logger=logger)

    try:
        while rclpy.ok():
            loop_start_time = time.time()

            waypoint_msg = Float32MultiArray()

            if len(context_queue) > model_params["context_size"]:
                if model_params["model_type"] == "nomad":
                    obs_images = transform_images(
                        context_queue, model_params["image_size"], center_crop=False
                    )
                    obs_images = torch.split(obs_images, 3, dim=1)
                    obs_images = torch.cat(obs_images, dim=1)
                    obs_images = obs_images.to(device)
                    mask = torch.zeros(1).long().to(device)

                    start = max(closest_node - args.radius, 0)
                    end = min(closest_node + args.radius + 1, goal_node)
                    goal_image = [
                        transform_images(
                            g_img, model_params["image_size"], center_crop=False
                        ).to(device)
                        for g_img in topomap[start: end + 1]
                    ]
                    goal_image = torch.concat(goal_image, dim=0)

                    obsgoal_cond = model(
                        "vision_encoder",
                        obs_img=obs_images.repeat(len(goal_image), 1, 1, 1),
                        goal_img=goal_image,
                        input_goal_mask=mask.repeat(len(goal_image)),
                    )
                    dists = model("dist_pred_net", obsgoal_cond=obsgoal_cond)
                    dists = to_numpy(dists.flatten())
                    min_idx = int(np.argmin(dists))
                    closest_node = min_idx + start
                    print(f"closest node: {closest_node}, distance: {dists[min_idx]}")

                    sg_idx = min(
                        min_idx + int(dists[min_idx] < args.close_threshold),
                        len(obsgoal_cond) - 1,
                    )
                    obs_cond = obsgoal_cond[sg_idx].unsqueeze(0)

                    # infer action (original)
                    with torch.no_grad():
                        if len(obs_cond.shape) == 2:
                            obs_cond = obs_cond.repeat(args.num_samples, 1)
                        else:
                            obs_cond = obs_cond.repeat(args.num_samples, 1, 1)

                        noisy_action = torch.randn(
                            (args.num_samples, model_params["len_traj_pred"], 2),
                            device=device,
                        )
                        naction = noisy_action

                        noise_scheduler.set_timesteps(num_diffusion_iters)

                        for k in noise_scheduler.timesteps[:]:
                            noise_pred = model(
                                "noise_pred_net",
                                sample=naction,
                                timestep=k,
                                global_cond=obs_cond,
                            )
                            naction = noise_scheduler.step(
                                model_output=noise_pred, timestep=k, sample=naction
                            ).prev_sample

                    naction = to_numpy(get_action(naction))
                    sampled_actions_msg = Float32MultiArray()
                    sampled_actions_msg.data = np.concatenate((np.array([0]), naction.flatten())).tolist()
                    node.sampled_actions_pub.publish(sampled_actions_msg)

                    naction = naction[0]
                    chosen_waypoint = naction[args.waypoint]

                    if model_params["normalize"]:
                        chosen_waypoint *= MAX_V / RATE
                    waypoint_msg.data = chosen_waypoint.tolist()
                    node.waypoint_pub.publish(waypoint_msg)

                    # --- NEW: log step metadata (does not affect nav) ---
                    logger.log_step(
                        closest_node=closest_node,
                        goal_node=goal_node,
                        topomap_dist=float(dists[min_idx]),
                        start_node=start,
                        end_node=end,
                        min_idx=min_idx,
                        sg_idx=int(sg_idx),
                        waypoint_xy=chosen_waypoint,
                    )

                    # keep timing logic identical
                    elapsed_time = time.time() - loop_start_time
                    sleep_time = max(0, (1.0 / RATE) - elapsed_time)
                    time.sleep(sleep_time)

            # --- reached_goal logic MUST stay identical ---
            reached_goal = closest_node == goal_node
            if reached_goal:
                print("Reached goal! Stopping...")

                # --- NEW: verify by physical distance to fixed goal using latest ODOM ---
                logger.verify_on_reached_goal()
                if logger.dist_to_goal_xy_m is None:
                    print("[VERIFY] No ODOM received -> FAIL")
                else:
                    verdict = "SUCCESS" if logger.success else "FAIL"
                    print(
                        f"[VERIFY] goal=({args.goal_x:.3f},{args.goal_y:.3f}) "
                        f"dist_xy={logger.dist_to_goal_xy_m:.3f}m "
                        f"radius={args.success_radius:.3f}m => {verdict}"
                    )

                logger.write_summary(args)
                print(f"[SAVED] {logger.summary_path}")
                break

            rclpy.spin_once(node, timeout_sec=0)

    finally:
        try:
            logger.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="NoMaD navigation (same logic) + verification + trajectory logging"
    )
    parser.add_argument(
        "--model",
        "-m",
        default="nomad",
        type=str,
        help="model name (only nomad is supported) (hint: check config/models.yaml) (default: nomad)",
    )
    parser.add_argument(
        "--waypoint",
        "-w",
        default=2,
        type=int,
        help="index of the waypoint used for navigation (default: 2)",
    )
    parser.add_argument(
        "--dir",
        "-d",
        default="topomap",
        type=str,
        help="path to topomap images (subdir under topomaps/images)",
    )
    parser.add_argument(
        "--goal-node",
        "-g",
        default=-1,
        type=int,
        help="goal node index in the topomap (if -1, then last node) (default: -1)",
    )
    parser.add_argument(
        "--close-threshold",
        "-t",
        default=3,
        type=int,
        help="temporal distance threshold before localizing to next node (default: 3)",
    )
    parser.add_argument(
        "--radius",
        "-r",
        default=4,
        type=int,
        help="number of local nodes to look at in the topomap for localization (default: 4)",
    )
    parser.add_argument(
        "--num-samples",
        "-n",
        default=8,
        type=int,
        help="number of actions sampled from the model (default: 8)",
    )

    # --- NEW: verification / output args ---
    parser.add_argument("--goal-x", type=float, required=True, help="fixed goal x in the same frame as /simple_drone/odom")
    parser.add_argument("--goal-y", type=float, required=True, help="fixed goal y in the same frame as /simple_drone/odom")
    parser.add_argument("--success-radius", type=float, default=0.5, help="success radius in meters (default: 0.5)")
    parser.add_argument("--out", type=str, default="runs_nomad", help="output root directory (default: runs_nomad)")
    parser.add_argument("--run-id", type=str, default=None, help="run id folder name (default: run_<unix>)")

    args = parser.parse_args()
    print(f"Using {device}")
    main(args)