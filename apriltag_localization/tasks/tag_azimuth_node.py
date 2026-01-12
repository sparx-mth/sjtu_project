from __future__ import annotations

import time
import math
import yaml
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple, Union
import json
import cv2
import numpy as np
from pupil_apriltags import Detector

from apriltag_localization.core.localization.tag_azimuth_estimator import (
    TagAzimuthEstimator,
    TagObservation,
)
from datetime import datetime

@dataclass(frozen=True)
class CameraCalib:
    K: np.ndarray          # 3x3
    D: np.ndarray          # (n,) usually 5 or 8


def load_tag_config(path: str) -> Dict[int, float]:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"tag_config_path does not exist: {path}")

    with p.open("r") as f:
        data = yaml.safe_load(f)

    tags = data.get("tags", data)
    out: Dict[int, float] = {}
    for k, v in tags.items():
        out[int(k)] = float(v)

    if not out:
        raise ValueError(f"No tags found in config: {path}")
    return out


def load_camera_calib_yaml(path: str) -> CameraCalib:
    """
    Supports common formats like:
    camera_matrix: {data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]}
    distortion_coefficients: {data: [k1,k2,p1,p2,k3]}
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"camera_calib_path does not exist: {path}")

    with p.open("r") as f:
        data = yaml.safe_load(f)

    # Try ROS-like calibration YAML keys
    cm = data.get("camera_matrix", {})
    dc = data.get("distortion_coefficients", {})

    K_data = cm.get("data", None)
    D_data = dc.get("data", None)

    if K_data is None:
        # allow fallback: K: [[...],[...],[...]]
        K_data = data.get("K", None)
    if D_data is None:
        D_data = data.get("D", None)

    if K_data is None:
        raise ValueError("Could not find camera matrix in calib file (camera_matrix.data or K).")
    if D_data is None:
        D_data = [0, 0, 0, 0, 0]

    K = np.array(K_data, dtype=np.float64).reshape(3, 3)
    D = np.array(D_data, dtype=np.float64).reshape(-1)
    return CameraCalib(K=K, D=D)


def tag_object_points(tag_size_m: float) -> np.ndarray:
    """
    3D model points for the 4 tag corners in tag frame (Z=0 plane).
    Order MUST match the detector corners order (pupil_apriltags returns corners in
    consistent order around the tag).
    """
    s = tag_size_m / 2.0
    return np.array(
        [
            [-s, -s, 0],
            [ s, -s, 0],
            [ s,  s, 0],
            [-s,  s, 0],
        ],
        dtype=np.float64,
    )


class TagAzimuthOpenCVTask:
    """
    OpenCV adapter (no ROS):
    - Reads tag config YAML (id -> wall azimuth)
    - Detects AprilTags in frames (camera or video)
    - Estimates pose with solvePnP -> uses tx,tz
    - Runs core TagAzimuthEstimator
    """

    def __init__(
        self,
        tag_config_path: str,
        camera_calib_path: str,
        tag_size_m: float,
        video_source: Union[int, str] = 0,
        tag_family: str = "tag36h11",
        max_history: int = 20,
        max_time_diff_sec: float = 1.0,
        visualize: bool = True,
        out_json_path: str = "",
    ):
        self.tag_config_deg = load_tag_config(tag_config_path)
        self.estimator = TagAzimuthEstimator(
            tag_config_deg=self.tag_config_deg,
            max_history=max_history,
            max_time_diff_sec=max_time_diff_sec,
        )

        self.calib = load_camera_calib_yaml(camera_calib_path)
        self.obj_pts = tag_object_points(float(tag_size_m))

        self.detector = Detector(
            families=tag_family,
            nthreads=2,
        )

        self.video_source = video_source
        self.image_dir = None
        self.cap = None
        self.out_json_path = out_json_path.strip()
        self._json_f = None
        if self.out_json_path:
            out_p = Path(self.out_json_path).expanduser().resolve()
            out_p.parent.mkdir(parents=True, exist_ok=True)
            self._json_f = out_p.open("a", buffering=1)  # line-buffered


        if isinstance(video_source, str) and video_source.startswith("dir:"):
            # e.g. video_source="dir:/ros2_ws/apriltag_localization/PS"
            self.image_dir = Path(video_source[4:]).expanduser().resolve()
            if not self.image_dir.exists():
                raise FileNotFoundError(f"Image dir does not exist: {self.image_dir}")
        else:
            self.cap = cv2.VideoCapture(video_source)
            if not self.cap.isOpened():
                raise RuntimeError(f"Could not open video source: {video_source}")


        self.visualize = bool(visualize)

    def _solve_tag_pose(self, corners_2d: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        corners_2d: (4,2) float64
        returns (rvec, tvec) where tvec is tag position in camera frame.
        """
        ok, rvec, tvec = cv2.solvePnP(
            self.obj_pts,
            corners_2d,
            self.calib.K,
            self.calib.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,  # great for planar squares
        )
        if not ok:
            return None
        if float(tvec[2]) < 0:
            tvec = -tvec
            rvec = -rvec
        return rvec.reshape(3), tvec.reshape(3)

    @staticmethod
    def _pick_best(observations_with_meta):
        """
        Choose best tag by minimal |relative_yaw| (same as core logic) OR by largest area.
        We'll use same criterion as core by reusing relative_yaw,
        but we also keep 'area' for debugging.
        """
        best = None  # (abs_rel_yaw, area, obs, corners)
        for obs, corners in observations_with_meta:
            rel = TagAzimuthEstimator.relative_yaw_deg(obs.tx, obs.tz)
            abs_rel = abs(rel)
            area = float(cv2.contourArea(corners.astype(np.float32)))
            if best is None or abs_rel < best[0]:
                best = (abs_rel, area, obs, corners)
        return best
    

    def _iter_images_in_dir(self, exts=(".png", ".jpg", ".jpeg", ".bmp")):
        files = []
        for p in self.image_dir.iterdir():
            if p.is_file() and p.suffix.lower() in exts:
                files.append(p)
        files.sort()  
        return files
    
    def _log_json(self, record: dict):
        if self._json_f is None:
            return
        self._json_f.write(json.dumps(record, ensure_ascii=False) + "\n")



    def run(self):
        processed = set()

        while True:
            if self.image_dir is not None:
                files = self._iter_images_in_dir()
                next_file = None
                for f in files:
                    if str(f) not in processed:
                        next_file = f
                        break

                if next_file is None:
                    time.sleep(0.2)
                    continue

                frame = cv2.imread(str(next_file), cv2.IMREAD_COLOR)
                processed.add(str(next_file))

                if frame is None:
                    print(f"Failed to read image: {next_file}")
                    continue

                stamp_sec = next_file.stat().st_mtime  
                src_name = next_file.name

            else:
                ok, frame = self.cap.read()
                if not ok:
                    break
                stamp_sec = time.time()
                src_name = "camera/video"
            
            record_base = {
                "timestamp_sec": float(stamp_sec),
                "source_name": src_name,
                "source_type": "dir" if self.image_dir is not None else "video",
            }


            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            dets = self.detector.detect(gray)

            observations: List[TagObservation] = []
            observations_with_meta = []
            detections_info = []
            
            for d in dets:
                tag_id = int(d.tag_id)
                if tag_id not in self.tag_config_deg:
                    continue

                corners = np.array(d.corners, dtype=np.float64)

                pose = self._solve_tag_pose(corners)
                if pose is None:
                    continue
                _, tvec = pose

                obs = TagAzimuthEstimator.obs_from_tvec(tag_id=tag_id, tvec=tvec)
                observations.append(obs)
                observations_with_meta.append((obs, corners))
                rel = TagAzimuthEstimator.relative_yaw_deg(obs.tx, obs.tz)

                detections_info.append({
                    "tag_id": int(tag_id),
                    "tx": float(obs.tx),
                    "tz": float(obs.tz),
                    "rel_yaw_deg": float(rel),
                    "corners_px": [[float(x), float(y)] for (x, y) in corners],
                })

            if not observations:
                if self.visualize:
                    cv2.putText(
                        frame,
                        f"{src_name} | No known tags visible",
                        (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )
                    cv2.imshow("tag_azimuth", frame)
                    k = cv2.waitKey(0) & 0xFF
                    if k == 27 or k == ord('q'):
                        break
                self._log_json({
                    **record_base,
                    "found": False,
                    "best_tag_id": None,
                    "camera_azimuth_deg": None,
                    "detections": detections_info,  
                })
                continue

            result = self.estimator.update(observations, stamp_sec=stamp_sec)
            if result is None:
                continue

            yaw_deg, best_tag = result
            self._log_json({
                **record_base,
                "found": True,
                "best_tag_id": int(best_tag),
                "camera_azimuth_deg": float(yaw_deg),
                "detections": detections_info,
            })

            if self.visualize:
                best = self._pick_best(observations_with_meta)
                if best is not None:
                    _, _, obs, corners = best
                    cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 0), 2)
                    line1 = f"{src_name}"
                    line2 = f"Azimuth: {yaw_deg:.1f} deg | best_tag={best_tag}"
                    rel = TagAzimuthEstimator.relative_yaw_deg(obs.tx, obs.tz)
                    line3 = f"rel_yaw: {rel:.1f} deg | tx={obs.tx:.2f} tz={obs.tz:.2f}"

                    cv2.putText(frame, line1, (20, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                    cv2.putText(frame, line2, (20, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                    cv2.putText(frame, line3, (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                cv2.imshow("tag_azimuth", frame)
                k = cv2.waitKey(0) & 0xFF   # 0 = wait forever
                if k == 27 or k == ord('q'):  # ESC או q
                    break
        
            else:
                print(f"[{stamp_sec:.3f}] azimuth={yaw_deg:.1f} best_tag={best_tag}")

        if self.cap is not None:
            self.cap.release()
        if self._json_f is not None:
            self._json_f.close()
            self._json_f = None
        cv2.destroyAllWindows()


def main():
    """
    Example usage:
    python -m sparx_agency.robots.common.ros2.tag_azimuth_node \
        --tag_config_path /path/tags.yaml \
        --camera_calib_path /path/cam.yaml \
        --tag_size_m 0.16 \
        --source 0
    """
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--tag_config_path", required=True)
    ap.add_argument("--camera_calib_path", required=True)
    ap.add_argument("--tag_size_m", type=float, required=True)
    ap.add_argument("--source", default="0", help="camera index (0) or video path")
    ap.add_argument("--tag_family", default="tag36h11")
    ap.add_argument("--no_vis", action="store_true")
    ap.add_argument("--history_len", type=int, default=20)
    ap.add_argument("--max_time_diff_sec", type=float, default=1.0)
    ap.add_argument("--image_dir", default="", help="Directory to read images from (instead of camera/video).")
    ap.add_argument("--poll", type=float, default=0.2, help="Sleep seconds when no new images in dir.")
    ap.add_argument("--out_json", default="", help="Path to output JSONL log (one JSON per frame).")
    args = ap.parse_args()

    src: Union[int, str]
    if args.image_dir:
        src = f"dir:{args.image_dir}"
    else:
        if isinstance(args.source, str) and args.source.isdigit():
            src = int(args.source)
        else:
            src = args.source


    task = TagAzimuthOpenCVTask(
        tag_config_path=args.tag_config_path,
        camera_calib_path=args.camera_calib_path,
        tag_size_m=args.tag_size_m,
        video_source=src,
        tag_family=args.tag_family,
        max_history=args.history_len,
        max_time_diff_sec=args.max_time_diff_sec,
        visualize=(not args.no_vis),
        out_json_path=args.out_json,
    )
    task.run()


if __name__ == "__main__":
    main()
