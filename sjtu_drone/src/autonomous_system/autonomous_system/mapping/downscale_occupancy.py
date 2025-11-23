#!/usr/bin/env python3
"""
downscale_occupancy.py
----------------------
Reduce the resolution of an occupancy grid map so each new cell represents ~0.5m.
If any pixel in the region is occupied (value=1), the new cell is marked as occupied.
"""

import cv2
import numpy as np
import yaml
import os
import math

# === Input map paths ===
MAP_DIR = "/home/nadavc/drone_workspace/sjtu_drone/maps"
INPUT_YAML = os.path.join(MAP_DIR, "hospital_map_cropped.yaml")

# === Desired physical cell size ===
TARGET_RES = 0.4  # meters per new cell

# === Load YAML + image ===
with open(INPUT_YAML, "r") as f:
    info = yaml.safe_load(f)

orig_res = info["resolution"]
orig_origin = info["origin"]
pgm_path = info["image"]
if not os.path.isabs(pgm_path):
    pgm_path = os.path.join(MAP_DIR, pgm_path)

img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
if img is None:
    raise FileNotFoundError(f"Failed to load {pgm_path}")

# Convert to binary occupancy: 1 = occupied, 0 = free
occ = (img < 50).astype(np.uint8)
h, w = occ.shape
print(f"[INFO] Original grid: {w}x{h} pixels, resolution {orig_res} m")

# === Compute block size (number of pixels per new cell) ===
block = int(round(TARGET_RES / orig_res))
if block < 1:
    block = 1
print(f"[INFO] Grouping {block}x{block} pixels per new cell (~{block*orig_res:.2f} m each)")

# === Pad image so it divides evenly ===
new_h = math.ceil(h / block) * block
new_w = math.ceil(w / block) * block
padded = np.zeros((new_h, new_w), dtype=np.uint8)
padded[:h, :w] = occ

# === Reshape into blocks and apply max pooling (if any occupied -> occupied) ===
reshaped = padded.reshape(new_h // block, block, new_w // block, block)
downscaled = reshaped.max(axis=(1, 3)).astype(np.uint8)

# === Save results ===
out_pgm = os.path.join(MAP_DIR, "hospital_map_downscaled.pgm")
cv2.imwrite(out_pgm, (1 - downscaled) * 255)

new_info = info.copy()
new_info["image"] = os.path.basename(out_pgm)
new_info["resolution"] = block * orig_res  # new resolution
new_info["origin"] = orig_origin  # same origin

out_yaml = os.path.join(MAP_DIR, "hospital_map_downscaled.yaml")
with open(out_yaml, "w") as f:
    yaml.dump(new_info, f, sort_keys=False)

print(f"[INFO] Saved downscaled map: {out_pgm}")
print(f"[INFO] New resolution: {new_info['resolution']:.2f} m per cell")
print(f"[INFO] New grid size: {downscaled.shape[1]}x{downscaled.shape[0]}")
