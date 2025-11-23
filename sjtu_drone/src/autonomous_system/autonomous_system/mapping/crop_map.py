#!/usr/bin/env python3
"""
crop_map_safe.py
----------------
Safely crop a large occupancy grid map (PGM + YAML) into a smaller region of interest.
Does NOT overwrite the original map. Creates new files with suffix '_cropped'.
"""

import os
import cv2
import yaml

# === USER CONFIG ===
MAP_DIR = "/home/nadavc/drone_workspace/sjtu_drone/maps"
INPUT_YAML = os.path.join(MAP_DIR, "hospital_map.yaml")

# Define crop rectangle (in pixels)
# You can adjust these once you know where your area of interest is.
# Example: keep a 1200x1200 area around coordinates (x1,y1) = (800, 900)
x1, y1 = 680, 475
x2, y2 = 1320, 1875

# === LOAD YAML ===
with open(INPUT_YAML, 'r') as f:
    info = yaml.safe_load(f)

input_pgm = info['image']
if not os.path.isabs(input_pgm):
    input_pgm = os.path.join(MAP_DIR, input_pgm)

if not os.path.exists(input_pgm):
    raise FileNotFoundError(f"Cannot find map image: {input_pgm}")

print(f"[INFO] Loading map from: {input_pgm}")
img = cv2.imread(input_pgm, cv2.IMREAD_UNCHANGED)
if img is None:
    raise RuntimeError("Failed to read map image (check PGM format).")

h, w = img.shape
print(f"[INFO] Original map size: {w}x{h}")

# === BOUND CHECK ===
x1 = max(0, min(x1, w))
x2 = max(0, min(x2, w))
y1 = max(0, min(y1, h))
y2 = max(0, min(y2, h))
if x2 <= x1 or y2 <= y1:
    raise ValueError("Invalid crop coordinates")

# === CROP ===
cropped = img[y1:y2, x1:x2]
print(f"[INFO] Cropped region: x=[{x1},{x2}], y=[{y1},{y2}] → size {cropped.shape[1]}x{cropped.shape[0]}")

# === SAVE NEW MAP FILES ===
base_name = os.path.splitext(os.path.basename(info['image']))[0]
out_pgm = os.path.join(MAP_DIR, f"{base_name}_cropped.pgm")
out_yaml = os.path.join(MAP_DIR, f"{base_name}_cropped.yaml")

cv2.imwrite(out_pgm, cropped)
print(f"[INFO] Saved cropped PGM: {out_pgm}")

# === UPDATE YAML ===
res = info['resolution']
ox, oy, oz = info['origin']
# Adjust origin: move it by crop offset in world coordinates
new_ox = ox + x1 * res
new_oy = oy + y1 * res

new_info = info.copy()
new_info['image'] = os.path.basename(out_pgm)
new_info['origin'] = [float(new_ox), float(new_oy), float(oz)]

with open(out_yaml, 'w') as f:
    yaml.dump(new_info, f, sort_keys=False)

print(f"[INFO] Saved cropped YAML: {out_yaml}")
print(f"[INFO] New origin: {new_info['origin']}")
