"""
trajectory_safety_corrector.py
==============================
Nudges a body-frame NavDP trajectory away from walls.

Approach
--------
Consumes the repulsive potential field U_rep published by PotentialMapperNode
(computed via PotentialFieldLayer as a Gaussian blur of the obstacle mask, so
the gradient at any point is the weighted sum of contributions from ALL nearby
walls). Correction is iterative bilinear gradient descent with a decaying
step, a total-displacement clamp to prevent replacement, and a light smoothing
pass to remove kinks. Waypoint 0 is pinned (drone's current pose).

Topics
------
  /map_local              (OccupancyGrid) — grid metadata + occupancy for viz
  /potential_field/u_rep  (Image 32FC1)   — the precomputed field
"""

import cv2
import numpy as np
from typing import Optional


class TrajectorySafetyCorrector:
    def __init__(
        self,
        n_iterations: int = 5,
        correction_gain: float = 0.6,
        step_decay: float = 0.7,
        max_correction_m: float = 0.25,
        max_total_displacement_m: float = 0.6,
        smoothing_passes: int = 2,
        pin_first_k: int = 1,
        u_floor: float = 1e-3,
        depth_scale: float = 1.73,
    ):
        self.n_iterations = n_iterations
        self.correction_gain = correction_gain
        self.step_decay = step_decay
        self.max_correction_m = max_correction_m
        self.max_total_displacement_m = max_total_displacement_m
        self.smoothing_passes = smoothing_passes
        self.pin_first_k = pin_first_k
        self.u_floor = u_floor
        self._depth_scale = depth_scale

        # Grid metadata from /map_local
        self._resolution: float = 0.0
        self._origin_x: float = 0.0
        self._origin_y: float = 0.0
        self._width: int = 0
        self._height: int = 0

        # Fields rebuilt on every /map_local
        self._occ: Optional[np.ndarray] = None       # (H,W) 0..1 occupancy
        self._U_rep: Optional[np.ndarray] = None     # (H,W) Gaussian field
        self._grad: Optional[np.ndarray] = None      # (H,W,2) descent direction

    # ------------------------------------------------------------------
    # Data ingestion
    # ------------------------------------------------------------------
    def update_grid_metadata(self, occ_grid_msg) -> None:
        """Extract metadata + raw occupancy (for visualisation) from /map_local."""
        info = occ_grid_msg.info
        self._resolution = info.resolution
        self._width = info.width
        self._height = info.height
        self._origin_x = info.origin.position.x
        self._origin_y = info.origin.position.y

        raw = np.array(occ_grid_msg.data, dtype=np.int8).reshape(
            (self._height, self._width))
        occ = raw.astype(np.float32)
        occ[occ < 0] = 0.0
        occ /= 100.0
        self._occ = occ

    def update_u_rep(self, u_rep: np.ndarray) -> None:
        """Store U_rep (built by PotentialFieldLayer) and cache its descent gradient."""
        self._U_rep = u_rep.astype(np.float32)
        self._height, self._width = u_rep.shape
        if self._resolution > 0:
            g_row, g_col = np.gradient(self._U_rep, self._resolution)
            self._grad = np.stack([-g_row, -g_col], axis=-1).astype(np.float32)

    # ------------------------------------------------------------------
    # Coordinate + bilinear sampling helpers
    # ------------------------------------------------------------------
    def _body_to_grid_f(self, fwd: float, left: float) -> tuple[float, float]:
        """Body (fwd, left) → fractional (row, col) in the published grid."""
        fwd_s = fwd * self._depth_scale
        left_s = left * self._depth_scale
        col_f = (fwd_s - self._origin_x) / self._resolution
        row_f = (left_s - self._origin_y) / self._resolution
        return row_f, col_f

    def _body_to_grid(self, fwd: float, left: float) -> tuple[int, int]:
        row_f, col_f = self._body_to_grid_f(fwd, left)
        return int(round(row_f)), int(round(col_f))

    def _bilinear(self, grid: np.ndarray, row_f: float, col_f: float):
        """Bilinear sample of a scalar or (H,W,2) vector grid. None if OOB."""
        if not (1 <= row_f < self._height - 1 and 1 <= col_f < self._width - 1):
            return None
        r0 = int(np.floor(row_f)); c0 = int(np.floor(col_f))
        dr = row_f - r0; dc = col_f - c0
        v00 = grid[r0,     c0]
        v01 = grid[r0,     c0 + 1]
        v10 = grid[r0 + 1, c0]
        v11 = grid[r0 + 1, c0 + 1]
        return ((1 - dr) * ((1 - dc) * v00 + dc * v01)
                + dr       * ((1 - dc) * v10 + dc * v11))

    # ------------------------------------------------------------------
    # Trajectory correction
    # ------------------------------------------------------------------
    def correct(self, trajectory: np.ndarray, verbose: bool = False) -> np.ndarray:
        if self._U_rep is None or self._grad is None:
            if verbose:
                print("[TrajCorrector] No field yet — returning original")
            return trajectory.copy()

        N = trajectory.shape[0]
        original = trajectory[:, :2].astype(np.float64).copy()
        corrected = trajectory.astype(np.float64).copy()
        step = 1.0

        # Iterative descent with decaying step
        for it in range(self.n_iterations):
            moved = 0.0
            for i in range(self.pin_first_k, N):
                row_f, col_f = self._body_to_grid_f(
                    float(corrected[i, 0]), float(corrected[i, 1]))

                u = self._bilinear(self._U_rep, row_f, col_f)
                if u is None or float(u) < self.u_floor:
                    continue
                g = self._bilinear(self._grad, row_f, col_f)
                if g is None:
                    continue

                push = g * self.correction_gain * step
                mag = float(np.linalg.norm(push))
                if mag < 1e-5:
                    continue
                if mag > self.max_correction_m:
                    push *= self.max_correction_m / mag
                    mag = self.max_correction_m

                # grid (row, col) → body (fwd, left):  col ↔ fwd,  row ↔ left
                corrected[i, 0] += float(push[1])
                corrected[i, 1] += float(push[0])
                moved = max(moved, mag)

                if verbose and it == 0:
                    print(f"  wp{i:2d} u={float(u):.3f} "
                          f"push=({float(push[1]):+.3f},{float(push[0]):+.3f})")

            step *= self.step_decay
            if moved < 1e-3:
                break

        # Cap total displacement per waypoint so we nudge, never replace
        delta = corrected[:, :2] - original
        dmag = np.linalg.norm(delta, axis=1)
        too_far = dmag > self.max_total_displacement_m
        if np.any(too_far):
            scale = self.max_total_displacement_m / np.maximum(dmag[too_far], 1e-9)
            corrected[too_far, :2] = original[too_far] + delta[too_far] * scale[:, None]

        # Light smoothing, endpoints pinned
        for _ in range(self.smoothing_passes):
            if N < 3:
                break
            s = corrected.copy()
            s[1:-1, :2] = (0.25 * corrected[:-2, :2]
                           + 0.5  * corrected[1:-1, :2]
                           + 0.25 * corrected[2:,  :2])
            corrected = s

        return corrected.astype(np.float32)

    # ------------------------------------------------------------------
    # Visualisation (two-panel BEV: occupancy + U_rep)
    # ------------------------------------------------------------------
    def visualize_corrections(
        self,
        original: np.ndarray,
        corrected: np.ndarray,
        pad_m: float = 1.5,
        scale: int = 4,
    ) -> Optional[np.ndarray]:
        if self._U_rep is None or self._resolution <= 0:
            return None

        N = original.shape[0]
        all_fwd = np.concatenate([original[:, 0], corrected[:, 0]])
        all_left = np.concatenate([original[:, 1], corrected[:, 1]])
        fwd_min = min(float(all_fwd.min()) - pad_m, -pad_m)
        fwd_max = float(all_fwd.max()) + pad_m
        left_min = min(float(all_left.min()) - pad_m, -pad_m)
        left_max = max(float(all_left.max()) + pad_m, pad_m)

        r_a, c_a = self._body_to_grid(fwd_min, left_min)
        r_b, c_b = self._body_to_grid(fwd_max, left_max)
        r_min = max(0, min(r_a, r_b));  r_max = min(self._height, max(r_a, r_b) + 1)
        c_min = max(0, min(c_a, c_b));  c_max = min(self._width,  max(c_a, c_b) + 1)
        if r_max <= r_min or c_max <= c_min:
            return None

        def crop_bev(grid):
            return grid[r_min:r_max, c_min:c_max].T[::-1, ::-1]  # fwd=up, left=left

        bev_urep = crop_bev(self._U_rep)
        h_bev, w_bev = bev_urep.shape
        u_ceil = max(float(bev_urep.max()), 0.01)
        norm_u = (np.clip(bev_urep / u_ceil, 0.0, 1.0) * 255).astype(np.uint8)
        panel_urep = cv2.applyColorMap(norm_u, cv2.COLORMAP_JET)
        panel_urep = cv2.resize(panel_urep, (w_bev * scale, h_bev * scale),
                                interpolation=cv2.INTER_NEAREST)

        if self._occ is not None:
            gray = (255 - np.clip(crop_bev(self._occ), 0.0, 1.0) * 255).astype(np.uint8)
            panel_occ = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            panel_occ = cv2.resize(panel_occ, (w_bev * scale, h_bev * scale),
                                   interpolation=cv2.INTER_NEAREST)
        else:
            panel_occ = np.full_like(panel_urep, 200)

        def to_px(fwd_, left_):
            row, col = self._body_to_grid(fwd_, left_)
            return (int((r_max - 1 - row + 0.5) * scale),
                    int((c_max - 1 - col + 0.5) * scale))

        for panel in (panel_occ, panel_urep):
            for i in range(N - 1):
                cv2.line(panel,
                         to_px(float(original[i, 0]),   float(original[i, 1])),
                         to_px(float(original[i+1, 0]), float(original[i+1, 1])),
                         (255, 255, 255), 1)
                cv2.line(panel,
                         to_px(float(corrected[i, 0]),   float(corrected[i, 1])),
                         to_px(float(corrected[i+1, 0]), float(corrected[i+1, 1])),
                         (0, 255, 0), 1)
            for i in range(N):
                ox, oy = to_px(float(original[i, 0]),  float(original[i, 1]))
                cx, cy = to_px(float(corrected[i, 0]), float(corrected[i, 1]))
                row, col = self._body_to_grid(float(original[i, 0]), float(original[i, 1]))
                if 0 <= row < self._height and 0 <= col < self._width:
                    g = self._grad[row, col]
                    gn = float(np.linalg.norm(g))
                    if gn > 1e-6:
                        gu = g / gn
                        cv2.arrowedLine(panel, (ox, oy),
                                        (ox - int(gu[0] * scale * 2.5),
                                         oy - int(gu[1] * scale * 2.5)),
                                        (255, 255, 0), 1, tipLength=0.4)
                if np.hypot(float(corrected[i, 0]) - float(original[i, 0]),
                            float(corrected[i, 1]) - float(original[i, 1])) > 1e-3:
                    cv2.arrowedLine(panel, (ox, oy), (cx, cy),
                                    (0, 0, 255), 2, tipLength=0.35)
                cv2.circle(panel, (ox, oy), 3, (255, 255, 255), -1)
                cv2.circle(panel, (cx, cy), 3, (0, 255, 0), -1)

            rx, ry = to_px(0.0, 0.0)
            cv2.drawMarker(panel, (rx, ry), (255, 255, 0),
                           cv2.MARKER_DIAMOND, markerSize=14, thickness=2)
            h_p, w_p = panel.shape[:2]
            cv2.putText(panel, "FWD", (w_p // 2 - 14, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.putText(panel_occ,  "OCCUPANCY", (6, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        cv2.putText(panel_urep, "U_REP (gaussian)", (6, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        vis = np.hstack([panel_occ, panel_urep])
        cv2.putText(vis,
                    f"iters={self.n_iterations} gain={self.correction_gain} "
                    f"decay={self.step_decay} "
                    f"max_step={self.max_correction_m}m max_total={self.max_total_displacement_m}m",
                    (6, vis.shape[0] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)
        return vis