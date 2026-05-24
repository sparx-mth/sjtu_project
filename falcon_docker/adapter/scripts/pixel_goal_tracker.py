#!/usr/bin/env python3
"""
pixel_goal_tracker.py — keep the POINTGOAL fresh WITHOUT trusting
drifting localization.

Problem (user's words): the first time we switch to VISUAL_SERVOING the
pose is still accurate, so we can turn the furthest visible A* waypoint
into a body-frame goal. But once we start moving the localization drifts
badly, so recomputing "how far is the goal now" from the pose is wrong.

Idea (user's plan): on the FIRST RGB-D frame, lock onto the small group
of pixels we are aiming at and remember their depth. On every later
frame, re-find that same patch, read its new depth, and back-project it
into the *current* body frame. That relative vector is the new
pointgoal — derived from vision only, immune to pose drift.

This class is pure CV + geometry. No ROS.

  lock(rgb, depth, goal_body, K, T_b_c)   → True/False
  update(rgb, depth, K, T_b_c)            → (gx, gy) or None
"""
import numpy as np

import nav_geom


def _to_gray(rgb):
    return (0.299 * rgb[:, :, 0] +
            0.587 * rgb[:, :, 1] +
            0.114 * rgb[:, :, 2]).astype(np.float32)


def _patch_depth(depth, u, v, half):
    """Median of finite, positive depth in a (2*half+1) box around (u,v)."""
    h, w = depth.shape
    u0, u1 = max(0, u - half), min(w, u + half + 1)
    v0, v1 = max(0, v - half), min(h, v + half + 1)
    win = depth[v0:v1, u0:u1].astype(np.float32)
    win = win[np.isfinite(win) & (win > 0.0)]
    if win.size == 0:
        return None
    return float(np.median(win))


class PixelGoalTracker:
    def __init__(self, patch=21, search=80, min_score=0.45, logger=None):
        self.patch = int(patch)            # template half-size is patch//2
        self.search = int(search)          # +/- search window in px
        self.min_score = float(min_score)  # NCC acceptance threshold
        self._log = logger or (lambda *a, **k: None)
        self._tmpl = None                  # locked grayscale template
        self._uv = None                    # last known pixel (u,v)
        self.locked = False

    # ── lock onto the goal direction in the first frame ─────────────
    def lock(self, rgb, depth, goal_body, K, T_b_c):
        """goal_body = (gx, gy[, gz]) body-frame goal. We project it to
        a pixel, clip the pixel into the image, and snapshot the patch
        there as the visual landmark to chase."""
        gx, gy = goal_body[0], goal_body[1]
        gz = goal_body[2] if len(goal_body) > 2 else 0.0
        proj = nav_geom.point_body_to_pixel(gx, gy, gz, K, T_b_c)
        h, w = depth.shape
        if proj is None:
            u, v = w // 2, h // 2          # goal not projectable → look ahead
        else:
            u, v = int(round(proj[0])), int(round(proj[1]))
        u = min(max(u, 0), w - 1)
        v = min(max(v, 0), h - 1)
        hp = self.patch // 2
        if not (hp <= u < w - hp and hp <= v < h - hp):
            self._log("pixel_goal: goal pixel too close to border (%d,%d)", u, v)
            self.locked = False
            return False
        gray = _to_gray(rgb)
        self._tmpl = gray[v - hp:v + hp + 1, u - hp:u + hp + 1].copy()
        self._uv = (u, v)
        self.locked = True
        d = _patch_depth(depth, u, v, hp)
        self._log("pixel_goal: LOCKED at px=(%d,%d) depth=%s", u, v,
                  "n/a" if d is None else "%.2fm" % d)
        return True

    # ── re-find the patch and rebuild the pointgoal ─────────────────
    def update(self, rgb, depth, K, T_b_c):
        """Return the refreshed body-frame (gx, gy) or None if the patch
        was lost / has no valid depth (caller should then fall back to
        the localization estimate)."""
        if not self.locked or self._tmpl is None:
            return None
        gray = _to_gray(rgb)
        h, w = gray.shape
        u_prev, v_prev = self._uv
        hp = self.patch // 2

        # Search only a window around the previous location — the patch
        # moves slowly between inferences and this keeps it cheap/robust.
        su0 = max(hp, u_prev - self.search)
        su1 = min(w - hp, u_prev + self.search)
        sv0 = max(hp, v_prev - self.search)
        sv1 = min(h - hp, v_prev + self.search)
        if su1 <= su0 or sv1 <= sv0:
            return None

        best = self._match(gray, su0, su1, sv0, sv1, hp)
        if best is None:
            self._log("pixel_goal: patch LOST (no match)")
            return None
        u, v, score = best
        if score < self.min_score:
            self._log("pixel_goal: weak match score=%.2f < %.2f",
                      score, self.min_score)
            return None

        d = _patch_depth(depth, u, v, hp)
        if d is None:
            self._log("pixel_goal: no valid depth at re-found patch")
            return None

        self._uv = (u, v)                  # track forward for next call
        gx, gy, _ = nav_geom.pixel_to_point_body(u, v, d, K, T_b_c)
        return (gx, gy)

    # ── matcher: cv2 NCC if available, else numpy NCC fallback ──────
    def _match(self, gray, su0, su1, sv0, sv1, hp):
        roi = gray[sv0 - hp:sv1 + hp, su0 - hp:su1 + hp]
        tmpl = self._tmpl
        try:
            import cv2
            res = cv2.matchTemplate(roi, tmpl, cv2.TM_CCOEFF_NORMED)
            _, mx, _, mloc = cv2.minMaxLoc(res)
            u = su0 + mloc[0]
            v = sv0 + mloc[1]
            return (u, v, float(mx))
        except ImportError:
            return self._match_numpy(gray, su0, su1, sv0, sv1, hp)

    def _match_numpy(self, gray, su0, su1, sv0, sv1, hp):
        t = self._tmpl
        t = t - t.mean()
        tn = np.sqrt((t * t).sum()) + 1e-6
        best, best_uv = -1.0, None
        # coarse stride keeps the pure-numpy path affordable
        step = 2
        for v in range(sv0, sv1, step):
            for u in range(su0, su1, step):
                win = gray[v - hp:v + hp + 1, u - hp:u + hp + 1]
                if win.shape != t.shape:
                    continue
                wc = win - win.mean()
                denom = (np.sqrt((wc * wc).sum()) + 1e-6) * tn
                score = float((wc * t).sum() / denom)
                if score > best:
                    best, best_uv = score, (u, v)
        if best_uv is None:
            return None
        return (best_uv[0], best_uv[1], best)