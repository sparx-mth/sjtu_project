#!/usr/bin/env python3
"""
navdp_client.py — thin wrapper around the NavDP pointgoal server.

The user already has a working HTTP transport to NavDP, so this file
only owns the *contract*, not the networking. Everything above this
file is frame-agnostic and does not care how bytes get to the GPU box.

NavDP pointgoal contract (from InternRobotics/NavDP, eval_*_wheeled.py):

    pointgoal_step(goal, rgb, depth, port) ->
        ( trajectory_points_camera,    # (T, 2) chosen path, ROBOT frame
          all_trajectories_camera,     # (K, T, 2) all candidates
          all_values_camera,           # (K,) critic scores
          sub_pointgoal_pd )           # (2,) network's own sub-goal est.

  * goal  : [x, y, z] POINTGOAL in the ROBOT/body frame (z ignored).
  * rgb   : HxWx3 uint8.
  * depth : HxW float32 metres.
  * The returned trajectory is 24 relative (Δx, Δy) steps expressed in
    the robot frame *at the moment of inference* — i.e. the drone is
    (0,0) heading +x for that trajectory. That is exactly the
    "telemetry is my new zero every inference" semantics the user
    wants: each inference's trajectory is self-anchored.

`infer()` returns an (N, 2) float32 array of body-frame waypoints,
already in the convention nav_geom uses (x forward, y left).

Replace the body of `_call_navdp()` with the real HTTP call. The
default implementation posts JSON to ~navdp_url and expects
{"trajectory": [[x,y], ...]} back; adjust to match your server.
"""
import json
import numpy as np


class NavDPError(RuntimeError):
    pass


class NavDPClient:
    def __init__(self, url, timeout_s=2.0, logger=None):
        self.url = url
        self.timeout_s = float(timeout_s)
        self._log = logger or (lambda *a, **k: None)

    # ── public API ──────────────────────────────────────────────────
    def infer(self, pointgoal_xy, rgb, depth):
        """pointgoal_xy : (gx, gy) goal in the body frame.
           rgb          : HxWx3 uint8
           depth        : HxW   float32 (metres)
        Returns (N,2) float32 body-frame trajectory. Raises NavDPError
        on any transport/decoding failure so the caller can hold position."""
        gx, gy = float(pointgoal_xy[0]), float(pointgoal_xy[1])
        try:
            traj = self._call_navdp([gx, gy, 0.0], rgb, depth)
        except Exception as e:                       # noqa: BLE001
            raise NavDPError("NavDP transport failed: %s" % e)
        traj = np.asarray(traj, dtype=np.float32)
        if traj.ndim != 2 or traj.shape[1] < 2 or traj.shape[0] == 0:
            raise NavDPError("bad NavDP trajectory shape %r" % (traj.shape,))
        return traj[:, :2].copy()

    # ── the ONE integration point ───────────────────────────────────
    def _call_navdp(self, goal_xyz, rgb, depth):
        """The only function that knows the wire format. The user said
        "assume I have a working HTTP request" — so this is deliberately
        a single, swappable seam. Default: JSON POST.

        Expected response JSON:
            {"trajectory": [[x0,y0], [x1,y1], ... 24 points ...]}
        all in the ROBOT frame relative to the current pose.
        """
        import urllib.request

        payload = json.dumps({
            "goal": goal_xyz,
            "rgb": _png_b64(rgb),
            "depth": _depth_b64(depth),
        }).encode("utf-8")
        req = urllib.request.Request(
            self.url, data=payload,
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout_s) as r:
            resp = json.loads(r.read().decode("utf-8"))
        return resp["trajectory"]


# ── encoding helpers (kept here so the contract is self-contained) ──
def _png_b64(rgb):
    import base64
    import io
    try:
        import cv2
        ok, buf = cv2.imencode(".png", rgb[:, :, ::-1])  # RGB->BGR
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return base64.b64encode(buf.tobytes()).decode("ascii")
    except ImportError:
        from PIL import Image
        bio = io.BytesIO()
        Image.fromarray(rgb).save(bio, format="PNG")
        return base64.b64encode(bio.getvalue()).decode("ascii")


def _depth_b64(depth):
    import base64
    d = np.ascontiguousarray(depth, dtype=np.float32)
    return base64.b64encode(d.tobytes()).decode("ascii")