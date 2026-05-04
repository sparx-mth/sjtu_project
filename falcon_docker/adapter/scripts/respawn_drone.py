#!/usr/bin/env python3
"""
respawn_drone.py — teleport the simulated drone to a random valid pose.

Runs on the HOST or inside the FALCON container (with /var/run/docker.sock
mounted). Picks a random pose inside the environment's valid-region box,
then teleports the drone in Gazebo. Tries four methods in order:

    1. reset + gz teleport     — ROBUST. Sends /reset to clear plugin
                                  state (including flipped orientation),
                                  pauses physics, gz-teleports, unpauses,
                                  re-arms with /takeoff. Works even when
                                  the drone has flipped, wedged in a wall,
                                  or otherwise lost flight capability.
                                  THIS IS THE ONE THAT ACTUALLY WORKS.
    2. sjtu_drone posctrl mode — Smoother (drone flies to target). Only
                                  works when drone is upright and
                                  flight-capable, won't recover a flipped
                                  drone.
    3. ros2 /gazebo/set_entity_state — works for sims that load
                                       libgazebo_ros_state. Fights
                                       sjtu_drone's plugin.
    4. gz model -p (no reset)  — last-resort CLI fallback. Won't stick
                                  on sjtu_drone for the same reason.

Per-environment valid regions live in the VALID_REGIONS dict below,
drawn from each <env>.yaml's box_min_*/box_max_* fields with a 1 m
border so the random pose is never right next to a wall.

Usage
═════
    # Random pose inside the playground's valid region (auto method)
    ./respawn_drone.py playground

    # Specific pose, force reset+teleport
    ./respawn_drone.py playground --x 2 --y 3 --z 1 --yaw 1.57 --method reset

    # Custom drone model name + sim container name
    ./respawn_drone.py hospital --drone-name simple_drone --sim-container sjtu_drone_hospital

The script exits 0 on success, non-zero on failure. batch_runner.py uses
that exit code to decide whether the respawn actually took effect.
"""

import argparse
import math
import random
import subprocess
import sys
import time

# Per-env valid spawn regions in WORLD coordinates (meters, FLU). Drawn
# from each <env>.yaml's box_min_*/box_max_*, shrunk by ~1 m so the
# random pose isn't pinned against a wall. z is fixed at 1 m (drone
# starts hovering); yaw is randomized over [-π, π).
#
# Add new envs here as you bring them online.
VALID_REGIONS = {
    "playground":      dict(x=(-9.0,  9.0), y=(-9.0,  9.0), z=1.0),
    "small_house":     dict(x=(-7.0,  7.0), y=(-7.0,  7.0), z=1.0),
    "hospital":        dict(x=(-15.0, 15.0), y=(-15.0, 15.0), z=1.0),
    "bookstore":       dict(x=(-5.0,  5.0), y=(-3.0,  3.0), z=1.0),
    "small_warehouse": dict(x=(-7.0,  7.0), y=(-5.0,  5.0), z=1.0),
}


def pick_random_pose(env_name):
    """Return (x, y, z, yaw) inside the env's valid region."""
    if env_name not in VALID_REGIONS:
        raise SystemExit(
            f"unknown env '{env_name}' — add it to VALID_REGIONS in "
            f"{__file__} (or pass --x/--y/--z/--yaw directly)")
    r = VALID_REGIONS[env_name]
    return (
        random.uniform(*r["x"]),
        random.uniform(*r["y"]),
        r["z"],
        random.uniform(-math.pi, math.pi),
    )


# ─────────────── Method 1: reset + gz model teleport ─────────────────────

def respawn_via_reset_and_teleport(drone_name, sim_container,
                                    x, y, z, yaw, timeout=15):
    """
    Teleport the drone to a target pose, leaving it idle on the ground.
    FALCON's adapter handles takeoff during its own startup.

    Sequence:
      1. /reset                  → clears flipped/wedged plugin state
      2. /land                   → drone enters landed state, no residual
                                   commands
      3. gz world -p 1           → pause physics
      4. gz model -m ... -x -y   → snap to target xy/yaw
      5. gz world -p 0           → unpause

    The /reset is the critical step — it clears the plugin's flight
    state, including any flipped orientation that prevented earlier
    approaches from working.

    Returns True if the drone ends up within 1m of the target xy.
    """
    ns = f"/{drone_name}"
    # Gazebo's model name in the world is "/simple_drone" (with the
    # leading slash). The leading slash is REQUIRED.
    gz_model_name = ns if drone_name.startswith("/") else f"/{drone_name}"

    # Sequence:
    #   1. /reset      → clears flipped/wedged plugin state
    #   2. /land       → ensures drone is in landed state, no residual
    #                    velocity commands left in flight
    #   3. pause + gz model + unpause → snap to target xy/yaw
    #
    # NO takeoff here — FALCON's adapter handles takeoff as part of its
    # normal startup. This gives FALCON a clean, idle drone at the new
    # spawn point.
    bash_script = f"""
        source /opt/ros/humble/setup.bash 2>/dev/null
        ros2 topic pub --once {ns}/reset std_msgs/msg/Empty '{{}}' >/dev/null 2>&1
        sleep 0.5
        ros2 topic pub --once {ns}/land std_msgs/msg/Empty '{{}}' >/dev/null 2>&1
        sleep 0.5
        gz world -p 1 >/dev/null 2>&1
        gz model -m '{gz_model_name}' -x {x:.3f} -y {y:.3f} -z {z:.3f} -Y {yaw:.6f} >/dev/null 2>&1
        sleep 0.3
        gz world -p 0 >/dev/null 2>&1
    """

    cmd = ["docker", "exec", sim_container, "bash", "-c", bash_script]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"[respawn] reset+teleport timed out after {timeout}s",
              file=sys.stderr)
        return False
    except FileNotFoundError:
        print("[respawn] docker not on PATH", file=sys.stderr)
        return False

    if result.returncode != 0:
        print(f"[respawn] reset+teleport failed (rc={result.returncode})",
              file=sys.stderr)
        return False

    # Verify: read /gt_pose and check distance to target.
    time.sleep(1.0)
    verify_cmd = ["docker", "exec", sim_container, "bash", "-c",
                  "source /opt/ros/humble/setup.bash 2>/dev/null && "
                  f"ros2 topic echo --once {ns}/gt_pose --field position"]
    try:
        verify = subprocess.run(verify_cmd, capture_output=True, text=True,
                                 timeout=5)
    except subprocess.TimeoutExpired:
        return False

    cur = {}
    for line in verify.stdout.splitlines():
        line = line.strip()
        if line.startswith(("x:", "y:", "z:")):
            k, v = line.split(":", 1)
            try:
                cur[k] = float(v)
            except ValueError:
                pass
    if {"x", "y"}.issubset(cur):
        # Only check x/y — z drifts because we teleport above ground and
        # gravity may pull the drone down a bit before plugin re-arms.
        dx = cur["x"] - x; dy = cur["y"] - y
        dist = (dx * dx + dy * dy) ** 0.5
        if dist < 1.0:
            print(f"  [respawn] drone at ({cur['x']:.2f}, {cur['y']:.2f}) "
                  f"— {dist:.2f}m from target xy")
            return True
        print(f"[respawn] teleport didn't stick — drone at "
              f"({cur['x']:.2f}, {cur['y']:.2f}), {dist:.2f}m from target",
              file=sys.stderr)
        return False
    print("[respawn] could not read /gt_pose to verify", file=sys.stderr)
    return False


# ─────────────── Method 2: ros2 set_entity_state ─────────────────────────

def respawn_via_ros2_service(drone_name, sim_container, x, y, z, yaw,
                             timeout=10):
    """
    Call /gazebo/set_entity_state from inside the sim container so we
    don't need ROS2 installed on the host. Returns True on success.

    Requires the world file to load libgazebo_ros_state.so. Most AWS
    RoboMaker worlds do; the sjtu_drone playground does too.
    """
    # Build the YAML payload as a single line. Quaternion: yaw-only,
    # so qz = sin(yaw/2), qw = cos(yaw/2), qx = qy = 0.
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    state_yaml = (
        f"{{state: {{name: '{drone_name}', "
        f"pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}}}, "
        f"twist: {{linear: {{x: 0, y: 0, z: 0}}, "
        f"angular: {{x: 0, y: 0, z: 0}}}}, "
        f"reference_frame: 'world'}}}}"
    )
    # The container's run.sh sources the workspace setup, so that's where
    # gazebo_msgs is reachable. We keep the call short — if it doesn't
    # respond in `timeout` seconds the service almost certainly isn't
    # registered (no ros_state plugin) and we fall back to gz CLI.
    cmd = [
        "docker", "exec", sim_container, "bash", "-c",
        "source /opt/ros/humble/setup.bash 2>/dev/null && "
        "ros2 service call /gazebo/set_entity_state "
        "gazebo_msgs/srv/SetEntityState "
        f"\"{state_yaml}\""
    ]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"[respawn] ros2 service call timed out after {timeout}s",
              file=sys.stderr)
        return False
    except FileNotFoundError:
        print("[respawn] docker not on PATH", file=sys.stderr)
        return False

    out = (result.stdout or "") + (result.stderr or "")
    if "success=True" in out or "success: True" in out:
        return True
    if "service not available" in out.lower() or \
       "service is not available" in out.lower():
        return False
    # Some ros2 builds print the response without the success= prefix —
    # treat zero exit + no obvious error as success.
    if result.returncode == 0 and "ERROR" not in out.upper():
        return True
    print(f"[respawn] ros2 service call output:\n{out.strip()}",
          file=sys.stderr)
    return False


# ─────────────── Method 3: gz model CLI (no reset) ──────────────────────

def respawn_via_gz_cli(drone_name, sim_container, x, y, z, yaw, timeout=10):
    """
    Use Gazebo Classic's `gz` CLI to teleport the drone — pause physics,
    set pose, briefly wait, unpause. Bypasses ROS entirely.

    NOTE: For sjtu_drone specifically, this method DOES NOT WORK reliably.
    The drone plugin runs its own internal PID on a hidden setpoint and
    snaps the drone back as soon as physics resumes. Kept here as a
    fallback for environments without sjtu_drone's posctrl interface.
    """
    gz_model_name = drone_name if drone_name.startswith("/") else f"/{drone_name}"
    bash_script = (
        f"gz world -p 1 && "
        f"gz model -m '{gz_model_name}' -x {x:.3f} -y {y:.3f} -z {z:.3f} -Y {yaw:.6f} && "
        f"sleep 0.3 && "
        f"gz world -p 0"
    )
    cmd = ["docker", "exec", sim_container, "bash", "-c", bash_script]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"[respawn] gz teleport sequence timed out after {timeout}s",
              file=sys.stderr)
        return False
    except FileNotFoundError:
        print("[respawn] docker not on PATH", file=sys.stderr)
        return False
    if result.returncode == 0:
        return True
    print(f"[respawn] gz teleport failed (rc={result.returncode}):\n"
          f"{(result.stdout or '') + (result.stderr or '')}",
          file=sys.stderr)
    return False


# ─────────────── Method 4: sjtu_drone posctrl mode ───────────────────────

def respawn_via_sjtu_posctrl(drone_name, sim_container, x, y, z, yaw,
                              timeout=30):
    """
    Use sjtu_drone's NATIVE position-control mode to teleport. The drone
    has its own PID-based flight controller that holds whatever target
    it last received — meaning gz teleport is futile (the plugin snaps
    the drone back to its last setpoint within milliseconds).

    Per the sjtu_drone README:
      ~/posctrl [Bool]: toggling between position control (give drone
                        a pose via cmd_vel) and normal velocity control
      ~/cmd_vel [Twist]: in posctrl mode, .linear is interpreted as a
                        target POSITION (not velocity); .angular.z is
                        the target yaw

    Sequence:
      1. Publish posctrl=True
      2. Publish cmd_vel with target xyz + yaw
      3. Wait for the drone to arrive (poll /gt_pose)
      4. Publish posctrl=False so FALCON's velocity commands work after

    This is the RIGHT way to teleport an sjtu_drone — no fighting plugins,
    no physics races, the drone flies smoothly to the target via its
    own internal controller.

    Returns True if the drone reached within 0.5 m of the target.
    """
    ns = f"/{drone_name}"  # sjtu_drone namespaces all its topics

    # All ros2 calls happen inside the sim container so we don't need
    # ros2 on the host. Shell wrapper:
    def ros2_in_sim(cmd_str, exec_timeout=8):
        full = ["docker", "exec", sim_container, "bash", "-c",
                "source /opt/ros/humble/setup.bash 2>/dev/null && " + cmd_str]
        try:
            return subprocess.run(full, capture_output=True, text=True,
                                   timeout=exec_timeout)
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            return e

    # Step 0: ensure the drone is FLYING. The plugin ignores posctrl/
    # cmd_vel when in the landed state (state=0). On the very first
    # respawn (before any FALCON run), the drone has just spawned and
    # is sitting on the ground. We need to send /takeoff first.
    #
    # State semantics from sjtu_drone:
    #   0 = landed, 1 = flying, 2 = hovering
    state_val = None
    r = ros2_in_sim(
        f"ros2 topic echo --once {ns}/state --field data",
        exec_timeout=4)
    if isinstance(r, subprocess.CompletedProcess) and r.returncode == 0:
        try:
            state_val = int(r.stdout.strip())
        except ValueError:
            pass

    if state_val == 0 or state_val is None:
        # Either we KNOW it's landed, or we couldn't read the state
        # (in which case taking off is harmless — the takeoff topic is
        # idempotent if already flying).
        print(f"[respawn] drone state={state_val} → sending takeoff first")
        ros2_in_sim(
            f"ros2 topic pub --once {ns}/takeoff std_msgs/msg/Empty '{{}}'",
            exec_timeout=5)
        # Wait for the takeoff to complete — the plugin needs a few
        # seconds to lift off and stabilise before it'll respond to
        # position targets. Poll /state until it leaves landed (0).
        deadline = time.time() + 8.0
        while time.time() < deadline:
            time.sleep(0.5)
            r = ros2_in_sim(
                f"ros2 topic echo --once {ns}/state --field data",
                exec_timeout=3)
            if isinstance(r, subprocess.CompletedProcess) and r.returncode == 0:
                try:
                    if int(r.stdout.strip()) != 0:
                        break
                except ValueError:
                    pass
        time.sleep(1.5)  # give the controller a moment to settle

    # Step 1: enable position-control mode (latched-style: --once is
    # fine because the plugin captures the new mode immediately).
    r = ros2_in_sim(
        f"ros2 topic pub --once {ns}/posctrl std_msgs/msg/Bool "
        f"\"{{data: true}}\"")
    if not isinstance(r, subprocess.CompletedProcess) or r.returncode != 0:
        print("[respawn] posctrl enable failed", file=sys.stderr)
        return False

    # Step 2: publish target position as cmd_vel.
    # In posctrl mode: linear.x = target x, linear.y = y, linear.z = z,
    # angular.z = target yaw. Other fields ignored.
    twist_yaml = (f"{{linear: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, "
                  f"angular: {{x: 0.0, y: 0.0, z: {yaw:.6f}}}}}")
    r = ros2_in_sim(
        f"ros2 topic pub --once {ns}/cmd_vel geometry_msgs/msg/Twist "
        f"\"{twist_yaml}\"")
    if not isinstance(r, subprocess.CompletedProcess) or r.returncode != 0:
        print("[respawn] cmd_vel position target publish failed",
              file=sys.stderr)
        return False

    # Step 3: poll /gt_pose until the drone reports arrival, or timeout.
    # We use ros2 topic echo --once; the plugin publishes gt_pose at
    # ~30Hz so a fresh sample is ~always available.
    deadline = time.time() + timeout
    arrived = False
    last_dist = None
    while time.time() < deadline:
        r = ros2_in_sim(
            f"ros2 topic echo --once {ns}/gt_pose --field position",
            exec_timeout=4)
        if not isinstance(r, subprocess.CompletedProcess) or r.returncode != 0:
            time.sleep(0.5)
            continue
        # Output format from `--field position`:
        #   x: 1.234
        #   y: 5.678
        #   z: 1.000
        try:
            cur = {}
            for line in r.stdout.splitlines():
                line = line.strip()
                if line.startswith(("x:", "y:", "z:")):
                    k, v = line.split(":", 1)
                    cur[k] = float(v)
            if {"x", "y", "z"}.issubset(cur):
                dx = cur["x"] - x; dy = cur["y"] - y; dz = cur["z"] - z
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                last_dist = dist
                if dist < 0.5:
                    arrived = True
                    break
        except (ValueError, KeyError):
            pass
        time.sleep(0.4)

    # Step 4: unconditionally disable posctrl so FALCON's velocity
    # commands work for the actual run.
    ros2_in_sim(
        f"ros2 topic pub --once {ns}/posctrl std_msgs/msg/Bool "
        f"\"{{data: false}}\"")

    if arrived:
        return True
    print(f"[respawn] drone did not reach target within {timeout}s "
          f"(last distance: {last_dist:.2f}m)" if last_dist is not None
          else f"[respawn] drone did not reach target within {timeout}s "
               "(could not read /gt_pose)",
          file=sys.stderr)
    return False


# ─────────────── Main ────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("env", help="Environment name (key in VALID_REGIONS)")
    p.add_argument("--x",   type=float)
    p.add_argument("--y",   type=float)
    p.add_argument("--z",   type=float)
    p.add_argument("--yaw", type=float)
    p.add_argument("--seed", type=int,
                   help="Random seed for reproducible pose selection")
    p.add_argument("--drone-name", default="simple_drone",
                   help="Gazebo model name (default: simple_drone)")
    p.add_argument("--sim-container", default=None,
                   help="Docker container name for the sim (default: "
                        "sjtu_drone_<env>)")
    p.add_argument("--method",
                   choices=("auto", "reset", "posctrl", "ros2", "gz"),
                   default="auto",
                   help="Force a specific respawn method (default: auto). "
                        "Order tried in auto: reset → posctrl → ros2 → gz. "
                        "'reset' is the ROBUST method — works even when the "
                        "drone has flipped or wedged. 'posctrl' is smoother "
                        "but only works when the drone is upright and "
                        "flight-capable.")
    args = p.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    if all(v is not None for v in (args.x, args.y, args.z, args.yaw)):
        x, y, z, yaw = args.x, args.y, args.z, args.yaw
    else:
        x, y, z, yaw = pick_random_pose(args.env)

    sim_container = args.sim_container or f"sjtu_drone_{args.env}"

    print(f"[respawn] env={args.env}  pose=({x:.2f}, {y:.2f}, {z:.2f}, "
          f"yaw={math.degrees(yaw):.1f}°)  container={sim_container}")

    ok = False

    # 1. reset + gz teleport — the ROBUST method. Works even when the
    #    drone has flipped, wedged in a wall, or otherwise lost flight
    #    capability. Calls /reset to clear the plugin's broken state,
    #    pauses physics, gz-teleports, unpauses, sends takeoff.
    if args.method in ("auto", "reset"):
        print("[respawn] trying reset + gz teleport ...")
        ok = respawn_via_reset_and_teleport(args.drone_name, sim_container,
                                             x, y, z, yaw)
        if ok:
            print("[respawn] ✓ reset+teleport succeeded")

    # 2. posctrl mode — sjtu_drone's smooth-flight teleport. Only works
    #    when the drone is upright and flight-capable. Won't recover a
    #    flipped drone.
    if not ok and args.method in ("auto", "posctrl"):
        print("[respawn] trying sjtu_drone posctrl mode ...")
        ok = respawn_via_sjtu_posctrl(args.drone_name, sim_container,
                                       x, y, z, yaw)
        if ok:
            print("[respawn] ✓ sjtu_drone posctrl succeeded")

    # 3. ros2 set_entity_state — non-disruptive teleport for any sim
    #    that loads libgazebo_ros_state. Fights sjtu_drone's plugin.
    if not ok and args.method in ("auto", "ros2"):
        print("[respawn] trying ros2 set_entity_state ...")
        ok = respawn_via_ros2_service(args.drone_name, sim_container,
                                       x, y, z, yaw)
        if ok:
            print("[respawn] ✓ ros2 set_entity_state succeeded")

    # 4. gz CLI without reset — last-resort, won't stick on sjtu_drone
    #    but useful for other drones / sims.
    if not ok and args.method in ("auto", "gz"):
        print("[respawn] trying gz model CLI (no reset) ...")
        ok = respawn_via_gz_cli(args.drone_name, sim_container,
                                 x, y, z, yaw)
        if ok:
            print("[respawn] ✓ gz model succeeded "
                  "(may not stick on sjtu_drone)")

    if not ok:
        print("[respawn] ✗ all methods failed", file=sys.stderr)
        sys.exit(1)

    # Give Gazebo a beat to apply the pose change before the caller
    # starts FALCON. Without this, the first /gt_pose message FALCON
    # sees can be the OLD pose, leaving its initial map state confused.
    time.sleep(2.0)
    print("[respawn] done.")


if __name__ == "__main__":
    main()