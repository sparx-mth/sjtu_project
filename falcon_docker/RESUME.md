# Where the SJTU warehouse campaign was left, and what to do next

Newest first. This is the Gazebo `small_warehouse` deployment of FALCON — the
one `run_warehouse.sh` brings up. It is a different stack from the Isaac/PEGASUS
one, whose log lives in
`TheAgency/sparx_agency/tasks/planning/falcon_pegasus/RESUME.md`.

## 2026-08-12 — FIXED: FALCON was unprojecting with the wrong camera

**The symptom** was "the depth image clearly shows the floor, but no voxels are
ever built into the floor". The instinct is to blame the cruise altitude or the
visualisation box. It was neither: `small_warehouse.yaml` already spans
`vbox_min_z: -0.3 .. vbox_max_z: 2.8`, so the floor would have been drawn if it
had ever been mapped.

**The cause.** `gazebo_exploration.launch` was giving the mapper FALCON's own
reference camera instead of the drone's:

| | was configured | actual `camera_info` |
|---|---|---|
| size | 640 x 480 | **600 x 600** |
| fx, fy | 320.0 | **390.6427353027365** |
| cx, cy | 320.0, 240.0 | **300.5, 300.5** |
| fov h x v | 90 deg x 73.74 deg | **75.14 deg x 75.14 deg** |

Nothing detects this. `voxel_mapping/src/map_server.cpp` takes the image
DIMENSIONS from the incoming message and the INTRINSICS from the param server,
so the two can disagree silently and still produce a plausible-looking map.

A principal point 60.5 px too high tilts the whole cloud downward; an `fy` 22%
too small stretches that tilt with range. A floor pixel at horizontal range `R`
with the camera at height `h` came out at

    z_world = h - (0.189*R + 1.2208*h)  =  -0.221*h - 0.189*R

so the flat floor reconstructed as a **bowl sloping away from the aircraft**,
never a plane — no coherent surface to accumulate, all of it below
`vbox_min_z = -0.3` so RViz never drew it, and much of it below
`map_min_z = -1.0` where `tsdf.cpp` clamps the ray and discards it.

**Measured, on one live frame at camera height 4.76 m** (lowest 15% of returns,
i.e. whatever the camera calls "the floor"):

| intrinsics | world z of the floor band |
|---|---|
| as configured | **-2.49 .. +0.03 m** (median -0.48) |
| actual camera | **+0.05 .. +1.76 m** (median +1.33) |

**The second symptom, same cause.** `fov/horizontal` claimed 90 deg against a
true 75.14 deg, and `perception_utils` uses those angles to decide whether a
frontier cell is visible from a candidate viewpoint. FALCON therefore credited
coverage at the edges of frame for cells the camera never saw, declared the
space explored about 15 minutes in, and shut down `traj_server` — after which
the drone drifted upward uncommanded (observed climbing 4.30 -> 5.29 m). That
reads exactly like an altitude/control bug and is not one.

**After the fix**, occupied voxels spanned z -0.15 .. 2.75 with ~18,500 of
52,238 in the -0.2 .. +0.2 m band: a floor slab, where a floor belongs.

**How to re-verify in one minute** if this is ever suspected again — unproject a
live `/map_ros/depth` frame both ways and compare the floor band, rather than
reasoning about it:

```bash
docker exec falcon bash -lc 'source /opt/ros/noetic/setup.bash && \
  export ROS_MASTER_URI=http://localhost:11311 && \
  rostopic echo -n1 /simple_drone/front_depth/depth/camera_info | head -12; \
  rosparam get /uav_model/sensing_parameters'
```

The `K` matrix and the params must agree. They are two independent sources of
the same truth and nothing reconciles them.

## OPEN, and where to start next: "No path to next viewpoint"

The corrected-camera run **did not finish**. It flew 68 s, mapped 61.7 m3, then
wedged at `(2.50, -5.47, 0.73)` and the watchdog ended it as `no_progress`.
FALCON's last words, repeating:

```
[ExplorationManager] planTrajToView: No path to next viewpoint using coarse A*
[ExplorationManager] planTrajToView: No path to next viewpoint using default A*
```

Reproduce with exactly what produced it:

```bash
cd falcon_docker && RVIZ=1 ./run_warehouse.sh small_warehouse small_warehouse 600
```

(`run_warehouse.sh` defaults at the time: `INFLATE=0.15 SAFE=0.15
ASTAR_INFLATE=0.30 MAX_VEL=0.25`.)

**This is a new failure, not the old one.** Before the camera fix FALCON
wrongly believed the space was explored; now it correctly still sees frontiers
and instead cannot route to them. Note what changed underneath: the map the
planner searches is now a genuinely solid one, with a real floor and walls that
no longer lean. **Every inflation and clearance number on this stack was tuned
against the old hollow map**, so the first suspects are the ones that interact
with newly-solid geometry — `obstacles_inflation`, `astar_inflate`,
`frontier_min_clearance`, and the `box_min_z: 0.6 / box_max_z: 1.0` band the
route must stay inside. That is a hypothesis, not a diagnosis; it has not been
tested.

## Starting from a fresh machine

Nothing below is on the remote, so a new checkout needs it done once.

1. **The world repos are NOT in this repository.** `aws-robomaker-hospital-world`
   is tracked in-repo; the other three are external clones (~313 MB) and are
   gitignored. For the warehouse you need:

   ```bash
   cd "$SJTU_PROJECT_DIR"
   git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
   # optional, for the other worlds:
   git clone https://github.com/aws-robotics/aws-robomaker-bookstore-world.git
   git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git
   ```

2. **`export SJTU_PROJECT_DIR=/path/to/sjtu_project`** — TheAgency's
   `robots/SJTU/setup/env.sh` derives every other path from it and refuses to
   guess. `run_warehouse.sh` calls `bringup_world.sh` through it.

3. **Build the FALCON image**, which is what carries the C++ patches:

   ```bash
   cd falcon_docker && docker build -t falcon-ros-custom:v3 .
   ```

   This takes the better part of an hour. Note the image build OOMs at `-j32` on
   this hardware; cap the jobs if it dies.

## What is NOT in the image, and will bite you

The C++ behaviour changes are all script patches applied at build time
(`fix_falcon_*.sh` + `*.patch`), so `docker build` reproduces them. **FALCON's
configuration is not**, and it arrives three other ways:

* **Inline `sed -i` in the Dockerfile** (`Dockerfile:144`, `:150`) — the
  `multi_map_server` CMake rename and the `open3d_resource_path` rewrite. No
  script, no patch file, invisible in `patches/`.
* **`COPY` straight into FALCON's tree** (`Dockerfile:239`) — `hospital.yaml`
  lands in `exploration_manager/config/map/`.
* **Bind-mounts that overwrite FALCON files at runtime**
  (`run_warehouse.sh:119-120`) — our `small_warehouse.yaml` over FALCON's
  `config/map/`, and our `rviz_warehouse.rviz` **over FALCON's own
  `config/rviz.rviz`**. These exist only in the running container. Reading the
  Dockerfile will tell you FALCON's stock RViz config is what runs. It is not.

The practical consequence: **the camera fix above needs no rebuild.** It lives
in `adapter/launch/gazebo_exploration.launch`, which `run_warehouse.sh`
bind-mounts, so editing it and re-running is the whole loop. Only a change to a
`fix_falcon_*.sh` patch requires the hour.

## Inspecting a live run

`RVIZ=1` leaves the whole stack up after the watchdog fires, on purpose, so the
map can be read afterwards. Tear it down with:

```bash
docker rm -f falcon ros1_bridge roscore sjtu_drone_small_warehouse
```

Everything runs on `--network host` with a `roscore` on port 11311, so **only
one of these stacks can run at a time** — starting the Isaac/PEGASUS stack while
this one is up silently joins the same ROS graph and corrupts both.
