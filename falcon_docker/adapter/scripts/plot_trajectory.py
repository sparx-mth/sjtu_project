#!/usr/bin/env python3
"""
plot_trajectory_ros1.py - 2D drone trajectory plotter with heading (ROS 1 / rospy).

Subscribes to a PoseStamped localization topic (frame_id "world") and
draws the flight path in the XY plane: every pose is a point, connected
by a straight line to the previous one. A red arrow at the current
position shows the drone's yaw (heading). The path starts at (0, 0).
The grid is fine-grained: labeled lines every GRID_MAJOR m, faint lines
every GRID_MINOR m.

Run inside FALCON's Noetic container:
    python3 src/falcon_adapter/scripts/plot_trajectory_ros1.py
"""
import math
import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

TOPIC           = "/xtend/april_tag_pose_1"
# TOPIC           = "/flow_depth/pose_est"
START_AT_ORIGIN = True    # seed the path at (0, 0)
MIN_HALF_RANGE  = 3.0     # axes span at least [-6, 6] on both x and y
MARGIN          = 0.5     # extra room if the path ever exceeds the range
ARROW_LEN       = 0.8     # yaw arrow length, in meters
GRID_MAJOR      = 1.0     # labeled gridlines every this many meters
GRID_MINOR      = 0.25    # faint gridlines every this many meters

xs, ys, yaws = ([0.0], [0.0], [0.0]) if START_AT_ORIGIN else ([], [], [])


def quat_to_yaw(x, y, z, w):
    """Yaw (rotation about world Z) from a quaternion, in radians."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def cb(msg):
    p, o = msg.pose.position, msg.pose.orientation
    xs.append(p.x)
    ys.append(p.y)
    yaws.append(quat_to_yaw(o.x, o.y, o.z, o.w))


def main():
    rospy.init_node("trajectory_plotter", anonymous=True)
    rospy.Subscriber(TOPIC, PoseStamped, cb, queue_size=200)
    rospy.loginfo("listening on %s", TOPIC)

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")                 # 1 m in x == 1 m in y, no distortion

    # fine grid: major (labeled) + minor lines at fixed metric intervals
    for axis in (ax.xaxis, ax.yaxis):
        axis.set_major_locator(MultipleLocator(GRID_MAJOR))
        axis.set_minor_locator(MultipleLocator(GRID_MINOR))
    ax.grid(True, which="major", ls="-",  lw=0.8, alpha=0.45)
    ax.grid(True, which="minor", ls=":",  lw=0.5, alpha=0.30)
    ax.tick_params(which="minor", length=0)
    ax.axhline(0, color="gray", lw=1.0)
    ax.axvline(0, color="gray", lw=1.0)

    ax.set_title("Drone trajectory (world XY)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    # one point per pose, connected by straight lines
    path,  = ax.plot([], [], "-o", color="tab:blue",  ms=3, lw=1.2, label="path")
    start, = ax.plot([0], [0], "s",  color="tab:green", ms=11, label="start (0,0)")
    cur,   = ax.plot([], [], "o",   color="tab:red",   ms=9,  label="current")
    # yaw arrow: xy units so its length is ARROW_LEN meters at any zoom
    head = ax.quiver([0], [0], [0], [0], angles="xy", scale_units="xy", scale=1,
                     color="tab:red", width=0.012, zorder=5, label="heading (yaw)")
    ax.legend(loc="upper right")

    while not rospy.is_shutdown():
        if xs:
            path.set_data(xs, ys)
            cur.set_data([xs[-1]], [ys[-1]])
            head.set_offsets([[xs[-1], ys[-1]]])
            head.set_UVC(ARROW_LEN * math.cos(yaws[-1]),
                         ARROW_LEN * math.sin(yaws[-1]))
            # fixed, square, origin-centered window; grows only if needed
            reach = max(max(abs(x) for x in xs), max(abs(y) for y in ys))
            half  = max(MIN_HALF_RANGE, reach + MARGIN)
            ax.set_xlim(-half, half)
            ax.set_ylim(-half, half)
        plt.pause(0.1)


if __name__ == "__main__":
    main()
