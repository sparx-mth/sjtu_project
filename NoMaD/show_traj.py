import numpy as np
import matplotlib.pyplot as plt

# load trajectory from evo
tum = np.loadtxt("simple_drone_odom.tum")
t = tum[:,0]
x = tum[:,1]
y = tum[:,2]

# goal from RViz (/goal_pose)
goal_x =  -6.004440784454346

goal_y =  6.920757293701172


plt.figure(figsize=(6,6))

# trajectory
plt.plot(x, y, linewidth=2, label="Trajectory")

# start
plt.scatter(x[0], y[0], c="green", s=80, marker="o", label="Start")

# end (actual)
plt.scatter(x[-1], y[-1], c="red", s=80, marker="x", label="End")

# goal
plt.scatter(goal_x, goal_y, c="gold", s=200, marker="*", label="Goal")

plt.axis("equal")
plt.grid(True)
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()
plt.title("Trajectory with Start & Goal")
plt.savefig(
    "trajectory_with_goal.png",
    dpi=300,
    bbox_inches="tight"
)

plt.show()