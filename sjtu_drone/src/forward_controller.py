#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Forward 0.05 m controller with double correction and verified full stop.
"""
import rclpy
import math
import time
import tkinter as tk
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ForwardController(Node):
    def __init__(self):
        super().__init__('forward_controller')
        self.pose = None
        self.yaw = 0.0
        self.create_subscription(Pose, '/simple_drone/gt_pose', self.cb_pose, 10)
        self.pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)

        # Parameters
        self.dt = 0.02
        self.step = 0.2  # much smaller distance
        self.v1, self.v2, self.v3 = 0.3, 0.15, 0.0075
        self.tol1, self.tol2, self.tol3 = 0.01, 0.006, 0.003

        # GUI
        self.root = tk.Tk()
        self.root.title('Forward Precise Move')
        self.msg = tk.StringVar(value='Ready')
        tk.Label(self.root, text='↑ move 0.05m | Space STOP', font=('Arial', 12)).pack(pady=10)
        tk.Label(self.root, textvariable=self.msg, font=('Consolas', 10)).pack()
        self.root.bind('<Key>', self.on_key)
        self.root.after(40, self.spin)
        self.root.mainloop()

    def cb_pose(self, msg: Pose):
        self.pose = msg.position
        self.yaw = yaw_from_quat(msg.orientation)

    def stop_until_still(self, timeout=2.0):
        """Send zero Twist until drone movement stops."""
        z = Twist()
        if not self.pose:
            return
        last_x, last_y = self.pose.x, self.pose.y
        start = time.time()
        print(f"[STOP] Start enforced stop for up to {timeout:.1f}s")
        while rclpy.ok() and (time.time() - start < timeout):
            self.pub.publish(z)
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.02)

            # measure speed by position delta
            dx = self.pose.x - last_x
            dy = self.pose.y - last_y
            speed = math.hypot(dx, dy) / 0.02
            if abs(speed) < 0.005:  # ~0.5 cm/s threshold
                print(f"[STOP] Stopped. residual speed={speed:.4f} m/s\n")
                return
            last_x, last_y = self.pose.x, self.pose.y
        print("[STOP] Timeout reached but continuing anyway\n")

    def move_forward(self):
        if not self.pose:
            self.msg.set("Waiting for pose…")
            return

        sx, sy = self.pose.x, self.pose.y
        yaw = self.yaw
        tx, ty = sx + math.cos(yaw) * self.step, sy + math.sin(yaw) * self.step
        print("=" * 60)
        print(f"START move: ({sx:.3f},{sy:.3f}) → ({tx:.3f},{ty:.3f}) yaw={math.degrees(yaw):.1f}°")

        def phase(v, tol, label):
            print(f"\n>>> {label.upper()} speed={v:.2f} tol={tol:.3f}")
            last_pose = (self.pose.x, self.pose.y)
            stable = 0
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)
                dx, dy = tx - self.pose.x, ty - self.pose.y
                d = math.hypot(dx, dy)
                moved = math.hypot(self.pose.x - sx, self.pose.y - sy)
                err = self.step - moved

                # compute speed
                vx = (self.pose.x - last_pose[0]) / self.dt
                vy = (self.pose.y - last_pose[1]) / self.dt
                spd = math.hypot(vx, vy)
                last_pose = (self.pose.x, self.pose.y)

                print(f"[{label}] pos=({self.pose.x:.3f},{self.pose.y:.3f}) "
                      f"d={d:.4f} moved={moved:.4f} err={err:+.4f} v={spd:.3f}")

                if d < tol:
                    stable += 1
                else:
                    stable = 0
                if stable > 3:
                    print(f"[{label}] target reached d={d:.4f}")
                    break

                tw = Twist()
                tw.linear.x = v * math.cos(yaw)
                tw.linear.y = v * math.sin(yaw)
                self.pub.publish(tw)
                time.sleep(self.dt)

            self.stop_until_still(timeout=2.0)
            dist = math.hypot(self.pose.x - sx, self.pose.y - sy)
            print(f"[{label}] done dist={dist:.3f} err={self.step - dist:+.3f}\n")

        # main phases
        phase(self.v1, self.tol1, "coarse")
        phase(self.v2, self.tol2, "correction #1")
        phase(self.v3, self.tol3, "correction #2 (final)")

        dist = math.hypot(self.pose.x - sx, self.pose.y - sy)
        print(f"FINAL dist={dist:.3f} err={self.step - dist:+.3f}\n")
        print("=" * 60)
        self.msg.set(f"Done: {dist:.3f} m")

    def on_key(self, e):
        if e.keysym == 'Up':
            self.move_forward()
        elif e.keysym == 'space':
            self.stop_until_still()
            self.msg.set("STOP")

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(40, self.spin)


def main():
    rclpy.init()
    try:
        ForwardController()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
