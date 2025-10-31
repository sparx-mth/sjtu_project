#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
45° rotation with 3-phase slow corrections and angle logging
------------------------------------------------------------
←  : +45° (CCW)
→  : −45° (CW)
Space : STOP

Phases:
1) coarse move (slow)
2) correction #1 (slower)
3) correction #2 (slowest)
Prints yaw after each phase.
"""
import rclpy, math, time, tkinter as tk
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose


def yaw_from_quat(q):
    siny = 2.0 * (q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)


def ang_norm(a):  # normalize to [-pi, pi)
    return (a + math.pi) % (2 * math.pi) - math.pi


class YawStepper45(Node):
    def __init__(self):
        super().__init__('yaw_stepper_45')
        self.pose_topic = '/simple_drone/gt_pose'
        self.cmd_topic  = '/simple_drone/cmd_vel'
        self.yaw = None

        self.create_subscription(Pose, self.pose_topic, self.cb_pose, 10)
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # parameters
        self.dt = 0.02             # 50 Hz control
        self.step = math.radians(45.0)
        self.w1 = 0.3              # phase 1 angular speed
        self.w2 = 0.15             # correction 1
        self.w3 = 0.08             # correction 2
        self.tol1 = math.radians(2.0)
        self.tol2 = math.radians(0.5)
        self.tol3 = math.radians(0.15)

        # UI
        self.root = tk.Tk()
        self.root.title('Yaw 45° Stepper – triple correction')
        self.msg = tk.StringVar(value='Ready')
        tk.Label(self.root, text='← +45° | → −45° | Space STOP', font=('Arial', 12)).pack(pady=10)
        tk.Label(self.root, textvariable=self.msg, font=('Consolas', 10)).pack()
        self.root.bind('<Key>', self.on_key)
        self.root.after(40, self.spin)
        self.root.mainloop()

    def cb_pose(self, m: Pose):
        self.yaw = yaw_from_quat(m.orientation)

    def publish_stop(self, n=10):
        z = Twist()
        for _ in range(n):
            self.pub.publish(z)
            time.sleep(0.01)

    def rotate(self, sign):
        if self.yaw is None:
            self.msg.set('Waiting for pose…')
            return

        yaw_start = self.yaw
        target = ang_norm(yaw_start + sign * self.step)
        self.msg.set(f"Rotate to {math.degrees(target):.1f}°")
        start_t = time.time()

        def phase(speed, tol, label):
            """Rotate at constant speed until |error| < tol"""
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                err = ang_norm(target - self.yaw)
                if abs(err) < tol:
                    break
                tw = Twist()
                tw.angular.z = speed if err > 0 else -speed
                self.pub.publish(tw)
                time.sleep(self.dt)
            self.publish_stop(10)
            time.sleep(0.1)
            print(f"[{label}] yaw={math.degrees(self.yaw):.2f}°, err={math.degrees(ang_norm(target - self.yaw)):.2f}°")

        print(f"--- Starting rotation ---")
        print(f"Initial yaw = {math.degrees(yaw_start):.2f}°  Target = {math.degrees(target):.2f}°")

        # 1. coarse
        phase(self.w1, self.tol1, "after coarse move")

        # 2. correction #1
        phase(self.w2, self.tol2, "after correction #1")

        # 3. correction #2
        phase(self.w3, self.tol3, "after correction #2 (final)")

        yaw_final = self.yaw
        total_t = time.time() - start_t
        final_err = math.degrees(ang_norm(target - yaw_final))
        print(f"Final yaw = {math.degrees(yaw_final):.2f}°, total time={total_t:.2f}s, final_err={final_err:.3f}°")
        self.msg.set(f"Done. yaw={math.degrees(yaw_final):.2f}°, err={final_err:.2f}°")

    def on_key(self, e):
        if e.keysym == 'Left':
            self.rotate(+1)
        elif e.keysym == 'Right':
            self.rotate(-1)
        elif e.keysym == 'space':
            self.publish_stop()
            self.msg.set('STOP')

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(40, self.spin)


def main():
    rclpy.init()
    try:
        YawStepper45()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
