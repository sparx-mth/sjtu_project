#!/usr/bin/env python3
import math, numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

def q_to_np(q): return np.array([q.x, q.y, q.z, q.w], float)
def np_to_q(xyzqw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.x, q.y, q.z, q.w = xyzqw
    return q

def inv_tf(t: TransformStamped):
    """ invert cam->tag to get tag->cam as 4x4 """
    import numpy as np
    tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
    q = q_to_np(t.transform.rotation)
    # rotation matrix
    x, y, z, w = q
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
    ])
    tvec = np.array([tx, ty, tz])
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = tvec
    T_inv = np.eye(4)
    R_inv = R.T
    T_inv[:3,:3] = R_inv
    T_inv[:3, 3] = -R_inv @ tvec
    return T_inv

def mul_T(A,B):
    import numpy as np
    return A @ B

def T_to_pos_quat(T):
    import numpy as np
    pos = T[:3,3]
    R = T[:3,:3]
    # rotation->quat
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr+1.0)*2
        w = 0.25 * S
        x = (R[2,1]-R[1,2]) / S
        y = (R[0,2]-R[2,0]) / S
        z = (R[1,0]-R[0,1]) / S
    else:
        i = np.argmax(np.diag(R))
        if i == 0:
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])*2
            x = 0.25*S
            y = (R[0,1]+R[1,0])/S
            z = (R[0,2]+R[2,0])/S
            w = (R[2,1]-R[1,2])/S
        elif i == 1:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])*2
            x = (R[0,1]+R[1,0])/S
            y = 0.25*S
            z = (R[1,2]+R[2,1])/S
            w = (R[0,2]-R[2,0])/S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])*2
            x = (R[0,2]+R[2,0])/S
            y = (R[1,2]+R[2,1])/S
            z = 0.25*S
            w = (R[1,0]-R[0,1])/S
    quat = np.array([x,y,z,w], float)
    # normalize
    quat = quat/np.linalg.norm(quat)
    return pos, quat

def slerp(q1, q2, w=0.5):
    """ simple slerp between two unit quaternions """
    import numpy as np
    q1 = q1/np.linalg.norm(q1); q2 = q2/np.linalg.norm(q2)
    dot = np.dot(q1, q2)
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    if dot > 0.9995:
        return (q1 + w*(q2-q1)) / np.linalg.norm(q1 + w*(q2-q1))
    theta0 = math.acos(dot)
    theta = theta0*w
    q3 = q2 - q1*dot
    q3 = q3/np.linalg.norm(q3)
    return q1*math.cos(theta) + q3*math.sin(theta)

def angle_deg(q1, q2):
    dot = abs(float(np.dot(q1/np.linalg.norm(q1), q2/np.linalg.norm(q2))))
    dot = max(min(dot, 1.0), 0.0)
    return math.degrees(2.0 * math.acos(dot))

class Triang(Node):
    def __init__(self):
        super().__init__('triangulate_two_tags')
        import os
        self.world = os.environ.get('WORLD_FRAME', 'world')
        self.cam = os.environ.get('CAM_OPTICAL', 'simple_drone/front_cam_optical')

        # Tag1
        self.tag1_est = os.environ.get('TAG_EST_FRAME', 'tag36h11:14')
        self.tag1_gt  = os.environ.get('TAG1_GT_ALIGNED',  'tag36h11:14_gt')

        # Tag2
        self.tag2_est = os.environ.get('TAG2_EST_FRAME', 'tag36h11:15')
        self.tag2_gt  = os.environ.get('TAG2_GT_ALIGNED',  'tag36h11:15_gt')

        self.buf = Buffer()
        self.lst = TransformListener(self.buf, self)
        self.timer = self.create_timer(0.5, self.tick)

    def _lookup(self, target, source):
        # return TransformStamped for source->target
        return self.buf.lookup_transform(source, target, rclpy.time.Time())

    def pose_from_tag(self, cam_to_tag, world_to_tag):
        """ world_T_cam = world_T_tag * inv(cam_T_tag) """
        T_tag_cam = inv_tf(cam_to_tag)
        # world->tag
        T_world_tag = self.tf_to_mat(world_to_tag)
        T_world_cam = mul_T(T_world_tag, T_tag_cam)
        pos, quat = T_to_pos_quat(T_world_cam)
        return pos, quat, T_world_cam

    def tf_to_mat(self, t: TransformStamped):
        import numpy as np
        tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        x, y, z, w = q_to_np(t.transform.rotation)
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y - z*w),   2*(x*z + y*w)],
            [2*(x*y + z*w),   1-2*(x*x+z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
        ])
        T = np.eye(4)
        T[:3,:3] = R
        T[:3, 3] = [tx,ty,tz]
        return T

    def tick(self):
        try:
            # EST (from apriltag node)
            cam_tag1_est = self._lookup(self.tag1_est, self.cam)
            cam_tag2_est = self._lookup(self.tag2_est, self.cam)
            # GT (cam->tag_gt via TF chain)
            cam_tag1_gt  = self._lookup(self.tag1_gt,  self.cam)
            cam_tag2_gt  = self._lookup(self.tag2_gt,  self.cam)
            # World->tag GT
            world_tag1_gt = self._lookup(self.tag1_gt, self.world)
            world_tag2_gt = self._lookup(self.tag2_gt, self.world)
        except Exception:
            return

        # === Using GT cam->tag ===
        p1_gt, q1_gt, _ = self.pose_from_tag(cam_tag1_gt, world_tag1_gt)
        p2_gt, q2_gt, _ = self.pose_from_tag(cam_tag2_gt, world_tag2_gt)
        # simple weights: inverse distance to tag (closer tag -> higher weight)
        d1 = np.linalg.norm(np.array([cam_tag1_gt.transform.translation.x,
                                      cam_tag1_gt.transform.translation.y,
                                      cam_tag1_gt.transform.translation.z]))
        d2 = np.linalg.norm(np.array([cam_tag2_gt.transform.translation.x,
                                      cam_tag2_gt.transform.translation.y,
                                      cam_tag2_gt.transform.translation.z]))
        w1 = 1.0/max(d1,1e-3); w2 = 1.0/max(d2,1e-3)
        wsum = w1+w2
        p_gt = (w1*np.array(p1_gt)+w2*np.array(p2_gt))/wsum
        q_gt = slerp(np.array(q1_gt), np.array(q2_gt), w=w2/wsum)

        # === Using EST cam->tag (from detector) with world->tag GT ===
        p1_est, q1_est, _ = self.pose_from_tag(cam_tag1_est, world_tag1_gt)
        p2_est, q2_est, _ = self.pose_from_tag(cam_tag2_est, world_tag2_gt)
        d1e = np.linalg.norm(np.array([cam_tag1_est.transform.translation.x,
                                       cam_tag1_est.transform.translation.y,
                                       cam_tag1_est.transform.translation.z]))
        d2e = np.linalg.norm(np.array([cam_tag2_est.transform.translation.x,
                                       cam_tag2_est.transform.translation.y,
                                       cam_tag2_est.transform.translation.z]))
        w1e = 1.0/max(d1e,1e-3); w2e = 1.0/max(d2e,1e-3)
        wse = w1e+w2e
        p_est = (w1e*np.array(p1_est)+w2e*np.array(p2_est))/wse
        q_est = slerp(np.array(q1_est), np.array(q2_est), w=w2e/wse)

        # Diagnostics
        pos_err = float(np.linalg.norm(p_gt - p_est))
        rot_err = angle_deg(q_gt, q_est)

        self.get_logger().info(
            "\n[GT]  tag1→cam: pos={}  | tag2→cam: pos={}\n"
            "      fused GT pos={}, rot=[{:.3f},{:.3f},{:.3f},{:.3f}]\n"
            "[EST] tag1→cam: pos={}  | tag2→cam: pos={}\n"
            "      fused EST pos={}, rot=[{:.3f},{:.3f},{:.3f},{:.3f}]\n"
            "Δ(fused EST vs GT): {:.3f} m, {:.2f} deg".format(
                np.round(p1_gt,3), np.round(p2_gt,3),
                np.round(p_gt,3), q_gt[0], q_gt[1], q_gt[2], q_gt[3],
                np.round(p1_est,3), np.round(p2_est,3),
                np.round(p_est,3), q_est[0], q_est[1], q_est[2], q_est[3],
                pos_err, rot_err
            )
        )

def main():
    rclpy.init()
    rclpy.spin(Triang())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
