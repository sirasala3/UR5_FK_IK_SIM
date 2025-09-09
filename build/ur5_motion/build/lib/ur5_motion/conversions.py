# conversions.py
import numpy as np
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler, quaternion_from_matrix
from math import atan2, sqrt, copysign


def rot_from_quat(qx, qy, qz, qw):
    """Convert quaternion to rotation matrix"""
    x, y, z, w = qx, qy, qz, qw
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=float)
    return R

def pose_to_T(pose: Pose):
    """Convert ROS Pose to homogeneous transformation matrix"""
    R = rot_from_quat(pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = np.array([pose.position.x, pose.position.y, pose.position.z])
    return T
    
def T_to_pose(T):
    """Convert transformation matrix to ROS Pose (Corrected Version)"""
    pose = Pose()
    pose.position.x = float(T[0, 3])
    pose.position.y = float(T[1, 3])
    pose.position.z = float(T[2, 3])
    
    # Direct rotation matrix to quaternion conversion
    R = T[:3, :3]
    qw = 0.5 * sqrt(max(0, 1 + R[0,0] + R[1,1] + R[2,2]))
    qx = 0.5 * sqrt(max(0, 1 + R[0,0] - R[1,1] - R[2,2]))
    qy = 0.5 * sqrt(max(0, 1 - R[0,0] + R[1,1] - R[2,2]))
    qz = 0.5 * sqrt(max(0, 1 - R[0,0] - R[1,1] + R[2,2]))
    
    qx = copysign(qx, R[2,1] - R[1,2])
    qy = copysign(qy, R[0,2] - R[2,0])
    qz = copysign(qz, R[1,0] - R[0,1])
    
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    
    return pose
