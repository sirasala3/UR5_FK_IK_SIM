# ur5_fk.py
import numpy as np

# -------------------------------
# UR5 DH Parameters
# -------------------------------
d1 = 0.089159
a2 = -0.42500
a3 = -0.39225
d4 = 0.10915
d5 = 0.09465
d6 = 0.0823

# UR5 joint limits (radians)
JOINT_LIMITS = [
    (-2.9671, 2.9671),  # shoulder_pan_joint
    (-1.8326, 1.8326),  # shoulder_lift_joint
    (-2.9671, 2.9671),  # elbow_joint
    (-3.0543, 3.0543),  # wrist_1_joint
    (-2.0944, 2.0944),  # wrist_2_joint
    (-6.1087, 6.1087)   # wrist_3_joint
]

# Joint names for UR5e
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

# -------------------------------
# Forward Kinematics
# -------------------------------
def dh(a, alpha, d, theta):
    """Denavit-Hartenberg transformation matrix"""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ], dtype=float)

def fk_ur5(q):
    """Forward kinematics for UR5"""
    alphas = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    as_ = [0, a2, a3, 0, 0, 0]
    ds = [d1, 0, 0, d4, d5, d6]
    
    T = np.eye(4)
    for i in range(6):
        T = T @ dh(as_[i], alphas[i], ds[i], q[i])
    return T

