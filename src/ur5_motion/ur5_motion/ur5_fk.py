# ur5_fk.py (refactored to use RobotDescription with validation)
import numpy as np
from robot_description import RobotDescription, ur5_description, JointLimit

# Backward compatibility constants for external code (derived from description)
_DESC = ur5_description()
JOINT_LIMITS = [(lim.min, lim.max) for lim in _DESC.joint_limits]
JOINT_NAMES = _DESC.joint_names
# Named UR5 constants (for older code imports)
d1, a2, a3, d4, d5, d6 = _DESC.d[0], _DESC.a[1], _DESC.a[2], _DESC.d[3], _DESC.d[4], _DESC.d[5]

def dh(a, alpha, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0 ,     sa,     ca,    d],
        [0 ,      0,      0,    1],
    ], dtype=float)

def fk(q, desc: RobotDescription = None):
    """Forward kinematics using DH params from RobotDescription."""
    desc = desc or _DESC
    # Pydantic BaseModel already validated at construction
    if len(q) != 6:
        raise ValueError("q must be length 6")
    T = np.eye(4, dtype=float)
    for i in range(6):
        T = T @ dh(desc.a[i], desc.alpha[i], desc.d[i], q[i])
    return T @ desc.tool_T

# Backward-compatible function name
def fk_ur5(q):
    return fk(np.array(q, dtype=float), _DESC)
