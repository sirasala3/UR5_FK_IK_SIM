import numpy as np
from ur5_motion.ur5_fk import JOINT_LIMITS
from ur5_motion.conversions import pose_to_T, T_to_pose
import pytest
geometry_msgs = pytest.importorskip('geometry_msgs')
from geometry_msgs.msg import Pose

def test_joint_limits_span():
    for lo, hi in JOINT_LIMITS:
        assert hi > lo

def test_pose_roundtrip_identity_rotation():
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 0.3, 0.0, 0.2
    # identity quat
    pose.orientation.w = 1.0
    T = pose_to_T(pose)
    back = T_to_pose(T)
    assert abs(back.position.x - 0.3) < 1e-9
