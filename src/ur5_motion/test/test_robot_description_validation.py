
import numpy as np
import pytest
from ur5_motion.robot_description import RobotDescription, JointLimit, ur5_description

def test_ur5_description_validates():
    desc = ur5_description()
    desc.validate()  # should not raise
    assert len(desc.joint_names) == 6
    assert desc.a.shape == (6,)
    assert desc.alpha.shape == (6,)
    assert desc.d.shape == (6,)
    assert len(desc.joint_limits) == 6
    assert desc.tool_T.shape == (4,4)

def test_invalid_limits_raise():
    bad = ur5_description()
    # replace one limit with invalid range
    jl = list(bad.joint_limits)
    jl = [JointLimit(-1.0, 1.0) for _ in range(6)]
    jl[3] = JointLimit(0.5, 0.4)  # min >= max
    bad2 = RobotDescription(joint_names=bad.joint_names, a=bad.a, alpha=bad.alpha, d=bad.d, joint_limits=jl, tool_T=bad.tool_T)
    with pytest.raises(ValidationError):
        bad2.validate()

def test_tool_T_last_row_is_identity_row():
    bad = ur5_description()
    T = bad.tool_T.copy()
    T[3,:] = np.array([0,0,0,2])
    broken = RobotDescription(joint_names=bad.joint_names, a=bad.a, alpha=bad.alpha, d=bad.d, joint_limits=bad.joint_limits, tool_T=T)
    with pytest.raises(ValidationError):
        broken.validate()
