import numpy as np
import pytest
geometry_msgs = pytest.importorskip('geometry_msgs')
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
from ur5_motion.ur5_controller import linspace_cartesian, plan_cartesian_with_obstacles, SphereObstacle

def _pose(pos, rpy):
    p = Pose()
    p.position.x, p.position.y, p.position.z = pos
    q = quaternion_from_euler(*rpy)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p

def test_linspace_cartesian_is_straight_line():
    a = _pose([0.0,0.0,0.0],[0,0,0])
    b = _pose([1.0,0.0,0.0],[0,0,0])
    pts = linspace_cartesian(a,b,11)
    xs = [p.position.x for p in pts]
    # monotonic increasing
    assert all(xs[i] <= xs[i+1] for i in range(len(xs)-1))
    # unit-quaternion approximately
    for p in pts:
        q = np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
        assert abs(np.linalg.norm(q)-1) < 1e-6

def test_plan_with_obstacle_inserts_detour():
    a = _pose([0.0,0.0,0.0],[0,0,0])
    b = _pose([1.0,0.0,0.0],[0,0,0])
    obs = [SphereObstacle([0.5,0.0,0.0], 0.2)]
    pts = plan_cartesian_with_obstacles(a,b,21,obs)
    # midpoint should deviate from straight line (y or z non-zero)
    mid = pts[len(pts)//2]
    assert abs(mid.position.y) > 1e-6 or abs(mid.position.z) > 1e-6
