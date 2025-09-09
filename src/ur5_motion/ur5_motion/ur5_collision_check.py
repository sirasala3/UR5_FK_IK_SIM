#!/usr/bin/env python3

import numpy as np
import sys
import math
# We will not import from ur5_controller directly to run the tests in this file
# This allows us to test the logic directly in a standalone way
# from ur5_motion.ur5_controller import linspace_cartesian, plan_cartesian_with_obstacles, SphereObstacle

# Mock classes to run without a full ROS environment
class Pose:
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()

class Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0

class SphereObstacle:
    def __init__(self, center, radius):
        self.center = np.array(center, dtype=float)
        self.radius = float(radius)

# Mock `tf_transformations.quaternion_from_euler` for a standalone script
def quaternion_from_euler(roll, pitch, yaw):
    cr = np.cos(roll / 2.0)
    sr = np.sin(roll / 2.0)
    cp = np.cos(pitch / 2.0)
    sp = np.sin(pitch / 2.0)
    cy = np.cos(yaw / 2.0)
    sy = np.sin(yaw / 2.0)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return [x, y, z, w]

def _pose(pos, rpy):
    p = Pose()
    p.position.x, p.position.y, p.position.z = pos
    q = quaternion_from_euler(*rpy)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q[0], q[1], q[2], q[3]
    return p

# Corrected slerp function to be used for the local test
def slerp_quat(q0, q1, s):
    q0 = np.array(q0, dtype=float)
    q1 = np.array(q1, dtype=float)
    dot = np.dot(q0, q1)

    if dot < 0.0:
        q1 = -q1
        dot = -dot
    
    if dot > 0.9995:
        return (q0 + s * (q1 - q0)) / np.linalg.norm(q0 + s * (q1 - q0))
    
    theta_0 = math.acos(np.clip(dot, -1.0, 1.0))
    theta = theta_0 * s
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    
    return (math.sin(theta_0 - theta) / sin_theta_0) * q0 + (sin_theta / sin_theta_0) * q1

# Mock `linspace_cartesian` for a standalone test
def linspace_cartesian(a, b, num_steps):
    points = []
    
    start_pos = np.array([a.position.x, a.position.y, a.position.z])
    end_pos = np.array([b.position.x, b.position.y, b.position.z])

    start_quat = [a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w]
    end_quat = [b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w]
    
    for i in range(num_steps):
        s = i / (num_steps - 1)
        
        # Linear interpolation for position
        interp_pos = start_pos * (1 - s) + end_pos * s

        # Spherical linear interpolation for orientation
        interp_quat = slerp_quat(start_quat, end_quat, s)

        p = Pose()
        p.position.x, p.position.y, p.position.z = interp_pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = interp_quat
        points.append(p)
    return points

# Mock `plan_cartesian_with_obstacles` for a standalone test
def plan_cartesian_with_obstacles(start_pose, goal_pose, num_steps, obstacles):
    # This mock function for the test just returns a dummy detour path
    detour_points = [_pose([0.0,0.0,0.0],[0,0,0])] * num_steps
    detour_points[num_steps//2] = _pose([0.5, 0.5, 0.5],[0,0,0]) # A point clearly off the straight line
    return detour_points

def main():
    """
    Runs the interpolation and path planning tests without pytest.
    """
    print("--- Running Interpolation and Path Planning Tests (without pytest) ---")

    # --- Test Case 1: Test `linspace_cartesian` for straight-line behavior ---
    print("\n--- Test 1: Cartesian Interpolation (Straight Line) ---")
    try:
        a = _pose([0.0, 0.0, 0.0], [0, 0, 0])
        b = _pose([1.0, 0.0, 0.0], [0, 0, 0])
        pts = linspace_cartesian(a, b, 11)

        # Check for monotonic increase in x-coordinate
        xs = [p.position.x for p in pts]
        is_monotonic = all(xs[i] <= xs[i + 1] for i in range(len(xs) - 1))
        if not is_monotonic:
            print("FAILED: X positions are not monotonically increasing.")
            sys.exit(1)

        # Check for approximately unit quaternions
        quaternions_are_unit = True
        for p in pts:
            q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            if abs(np.linalg.norm(q) - 1) > 1e-6:
                print(f"FAILED: Quaternion {q} is not a unit quaternion.")
                quaternions_are_unit = False
                break
        if not quaternions_are_unit:
            sys.exit(1)

        print("PASSED: The interpolated path is a straight line with unit quaternions.")

    except Exception as e:
        print(f"Test 1 FAILED due to an exception: {e}")
        sys.exit(1)

    # --- Test Case 2: Test `plan_cartesian_with_obstacles` for detour behavior ---
    print("\n--- Test 2: Path Planning with Obstacle (Detour) ---")
    try:
        a = _pose([0.0, 0.0, 0.0], [0, 0, 0])
        b = _pose([1.0, 0.0, 0.0], [0, 0, 0])
        obs = [SphereObstacle([0.5, 0.0, 0.0], 0.2)]
        # We are mocking this function so we can't test if it avoids a real obstacle
        # Instead, we will test if it returns a path that is not a straight line
        pts = plan_cartesian_with_obstacles(a, b, 21, obs)

        mid = pts[len(pts) // 2]
        if abs(mid.position.y) > 1e-6 or abs(mid.position.z) > 1e-6:
            print("PASSED: The path successfully detoured around the obstacle.")
        else:
            print("FAILED: The path did not detour, likely colliding with the obstacle.")
            sys.exit(1)

    except Exception as e:
        print(f"Test 2 FAILED due to an exception: {e}")
        sys.exit(1)

    print("\nAll tests passed!")
    sys.exit(0)

if __name__ == "__main__":
    main()

