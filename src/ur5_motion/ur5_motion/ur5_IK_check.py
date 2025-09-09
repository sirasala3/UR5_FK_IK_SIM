import numpy as np
import math
from ur5_fk import fk
from ur5_ik import ur5_ik_hybrid
from conversions import pose_to_T, T_to_pose

# To make this script runnable without a full ROS environment,
# we need to mock the Pose, Point, and Quaternion classes from geometry_msgs.
class MockPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class MockQuaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class MockPose:
    def __init__(self, position=None, orientation=None):
        self.position = position if position is not None else MockPoint()
        self.orientation = orientation if orientation is not None else MockQuaternion()

def rot_angle(R):
    """
    Calculates the rotation angle from a 3x3 rotation matrix.

    Args:
        R (np.array): A 3x3 rotation matrix.

    Returns:
        float: The rotation angle in radians.
    """
    tr = np.trace(R)
    # Clamp the value to the valid range [-1, 1] to prevent floating point errors
    # from causing issues with np.arccos.
    val = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
    return np.arccos(val)

def pose_close(T1, T2, pos_tol=1e-4, rot_tol=1e-3):
    """
    Checks if two poses (homogeneous transformation matrices) are close
    within specified tolerances.

    Args:
        T1 (np.array): The first 4x4 homogeneous transformation matrix.
        T2 (np.array): The second 4x4 homogeneous transformation matrix.
        pos_tol (float): The maximum allowed position error.
        rot_tol (float): The maximum allowed rotation angle error (in radians).

    Returns:
        tuple: A tuple containing:
               - bool: True if poses are close, False otherwise.
               - float: The position error.
               - float: The rotation angle error.
    """
    # Calculate the position error as the Euclidean distance
    perr = np.linalg.norm(T1[:3, 3] - T2[:3, 3])

    # Calculate the rotation error matrix
    Rerr = T1[:3, :3] @ T2[:3, :3].T
    
    # Calculate the rotation angle error
    aerr = rot_angle(Rerr)

    return perr <= pos_tol and aerr <= rot_tol, perr, aerr

def main():
    """
    Main function to test the UR5 IK solution checker.
    1. Generates a random target pose using FK.
    2. Converts the pose to a mock ROS Pose message and back.
    3. Computes IK solutions for that pose.
    4. Checks if any solution is valid by comparing its FK pose to the target.
    """
    # 1. Generate a target pose from a known set of joint angles
    initial_q = np.array([0.1, -0.5, 1.2, 0.4, 0.8, -0.2])
    T_ground_truth = fk(initial_q)

    # 2. Convert the ground truth T matrix to a mock ROS Pose and back to simulate a real workflow
    # This checks the conversions.py functions as well.
    mock_pose = T_to_pose(T_ground_truth)
    T_target = pose_to_T(mock_pose)

    print("--- UR5 IK Solution Verification ---")
    print(f"Target Pose T_target: \n{np.round(T_target, 4)}")

    # 3. Compute IK solutions for the target pose
    ik_solutions = ur5_ik_hybrid(T_target)
    print(f"\nFound {len(ik_solutions)} IK solution(s).")
    
    test_passed = False
    
    # 4. Check each solution
    for i, sol in enumerate(ik_solutions):
        print(f"\n--- Checking solution {i+1} ---")
        
        # Calculate the forward kinematics pose for the solution
        T_sol = fk(sol)

        # Check if the solution is valid
        ok, perr, aerr = pose_close(T_sol, T_target)

        print(f"Position error: {perr:.6f}")
        print(f"Rotation angle error: {aerr:.6f} radians")
        print(f"Joint angles q: {np.round(sol, 4)}")
        
        if ok:
            print("Status: PASSED - Solution is within tolerances!")
            test_passed = True
        else:
            print("Status: FAILED - Solution is outside tolerances.")
    
    # Final summary
    print("\n--- Test Summary ---")
    if test_passed:
        print("Test passed: At least one IK solution was found to be valid.")
    else:
        print("Test failed: No valid IK solutions were found.")

if __name__ == "__main__":
    # We redefine `pose_to_T` and `T_to_pose` for this mock environment.
    # In a real environment, you would just import them.
    from conversions import pose_to_T, T_to_pose
    main()

