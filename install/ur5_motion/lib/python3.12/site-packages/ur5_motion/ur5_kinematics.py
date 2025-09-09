import numpy as np
from ur5_fk import fk_ur5
from ur5_ik import ur5_ik_hybrid
from robot_description import ur5_description
from conversions import pose_to_T, T_to_pose
import time
from typing import List

def wrapped_joint_distance(q1, q2):
    """Calculates the wrapped distance between two joint angle configurations."""
    d = q1 - q2
    return np.linalg.norm((d + np.pi) % (2 * np.pi) - np.pi)

def run_multiple_tests(test_points: List[np.ndarray], tolerance=1e-3):
    """
    Runs kinematics tests for a list of known joint configurations with detailed debug prints.
    """
    print("--- Starting Multiple Kinematics Tests with Debugging ---")
    total_tests = len(test_points)
    failures = 0
    
    for i, q_original in enumerate(test_points):
        print(f"\n--- Test {i+1}/{total_tests} ---")
        print(f"Original Joints: {np.round(q_original, 5)}")

        # 1. Perform Forward Kinematics
        T = fk_ur5(q_original)
        print("\n  --- FK Result ---")
        print("  FK Transformation Matrix (T):\n", np.round(T, 5))
        
        # 2. Convert T to Pose for debugging (note: T_to_pose is often a source of error)
        try:
            pose = T_to_pose(T)
            print("\n  --- Pose from FK ---")
            print(f"  Position: x={pose.position.x:.5f}, y={pose.position.y:.5f}, z={pose.position.z:.5f}")
            print(f"  Orientation (Quaternion): x={pose.orientation.x:.5f}, y={pose.orientation.y:.5f}, z={pose.orientation.z:.5f}, w={pose.orientation.w:.5f}")
        except Exception as e:
            print(f"  Warning: T_to_pose failed with error: {e}")

        # 3. Perform Inverse Kinematics
        start_time = time.time()
        q_solutions = ur5_ik_hybrid(T)
        end_time = time.time()
        print(f"\n  --- IK Solutions ({len(q_solutions)} found in {end_time - start_time:.4f}s) ---")

        # 4. Check all solutions and find the closest one
        found_match = False
        if q_solutions:
            min_dist = np.inf
            closest_q = None
            for j, q_found in enumerate(q_solutions):
                dist = wrapped_joint_distance(q_original, q_found)
                print(f"  Solution {j+1}: {np.round(q_found, 5)} (Distance: {dist:.5f})")
                if dist < min_dist:
                    min_dist = dist
                    closest_q = q_found
                if dist < tolerance:
                    found_match = True
        
        print("\n  --- Test Summary ---")
        if found_match:
            print("  Status: SUCCESS! A matching IK solution was found.")
        else:
            failures += 1
            print("  Status: FAILURE! The original solution was NOT found.")
            if not q_solutions:
                print("  Reason: No IK solutions found for this pose.")
            else:
                print(f"  Reason: Closest IK solution is off by {min_dist:.5f} (>{tolerance})")
                print(f"  Closest Solution: {np.round(closest_q, 5)}")
            
    print("\n--- Final Results ---")
    if failures == 0:
        print(f"SUCCESS: All {total_tests} tests passed!")
    else:
        print(f"FAILURE: {failures}/{total_tests} tests failed. There is a bug in your kinematics.")


if __name__ == "__main__":
    # A set of 10 known, non-singular test points
    test_points = [
        # 1. Home Pose
        np.array([0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]),
        # 2. Stretched out
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 3. High reach
        np.array([0.0, -np.pi/2, np.pi/2, -np.pi/2, 0.0, 0.0]),
        # 4. Low reach
        np.array([np.pi/2, np.pi/4, np.pi/4, -np.pi/2, -np.pi/4, 0.0]),
        # 5. Complex configuration
        np.array([-0.5, -1.0, 1.5, -2.0, 0.5, 1.0]),
        # 6. Another complex one
        np.array([1.2, 0.8, -1.2, 0.5, -1.5, 0.3]),
        # 7. Close to singularity (but not singular)
        np.array([0.0, -0.01, 0.01, -0.01, 0.0, 0.0]),
        # 8. All negative angles
        np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]),
        # 9. All positive angles
        np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        # 10. Random-ish but non-singular
        np.array([2.5, -0.5, 1.0, -1.5, 2.0, -0.7])
    ]
    
    run_multiple_tests(test_points)
