import numpy as np
import math
from ur5_fk import fk, JOINT_LIMITS
from ur5_ik import ur5_ik_hybrid

def joint_angles_close(q1, q2, tol=1e-4):
    """
    Checks if two sets of joint angles are close within a specified tolerance.

    Args:
        q1 (np.array): The first set of joint angles.
        q2 (np.array): The second set of joint angles.
        tol (float): The maximum allowed difference for each joint angle.

    Returns:
        bool: True if all joint angles are close, False otherwise.
    """
    # Normalize angles to [-pi, pi] to handle wrap-around cases
    q1_wrapped = np.array([(a + np.pi) % (2 * np.pi) - np.pi for a in q1])
    q2_wrapped = np.array([(a + np.pi) % (2 * np.pi) - np.pi for a in q2])
    
    return np.allclose(q1_wrapped, q2_wrapped, atol=tol)

def main():
    """
    Main function to test the UR5 FK solution checker.
    1. Generates a random set of joint angles.
    2. Calculates the FK pose for these angles.
    3. Computes IK solutions for the resulting pose.
    4. Verifies if the original joint angles are one of the IK solutions.
    """
    print("--- UR5 FK Solution Verification ---")

    # 1. Generate a random, valid set of joint angles within limits
    q_target = np.array([
        np.random.uniform(JOINT_LIMITS[i][0], JOINT_LIMITS[i][1]) for i in range(6)
    ])
    
    print(f"Randomly generated joint angles (q_target):\n{np.round(q_target, 4)}")
    
    # 2. Calculate the FK pose for the target angles
    T_target = fk(q_target)
    
    print(f"\nFK Pose (T_target):\n{np.round(T_target, 4)}")

    # 3. Compute IK solutions for the target pose
    ik_solutions = ur5_ik_hybrid(T_target)
    print(f"\nFound {len(ik_solutions)} IK solution(s) for the pose.")
    
    # 4. Check if the original joint angles are in the list of solutions
    found_solution = False
    found_index = -1
    for i, sol in enumerate(ik_solutions):
        if joint_angles_close(q_target, sol):
            found_solution = True
            found_index = i
            break
            
    print("\n--- Test Summary ---")
    if found_solution:
        print(f"Test passed: Original joint angles found in IK solutions at index {found_index}.")
        print(f"Matching solution:\n{np.round(ik_solutions[found_index], 4)}")
    else:
        print("Test failed: Original joint angles were not found in the IK solutions.")

if __name__ == "__main__":
    main()

