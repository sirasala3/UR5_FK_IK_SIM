import numpy as np
from ur5_fk import fk_ur5, d1, a2, a3, d4, d5, d6, JOINT_LIMITS

def ur5_ik_analytic(T):
    """
    Analytical inverse kinematics for UR5.
    Returns a list of valid 6-DOF joint solutions.
    """

    solutions = []

    # End-effector position and orientation
    px, py, pz = T[0,3], T[1,3], T[2,3]
    R = T[:3,:3]
    ax, ay, az = R[:,2]  # z-axis of end-effector

    # Wrist center
    wc_x = px - d6 * ax
    wc_y = py - d6 * ay
    wc_z = pz - d6 * az

    # ----- Joint 1 -----
    q1_options = []
    theta1 = np.arctan2(wc_y, wc_x)
    q1_options.append(theta1)
    q1_options.append(theta1 + np.pi if theta1 < 0 else theta1 - np.pi)

    # Loop over q1 options
    for q1 in q1_options:
        # Transform wrist center to frame 1
        r = np.sqrt(wc_x**2 + wc_y**2)
        s = wc_z - d1

        # ----- Joint 3 -----
        D = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
        if abs(D) > 1:
            continue  # No solution
        q3_options = [np.arctan2(np.sqrt(1 - D**2), D),
                      np.arctan2(-np.sqrt(1 - D**2), D)]

        for q3 in q3_options:
            # ----- Joint 2 -----
            k1 = a2 + a3 * np.cos(q3)
            k2 = a3 * np.sin(q3)
            q2 = np.arctan2(s, r) - np.arctan2(k2, k1)

            # ----- Wrist (q4, q5, q6) -----
            T03 = fk_ur5([q1, q2, q3, 0, 0, 0])
            R03 = T03[:3,:3]
            R36 = R03.T @ R

            # Clamp numerical errors
            R36_22 = max(-1.0, min(1.0, R36[2,2]))
            q5 = np.arccos(R36_22)

            if abs(np.sin(q5)) < 1e-6:
                q4 = 0
                q6 = np.arctan2(-R36[1,0], R36[0,0])
            else:
                q4 = np.arctan2(R36[1,2], R36[0,2])
                q6 = np.arctan2(R36[2,1], -R36[2,0])

            # Normalize to [-pi, pi]
            q_all = [(q + np.pi) % (2*np.pi) - np.pi for q in [q1, q2, q3, q4, q5, q6]]

            # Check joint limits
            if all(JOINT_LIMITS[i][0] <= q_all[i] <= JOINT_LIMITS[i][1] for i in range(6)):
                solutions.append(q_all)

    return solutions

