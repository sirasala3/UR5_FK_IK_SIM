import numpy as np
import math
from robot_description import RobotDescription, ur5_description
from ur5_fk import fk as fk_generic


def _wrap_pi(a: float) -> float:
    """Wrap a single angle to [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def _wrap_q(q: np.ndarray) -> np.ndarray:
    """Wrap every element of q to [-pi, pi]."""
    q = np.array(q, dtype=float)
    for i in range(len(q)):
        q[i] = _wrap_pi(q[i])
    return q


def _rot_log(R: np.ndarray) -> np.ndarray:
    """
    Log map of a rotation matrix R (SO(3)) returning the rotation vector (axis * angle).
    Small-angle safe.
    """
    tr = np.trace(R)
    val = (tr - 1.0) / 2.0
    val = np.clip(val, -1.0, 1.0)
    th = math.acos(val)
    if th < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * (th / (2.0 * math.sin(th)))


def _pose_residual(
    q: np.ndarray,
    T_target: np.ndarray,
    desc: RobotDescription,
    w_pos: float = 1.0,
    w_rot: float = 1.0,
) -> np.ndarray:
    """
    Residual between FK(q) and target transform:
      [ w_pos * delta_position,
        w_rot * rotation_vector ]
    where rotation_vector is log(R_q * R_target^T).
    """
    Tq = fk_generic(q, desc)
    dp = Tq[:3, 3] - T_target[:3, 3]
    Rerr = Tq[:3, :3] @ T_target[:3, :3].T
    r = _rot_log(Rerr)
    return np.hstack([w_pos * dp, w_rot * r])


def _numeric_jacobian(f, x: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """
    Central-difference numerical jacobian of vector function f at x.
    f: R^n -> R^m
    returns J (m x n)
    """
    x = x.astype(float)
    y0 = f(x)
    J = np.zeros((y0.size, x.size))
    for i in range(x.size):
        xp = x.copy()
        xm = x.copy()
        xp[i] += eps
        xm[i] -= eps
        J[:, i] = (f(xp) - f(xm)) / (2.0 * eps)
    return J


def _within_limits(q: np.ndarray, desc: RobotDescription) -> bool:
    """Return True if q lies inside desc.joint_limits (inclusive)."""
    for i, lim in enumerate(desc.joint_limits):
        if not (lim.min <= q[i] <= lim.max):
            return False
    return True


def _project_limits(q: np.ndarray, desc: RobotDescription) -> np.ndarray:
    """Wrap q to [-pi,pi] and clip to joint limits."""
    q = _wrap_q(q)
    q_proj = q.copy()
    for i, lim in enumerate(desc.joint_limits):
        if q_proj[i] < lim.min:
            q_proj[i] = lim.min
        elif q_proj[i] > lim.max:
            q_proj[i] = lim.max
    return q_proj


def _seed_solutions(T: np.ndarray, desc: RobotDescription) -> list:
    """
    Analytic seeding for UR5:
    - compute wrist center (subtracting d6 along EE z)
    - compute q1 candidates (two)
    - compute q3 via cosine law (two)
    - compute q2 from geometry
    - compute q4,q5,q6 from R36
    Returns a list of seed joint vectors.
    """
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]
    R = T[:3, :3]
    d = desc.d
    a = desc.a
    d1, d6 = d[0], d[5]
    a2, a3 = a[1], a[2]

    # wrist center
    ax, ay, az = R[:, 2]
    wc = np.array([px - d6 * ax, py - d6 * ay, pz - d6 * az])
    q1_list = [math.atan2(wc[1], wc[0]), math.atan2(-wc[1], -wc[0])]

    r = math.hypot(wc[0], wc[1])
    z = wc[2] - d1

    seeds = []
    denom = 2.0 * a2 * a3
    if abs(denom) > 1e-12:
        D = (r * r + z * z - a2 * a2 - a3 * a3) / denom
        D = np.clip(D, -1.0, 1.0)
        q3_list = [math.acos(D), -math.acos(D)]

        for q1 in q1_list:
            for q3 in q3_list:
                phi = math.atan2(z, r)
                psi = math.atan2(a3 * math.sin(q3), a2 + a3 * math.cos(q3))
                q2 = phi - psi

                # compute R03 from FK with these q1,q2,q3
                T03 = fk_generic([q1, q2, q3, 0.0, 0.0, 0.0], desc)
                R03 = T03[:3, :3]
                R36 = R03.T @ R

                s = math.hypot(R36[0, 2], R36[1, 2])
                q5a = math.atan2(s, np.clip(R36[2, 2], -1.0, 1.0))

                for q5 in [q5a, -q5a]:
                    if abs(math.sin(q5)) > 1e-7:
                        q4 = math.atan2(R36[1, 2], R36[0, 2])
                        q6 = math.atan2(R36[2, 1], -R36[2, 0])
                    else:
                        # singular case for q5 ~ 0
                        q4 = 0.0
                        q6 = math.atan2(R36[0, 1], R36[0, 0])

                    seeds.append(_wrap_q([q1, q2, q3, q4, q5, q6]))

    # Add a couple of fallback seeds to improve robustness
    seeds.append(np.zeros(6))
    seeds.append(_wrap_q([0.0, -np.pi / 2.0, np.pi / 2.0, 0.0, 0.0, 0.0]))
    return seeds


def _dls_refine(
    T_target: np.ndarray,
    desc: RobotDescription,
    seed: np.ndarray,
    max_iters: int = 12,
    damping: float = 2e-3,
    w_pos: float = 1.0,
    w_rot: float = 1.0,
) -> np.ndarray:
    """
    Damped Least Squares refinement of a seed solution.
    Returns refined q (wrapped).
    """
    q = _project_limits(seed.copy(), desc)

    for _ in range(max_iters):
        # Residual function (projects & wraps internally)
        f = lambda x: _pose_residual(_project_limits(_wrap_q(x), desc), T_target, desc, w_pos, w_rot)

        r = f(q)
        if np.linalg.norm(r) < 1e-6:
            break

        J = _numeric_jacobian(f, q)

        # Build damped normal equations
        JT = J.T
        H = JT @ J + (damping**2) * np.eye(6)

        # Solve for step
        step = -np.linalg.solve(H, JT @ r)

        q = _project_limits(_wrap_q(q + step), desc)

    return _wrap_q(q)


def ur5_ik_hybrid(
    T: np.ndarray,
    desc: RobotDescription = None,
    max_seeds: int = 6,
    max_sols: int = 8,
    max_iters: int = 12,
    damping: float = 2e-3,
    w_pos: float = 1.0,
    w_rot: float = 1.0,
) -> list:
    """
    Hybrid IK for UR5: analytic seeding followed by DLS refinement.

    Returns a list of unique solutions (each is an ndarray of length 6).
    """
    desc = desc or ur5_description()
    seeds = _seed_solutions(T, desc)
    sols = []

    for s in seeds[:max_seeds]:
        q = _dls_refine(T, desc, s, max_iters=max_iters, damping=damping, w_pos=w_pos, w_rot=w_rot)

        if _within_limits(q, desc):
            # Check uniqueness (accounting for angle wrapping)
            is_new = not any(np.allclose(_wrap_q(q - p), 0.0, atol=1e-5) for p in sols)
            if is_new:
                sols.append(q)

        if len(sols) >= max_sols:
            break

    return sols

