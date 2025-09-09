# scripts/viz_no_ros.py
# Minimal, ROS-free 3D visualization for the challenge.
import argparse, time, numpy as np
import matplotlib.pyplot as plt
from ur5_motion.ur5_fk import fk_ur5
from ur5_motion.ur5_ik import ur5_ik_analytic

def slerp(q0, q1, s):
    q0, q1 = np.array(q0, float), np.array(q1, float)
    dot = np.dot(q0, q1)
    if dot < 0: q1, dot = -q1, -dot
    if dot > 0.9995:
        out = q0 + s*(q1-q0)
        return out/np.linalg.norm(out)
    th0 = np.arccos(np.clip(dot, -1, 1))
    th = th0 * s
    q2 = (q1 - q0*dot); q2 /= (np.linalg.norm(q2) + 1e-12)
    return q0*np.cos(th) + q2*np.sin(th)

def quat_from_rpy(r, p, y):
    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return np.array([qx,qy,qz,qw], float)

def link_frames(q):
    frames = [np.eye(4)]
    for i in range(6):
        q_i = np.zeros(6); q_i[:i+1] = q[:i+1]
        frames.append(fk_ur5(q_i))
    return frames

def linspace_cartesian(p0, p1, q0, q1, steps):
    path = []
    for i in range(steps):
        s = i/(steps-1) if steps>1 else 1.0
        p = (1-s)*p0 + s*p1
        q = slerp(q0, q1, s)
        path.append((p, q))
    return path

class SphereObstacle:
    def __init__(self, c, r): self.c=np.array(c,float); self.r=float(r)

def segment_hits_sphere(p0, p1, c, r):
    v = p1-p0; w = p0-c
    a = v@v; b = 2*(w@v); cterm = w@w - r*r
    disc = b*b - 4*a*cterm
    if disc < 0: return False
    t1 = (-b - np.sqrt(disc))/(2*a); t2 = (-b + np.sqrt(disc))/(2*a)
    return (0<=t1<=1) or (0<=t2<=1)

def plan_cartesian_with_obstacles(p0, p1, q0, q1, steps, obstacles):
    if not obstacles or not any(segment_hits_sphere(p0,p1,ob.c,ob.r*1.05) for ob in obstacles):
        return linspace_cartesian(p0,p1,q0,q1,steps)
    mid = 0.5*(p0+p1)
    nearest = min(obstacles, key=lambda ob: np.linalg.norm(mid-ob.c)-ob.r)
    dir_line = (p1-p0); dir_line/= (np.linalg.norm(dir_line)+1e-9)
    away = (mid-nearest.c); away/= (np.linalg.norm(away)+1e-9)
    tangent = np.cross(dir_line, np.cross(away, dir_line))
    tangent/= (np.linalg.norm(tangent)+1e-9)
    via = mid + tangent*(nearest.r*1.5)
    first = linspace_cartesian(p0, via, q0, q1, steps//2+1)
    second = linspace_cartesian(via, p1, q1, q1, steps - len(first) + 1)
    return first[:-1] + second

def animate_path(q_start, path, obstacles=None, dt=0.02):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_box_aspect([1,1,1])
    ax.set_xlim(-0.6, 0.6); ax.set_ylim(-0.6, 0.6); ax.set_zlim(0.0, 0.7)
    ax.view_init(elev=30, azim=45)

    def draw_obstacles():
        if obstacles:
            for ob in obstacles:
                u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
                x = ob.c[0] + ob.r*np.cos(u)*np.sin(v)
                y = ob.c[1] + ob.r*np.sin(u)*np.sin(v)
                z = ob.c[2] + ob.r*np.cos(v)
                ax.plot_wireframe(x,y,z, linewidth=0.3)

    q = q_start.copy()
    for i, (pos, quat) in enumerate(path):
        # quat -> R
        qx,qy,qz,qw = quat
        R = np.array([
            [1-2*(qy*qy+qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1-2*(qx*qx+qy*qy)]
        ], float)
        T = np.eye(4); T[:3,:3]=R; T[:3,3]=pos
        sols = ur5_ik_analytic(T)
        if not sols:
            print(f"Step {i+1:03d}: IK failed, skipping.")
            continue
        q = np.array(min(sols, key=lambda s: np.linalg.norm(np.array(s)-q)))

        frames = link_frames(q)
        xs = [f[0,3] for f in frames]
        ys = [f[1,3] for f in frames]
        zs = [f[2,3] for f in frames]

        # print EE pose (position only; orientation implicitly follows path)
        print(f"Step {i+1:03d}: pos={np.round(pos,3)}")

        ax.cla()
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.set_box_aspect([1,1,1])
        ax.set_xlim(-0.6, 0.6); ax.set_ylim(-0.6, 0.6); ax.set_zlim(0.0, 0.7)
        ax.view_init(elev=30, azim=45)
        draw_obstacles()
        ax.plot(xs, ys, zs, marker='o', linewidth=2)
        plt.pause(dt)
    plt.show(block=True)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", nargs=6, type=float, required=True, help="x y z roll pitch yaw (radians)")
    ap.add_argument("--steps", type=int, default=50)
    ap.add_argument("--obstacle", nargs=4, type=float, metavar=("cx","cy","cz","r"))
    args = ap.parse_args()

    q_start = np.array([0.0, -1.2, 1.4, 0.0, 1.2, 0.0])
    T0 = fk_ur5(q_start)
    p0 = T0[:3,3]
    # derive quat from current FK rotation
    R0 = T0[:3,:3]
    sy = np.sqrt(R0[0,0]**2 + R0[1,0]**2)
    if sy > 1e-6:
        r0 = np.arctan2(R0[2,1], R0[2,2]); p0r = np.arctan2(-R0[2,0], sy); y0 = np.arctan2(R0[1,0], R0[0,0])
    else:
        r0 = np.arctan2(-R0[1,2], R0[1,1]); p0r = np.arctan2(-R0[2,0], sy); y0 = 0.0
    q0 = quat_from_rpy(r0, p0r, y0)

    tx, ty, tz, rr, pp, yy = args.target
    p1 = np.array([tx, ty, tz], float)
    q1 = quat_from_rpy(rr, pp, yy)

    obstacles = []
    if args.obstacle:
        cx,cy,cz,r = args.obstacle
        obstacles = [SphereObstacle([cx,cy,cz], r)]

    path = plan_cartesian_with_obstacles(p0, p1, q0, q1, args.steps, obstacles)
    animate_path(q_start, path, obstacles=obstacles)

if __name__ == "__main__":
    main()
