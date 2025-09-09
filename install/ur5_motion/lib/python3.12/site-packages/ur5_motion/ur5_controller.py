import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import time
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from ur5_motion.robot_description import ur5_description
from ur5_fk import fk_ur5, JOINT_LIMITS
from ur5_ik import ur5_ik_hybrid
from conversions import pose_to_T, T_to_pose

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def within_limits(q):
    return all(JOINT_LIMITS[i][0] - 1e-6 <= q[i] <= JOINT_LIMITS[i][1] + 1e-6 for i in range(6))

def slerp_quat(q0, q1, s):
    q0 = np.array(q0, dtype=float); q1 = np.array(q1, dtype=float)
    dot = np.dot(q0, q1)
    if dot < 0: q1 = -q1; dot = -dot
    if dot > 0.9995:
        return (q0 + s*(q1-q0)) / np.linalg.norm(q0 + s*(q1-0))
    th0 = np.arccos(np.clip(dot, -1, 1))
    th = th0 * s
    q2 = (q1 - q0*dot); q2 /= (np.linalg.norm(q2) + 1e-12)
    return q0*np.cos(th) + q2*np.sin(th)

def linspace_cartesian(start_pose: Pose, target_pose: Pose, steps: int):
    p0 = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
    p1 = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
    q0 = np.array([start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w])
    q1 = np.array([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w])
    out = []
    for i in range(steps):
        s = i/(steps-1) if steps > 1 else 1
        p = (1-s)*p0 + s*p1
        q = slerp_quat(q0, q1, s)
        ps = Pose()
        ps.position.x, ps.position.y, ps.position.z = p.tolist()
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = q.tolist()
        out.append(ps)
    return out

class SphereObstacle:
    def __init__(self, center, radius):
        self.c = np.array(center, dtype=float)
        self.r = float(radius)

def _segment_hits_sphere(p0, p1, c, r):
    v = p1 - p0
    w = p0 - c
    a = v @ v
    b = 2*(w @ v)
    cterm = w @ w - r*r
    disc = b*b - 4*a*cterm
    if disc < 0: return False
    t1 = (-b - np.sqrt(disc)) / (2*a)
    t2 = (-b + np.sqrt(disc)) / (2*a)
    return (0 <= t1 <= 1) or (0 <= t2 <= 1)

def plan_cartesian_with_obstacles(start_pose, target_pose, steps, obstacles):
    p0 = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
    p1 = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
    if not obstacles or not any(_segment_hits_sphere(p0, p1, ob.c, ob.r*1.05) for ob in obstacles):
        return linspace_cartesian(start_pose, target_pose, steps)
    
    mid = 0.5*(p0+p1)
    nearest = min(obstacles, key=lambda ob: np.linalg.norm(mid - ob.c) - ob.r)
    dir_line = (p1 - p0); dir_line /= (np.linalg.norm(dir_line)+1e-9)
    away = (mid - nearest.c); away /= (np.linalg.norm(away)+1e-9)
    tangent = np.cross(dir_line, np.cross(away, dir_line)); tangent /= (np.linalg.norm(tangent)+1e-9)
    via = mid + tangent * (nearest.r * 1.5)

    via_pose = Pose()
    via_pose.position.x, via_pose.position.y, via_pose.position.z = via.tolist()
    via_pose.orientation = target_pose.orientation 
    first = linspace_cartesian(start_pose, via_pose, steps//2 + 1)
    second = linspace_cartesian(via_pose, target_pose, steps - len(first) + 1)
    return first[:-1] + second

def wrapped_joint_distance(q1, q2):
    """
    Calculates the shortest angular distance between two joint configurations,
    accounting for the circular nature of the joints.
    """
    q1 = np.array(q1)
    q2 = np.array(q2)
    diff = q1 - q2
    # Normalize joint angles to [-pi, pi) to find the shortest path
    diff = (diff + np.pi) % (2 * np.pi) - np.pi
    return np.linalg.norm(diff)
    
def find_best_trajectory(path, initial_joints):
    """
    Finds the best series of IK solutions that creates the smoothest
    trajectory in joint space.
    """
    if not path:
        return []

    MAX_JOINT_STEP = np.pi # 180 degrees, very permissive

    # Get all IK solutions for the first pose
    print("DEBUG: Checking IK for initial pose...")
    first_pose_sols = ur5_ik_hybrid(pose_to_T(path[0]))
    
    if not first_pose_sols:
        print("DEBUG: No IK solutions for the first pose.")
        return []
    
    valid_sols_first = [q for q in first_pose_sols if not (abs(q[4]) < 1e-3 or abs(abs(q[4]) - np.pi) < 1e-3)]
    
    if not valid_sols_first:
        print("DEBUG: No valid, non-singular IK solutions for the first pose. Aborting.")
        return []

    q_start = min(valid_sols_first, key=lambda q: wrapped_joint_distance(q, initial_joints))
    
    trajectory = [q_start]
    last_q = np.array(q_start)
    print(f"DEBUG: Selected initial joint configuration: {np.round(q_start, 3)}")

    for i in range(1, len(path)):
        print(f"\nDEBUG: Processing path step {i}/{len(path)-1}...")
        next_pose_sols = ur5_ik_hybrid(pose_to_T(path[i]))
        
        if not next_pose_sols:
            print(f"DEBUG: No IK solution for path step {i}. Aborting.")
            return []
        
        # Filter and sort all valid solutions by distance to the last joint state
        valid_and_close_sols = []
        for q in next_pose_sols:
            is_singular = abs(q[4]) < 1e-3 or abs(abs(q[4]) - np.pi) < 1e-3
            is_large_step = np.any(np.abs((q - last_q + np.pi) % (2*np.pi) - np.pi) > MAX_JOINT_STEP)
            if not is_singular and not is_large_step:
                valid_and_close_sols.append(q)
        
        print(f"DEBUG: Found {len(next_pose_sols)} raw IK solutions.")
        print(f"DEBUG: {len(valid_and_close_sols)} solutions passed singularity/step check.")

        # Sort the valid solutions by their distance to the previous joint state
        valid_and_close_sols.sort(key=lambda q: wrapped_joint_distance(q, last_q))
        
        if not valid_and_close_sols:
            print(f"DEBUG: No valid solutions found for path step {i}. Aborting.")
            return []
            
        # Select the best solution from the filtered and sorted list
        q_next = valid_and_close_sols[0]
        
        print(f"DEBUG: Chose solution with wrapped joint distance of {wrapped_joint_distance(q_next, last_q):.3f}.")
        print(f"DEBUG: Selected joint configuration: {np.round(q_next, 3)}")
            
        trajectory.append(q_next)
        last_q = np.array(q_next)
        
    return trajectory

def is_in_workspace(pose: Pose, desc=None):
    """
    Checks if a given Cartesian pose is within the robot's workspace.
    This is a quick sanity check before attempting IK.
    """
    desc = desc or ur5_description()
    
    # Calculate min and max theoretical reach
    # Max reach is when all arm segments are fully extended.
    # Use absolute values for the 'a' parameters as they define arm segment lengths.
    max_reach = abs(desc.a[1]) + abs(desc.a[2]) + desc.d[3] + desc.d[4] + desc.d[5]
    
    # Min reach can be approximated as the distance to the base for the 
    # first joint, or more accurately by considering the folded-in arm.
    min_reach = desc.d[3]

    x, y, z = pose.position.x, pose.position.y, pose.position.z
    dist_from_base = np.sqrt(x**2 + y**2 + z**2)
    
    if dist_from_base > max_reach:
        print(f"ERROR: Point is outside the maximum workspace (dist: {dist_from_base:.3f}m, max: {max_reach:.3f}m).")
        return False
        
    if dist_from_base < min_reach:
        print(f"ERROR: Point is inside the minimum workspace (dist: {dist_from_base:.3f}m, min: {min_reach:.3f}m).")
        return False
        
    return True


class UR5JazzyController(Node):
    def __init__(self):
        super().__init__('ur5_jazzy_controller')
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_target = self.create_publisher(PoseStamped, '/pose_target', 10)
        self.current_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.obstacles = []
        self.get_logger().info("Controller ready.")
        
        self.pub_timer = self.create_timer(0.02, self.publish_current_state)

    def publish_current_state(self):
        """Publishes the current joint state to keep RViz updated."""
        msg = JointState(
            header=Header(stamp=self.get_clock().now().to_msg()),
            name=ur5_description().joint_names,
            position=self.current_joints.tolist()
        )
        self.pub_joint.publish(msg)

    def publish_pose_target(self, pose: Pose):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "base_link"
        ps.pose = pose
        self.pub_target.publish(ps)

    def get_current_pose(self):
        """
        Gets the current end-effector pose by performing forward kinematics.
        """
        self.get_logger().info(f"DEBUG: Getting current pose...")
        if not isinstance(self.current_joints, np.ndarray) or self.current_joints.shape != (6,):
            self.get_logger().error("ERROR: current_joints is not a valid 6-element numpy array. Cannot perform FK.")
            return None
        try:
            T = fk_ur5(self.current_joints)
            if not isinstance(T, np.ndarray) or T.shape != (4, 4):
                 self.get_logger().error(f"ERROR: FK function did not return a valid 4x4 matrix. Instead, got: {T}")
                 return None
            return T_to_pose(T)
        except Exception as e:
            self.get_logger().error(f"Failed to get current pose: {e}")
            return None

    def _execute_blocking_trajectory(self, trajectory):
        """
        Executes a trajectory by updating joint positions step by step,
        ensuring the state is synchronized before returning.
        """
        if not trajectory:
            return
        
        rate_hz = 50
        step_duration = 1.0 / rate_hz

        start_time = self.get_clock().now()
        for i, next_q in enumerate(trajectory):
            # Wait for the next step time to pass to maintain a consistent rate
            expected_time = start_time + rclpy.duration.Duration(seconds=i * step_duration)
            while self.get_clock().now() < expected_time:
                time.sleep(0.001)

            self.current_joints = np.array(next_q)
            self.publish_current_state()
            
    def move_to_joint_positions_smooth(self, target_joints, duration=8.0):
        current_joints = np.array(self.current_joints)
        target_joints = np.array(target_joints)
        steps = max(1, int(duration * 50))
        
        trajectory_plan = []
        for i in range(1, steps + 1):
            s = i/steps
            q = (1-s)*current_joints + s*target_joints
            trajectory_plan.append(q.tolist())
        
        self.get_logger().info(f"Starting joint space move with {len(trajectory_plan)} steps.")
        self._execute_blocking_trajectory(trajectory_plan)
        self.get_logger().info("Joint move complete.")

    def move_cartesian(self, target_pose: Pose, duration=8.0, avoid_obstacles=False):
        # Workspace check
        if not is_in_workspace(target_pose):
            self.get_logger().warn("Target pose is outside the robot's workspace. Aborting motion.")
            return
            
        start_pose = self.get_current_pose()
        if start_pose is None:
            self.get_logger().error("Cannot start cartesian move. Initial pose not available.")
            return
        
        if not ur5_ik_hybrid(pose_to_T(start_pose)):
            self.get_logger().warn("Current pose is in a problematic configuration (no IK solution). Aborting motion.")
            return

        steps = max(1, int(duration * 50))
        path = plan_cartesian_with_obstacles(start_pose, target_pose, steps, self.obstacles) if avoid_obstacles \
                 else linspace_cartesian(start_pose, target_pose, steps)
        self.publish_pose_target(target_pose)
        
        trajectory_plan = find_best_trajectory(path, self.current_joints)
        
        if not trajectory_plan:
            self.get_logger().warn("Failed to find a continuous IK trajectory. Aborting motion.")
            return
        
        self.get_logger().info(f"Starting cartesian move with {len(trajectory_plan)} steps.")
        self._execute_blocking_trajectory(trajectory_plan)
        self.get_logger().info("Cartesian move complete.")

    def execute_waypoint_path(self, poses, duration_per_segment=3.0, avoid_obstacles=True):
        if not poses: return
        
        # Workspace check for all waypoints
        for i, pose in enumerate(poses):
            if not is_in_workspace(pose):
                self.get_logger().warn(f"Waypoint {i} is outside the robot's workspace. Aborting motion.")
                return

        start_pose = self.get_current_pose()
        if start_pose is None:
            self.get_logger().error("Cannot start waypoint path. Initial pose not available.")
            return
        
        full_cartesian_path = []
        current_pose = start_pose
        steps_per_segment = max(1, int(duration_per_segment * 50))

        for target_pose in poses:
            segment = plan_cartesian_with_obstacles(current_pose, target_pose, steps_per_segment, self.obstacles) if avoid_obstacles \
                        else linspace_cartesian(current_pose, target_pose, steps_per_segment)
            full_cartesian_path.extend(segment)
            current_pose = target_pose
        
        self.get_logger().info(f"Planning a continuous path with {len(full_cartesian_path)} total steps.")
        full_joint_trajectory = find_best_trajectory(full_cartesian_path, self.current_joints)
        
        if not full_joint_trajectory:
            self.get_logger().warn("Failed to find a continuous IK trajectory for waypoints. Aborting motion.")
            return

        self._execute_blocking_trajectory(full_joint_trajectory)
        self.get_logger().info("Waypoint path complete.")

    def _print_pose(self, tag, pose: Pose):
        pos = (pose.position.x, pose.position.y, pose.position.z)
        r,p,y = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        self.get_logger().info(f"{tag}: pos={np.round(pos,3)} rpy={np.round([r,p,y],3)}")

    def run_demo(self):
        self.get_logger().info("=== UR5 Jazzy Controller Demo ===")
        safe_poses = {
            "home":  {"pos":[0.486, -0.109, 0.597], "rpy":[-0.002, -0.001, 1.571]},
            "front": {"pos":[0.4,0.0,0.3], "rpy":[0,1.57,0]},
            "left":  {"pos":[0.3,0.3,0.3], "rpy":[0,1.57,0.5]},
            "right": {"pos":[0.3,-0.3,0.3],"rpy":[0,1.57,-0.5]}
        }
        self.get_logger().info("Homing the robot to a safe, initial position...")
        
        home_pose_data = safe_poses["home"]
        home_pose = Pose()
        home_pose.position.x, home_pose.position.y, home_pose.position.z = home_pose_data["pos"]
        qx, qy, qz, qw = quaternion_from_euler(*home_pose_data["rpy"])
        home_pose.orientation.x, home_pose.orientation.y, home_pose.orientation.z, home_pose.orientation.w = qx, qy, qz, qw  
        home_sols = ur5_ik_hybrid(pose_to_T(home_pose))
        
        if home_sols:
            home_joints = min(home_sols, key=lambda q: np.linalg.norm(q))
            self.move_to_joint_positions_smooth(home_joints, duration=1.0)
            self.get_logger().info("Successfully homed to a valid joint configuration.")
        else:
            self.get_logger().error("Could not find a valid IK solution for the home pose. Robot may be in a bad state.")
        
        self.get_logger().info("Homing complete. Ready for commands.")

        while rclpy.ok():
            print("\n1. Move to Predefined Pose\n2. Move to Custom Pose\n3. Execute Cartesian Waypoints\n4. Show Current EE Pose\n5. Toggle Sphere Obstacle\n6. Pick & Place demo\n7. Exit")
            choice = input("Select option (1-7): ").strip()
            if choice == '1':
                print("Available:", ", ".join(safe_poses.keys()))
                name = input("Enter pose name: ").strip()
                if name in safe_poses:
                    pos = safe_poses[name]["pos"]; rpy = safe_poses[name]["rpy"]
                    ps = Pose()
                    ps.position.x, ps.position.y, ps.position.z = pos
                    qx,qy,qz,qw = quaternion_from_euler(*rpy)
                    ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w = qx,qy,qz,qw
                    self.move_cartesian(ps, duration=1.0, avoid_obstacles=True)
            elif choice == '2':
                try:
                    pos = [float(input(f"{a} (m): ")) for a in "xyz"]
                    rpy = [float(input(f"{a} (rad): ")) for a in ["roll","pitch","yaw"]]
                    ps = Pose()
                    ps.position.x, ps.position.y, ps.position.z = pos
                    qx,qy,qz,qw = quaternion_from_euler(*rpy)
                    ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w = qx,qy,qz,qw
                    self.move_cartesian(ps, duration=1.0, avoid_obstacles=True)
                except ValueError:
                    print("Invalid input. Please enter numbers.")
            elif choice == '3':
                try:
                    n = int(input("Number of waypoints: "))
                    waypoints = []
                    for i in range(n):
                        print(f"Waypoint {i+1}:")
                        pos = [float(input(f"{a} (m): ")) for a in "xyz"]
                        rpy = [float(input(f"{a} (rad): ")) for a in ["roll","pitch","yaw"]]
                        ps = Pose()
                        ps.position.x, ps.position.y, ps.position.z = pos
                        qx,qy,qz,qw = quaternion_from_euler(*rpy)
                        ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w = qx,qy,qz,qw
                        waypoints.append(ps)
                    self.execute_waypoint_path(waypoints, duration_per_segment=1.0, avoid_obstacles=True)
                except ValueError:
                    print("Invalid input. Please enter numbers.")
            elif choice == '4':
                pose = self.get_current_pose()
                if pose:
                    self._print_pose("Current", pose)
            elif choice == '5':
                if self.obstacles:
                    self.obstacles = []
                    print("Obstacle removed.")
                else:
                    try:
                        cx = float(input("Sphere center x: ")); cy = float(input("y: ")); cz = float(input("z: "))
                        r  = float(input("radius: "))
                        self.obstacles = [SphereObstacle([cx,cy,cz], r)]
                        print("Obstacle added.")
                    except ValueError:
                        print("Invalid input. Please enter numbers.")
            elif choice == '6':
                self.pick_place_demo()
            elif choice == '7':
                break
            else:
                print("Invalid choice.")

    def pick_place_demo(self, pick=[0.35,0.10,0.10], place=[0.25,-0.20,0.15], approach=0.08, lift=0.10):
        pos = safe_poses[name]["pos"]; rpy = safe_poses[name]["rpy"]
        ps = Pose()
        ps.position.x, ps.position.y, ps.position.z = pos
        qx,qy,qz,qw = quaternion_from_euler(*rpy)
        ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w = qx,qy,qz,qw
        self.move_cartesian(ps, duration=1.0, avoid_obstacles=True)
        
        self.get_logger().info("Starting Pick & Place sequence...")
        self.execute_waypoint_path(waypoints, duration_per_segment=1.0, avoid_obstacles=True)
        self.get_logger().info("Pick & Place sequence done.")
