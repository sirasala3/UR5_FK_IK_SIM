# ur5_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import time
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# Import modular utilities
from ur5_fk import fk_ur5, JOINT_NAMES, JOINT_LIMITS
from ur5_ik import ur5_ik_analytic
from conversions import pose_to_T, T_to_pose

# -------------------------------
# Workspace Analysis
# -------------------------------
def check_workspace_limits(pose: Pose):
    x, y, z = pose.position.x, pose.position.y, pose.position.z
    r = np.sqrt(x**2 + y**2)
    max_reach = 0.85
    min_reach = 0.1
    max_height = 1.0
    min_height = -0.2

    if r > max_reach: return False, f"Too far: {r:.3f} > {max_reach}m"
    if r < min_reach: return False, f"Too close: {r:.3f} < {min_reach}m"
    if z > max_height: return False, f"Too high: {z:.3f} > {max_height}m"
    if z < min_height: return False, f"Too low: {z:.3f} < {min_height}m"
    return True, "Within workspace"

# -------------------------------
# UR5 Controller Node
# -------------------------------
class UR5JazzyController(Node):
    def __init__(self):
        super().__init__("ur5_jazzy_controller")
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # Subscriber
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Current state
        self.current_joints = np.zeros(6)
        self.joint_state_received = False
        
        # Timer for periodic tasks
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("UR5 Jazzy Controller initialized! Waiting for joint states...")

    # ---------------------------
    # ROS Callbacks
    # ---------------------------
    def joint_state_callback(self, msg: JointState):
        joint_map = {JOINT_NAMES.index(name): pos for name, pos in zip(msg.name, msg.position) if name in JOINT_NAMES}
        if len(joint_map) == 6:
            for i in range(6):
                self.current_joints[i] = joint_map[i]
            if not self.joint_state_received:
                self.joint_state_received = True
                self.get_logger().info("Joint states received!")

    def timer_callback(self):
        pass  # Could be used for periodic updates

    # ---------------------------
    # Publishing Functions
    # ---------------------------
    def publish_joint_target(self, joint_positions, velocity=None):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(q) for q in joint_positions]
        msg.velocity = [0.0]*6 if velocity is None else [float(v) for v in velocity]
        msg.effort = [0.0]*6
        self.joint_pub.publish(msg)
        self.get_logger().info(f"Published joint target: {[f'{q:.3f}' for q in joint_positions]}")

    def publish_pose_target(self, pose: Pose):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose = pose
        self.target_pose_pub.publish(msg)

    # ---------------------------
    # Motion Functions
    # ---------------------------
    def move_to_pose(self, target_pose: Pose, use_current_as_reference=True):
        # Check workspace
        in_workspace, msg = check_workspace_limits(target_pose)
        if not in_workspace:
            self.get_logger().warn(f"Target pose out of workspace: {msg}")
            return False

        # Publish pose for visualization
        self.publish_pose_target(target_pose)
        
        # Compute IK
        T_target = pose_to_T(target_pose)
        solutions = ur5_ik_analytic(T_target)
        if not solutions:
            self.get_logger().error("No IK solutions found for target pose")
            return False

        # Choose closest solution
        reference_joints = self.current_joints if use_current_as_reference else np.zeros(6)
        best_solution = min(solutions, key=lambda q: np.linalg.norm(np.array(q) - reference_joints))

        # FK verification
        pose_fk = T_to_pose(fk_ur5(best_solution))
        pos_error = np.linalg.norm([
            target_pose.position.x - pose_fk.position.x,
            target_pose.position.y - pose_fk.position.y,
            target_pose.position.z - pose_fk.position.z
        ])
        #if pos_error > 0.01:
        #   self.get_logger().warn(f"FK verification error: {pos_error:.4f}m")
        #else:
        #   self.get_logger().info(f"FK verification passed. Error: {pos_error:.6f}m")

        # Move smoothly
        self.move_to_joint_positions_smooth(best_solution, duration=4.0)
        return True

    def move_to_joint_positions_smooth(self, target_joints, duration=3.0, rate_hz=50):
        current_joints = np.array(self.current_joints)
        target_joints = np.array(target_joints)
        steps = int(duration * rate_hz)
        for i in range(1, steps+1):
            interpolated = current_joints + (target_joints - current_joints) * i / steps
            self.publish_joint_target(interpolated.tolist())
            time.sleep(1 / rate_hz)
        self.publish_joint_target(target_joints.tolist())
        self.current_joints = np.array(target_joints)

    def get_current_pose(self):
        return T_to_pose(fk_ur5(self.current_joints))

    # ---------------------------
    # Interactive Demo
    # ---------------------------
    def run_demo(self):
        self.get_logger().info("=== UR5 Jazzy Controller Demo ===")
        safe_poses = {
            'home': {'pos': [0.0, 0.0, 0.5], 'rpy': [0, 1.57, 0]},
            'front': {'pos': [0.4, 0.0, 0.3], 'rpy': [0, 1.57, 0]},
            'left': {'pos': [0.3, 0.3, 0.3], 'rpy': [0, 1.57, 0.5]},
            'right': {'pos': [0.3, -0.3, 0.3], 'rpy': [0, 1.57, -0.5]},
        }
        while True:
            try:
                print("\n=== UR5 Jazzy Controller ===")
                print("1. Predefined pose  2. Custom pose  3. Waypoints  4. Current pose  5. Workspace check  6. IK solutions  7. Exit")
                choice = input("Select option (1-7): ").strip()
                if choice == '1':
                    self._demo_predefined_poses(safe_poses)
                elif choice == '2':
                    self._demo_custom_pose()
                elif choice == '3':
                    self._show_all_ik_solutions()
                elif choice == '4':
                    self._show_current_pose()
               # elif choice == '5':
               #    self._test_workspace_point()
               # elif choice == '6':
               #    self._demo_waypoint_sequence()
                elif choice == '5':
                    break
                else:
                    print("Invalid choice.")
            except KeyboardInterrupt:
                self.get_logger().info("Demo interrupted")
                break

    # ---------------------------
    # Demo Helper Functions
    # ---------------------------
    def _create_pose(self, position, rpy):
        qx, qy, qz, qw = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        return pose

    def _demo_predefined_poses(self, poses):
        print("Available poses:", ", ".join(poses.keys()))
        name = input("Enter pose name: ").strip().lower()
        if name in poses:
            data = poses[name]
            self.move_to_pose(self._create_pose(data['pos'], data['rpy']))
        else:
            print("Unknown pose")

    def _demo_custom_pose(self):
        try:
            pos = [float(input(f"{axis} (m): ")) for axis in "xyz"]
            rpy = [float(input(f"{r} (rad): ")) for r in "roll pitch yaw".split()]
            self.move_to_pose(self._create_pose(pos, rpy))
        except ValueError as e:
            print("Invalid input:", e)

    def _demo_waypoint_sequence(self):
        try:
            n = int(input("Number of waypoints: "))
            waypoints = []
            for i in range(n):
                print(f"Waypoint {i+1}:")
                pos = [float(input(f"{axis} (m): ")) for axis in "xyz"]
                rpy = [float(input(f"{r} (rad): ")) for r in "roll pitch yaw".split()]
                waypoints.append(self._create_pose(pos, rpy))
            delay = float(input("Delay between waypoints (s, default 1.0): ") or "1.0")
            for wp in waypoints:
                self.move_to_pose(wp)
                time.sleep(delay)
        except ValueError as e:
            print("Invalid input:", e)

    def _show_current_pose(self):
        pose = self.get_current_pose()
        roll, pitch, yaw = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        print(f"Position: {pose.position.x:.3f},{pose.position.y:.3f},{pose.position.z:.3f}")
        print(f"Orientation RPY: {roll:.3f},{pitch:.3f},{yaw:.3f}")

    def _test_workspace_point(self):
        try:
            pos = [float(input(f"{axis} (m): ")) for axis in "xyz"]
            pose = self._create_pose(pos, [0,1.57,0])
            in_ws, msg = check_workspace_limits(pose)
            print("Workspace check:", msg)
        except ValueError as e:
            print("Invalid input:", e)

    def _show_all_ik_solutions(self):
        try:
            pos = [float(input(f"{axis} (m): ")) for axis in "xyz"]
            rpy = [float(input(f"{r} (rad): ")) for r in "roll pitch yaw".split()]
            pose = self._create_pose(pos, rpy)
            sols = ur5_ik_analytic(pose_to_T(pose))
            print(f"Found {len(sols)} IK solutions")
            for i, sol in enumerate(sols):
                print(f"Solution {i+1}: {[f'{q:.3f}' for q in sol]}")
        except ValueError as e:
            print("Invalid input:", e)

