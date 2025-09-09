# ur5_controller_enhanced.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from robot_description import ur5_description
from ur5_fk import fk_ur5, JOINT_LIMITS
from ur5_ik import ur5_ik_analytic
from conversions import pose_to_T, T_to_pose
from builtin_interfaces.msg import Duration
from threading import Lock

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def within_limits(q):
    return all(JOINT_LIMITS[i][0] - 1e-6 <= q[i] <= JOINT_LIMITS[i][1] + 1e-6 for i in range(6))

def slerp_quat(q0, q1, s):
    q0 = np.array(q0, dtype=float)
    q1 = np.array(q1, dtype=float)
    dot = np.dot(q0, q1)
    if dot < 0: 
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        result = q0 + s*(q1-q0)
        return result / np.linalg.norm(result)
    th0 = np.arccos(np.clip(dot, -1, 1))
    th = th0 * s
    q2 = q1 - q0*dot
    q2 /= np.linalg.norm(q2) + 1e-12
    return q0*np.cos(th) + q2*np.sin(th)

class SphereObstacle:
    def __init__(self, center, radius, name="obstacle"):
        self.c = np.array(center, dtype=float)
        self.r = float(radius)
        self.name = name

class UR5EnhancedController(Node):
    def __init__(self):
        super().__init__('ur5_enhanced_controller')
        
        # Publishers
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_target = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.pub_trajectory = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.pub_gripper = self.create_publisher(JointState, '/gripper_states', 10)
        
        # State
        self.current_joints = np.zeros(6)
        self.obstacles = {}  # dict of name -> SphereObstacle
        self.gripper_state = 0.0  # 0=open, 1=closed
        self.execution_lock = Lock()
        self.is_executing = False
        
        # Predefined poses
        self.safe_poses = {
            "home": {"pos": [0.3, 0.0, 0.4], "rpy": [0, np.pi/2, 0]},
            "front": {"pos": [0.4, 0.0, 0.3], "rpy": [0, np.pi/2, 0]},
            "left": {"pos": [0.3, 0.3, 0.3], "rpy": [0, np.pi/2, np.pi/6]},
            "right": {"pos": [0.3, -0.3, 0.3], "rpy": [0, np.pi/2, -np.pi/6]},
            "pick_approach": {"pos": [0.35, 0.1, 0.18], "rpy": [0, np.pi/2, 0]},
            "place_approach": {"pos": [0.25, -0.2, 0.23], "rpy": [0, np.pi/2, 0]}
        }
        
        # Timer for marker publishing
        self.marker_timer = self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info("Enhanced UR5 Controller initialized")

    def publish_markers(self):
        """Publish collision objects as visualization markers for RViz"""
        marker_array = MarkerArray()
        
        # Publish obstacles
        for idx, (name, obs) in enumerate(self.obstacles.items()):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = obs.c[0]
            marker.pose.position.y = obs.c[1]
            marker.pose.position.z = obs.c[2]
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = obs.r * 2
            marker.scale.y = obs.r * 2
            marker.scale.z = obs.r * 2
            
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            marker.lifetime = Duration(sec=0, nanosec=200000000)  # 0.2 seconds
            
            marker_array.markers.append(marker)
        
        # Publish end-effector marker
        ee_pose = self.get_current_pose()
        ee_marker = Marker()
        ee_marker.header.frame_id = "base_link"
        ee_marker.header.stamp = self.get_clock().now().to_msg()
        ee_marker.ns = "end_effector"
        ee_marker.id = 100
        ee_marker.type = Marker.ARROW
        ee_marker.action = Marker.ADD
        
        ee_marker.pose = ee_pose
        ee_marker.scale.x = 0.1
        ee_marker.scale.y = 0.02
        ee_marker.scale.z = 0.02
        ee_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        ee_marker.lifetime = Duration(sec=0, nanosec=200000000)
        
        marker_array.markers.append(ee_marker)
        
        # Publish trajectory path if executing
        if self.is_executing and hasattr(self, 'current_trajectory'):
            path_marker = Marker()
            path_marker.header.frame_id = "base_link"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "trajectory"
            path_marker.id = 200
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            
            path_marker.scale.x = 0.005
            path_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)
            
            for pose in self.current_trajectory:
                p = Point()
                p.x = pose.position.x
                p.y = pose.position.y
                p.z = pose.position.z
                path_marker.points.append(p)
            
            path_marker.lifetime = Duration(sec=0, nanosec=500000000)
            marker_array.markers.append(path_marker)
        
        self.pub_markers.publish(marker_array)

    def publish_joint_target(self, q):
        msg = JointState(
            header=Header(stamp=self.get_clock().now().to_msg()),
            name=ur5_description().joint_names,
            position=q
        )
        self.pub_joint.publish(msg)

    def publish_gripper_state(self, state):
        """Publish gripper state (0=open, 1=closed)"""
        msg = JointState(
            header=Header(stamp=self.get_clock().now().to_msg()),
            name=['gripper_finger_joint'],
            position=[state * 0.04]  # Max opening 4cm
        )
        self.pub_gripper.publish(msg)
        self.gripper_state = state

    def publish_trajectory(self, joint_positions_list, time_from_start_list):
        """Publish a joint trajectory"""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ur5_description().joint_names
        
        for q, t in zip(joint_positions_list, time_from_start_list):
            point = JointTrajectoryPoint()
            point.positions = q.tolist() if isinstance(q, np.ndarray) else q
            point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            traj.points.append(point)
        
        self.pub_trajectory.publish(traj)

    def get_current_pose(self):
        return T_to_pose(fk_ur5(self.current_joints))

    def add_obstacle(self, name, center, radius):
        """Add a collision obstacle"""
        self.obstacles[name] = SphereObstacle(center, radius, name)
        self.get_logger().info(f"Added obstacle '{name}' at {center} with radius {radius}")

    def remove_obstacle(self, name):
        """Remove a collision obstacle"""
        if name in self.obstacles:
            del self.obstacles[name]
            self.get_logger().info(f"Removed obstacle '{name}'")
            return True
        return False

    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()
        self.get_logger().info("Cleared all obstacles")

    def check_collision(self, p0, p1):
        """Check if line segment collides with any obstacle"""
        for obs in self.obstacles.values():
            if self._segment_hits_sphere(p0, p1, obs.c, obs.r * 1.1):  # 10% safety margin
                return True
        return False

    def _segment_hits_sphere(self, p0, p1, c, r):
        """Check if line segment p0-p1 intersects sphere at c with radius r"""
        v = p1 - p0
        w = p0 - c
        a = v @ v
        if a < 1e-9:
            return np.linalg.norm(w) <= r
        b = 2 * (w @ v)
        c_term = w @ w - r * r
        disc = b * b - 4 * a * c_term
        if disc < 0:
            return False
        
        sqrt_disc = np.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2 * a)
        t2 = (-b + sqrt_disc) / (2 * a)
        
        # Check if intersection points are within segment
        if (0 <= t1 <= 1) or (0 <= t2 <= 1):
            return True
        
        # Check if segment is entirely inside sphere
        if t1 < 0 and t2 > 1:
            return True
        
        return False

    def plan_cartesian_path(self, target_pose: Pose, steps=50, avoid_obstacles=True):
        """Plan a Cartesian path with optional obstacle avoidance"""
        start_pose = self.get_current_pose()
        p0 = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
        p1 = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
        
        if not avoid_obstacles or not self.check_collision(p0, p1):
            # Direct path is clear
            return self._interpolate_poses(start_pose, target_pose, steps)
        
        # Need to plan around obstacles - use simple waypoint approach
        self.get_logger().warn("Collision detected, planning alternative path")
        
        # Find a waypoint that avoids obstacles
        mid = 0.5 * (p0 + p1)
        
        # Try multiple bypass directions
        bypass_found = False
        for direction in [np.array([0, 0, 1]), np.array([0, 1, 0]), np.array([1, 0, 0])]:
            for offset in [0.15, 0.25, 0.35]:
                waypoint = mid + direction * offset
                if not self.check_collision(p0, waypoint) and not self.check_collision(waypoint, p1):
                    bypass_found = True
                    break
            if bypass_found:
                break
        
        if not bypass_found:
            self.get_logger().error("Could not find collision-free path")
            return [target_pose]
        
        # Create waypoint pose
        waypoint_pose = Pose()
        waypoint_pose.position.x = waypoint[0]
        waypoint_pose.position.y = waypoint[1]
        waypoint_pose.position.z = waypoint[2]
        waypoint_pose.orientation = target_pose.orientation
        
        # Combine paths
        path1 = self._interpolate_poses(start_pose, waypoint_pose, steps // 2)
        path2 = self._interpolate_poses(waypoint_pose, target_pose, steps - len(path1) + 1)
        
        return path1[:-1] + path2

    def _interpolate_poses(self, start_pose: Pose, end_pose: Pose, steps: int):
        """Interpolate between two poses"""
        poses = []
        p0 = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
        p1 = np.array([end_pose.position.x, end_pose.position.y, end_pose.position.z])
        
        q0 = np.array([start_pose.orientation.x, start_pose.orientation.y,
                      start_pose.orientation.z, start_pose.orientation.w])
        q1 = np.array([end_pose.orientation.x, end_pose.orientation.y,
                      end_pose.orientation.z, end_pose.orientation.w])
        
        for i in range(steps):
            s = i / (steps - 1) if steps > 1 else 1.0
            
            # Linear interpolation for position
            p = (1 - s) * p0 + s * p1
            
            # Spherical linear interpolation for orientation
            q = slerp_quat(q0, q1, s)
            
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = p[2]
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            poses.append(pose)
        
        return poses

    def move_cartesian(self, target_pose: Pose, velocity=0.05, avoid_obstacles=True):
        """Move to target pose following a Cartesian path"""
        with self.execution_lock:
            self.is_executing = True
            
            # Plan path
            path = self.plan_cartesian_path(target_pose, steps=50, avoid_obstacles=avoid_obstacles)
            self.current_trajectory = path
            
            # Publish target
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = "base_link"
            ps.pose = target_pose
            self.pub_target.publish(ps)
            
            # Execute path
            joint_trajectory = []
            time_stamps = []
            
            for i, pose in enumerate(path):
                sols = ur5_ik_analytic(pose_to_T(pose))
                if not sols:
                    self.get_logger().warn(f"No IK solution for waypoint {i}")
                    continue
                
                # Select best solution (closest to current)
                q_best = min(sols, key=lambda q: np.linalg.norm(np.array(q) - self.current_joints))
                joint_trajectory.append(q_best)
                time_stamps.append(i * 0.1)  # 10Hz execution
                
                # Publish current state
                self.publish_joint_target(q_best)
                self.current_joints = np.array(q_best)
                time.sleep(0.1)
            
            # Publish complete trajectory
            if joint_trajectory:
                self.publish_trajectory(joint_trajectory, time_stamps)
            
            self.is_executing = False
            return True

    def move_to_named_pose(self, name):
        """Move to a predefined pose"""
        if name not in self.safe_poses:
            self.get_logger().error(f"Unknown pose: {name}")
            return False
        
        pose_def = self.safe_poses[name]
        pose = Pose()
        pose.position.x = pose_def["pos"][0]
        pose.position.y = pose_def["pos"][1]
        pose.position.z = pose_def["pos"][2]
        
        q = quaternion_from_euler(*pose_def["rpy"])
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        return self.move_cartesian(pose)

    def pick_and_place(self, pick_pos, place_pos, approach_height=0.08, gripper_delay=0.5):
        """Execute pick and place sequence with gripper control"""
        self.get_logger().info(f"Starting pick and place: {pick_pos} -> {place_pos}")
        
        # Define downward orientation
        down_orientation = quaternion_from_euler(0, np.pi/2, 0)
        
        def make_pose(pos):
            pose = Pose()
            pose.position.x = pos[0]
            pose.position.y = pos[1]
            pose.position.z = pos[2]
            pose.orientation.x = down_orientation[0]
            pose.orientation.y = down_orientation[1]
            pose.orientation.z = down_orientation[2]
            pose.orientation.w = down_orientation[3]
            return pose
        
        try:
            # 1. Open gripper
            self.publish_gripper_state(0.0)
            time.sleep(gripper_delay)
            
            # 2. Move to approach position above pick
            approach_pos = [pick_pos[0], pick_pos[1], pick_pos[2] + approach_height]
            self.move_cartesian(make_pose(approach_pos), avoid_obstacles=True)
            
            # 3. Move down to pick position
            self.move_cartesian(make_pose(pick_pos), avoid_obstacles=False)
            
            # 4. Close gripper
            self.publish_gripper_state(1.0)
            time.sleep(gripper_delay)
            
            # 5. Lift object
            lift_pos = [pick_pos[0], pick_pos[1], pick_pos[2] + approach_height * 1.5]
            self.move_cartesian(make_pose(lift_pos), avoid_obstacles=False)
            
            # 6. Move to approach position above place
            place_approach = [place_pos[0], place_pos[1], place_pos[2] + approach_height]
            self.move_cartesian(make_pose(place_approach), avoid_obstacles=True)
            
            # 7. Move down to place position
            self.move_cartesian(make_pose(place_pos), avoid_obstacles=False)
            
            # 8. Open gripper
            self.publish_gripper_state(0.0)
            time.sleep(gripper_delay)
            
            # 9. Retreat
            retreat_pos = [place_pos[0], place_pos[1], place_pos[2] + approach_height]
            self.move_cartesian(make_pose(retreat_pos), avoid_obstacles=False)
            
            self.get_logger().info("Pick and place completed successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Pick and place failed: {e}")
            return False

    def get_status(self):
        """Get current controller status"""
        return {
            'is_executing': self.is_executing,
            'current_joints': self.current_joints.tolist(),
            'current_pose': self.get_current_pose(),
            'gripper_state': self.gripper_state,
            'obstacles': list(self.obstacles.keys())
        }
