<<<<<<< HEAD
# 6-DOF UR5 Cartesian Control System

A comprehensive robotics control system implementing forward kinematics, inverse kinematics, and Cartesian path planning for the Universal Robots UR5 manipulator using ROS 2 Jazzy.

## 🚀 Features

### Core Functionality
- **Forward Kinematics (FK)**: The system accurately computes the end-effector's pose (position and orientation) by applying a forward kinematics chain based on the Denavit-Hartenberg (DH) parameters. This provides a fundamental mapping from the robot's joint angles to its pose in Cartesian space.
- **Inverse Kinematics (IK)**: The core of the control system uses a hybrid approach to inverse kinematics. It first leverages an analytical IK solver to quickly find an initial set of joint configurations (a seed solution). This solution is then refined by an iterative Damped Least Squares (DLS) method, which ensures accuracy and robustness by minimizing errors while preventing singularities.
- **Cartesian Path Planning**: The system's path planner generates smooth, continuous trajectories in Cartesian space. It interpolates between waypoints, controlling both the robot's position and orientation simultaneously to ensure a fluid and predictable motion.
- **Obstacle Avoidance**: It integrates spherical obstacle detection to prevent collisions. During path execution, the system can automatically generate dynamic path detours to navigate around obstacles without interrupting the overall task.
- **Pick & Place Operations**: The control stack is designed to handle complex manipulation tasks. It includes logic for precise approach and retreat movements, ensuring a secure and efficient sequence for grasping and releasing objects.
- **Workspace Validation**: Before any motion is executed, the system performs real-time workspace validation. This pre-computation step checks if a target pose is reachable within the UR5's kinematic and joint limits, preventing unfeasible motion requests and ensuring task safety and reliability.

## 📁 Repository Structure

```
sixDOF_robot_ws/
├── src/ur5_motion/
│   ├── ur5_motion/
│   │   ├── __init__.py
│   │   ├── main.py                     # ROS 2 entry point
│   │   ├── robot_description.py        # Robot model
│   │   ├── ur5_fk.py                   # Forward kinematics implementation
│   │   ├── ur5_ik.py                   # Hybrid IK solver
│   │   ├── ur5_controller.py           # Main control logic and ROS interface
│   │   └── conversions.py              # Pose/transform utilities
│   ├── launch/
│   │   └── ur5_rviz_view.launch.py     # Visualization launch file
│   ├── tests_added/                    # Pytest test suite
│   ├── package.xml                     # ROS package manifest
│   └── setup.py                        # Python package setup
├── scripts/
│   └── setup_workspace.sh              # Automated workspace setup
├── requirements.txt                    # Python dependencies
└── README.md                           # Installation guide
```

## 🤔 Design Decisions
- This project is built upon ROS 2 Jazzy, leveraging its modern, real-time-capable architecture for efficient inter-node communication. As a Long-Term Support (LTS) release, it provides a stable environment for long-term development. The system's cross-platform nature and its integration with the extensive ROS ecosystem—including tools like RViz and MoveIt—streamlined the development process and enabled powerful visualization capabilities.
- The Universal Robots UR5 was selected as the robot platform. It's a widely-used 6-DOF collaborative robot, making it an excellent choice for real-world application. Its well-documented kinematics and publicly available models provided a solid foundation for implementing custom control algorithms and understanding advanced robotics concepts in a practical setting.
- A custom Inverse Kinematics (IK) implementation was developed to demonstrate a deeper understanding of robotics fundamentals. This hybrid approach combines the speed of an analytical solver with the accuracy of a Damped Least Squares (DLS) method. This choice provides complete control over the solver's behavior and offers a transparent view of the underlying algorithms, which is not possible with black-box libraries.
-MoveIt was integrated primarily for its powerful visualization capabilities within RViz. This provided a seamless way to visualize the robot's state and target poses, even without using MoveIt's motion planning features. This decision aligns the project with ROS standards and prepares it for future extensions, such as incorporating advanced collision detection and full motion planning.
- The project's codebase relies on a curated set of libraries, each selected for its specific strengths. NumPy provides the foundation for all kinematic calculations through its highly efficient and optimized matrix operations. For data integrity and type safety, Pydantic is used for runtime validation of critical robot parameters. Coordinate frame transformations are handled by tf_transformations, a standard ROS library that ensures consistent and reliable spatial reasoning. Finally, the codebase's robustness and functionality are verified with a comprehensive test suite built using pytest.


## 🔧 Installation

### Prerequisites
- Ubuntu 24.04 (Noble Numbat)
- ROS 2 Jazzy Jalopy (pre-installed)

### System Dependencies
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install development tools
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    git \
    wget
```

### ROS 2 Packages
```bash
# Install MoveIt and UR robot packages
sudo apt install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-moveit-py \
    ros-jazzy-ur-moveit-config \
    ros-jazzy-ur-description \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-common \
    ros-jazzy-rviz-default-plugins \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-xacro
```

### Workspace Setup
```bash
# Create workspace
mkdir -p ~/sixDOF_robot_ws/src
cd ~/sixDOF_robot_ws/

# Clone your project
cd src
# Extract/clone your ur5_motion package here

# Install dependencies
cd ~/sixDOF_robot_ws
source /opt/ros/jazzy/setup.bash
python3 -m venv venv
pip3 install -r src/requirements.txt #After sourcing venv add colcon ignore to it, to avoid issues with colcon build

# Initialize rosdep
sudo rosdep init 
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Add your virtual environment path to the pythonpath
export PYTHONPATH="/path/to/your/venv/lib/pythonX.Y/site-packages:$PYTHONPATH"

# Build workspace
colcon build --symlink-install 

# Source workspace
source install/setup.bash
echo "source ~/sixDOF_robot_ws/install/setup.bash" >> ~/.bashrc
```

## 🎮 Usage

### Launch System
Open two terminals:

**Terminal 1 - Robot Visualization:**
```bash
cd ~/sixDOF_robot_ws
source install/setup.bash
ros2 ros2 run ur5_motion ur5_controller
```

**Terminal 2 - Controller:**
```bash
cd ~/sixDOF_robot_ws/src/ur5_motion/ur5_motion/
$ source ~/sixDOF_robot_ws_full_checks/src/ur5_motion/ur5_motion/venv/bin/activate
ros2 launch ur5_motion viz.launch.py ur_type:=ur5
```

### Interactive Commands

The controller presents a menu-driven interface:

```
1. Move to Predefined Pose
2. Move to Custom Pose
3. Execute Cartesian Waypoints
4. Show Current End-Effector Pose
5. Toggle Sphere Obstacle
6. Pick & Place demo
7. Exit
```

## 📝 Command Examples

### 1. Move to Predefined Pose
```
Select option (1-7): 1
Available: home, front, left, right
Enter pose name: front
```
**Result**: Robot smoothly moves to the front position `[0.4, 0.0, 0.3]` with downward orientation.

### 2. Custom Cartesian Move
```
Select option (1-7): 2
x (m): 0.35
y (m): 0.15
z (m): 0.25
roll (rad): 0
pitch (rad): 1.57
yaw (rad): 0
```
**Result**: Robot plans and executes a smooth Cartesian path to the specified pose.

### 3. Multi-Waypoint Path
```
Select option (1-7): 3
Number of waypoints: 3

Waypoint 1:
  x (m): 0.3
  y (m): 0.0
  z (m): 0.3
  roll (rad): 0.0
  pitch (rad): 1.57
  yaw (rad): 0.0

Waypoint 2:
  x (m): 0.3
  y (m): 0.2
  z (m): 0.4
  roll (rad): 0.0
  pitch (rad): 1.57
  yaw (rad): 0.5

Waypoint 3:
  x (m): 0.4
  y (m): 0.0
  z (m): 0.3
  roll (rad): 0.0
  pitch (rad): 1.57
  yaw (rad): 0.0
```
**Result**: Robot executes a continuous path through all waypoints with smooth transitions.

### 4. Current Pose Query
```
Select option (1-7): 4

Output:
Current joint positions (rad): [0.000, -1.570, 1.570, 0.000, 1.570, 0.000]
Current end-effector pose:
  Position: x=0.400, y=0.000, z=0.300
  Orientation (RPY): roll=0.000, pitch=1.570, yaw=0.000
```

### 5. Obstacle Avoidance
```
Select option (1-7): 5
Sphere center x: 0.3
y: 0.0
z: 0.3
radius: 0.1

Select option (1-7): 2
# Now move to a pose that would intersect the obstacle
# The system automatically plans a detour path
```

### 6. Pick & Place Demo
```
Select option (1-7): 6
```
**Result**: Robot executes a complete pick and place sequence:
1. Approach pick location from above
2. Descend to grasp position
3. Simulate grasp (close gripper)
4. Lift object
5. Move to place location
6. Descend to place position
7. Release object (open gripper)
8. Retreat to safe position

**For faster execution**:
```python
# In ur5_controller.py, reduce trajectory steps
steps = max(1, int(duration * 25))  # Instead of 50
```

**For higher accuracy**:
```python
# In ur5_ik.py, increase refinement iterations
max_iters = 20  # Instead of 12
```

## 🧪 Testing

The system includes comprehensive testing at multiple levels:

### Unit Tests
```bash
# Run all tests
source install/setup.bash
pytest -q
```

### Test Coverage
The provided tests cover the core functionalities of the robotics control system, including kinematics, data validation, and path planning.
- **Kinematics**: 
-Forward Kinematics (FK): The tests verify that the fk_ur5 function returns a (4,4) transformation matrix with the correct bottom row.
-Inverse Kinematics (IK): Tests confirm that the ur5_ik_analytic solver provides solutions that result in a small position error when converted back to a pose via FK. The test suite also ensures that the IK solver returns multiple solutions, respects joint limits, and that all returned solutions are kinematically valid.
- **Parameter Validation and Conversions**:
-Robot Description: The tests check that the ur5_description object has correct shapes for its kinematic parameters and that its tool_T matrix has a valid identity row at the bottom.
-Pydantic Validation: The tests verify that pydantic correctly raises a ValidationError when an invalid JointLimit or RobotDescription is instantiated, such as when min is greater than max.
-Coordinate Conversions: The tests confirm that the pose_to_T and T_to_pose functions correctly convert between a geometry_msgs/Pose message and a homogeneous transformation matrix for both identity and random poses.
- **Path Planning and Obstacle Avoidance**:
-Cartesian Path Planning: The tests confirm that linspace_cartesian generates a straight-line path and that the quaternions representing orientation remain normalized.
-Obstacle Avoidance: A test for plan_cartesian_with_obstacles verifies that the path correctly deviates from a straight line when a spherical obstacle is present, ensuring the avoidance logic is functioning.

## 🎯 Working of the Controller

**Forward Kinematics**
-Forward kinematics is the process of calculating the position and orientation of a robot's end-effector based on the values of its joint angles. In the provided files, this is handled by the fk function in ur5_fk.py. This function uses a series of homogeneous transformation matrices, which are built from Denavit-Hartenberg (DH) parameters.
-The robot_description.py file defines the specific DH parameters (a, alpha, and d) for the UR5 robot, which are used to define the geometry of each link and joint. The fk function then multiplies these matrices together to get the final transformation matrix T, representing the end-effector's pose relative to the robot's base. The conversions.py file contains helper functions like pose_to_T and T_to_pose for converting between homogeneous transformation matrices and ROS Pose messages.

**Inverse Kinematics and Interpolation**
-Inverse kinematics is the inverse of forward kinematics: it calculates the joint angles required to move the end-effector to a desired position and orientation. The ur5_ik.py file contains an analytical inverse kinematics solver, ur5_ik_analytic, which can provide multiple possible solutions for a given target pose.
-The ur5_controller.py file uses a hybrid approach with ur5_ik_hybrid. This approach combines the analytical solver with a numerical method called Damped Least Squares to refine the solution and account for obstacles.
-To ensure smooth motion between waypoints, the system uses interpolation. The provided code handles this in two ways 
-Cartesian Interpolation: The linspace_cartesian function, used within the ur5_controller.py, generates a straight-line path in Cartesian space. This is used for precise linear movements, such as approaching or retreating from an object.
-Quaternion Interpolation: For rotational movements, the slerp_quat function is used to perform Spherical Linear Interpolation (Slerp) on the end-effector's orientation. This ensures a smooth, constant-velocity rotation between two orientations without causing unwanted twisting, which can occur with linear interpolation of quaternions.

**Pick and Place Working**
The pick and place operation is a sequence of pre-defined steps to move an object from one location to another. The pick_place_demo function in ur5_controller.py provides a clear example of this process. The typical workflow demonstrated in the code includes:
-Move to a pre-pick pose: The robot moves to a safe position above the object.
-Move to the pick pose: The robot moves down linearly to the object's position, ready to grasp it.
-Grasp: The robot performs the action to grasp the object (e.g., closing a gripper).
-Move to a pre-place pose: The robot lifts the object and moves it to a safe position above the target destination.
-Move to the place pose: The robot moves down linearly to place the object.
-Release: The robot releases the object.
-Retreat: The robot moves back to a safe position.
-In the pick and place demonstration, the robot operates within an environment defined by two key points: a pick point and a place point. Both are represented by Pose objects, which contain hardcoded position and orientation values. The pick point's position is [0.4, 0.3, 0.3] meters, and its orientation is quaternion_from_euler(0, 1.57, 0), which ensures the end-effector faces downward. Similarly, the place point's position is [0.4, -0.3, 0.3] meters with the same downward-facing orientation. The sequence of actions begins with the robot moving to a "pre-pick" position, which is a safe distance of 15 cm (0.15 meters) directly above the pick point. It then descends to the pick point to perform the grasping action, after which it lifts the object back up to a safe offset of 15 cm. The robot then travels to the place location, approaching it at a safe height 15 cm above the place point before descending to release the object. Finally, after placing the object, the robot moves up 15 cm and retreats to a predetermined safe state.

**Robot Base Frame**:
- **Origin**: Center of robot base
- **X-axis**: Forward (positive X away from base)
- **Y-axis**: Left (positive Y to robot's left)
- **Z-axis**: Upward (positive Z points up)
- **Units**: Meters for position, radians for orientation

**Working Envelope**:
- **Reach**: 0.2m to 0.85m from base center
- **Height**: -0.2m to 1.0m from base
- **Rotation**: Full 360° around Z-axis

**Orientation Convention (RPY)**:
- **Roll**: Rotation around X-axis
- **Pitch**: Rotation around Y-axis
- **Yaw**: Rotation around Z-axis
- **Common Values**: 0, ±π/2 (±90°), ±π (±180°)

## Further deevelopment

1. Computer Vision Integration
- Currently, the pick and place locations are defined by hardcoded values in ur5_controller.py. A significant advancement would be to integrate a computer vision system (like an RGB-D camera) to enable the robot to dynamically detect and locate objects in its environment.
- Object Detection: Use image processing or machine learning models to identify the position and type of objects to be picked.
- Dynamic Pick & Place: Instead of static coordinates, the robot would calculate the pick and place poses in real-time based on the detected object's location.

2. Advanced Obstacle Avoidance
The existing project includes a plan_cartesian_with_obstacles function that plans a simple detour around predefined SphereObstacle objects. This could be expanded to handle a more complex and realistic environment.

Environmental Mapping: Use a point cloud from a 3D sensor to create a real-time map of the robot's surroundings.

Dynamic Obstacles: Implement a system to detect and react to moving obstacles or people, ensuring safe operation in a shared workspace.

Path Optimization: Move beyond simple detours to incorporate more sophisticated algorithms that find the most efficient, collision-free path, considering joint limits and singularity avoidance.

3. Advanced Motion Planning Algorithms
While the current system uses linear and quaternion interpolation for basic path planning, a more advanced control system would integrate sophisticated motion planning algorithms. These algorithms can handle complex environments, optimize trajectories for smoothness and speed, and ensure safety in crowded workspaces.

Sampling-Based Planners: Algorithms like Rapidly-exploring Random Tree (RRT) or Probabilistic Roadmap (PRM) are well-suited for high-dimensional robotic systems like the UR5. They efficiently explore the robot's configuration space to find a collision-free path between two points. Integrating these would allow the robot to navigate around obstacles of any shape, moving beyond the simple spherical models currently used.

Optimization-Based Planners: To create smoother and more efficient movements, algorithms such as CHOMP (Covariant Hamiltonian Optimization for Motion Planning) could be implemented. These methods start with an initial path and iteratively refine it to minimize a cost function (e.g., path length, smoothness, or time), resulting in a more natural-looking and energy-efficient trajectory.

4. Human-Robot Interaction (HRI)
The current system is controlled via a simple script. Implementing HRI would make the robot more versatile and user-friendly.

Graphical User Interface (GUI): Develop a GUI where a user can specify pick and place locations with a simple click, or even drag and drop waypoints.

Voice or Gesture Control: Allow the robot to receive commands through spoken language or simple gestures, enabling a more natural interaction with a human operator

## 🔍 Troubleshooting

**"No IK solutions found"**:
- Check if target pose is within workspace bounds
- Verify orientation is achievable (not in singularity)
- Try different approach angles

**"Robot moves erratically"**:
- Ensure joint limits are respected
- Check for singularity avoidance
- Reduce motion speed/increase steps

**"RViz not showing robot"**:
- Verify ROS topics are publishing (`ros2 topic list`)
- Check if robot description is loaded
- Restart RViz with correct launch file

## 🙏 Acknowledgments

- Universal Robots for UR5 specifications and DH parameters
- Open Source Robotics Foundation for ROS 2 framework
- MoveIt community for robotics motion planning standards
