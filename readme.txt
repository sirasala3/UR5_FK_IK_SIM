================================================================================
                    UR5 Robot Demo with MoveIt - Installation Guide
================================================================================

Project: 6-DOF Robot Arm Control System
ROS2: Jazzy Jalopy
Robot: Universal Robots UR5
Motion Planning: MoveIt2

================================================================================
                                Prerequisites
================================================================================

- Operating System: Ubuntu 24.04 Noble Numbat
- ROS2 Distribution: Jazzy (must be pre-installed)
- Hardware: Minimum 4GB RAM, 20GB free disk space
- Network: Internet connection for package downloads

================================================================================
                          INSTALLATION INSTRUCTIONS
================================================================================

Step 1: Install System Dependencies
-----------------------------------

# Update system packages
sudo apt update && sudo apt upgrade -y

# Install development tools
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    git \
    wget

# Install Python dependencies
pip3 install transforms3d numpy


Step 2: Install ROS2 Packages
------------------------------

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


Step 3: Workspace Setup
-----------------------

# Create and setup workspace
mkdir -p ~/sixDOF_robot_ws/src
cd ~/sixDOF_robot_ws/

# Initialize rosdep (if not done before)
sudo rosdep init 2>/dev/null || true
rosdep update

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Clone required repositories
cd src
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
git clone -b ros2 https://github.com/ros-industrial/universal_robot.git

# Extract your custom package here (ur5_motion)
# Place your ur5_motion package in ~/sixDOF_robot_ws/src/

# Install workspace dependencies
cd ~/sixDOF_robot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
echo "source ~/sixDOF_robot_ws/install/setup.bash" >> ~/.bashrc

================================================================================
                              LAUNCHING THE SYSTEM
================================================================================

You need to open THREE terminals and run the following commands in sequence:

Terminal 1 - Launch Robot Visualization:
----------------------------------------
cd ~/sixDOF_robot_ws
source install/setup.bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5

Terminal 2 - Launch MoveIt Motion Planning:

cd ~/sixDOF_robot_ws
source install/setup.bash
python3 src/ur5_motion/ur5_motion/main.py if not working run in venv

================================================================================
                          INTERACTIVE CONSOLE USAGE
================================================================================

Once the controller is running, you'll see a menu with 7 options:

Menu Options:
1. Move to Predefined Pose
2. Move to Custom Pose  
3. Execute Waypoint Sequence
4. Show Current End-Effector Pose
5. Test if Point is in Workspace
6. Show All IK Solutions
7. Exit

================================================================================
                              USAGE EXAMPLES
================================================================================

Example 1: Move to Predefined Pose
-----------------------------------
Select option (1-7): 1

Available poses:
  home: pos=[0.0, 0.0, 0.5], rpy=[0.00, 1.57, 0.00]
  front: pos=[0.4, 0.0, 0.3], rpy=[0.00, 1.57, 0.00]
  left: pos=[0.3, 0.3, 0.3], rpy=[0.00, 1.57, 0.50]
  right: pos=[0.3, -0.3, 0.3], rpy=[0.00, 1.57, -0.50]

Enter pose name: front

Result: Robot moves to the front position safely.


Example 2: Move to Custom Pose
-------------------------------
Select option (1-7): 2

Enter target pose:
x (m): 0.3
y (m): 0.0
z (m): 0.3
roll (rad): 0.0
pitch (rad): 1.57
yaw (rad): 0.0

Result: Robot moves to the specified custom position and orientation.


Example 3: Execute Waypoint Sequence
-------------------------------------
Select option (1-7): 3

Number of waypoints: 2

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
  z (m): 0.3
  roll (rad): 0.0
  pitch (rad): 1.57
  yaw (rad): 0.0

Delay between waypoints (seconds, default 1.0): 1.0

Result: Robot moves through each waypoint sequentially with 1-second delays.


Example 4: Show Current End-Effector Pose
------------------------------------------
Select option (1-7): 4

Output:
Current joint positions (rad): [0.000, -1.570, 1.570, 0.000, 1.570, 0.000]
Current end-effector pose:
  Position: x=0.300, y=0.000, z=0.300
  Orientation (RPY): roll=0.000, pitch=1.570, yaw=0.000
  Orientation (Quat): x=0.000, y=0.707, z=0.000, w=0.707


Example 5: Test if Point is in Workspace
-----------------------------------------
Select option (1-7): 5

Enter point to test:
x (m): 0.3
y (m): 0.0
z (m): 0.3

Output:
Workspace check: Within workspace
Found 2 IK solutions
First solution (rad): [0.000, -1.200, 1.000, 0.000, 1.570, 0.000]


Example 6: Show All IK Solutions
---------------------------------
Select option (1-7): 6

Enter pose for IK analysis:
x (m): 0.3
y (m): 0.0
z (m): 0.3
roll (rad): 0.0
pitch (rad): 1.57
yaw (rad): 0.0

Output:
Found 2 IK solutions:
Solution 1: [0.000, -1.200, 1.000, 0.000, 1.570, 0.000]
  FK verification error: 0.002 m
Solution 2: [0.000, -1.000, 1.200, 0.000, 1.570, 0.000]
  FK verification error: 0.003 m


Example 7: Exit Program
------------------------
Select option (1-7): 7

Output: Exiting UR5 Jazzy Controller...

================================================================================
                              COORDINATE SYSTEM
================================================================================

Robot Base Frame:
- Origin: Center of robot base
- X-axis: Forward direction (positive X points away from base)
- Y-axis: Left direction (positive Y points to robot's left)
- Z-axis: Upward direction (positive Z points up)
- Units: Meters for position, Radians for orientation

Working Range:
- Typical reach: 0.2m to 0.85m from base center
- Height range: -0.2m to 1.0m from base
- Full 360° rotation around Z-axis

Orientation (Roll-Pitch-Yaw):
- Roll: Rotation around X-axis
- Pitch: Rotation around Y-axis  
- Yaw: Rotation around Z-axis
- Common values: 0, ±1.57 (±90°), ±3.14 (±180°)


