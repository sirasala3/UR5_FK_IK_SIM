<<<<<<< HEAD
# UR5_FK_IK_SIM
Foward kinematics, Inverse Kinematics, Displacement and simulation of UR5 robot
=======
# 6-DOF UR5 Cartesian Control — Challenge-Ready Submission (ROS 2 Jazzy)

This repository implements the full coding challenge using a UR5 model and pure Python FK/IK (no vendor SDK required). MoveIt is optional. The program:
- Defines the robot structure (DH), joint limits, and provides **FK & analytic IK**
- Accepts **Cartesian targets** (x, y, z, roll, pitch, yaw) and checks reachability
- Generates **smooth Cartesian paths** (linear XYZ + slerp orientation) and executes them by solving IK per waypoint
- Prints **end-effector pose at each step** (position + RPY)
- **Optional:** obstacle avoidance using a simple spherical obstacle with an automatic detour waypoint
- **Optional:** a simple **pick & place** routine

## Repository Layout

```
sixDOF_robot_ws/
  ├─ src/ur5_motion/
  │   ├─ ur5_motion/
  │   │   ├─ ur5_fk.py                 # DH, FK, limits
  │   │   ├─ ur5_ik.py                 # Analytic IK
  │   │   ├─ conversions.py            # Pose/Transform utilities
  │   │   ├─ ur5_controller.py         # Controller (Cartesian path, obstacle, pick&place)
  │   │   ├─ main.py                   # rclpy entrypoint
  │   │   └─ __init__.py
  │   ├─ launch/
  │   │   ├─ ur5_demo.launch.py        # Launch only the controller
  │   │   └─ ur5_rviz_view.launch.py   # Launch UR5 description RViz view (optional)
  │   ├─ tests_added/                  # Pytests for functionality & edge cases
  │   ├─ package.xml
  │   └─ setup.py
  ├─ scripts/
  │   └─ setup_workspace.sh            # One-shot workspace setup + build
  ├─ Dockerfile
  └─ docker-entrypoint.sh
```

---

## Quickstart (Native)

> **Prereqs:** Ubuntu 24.04 + ROS 2 **Jazzy** already installed.

```bash
# 1) Build
cd ~/sixDOF_robot_ws
# Install Python deps
pip install -r requirements.txt

# Install ROS/system deps
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# 2) Terminal A — (optional) RViz UR5 visualization
source install/setup.bash
ros2 launch ur5_motion ur5_rviz_view.launch.py

# 3) Terminal B — run the controller
source install/setup.bash
ros2 launch ur5_motion ur5_demo.launch.py
# or directly: ros2 run ur5_motion ur5_controller
```

### Interactive console — See each challenge step

When the controller starts, you get a menu:

1. **Move to Predefined Pose** → shows **FK/IK**, reachability, and prints EE states.
2. **Move to Custom Pose** → enter `x y z roll pitch yaw`, solver checks reachability and prints EE states every step.
3. **Execute Cartesian Waypoints** → generates a sequence of intermediate Cartesian poses and executes them smoothly.
4. **Show Current End-Effector Pose** → prints FK of current joints.
5. **Toggle Sphere Obstacle** → add/remove a spherical obstacle; the Cartesian planner auto-inserts a detour if needed.
6. **Pick & Place demo** → approach → pick → lift → move → place with tool-down orientation (no perception).
7. **Exit**

> During execution the EE pose (position & RPY) is **printed at every step**, which satisfies the visualization/logging requirement.

---

## Tests

```bash
source install/setup.bash
pytest src/ur5_motion/tests_added -q
```

Tests cover:
- FK shape and numerical sanity
- IK → FK closure (position error small)
- Reachability edge cases (targets outside workspace)
- Cartesian interpolation properties (straight line & normalized quaternions)
- Obstacle planner behavior (detour is created when the straight line intersects a sphere)

---

## Docker (Optional)

Build:
```bash
docker build -t ur5-challenge:jazzy .
```

Run:
```bash
# RViz (if you have X11/Wayland; otherwise run headless)
xhost +local:root
docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  ur5-challenge:jazzy

# Inside container:
source /opt/ros/jazzy/setup.bash
cd /workspace && colcon build --symlink-install && source install/setup.bash
ros2 launch ur5_motion ur5_demo.launch.py
```

---

## Notes

- The solution uses **pure Python FK/IK** and **rclpy** publishers. MoveIt is **not required** for the challenge; you may add it if you want PlanningScene & RViz collision objects.
- All function and file names are descriptive and consistent.
- The obstacle model is intentionally simple (spherical) to keep this challenge concise.


### Tests Layout
All tests live under **`src/ur5_motion/test/`** (single directory). Run them with:
```bash
source install/setup.bash
pytest src/ur5_motion/test -q
```


---

## Minimal ROS vs No-ROS

This repository keeps **minimal ROS** (a small `rclpy` node) to allow RViz viewing and future integration. If you want to run **without ROS**, use the pure-Python visualizer:

```bash
# deps
pip install -r requirements.txt matplotlib

# run (from workspace root)
PYTHONPATH=./src python scripts/viz_no_ros.py --target 0.35 0.15 0.25 0 1.57 0 --steps 60

# with obstacle (cx cy cz r)
PYTHONPATH=./src python scripts/viz_no_ros.py --target 0.35 0.0 0.20 0 1.57 0 --obstacle 0.30 0.00 0.20 0.07 --steps 60
```


## Robot Structure as a Data Object (Validated)

We define the UR5 model in **`ur5_motion/robot_description.py`**:

- `RobotDescription` dataclass: joint names, DH arrays (`a`, `alpha`, `d`), joint limits, and tool transform (`tool_T`).
- `validate()` enforces shape/size constraints, finite limits, and a proper 4×4 tool transform.
- `ur5_description()` builds a ready-to-use UR5 instance and calls `validate()`.

Both **FK** (`ur5_motion/ur5_fk.py`) and **IK** (`ur5_motion/ur5_ik.py`) consume this description, keeping a **single source of truth** for the arm’s structure.


### Type- & Schema-Safety
- The robot model is defined with **Pydantic v2** (`RobotDescription`, `JointLimit`), which validates:
  - shapes of DH arrays (`(6,)`),
  - 4×4 `tool_T` with the correct last row,
  - joint limit ordering (`min < max`),
  - exactly 6 joint names and 6 joint limits.
- We include **mypy** configuration in `pyproject.toml` for static type checking:
```bash
mypy
```


## Quality Gates (run before you ship)
```bash
# Install dev tools
pip install -r requirements.txt

# Type check
mypy

# Lint & format
ruff check
ruff format --check  # or: ruff format  (to apply)

# Run tests
pytest -q

# Enable pre-commit hooks (runs ruff + mypy on each commit)
pre-commit install
```
>>>>>>> 2e21357 (Initial commit)
