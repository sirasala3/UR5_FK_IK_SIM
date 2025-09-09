#!/usr/bin/env bash
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "[Setup] Workspace: $WS_DIR"
source /opt/ros/jazzy/setup.bash
cd "$WS_DIR"

# Python requirements
python3 -m pip install --upgrade pip
pip install -r requirements.txt

# ROS/system deps
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Auto-source
grep -qxF "source $WS_DIR/install/setup.bash" ~/.bashrc || echo "source $WS_DIR/install/setup.bash" >> ~/.bashrc
echo "[Setup] Done."
