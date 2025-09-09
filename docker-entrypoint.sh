#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
cd /workspace
source install/setup.bash || true
exec "$@"
