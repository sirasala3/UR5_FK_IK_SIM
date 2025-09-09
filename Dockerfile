FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-c"]

# Tools
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool git \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace/

# Python deps
RUN python3 -m pip install --upgrade pip && pip install -r requirements.txt

# ROS/system deps
RUN rosdep init 2>/dev/null || true && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y

# Build
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

# Env
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
 && echo "source /workspace/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/workspace/docker-entrypoint.sh"]
