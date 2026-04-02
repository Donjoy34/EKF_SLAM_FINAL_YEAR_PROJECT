FROM osrf/ros:galactic-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-tk \
    python3-scipy \
    python3-pandas \
    libyaml-cpp-dev \
    ros-galactic-gazebo-dev \
    ros-galactic-gazebo-msgs \
    ros-galactic-gazebo-plugins \
    ros-galactic-gazebo-ros \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-ackermann-msgs \
    ros-galactic-xacro \
    ros-galactic-joint-state-publisher \
    ros-galactic-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi
RUN rosdep update

WORKDIR /workspace
COPY src ./src
COPY launch ./launch
COPY demo.rviz ./demo.rviz
COPY README.md ./README.md

RUN source /opt/ros/galactic/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install

RUN echo "source /opt/ros/galactic/setup.bash" >> /root/.bashrc \
    && echo "source /workspace/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
