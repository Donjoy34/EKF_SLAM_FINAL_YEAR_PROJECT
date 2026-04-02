This repository maintains the FS-AI developments for the 2023 competition at Silverstone.

## Prerequisites

### Supported OS and toolchain
- **Linux:** Ubuntu 20.04 LTS (recommended)
- **Python:** 3.8 (default on Ubuntu 20.04)
- **ROS 2:** Galactic (desktop)

- Install Ubuntu 20.04 LTS
- Install [ros-galactic-desktop](http://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

**Install colcon**

```shell
$ sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

$ sudo apt update
$ sudo apt install python3-colcon-common-extensions
```

## Clone the repo

Setup ssh-key following the instructions [here](https://gitlab.com/uh4662410/uhra/training/herts-autonomous/-/wikis/ssh-key-setup-and-configure-git) to enable you to authenticates to the GitLab server without using username and password each time.

```shell
git clone --recurse-submodules -j8 git@gitlab.com:uh4662410/uhra/uh-fs-ai.git
```

## Offline bundle (zip/tar)

If you received an archive instead of Git, extract it and run from the repo root.

**For .tar.gz bundles**
```shell
tar -xzf uh-fs-ai_bundle.tar.gz
cd uh-fs-ai
```

**For .zip bundles**
```shell
unzip uh-fs-ai_bundle.zip
cd uh-fs-ai
```

**Repo root folder name:** `uh-fs-ai`

**Setup EUFS_MASTER Variable** 

execute `pwd` command from your `uh-fs-ai` directory and replace `/path/to/the/directory` in the below command with output of `pwd` command to set the path of this directory as the EUFS_MASTER environment variable 

`Example : echo 'export EUFS_MASTER=/home/don/uh-fs-ai' >> ~/.bashrc`


```shell
echo 'export EUFS_MASTER=/path/to/the/directory' >> ~/.bashrc
source ~/.bashrc
```

Install dependencies through rosdep.

```shell
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths $EUFS_MASTER --ignore-src -r -y
```
Few additional dependencies:

```shell
sudo apt install ros-${ROS_DISTRO}-gazebo-dev ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-gazebo-plugins ros-${ROS_DISTRO}-gazebo-ros ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-joint-state-publisher python3-tk ros-${ROS_DISTRO}-plotjuggler-ros
```


## Compiling

```shell
$ cd ~/uh-fs-ai
$ colcon build --symlink-install --parallel-workers $(nproc)
$ source install/setup.bash
```

## Running

### Source the workspace
```shell
source ~/uh-fs-ai/install/setup.bash
```

### Launch the simulator

```shell
ros2 launch eufs_launcher eufs_launcher.launch.py

ros2 launch launch/simulation.launch.py (Updates Pending)
```

### Launch planning and control (launch file)

```shell
source install/setup.bash
ros2 launch launch/plan_con.launch.py
```

### Launch EKF-SLAM (launch file)

```shell
source install/setup.bash
ros2 launch launch/launch.py
```

### Run nodes directly (no launch file)

**Planning node**
```shell
source install/setup.bash
ros2 run planning planning_node
```

**Control node**
```shell
source install/setup.bash
ros2 run control control_node
```

**EKF-SLAM**
```shell
source install/setup.bash
ros2 run localization ekf_slam
```

**SLAM evaluator (real-time CSV + plots)**
```shell
source install/setup.bash
ros2 run localization slam_evaluator --ros-args \
	-p slam_odom_topic:=/ekf_slam/odom \
	-p gt_odom_topic:=/ground_truth/odom \
	-p enable_realtime_csv:=true
```

**PID evaluator (CSV, optional plots)**
```shell
source install/setup.bash
ros2 run control pid_eval --ros-args \
	-p enable_pid_eval_csv:=true \
	-p pid_eval_csv_path:=~/uh-fs-ai/slam_eval/pid_eval.csv
```

### Common run order (recommended)
1) Simulator: `ros2 launch eufs_launcher eufs_launcher.launch.py`
2) EKF-SLAM: `ros2 run localization ekf_slam`
3) Planning: `ros2 run planning planning_node`
4) Control: `ros2 run control control_node`

### Notes
- Always re-run `source install/setup.bash` in any new terminal.
- If you change Python nodes, rebuild: `colcon build --packages-select <pkg>` and re-source.

## Docker (optional)

### Install Docker (Ubuntu 20.04)
```shell
sudo apt update
sudo apt install docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
```
Log out and log back in so your user can run Docker without sudo.

### Build the image
From the repo root (same folder as the Dockerfile):
```shell
docker build -t slam_sim .
```

### Run the image
```shell
docker run -it --rm slam_sim
```
Inside the container, you can run the same `ros2` commands listed above.

### Run the simulator with Docker (GUI)
On the host (outside the container):
```shell
sudo apt install x11-xserver-utils
xhost +local:root
```

Then start the container with X11 support:
```shell
docker run -it --rm \
	--env DISPLAY=$DISPLAY \
	--env QT_X11_NO_MITSHM=1 \
	--env LIBGL_ALWAYS_SOFTWARE=1 \
	--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
	slam_sim
```

Inside the container:
```shell
source /opt/ros/galactic/setup.bash
source /workspace/install/setup.bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```
