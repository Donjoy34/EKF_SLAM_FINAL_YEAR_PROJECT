This Repository Maintains the FS-AI Developments for 2023 compatition at Silverstone.

## Prerequisites

 - Install Ubuntu 20.04 LTS
 - Install [ros-galactic-desktop](http://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

**Install colcon**

```shell
$ sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

$ sudo apt update
$ sudo apt install python3-colcon-common-extensions
```

## clone the repo

Setup ssh-key following the instructions [here](https://gitlab.com/uhra/uh-fs-ai/-/wikis/Git-Commands) to enable you to authenticates to the GitLab server without using username and password each time.

```shell
git clone --recurse-submodules -j8 git@gitlab.com:uhra/uh-fs-ai.git
```

**Setup EUFS_MASTER Variable** 

execute `pwd` command from your `uh-fs-ai` directory and replace `/path/to/the/directory` in the below command with output of `pwd` command to set the path of this directory as the EUFS_MASTER environment variable 

`Example : echo 'export EUFS_MASTER=/home/nihad/uh-fs-ai' >> ~/.bashrc`


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
Few additional dependencies : 

```shell
sudo apt install ros-${ROS_DISTRO}-gazebo-dev ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-gazebo-plugins ros-${ROS_DISTRO}-gazebo-ros ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-joint-state-publisher python3-tk ros-${ROS_DISTRO}-plotjuggler-ros
```


## Compiling

```shell
$ cd ~/uh-fs-ai
$ colcon build --symlink-install --parallel-workers $(nproc)
$ . install/setup.bash
```

## Running

To launch the simulator 

```shell
ros2 launch eufs_launcher eufs_launcher.launch.py

ros2 launch launch/simulation.launch.py (Updates Pending)
```

To launch the planning & control nodes (Run in New Terminal)

```shell
. install/setup.bash
ros2 launch launch/plan_con.launch.py
```

To launch the sensor_fusion node (Run in New Terminal)

```shell
. install/setup.bash
ros2 launch launch/launch.py
```
