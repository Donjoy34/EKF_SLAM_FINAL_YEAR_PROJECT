This Repository Maintains the FS-AI Developments for 2023 compatition at Silverstone.


## clone the repo

Setup ssh-key following the instructions [here](https://gitlab.com/uh-fs-ai/uh-fs-ai/-/wikis/Git-Commands) to enable you to authenticates to the GitLab server without using username and password each time.

```shell
git clone --recurse-submodules -j8 git@gitlab.com:uh-fs-ai/uh-fs-ai.git
```

> NOTE: if you're using Galactic & Ubuntu 20.04 please remember to checkout eufs_sim repo to master
> 
> `cd uh-fs-ai/src/eufs_sim`
> 
> `git checkout master`

> NOTE 2: If you are working on any development brach other than main please remember to use the below command line.
>
> `git submodule update --init --recursive`

## Prerequisits
Install colcon

```shell
$ sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

$ sudo apt update
$ sudo apt install python3-colcon-common-extensions
```
 Intel Realsense SDK Installation
 
 ```shell
 chmod 777 bin/Signed_Image_UVC_5_14_0_0.bin
 . bin/Signed_Image_UVC_5_14_0_0.bin
 ```
 
Clone this repository and eufs_msgs v2.0.0 under the same directory. Then, set the path of this directory as the EUFS_MASTER environment variable.

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


## Compiling

```shell
$ cd ~/uh-fs-ai
$ colcon build
$ . install/setup.bash
```

## Running

To launch the simulator 

```shell
ros2 launch launch/simulation.launch.py
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
