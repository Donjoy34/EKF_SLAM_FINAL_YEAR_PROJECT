This Repository Maintains the FS-AI Developments for 2023 compatition at Silverstone.


## clone the repo

Setup ssh-key following the instructions [here](https://gitlab.com/uh-fs-ai/uh-fs-ai/-/wikis/Git-Commands) to enable you to authenticates to the GitLab server without using username and password each time.

```shell
git clone --recurse-submodules -j8 git@gitlab.com:uh-fs-ai/uh-fs-ai.git
```

> NOTE: if you're using Galactic & Ubuntu 20.04 please remember to checkout eufs_sim repo to master
> 
> `cd us-fs-ai/src/eufs_sim`
> 
> `git checkout master`
>

## Compiling

```shell
$ cd [your-workspace]
$ colcon build
$ . install/setup.bash
```

## Running

To launch the simulator 

```shell
ros2 launch launch/simulation.launch.py
```

To launch the planning & control nodes

```shell
ros2 launch launch/plan_con.launch.py
```
