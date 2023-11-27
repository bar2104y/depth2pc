# DepthImage to PointCloud

## Install
```
cd ~/
mkdir ros2_ws/src # if not exist
cd ros2_ws/src
git clone https://github.com/bar2104y/depth2pc.git
colcon build --packages-select depth_to_pc
. install/setup.bash
```

add to `~/.bashrc` line `~/ros2_ws/install/setup.bash`

My `~/.bashrc` looks like:
```
...
<other lines>
...

source /opt/ros/humble/setup.bash
. ~/ros2_ws/install/setup.bash
```

## Run
```
ros2 run depth_to_pc realsense
```