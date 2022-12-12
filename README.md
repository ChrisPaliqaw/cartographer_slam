# cartographer_slam
Use Google's cartographer package with an RB-1 Base

## Preliminaries

In your `gz` shell
```
source ~/simulation_ws/devel/setup.bash
roslaunch rb1_base_gazebo warehouse_rb1.launch
```
`bridge` shell
```
source ~/catkin_ws/devel/setup.bash
roslaunch load_params load_params_base.launch

source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge parameter_bridge
```
test the bridge in the `teleop` window
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
```

## Step 1: Mapping

`build` shell
```
cd ros2_ws/
colcon build --packages-select cartographer_slam
source ~/ros2_ws/install/setup.bash
```

**Remember to use `save as...` in whether you use option 1 or 2 when altering the rviz config file, since the launch files uses copies of the rviz config files from the `install` folder**

### Option 1: full `mapper` launch: cartographer + rviz2
```
cd ros2_ws/
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch cartographer_slam cartographer_full.launch.py
```

### Option 2: separate `cartographer` and `rviz` shells
`cartographer` shell: cartographer and occupancy grid nodes
```
cd ros2_ws/
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch cartographer_slam cartographer.launch.py
```
`rviz` shell
```
cd ros2_ws/
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch cartographer_slam rviz.launch.py
```

### Configure cartographer

Get data needed for config file using the `scan` topic, then edit `cartographer.lua` 
```
ros2 topic echo -f /scan > scan.txt
```

### Map

Use teleop to move all around the warehouse. When ready, save the map to the config folder
```
cd ~/ros2_ws/src/cartographer_slam/config
```
Save the map files
```
ros2 run nav2_map_server map_saver_cli -f neobotix_area_gazebo
```

Rename the resulting map files to make it clear whether they were generated using gazebo or the real warehouse
```
user:~/ros2_ws/src/cartographer_slam/config$ ls
cartographer.lua          neobotix_area_gazebo.yaml
neobotix_area_gazebo.pgm
```

To continue with localization, use the [localization_server](https://github.com/christophomos/localization_server) package
