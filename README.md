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

# Step 2: Localization
When running in the warehouse, you MUST run the following script first in each shell
```
cd
source ./implement_fastrtps.sh
```
- Run gazebo if needed (see step 1)
- Run the bridge (see step 1)

Edit the launch file as needed to use the gazebo or warehouse config directories, then run in the `local` shell
```
cd ros2_ws/
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch localization_server localization.launch.py
```
If localization is working correctly, and no poses have yet been sent to `/initialpose`, you should see the message in the `local` shell
```
[amcl-2] [WARN] [1661045843.779845623] [amcl]: ACML cannot publish a pose or update the transform. Please set the initial pose...
```
if you don't see that, try restarting the `bridge` and `rviz2`

Run rviz2
If creating a new rviz2 config file, make especially sure to match the QOS of the map plugin to the QOS setting of the map server
Note that amcl won't publish the map frame until given a 2D pose estimate
```
user:~$ ros2 topic info -v /map  
Type: nav_msgs/msg/OccupancyGrid  
  
Publisher count: 1  
  
Node name: map_server  
Node namespace: /  
Topic type: nav_msgs/msg/OccupancyGrid  
Endpoint type: PUBLISHER  
GID: 01.0f.23.d4.05.34.00.00.01.00.00.00.00.00.20.03.00.00.00.00.00.00.00.00  
QoS profile:  
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE  
  Durability: RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
  Lifespan: 2147483651294967295 nanoseconds  
  Deadline: 2147483651294967295 nanoseconds  
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC  
  Liveliness lease duration: 2147483651294967295 nanoseconds
```
This also goes for visualizing `/particlecloud` using a PoseArray
```
user:~$ ros2 topic info -v /particlecloud
Type: geometry_msgs/msg/PoseArray

Publisher count: 1

Node name: amcl
Node namespace: /
Topic type: geometry_msgs/msg/PoseArray
Endpoint type: PUBLISHER
GID: 01.0f.23.d4.56.2c.00.00.01.00.00.00.00.00.33.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds
```
open `rviz2` in its own shell, using the config file `localization_server/rviz/localization_server.rviz`

provide an initial estimate in rviz2 using the 2D Pose Estimate, then move around the around until the robot is localized

# Step 3 Path planning
Run everything from step 2, but use `rviz2` with the config file `path_planner_server/rviz/pathplanner.rviz`
in the `plan` shell
```
cd ros2_ws/
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch path_planner_server pathplanner.launch.py
```
When run on real robot, crashes rviz
```
61159914.957 for reason 'Unknown'
terminate called after throwing an instance of 'tf2::ConnectivityException'
  what():  Could not find a connection between 'map' and 'robot_front_laser_link' because they are not part of the same tree.Tf has two or more unconnected trees.
Aborted (core dumped)
```
# # Step 4   Navigate programmatically
Run step 3 then in `nav prog` shell run
```
cd ros2_ws/
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch path_planner_server nav_to_pose_action_client.launch.py
```
