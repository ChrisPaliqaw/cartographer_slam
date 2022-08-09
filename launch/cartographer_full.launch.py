import os.path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cartographer_slam_launch_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'launch')
    cartographer_launch_filename = "/cartographer.launch.py"
    rviz2_launch_filename = "/rviz2.launch.py"
    cartographer_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([cartographer_slam_launch_dir, cartographer_launch_filename])
      )
    rviz2_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([cartographer_slam_launch_dir, rviz2_launch_filename])
      )
    return LaunchDescription([
        cartographer_launch,
        rviz2_launch
    ])