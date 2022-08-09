import os.path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz')
    rviz_config_file = 'cartographer_slam.rviz'
    return LaunchDescription([
        Node(package='rviz2',
             executable='rviz2',
             arguments=['-d', os.path.join(rviz_dir, rviz_config_file)])
    ])