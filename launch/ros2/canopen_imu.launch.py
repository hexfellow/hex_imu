from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('hex_imu')
    
    config_file = os.path.join(pkg_dir, 'config/ros2/imu.yaml')
    
    canopen_imu_node = Node(
        package='hex_imu',
        executable='CanopenImu',
        name='hex_imu',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/imu_data', '/imu_data'),
            ('/magnetic_data', '/magnetic_data')
        ]
    )
    
    return LaunchDescription([
        canopen_imu_node
    ])
