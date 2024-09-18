import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('eimu_ros2'))
    eimu_ros2_config_file = os.path.join(pkg_path,'config','eimu_ros2_start_params.yaml')

    eimu_ros2_node = Node(
        package='eimu_ros2',
        executable='eimu_ros2',
        name='eimu_ros2',
        output='screen',
        parameters=[eimu_ros2_config_file],
    )

     # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(eimu_ros2_node)
    
    return ld      # return (i.e send) the launch description for excecution
    
