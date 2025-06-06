from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    params_file = os.path.join(get_package_share_directory("rm_localization"),
                               'config', 'ekf_config.yaml')

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[params_file, {'use_sim_time':True}]
    )

    return LaunchDescription([
        ekf_filter_node
    ])