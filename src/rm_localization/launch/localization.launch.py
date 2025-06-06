import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rm_localization'), 'launch', 'ekf.launch.py')]))
    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rm_localization'), 'launch', 'amcl.launch.py')]))
    

    
    
    return LaunchDescription([
        ekf,
        amcl
    ])