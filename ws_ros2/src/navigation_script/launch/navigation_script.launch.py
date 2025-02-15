from ament_index_python.packages import get_package_share_directory

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    
    main_script = Node(
        package='navigation_script',
        executable='navigation_node',
    )

    camera_node = Node(
        package='image_view',
        executable='image_view',
        name='camera_viewer',
        remappings=[("image", "/camera/image_raw")],
        parameters=[{'autosize': True}]
    )


    return LaunchDescription([
        camera_node,
        main_script
    ])
