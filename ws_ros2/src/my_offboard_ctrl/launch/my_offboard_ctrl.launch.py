import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    pub_cam = Node(
        package='my_offboard_ctrl',
        executable='pub_cam',
    )
    aruco = Node(
        package='my_offboard_ctrl',
        executable='aruco_node',
    )
    main_script = Node(
        package='my_offboard_ctrl',
        executable='offboard_ctrl_example',
    )

    return LaunchDescription([
        pub_cam,
        #aruco,
        main_script
    ])
