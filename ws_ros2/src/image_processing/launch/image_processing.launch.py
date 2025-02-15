from ament_index_python.packages import get_package_share_directory

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    # Declare a launch argument to specify simulation mode
    use_sim = LaunchConfiguration("use_sim")

    declare_use_sim = DeclareLaunchArgument(
        "use_sim", default_value="true", description="Set to true to use Gazebo camera"
    )
    
    aruco = Node(
        package='image_processing',
        executable='aruco_node',
    )
    line = Node(
        package='image_processing',
        executable='line_node',
    )
    
    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="screen",
        parameters=[{"image_topic": "/image_raw"}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("use_sim"), "' == 'false'"])),
    )

    gazebo_to_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_image_bridge",
        arguments=[f"/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"],
        output="screen",
        condition=IfCondition(use_sim),
    )
    
    
    gazebo_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_info_bridge",
        arguments=[f"/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        output="screen",
        condition=IfCondition(use_sim),
    )



    return LaunchDescription([
        declare_use_sim,
        camera_node,
        gazebo_to_ros_bridge,
        gazebo_camera_info_bridge,
        aruco,
        line
    ])
