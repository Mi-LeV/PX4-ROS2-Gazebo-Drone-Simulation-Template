#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # Ensure this is imported

class NavigationNode(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('navigation_node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_cb, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_cb, qos_profile)
        
        self.aruco_markers_subscriber = self.create_subscription(
            ArucoMarkers, '/aruco_markers', self.aruco_markers_cb, qos_profile)

        self.line_pose_subscriber = self.create_subscription(
            Pose2D, '/line_pose', self.line_pose_cb, qos_profile)
        
        
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.aruco_markers = ArucoMarkers()
        self.line_pose = Pose2D() 

        self.command_delay_counter = 0
        self.takeoff_height = -1
        self.nav_state = "idle" #idle, takeoff,task1,task2,task3...,land
        

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_cb)

    def vehicle_local_position_cb(self, vehicle_local_position):
        """cb function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_cb(self, vehicle_status):
        """cb function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def aruco_markers_cb(self, aruco_markers):
        self.aruco_markers = aruco_markers
    
    def line_pose_cb(self, line_pose):
        self.line_pose = line_pose
    

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.  # (0 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
    
    def publish_velocity_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.velocity = [x, y, z]
        msg.position = [float('nan'),float('nan'),float('nan')]
        msg.yaw = 0.  # (0 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    

    def move_drone_line(self, offset_x, offset_y, line_heading, speed=0.001):
        """Generate a movement command based on offsets and line heading."""

        if line_heading is not None: # if the direction of the line is found
            far_factor = min(max(abs(offset_x), abs(offset_y)), 200) / 200 # normalise the offset between 0 and 1

            # the factor is the weight between centering the drone on the line and going down the line
            vel_x = (np.cos(line_heading )) *100* speed * (1 - far_factor)\
            + (-offset_y * speed) * far_factor 
            vel_y = (np.sin(line_heading))  *100* speed * (1 - far_factor)\
            + (-offset_x * speed) * far_factor
        else:
            vel_x = -offset_y * speed
            vel_y = -offset_x * speed
        vel_z = 0.0
        return vel_x,vel_y,vel_z

    def timer_cb(self) -> None:
        """cb function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.nav_state == "idle":
            if self.command_delay_counter == 10:
                self.engage_offboard_mode()
                self.arm()

            if self.vehicle_local_position.z > self.takeoff_height \
                and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.nav_state = "takeoff"

        elif self.nav_state == "takeoff":
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height * 3) # times 3 to T/O faster

            if self.vehicle_local_position.z <= self.takeoff_height:
                self.nav_state = "task1"

        if self.nav_state == "task1":
            self.nav_state = "land"

        if self.nav_state == "land":
            if self.command_delay_counter == 10:
                self.land()
                
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                exit(0)
        
        if self.command_delay_counter > 10:
            self.command_delay_counter = 0
        else:
            self.command_delay_counter += 1

        for i, pose in enumerate(self.aruco_markers.poses): # print arucos
            self.get_logger().info(f"A{i} [{self.aruco_markers.marker_ids[i]}] {pose.position.x:.2f} {pose.position.y:.2f} {pose.position.z:.2f} | {pose.orientation.x:.2f} {pose.orientation.y:.2f} {pose.orientation.z:.2f} ")

        self.get_logger().info(f"L [{self.line_pose.x:.2f} {self.line_pose.y:.2f} {self.line_pose.theta:.2f}]")

        self.get_logger().info(f"S {self.nav_state} POS [{self.vehicle_local_position.x:.2f}, {self.vehicle_local_position.y:.2f}, {self.vehicle_local_position.z:.2f}]")




def main(args=None) -> None:
    

    print('Starting offboard control node...')
    rclpy.init(args=args)

    navigation_node = NavigationNode()
    
    rclpy.spin(navigation_node)

    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)