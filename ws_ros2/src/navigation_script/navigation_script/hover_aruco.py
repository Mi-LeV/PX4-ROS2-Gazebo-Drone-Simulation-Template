#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import String as String_msg
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

        self.gripper_publisher = self.create_publisher(
            String_msg, '/gripper', qos_profile)

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
        self.takeoff_height = -1.5
        self.nav_state = "IDLE" #idle, takeoff,task1,task2,task3...,land
        

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
    
    def open_gripper(self, command):
        msg = String_msg()
        msg.data = "open" if True else "close"
        self.gripper_publisher.publish(msg)

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
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, vx: float = float('nan'), vy: float= float('nan'), vz: float= float('nan')):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity =[vx, vy, vz]
        msg.yaw = 0.  # (0 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"POS CMD [{msg.position[0]:.2f}, {msg.position[1]:.2f}, {msg.position[2]:.2f}]")
    
    def publish_velocity_setpoint(self,  vx: float = float('nan'), vy: float= float('nan'), vz: float= float('nan')):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, vz]
        msg.position = [float('nan'),float('nan'),float('nan')]
        msg.yaw = 0.  # (0 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"VEL CMD [{vx:.2f}, {vy:.2f}, {vz:.2f}]")

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
    
    def reset_home(self):
        """Reset the home location of the vehicle."""
        command = 179  # MAV_CMD_NAV_SET_HOME command ID
        params = {
            "param1": 1.0,  # Set home to current location (1 = current location, 0 = GPS location)
            "param2": 0.0,  # Latitude (not needed when param1 is set to 1)
            "param3": 0.0,  # Longitude (not needed when param1 is set to 1)
            "param4": 0.0,  # Altitude (not needed when param1 is set to 1)
            "param5": 0.0,  # Desired home altitude
            "param6": 0.0,  # Optional parameter, leave as 0
            "param7": 0.0   # Optional parameter, leave as 0
        }
        self.publish_vehicle_command(command, **params)

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

        if self.nav_state == "IDLE":
            if self.command_delay_counter == 10:
                self.engage_offboard_mode()
            if self.command_delay_counter == 15:
                self.arm()
            #if self.command_delay_counter == 20:
            #    self.reset_home()

            if self.vehicle_local_position.z > self.takeoff_height \
                and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.nav_state = "TAKEOFF"

        elif self.nav_state == "TAKEOFF":
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, vz=-1.0) 

            if self.vehicle_local_position.z <= self.takeoff_height:
                self.nav_state = "HOVER_ARUCO"

        if self.nav_state == "HOVER_ARUCO":

            
            if 0 in self.aruco_markers.marker_ids:
                aruco_i = self.aruco_markers.marker_ids.index(0)
                pose = self.aruco_markers.poses[aruco_i]
                
                command_x = self.vehicle_local_position.x - pose.position.x
                command_y = self.vehicle_local_position.y - pose.position.y
                command_z = self.vehicle_local_position.z - (pose.position.z + self.takeoff_height) 

                self.publish_position_setpoint(command_x, command_y, command_z)


            #self.nav_state = "LAND"

        if self.nav_state == "LAND":
            if self.command_delay_counter == 10:
                self.land()
                
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                exit(0)
        
        if self.command_delay_counter > 20:
            self.command_delay_counter = 0
        else:
            self.command_delay_counter += 1

        for i, pose in enumerate(self.aruco_markers.poses): # print arucos
            self.get_logger().info(f"A{i} [{self.aruco_markers.marker_ids[i]}] {pose.position.x:.2f} {pose.position.y:.2f} {pose.position.z:.2f} | {pose.orientation.x:.2f} {pose.orientation.y:.2f} {pose.orientation.z:.2f} ")

        #self.get_logger().info(f"L [{self.line_pose.x:.2f} {self.line_pose.y:.2f} {self.line_pose.theta:.2f}]")

        self.get_logger().info(f"{self.nav_state} POS [{self.vehicle_local_position.x:.2f}, {self.vehicle_local_position.y:.2f}, {self.vehicle_local_position.z:.2f}]")




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