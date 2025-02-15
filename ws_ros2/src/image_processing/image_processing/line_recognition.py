
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R  # Replacement for tf_transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class LineNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("line_node")

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )


        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")




        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(Pose2D, "line_pose", qos_profile)

        self.bridge = CvBridge()


    def image_callback(self, img_msg):


        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        
        pose_2D = Pose2D()
        
        pose_2D.x, pose_2D.y, pose_2D.theta = process_image_line(cv_image)

        self.poses_pub.publish(pose_2D)

    
def get_largest_contour_center(mask):
    """Helper to find the center of the largest contour in a binary mask."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(largest_contour)
        if moments["m00"] > 0:
            return (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))
    return None

def process_image_line(image):
    """Convert the image to HSV and create masks for yellow and green colors."""
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define HSV color ranges for yellow and green
    yellow_lower, yellow_upper = np.array([20, 100, 100]), np.array([30, 255, 255])
    green_lower, green_upper = np.array([35, 100, 100]), np.array([85, 255, 255])

    # Create masks for yellow and green
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    # Apply morphological closing to reduce noise in masks
    kernel = np.ones((5, 5), np.uint8)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

    yellow_center = get_largest_contour_center(yellow_mask)
    green_center = get_largest_contour_center(green_mask)

    image_center = (image.shape[1] // 2, image.shape[0] // 2)
    if yellow_center and green_center:
        # Find midpoint between yellow and green centers
        line_center_x = (yellow_center[0] + green_center[0]) // 2
        line_center_y = (yellow_center[1] + green_center[1]) // 2
        line_heading = np.arctan2(green_center[1] - yellow_center[1], green_center[0] - yellow_center[0])

    elif yellow_center or green_center :
        line_center_x, line_center_y = yellow_center or green_center
        line_heading = float('nan')
    else:
        line_center_x, line_center_y = float('nan'), float('nan')
        line_heading = float('nan')

    # Calculate offset from the image center
    offset_x = float(line_center_x - image_center[0])

    offset_y = float(line_center_y - image_center[1])
    

    
    return offset_x, offset_y, line_heading
        

def main():
    rclpy.init()
    node = LineNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
