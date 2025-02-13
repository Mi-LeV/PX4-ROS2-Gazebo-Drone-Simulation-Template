import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rclpy
from .gzcam import GzCam

class PubCam(Node):
    """Node for controlling a vehicle in offboard mode."""
    
    def __init__(self) -> None:
        super().__init__('pub_cam')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.cam_publisher = self.create_publisher(
            Image, '/camera/image_raw', qos_profile)
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, '/camera/camera_info', qos_profile)

        # Timer to call publish_image function at a regular interval
        self.timer = self.create_timer(0.1, self.publish_image)

        # Initialize camera object
        self.cam = None

        # Define camera parameters (these can be updated based on actual camera data)
        self.camera_matrix = np.array([
            [640.0, 0.0, 320.0],  # fx, 0, cx
            [0.0, 480.0, 240.0],  # 0, fy, cy
            [0.0, 0.0, 1.0]       # 0, 0, 1
        ])  # Example intrinsic matrix
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Example distortion coefficients

    def publish_image(self) -> None:
        """Publish the latest camera image and camera info."""
        if self.cam is None:
            return

        # Fetch next image from GzCam (assuming get_next_image is implemented)
        img = self.cam.get_next_image()

        if img is None:
            self.get_logger().warn('No image data received.')
            return

        # Convert the image data (assuming img is a numpy array)
        img_data = img.flatten().tolist()  # Flatten numpy array to a list of bytes

        # Prepare Image message (sensor_msgs)
        msg_image = Image()
        msg_image.data = img_data  # Assign the flattened image data
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.height = img.shape[0]  # Set the height of the image
        msg_image.width = img.shape[1]   # Set the width of the image
        msg_image.encoding = 'rgb8'      # Set encoding to match the format (this is just an example, adjust as necessary)
        msg_image.step = img.shape[1] * 3  # Assuming 3 bytes per pixel for 'rgb8'

        # Publish the image
        self.cam_publisher.publish(msg_image)

        # Prepare CameraInfo message (sensor_msgs)
        msg_camera_info = CameraInfo()
        msg_camera_info.header.stamp = self.get_clock().now().to_msg()
        msg_camera_info.height = img.shape[0]
        msg_camera_info.width = img.shape[1]
        
        # Assign intrinsic matrix (3x3)
        msg_camera_info.k = self.camera_matrix.flatten().tolist()
        
        # Assign distortion coefficients (k1, k2, p1, p2, k3)
        msg_camera_info.d = self.dist_coeffs.tolist()

        # Assign other camera info (optional but good practice)
        msg_camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Identity matrix for rotation
        msg_camera_info.p = self.camera_matrix.flatten().tolist()  # Projection matrix (same as camera matrix for simplicity)

        # Publish CameraInfo message
        self.camera_info_publisher.publish(msg_camera_info)

def main(args=None) -> None:
    rclpy.init(args=args)

    # Initialize camera object here before node creation
    cam = GzCam("/camera/image_raw", (640, 480))

    # Create the node instance
    pub_cam = PubCam()
    pub_cam.cam = cam
    
    rclpy.spin(pub_cam)

    pub_cam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
