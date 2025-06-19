import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Parameters
        self.marker_size = self.declare_parameter("marker_size", 0.20).value
        dictionary_id_name = self.declare_parameter("aruco_dictionary_id", "DICT_4X4_50").value
        image_topic = self.declare_parameter("image_topic", "/camera/image_raw").value
        info_topic = self.declare_parameter("camera_info_topic", "/camera/camera_info").value
        self.camera_frame = self.declare_parameter("camera_frame", "").value

        # Validate dictionary ID
        try:
            dictionary_id = getattr(cv2.aruco, dictionary_id_name)
        except AttributeError:
            self.get_logger().error(f"Invalid ArUco dictionary: {dictionary_id_name}")
            return

        # Set up camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # Initialize ArUco detector using the older OpenCV method
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Subscriptions
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)

        # Publisher
        qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", qos_profile)

        self.bridge = CvBridge()

    def info_callback(self, msg):
        self.info_msg = msg
        self.intrinsic_mat = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.distortion = np.array(msg.d, dtype=np.float32)
        self.destroy_subscription(self.info_sub)  # Unsubscribe to avoid unnecessary calls

    def image_callback(self, img_msg):
        if self.info_msg is None:
            return  # Avoid processing without camera parameters

        # Convert image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "mono8")

        # Detect ArUco markers (using the older OpenCV method)
        corners, marker_ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

        if marker_ids is not None:
            markers_msg = ArucoMarkers()
            markers_msg.header.stamp = img_msg.header.stamp
            markers_msg.header.frame_id = self.camera_frame or self.info_msg.header.frame_id

            for i, marker_id in enumerate(marker_ids.flatten()):
                success, rvec, tvec = cv2.solvePnP(
                    self.get_object_points(),
                    corners[i],
                    self.intrinsic_mat,
                    self.distortion,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE  # Faster and more robust
                )

                if not success:
                    continue

                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = tvec.flatten()

                # Convert rotation to quaternion
                rot_matrix, _ = cv2.Rodrigues(rvec)
                quat = R.from_matrix(rot_matrix).as_quat()
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat

                markers_msg.poses.append(pose)
                markers_msg.marker_ids.append(marker_id)

            self.markers_pub.publish(markers_msg)

    def get_object_points(self):
        """Precomputed marker corners in local marker frame."""
        half_size = self.marker_size / 2.0
        return np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
