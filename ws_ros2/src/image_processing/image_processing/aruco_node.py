import rclpy
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from geometry_msgs.msg import Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import time

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Parameters
        self.marker_size = self.declare_parameter("marker_size", 0.20).value
        dictionary_id_name = self.declare_parameter("aruco_dictionary_id", "DICT_4X4_50").value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.declare_parameter("camera_frame", "").value

        roi_dict = self.get_parameter("roi").get_parameter_value().string_value

        self.expected_roi = RegionOfInterest()
        self.expected_roi.x_offset = roi_dict['x_offset']
        self.expected_roi.y_offset = roi_dict['y_offset']
        self.expected_roi.height = roi_dict['height']
        self.expected_roi.width = roi_dict['width']

        half_size = self.marker_size / 2.0
        self.objects_points = np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # Validate dictionary ID
        try:
            self.dictionary_id = getattr(cv2.aruco, dictionary_id_name)
        except AttributeError:
            self.get_logger().error(f"Invalid ArUco dictionary: {dictionary_id_name}")
            return

        

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested.

        on_configure callback is being called when the lifecycle node
        enters the "configuring" state.

        :return: The state machine either invokes a transition to the "inactive" state or stays
            in "unconfigured" depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "inactive".
            TransitionCallbackReturn.FAILURE transitions to "unconfigured".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """

        self.get_logger().info('Aruco_node CONFIGURING')

        # Set up camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # Initialize ArUco detector using the older OpenCV method
        self.aruco_dictionary = cv2.aruco.Dictionary_get(self.dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Subscriptions
        self.info_sub = self.create_subscription(CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data)

        timeout = 1.0
        start_time = time.time()

        while self.info_msg is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self)

        if info_msg is None:
            self.get_logger().error('Aruco_node CONFIGURING FAILURE : COULDNT GET CAMERA PARAMS')
            return TransitionCallbackReturn.FAILURE
        elif self.roi == self.expected_roi:
            self.get_logger().error('Aruco_node CONFIGURING FAILURE : UNEXECTED IMAGE ROI')
            return TransitionCallbackReturn.FAILURE
        else:
            return TransitionCallbackReturn.SUCCESS


    def on_activate(self, state: State) -> TransitionCallbackReturn:
        
        self.get_logger().info('Aruco_node ACTIVATING')

        self.create_subscription(Image, self.image_topic, self.image_callback, qos_profile_sensor_data)

        # Publisher
        qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", qos_profile)

        self.bridge = CvBridge()

        return super().on_activate(state) #transition to enabled

    def info_callback(self, msg):
        self.info_msg = msg
        self.intrinsic_mat = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.distortion = np.array(msg.d, dtype=np.float32)
        self.roi = RegionOfInterest(msg.roi)

        self.destroy_subscription(self.info_sub)  # Unsubscribe to avoid unnecessary calls

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Aruco_node DEACTIVATING')

        self.destroy_subscription(self.image_topic)  # Unsubscribe from image topic - callback

        return super().on_deactivate(state)

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
                    self.object_points(),
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


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
