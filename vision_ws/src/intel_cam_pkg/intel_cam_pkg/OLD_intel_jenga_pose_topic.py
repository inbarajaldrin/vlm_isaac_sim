# run ros2 launch realsense2_camera rs_launch.py to initialize the camera before running this code

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import time

# 1) Import Pose
from geometry_msgs.msg import Pose

class ArucoJengaBlockDetector(Node):
    def __init__(self):
        super().__init__('aruco_jenga_block_detector')

        # -----------------------
        # User-defined height in millimeters
        # You can change this value to match the actual object height.
        # -----------------------
        self.block_height_mm = 75.0  # Example: 75 mm tall

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Load the camera calibration data
        self.camera_matrix, self.dist_coeffs = self.load_calibration_data('camera_calibration.yaml')

        # ArUco dictionary for detecting ID 0 for origin (5x5_1000)
        self.aruco_dict_origin = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.aruco_params_origin = aruco.DetectorParameters()

        # ArUco dictionary for Jenga block markers (4x4_1000, capped at 10)
        self.aruco_dict_jenga = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        self.aruco_params_jenga = aruco.DetectorParameters()

        # Load YOLO model for detecting the Jenga block with OBB
        self.model = YOLO('jenga_top_obb.pt')

        # Field of view dimensions in mm (real world size)
        self.fov_x_mm = 325  # Width of FOV in mm
        self.fov_y_mm = 265  # Height of FOV in mm

        # Variable to store the pixel coordinates of the new origin (ID 0)
        self.new_origin_x = None
        self.new_origin_y = None

        # Publishers: 1) For a text summary, 2) For Pose
        self.jenga_pub = self.create_publisher(String, '/jenga_detection_data', 10)
        self.pose_pub = self.create_publisher(Pose, '/jenga_detection_pose', 10)

        # Flags to control printing, ROS topic, and video streaming
        self.print_to_terminal = True
        self.publish_to_rostopic = True
        self.stream_video = True

        # Subscribe to the image topic for ArUco detection (origin)
        self.aruco_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.aruco_callback,
            10
        )

        # A flag to indicate whether ArUco detection is complete
        self.aruco_detected = False

    def load_calibration_data(self, filename):
        """ Load camera calibration data from a YAML file """
        with open(filename) as f:
            calibration_data = yaml.safe_load(f)
        camera_matrix = np.array(calibration_data['camera_matrix'], dtype=np.float32)
        dist_coeffs = np.array(calibration_data['dist_coeffs'], dtype=np.float32)
        return camera_matrix, dist_coeffs

    def aruco_callback(self, msg):
        """ Process the image to detect the ArUco marker and set its centroid as the new origin. """
        if not self.aruco_detected:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridge.CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {str(e)}')
                return

            self.detect_aruco_marker(cv_image)

    def detect_aruco_marker(self, cv_image):
        """ Detect ArUco marker (ID 0) and set new origin """
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict_origin, parameters=self.aruco_params_origin)

        if ids is not None and 0 in ids:
            # Find the index of marker ID 0
            idx = np.where(ids == 0)[0][0]
            marker_corners = corners[idx][0]

            # Calculate the centroid of ArUco marker ID 0
            self.new_origin_x = marker_corners[:, 0].mean()
            self.new_origin_y = marker_corners[:, 1].mean()

            # Draw the detected ArUco marker
            aruco.drawDetectedMarkers(cv_image, corners[idx:idx+1], ids[idx:idx+1])

            if self.print_to_terminal:
                print(f"New Origin (ID 0) Pixel Coordinates: X = {self.new_origin_x}, Y = {self.new_origin_y}")

            if self.stream_video:
                cv2.imshow('ArUco Marker Detection', cv_image)
                cv2.waitKey(1000)  # wait for 1 second
                cv2.destroyAllWindows()

            self.aruco_detected = True

            # Switch from ArUco detection to Jenga block detection
            self.switch_to_jenga_detection()
        else:
            if self.print_to_terminal:
                print("Aruco marker ID 0 not found.")

    def switch_to_jenga_detection(self):
        """ Unsubscribe from ArUco detection, subscribe to Jenga detection. """
        self.destroy_subscription(self.aruco_subscription)
        self.jenga_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.jenga_callback,
            10
        )

    def jenga_callback(self, msg):
        """ Process the image to detect Jenga blocks, compute their positions. """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridge.CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {str(e)}')
            return

        if self.new_origin_x is None or self.new_origin_y is None:
            if self.print_to_terminal:
                print("Origin not set. Cannot detect Jenga block.")
            return

        # Detect all ArUco markers for Jenga blocks
        aruco_markers = self.detect_aruco_markers(cv_image)

        # Detect Jenga blocks via YOLO OBB
        self.detect_jenga_block(cv_image, aruco_markers)

    def detect_jenga_block(self, cv_image, aruco_markers):
        """ Detect the Jenga block with YOLO, label & publish data """
        results = self.model.predict(cv_image, conf=0.79, verbose=False)
        jenga_data_list = []

        if results and results[0].obb is not None:
            obb_boxes = results[0].obb
            for i, obb in enumerate(obb_boxes):
                obb_xyxyxyxy = obb.xyxyxyxy.cpu().numpy()
                jenga_data = self.draw_oriented_bounding_box(cv_image, obb_xyxyxyxy, aruco_markers)
                if jenga_data:
                    jenga_data_list.append(jenga_data)

            # Publish text data if any blocks detected
            if self.publish_to_rostopic and jenga_data_list:
                msg = String()
                msg.data = "\n".join(jenga_data_list)
                self.jenga_pub.publish(msg)

        if self.stream_video:
            cv2.imshow('Jenga Block OBB Detection', cv_image)
            cv2.waitKey(1)

    def draw_oriented_bounding_box(self, cv_image, obb_xyxyxyxy, aruco_markers):
        """ Draw the bounding box, label the block, and publish Pose """
        obb_corners = obb_xyxyxyxy.reshape(4, 2).astype(int)
        cv2.polylines(cv_image, [obb_corners], isClosed=True, color=(0, 255, 0), thickness=2)

        # Centroid in pixel space
        centroid_x = int(obb_corners[:, 0].mean())
        centroid_y = int(obb_corners[:, 1].mean())

        # Check each detected ArUco marker to see if it lies within this bounding box
        for marker_id, (aruco_x, aruco_y) in aruco_markers.items():
            if (obb_corners[:, 0].min() <= aruco_x <= obb_corners[:, 0].max() and
                obb_corners[:, 1].min() <= aruco_y <= obb_corners[:, 1].max()):
                
                label = f"ID {marker_id}"

                # Relative offset from origin
                rel_x = centroid_x - self.new_origin_x
                rel_y = centroid_y - self.new_origin_y

                # Flip Y-axis
                rotated_x = rel_x
                rotated_y = -rel_y

                # Convert to mm
                img_height, img_width = cv_image.shape[:2]
                scale_x = self.fov_x_mm / img_width
                scale_y = self.fov_y_mm / img_height
                real_x_mm = rotated_x * scale_x
                real_y_mm = rotated_y * scale_y

                # Calculate the angle w.r.t. x-axis
                dx = obb_corners[2][0] - obb_corners[1][0]
                dy = obb_corners[2][1] - obb_corners[1][1]
                theta = math.atan2(dy, dx)
                theta_degrees = math.degrees(theta)

                # Normalize angle to 0..180
                theta_degrees = theta_degrees % 360
                if theta_degrees > 180:
                    theta_degrees = 360 - theta_degrees
                elif theta_degrees < 0:
                    theta_degrees = -theta_degrees

                # Build label with all data
                label_with_coords = f"{label} ({real_x_mm:.2f}mm, {real_y_mm:.2f}mm, {theta_degrees:.2f} deg)"

                # Draw the label
                cv2.putText(
                    cv_image,
                    label_with_coords,
                    (centroid_x, centroid_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2
                )

                # 3) Build and publish a Pose
                pose_msg = self.make_pose_message(real_x_mm, real_y_mm, theta_degrees)
                self.pose_pub.publish(pose_msg)

                return label_with_coords

    def make_pose_message(self, x_mm, y_mm, angle_deg):
        """
        Builds a geometry_msgs/Pose message from the given position (mm)
        and the orientation angle in degrees around the Z-axis.
        """
        pose = Pose()

        # Convert mm -> meters for the X/Y position
        pose.position.x = x_mm / 1000.0
        pose.position.y = y_mm / 1000.0

        # Use the user-defined height in mm -> meters
        pose.position.z = self.block_height_mm / 1000.0

        # Convert angle (degrees) -> radians
        yaw = math.radians(angle_deg)

        # For a rotation purely about Z, the quaternion is [0, 0, sin(yaw/2), cos(yaw/2)]
        half_yaw = yaw / 2.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(half_yaw)
        pose.orientation.w = math.cos(half_yaw)

        return pose

    def detect_aruco_markers(self, cv_image):
        """ Detect ArUco markers in the image and return centroids """
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict_jenga, parameters=self.aruco_params_jenga)

        aruco_markers = {}
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id > 10:
                    if self.print_to_terminal:
                        print(f"Marker ID {marker_id} ignored (greater than 10)")
                    continue

                aruco.drawDetectedMarkers(cv_image, [corners[i]])
                c = corners[i][0]
                centroid_x = int(c[:, 0].mean())
                centroid_y = int(c[:, 1].mean())

                if self.print_to_terminal:
                    print(f"Detected ArUco marker ID: {marker_id} with centroid: ({centroid_x}, {centroid_y})")

                aruco_markers[marker_id] = (centroid_x, centroid_y)

        return aruco_markers

def main(args=None):
    rclpy.init(args=args)
    node = ArucoJengaBlockDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
