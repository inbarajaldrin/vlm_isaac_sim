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

class ArucoJengaBlockDetector(Node):
    def __init__(self):
        super().__init__('aruco_jenga_block_detector')

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Load the camera calibration data
        self.camera_matrix, self.dist_coeffs = self.load_calibration_data('camera_calibration.yaml')

        # ArUco dictionary for detecting ID 0 for origin using 5x5_1000
        self.aruco_dict_origin = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.aruco_params_origin = aruco.DetectorParameters()
        self.aruco_detector_origin = cv2.aruco.ArucoDetector(self.aruco_dict_origin, self.aruco_params_origin)

        # ArUco dictionary for Jenga block markers (4x4_1000 capped at 10)
        self.aruco_dict_jenga = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        self.aruco_params_jenga = aruco.DetectorParameters()
        self.aruco_detector_jenga = cv2.aruco.ArucoDetector(self.aruco_dict_jenga, self.aruco_params_jenga)

        # Load YOLO model for detecting the Jenga block with OBB
        self.model = YOLO('jenga_top_obb.pt')

        # Field of view dimensions in mm (real world size)
        self.fov_x_mm = 325  # Width of field of view in mm
        self.fov_y_mm = 265  # Height of field of view in mm

        # Variable to store the pixel coordinates of the new origin (ID 0)
        self.new_origin_x = None
        self.new_origin_y = None

        # ROS publisher for Jenga detection data
        self.jenga_pub = self.create_publisher(String, '/jenga_detection_data', 10)

        # Flags to control printing, ROS topic, and video streaming
        self.print_to_terminal = True  # Control terminal output
        self.publish_to_rostopic = True  # Control ROS topic publishing
        self.stream_video = True  # Control video streaming

        # Subscribe to the ROS image topic
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
                # Convert the ROS image message to a format OpenCV can use
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imshow("Current Image", cv_image)
                cv2.waitKey(1)
            except CvBridge.CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {str(e)}')
                return

            self.detect_aruco_marker(cv_image)

    def detect_aruco_marker(self, cv_image):
        """ Detect ArUco marker and set the new origin """
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector_origin.detectMarkers(gray)

        if ids is not None and 0 in ids:
            # Find the index of marker ID 0
            idx = np.where(ids == 0)[0][0]
            marker_corners = corners[idx][0]

            # Calculate the centroid of ArUco marker ID 0
            self.new_origin_x = marker_corners[:, 0].mean()
            self.new_origin_y = marker_corners[:, 1].mean()

            # Fix: Convert ids[idx] to a NumPy array before passing it to drawDetectedMarkers
            marker_id_array = np.array([[ids[idx]]], dtype=np.int32)

            # Draw the detected ArUco marker
            cv2.aruco.drawDetectedMarkers(cv_image, [corners[idx]], marker_id_array)

            if self.print_to_terminal:
                print(f"New Origin (ID 0) Pixel Coordinates: X = {self.new_origin_x}, Y = {self.new_origin_y}")

            # Show the ArUco detection result in a window (if streaming is enabled)
            if self.stream_video:
                cv2.imshow('ArUco Marker Detection', cv_image)
                cv2.waitKey(1000)  # Display the window for 1 second
                cv2.destroyAllWindows()

            # Set the flag to indicate ArUco detection is complete
            self.aruco_detected = True

            # Now that the origin is set, switch to Jenga block detection
            self.switch_to_jenga_detection()

        else:
            if self.print_to_terminal:
                print("Aruco marker ID 0 not found.")


    def switch_to_jenga_detection(self):
        """Switch from ArUco detection to Jenga block detection."""
        # Destroy the ArUco marker detection subscription
        self.destroy_subscription(self.aruco_subscription)
        cv2.destroyAllWindows()

        # Now subscribe to the Jenga block detection callback
        self.jenga_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Reuse the same topic for Jenga block detection
            self.jenga_callback,
            10
        )

    def jenga_callback(self, msg):
        """ Process the image to detect the Jenga block and compute its position relative to the new origin. """
        try:
            # Convert the ROS image message to a format OpenCV can use
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridge.CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {str(e)}')
            return

        if self.new_origin_x is None or self.new_origin_y is None:
            if self.print_to_terminal:
                print("Origin not set. Cannot detect Jenga block.")
            return

        # First, detect the ArUco markers for Jenga blocks
        aruco_markers = self.detect_aruco_markers(cv_image)

        # Then, detect and label the Jenga block based on the detected ArUco markers
        self.detect_jenga_block(cv_image, aruco_markers)

    def detect_jenga_block(self, cv_image, aruco_markers):
        """ Detect the Jenga block and compute its position relative to the new origin """
        # Get YOLO model prediction results for detecting the Jenga block with OBB
        results = self.model.predict(cv_image, conf=0.79, verbose=False)

        # Create a list to hold data to publish
        jenga_data_list = []

        if results and results[0].obb is not None:
            obb_boxes = results[0].obb
            for i, obb in enumerate(obb_boxes):
                obb_xyxyxyxy = obb.xyxyxyxy.cpu().numpy()  # Get OBB corner coordinates
                jenga_data = self.draw_oriented_bounding_box(cv_image, obb_xyxyxyxy, aruco_markers)
                if jenga_data:
                    jenga_data_list.append(jenga_data)

            # Publish data to ROS topic every 1 second
            if self.publish_to_rostopic and jenga_data_list:
                msg = String()
                msg.data = "\n".join(jenga_data_list)
                self.jenga_pub.publish(msg)

        # Show the Jenga block detection result in a new window (if streaming is enabled)
        if self.stream_video:
            cv2.imshow('Jenga Block OBB Detection', cv_image)
            cv2.waitKey(1)  # Keep this window open until manually closed

    def draw_oriented_bounding_box(self, cv_image, obb_xyxyxyxy, aruco_markers):
        """ Draw the oriented bounding box on the image, label it, and compute the angle with the x-axis """
        # Convert OBB corners to integer for drawing
        obb_corners = obb_xyxyxyxy.reshape(4, 2).astype(int)

        # Draw the OBB as a polygon with 4 corners
        cv2.polylines(cv_image, [obb_corners], isClosed=True, color=(0, 255, 0), thickness=2)

        # Calculate the centroid of the Jenga block in the pixel frame
        centroid_x = int(obb_corners[:, 0].mean())
        centroid_y = int(obb_corners[:, 1].mean())

        # Find the corresponding ArUco marker within the OBB and label the block
        for marker_id, (aruco_x, aruco_y) in aruco_markers.items():
            if obb_corners[:, 0].min() <= aruco_x <= obb_corners[:, 0].max() and \
               obb_corners[:, 1].min() <= aruco_y <= obb_corners[:, 1].max():
                label = f"ID {marker_id}"  # Use the detected ArUco marker ID as the label

                # Real-world coordinates conversion
                rel_x = centroid_x - self.new_origin_x
                rel_y = centroid_y - self.new_origin_y
                rotated_x = rel_x  # No change in x
                rotated_y = -rel_y  # Flip y-axis for 180-degree rotation
                img_height, img_width = cv_image.shape[:2]
                scale_x = self.fov_x_mm / img_width  # mm per pixel in x direction
                scale_y = self.fov_y_mm / img_height  # mm per pixel in y direction
                real_x_mm = rotated_x * scale_x  # Convert to mm
                real_y_mm = rotated_y * scale_y  # Convert to mm

                # Calculate the angle of the Jenga block with respect to the x-axis (origin)
                dx = obb_corners[2][0] - obb_corners[1][0]
                dy = obb_corners[2][1] - obb_corners[1][1]
                theta = math.atan2(dy, dx)  # Calculate the angle in radians
                theta_degrees = math.degrees(theta)

                # Normalize the angle to be within 0 to 180 degrees
                theta_degrees = theta_degrees % 360
                if theta_degrees > 180:
                    theta_degrees = 360 - theta_degrees
                elif theta_degrees < 0:
                    theta_degrees = -theta_degrees

                # Label the Jenga block with ID, real-world coordinates, and angle (replace ° with "deg")
                label_with_coords = f"{label} ({real_x_mm:.2f}mm, {real_y_mm:.2f}mm, {theta_degrees:.2f} deg)"
                
                # Optionally print to terminal
                # if self.print_to_terminal:
                #     print(f"Labeled {label} with coordinates: X = {real_x_mm:.2f} mm, Y = {real_y_mm:.2f} mm, Angle = {theta_degrees:.2f} deg")

                # Draw the label on the image
                cv2.putText(cv_image, label_with_coords, (centroid_x, centroid_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Return the data for ROS topic publishing
                return label_with_coords

    def detect_aruco_markers(self, cv_image):
        """ Detect ArUco markers in the image and return their centroids and IDs """
        # Convert the image to grayscale for ArUco detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers using 4x4_1000 dictionary
        corners, ids, _ = self.aruco_detector_jenga.detectMarkers(gray)

        # Dictionary to store ArUco marker centroids and IDs
        aruco_markers = {}

        # If markers are detected, process them
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]

                # Cap marker IDs at 10
                if marker_id > 10:
                    if self.print_to_terminal:
                        print(f"Marker ID {marker_id} ignored (greater than 10)")
                    continue

                # Draw the detected ArUco marker
                aruco.drawDetectedMarkers(cv_image, [corners[i]])

                # Get the centroid of the ArUco marker
                c = corners[i][0]
                centroid_x = int(c[:, 0].mean())
                centroid_y = int(c[:, 1].mean())

                if self.print_to_terminal:
                    print(f"Detected ArUco marker ID: {marker_id} with centroid: ({centroid_x}, {centroid_y})")

                # Store the marker ID and its centroid
                aruco_markers[marker_id] = (centroid_x, centroid_y)

        return aruco_markers

def main(args=None):
    rclpy.init(args=args)
    node = ArucoJengaBlockDetector()

    # Keep the node running
    rclpy.spin(node)

    # Cleanup on shutdown
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()