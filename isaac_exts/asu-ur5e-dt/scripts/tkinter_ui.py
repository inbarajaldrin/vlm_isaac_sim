import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2

# Camera topics to subscribe
CAMERA_TOPICS = [
    "/camera/color/image_raw",
    "/intel_camera_rgb_raw"
]

class CameraNode(Node):
    def __init__(self):
        super().__init__('tk_ros2_camera_subscriber')
        self.bridge = CvBridge()
        self.images = {topic: np.zeros((480, 640, 3), dtype=np.uint8) for topic in CAMERA_TOPICS}
        for topic in CAMERA_TOPICS:
            self.create_subscription(
                ROSImage, topic, lambda msg, t=topic: self.callback(msg, t), 10)

    def callback(self, msg, topic):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self.images[topic] = img
        except Exception as e:
            self.get_logger().error(f"Image convert failed: {e}")

class ROS2App:
    def __init__(self, root):
        self.root = root
        self.root.title("Isaac Sim Camera Viewer")

        rclpy.init()
        self.node = CameraNode()

        # Create main container - full width for cameras only
        main_container = ttk.Frame(root)
        main_container.pack(fill=tk.BOTH, expand=True)

        # Camera feeds - vertical layout without tabs
        self.labels = {}
        for i, topic in enumerate(CAMERA_TOPICS):
            # Create frame for each camera
            camera_frame = ttk.Frame(main_container)
            camera_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
            
            # Add label for camera name
            # camera_label = ttk.Label(camera_frame, text=topic, font=("Arial", 12, "bold"))
            # camera_label.pack(pady=(5, 0))
            
            # Add image label
            label = tk.Label(camera_frame)
            label.pack(expand=True)
            self.labels[topic] = label

        self.running = True
        threading.Thread(target=self.spin_ros, daemon=True).start()
        self.update_images()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def spin_ros(self):
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"Spin error: {e}")

    def update_images(self):
        for topic in CAMERA_TOPICS:
            img = self.node.images[topic]

            # Display image with same ratio and width alignment for both cameras
            h, w, _ = img.shape
            
            # Use same dimensions for both cameras
            max_width = 900
            max_height = 480
            
            # Calculate scale to fit within max dimensions while maintaining ratio
            scale_w = max_width / w
            scale_h = max_height / h
            scale = min(scale_w, scale_h)  # Use smaller scale to maintain ratio
            
            new_w = int(w * scale)
            new_h = int(h * scale)
            img_resized = cv2.resize(img, (new_w, new_h))

            img_pil = Image.fromarray(img_resized)
            img_tk = ImageTk.PhotoImage(image=img_pil)
            self.labels[topic].imgtk = img_tk
            self.labels[topic].configure(image=img_tk)
            
        if self.running:
            self.root.after(100, self.update_images)



    def on_close(self):
        self.running = False
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except:
            pass
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("900x900")  # Made wider to accommodate the new panel
    app = ROS2App(root)
    root.mainloop()