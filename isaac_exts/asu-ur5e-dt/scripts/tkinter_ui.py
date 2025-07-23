import tkinter as tk
from tkinter import ttk, messagebox
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
    "/exocentric_camera",
    "/camera/color/image_raw",
    "/intel_camera_rgb",
    "/intel_camera_depth",
    "/custom_camera"
]

# Prompts
PROMPTS = {
    "Setup Scene": """list available topics and read jenga pose.
set pz values =0 and place the blocks in the scene in isaac sim...""",
    "Pick & Place": """Step0: pick one jenga block out of available jenga blocks...""",
    "Stack": "Stack the red cube on top of the blue cylinder.",
    "Push": "Push the green sphere 10 cm forward."
}

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
        self.root.title("Isaac Sim Control Panel with ROS2")

        # Initialize ROS 2
        rclpy.init()
        self.node = CameraNode()

        # Tabs for camera views
        notebook = ttk.Notebook(root)
        notebook.pack(fill=tk.BOTH, expand=True)

        self.labels = {}
        for topic in CAMERA_TOPICS:
            frame = ttk.Frame(notebook)
            notebook.add(frame, text=topic.split('/')[-1])
            label = tk.Label(frame)
            label.pack()
            self.labels[topic] = label

        # Prompt Buttons
        prompt_frame = ttk.LabelFrame(root, text="Copy Prompts", padding=10)
        prompt_frame.pack(fill=tk.X, padx=5, pady=5)
        for key in PROMPTS:
            btn = ttk.Button(prompt_frame, text=key, command=lambda k=key: self.copy_prompt(k))
            btn.pack(side=tk.LEFT, padx=5)

        # ROS2 spin in background
        self.running = True
        threading.Thread(target=self.spin_ros, daemon=True).start()
        self.update_images()

        # Handle close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def spin_ros(self):
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"Spin error: {e}")

    def update_images(self):
        for topic in CAMERA_TOPICS:
            img = self.node.images[topic]

            # Optionally resize only if the image is too large for the screen
            max_width = 960  # limit width
            h, w, _ = img.shape

            if w > max_width:
                scale = max_width / w
                new_w = int(w * scale)
                new_h = int(h * scale)
                img_resized = cv2.resize(img, (new_w, new_h))
            else:
                img_resized = img

            img_pil = Image.fromarray(img_resized)
            img_tk = ImageTk.PhotoImage(image=img_pil)
            self.labels[topic].imgtk = img_tk
            self.labels[topic].configure(image=img_tk)
        if self.running:
            self.root.after(100, self.update_images)

    def copy_prompt(self, key):
        self.root.clipboard_clear()
        self.root.clipboard_append(PROMPTS[key])
        messagebox.showinfo("Prompt Copied", f"'{key}' prompt copied to clipboard.")

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
    app = ROS2App(root)
    root.mainloop()
