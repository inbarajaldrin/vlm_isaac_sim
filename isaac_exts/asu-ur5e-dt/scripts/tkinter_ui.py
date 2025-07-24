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
    "/exocentric_camera",
    "/camera/color/image_raw",
    "/intel_camera_rgb",
    "/intel_camera_depth",
    "/custom_camera"
]

# Prompts
PROMPTS = {
    "Setup Scene": """list available topics and read jenga pose.
set pz values =0 and place the blocks in the scene in isaac sim
while following this convention
isaac sim px = -1x px of jenga
same for y axis 

jenga blocks at omniverse://localhost/Library/Aruco/objs/jenga.usd

Rotation- follow [w, x, y, z] order.
""",

    "Reset Scene": """list available topics and read jenga pose.
list prims in isaac sim and find if the jenga blocks are already imported. if they are then just update the pose of the jenga block using the following logic.
if the jenga block doesnt exist in topics and it exists in isaac sim prim list then delete that prim.
If there are more jenga blocks in topics and some are missing in the scene, then add new jenga block in the right orientation.
set pz values =0 and place the blocks in the scene in isaac sim
while following this convention
isaac sim px = -1x px of jenga
same for y axis 

jenga blocks at omniverse://localhost/Library/Aruco/objs/jenga.usd

Rotation- follow [w, x, y, z] order.
""",

    "Pick & Place": """Step0: pick one jenga block out of available jenga blocks (not the one already moved to new position)

Step1: read jenga block pose : x,y,z , rx,ry,rz using get object info from isaac sim

Step2: move UR ee to : -x,-y,z=0.2 with 0,180,0

Step3: move ee orientation to pick jenga block = 0,180,(UR rz) where UR rz= (90-Jenga rz)x-1

Step4: move UR ee to : -x,-y,z=0.15 with 0,180,(UR rz)

Step5: Close gripper 

Step6: move UR ee to : -x,-y,z=0.2 # and confirm grasp visually (access exocentric camera view)

Step7: set ee orientation with 0,180,(desired final rotation)- no changes if nothing specified
required_rotation = desired_final_rotation - current_jenga_rz new_ee_rz = current_ee_rz + required_rotation

Step8: move UR ee to : final pose x,y,0.2 to drop (Final pose - remember x,y,z in isaac sim is -x,-y ee in UR frame. So if i want to place it at 0.3,0.5 then use -0.3,-0.5 to perform ik calculation.)

Step9: move the ee z to  0.151

Step10: open gripper

Step11: go home: HOME_POSE = [0.065, -0.385, 0.481, 0, 180, 0] 

Target Isaac Sim position: [0.3, 0.5] with jenga block position rz 0

repeat till no jenga blocks remain in their initial pose.
""",

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
        self.step_widgets = {}

        rclpy.init()
        self.node = CameraNode()

        notebook = ttk.Notebook(root)
        notebook.pack(fill=tk.BOTH, expand=True)

        self.labels = {}
        for topic in CAMERA_TOPICS:
            frame = ttk.Frame(notebook)
            notebook.add(frame, text=topic.split('/')[-1])
            label = tk.Label(frame)
            label.pack()
            self.labels[topic] = label

        prompt_tabs = ttk.Notebook(self.root)
        prompt_tabs.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        for key, full_text in PROMPTS.items():
            frame = ttk.Frame(prompt_tabs)
            prompt_tabs.add(frame, text=key)
            self.step_widgets[key] = []

            if key == "Pick & Place" and "Step0:" in full_text:
                canvas = tk.Canvas(frame)
                scrollbar = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
                scroll_frame = ttk.Frame(canvas)

                scroll_frame.bind(
                    "<Configure>",
                    lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
                )

                canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
                canvas.configure(yscrollcommand=scrollbar.set)

                canvas.pack(side="left", fill="both", expand=True)
                scrollbar.pack(side="right", fill="y")

                steps = full_text.split("Step")
                steps = [f"Step{step.strip()}" for step in steps if step.strip()]

                for i, step in enumerate(steps):
                    sub_frame = ttk.Frame(scroll_frame)
                    sub_frame.pack(fill=tk.X, padx=5, pady=5)

                    label = ttk.Label(sub_frame, text=f"Step {i}:", font=("Arial", 10, "bold"))
                    label.pack(anchor="w")

                    text_box = tk.Text(sub_frame, wrap=tk.WORD, height=4)
                    text_box.insert(tk.END, step.strip())
                    text_box.config(state=tk.DISABLED)
                    text_box.pack(side=tk.LEFT, fill=tk.X, expand=True)

                    copy_btn = ttk.Button(sub_frame, text="Copy", width=8,
                                          command=lambda tb=text_box: self.copy_step(tb))
                    copy_btn.pack(side=tk.RIGHT, padx=5)

                    self.step_widgets[key].append(text_box)

            else:
                text_box = tk.Text(frame, wrap=tk.WORD, height=15)
                text_box.insert(tk.END, full_text.strip())
                text_box.config(state=tk.DISABLED)
                text_box.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

                copy_btn = ttk.Button(frame, text="Copy Full Prompt", command=lambda t=full_text: self.copy_full_prompt(t))
                copy_btn.pack(pady=5)

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

            max_width = 960
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

    def copy_step(self, text_widget):
        text_widget.config(state=tk.NORMAL)
        text = text_widget.get("1.0", tk.END).strip()
        text_widget.config(state=tk.DISABLED)

        self.root.clipboard_clear()
        self.root.clipboard_append(text)

    def copy_full_prompt(self, full_text):
        self.root.clipboard_clear()
        self.root.clipboard_append(full_text.strip())

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
    root.geometry("1200x900")
    app = ROS2App(root)
    root.mainloop()
