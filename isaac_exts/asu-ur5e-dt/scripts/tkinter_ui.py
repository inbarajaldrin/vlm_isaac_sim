import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageTk
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

# Camera topics to subscribe
CAMERA_TOPICS = [
    "/exocentric_camera",
    "/camera/color/image_raw",
    "/intel_camera_rgb",
    "/intel_camera_rgb_raw",
    "/custom_camera"
]

# Prompts
PROMPTS = {
    "Setup Scene": """list available topics and read jenga pose.
can you import  jenga blocks in isaam sim
# -0.25,-0.5,0 and -0.5,-0.5,0
jenga blocks at omniverse://localhost/Library/Aruco/objs/jenga.usd
""",

    "Reset Scene": """list available topics and read jenga pose. Do not load scene
list prims in isaac sim and find if the jenga blocks are already imported. if they are then just update the pose of the jenga block using the following logic.
if the jenga block doesnt exist in topics and it exists in isaac sim prim list then delete that prim.
If there are more jenga blocks in topics and some are missing in the scene, then add new jenga block in the right orientation.
place the blocks in the scene in isaac sim

jenga blocks at omniverse://localhost/Library/Aruco/objs/jenga.usd

""",

    "Pick & Place": """

    ## move to a closer view to get updated pose of the real wold jenga block and import it to isaac sim. from there dont read pose from topic, read pose from DT. Need a better scene counter.
Step0: List availbale topics/prims and pick one jenga block out of available jenga blocks (not the one already moved to new position)

Step1: read jenga block pose : x,y,z , rx,ry,rz 

Step2: move UR ee to : x,y,(z=0.25) with 0,180,0

Step3: read jenga block pose again #and update isaac sim scene# and move ee orientation to pick jenga block = 0,180,(UR rz) where UR rz= (90-Jenga rz)x-1

Step4: move UR ee to : x,y,(z=0.15) with 0,180,(UR rz)

Step5: Close gripper 

Step6: move UR ee to : x,y,(z=0.25) # and confirm grasp visually (access intel_camera_rgb_raw camera view)

Step7: set ee orientation with 0,180,(desired final rotation)
required_rotation = desired_final_rotation - current_jenga_rz new_ee_rz = current_ee_rz + required_rotation

Step8: move UR ee to : final pose x,y,0.25 to drop

Step9: move the ee z to  0.151

Step10: open gripper

Step11: go home: HOME_POSE = [0.065, -0.385, 0.481, 0, 180, 0] 

-0.25,-0.5
-0.25,-0.41
-0.25,-0.32

repeat till no jenga blocks remain in their initial pose.

""",
    "Stack": """
can you peform the whole pick and place and stack on top of jenga 2
use z = 0.162 m in step 9 to place the jenga block down on top of the other

perform step by step 
ive refreshed the scene

pick and place prompt incase you forgot

Step0: List availbale topics/prims and pick one jenga block out of available jenga blocks (not the one already moved to new position)  
Step1: read jenga block pose : x,y,z , rx,ry,rz   
Step2: move UR ee to : x,y,(z=0.25) with 0,180,0  
Step3: read jenga block pose again #and update isaac sim scene# and move ee orientation to pick jenga block = 0,180,(UR rz) where UR rz= (90-Jenga rz)x-1  
Step4: move UR ee to : x,y,(z=0.15) with 0,180,(UR rz)  
Step5: Close gripper   
Step6: move UR ee to : x,y,(z=0.25) # and confirm grasp visually (access intel_camera_rgb_raw camera view)  
Step7: set ee orientation with 0,180,(desired final rotation) required_rotation = desired_final_rotation - current_jenga_rz new_ee_rz = current_ee_rz + required_rotation  
Step8: move UR ee to : final pose x,y,0.25 to drop  
Step9: move the ee z to  0.151
Step10: open gripper  Step11: go home: HOME_POSE = [0.065, -0.385, 0.481, 0, 180, 0]

    """,

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
        
        # Recording variables
        self.recording = False
        self.video_writers = {}
        self.recording_folder = None
        self.frame_count = 0

        rclpy.init()
        self.node = CameraNode()

        # Create main container with horizontal layout
        main_container = ttk.Frame(root)
        main_container.pack(fill=tk.BOTH, expand=True)

        # Left side - existing camera feeds and prompts
        left_frame = ttk.Frame(main_container)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Camera feeds
        notebook = ttk.Notebook(left_frame)
        notebook.pack(fill=tk.BOTH, expand=True)

        self.labels = {}
        for topic in CAMERA_TOPICS:
            frame = ttk.Frame(notebook)
            notebook.add(frame, text=topic.split('/')[-1])
            label = tk.Label(frame)
            label.pack()
            self.labels[topic] = label

        # Prompt tabs
        prompt_tabs = ttk.Notebook(left_frame)
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

        # Right side - Recording panel
        self.create_recording_panel(main_container)

        self.running = True
        threading.Thread(target=self.spin_ros, daemon=True).start()
        self.update_images()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def create_recording_panel(self, parent):
        # Right side frame for recording
        right_frame = ttk.Frame(parent, width=300)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        right_frame.pack_propagate(False)

        # Recording panel title
        title_label = ttk.Label(right_frame, text="Record", font=("Arial", 14, "bold"))
        title_label.pack(pady=(10, 20))

        # Folder name section
        folder_name_frame = ttk.Frame(right_frame)
        folder_name_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(folder_name_frame, text="Folder Name:").pack(anchor="w")
        self.folder_name_entry = ttk.Entry(folder_name_frame)
        self.folder_name_entry.pack(fill=tk.X, pady=5)

        # Filename section
        filename_frame = ttk.Frame(right_frame)
        filename_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(filename_frame, text="Video/Photo Name:").pack(anchor="w")
        self.filename_entry = ttk.Entry(filename_frame)
        self.filename_entry.pack(fill=tk.X, pady=5)

        # Recording status
        self.status_label = ttk.Label(right_frame, text="Status: Ready", foreground="green")
        self.status_label.pack(pady=10)

        # Folder selection
        folder_frame = ttk.Frame(right_frame)
        folder_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(folder_frame, text="Select Recording Folder", 
                  command=self.select_recording_folder).pack(fill=tk.X)
        
        self.folder_label = ttk.Label(folder_frame, text="No folder selected", 
                                     foreground="red", wraplength=280)
        self.folder_label.pack(pady=5)

        # Recording buttons
        button_frame = ttk.Frame(right_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=20)

        self.record_button = ttk.Button(button_frame, text="Start Recording", 
                                       command=self.start_recording)
        self.record_button.pack(fill=tk.X, pady=2)

        self.stop_button = ttk.Button(button_frame, text="Stop Recording", 
                                     command=self.stop_recording, state="disabled")
        self.stop_button.pack(fill=tk.X, pady=2)

        self.photo_button = ttk.Button(button_frame, text="Take Photo", 
                                      command=self.take_photo)
        self.photo_button.pack(fill=tk.X, pady=2)

        # Recording info
        info_frame = ttk.Frame(right_frame)
        info_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.recording_info = ttk.Label(info_frame, text="", wraplength=280)
        self.recording_info.pack()

    def select_recording_folder(self):
        folder = filedialog.askdirectory(title="Select Recording Folder")
        if folder:
            self.recording_folder = folder
            self.folder_label.config(text=f"Folder: {os.path.basename(folder)}", 
                                   foreground="green")
        else:
            self.recording_folder = None
            self.folder_label.config(text="No folder selected", foreground="red")

    def toggle_recording(self):
        # This method is no longer used, keeping for compatibility
        pass

    def start_recording(self):
        if not self.recording_folder:
            messagebox.showerror("Error", "Please select a recording folder first!")
            return

        if self.recording:
            messagebox.showwarning("Warning", "Recording is already in progress!")
            return

        try:
            # Get folder name from entry field
            folder_name = self.folder_name_entry.get().strip()
            if not folder_name:
                messagebox.showerror("Error", "Please enter a folder name!")
                return
            
            # Get video filename
            video_filename = self.filename_entry.get().strip()
            if not video_filename:
                messagebox.showerror("Error", "Please enter a video/photo name!")
                return
            
            # Create custom folder in the selected recording directory
            session_folder = os.path.join(self.recording_folder, folder_name)
            os.makedirs(session_folder, exist_ok=True)

            # Initialize video writers for each camera topic
            self.video_writers = {}
            for topic in CAMERA_TOPICS:
                topic_name = topic.replace("/", "_").strip("_")
                video_path = os.path.join(session_folder, f"{video_filename}_{topic_name}.avi")
                
                # Get current image to determine dimensions
                img = self.node.images[topic]
                h, w, _ = img.shape
                
                # Create video writer
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_writers[topic] = cv2.VideoWriter(video_path, fourcc, 10.0, (w, h))

            self.recording = True
            self.frame_count = 0
            self.current_session_folder = session_folder
            self.current_video_name = video_filename
            
            # Update UI
            self.record_button.config(state="disabled")
            self.stop_button.config(state="normal")
            self.status_label.config(text="Status: Recording", foreground="red")
            self.recording_info.config(text=f"Recording: {video_filename}")
            
            print(f"Started recording '{video_filename}' to: {session_folder}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start recording: {str(e)}")
            print(f"Recording error: {e}")

    def stop_recording(self):
        if not self.recording:
            messagebox.showwarning("Warning", "No recording in progress!")
            return

        try:
            self.recording = False
            
            # Close all video writers
            for writer in self.video_writers.values():
                if writer:
                    writer.release()
            self.video_writers = {}
            
            # Update UI
            self.record_button.config(state="normal")
            self.stop_button.config(state="disabled")
            self.status_label.config(text="Status: Ready", foreground="green")
            self.recording_info.config(text=f"Recorded {self.frame_count} frames")
            
            print(f"Recording stopped. Total frames: {self.frame_count}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to stop recording: {str(e)}")
            print(f"Stop recording error: {e}")

    def take_photo(self):
        if not self.recording_folder:
            messagebox.showerror("Error", "Please select a recording folder first!")
            return
            
        try:
            # Get folder name from entry field
            folder_name = self.folder_name_entry.get().strip()
            if not folder_name:
                messagebox.showerror("Error", "Please enter a folder name!")
                return
            
            # Get photo filename
            photo_filename = self.filename_entry.get().strip()
            if not photo_filename:
                messagebox.showerror("Error", "Please enter a video/photo name!")
                return
            
            # Create custom folder in the selected recording directory
            photos_folder = os.path.join(self.recording_folder, folder_name)
            os.makedirs(photos_folder, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
            
            # Save photo from each camera topic
            for topic in CAMERA_TOPICS:
                img = self.node.images[topic]
                topic_name = topic.replace("/", "_").strip("_")
                photo_path = os.path.join(photos_folder, f"{photo_filename}_{topic_name}_{timestamp}.jpg")
                
                # Convert RGB to BGR for OpenCV
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                cv2.imwrite(photo_path, img_bgr)
            
            self.recording_info.config(text=f"Photo saved: {photo_filename}")
            print(f"Photos saved as '{photo_filename}' with timestamp: {timestamp} in folder: {folder_name}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to take photo: {str(e)}")
            print(f"Photo error: {e}")

    def spin_ros(self):
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"Spin error: {e}")

    def update_images(self):
        for topic in CAMERA_TOPICS:
            img = self.node.images[topic]

            # Record frame if recording is active
            if self.recording and topic in self.video_writers:
                try:
                    # Convert RGB to BGR for OpenCV video writer
                    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    self.video_writers[topic].write(img_bgr)
                    if topic == CAMERA_TOPICS[0]:  # Only count frames once
                        self.frame_count += 1
                except Exception as e:
                    print(f"Error writing frame for {topic}: {e}")

            # Display image
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
        if self.recording:
            self.stop_recording()
        
        self.running = False
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except:
            pass
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1500x900")  # Made wider to accommodate the new panel
    app = ROS2App(root)
    root.mainloop()