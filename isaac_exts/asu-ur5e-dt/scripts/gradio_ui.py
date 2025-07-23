import asyncio
import gradio as gr
import numpy as np
import cv2
import threading
import time
import queue
import os
import sys

# Fix NumPy compatibility issue
os.environ['NUMPY_ARRAY_API'] = '1'

# Add paths for your existing agent imports
current_dir = os.path.dirname(os.path.abspath(__file__))
isaac_mcp_dir = os.path.join(current_dir, "isaac_mcp")
sys.path.append(isaac_mcp_dir)

print(f"Looking for isaac_mcp files in: {isaac_mcp_dir}")

# ROS2 imports with better error handling
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    
    try:
        from cv_bridge import CvBridge
        ROS2_AVAILABLE = True
        print("ROS2 and cv_bridge loaded successfully")
    except Exception as cv_error:
        print(f"cv_bridge import failed: {cv_error}")
        print("ROS2 camera feeds will use placeholder images.")
        ROS2_AVAILABLE = False
        
except ImportError as e:
    print(f"ROS2 not available: {e}")
    print("Camera feeds will show placeholder images.")
    ROS2_AVAILABLE = False

# Import your existing working code
WORKING_AGENTS_AVAILABLE = False
try:
    from host import MCPHost, check_backend_availability
    WORKING_AGENTS_AVAILABLE = True
    print("Working MCP Host imported successfully")
except ImportError as e:
    print(f"Working MCP Host not available: {e}")
    print(f"Make sure host.py is in {isaac_mcp_dir}")
    WORKING_AGENTS_AVAILABLE = False

# Global agent instances
claude_agent = None
chatgpt_agent = None

# --- ROS2 Camera Node ---
if ROS2_AVAILABLE:
    class CameraNode(Node):
        def __init__(self):
            super().__init__('camera_subscriber')
            self.bridge = CvBridge()
            self.latest_images = {}
            
            # Subscribe to camera topics
            camera_topics = [
                '/custom_camera',
                '/intel_camera_rgb',
                '/intel_camera_depth',
                '/exocentric_camera', 
            ]
            
            # Store subscriptions properly
            self._subs = []
            
            for topic in camera_topics:
                try:
                    sub = self.create_subscription(
                        Image,
                        topic,
                        lambda msg, t=topic: self.image_callback(msg, t),
                        10
                    )
                    self._subs.append(sub)
                    print(f"Subscribed to {topic}")
                except Exception as e:
                    print(f"Failed to subscribe to {topic}: {e}")
        
        def image_callback(self, msg, topic):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                self.latest_images[topic] = cv_image
            except Exception as e:
                self.get_logger().error(f'Error converting image from {topic}: {e}')
else:
    class CameraNode:
        def __init__(self):
            self.latest_images = {}

def initialize_working_agents():
    """Initialize your working agents using existing host.py"""
    global claude_agent, chatgpt_agent
    
    if not WORKING_AGENTS_AVAILABLE:
        print("Working agents not available")
        return
    
    # Check which backends are available using your existing function
    available_backends = check_backend_availability()
    print(f"Available backends: {available_backends}")
    
    # Initialize Claude if available
    if "claude" in available_backends:
        try:
            claude_agent = MCPHost("claude")
            print("Working Claude agent initialized successfully")
        except Exception as e:
            print(f"Failed to initialize Claude agent: {e}")
            claude_agent = None
    else:
        print("Claude not available - check ANTHROPIC_API_KEY")
    
    # Initialize ChatGPT if available  
    if "chatgpt" in available_backends:
        try:
            chatgpt_agent = MCPHost("chatgpt")
            print("Working ChatGPT agent initialized successfully")
        except Exception as e:
            print(f"Failed to initialize ChatGPT agent: {e}")
            chatgpt_agent = None
    else:
        print("ChatGPT not available - check OPENAI_API_KEY")

def run_agent_request(message, history, agent_name):
    """Run agent request with proper async handling"""
    result_queue = queue.Queue()
    
    def run_in_thread():
        try:
            # Create new event loop for this thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                # Convert Gradio history format to your format
                converted_history = []
                for msg in history[-10:]:  # Last 10 messages
                    if isinstance(msg, dict):
                        converted_history.append({
                            "role": msg.get("role", "user"),
                            "content": msg.get("content", ""),
                            "backend": agent_name.lower()
                        })
                
                # Select the appropriate agent
                if agent_name == "Claude" and claude_agent:
                    result = loop.run_until_complete(claude_agent.process_request(message, converted_history))
                elif agent_name == "ChatGPT" and chatgpt_agent:
                    result = loop.run_until_complete(chatgpt_agent.process_request(message, converted_history))
                else:
                    result = f"{agent_name} agent not available. Please check your API keys."
                
                result_queue.put(("success", result))
            finally:
                # Clean up the loop
                try:
                    pending = asyncio.all_tasks(loop)
                    for task in pending:
                        task.cancel()
                    
                    if pending:
                        loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                        
                    loop.close()
                except Exception as cleanup_error:
                    print(f"Loop cleanup error (non-critical): {cleanup_error}")
                    
        except Exception as e:
            result_queue.put(("error", f"Error running {agent_name} agent: {e}"))
    
    # Start thread and wait for result
    thread = threading.Thread(target=run_in_thread, daemon=True)
    thread.start()
    
    try:
        status, result = result_queue.get(timeout=120)  # 2 minute timeout
        return result if status == "success" else result
    except queue.Empty:
        return "Operation timed out after 2 minutes."
    except Exception as e:
        return f"Error in async handling: {e}"

# --- Camera Feed Functions ---
def get_placeholder_image(width=640, height=480, text="No Camera Feed"):
    """Generate placeholder image when camera feed is not available"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img.fill(50)  # Dark gray background
    
    # Add text
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(text, font, 1, 2)[0]
    text_x = (width - text_size[0]) // 2
    text_y = (height + text_size[1]) // 2
    cv2.putText(img, text, (text_x, text_y), font, 1, (255, 255, 255), 2)
    
    return img

def get_camera_feed(topic_name):
    """Get latest image from specified camera topic"""
    if ROS2_AVAILABLE and hasattr(get_camera_feed, 'camera_node') and get_camera_feed.camera_node:
        if topic_name in get_camera_feed.camera_node.latest_images:
            return get_camera_feed.camera_node.latest_images[topic_name]
    
    return get_placeholder_image(text=f"No feed: {topic_name.split('/')[-1]}")

def update_camera_feeds():
    """Update all camera feeds"""
    exocentric = get_camera_feed('/exocentric_camera')
    intel_rgb = get_camera_feed('/intel_camera_rgb') 
    depth_map = get_camera_feed('/intel_camera_depth')
    custom_view = get_camera_feed('/custom_camera')
    
    return exocentric, intel_rgb, depth_map, custom_view

# --- ROS2 Initialization ---
def init_ros2():
    """Initialize ROS2 and camera node"""
    if not ROS2_AVAILABLE:
        print("ROS2 not available, using placeholder images")
        return False
        
    try:
        if not rclpy.ok():
            rclpy.init()
        
        get_camera_feed.camera_node = CameraNode()
        
        def spin_node():
            try:
                rclpy.spin(get_camera_feed.camera_node)
            except Exception as e:
                print(f"Error spinning ROS node: {e}")
        
        # Start ROS2 spinning in a separate thread
        ros_thread = threading.Thread(target=spin_node, daemon=True)
        ros_thread.start()
        print("ROS2 camera node initialized successfully")
        return True
    except Exception as e:
        print(f"Failed to initialize ROS2: {e}")
        get_camera_feed.camera_node = None
        return False

# --- Predefined Prompts ---
setup_scene_prompt = """
list available topics and read jenga pose.
set pz values =0 and place the blocks in the scene in isaac sim
while following this convention
isaac sim px = -1x px of jenga
same for y axis 

jenga blocks at omniverse://localhost/Library/Aruco/objs/jenga.usd

Rotation- follow [w, x, y, z] order.

"""
reset_scene_prompt="""
list available topics and read jenga pose.
list prims in isaac sim and find if the jenga blocks are already imported. if they are then just update the pose of the jenga block using the following logic.
if the jenga block doesnt exist in topics and it exists in isaac sim prim list then delete that prim.
If there are more jenga blocks in topics and some are missing in the scene, then add new jenga block in the right orientation.
set pz values =0 and place the blocks in the scene in isaac sim
while following this convention
isaac sim px = -1x px of jenga
same for y axis 

jenga blocks at omniverse://localhost/Library/Aruco/objs/jenga.usd

Rotation- follow [w, x, y, z] order.

"""
pick_place_prompt = """
Pick and place prompt

Step0: pick one jenga block out of avaible jenga blocks (not the one already moved to new position)

Step1: read jenga block pose : x,y,z , rx,ry,rz

Step2: move UR ee to : -x,-y,z=0.2 with 0,180,0

Step3: move ee orientation to pick jenga block = 0,180,(UR rz) where UR rz= (90-Jenga rz)x-1

Step4: move UR ee to : -x,-y,z=0.15 with 0,180,(UR rz)

Step5: Close gripper 

Step6: move UR ee to : -x,-y,z=0.2 # and confirm grasp visually (access exocentric camera view)

Step7: set ee orientation with 0,180,(desired final rotation)- no changes if nothing specified
required_rotation = desired_final_rotation - current_jenga_rz new_ee_rz = current_ee_rz + required_rotation

Step8: move UR ee to : final pose x,y,0.2 to drop (Final pose - remember x,y,z in isaac sim is -x,-y ee in UR frame. So if i want to place it at 0.3,0.5 then use -0.3,-0.5 to perform ik calculation.)

Step9: move the ee z to  0.15

Step10: open gripper

Step11: go home: HOME_POSE = [0.065, -0.385, 0.481, 0, 180, 0] 

Target Isaac Sim position: [0.3, 0.5] with jenga block position rz 0

repeat till no jenga blocks remain in their initial pose. """

stack_prompt = """Stack the red cube on top of the blue cylinder."""

push_prompt = """Push the green sphere 10 cm forward."""

# --- Gradio UI ---
def create_interface():
    # Initialize ROS2
    init_ros2()
    
    # Initialize working agents
    initialize_working_agents()
    
    # Build agent choices - only Claude and ChatGPT
    agent_choices = []
    
    if WORKING_AGENTS_AVAILABLE and claude_agent is not None:
        agent_choices.append("Claude")
    if WORKING_AGENTS_AVAILABLE and chatgpt_agent is not None:
        agent_choices.append("ChatGPT")
    
    # Fallback if no agents available
    if not agent_choices:
        agent_choices = ["No Agents Available"]
    
    with gr.Blocks(title="Isaac Sim Robot Control Panel", theme=gr.themes.Soft()) as demo:
        gr.Markdown("# Isaac Sim Robot Control Panel")
        gr.Markdown("Control your Isaac Sim environment with AI agents and view live camera feeds")
        
        # Status display
        status_items = []
        status_items.append(f"Claude: {'Available' if WORKING_AGENTS_AVAILABLE and claude_agent else 'Unavailable'}")
        status_items.append(f"ChatGPT: {'Available' if WORKING_AGENTS_AVAILABLE and chatgpt_agent else 'Unavailable'}")
        status_items.append(f"ROS2: {'Available' if ROS2_AVAILABLE else 'Unavailable'}")
        status_items.append(f"Isaac MCP: {'Available' if WORKING_AGENTS_AVAILABLE else 'Unavailable'}")
        
        gr.Markdown("**System Status:** " + " | ".join(status_items))
        
        with gr.Row():
            # LEFT side - Agent Control
            # with gr.Column(scale=1):
            #     gr.Markdown("## AI Agent Control")
            #     agent_selector = gr.Dropdown(
            #         label="Select AI Agent",
            #         choices=agent_choices,
            #         value=agent_choices[0],
            #         interactive=True
            #     )
                
            #     chatbox = gr.Chatbot(
            #         height=400, 
            #         label="Agent Communication",
            #         type="messages"
            #     )
                
            #     msg = gr.Textbox(
            #         placeholder="Enter your command for the robot...", 
            #         label="Your Message",
            #         lines=2
            #     )
                
            #     with gr.Row():
            #         send_btn = gr.Button("Send", variant="primary")
            #         clear_btn = gr.Button("Clear Chat")

            # RIGHT side - Camera Feeds and Primitives
            with gr.Column(scale=1):
                gr.Markdown("## Camera Feeds")
                
                # Four camera views in tabs
                with gr.Tabs():
                    with gr.Tab("Exocentric"):
                        exocentric_image = gr.Image(
                            label="Exocentric View (/exocentric_camera)", 
                            height=250,
                            type="numpy"
                        )
                    
                    with gr.Tab("Intel RGB"):
                        intel_rgb_image = gr.Image(
                            label="Intel RGB View (/intel_camera_rgb)", 
                            height=250,
                            type="numpy"
                        )
                    
                    with gr.Tab("Depth Map"):
                        depth_image = gr.Image(
                            label="Depth Map (/intel_camera_depth)", 
                            height=250,
                            type="numpy"
                        )
                    
                    with gr.Tab("Custom"):
                        custom_image = gr.Image(
                            label="Custom View (/custom_camera)", 
                            height=250,
                            type="numpy"
                        )
                
                # Camera refresh button
                refresh_btn = gr.Button("Refresh Cameras", variant="secondary")
                
                # Primitive Commands Section
                gr.Markdown("## Primitive Commands")
                with gr.Tabs():
                    with gr.Tab("Setup Scene"):
                        setup_textbox = gr.Textbox(
                            value=setup_scene_prompt,
                            lines=3, 
                            interactive=True, 
                            show_label=False,
                            placeholder="Setup scene command..."
                        )
                        setup_copy_btn = gr.Button("Copy Setup Command", size="sm")
                    
                    with gr.Tab("Pick & Place"):
                        pick_place_textbox = gr.Textbox(
                            value=pick_place_prompt, 
                            lines=8, 
                            interactive=True, 
                            show_label=False
                        )
                        pick_copy_btn = gr.Button("Copy Pick & Place Command", size="sm")
                    
                    with gr.Tab("Stack"):
                        stack_textbox = gr.Textbox(
                            value=stack_prompt, 
                            lines=3, 
                            interactive=True, 
                            show_label=False
                        )
                        stack_copy_btn = gr.Button("Copy Stack Command", size="sm")
                    
                    with gr.Tab("Push"):
                        push_textbox = gr.Textbox(
                            value=push_prompt, 
                            lines=3, 
                            interactive=True, 
                            show_label=False
                        )
                        push_copy_btn = gr.Button("Copy Push Command", size="sm")

        # Event handlers
        def send_message(message, history, agent_name):
            if message.strip() == "":
                return history, ""
            
            if agent_name == "No Agents Available":
                response = "No working agents available. Please check your API keys and file paths."
            else:
                response = run_agent_request(message, history, agent_name)
            
            # Add messages in proper format
            new_history = history + [
                {"role": "user", "content": message},
                {"role": "assistant", "content": response}
            ]
            return new_history, ""

        def clear_chat():
            return []

        def copy_to_message(command_text):
            return command_text

        # Chat event handlers
        # send_btn.click(
        #     fn=send_message,
        #     inputs=[msg, chatbox, agent_selector],
        #     outputs=[chatbox, msg]
        # )
        
        # msg.submit(
        #     fn=send_message,
        #     inputs=[msg, chatbox, agent_selector], 
        #     outputs=[chatbox, msg]
        # )
        
        # clear_btn.click(
        #     fn=clear_chat,
        #     outputs=chatbox
        # )
        
        # Camera refresh handler
        refresh_btn.click(
            fn=update_camera_feeds,
            outputs=[exocentric_image, intel_rgb_image, depth_image, custom_image]
        )

        # Copy button handlers
        setup_copy_btn.click(
            fn=copy_to_message,
            inputs=[setup_textbox],
            outputs=[]
        )
        
        pick_copy_btn.click(
            fn=copy_to_message,
            inputs=[pick_place_textbox],
            outputs=[]
        )
        
        stack_copy_btn.click(
            fn=copy_to_message,
            inputs=[stack_textbox],
            outputs=[]
        )
        
        push_copy_btn.click(
            fn=copy_to_message,
            inputs=[push_textbox],
            outputs=[]
        )

        # Initial load of camera feeds
        demo.load(
            fn=update_camera_feeds,
            outputs=[exocentric_image, intel_rgb_image, depth_image, custom_image]
        )

    return demo

# --- Main Execution ---
if __name__ == "__main__":
    try:
        print("\n" + "="*60)
        print("ISAAC SIM ROBOT CONTROL PANEL")
        print("="*60)
        
        # Check Gradio version
        try:
            import gradio
            print(f"Gradio version: {gradio.__version__}")
        except:
            print("Could not determine Gradio version")
        
        print(f"Looking for isaac_mcp files in: {isaac_mcp_dir}")
        
        demo = create_interface()
        print("\nStarting Gradio interface...")
        print("Available camera topics will be automatically detected")
        print("="*60)
        print("AGENT STATUS:")
        print(f"  Claude Agent: {'Available' if WORKING_AGENTS_AVAILABLE and claude_agent else 'Unavailable'}")
        print(f"  ChatGPT Agent: {'Available' if WORKING_AGENTS_AVAILABLE and chatgpt_agent else 'Unavailable'}")
        print(f"  ROS2: {'Available' if ROS2_AVAILABLE else 'Using placeholders'}")
        print(f"  Isaac MCP: {'Available' if WORKING_AGENTS_AVAILABLE else 'Check file paths'}")
        print("="*60)
        
        if not WORKING_AGENTS_AVAILABLE:
            print("WARNING: Working agents not available!")
            print("   Make sure your files are organized like this:")
            print("   isaac-sim-mcp/")
            print("   ├── isaac_mcp/")
            print("   │   ├── host.py")
            print("   │   ├── client.py") 
            print("   │   ├── server.py")
            print("   │   └── main.py")
            print("   └── gradio_interface.py")
            print("="*60)
        
        demo.launch(
            server_name="0.0.0.0",
            server_port=7860,
            share=False,
            debug=True
        )
    except KeyboardInterrupt:
        print("\nShutting down...")
        if ROS2_AVAILABLE:
            try:
                rclpy.shutdown()
            except:
                pass
    except Exception as e:
        print(f"Error starting interface: {e}")
        if ROS2_AVAILABLE:
            try:
                rclpy.shutdown()
            except:
                passmsg