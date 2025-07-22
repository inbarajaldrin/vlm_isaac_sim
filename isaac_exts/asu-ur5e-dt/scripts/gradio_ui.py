import asyncio
import random
import gradio as gr
import numpy as np
import cv2
import threading
import time
from typing import Optional
from contextlib import AsyncExitStack

# Fix NumPy compatibility issue
import os
import sys
os.environ['NUMPY_ARRAY_API'] = '1'

# Add paths for agent imports
sys.path.append("/home/aaugus11/Documents/isaac-sim-mcp/agent")
sys.path.append("/home/aaugus11/Documents/isaac-sim-mcp/isaac_mcp")

# ROS2 imports with better error handling
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    
    # Try cv_bridge import with fallback
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

# MCP imports 
try:
    from mcp import ClientSession, StdioServerParameters
    from mcp.client.stdio import stdio_client
    from anthropic import Anthropic
    MCP_AVAILABLE = True
except ImportError:
    print("MCP not available. Using fallback agent functions.")
    MCP_AVAILABLE = False

# --- MCP Client Integration ---
class MCPClient:
    def __init__(self):
        self.session: Optional[ClientSession] = None
        self.exit_stack = AsyncExitStack()
        if MCP_AVAILABLE:
            try:
                self.anthropic = Anthropic()
            except:
                print("Anthropic API key not found. MCP client will use basic responses.")
                self.anthropic = None

    async def connect_to_server(self, server_script_path: str = "/home/aaugus11/Documents/isaac-sim-mcp/isaac_mcp/server.py"):
        """Connect to Isaac Sim MCP server"""
        if not MCP_AVAILABLE:
            return False
            
        try:
            server_params = StdioServerParameters(
                command="python",
                args=[server_script_path],
                env=None
            )
            
            stdio_transport = await self.exit_stack.enter_async_context(stdio_client(server_params))
            self.stdio, self.write = stdio_transport
            self.session = await self.exit_stack.enter_async_context(ClientSession(self.stdio, self.write))
            
            await self.session.initialize()
            
            # List available tools
            response = await self.session.list_tools()
            tools = response.tools
            print("Connected to Isaac Sim MCP server with tools:", [tool.name for tool in tools])
            return True
        except Exception as e:
            print(f"Failed to connect to MCP server: {e}")
            return False

    async def process_query(self, query: str) -> str:
        """Process a query using Claude and available MCP tools"""
        if not MCP_AVAILABLE or not self.session:
            return "MCP not available or not connected to server."
        
        if not self.anthropic:
            return f"Isaac Sim MCP: Processing '{query}' - Anthropic API not configured"
        
        try:
            # Get available tools
            response = await self.session.list_tools()
            tools = response.tools
            
            # Format tools for Claude
            tools_formatted = []
            for tool in tools:
                tools_formatted.append({
                    "name": tool.name,
                    "description": tool.description,
                    "input_schema": tool.inputSchema
                })

            # Send to Claude
            messages = [{"role": "user", "content": query}]
            
            response = self.anthropic.messages.create(
                model="claude-3-sonnet-20240229",
                max_tokens=1000,
                tools=tools_formatted,
                messages=messages
            )
            
            # Handle tool calls if any
            if response.stop_reason == "tool_use":
                for content_block in response.content:
                    if content_block.type == "tool_use":
                        tool_name = content_block.name
                        tool_input = content_block.input
                        
                        # Call MCP tool
                        tool_result = await self.session.call_tool(tool_name, tool_input)
                        
                        # Add tool result to conversation and get final response
                        messages.extend([
                            {"role": "assistant", "content": response.content},
                            {"role": "user", "content": [{"type": "tool_result", "tool_use_id": content_block.id, "content": str(tool_result.content)}]}
                        ])
                        
                        final_response = self.anthropic.messages.create(
                            model="claude-3-sonnet-20240229",
                            max_tokens=1000,
                            messages=messages
                        )
                        return final_response.content[0].text
            
            return response.content[0].text
            
        except Exception as e:
            return f"Error processing query: {e}"

    async def cleanup(self):
        """Clean up resources"""
        if self.exit_stack:
            await self.exit_stack.aclose()

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
                # cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                self.latest_images[topic] = cv_image
            except Exception as e:
                self.get_logger().error(f'Error converting image from {topic}: {e}')
else:
    class CameraNode:
        def __init__(self):
            self.latest_images = {}

# --- Agent Functions ---
mcp_client = MCPClient()

def agent_chatgpt(message, history):
    """ChatGPT agent using OpenAI API"""
    try:
        import openai
        client = openai.OpenAI()  # Uses OPENAI_API_KEY from environment
        
        # Build conversation history
        messages = [
            {"role": "system", "content": "You are an AI assistant helping with Isaac Sim robotics simulation. You can help with robot control commands, scene setup, and general robotics questions."}
        ]
        
        for msg in history[-10:]:  # Keep last 10 messages for context
            if isinstance(msg, dict) and msg.get("role") in ["user", "assistant"]:
                messages.append({"role": msg["role"], "content": msg.get("content", "")})
        
        # Add current message
        messages.append({"role": "user", "content": message})
        
        response = client.chat.completions.create(
            model="gpt-4",
            messages=messages,
            max_tokens=2000,
            temperature=0.7
        )
        
        return response.choices[0].message.content
        
    except ImportError:
        return "ChatGPT agent not available. Please install the OpenAI library: pip install openai"
    except Exception as e:
        return f"ChatGPT agent error: {e}. Make sure OPENAI_API_KEY is set in your environment."

async def agent_isaac_sim_mcp(message, history):
    """Isaac Sim MCP Agent using real MCP connection"""
    if not MCP_AVAILABLE:
        return "MCP not available. Please install required dependencies."
    
    try:
        # Connect to MCP server if not already connected
        if not mcp_client.session:
            connected = await mcp_client.connect_to_server()
            if not connected:
                return "Failed to connect to Isaac Sim MCP server. Please ensure the server is running."
        
        # Process the query
        response = await mcp_client.process_query(message)
        return response
        
    except Exception as e:
        return f"Error communicating with Isaac Sim: {e}"

def agent_isaac_sim_fallback(message, history):
    """Fallback Isaac Sim agent for when MCP is not available"""
    if "scene" in message.lower():
        return "Isaac Sim: Scene analysis complete. Current objects detected: UR robot, Jenga blocks at various positions."
    elif "pick" in message.lower() or "place" in message.lower():
        return "Isaac Sim: Executing pick and place sequence. Moving to target position..."
    elif "home" in message.lower():
        return "Isaac Sim: Returning to home position [0.065, -0.385, 0.481, 0, 180, 0]"
    elif "gripper" in message.lower():
        return "Isaac Sim: Gripper command received. Adjusting gripper state..."
    else:
        return f"Isaac Sim: Processing command '{message}'. Simulation running..."

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

# --- Gradio Interface Functions ---
def route_agent(message, history, agent_name):
    """Route message to appropriate agent"""
    if agent_name == "Random":
        return agent_random(message, history)
    elif agent_name == "ChatGPT":
        return agent_chatgpt(message, history)
    elif agent_name == "Claude":
        if MCP_AVAILABLE:
            # Run async function in event loop
            try:
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    # Create a new task for the async function
                    import concurrent.futures
                    with concurrent.futures.ThreadPoolExecutor() as executor:
                        future = executor.submit(asyncio.run, agent_isaac_sim_mcp(message, history))
                        return future.result(timeout=30)
                else:
                    return asyncio.run(agent_isaac_sim_mcp(message, history))
            except Exception as e:
                return f"Error: {e}"
        else:
            return agent_isaac_sim_fallback(message, history)
    else:
        return agent_random(message, history)

def update_camera_feeds():
    """Update all camera feeds"""
    exocentric = get_camera_feed('/exocentric_camera')
    intel_rgb = get_camera_feed('/intel_camera_rgb') 
    depth_map = get_camera_feed('/intel_camera_depth')
    custom_view = get_camera_feed('/custom_camera')
    
    return exocentric, intel_rgb, depth_map, custom_view

# --- Predefined Prompts ---
setup_scene_prompt = """Set up a table with 3 objects: a red cube, a blue cylinder, and a green sphere placed randomly."""

pick_place_prompt = """**Pick and place prompt**
Step1: read jenga block pose : x,y,z , rx,ry,rz
Step2: move UR ee to : -x,-y,z=0.3 with 0,180,0
Step3: move ee orientation to pick jenga block = 0,180,(UR rz) where UR rz= (90-Jenga rz)x-1
Step4: move UR ee to : -x,-y,z=0.24 with 0,180,(UR rz)
Step5: Close gripper and confirm grasp visually (access exocentric camera view)
Step6: move UR ee to : -x,-y,z=0.3 
Step7: set ee orientation with 0,180,(desired final rotation)- no changes if nothing specified
required_rotation = desired_final_rotation - current_jenga_rz new_ee_rz = current_ee_rz + required_rotation

Step8: move UR ee to : final pose x,y,0.3 to drop (Final pose - remember x,y,z in isaac sim is -x,-y ee in UR frame. So if i want to place it at 0.3,0.5 then use -0.3,-0.5 to perform ik calculation.)
Step9: move the ee z to  0.24
Step10: open gripper
Step11: go home: HOME_POSE = [0.065, -0.385, 0.481, 0, 180, 0] Target Isaac Sim position: [0.0, 0.5] with jenga block position rz 30 
Target Isaac Sim position: [0, 0.5] with no rotation to jenga block when placing"""

stack_prompt = """Stack the red cube on top of the blue cylinder."""

push_prompt = """Push the green sphere 10 cm forward."""

# --- Gradio UI ---
def create_interface():
    # Initialize ROS2
    init_ros2()
    
    with gr.Blocks(title="Robot-Agent Control Panel", theme=gr.themes.Soft()) as demo:
        gr.Markdown("# Isaac Sim Robot Control Panel")
        gr.Markdown("Control your Isaac Sim environment with AI agents and view live camera feeds")
        
        with gr.Row():
            # LEFT side - Agent Control
            with gr.Column(scale=1):
                gr.Markdown("## AI Agent Control")
                agent_selector = gr.Dropdown(
                    label="Select AI Agent",
                    choices=["Claude", "ChatGPT"],
                    value="Claude",
                    interactive=True
                )
                
                # Fixed chatbot with proper type parameter
                chatbox = gr.Chatbot(
                    height=400, 
                    label="Agent Communication",
                    type="messages"
                )
                
                msg = gr.Textbox(
                    placeholder="Enter your command for the robot...", 
                    label="Your Message",
                    lines=2
                )
                
                with gr.Row():
                    send_btn = gr.Button("Send", variant="primary")
                    clear_btn = gr.Button("Clear Chat")

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
                
                # Primitive Commands Section - Below cameras
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
                    
                    with gr.Tab("Push Primitive"):
                        push_textbox = gr.Textbox(
                            value=push_prompt, 
                            lines=3, 
                            interactive=True, 
                            show_label=False
                        )
                        push_copy_btn = gr.Button("Copy Push Command", size="sm")

        # Event handlers - Fixed for new Gradio format
        def send_message(message, history, agent_name):
            if message.strip() == "":
                return history, ""
            
            response = route_agent(message, history, agent_name)
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
        send_btn.click(
            fn=send_message,
            inputs=[msg, chatbox, agent_selector],
            outputs=[chatbox, msg]
        )
        
        msg.submit(
            fn=send_message,
            inputs=[msg, chatbox, agent_selector], 
            outputs=[chatbox, msg]
        )
        
        clear_btn.click(
            fn=clear_chat,
            outputs=chatbox
        )
        
        # Camera refresh handler
        refresh_btn.click(
            fn=update_camera_feeds,
            outputs=[exocentric_image, intel_rgb_image, depth_image, custom_image]
        )

        # Copy button handlers
        setup_copy_btn.click(
            fn=copy_to_message,
            inputs=[setup_textbox],
            outputs=[msg]
        )
        
        pick_copy_btn.click(
            fn=copy_to_message,
            inputs=[pick_place_textbox],
            outputs=[msg]
        )
        
        stack_copy_btn.click(
            fn=copy_to_message,
            inputs=[stack_textbox],
            outputs=[msg]
        )
        
        push_copy_btn.click(
            fn=copy_to_message,
            inputs=[push_textbox],
            outputs=[msg]
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
        # Upgrade Gradio if needed
        try:
            import gradio
            print(f"Gradio version: {gradio.__version__}")
        except:
            print("Could not determine Gradio version")
        
        demo = create_interface()
        print("Starting Gradio interface...")
        print("Available camera topics will be automatically detected")
        print("Claude Agent:", "Available" if MCP_AVAILABLE else "Using fallback")
        print("ChatGPT Agent: Install 'openai' library and set OPENAI_API_KEY to enable")
        print("ROS2:", "Available" if ROS2_AVAILABLE else "Using placeholders")
        
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
        if MCP_AVAILABLE:
            asyncio.run(mcp_client.cleanup())
    except Exception as e:
        print(f"Error starting interface: {e}")
        if ROS2_AVAILABLE:
            try:
                rclpy.shutdown()
            except:
                pass
        if MCP_AVAILABLE:
            try:
                asyncio.run(mcp_client.cleanup())
            except:
                pass
