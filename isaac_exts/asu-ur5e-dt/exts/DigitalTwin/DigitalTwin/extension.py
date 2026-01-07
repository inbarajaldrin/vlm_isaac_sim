import omni.ext
import omni.ui as ui
import asyncio
import numpy as np
import os
import threading
import glob
import omni.client
import math
import re

from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.core.articulations import Articulation
import omni.graph.core as og
import omni.usd
from pxr import UsdGeom, Gf, Sdf, Usd
from scipy.spatial.transform import Rotation as R

class DigitalTwin(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[DigitalTwin] Digital Twin startup")

        self._timeline = omni.timeline.get_timeline_interface()
        self._ur5e_view = None
        self._articulation = None
        self._gripper_view = None
        
        # Add Objects UI state
        self._objects_folder_path = "omniverse://localhost/Library/DT demo/aruco_fmb/"
        self._object_spacing = 0.25  # Spacing between objects along X-axis in meters
        self._y_offset = -0.5  # Y offset for all objects
        self._z_offset = 0.0495  # Z offset for all objects

        # Isaac Sim handles ROS2 initialization automatically through its bridge
        print("ROS2 bridge will be initialized by Isaac Sim when needed")

        # Create the window UI
        self._window = ui.Window("UR5e Digital Twin", width=300, height=800)  # Increased height
        with self._window.frame:
            with ui.VStack(spacing=5):
                self.create_ui()

    def create_ui(self):
        with ui.VStack(spacing=5):
            with ui.CollapsableFrame(title="Setup", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Simulation Setup", alignment=ui.Alignment.LEFT)
                    with ui.HStack(spacing=5):
                        ui.Button("Load Scene", width=100, height=35, clicked_fn=lambda: asyncio.ensure_future(self.load_scene()))
                        ui.Button("Refresh Graphs", width=120, height=35, clicked_fn=self.refresh_graphs)

            with ui.CollapsableFrame(title="Randomize Object Poses", collapsed=False, height=0):
                ui.Button("Randomize Object Poses", width=250, height=35, clicked_fn=self.randomize_object_poses)

            with ui.CollapsableFrame(title="UR5e Control", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("UR5e Robot Control", alignment=ui.Alignment.LEFT)
                    with ui.HStack(spacing=5):
                        ui.Button("Load UR5e", width=100, height=35, clicked_fn=lambda: asyncio.ensure_future(self.load_ur5e()))
                        ui.Button("Setup UR5e Action Graph", width=200, height=35, clicked_fn=lambda: asyncio.ensure_future(self.setup_action_graph()))

            with ui.CollapsableFrame(title="RG2 Gripper", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("RG2 Gripper Control", alignment=ui.Alignment.LEFT)
                    with ui.HStack(spacing=5):
                        ui.Button("Import RG2 Gripper", width=150, height=35, clicked_fn=self.import_rg2_gripper)
                        ui.Button("Attach Gripper to UR5e", width=180, height=35, clicked_fn=self.attach_rg2_to_ur5e)
                    
                    with ui.HStack(spacing=5):
                        ui.Button("Setup Gripper Action Graph", width=200, height=35, clicked_fn=self.setup_gripper_action_graph)
                        ui.Button("Setup Force Publish Graph", width=200, height=35, clicked_fn=self.setup_force_publish_action_graph)

            # Intel RealSense Camera
            with ui.CollapsableFrame(title="Intel RealSense Camera", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Intel RealSense D455 Camera", alignment=ui.Alignment.LEFT)
                    with ui.HStack(spacing=5):
                        ui.Button("Import RealSense Camera", width=170, height=35, clicked_fn=self.import_realsense_camera)
                        ui.Button("Attach Camera to UR5e", width=160, height=35, clicked_fn=self.attach_camera_to_ur5e)
                    
                    with ui.HStack(spacing=5):
                        ui.Button("Setup Camera Action Graph", width=200, height=35, clicked_fn=self.setup_camera_action_graph)

            # NEW SECTION: Additional Camera
            with ui.CollapsableFrame(title="Additional Camera", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Camera Configuration", alignment=ui.Alignment.LEFT)
                    
                    # Camera type selection with checkboxes and labels
                    with ui.VStack(spacing=5):
                        with ui.HStack(spacing=5):
                            self._exocentric_checkbox = ui.CheckBox(width=20)
                            self._exocentric_checkbox.model.set_value(True)  # Default checked
                            ui.Label("Exocentric View", alignment=ui.Alignment.LEFT, width=120)

                        with ui.HStack(spacing=5):
                            self._custom_checkbox = ui.CheckBox(width=20)
                            ui.Label("Close Up View", alignment=ui.Alignment.LEFT, width=120)

                    # Camera control buttons
                    with ui.HStack(spacing=5):
                        ui.Button("Create Camera", width=150, height=35, clicked_fn=self.create_additional_camera)
                        ui.Button("Create Action Graph", width=180, height=35, clicked_fn=self.create_additional_camera_actiongraph)

            # Add Objects section
            with ui.CollapsableFrame(title="Add Objects", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Import Objects from Folder", alignment=ui.Alignment.LEFT)
                    
                    # Folder path input
                    with ui.HStack(spacing=5):
                        ui.Label("Objects Folder Path:", alignment=ui.Alignment.LEFT, width=150)
                        self._objects_path_field = ui.StringField(width=300)
                        self._objects_path_field.model.set_value(self._objects_folder_path)
                    
                    # Add Objects button
                    with ui.HStack(spacing=5):
                        ui.Button("Add Objects", width=150, height=35, clicked_fn=self.add_objects)
                        ui.Button("Setup pose publisher action graph", width=250, height=35, clicked_fn=self.create_pose_publisher)

    async def load_scene(self):
        world = World()
        await world.initialize_simulation_context_async()
        world.scene.add_default_ground_plane()
        print("Scene loaded successfully.")

    def refresh_graphs(self):
        """Delete /World/Graphs folder and automatically recreate all graphs"""
        import omni.usd
        from pxr import Usd, Sdf
        import asyncio
        
        print("Refreshing graphs...")
        
        # Get the current stage
        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("Error: No stage found")
            return
        
        # Check which graphs exist BEFORE deleting the Graphs folder
        graphs_path = "/World/Graphs"
        graph_paths_to_recreate = []
        
        # Check for each graph's existence
        ur5e_graph = stage.GetPrimAtPath(f"{graphs_path}/ActionGraph_UR5e")
        if ur5e_graph.IsValid():
            graph_paths_to_recreate.append("UR5e")
        
        gripper_graph = stage.GetPrimAtPath(f"{graphs_path}/ActionGraph_RG2")
        if gripper_graph.IsValid():
            graph_paths_to_recreate.append("RG2")
        
        force_graph = stage.GetPrimAtPath(f"{graphs_path}/ActionGraph_RG2_ForcePublish")
        if force_graph.IsValid():
            graph_paths_to_recreate.append("RG2_ForcePublish")
        
        camera_graph = stage.GetPrimAtPath(f"{graphs_path}/ActionGraph_Camera")
        if camera_graph.IsValid():
            graph_paths_to_recreate.append("Camera")
        
        exocentric_graph = stage.GetPrimAtPath(f"{graphs_path}/ActionGraph_ExocentricCamera")
        if exocentric_graph.IsValid():
            graph_paths_to_recreate.append("ExocentricCamera")
        
        custom_graph = stage.GetPrimAtPath(f"{graphs_path}/ActionGraph_CustomCamera")
        if custom_graph.IsValid():
            graph_paths_to_recreate.append("CustomCamera")
        
        objects_poses_graph = stage.GetPrimAtPath(f"{graphs_path}/ActionGraph_objects_poses")
        if objects_poses_graph.IsValid():
            graph_paths_to_recreate.append("objects_poses")
        
        # Delete the entire /World/Graphs folder if it exists
        graphs_prim = stage.GetPrimAtPath(graphs_path)
        if graphs_prim.IsValid():
            stage.RemovePrim(graphs_path)
            print(f"Deleted existing {graphs_path} folder")
        
        # Recreate the Graphs folder
        UsdGeom.Xform.Define(stage, graphs_path)
        print(f"Created new {graphs_path} folder")
        
        # Automatically recreate only graphs that existed
        print(f"Recreating {len(graph_paths_to_recreate)} existing graphs...")
        
        # Create UR5e Action Graph
        if "UR5e" in graph_paths_to_recreate:
            try:
                asyncio.ensure_future(self.setup_action_graph())
                print("✓ UR5e Action Graph recreated")
            except Exception as e:
                print(f"✗ Failed to recreate UR5e Action Graph: {e}")
        
        # Create Gripper Action Graph
        if "RG2" in graph_paths_to_recreate:
            try:
                self.setup_gripper_action_graph()
                print("✓ Gripper Action Graph recreated")
            except Exception as e:
                print(f"✗ Failed to recreate Gripper Action Graph: {e}")
        
        # Create Force Publish Action Graph
        if "RG2_ForcePublish" in graph_paths_to_recreate:
            try:
                self.setup_force_publish_action_graph()
                print("✓ Force Publish Action Graph recreated")
            except Exception as e:
                print(f"✗ Failed to recreate Force Publish Action Graph: {e}")
        
        # Create Camera Action Graph
        if "Camera" in graph_paths_to_recreate:
            try:
                self.setup_camera_action_graph()
                print("✓ Camera Action Graph recreated")
            except Exception as e:
                print(f"✗ Failed to recreate Camera Action Graph: {e}")
        
        # Create Additional Camera Action Graphs (only if graphs existed)
        if "ExocentricCamera" in graph_paths_to_recreate or "CustomCamera" in graph_paths_to_recreate:
            try:
                stage = omni.usd.get_context().get_stage()
                
                if "ExocentricCamera" in graph_paths_to_recreate:
                    # Get camera prim path from the graph's render product node if possible
                    # For now, use default path
                    self._create_camera_actiongraph(
                        "/World/exocentric_camera", 
                        1280, 720, 
                        "exocentric_camera", 
                        "ExocentricCamera"
                    )
                    print("✓ Exocentric Camera Action Graph recreated")
                
                if "CustomCamera" in graph_paths_to_recreate:
                    self._create_camera_actiongraph(
                        "/World/custom_camera", 
                        640, 480, 
                        "custom_camera", 
                        "CustomCamera"
                    )
                    print("✓ Custom Camera Action Graph recreated")
            except Exception as e:
                print(f"✗ Failed to recreate Additional Camera Action Graphs: {e}")
        
        # Create Pose Publisher Action Graph (only if graph existed)
        if "objects_poses" in graph_paths_to_recreate:
            try:
                self.create_pose_publisher()
                print("✓ Pose Publisher Action Graph recreated")
            except Exception as e:
                print(f"✗ Failed to recreate Pose Publisher Action Graph: {e}")
        
        print("Graph refresh completed!")



    async def load_ur5e(self):
        asset_path = "omniverse://localhost/Library/ur5e.usd"
        prim_path = "/World/UR5e"
        
        # 1. Add the USD asset
        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)

        # 2. Wait for prim to exist
        import time
        stage = omni.usd.get_context().get_stage()
        for _ in range(10):
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(f"Failed to load prim at {prim_path}")

        # 3. Apply translation and orientation
        xform = UsdGeom.Xform(prim)
        xform.ClearXformOpOrder()

        # Custom position and orientation
        position = Gf.Vec3d(0.0, 0.0, 0.0)  # Replace with your desired position
        rpy_deg = np.array([0.0, 0.0, 180.0])  # Replace with your desired RPY
        rpy_rad = np.deg2rad(rpy_deg)
        quat_xyzw = euler_angles_to_quats(rpy_rad)
        quat = Gf.Quatd(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3])

        xform.AddTranslateOp().Set(position)
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)

        print(f"Applied translation and rotation to {prim_path}")

        # 4. Setup Articulation
        self._ur5e_view = ArticulationView(prim_paths_expr=prim_path, name="ur5e_view")
        World.instance().scene.add(self._ur5e_view)
        await World.instance().reset_async()
        self._timeline.stop()

        self._articulation = Articulation(prim_path)

        print("UR5e robot loaded successfully!")

        
    async def setup_action_graph(self):
        import omni.graph.core as og
        import omni.isaac.core.utils.stage as stage_utils
        from omni.isaac.core.utils.extensions import enable_extension

        print("Setting up ROS 2 Action Graph...")

        # Ensure extensions are enabled
        enable_extension("isaacsim.ros2.bridge")
        enable_extension("isaacsim.core.nodes")
        enable_extension("omni.graph.action")

        graph_path = "/World/Graphs/ActionGraph_UR5e"
        (graph, nodes, _, _) = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution"
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("isaac_read_simulation_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ros2_subscribe_joint_state", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ros2_publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("articulation_controller", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ros2_context.inputs:useDomainIDEnvVar", True),
                    ("ros2_context.inputs:domain_id", 0),
                    ("ros2_subscribe_joint_state.inputs:topicName", "/joint_states"),
                    ("ros2_subscribe_joint_state.inputs:nodeNamespace", ""),
                    ("ros2_subscribe_joint_state.inputs:queueSize", 10),
                    ("ros2_publish_clock.inputs:topicName", "/clock"),
                    ("ros2_publish_clock.inputs:nodeNamespace", ""),
                    ("ros2_publish_clock.inputs:qosProfile", "SYSTEM_DEFAULT"),
                    ("ros2_publish_clock.inputs:queueSize", 10),
                    ("isaac_read_simulation_time.inputs:resetOnStop", True),
                    ("articulation_controller.inputs:robotPath", "/World/UR5e"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "ros2_subscribe_joint_state.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "ros2_publish_clock.inputs:execIn"),
                    ("on_playback_tick.outputs:tick", "articulation_controller.inputs:execIn"),
                    ("ros2_context.outputs:context", "ros2_subscribe_joint_state.inputs:context"),
                    ("ros2_context.outputs:context", "ros2_publish_clock.inputs:context"),
                    ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_clock.inputs:timeStamp"),
                    ("ros2_subscribe_joint_state.outputs:positionCommand", "articulation_controller.inputs:positionCommand"),
                    ("ros2_subscribe_joint_state.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
                    ("ros2_subscribe_joint_state.outputs:effortCommand", "articulation_controller.inputs:effortCommand"),
                    ("ros2_subscribe_joint_state.outputs:jointNames", "articulation_controller.inputs:jointNames"),
                ],
            }
        )

        print("ROS 2 Action Graph setup complete.")

    def setup_gripper_action_graph(self):
        """Setup gripper action graph for ROS2 control"""
        print("Setting up Gripper Action Graph...")
        
        graph_path = "/World/Graphs/ActionGraph_RG2"
        
        # Delete existing
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(graph_path):
            stage.RemovePrim(graph_path)
        
        keys = og.Controller.Keys
        
        print("Creating nodes...")
        # Create nodes including the new publisher and script_node
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    (f"{graph_path}/tick", "omni.graph.action.OnPlaybackTick"),
                    (f"{graph_path}/context", "isaacsim.ros2.bridge.ROS2Context"),
                    (f"{graph_path}/subscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    (f"{graph_path}/script", "omni.graph.scriptnode.ScriptNode"),
                    (f"{graph_path}/script_node", "omni.graph.scriptnode.ScriptNode"),
                    (f"{graph_path}/ros2_publisher", "isaacsim.ros2.bridge.ROS2Publisher")
                ]
            }
        )
        
        print("Adding script attributes...")
        # Add script attributes for command script
        script_node = og.Controller.node(f"{graph_path}/script")
        og.Controller.create_attribute(script_node, "inputs:String", og.Type(og.BaseDataType.TOKEN))
        og.Controller.create_attribute(script_node, "outputs:Integer", og.Type(og.BaseDataType.INT))
        
        # Add script_node attributes for reading gripper state - use proper pattern
        script_node_state = og.Controller.node(f"{graph_path}/script_node")
        og.Controller.create_attribute(
            script_node_state, 
            "gripper_width", 
            og.Type(og.BaseDataType.DOUBLE),
            attr_port=og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
        )
        print("Created outputs:gripper_width attribute on script_node")
        
        print("Setting values...")
        # Set values with try/catch for each one
        def safe_set(path, value, desc=""):
            try:
                attr = og.Controller.attribute(path)
                if attr.is_valid():
                    og.Controller.set(attr, value)
                    print(f"Set {desc}")
                    return True
                else:
                    print(f"Invalid: {desc}")
                    return False
            except Exception as e:
                print(f"Failed {desc}: {e}")
                return False
        
        # Configure ROS2 Subscriber
        safe_set(f"{graph_path}/subscriber.inputs:messageName", "String", "ROS2 message type")
        safe_set(f"{graph_path}/subscriber.inputs:messagePackage", "std_msgs", "ROS2 package")
        safe_set(f"{graph_path}/subscriber.inputs:topicName", "gripper_command", "ROS2 topic")
        
        # Configure Script Node
        safe_set(f"{graph_path}/script.inputs:usePath", False, "Use inline script")
        
        # Script that handles string processing internally
        script_content = '''from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.types import ArticulationActions
import numpy as np
import omni.timeline

gripper_view = None
last_sim_frame = -1

def setup(db):
    global gripper_view
    try:
        # Always create a fresh gripper view in setup
        gripper_view = ArticulationView(prim_paths_expr="/World/RG2_Gripper", name="gripper")
        gripper_view.initialize()
        db.log_info("Gripper initialized successfully")
    except Exception as e:
        db.log_error(f"Gripper setup failed: {e}")
        gripper_view = None

def compute(db):
    global gripper_view, last_sim_frame
    
    try:
        # Get input string from ROS2
        input_str = str(db.inputs.String).strip()
        
        # Handle string replacements in Python
        if input_str == "open":
            processed_str = "1100"
        elif input_str == "close":
            processed_str = "0"
        else:
            processed_str = input_str
        
        # Convert to width in mm
        try:
            width_mm = float(processed_str) / 10.0
        except ValueError:
            db.log_error(f"Invalid input: {input_str}")
            return
        
        # Check if simulation restarted by monitoring frame count
        timeline = omni.timeline.get_timeline_interface()
        current_frame = timeline.get_current_time() * timeline.get_time_codes_per_seconds()
        
        # If frame went backwards, simulation was restarted
        if current_frame < last_sim_frame or last_sim_frame == -1:
            db.log_info("Simulation restart detected, reinitializing gripper...")
            try:
                gripper_view = ArticulationView(prim_paths_expr="/World/RG2_Gripper", name="gripper")
                gripper_view.initialize()
                db.log_info("Gripper reinitialized after restart")
            except Exception as e:
                db.log_error(f"Gripper reinitialization failed: {e}")
                gripper_view = None
        
        last_sim_frame = current_frame
        
        # Check if gripper is available
        if gripper_view is None:
            db.log_warning("Gripper not available")
            return
        
        # Check if simulation is running
        if timeline.is_stopped():
            return
        
        # Try to apply action, with graceful handling of physics not ready
        try:
            # Clamp to valid range
            width_mm = np.clip(width_mm, 0.0, 110.0)
            
            # Map width to joint angle: 0mm = -π/4 (closed), 110mm = π/6 (open)
            ratio = width_mm / 110.0
            joint_angle = -np.pi/4 + ratio * (np.pi/4 + np.pi/6)
            
            # Apply to gripper
            target_positions = np.array([joint_angle, joint_angle])
            action = ArticulationActions(
                joint_positions=target_positions,
                joint_indices=np.array([0, 1])
            )
            gripper_view.apply_action(action)
            
            # Set output
            db.outputs.Integer = int(width_mm)
            
            db.log_info(f"Gripper: \\'{input_str}\\' -> {width_mm:.1f}mm -> {joint_angle:.3f}rad")
            
        except Exception as e:
            # This is expected during the first few frames after restart
            if "Physics Simulation View is not created yet" in str(e):
                # Physics not ready yet, just wait
                db.outputs.Integer = int(np.clip(width_mm, 0.0, 110.0))
            else:
                db.log_warning(f"Action failed: {e}")
                db.outputs.Integer = 0
        
    except Exception as e:
        db.log_error(f"Compute error: {e}")
        db.outputs.Integer = 0

def cleanup(db):
    global gripper_view
    db.log_info("Cleaning up gripper")
    gripper_view = None'''
        
        safe_set(f"{graph_path}/script.inputs:script", script_content, "Python script")
        
        # Configure script_node for reading gripper state
        script_node_content = '''from omni.isaac.core.articulations import ArticulationView
import numpy as np
import omni.timeline

gripper_view = None
last_sim_frame = -1
physics_ready = False

def setup(db):
    global gripper_view, physics_ready
    physics_ready = False
    try:
        gripper_view = ArticulationView(prim_paths_expr="/World/RG2_Gripper", name="gripper")
        gripper_view.initialize()
        db.log_info("Gripper initialized successfully")
    except Exception as e:
        db.log_error(f"Gripper setup failed: {e}")
        gripper_view = None

def compute(db):
    global gripper_view, last_sim_frame, physics_ready
    
    try:
        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_stopped():
            return
        
        current_frame = timeline.get_current_time() * timeline.get_time_codes_per_seconds()
        
        # Handle simulation restart
        if current_frame < last_sim_frame or last_sim_frame == -1:
            physics_ready = False
            try:
                gripper_view = ArticulationView(prim_paths_expr="/World/RG2_Gripper", name="gripper")
                gripper_view.initialize()
            except Exception as e:
                db.log_error(f"Gripper reinitialization failed: {e}")
                gripper_view = None
        
        last_sim_frame = current_frame
        
        if gripper_view is None:
            db.outputs.gripper_width = 0.0
            return
        
        # Read actual gripper state every frame
        actual_width_mm = 0.0
        
        try:
            joint_positions = gripper_view.get_joint_positions()
            
            if joint_positions is not None and len(joint_positions) > 0 and joint_positions.shape[1] >= 2:
                physics_ready = True
                
                # Get actual angle
                actual_angle = np.mean(joint_positions[0, :2])
                
                # Convert to width
                actual_ratio = (actual_angle + np.pi/4) / (np.pi/4 + np.pi/6)
                actual_width_mm = actual_ratio * 110.0
                actual_width_mm = np.clip(actual_width_mm, 0.0, 110.0)
                
        except Exception as e:
            pass  # Ignore during initialization
        
        # Always output actual width
        db.outputs.gripper_width = float(actual_width_mm)
        
    except Exception as e:
        db.log_error(f"Compute error: {e}")
        db.outputs.gripper_width = 0.0

def cleanup(db):
    global gripper_view, physics_ready
    gripper_view = None
    physics_ready = False'''
        
        safe_set(f"{graph_path}/script_node.inputs:usePath", False, "Use inline script for script_node")
        safe_set(f"{graph_path}/script_node.inputs:script", script_node_content, "Python script for script_node")
        
        # Configure ROS2 Publisher
        safe_set(f"{graph_path}/ros2_publisher.inputs:messageName", "Float64", "ROS2 message type")
        safe_set(f"{graph_path}/ros2_publisher.inputs:messagePackage", "std_msgs", "ROS2 package")
        safe_set(f"{graph_path}/ros2_publisher.inputs:topicName", "gripper_width_sim", "ROS2 topic")
        
        # Create input attribute on publisher and connect it using OmniGraph API
        try:
            stage = omni.usd.get_context().get_stage()
            publisher_prim = stage.GetPrimAtPath(f"{graph_path}/ros2_publisher")
            if publisher_prim.IsValid():
                # Check if attribute already exists
                existing_attr = publisher_prim.GetAttribute("inputs:data")
                if not existing_attr.IsValid():
                    data_attr = publisher_prim.CreateAttribute("inputs:data", Sdf.ValueTypeNames.Double, custom=True)
                    print("Created inputs:data attribute on ros2_publisher")
                else:
                    print("inputs:data attribute already exists on ros2_publisher")
                
                # Connect script_node output to publisher input using OmniGraph API
                og.Controller.connect(
                    f"{graph_path}/script_node.outputs:gripper_width",
                    f"{graph_path}/ros2_publisher.inputs:data"
                )
                print("Connected script_node.outputs:gripper_width to ros2_publisher.inputs:data")
            else:
                print(f"Warning: Could not find publisher prim at {graph_path}/ros2_publisher")
        except Exception as e:
            print(f"Error creating publisher connection: {e}")
        
        print("Creating connections...")
        # Connections for command handling (subscriber -> script)
        connections = [
            (f"{graph_path}/tick.outputs:tick", f"{graph_path}/subscriber.inputs:execIn", "Tick to subscriber"),
            (f"{graph_path}/context.outputs:context", f"{graph_path}/subscriber.inputs:context", "Context to subscriber"),
            (f"{graph_path}/subscriber.outputs:data", f"{graph_path}/script.inputs:String", "ROS2 data to script"),
            (f"{graph_path}/subscriber.outputs:execOut", f"{graph_path}/script.inputs:execIn", "ROS2 exec to script"),
            # Connections for state reading and publishing (script_node -> publisher)
            (f"{graph_path}/tick.outputs:tick", f"{graph_path}/script_node.inputs:execIn", "Tick to script_node"),
            (f"{graph_path}/script_node.outputs:execOut", f"{graph_path}/ros2_publisher.inputs:execIn", "Script_node exec to publisher"),
            (f"{graph_path}/context.outputs:context", f"{graph_path}/ros2_publisher.inputs:context", "Context to publisher")
        ]
        
        success_count = 0
        for src, dst, desc in connections:
            try:
                og.Controller.edit(graph, {keys.CONNECT: [(src, dst)]})
                print(f"Connected {desc}")
                success_count += 1
            except Exception as e:
                print(f"Failed {desc}: {e}")
        
        print(f"Graph created with {success_count}/7 connections (plus gripper_width connection)!")
        print("Location: /World/Graphs/ActionGraph_RG2")
        print("\nTest commands for gripper control:")
        print("ros2 topic pub /gripper_command std_msgs/String 'data: \"open\"'")
        print("ros2 topic pub /gripper_command std_msgs/String 'data: \"close\"'")
        print("ros2 topic pub /gripper_command std_msgs/String 'data: \"1100\"'")
        print("ros2 topic pub /gripper_command std_msgs/String 'data: \"550\"'")
        print("\nMonitor gripper width:")
        print("ros2 topic echo /gripper_width_sim")

    def setup_force_publish_action_graph(self):
        """Setup force publish action graph for ROS2 publishing gripper forces"""
        print("Setting up Force Publish Action Graph...")
        
        # Create the action graph
        graph_path = "/World/Graphs/ActionGraph_RG2_ForcePublish"
        keys = og.Controller.Keys

        # Delete existing graph if it exists
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(graph_path):
            stage.RemovePrim(graph_path)

        # Create nodes with initial values
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    (f"{graph_path}/tick", "omni.graph.action.OnPlaybackTick"),
                    (f"{graph_path}/context", "isaacsim.ros2.bridge.ROS2Context"),
                    (f"{graph_path}/script", "omni.graph.scriptnode.ScriptNode"),
                    (f"{graph_path}/publisher", "isaacsim.ros2.bridge.ROS2Publisher")
                ],
                keys.SET_VALUES: [
                    (f"{graph_path}/script.inputs:usePath", False),
                    (f"{graph_path}/publisher.inputs:messageName", "Float64"),
                    (f"{graph_path}/publisher.inputs:messagePackage", "std_msgs"),
                    (f"{graph_path}/publisher.inputs:topicName", "gripper_force"),
                ],
                keys.CONNECT: [
                    (f"{graph_path}/tick.outputs:tick", f"{graph_path}/script.inputs:execIn"),
                    (f"{graph_path}/script.outputs:execOut", f"{graph_path}/publisher.inputs:execIn"),
                    (f"{graph_path}/context.outputs:context", f"{graph_path}/publisher.inputs:context"),
                ]
            }
        )

        # Script content
        script_content = '''from omni.isaac.core.articulations import ArticulationView
import numpy as np
import omni.timeline

_gripper_view = None

def setup(db):
    global _gripper_view
    try:
        _gripper_view = ArticulationView(prim_paths_expr="/World/RG2_Gripper", name="gripper_force")
        _gripper_view.initialize()
        db.log_info("[FORCE] Force publisher initialized")
        print("[FORCE] Publisher ready")
    except Exception as e:
        db.log_error(f"[FORCE] Setup failed: {e}")
        print(f"[FORCE ERROR] {e}")

def compute(db):
    global _gripper_view
    
    try:
        if _gripper_view is None:
            db.outputs.max_force = 0.0
            return
        
        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_stopped():
            db.outputs.max_force = 0.0
            return
        
        try:
            forces = _gripper_view.get_measured_joint_forces()
            
            if forces is not None:
                forces_flat = np.asarray(forces).flatten()
                if len(forces_flat) > 0:
                    max_force = float(np.max(np.abs(forces_flat)))
                    db.outputs.max_force = max_force
                else:
                    db.outputs.max_force = 0.0
            else:
                db.outputs.max_force = 0.0
        
        except Exception as e:
            db.outputs.max_force = 0.0
    
    except Exception as e:
        db.outputs.max_force = 0.0

def cleanup(db):
    global _gripper_view
    try:
        _gripper_view = None
    except:
        pass'''

        # Set script FIRST
        og.Controller.set(og.Controller.attribute(f"{graph_path}/script.inputs:script"), script_content)

        # Now create output attribute on script node using OmniGraph API
        script_node = og.Controller.node(f"{graph_path}/script")
        og.Controller.create_attribute(
            script_node, 
            "max_force", 
            og.Type(og.BaseDataType.DOUBLE),
            attr_port=og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT
        )

        print("Created outputs:max_force attribute on script node")

        # Create input attribute on publisher and connect it using OmniGraph API
        publisher_prim = stage.GetPrimAtPath(f"{graph_path}/publisher")
        data_attr = publisher_prim.CreateAttribute("inputs:data", Sdf.ValueTypeNames.Double, custom=True)

        # Connect script output to publisher input using OmniGraph API
        og.Controller.connect(
            f"{graph_path}/script.outputs:max_force",
            f"{graph_path}/publisher.inputs:data"
        )

        print(f"RG2 Force Publish action graph created at {graph_path}")
        print("Publishing to topic: /gripper_force")

    def import_rg2_gripper(self):
        from omni.isaac.core.utils.stage import add_reference_to_stage
        rg2_usd_path = "omniverse://localhost/Library/RG2.usd"
        add_reference_to_stage(rg2_usd_path, "/World/RG2_Gripper")
        print("RG2 Gripper imported at /World/RG2_Gripper")

    def attach_rg2_to_ur5e(self):
        import omni.usd
        from pxr import Usd, Sdf, UsdGeom, Gf
        import math

        stage = omni.usd.get_context().get_stage()
        ur5e_gripper_path = "/World/UR5e/Gripper"
        rg2_path = "/World/RG2_Gripper"
        joint_path = "/World/UR5e/joints/robot_gripper_joint"
        rg2_base_link = "/World/RG2_Gripper/onrobot_rg2_base_link"

        ur5e_prim = stage.GetPrimAtPath(ur5e_gripper_path)
        rg2_prim = stage.GetPrimAtPath(rg2_path)
        joint_prim = stage.GetPrimAtPath(joint_path)

        if not ur5e_prim or not rg2_prim:
            print("Error: UR5e or RG2 gripper prim not found.")
            return

        # Copy transforms from UR5e gripper to RG2
        translate_attr = ur5e_prim.GetAttribute("xformOp:translate")
        orient_attr = ur5e_prim.GetAttribute("xformOp:orient")

        if translate_attr.IsValid() and orient_attr.IsValid():
            rg2_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3).Set(translate_attr.Get())
            rg2_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd).Set(orient_attr.Get())

        print("Setting RG2 gripper orientation with 180° Z offset and rotated position...")

        from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
        import numpy as np

        if rg2_prim.IsValid():
            # === 1. Apply rotated position ===
            original_pos = translate_attr.Get()
            x, y, z = original_pos[0], original_pos[1], original_pos[2]
            rotated_pos = Gf.Vec3d(-x, -y, z)

            # === 2. Apply combined orientation ===
            fixed_quat = Gf.Quatd(0.70711, Gf.Vec3d(-0.70711, 0.0, 0.0))
            offset_rpy_deg = np.array([0.0, 0.0, 180.0])
            offset_rpy_rad = np.deg2rad(offset_rpy_deg)
            offset_quat_arr = euler_angles_to_quats(offset_rpy_rad)
            offset_quat = Gf.Quatd(offset_quat_arr[0], offset_quat_arr[1], offset_quat_arr[2], offset_quat_arr[3])
            final_quat = offset_quat * fixed_quat

            # === 3. Apply to prim ===
            xform = UsdGeom.Xform(rg2_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(rotated_pos)
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(final_quat)

            print(f"RG2 position rotated to: {rotated_pos}")
            print("RG2 orientation set with fixed+180°Z rotation.")
        else:
            print(f"Gripper not found at {rg2_path}")


        # Create or update the physics joint
        if not joint_prim:
            joint_prim = stage.DefinePrim(joint_path, "PhysicsFixedJoint")

        joint_prim.CreateRelationship("physics:body1").SetTargets([Sdf.Path(rg2_base_link)])
        joint_prim.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(True)
        joint_prim.CreateAttribute("physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool).Set(True)

        # Set localRot0 and localRot1 for joint
        print("Setting joint rotation parameters...")
        if joint_prim.IsValid():
            def euler_to_quatf(x_deg, y_deg, z_deg):
                """Convert Euler angles (XYZ order, degrees) to Gf.Quatf"""
                rx = Gf.Quatf(math.cos(math.radians(x_deg) / 2), Gf.Vec3f(1, 0, 0) * math.sin(math.radians(x_deg) / 2))
                ry = Gf.Quatf(math.cos(math.radians(y_deg) / 2), Gf.Vec3f(0, 1, 0) * math.sin(math.radians(y_deg) / 2))
                rz = Gf.Quatf(math.cos(math.radians(z_deg) / 2), Gf.Vec3f(0, 0, 1) * math.sin(math.radians(z_deg) / 2))
                return rx * ry * rz  # Apply in XYZ order

            # Set the rotation quaternions for proper joint alignment
            quat0 = euler_to_quatf(-90, 0, -90)
            quat1 = euler_to_quatf(-180, 90, 0)
            
            joint_prim.CreateAttribute("physics:localRot0", Sdf.ValueTypeNames.Quatf, custom=True).Set(quat0)
            joint_prim.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf, custom=True).Set(quat1)
            print(" Set physics:localRot0 and localRot1 for robot_gripper_joint.")
        else:
            print(f" Joint not found at {joint_path}")

        print("RG2 successfully attached to UR5e with proper orientation and joint configuration.")

    def import_realsense_camera(self):
        """Import Intel RealSense D455 camera"""
        usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/5.0/Isaac/Sensors/Intel/RealSense/rsd455.usd"
        filename = os.path.splitext(os.path.basename(usd_path))[0]
        prim_path = f"/World/{filename}"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        print(f"Prim imported at {prim_path}")

    def attach_camera_to_ur5e(self):
        """Attach RealSense camera to UR5e wrist"""
        import omni.kit.commands
        from pxr import Gf
        
        # Move the prim
        omni.kit.commands.execute('MovePrim',
                                 path_from="/World/rsd455",
                                 path_to="/World/UR5e/wrist_3_link/rsd455")
        
        # Set transform properties
        omni.kit.commands.execute('ChangeProperty',
                                 prop_path="/World/UR5e/wrist_3_link/rsd455.xformOp:translate",
                                 value=Gf.Vec3d(-0.012, -0.055, 0.1),
                                 prev=None)
        omni.kit.commands.execute('ChangeProperty',
                                 prop_path="/World/UR5e/wrist_3_link/rsd455.xformOp:rotateZYX",
                                 value=Gf.Vec3d(-90, -180, 270),
                                 prev=None)
        
        print("RealSense camera attached to UR5e wrist_3_link")

    def setup_camera_action_graph(self):
        """Create ActionGraph for camera ROS2 publishing"""
        # Configuration
        CAMERA_PRIM = "/World/UR5e/wrist_3_link/rsd455/RSD455/Camera_OmniVision_OV9782_Color" 
        IMAGE_WIDTH = 1280
        IMAGE_HEIGHT = 720
        ROS2_TOPIC = "intel_camera_rgb_sim"
        
        graph_path = "/World/Graphs/ActionGraph_Camera"  # Different name to avoid conflicts
        print(f"Creating ActionGraph: {graph_path}")
        print(f"Camera: {CAMERA_PRIM}")
        print(f"Resolution: {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
        print(f"ROS2 Topic: {ROS2_TOPIC}")
        
        # Create ActionGraph
        try:
            og.Controller.create_graph(graph_path)
            print(f"Created ActionGraph at {graph_path}")
        except Exception:
            print(f"ActionGraph already exists at {graph_path}")
        
        # Create nodes
        nodes = [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("isaac_run_one_simulation_frame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("isaac_create_render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ros2_camera_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ]
        
        print("\nCreating nodes...")
        for node_name, node_type in nodes:
            try:
                node_path = f"{graph_path}/{node_name}"
                og.Controller.create_node(node_path, node_type)
                print(f"Created {node_name}")
            except Exception as e:
                print(f"Node {node_name} already exists")
        
        # Set node attributes
        print("\nConfiguring nodes...")
        # Configure render product
        try:
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:cameraPrim").set([CAMERA_PRIM])
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:width").set(IMAGE_WIDTH)
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:height").set(IMAGE_HEIGHT)
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:enabled").set(True)
            print(f"Configured render product: {CAMERA_PRIM} @ {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
        except Exception as e:
            print(f"Error configuring render product: {e}")
        
        # Configure ROS2 camera helper
        try:
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:topicName").set(ROS2_TOPIC)
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:frameId").set("camera_link")
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:type").set("rgb")
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:enabled").set(True)
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:queueSize").set(10)
            print(f"Configured ROS2 helper: topic={ROS2_TOPIC}")
        except Exception as e:
            print(f"Error configuring ROS2 helper: {e}")
        
        # Create connections
        print("\nConnecting nodes...")
        connections = [
            ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
            ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
            ("isaac_create_render_product.outputs:execOut", "ros2_camera_helper.inputs:execIn"),
            ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_helper.inputs:renderProductPath"),
            ("ros2_context.outputs:context", "ros2_camera_helper.inputs:context"),
        ]
        
        for source, target in connections:
            try:
                og.Controller.connect(f"{graph_path}/{source}", f"{graph_path}/{target}")
                print(f"Connected {source.split('.')[0]} -> {target.split('.')[0]}")
            except Exception as e:
                print(f"Failed to connect {source} -> {target}: {e}")
        
        print("\nCamera ActionGraph created successfully!")
        print(f"Test with: ros2 topic echo /{ROS2_TOPIC}")

    def create_additional_camera(self):
        """Create additional camera based on selected view type"""
        import omni.usd
        from pxr import Gf, UsdGeom
        import numpy as np

        def set_camera_pose(prim_path: str, position_xyz, quat_xyzw):
            """Apply translation and quaternion orientation (x, y, z, w) to a camera prim."""
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                raise RuntimeError(f"Camera prim '{prim_path}' not found.")
            xform = UsdGeom.Xform(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(*position_xyz))
            quat = Gf.Quatd(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)

        def configure_camera_properties(camera_prim, width, height):
            """Configure camera properties based on resolution"""
            # Calculate focal length and aperture based on resolution
            # Using standard 35mm film equivalent calculations
            # Horizontal aperture in mm (standard 35mm film is 36mm wide)
            horizontal_aperture_mm = 36.0
            # Vertical aperture calculated from aspect ratio
            aspect_ratio = height / width
            vertical_aperture_mm = horizontal_aperture_mm * aspect_ratio
            
            # Focal length (50mm is a standard "normal" lens)
            focal_length_mm = 50.0
            
            # Convert mm to USD units (USD uses cm, but aperture is in mm in USD)
            camera = UsdGeom.Camera(camera_prim)
            camera.CreateHorizontalApertureAttr().Set(horizontal_aperture_mm)
            camera.CreateVerticalApertureAttr().Set(vertical_aperture_mm)
            camera.CreateFocalLengthAttr().Set(focal_length_mm)
            camera.CreateProjectionAttr().Set("perspective")
            camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000.0))

        # Check which camera type is selected
        is_exocentric = self._exocentric_checkbox.model.get_value_as_bool()
        is_custom = self._custom_checkbox.model.get_value_as_bool()

        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("Error: No stage found")
            return

        if is_exocentric:
            prim_path = "/World/exocentric_camera"
            position = (1.5, -1.5, 0.85)
            quat_xyzw = (0.52787, 0.24907, 0.32102, 0.74583)  # x, y, z, w
            resolution = (1280, 720)

            # Create camera prim using UsdGeom.Camera
            camera_prim = UsdGeom.Camera.Define(stage, prim_path)
            if not camera_prim:
                print(f"Error: Failed to create camera at {prim_path}")
                return
            
            # Configure camera properties
            configure_camera_properties(camera_prim.GetPrim(), resolution[0], resolution[1])
            
            # Set camera pose
            set_camera_pose(prim_path, position, quat_xyzw)
            print(f"Exocentric camera created at {prim_path} with resolution {resolution[0]}x{resolution[1]}")

        if is_custom:
            prim_path = "/World/custom_camera"
            position = (0.0, 4.0, 0.5)
            quat_xyzw = (0.0, 0.67559, 0.73728, 0.0)  # x, y, z, w
            resolution = (640, 480)

            # Create camera prim using UsdGeom.Camera
            camera_prim = UsdGeom.Camera.Define(stage, prim_path)
            if not camera_prim:
                print(f"Error: Failed to create camera at {prim_path}")
                return
            
            # Configure camera properties
            configure_camera_properties(camera_prim.GetPrim(), resolution[0], resolution[1])
            
            # Set camera pose
            set_camera_pose(prim_path, position, quat_xyzw)
            print(f"Custom camera created at {prim_path} with resolution {resolution[0]}x{resolution[1]}")
            # Note: Motion vectors would need to be added via render settings or post-processing

    def create_additional_camera_actiongraph(self):
        """Create ActionGraph for additional camera ROS2 publishing"""
        # Check which cameras exist and create action graphs accordingly
        is_exocentric = self._exocentric_checkbox.model.get_value_as_bool()
        is_custom = self._custom_checkbox.model.get_value_as_bool()

        if is_exocentric:
            # TODO: Copy other camera properties into exocentric camera (e.g., from Intel camera setup)
            self._create_camera_actiongraph(
                "/World/exocentric_camera", 
                1280, 720, 
                "exocentric_camera", 
                "ExocentricCamera"
            )
        
        if is_custom:
            self._create_camera_actiongraph(
                "/World/custom_camera", 
                640, 480, 
                "custom_camera", 
                "CustomCamera"
            )

    def _create_camera_actiongraph(self, camera_prim, width, height, topic, graph_suffix):
        """Helper method to create camera ActionGraph"""
        graph_path = f"/World/Graphs/ActionGraph_{graph_suffix}"
        print(f"Creating ActionGraph: {graph_path}")
        print(f"Camera: {camera_prim}")
        print(f"Resolution: {width}x{height}")
        print(f"ROS2 Topic: {topic}")
        
        # Create ActionGraph
        try:
            og.Controller.create_graph(graph_path)
            print(f"Created ActionGraph at {graph_path}")
        except Exception:
            print(f"ActionGraph already exists at {graph_path}")
        
        # Create nodes
        nodes = [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("isaac_run_one_simulation_frame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("isaac_create_render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ros2_camera_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ]
        
        print("\nCreating nodes...")
        for node_name, node_type in nodes:
            try:
                node_path = f"{graph_path}/{node_name}"
                og.Controller.create_node(node_path, node_type)
                print(f"Created {node_name}")
            except Exception as e:
                print(f"Node {node_name} already exists")
        
        # Set node attributes
        print("\nConfiguring nodes...")
        
        # Configure render product
        try:
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:cameraPrim").set([camera_prim])
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:width").set(width)
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:height").set(height)
            og.Controller.attribute(f"{graph_path}/isaac_create_render_product.inputs:enabled").set(True)
            print(f"Configured render product: {camera_prim} @ {width}x{height}")
        except Exception as e:
            print(f"Error configuring render product: {e}")
        
        # Configure ROS2 camera helper
        try:
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:topicName").set(topic)
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:frameId").set("camera_link")
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:type").set("rgb")
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:enabled").set(True)
            og.Controller.attribute(f"{graph_path}/ros2_camera_helper.inputs:queueSize").set(10)
            print(f"Configured ROS2 helper: topic={topic}")
        except Exception as e:
            print(f"Error configuring ROS2 helper: {e}")
        
        # Create connections
        print("\nConnecting nodes...")
        connections = [
            ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
            ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
            ("isaac_create_render_product.outputs:execOut", "ros2_camera_helper.inputs:execIn"),
            ("isaac_create_render_product.outputs:renderProductPath", "ros2_camera_helper.inputs:renderProductPath"),
            ("ros2_context.outputs:context", "ros2_camera_helper.inputs:context"),
        ]
        
        for source, target in connections:
            try:
                og.Controller.connect(f"{graph_path}/{source}", f"{graph_path}/{target}")
                print(f"Connected {source.split('.')[0]} -> {target.split('.')[0]}")
            except Exception as e:
                print(f"Failed to connect {source} -> {target}: {e}")
        
        print(f"\n{graph_suffix} ActionGraph created successfully!")
        print(f"Test with: ros2 topic echo /{topic}")

    def _sample_non_overlapping_objects(
        self,
        num_objects,
        x_range=(-0.2, 0.6),
        y_range=(-0.6, -0.3),
        min_sep=0.2,
        yaw_range=(0.0, 135.0),
        z_values=None,
        fixed_positions=None,
        max_attempts=10_000,
    ):
        """
        Sample non-overlapping object poses in world frame.
        
        Args:
            num_objects: Number of objects to place
            x_range: X position range in world frame (default: -0.5 to 0.5)
            y_range: Y position range in world frame (default: -0.5 to 0.0)
            min_sep: Minimum separation between objects
            yaw_range: Yaw rotation range in degrees
            z_values: List of Z values for each object (preserves current Z)
            fixed_positions: List of fixed positions (e.g., base objects) to avoid
            max_attempts: Maximum placement attempts
        """
        poses, attempts = [], 0
        # Convert fixed positions to numpy arrays for distance checking
        fixed_xy = []
        if fixed_positions:
            for pos in fixed_positions:
                if isinstance(pos, (list, tuple, np.ndarray)):
                    fixed_xy.append(np.array(pos[:2]))  # Only X, Y
                elif hasattr(pos, '__getitem__'):
                    fixed_xy.append(np.array([pos[0], pos[1]]))
        
        while len(poses) < num_objects and attempts < max_attempts:
            attempts += 1
            candidate_xy = np.array([
                np.random.uniform(*x_range),
                np.random.uniform(*y_range)
            ])
            # Check separation against both existing poses AND fixed positions
            valid = True
            # Check against existing randomized poses
            if any(np.linalg.norm(candidate_xy - p["position"][:2]) < min_sep for p in poses):
                valid = False
            # Check against fixed positions (base objects)
            if valid and fixed_xy:
                if any(np.linalg.norm(candidate_xy - fixed) < min_sep for fixed in fixed_xy):
                    valid = False
            
            if valid:
                yaw_deg = np.random.uniform(*yaw_range)
                # Use provided Z value or default
                z = z_values[len(poses)] if z_values and len(poses) < len(z_values) else 0.0495
                poses.append({
                    "position": np.array([candidate_xy[0], candidate_xy[1], z]),
                    "yaw_deg": yaw_deg
                })
        if len(poses) < num_objects:
            raise RuntimeError("Could not place all objects without violating min_sep; reduce density.")
        return poses

    def _set_obj_prim_pose(self, prim_path, position, quat_wxyz):
        import omni.kit.commands
        # Handle both Gf.Vec3d and tuple/list positions
        if isinstance(position, Gf.Vec3d):
            pos_value = position
        else:
            pos_value = Gf.Vec3d(*position)
        
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=f"{prim_path}.xformOp:translate",
            value=pos_value,
            prev=None,
        )
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=f"{prim_path}.xformOp:orient",
            value=Gf.Quatf(*quat_wxyz),
            prev=None,
        )

    def randomize_object_poses(self, folder_path="/World/Objects"):
        """
        Randomize object poses in world frame, then transform to local child prim frame.
        Randomizes position of /World/Objects/{object_name} in world coordinates,
        then applies the appropriate local transform to the child prim.
        """
        stage = omni.usd.get_context().get_stage()
        objects_root = stage.GetPrimAtPath(folder_path)
        if not objects_root.IsValid():
            print(f"Warning: {folder_path} does not exist")
            return

        # Collect object info: parent path, child path, parent's current world position, and child's current Z
        # Also collect base object positions to respect during randomization
        object_info = []
        base_positions = []  # Store base object world positions
        
        for child in objects_root.GetChildren():
            obj = child.GetName()
            parent_path = f"{folder_path}/{obj}"
            child_path = f"{folder_path}/{obj}/{obj}/{obj}"
            
            parent_prim = stage.GetPrimAtPath(parent_path)
            child_prim = stage.GetPrimAtPath(child_path)
            
            if not parent_prim.IsValid() or not child_prim.IsValid():
                continue
            
            # Get child's current world transform
            child_xform = UsdGeom.Xformable(child_prim)
            child_world_transform = child_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            child_world_pos = child_world_transform.ExtractTranslation()
            
            # Check if this is a base object (matches pattern base{id})
            if re.match(r'^base\d+$', obj):
                # Store base position for separation checking
                base_positions.append(child_world_pos)
                print(f"Skipping {obj} (matches base{{id}} pattern) at world pos: ({child_world_pos[0]:.3f}, {child_world_pos[1]:.3f}, {child_world_pos[2]:.3f})")
                continue
            
            # Get parent's current world transform and position
            parent_xform = UsdGeom.Xformable(parent_prim)
            parent_world_transform = parent_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            parent_world_pos = parent_world_transform.ExtractTranslation()
            
            object_info.append({
                "parent_path": parent_path,
                "child_path": child_path,
                "parent_world_pos": parent_world_pos,
                "parent_world_transform": parent_world_transform,
                "current_z": child_world_pos[2]
            })
            print(f"Found: {child_path}")
            print(f"  Parent world pos: ({parent_world_pos[0]:.3f}, {parent_world_pos[1]:.3f}, {parent_world_pos[2]:.3f})")
            print(f"  Child world pos: ({child_world_pos[0]:.3f}, {child_world_pos[1]:.3f}, {child_world_pos[2]:.3f})")

        if not object_info:
            print("No valid objects found")
            return

        # Get Z values for preserving current heights
        z_values = [info["current_z"] for info in object_info]
        
        # Randomize positions in world frame
        # Pass base positions so randomization respects them for minimum separation
        poses = self._sample_non_overlapping_objects(
            num_objects=len(object_info),
            z_values=z_values,
            fixed_positions=base_positions
        )
        
        if base_positions:
            print(f"Randomization will respect {len(base_positions)} base object positions for minimum separation")

        # Apply randomized poses
        for obj_info, pose in zip(object_info, poses):
            parent_path = obj_info["parent_path"]
            child_path = obj_info["child_path"]
            parent_world_transform = obj_info["parent_world_transform"]
            parent_world_pos = obj_info["parent_world_pos"]
            
            # Target world position (randomized)
            target_world_pos = Gf.Vec3d(pose["position"][0], pose["position"][1], pose["position"][2])
            
            # Calculate local position relative to parent
            # The parent's world transform already accounts for its current position
            # Local = Parent^-1 * Target_World
            # This correctly transforms the target world position to local coordinates
            # accounting for where the parent currently is
            parent_inverse = parent_world_transform.GetInverse()
            local_pos = parent_inverse.Transform(target_world_pos)
            
            # Convert yaw to quaternion
            quat_xyzw = R.from_euler("xyz", [0.0, 0.0, pose["yaw_deg"]], degrees=True).as_quat()
            quat_wxyz = np.roll(quat_xyzw, 1)
            
            # Apply local transform to child prim (parent prim stays unchanged)
            self._set_obj_prim_pose(child_path, local_pos, quat_wxyz)
            
            print(f"Randomized {child_path}:")
            print(f"  Parent world pos (unchanged): ({parent_world_pos[0]:.3f}, {parent_world_pos[1]:.3f}, {parent_world_pos[2]:.3f})")
            print(f"  Target world pos: ({target_world_pos[0]:.3f}, {target_world_pos[1]:.3f}, {target_world_pos[2]:.3f})")
            print(f"  Child local pos: ({local_pos[0]:.3f}, {local_pos[1]:.3f}, {local_pos[2]:.3f})")

    def add_objects(self):
        """Import all objects from the specified folder into the scene"""
        # Get the folder path from the UI
        folder_path = self._objects_path_field.model.get_value_as_string()
        if not folder_path:
            print("Error: No folder path specified")
            return
        
        print(f"Adding objects from folder: {folder_path}")
        
        # Get the current stage and selection
        stage = omni.usd.get_context().get_stage()
        selection = omni.usd.get_context().get_selection()
        selected_paths = selection.get_selected_prim_paths()

        # Use current selection or fallback to /World/Objects
        if selected_paths:
            target_path = selected_paths[0]
        else:
            target_path = "/World/Objects"
            from pxr import UsdGeom
            if not stage.GetPrimAtPath(target_path):
                UsdGeom.Xform.Define(stage, target_path)

        # List all files in the folder
        result, entries = omni.client.list(folder_path)

        if result == omni.client.Result.OK:
            # Filter for USD files
            usd_files = [entry.relative_path for entry in entries 
                         if entry.relative_path.endswith(('.usd', '.usda', '.usdc'))]
            
            print(f"Found {len(usd_files)} USD files")
            
            # Import each USD file with positioning
            for i, usd_file in enumerate(usd_files):
                usd_file_path = folder_path + usd_file
                
                # Create a child prim under the selected path
                base_name = os.path.splitext(usd_file)[0]
                prim_path = f"{target_path}/{base_name}"
                
                # Create a reference to the USD file
                prim = stage.DefinePrim(prim_path)
                references = prim.GetReferences()
                references.AddReference(usd_file_path)
                
                # Position objects: first at center, then alternating +X and -X
                if i == 0:
                    # First object at center (0, 0, 0)
                    x_position = 0.0
                else:
                    # Calculate position: alternating +X and -X, using configurable spacing
                    if i % 2 == 1:  # Odd indices: +X direction
                        x_position = self._object_spacing * ((i + 1) // 2)
                    else:  # Even indices: -X direction
                        x_position = -self._object_spacing * (i // 2)
                
                # Apply position to the parent prim
                from pxr import UsdGeom, Gf
                xform = UsdGeom.Xform(prim)
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(x_position, self._y_offset, self._z_offset))
                
                # Rename Body1 to match the object name
                def rename_body1_to_object_name(stage, prim_path, object_name):
                    """Recursively search for Body1 and rename it to object_name"""
                    prim = stage.GetPrimAtPath(prim_path)
                    if not prim:
                        return False
                    
                    # Search for Body1 in the hierarchy
                    for child in prim.GetAllChildren():
                        if child.GetName() == "Body1":
                            # Found Body1, rename it
                            new_path = child.GetPath().GetParentPath().AppendChild(object_name)
                            import omni.kit.commands
                            omni.kit.commands.execute('MovePrim',
                                path_from=child.GetPath(),
                                path_to=new_path)
                            print(f"Renamed Body1 to {object_name} at {new_path}")
                            return True
                        # Recursively search in children
                        if rename_body1_to_object_name(stage, child.GetPath(), object_name):
                            return True
                    return False
                
                # Try to rename Body1 to the object name
                rename_body1_to_object_name(stage, prim_path, base_name)
                
                print(f"Added {usd_file_path} to {prim_path} at position ({x_position}, {self._y_offset}, {self._z_offset})")
        else:
            print(f"Failed to list folder: {folder_path}")

    def create_pose_publisher(self):
        """Create action graph for publishing object poses to ROS2"""
        import omni.kit.commands
        from pxr import Sdf, Usd, UsdGeom
        import omni.usd
        import omni.graph.core as og

        def get_objects_in_folder(stage, folder_path="/World/Objects"):
            """
            Scan the Objects folder and find all prims following the pattern:
            /World/Objects/{object_name}/{object_name}/{object_name}
            
            Args:
                stage: USD Stage
                folder_path: Path to the Objects folder
            
            Returns:
                List of paths to object prims
            """
            body_paths = []
            
            # Get the Objects prim
            objects_prim = stage.GetPrimAtPath(folder_path)
            
            if not objects_prim.IsValid():
                print(f"Warning: {folder_path} does not exist")
                return body_paths
            
            # Iterate through children (e.g., fork_orange, base, fork_yellow, line_brown)
            for child in objects_prim.GetChildren():
                object_name = child.GetName()
                
                # Look for the nested structure: {object_name}/{object_name}/object_name{}
                nested_path = f"{folder_path}/{object_name}/{object_name}/{object_name}"
                nested_prim = stage.GetPrimAtPath(nested_path)
                
                if nested_prim.IsValid():
                    body_paths.append(nested_path)
                    print(f"Found: {nested_path}")
            
            return body_paths

        def create_action_graph_with_transforms(target_prims, parent_prim="/World", topic_name="objects_poses_sim"):
            """
            Create an action graph with OnPlaybackTick and ROS2PublishTransformTree nodes
            
            Args:
                target_prims: List of prim paths to publish transforms for
                parent_prim: Parent prim for the transform tree
                topic_name: ROS2 topic name
            """
            
            graph_path = "/World/Graphs/ActionGraph_objects_poses"
            
            # Create the action graph using OmniGraph API
            keys = og.Controller.Keys
            
            (graph, nodes, _, _) = og.Controller.edit(
                {
                    "graph_path": graph_path,
                    "evaluator_name": "execution",
                },
                {
                    keys.CREATE_NODES: [
                        ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                        ("ros2_publish_transform_tree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    keys.CONNECT: [
                        ("on_playback_tick.outputs:tick", "ros2_publish_transform_tree.inputs:execIn"),
                    ],
                    keys.SET_VALUES: [
                        ("ros2_publish_transform_tree.inputs:topicName", topic_name),
                    ],
                },
            )
            
            # Get the stage to set relationships
            stage = omni.usd.get_context().get_stage()
            
            # Get the ROS2 node prim
            ros2_node_path = f"{graph_path}/ros2_publish_transform_tree"
            ros2_prim = stage.GetPrimAtPath(ros2_node_path)
            
            if ros2_prim.IsValid():
                # Set parent prim as a relationship
                parent_rel = ros2_prim.GetRelationship("inputs:parentPrim")
                if not parent_rel:
                    parent_rel = ros2_prim.CreateRelationship("inputs:parentPrim", custom=True)
                parent_rel.SetTargets([Sdf.Path(parent_prim)])
                
                # Set target prims as a relationship
                target_rel = ros2_prim.GetRelationship("inputs:targetPrims")
                if not target_rel:
                    target_rel = ros2_prim.CreateRelationship("inputs:targetPrims", custom=True)
                target_paths = [Sdf.Path(path) for path in target_prims]
                target_rel.SetTargets(target_paths)
                
                print(f"Action graph created at {graph_path}")
                print(f"Publishing {len(target_prims)} objects to topic: {topic_name}")
            else:
                print(f"Error: Could not find ROS2 node at {ros2_node_path}")

        # Get the current USD stage
        stage = omni.usd.get_context().get_stage()
        
        if not stage:
            print("Error: No stage found")
            return
        
        # Get all Body1 prims from the Objects folder
        object_paths = get_objects_in_folder(stage, "/World/Objects")
        
        if not object_paths:
            print("No objects found in /World/Objects")
            return
        
        print(f"\nFound {len(object_paths)} objects:")
        for path in object_paths:
            print(f"  - {path}")
        
        # Create the action graph with all found objects
        create_action_graph_with_transforms(
            target_prims=object_paths,
            parent_prim="/World",
            topic_name="objects_poses_sim"
        )
        
        print("\nAction graph created successfully!")

    def on_shutdown(self):
        """Clean shutdown"""
        print("[DigitalTwin] Digital Twin shutdown")
        
        # Isaac Sim handles ROS2 shutdown automatically
        print("ROS2 bridge shutdown handled by Isaac Sim")
        
        # Destroy window
        if self._window:
            self._window.destroy()
            self._window = None