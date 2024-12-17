import omni.ext
import omni.ui as ui
import omni.graph.core as og
from omni.isaac.dynamic_control import _dynamic_control
import omni.timeline
import omni.usd
from pxr import Usd, PhysxSchema, Sdf

class Ros2JointStateSubscriberArticulation(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._window = ui.Window("ROS2 Joint State Subscriber & Articulation Controller", width=300, height=400)
        self._received_data = ""
        self._ros_thread = None
        self._is_listening = False
        self._ur5_articulation = None
        self._dynamic_control = _dynamic_control.acquire_dynamic_control_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        self._simulation_running = False

        with self._window.frame:
            with ui.VStack():
                ui.Label("ROS2 Joint State Subscriber & Articulation Controller", alignment=ui.Alignment.CENTER)
                self._data_field = ui.StringField()
                self._data_field.model.set_value("Data will appear here...")

                ui.Button("Create UR5e", clicked_fn=self._create_ur5e)
                
                # Add "Import Action Graph" button
                ui.Button("Import Action Graph", clicked_fn=self._create_action_graph)   

                # Text input field for ROS topic name
                ui.Label("Enter ROS Topic Name:")
                self._ros_topic_field = ui.StringField()
                self._ros_topic_field.model.set_value("/joint_states")

                # Button to listen to ROS topic
                ui.Button("Listen to Topic", clicked_fn=self._on_listen_to_topic_clicked)

                # Rename "Apply Joints" button to "Link Joints to Graph"
                self._apply_joints_button = ui.Button("Link Joints to Graph", clicked_fn=self._on_apply_joints_clicked)

                # Buttons to control simulation
                ui.Button("Play Simulation", clicked_fn=self._play_simulation)
                ui.Button("Stop Simulation", clicked_fn=self._stop_simulation)

    def _create_ur5e(self):
        stage = omni.usd.get_context().get_stage()
        ur5e_path = "/World/UR5e"
        if not stage.GetPrimAtPath(ur5e_path):
            from omni.isaac.core.utils.stage import add_reference_to_stage
            ur5e_usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
            add_reference_to_stage(ur5e_usd_path, ur5e_path)
            self._update_data_field("UR5e created at /World/UR5e")
        else:
            self._update_data_field("UR5e already exists.")

    def _create_action_graph(self):
        # Define OmniGraph Controller Keys
        keys = og.Controller.Keys

        # Step 1: Create the Action Graph
        graph_path = "/ActionGraph"
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {}
        )

        # Step 2: Define all node names and types
        nodes = {
            "ros2_context": ("ROS2 Context", "omni.isaac.ros2_bridge.ROS2Context"),
            "playback_tick": ("On Playback Tick", "omni.graph.action.OnPlaybackTick"),
            "read_sim_time": ("Isaac Read Simulation Time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            "ros2_publish_clock": ("ROS2 Publish Clock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            "ros2_subscribe_joint": ("ROS2 Subscribe Joint State", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
            "articulation_controller": ("Articulation Controller", "omni.isaac.core_nodes.IsaacArticulationController"),
        }

        # Step 3: Create the nodes
        created_nodes = {}
        for node_name, (node_label, node_type) in nodes.items():
            created_nodes[node_name] = og.Controller.create_node(
                f"{graph_path}/{node_name}", node_type
            )

        # Step 4: Connect nodes together
        connections = [
            (f"{graph_path}/playback_tick.outputs:tick", f"{graph_path}/ros2_subscribe_joint.inputs:execIn"),
            (f"{graph_path}/playback_tick.outputs:tick", f"{graph_path}/ros2_publish_clock.inputs:execIn"),
            (f"{graph_path}/playback_tick.outputs:tick", f"{graph_path}/articulation_controller.inputs:execIn"),
            (f"{graph_path}/read_sim_time.outputs:simulationTime", f"{graph_path}/ros2_publish_clock.inputs:timeStamp"),
            (f"{graph_path}/ros2_context.outputs:context", f"{graph_path}/ros2_publish_clock.inputs:context"),
            (f"{graph_path}/ros2_context.outputs:context", f"{graph_path}/ros2_subscribe_joint.inputs:context"),
        ]

        for src, dest in connections:
            try:
                og.Controller.connect(src, dest)
            except Exception as e:
                print(f"Failed to connect {src} -> {dest}: {str(e)}")

        # Step 5: Connect ROS2 Subscribe Joint State outputs to Articulation Controller
        joint_connections = [
            (f"{graph_path}/ros2_subscribe_joint.outputs:effortCommand", f"{graph_path}/articulation_controller.inputs:effortCommand"),
            (f"{graph_path}/ros2_subscribe_joint.outputs:jointNames", f"{graph_path}/articulation_controller.inputs:jointNames"),
            (f"{graph_path}/ros2_subscribe_joint.outputs:positionCommand", f"{graph_path}/articulation_controller.inputs:positionCommand"),
            (f"{graph_path}/ros2_subscribe_joint.outputs:velocityCommand", f"{graph_path}/articulation_controller.inputs:velocityCommand"),
        ]

        for src, dest in joint_connections:
            try:
                og.Controller.connect(src, dest)
            except Exception as e:
                print(f"Failed to connect {src} -> {dest}: {str(e)}")

        self._update_data_field("Action Graph created successfully.")

    def _on_listen_to_topic_clicked(self):
        topic_name = self._ros_topic_field.model.get_value_as_string()
        graph_path = "/ActionGraph"
        try:
            ros2_subscribe_path = f"{graph_path}/ros2_subscribe_joint"
            ros2_subscribe_attr = og.Controller.attribute(f"{ros2_subscribe_path}.inputs:topicName")
            ros2_subscribe_attr.set(topic_name)
            self._update_data_field(f"Set topicName for ros2_subscribe_joint to {topic_name}.")
        except Exception as e:
            self._update_data_field(f"Error setting topic name: {str(e)}")

    def _on_apply_joints_clicked(self):
        articulation_path = "/World/UR5e/base_link"
        articulation_controller_path = "/ActionGraph/articulation_controller"
        stage = omni.usd.get_context().get_stage()

        try:
            articulation_controller_prim = stage.GetPrimAtPath(articulation_controller_path)
            if articulation_controller_prim.IsValid():
                target_prim_rel = articulation_controller_prim.CreateRelationship("inputs:targetPrim")
                target_prim_rel.SetTargets([Sdf.Path(articulation_path)])
                self._update_data_field(f"Linked articulation_controller targetPrim to {articulation_path}.")
            else:
                self._update_data_field(f"Error: Articulation controller prim at {articulation_controller_path} not found!")

        except Exception as e:
            self._update_data_field(f"Error linking joints to graph: {str(e)}")

    def _play_simulation(self):
        self._timeline.play()
        self._simulation_running = True
        self._update_data_field("Simulation started.")

    def _stop_simulation(self):
        self._timeline.stop()
        self._simulation_running = False
        self._update_data_field("Simulation stopped.")

    def _update_data_field(self, data):
        if isinstance(data, str):
            self._data_field.model.set_value(data)

    def on_shutdown(self):
        if self._window:
            self._window.destroy()
        self._window = None
