import omni.ext
import omni.ui as ui
import asyncio
import numpy as np
import os
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.motion_generation import LulaKinematicsSolver, ArticulationKinematicsSolver
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.core.articulations import Articulation

class InversekinematiccontrolExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[InverseKinematicControl] inv kin ctrl startup")

        self._timeline = omni.timeline.get_timeline_interface()
        self._ur5e_view = None
        self._articulation = None
        self._gripper_view = None

        # Create the window UI
        self._window = ui.Window("UR5e Digital Twin", width=300, height=800)
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
                        ui.Button("Load UR5e", width=100, height=35, clicked_fn=lambda: asyncio.ensure_future(self.load_ur5e()))
                        ui.Button("Setup Action Graph", width=180, height=35, clicked_fn=lambda: asyncio.ensure_future(self.setup_action_graph()))

            with ui.CollapsableFrame(title="RG2 Gripper", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("RG2 Gripper Control", alignment=ui.Alignment.LEFT)
                    with ui.HStack(spacing=5):
                        ui.Button("Import RG2 Gripper", width=150, height=35, clicked_fn=self.import_rg2_gripper)
                        ui.Button("Attach Gripper to UR5e", width=180, height=35, clicked_fn=self.attach_rg2_to_ur5e)
                    with ui.HStack(spacing=5):
                        ui.Button("Open Gripper", width=150, height=35, clicked_fn=self.open_gripper)
                        ui.Button("Close Gripper", width=150, height=35, clicked_fn=self.close_gripper)


    async def load_scene(self):
        world = World()
        await world.initialize_simulation_context_async()
        world.scene.add_default_ground_plane()
        print("Scene loaded successfully.")

    async def load_ur5e(self):
        asset_path = "omniverse://localhost/Library/ur5e.usd"
        #asset_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")

        self._ur5e_view = ArticulationView(prim_paths_expr="/World/UR5e", name="ur5e_view")
        World.instance().scene.add(self._ur5e_view)
        await World.instance().reset_async()
        self._timeline.stop()

        self._articulation = Articulation("/World/UR5e")

        print("UR5e robot loaded successfully!")
        
    async def setup_action_graph(self):
        import omni.graph.core as og
        import omni.isaac.core.utils.stage as stage_utils
        from omni.isaac.core.utils.extensions import enable_extension

        print("Setting up ROS 2 Action Graph...")

        # Ensure extensions are enabled
        enable_extension("omni.isaac.ros2_bridge")
        enable_extension("omni.isaac.core_nodes")
        enable_extension("omni.graph.action")

        graph_path = "/ActionGraph"
        (graph, nodes, _, _) = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution"
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ros2_subscribe_joint_state", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("ros2_publish_clock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ("articulation_controller", "omni.isaac.core_nodes.IsaacArticulationController"),
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

        # Set targetPrim after creation
        og.Controller.edit(
            graph,
            {
                og.Controller.Keys.SET_VALUES: [
                    ("articulation_controller.inputs:targetPrim", ["/World/UR5e"]),
                ]
            }
        )

        print("ROS 2 Action Graph setup complete.")

    def import_rg2_gripper(self):
        from omni.isaac.core.utils.stage import add_reference_to_stage
        rg2_usd_path = "omniverse://localhost/Library/RG2.usd"
        add_reference_to_stage(rg2_usd_path, "/RG2_Gripper")
        print("RG2 Gripper imported at /RG2_Gripper")

    def attach_rg2_to_ur5e(self):
        import omni.usd
        from pxr import Usd, Sdf, UsdGeom, Gf
        import math

        stage = omni.usd.get_context().get_stage()
        ur5e_gripper_path = "/World/UR5e/Gripper"
        rg2_path = "/RG2_Gripper"
        joint_path = "/World/UR5e/joints/robot_gripper_joint"
        rg2_base_link = "/RG2_Gripper/onrobot_rg2_base_link"

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

        # --------- 1. Set orientation for RG2 Gripper ---------
        print("Setting RG2 gripper orientation...")
        if rg2_prim.IsValid():
            quat_attr = rg2_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd, custom=True)
            quat_attr.Set(Gf.Quatd(0.70711, Gf.Vec3d(-0.70711, 0.0, 0.0)))
            print(" Set xformOp:orient for RG2 gripper.")
        else:
            print(f" Gripper not found at {rg2_path}")

        # Create or update the physics joint
        if not joint_prim:
            joint_prim = stage.DefinePrim(joint_path, "PhysicsFixedJoint")

        joint_prim.CreateRelationship("physics:body1").SetTargets([Sdf.Path(rg2_base_link)])
        joint_prim.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool).Set(True)
        joint_prim.CreateAttribute("physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool).Set(True)

        # --------- 2. Set localRot0 and localRot1 for joint ---------
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

    def open_gripper(self):

        if not self._gripper_view:
            # Initialize gripper view if not already done
            from omni.isaac.core.articulations import ArticulationView
            self._gripper_view = ArticulationView(prim_paths_expr="/RG2_Gripper", name="RG2_Gripper_View")
            self._gripper_view.initialize()
        
        from omni.isaac.core.utils.types import ArticulationActions
        import numpy as np
        
        target_positions = np.array([np.pi / 6, np.pi / 6])  # Open position
        action = ArticulationActions(joint_positions=target_positions, joint_indices=np.array([0, 1]))
        self._gripper_view.apply_action(action)
        print("RG2 Gripper opened.")

    def close_gripper(self):

        if not self._gripper_view:
            # Initialize gripper view if not already done
            from omni.isaac.core.articulations import ArticulationView
            self._gripper_view = ArticulationView(prim_paths_expr="/RG2_Gripper", name="RG2_Gripper_View")
            self._gripper_view.initialize()
        
        from omni.isaac.core.utils.types import ArticulationActions
        import numpy as np
        
        target_positions = np.array([-np.pi / 4, -np.pi / 4])  # Close position
        action = ArticulationActions(joint_positions=target_positions, joint_indices=np.array([0, 1]))
        self._gripper_view.apply_action(action)
        print("RG2 Gripper closed.")


    def on_shutdown(self):
        print("[InverseKinematicControl] inv kin ctrl shutdown")
