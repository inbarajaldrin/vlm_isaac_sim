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
        self._joint_positions = [0.0] * 6  # Initial joint positions for UR5e
        self._input_fields = []  # To store input text fields for joint values
        self._cartesian_values = [0.1, 0.1, 0.99, 0.0, 0.0, 0.0]  # Default Cartesian values
        self._cartesian_input_fields = []  # To store input text fields for Cartesian values
        self._ik_solver = None
        self._articulation_kinematics_solver = None
        self._articulation = None
        self._gripper_view = None

        # Create the window UI
        self._window = ui.Window("Inverse Kinematics Control", width=300, height=800)
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
                        ui.Button("Play", width=100, height=35, clicked_fn=self.play_simulation)
                        ui.Button("Stop", width=100, height=35, clicked_fn=self.stop_simulation)

            with ui.CollapsableFrame(title="Inverse Kinematic Solver", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Load Lula IK Solver", alignment=ui.Alignment.LEFT)
                    ui.Button("Load Lula", width=150, height=35, clicked_fn=self.load_lula_ik_solver)

            with ui.CollapsableFrame(title="Joint Control", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Joint Control Inputs", alignment=ui.Alignment.LEFT)
                    joint_labels = [
                        "Shoulder Pan Joint", "Shoulder Lift Joint", "Elbow Joint",
                        "Wrist 1 Joint", "Wrist 2 Joint", "Wrist 3 Joint"
                    ]
                    for i, joint_label in enumerate(joint_labels):
                        with ui.HStack(spacing=5):
                            ui.Label(f"{joint_label}:", width=150)
                            joint_field = ui.FloatField()
                            joint_field.model.set_value(self._joint_positions[i])
                            joint_field.model.add_value_changed_fn(self._on_value_change(i))
                            self._input_fields.append(joint_field)
                    with ui.HStack(spacing=5):
                        ui.Button("Set Joint Values", width=150, height=35, clicked_fn=self._apply_joint_positions)
                        ui.Button("Reset Joint Values", width=150, height=35, clicked_fn=self.reset_joint_positions)
                        
            with ui.CollapsableFrame(title="Cartesian Control", collapsed=False, height=0):
                with ui.VStack(spacing=5, height=0):
                    ui.Label("Cartesian Control Inputs", alignment=ui.Alignment.LEFT)
                    cartesian_labels = ["X Position", "Y Position", "Z Position", "Roll", "Pitch", "Yaw"]
                    for i, label in enumerate(cartesian_labels):
                        with ui.HStack(spacing=5):
                            ui.Label(f"{label}:", width=150)
                            field = ui.FloatField()
                            field.model.set_value(self._cartesian_values[i])
                            field.model.add_value_changed_fn(self._on_cartesian_value_change(i))
                            self._cartesian_input_fields.append(field)
                    with ui.HStack(spacing=5):
                        ui.Button("Set Cartesian Values", width=150, height=35, clicked_fn=self._apply_cartesian_position)
                        ui.Button("Reset Cartesian Values", width=150, height=35, clicked_fn=self.reset_cartesian_position)

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
        # asset_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
        asset_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")

        self._ur5e_view = ArticulationView(prim_paths_expr="/World/UR5e", name="ur5e_view")
        World.instance().scene.add(self._ur5e_view)
        await World.instance().reset_async()
        self._timeline.stop()

        self._articulation = Articulation("/World/UR5e")

        joint_count = self._ur5e_view.count
        if joint_count == 6:
            self._joint_positions = self._ur5e_view.get_joint_positions().tolist()
        else:
            print(f"Warning: Expected 6 joints, but found {joint_count}. Initializing with zeros.")
            self._joint_positions = [0.0] * 6

        print("UR5e robot loaded successfully!")

    def load_lula_ik_solver(self):
        if not self._articulation:
            print("Error: Load the UR5e robot first.")
            return

        # Dynamically find paths
        home_dir = os.getenv("HOME")
        pkg_dir = os.path.join(home_dir, ".local", "share", "ov", "pkg")
        isaac_sim_versions = [d for d in os.listdir(pkg_dir) if d.startswith("isaac-sim-")]
        isaac_sim_versions.sort(reverse=True)

        if isaac_sim_versions:
            isaac_sim_version = isaac_sim_versions[0]
            base_path = os.path.join(pkg_dir, isaac_sim_version, "exts", "omni.isaac.motion_generation")

            robot_description_path = os.path.join(base_path, "motion_policy_configs", "universal_robots", "ur5e", "rmpflow", "ur5e_robot_description.yaml")
            urdf_path = os.path.join(base_path, "motion_policy_configs", "universal_robots", "ur5e", "ur5e.urdf")

            self._ik_solver = LulaKinematicsSolver(
                robot_description_path=robot_description_path,
                urdf_path=urdf_path
            )

            self._articulation_kinematics_solver = ArticulationKinematicsSolver(
                self._articulation, self._ik_solver, "tool0"
            )

            print("Lula IK Solver loaded successfully!")
        else:
            print("Isaac Sim installation not found in the expected directory.")

    def play_simulation(self):
        self._timeline.play()
        print("Simulation started.")

    def stop_simulation(self):
        self._timeline.stop()
        print("Simulation stopped.")

    def _on_value_change(self, index):
        def value_changed(model):
            self._joint_positions[index] = model.as_float
        return value_changed

    def _apply_joint_positions(self):
        self._articulation.initialize()

        if len(self._joint_positions) != 6:
            print("Mismatch in joint positions count. Resetting to zero.")
            self._joint_positions = [0.0] * 6

        asyncio.ensure_future(self.control_ur5e())

    async def control_ur5e(self):
        from omni.isaac.dynamic_control import _dynamic_control
        dc = _dynamic_control.acquire_dynamic_control_interface()
        articulation_path = "/World/UR5e"
        articulation = dc.get_articulation(articulation_path)

        if not articulation:
            print(f"Failed to find articulation at {articulation_path}")
            return

        dc.wake_up_articulation(articulation)
        joint_dofs = dc.get_articulation_dof_count(articulation)

        if len(self._joint_positions) != joint_dofs:
            print(f"Mismatch in joint positions count: Expected {joint_dofs}, but got {len(self._joint_positions)}")
            return

        dc.set_articulation_dof_position_targets(articulation, self._joint_positions)
        print(f"Joint positions have been set to: {self._joint_positions}")

    def reset_joint_positions(self):
        self._joint_positions = [0.0] * 6
        for i, field in enumerate(self._input_fields):
            field.model.set_value(0.0)
        asyncio.ensure_future(self.control_ur5e())
        print("Joint positions reset to zero.")

    def _on_cartesian_value_change(self, index):
        def value_changed(model):
            self._cartesian_values[index] = model.as_float
        return value_changed

    def _apply_cartesian_position(self):
        self._articulation.initialize()

        if not self._articulation_kinematics_solver:
            print("Error: Load the Lula IK Solver first.")
            self.load_lula_ik_solver()
            if not self._articulation_kinematics_solver:
                print("Error: Unable to initialize Lula IK Solver.")
                return

        # Extract Cartesian values
        x, y, z, roll, pitch, yaw = self._cartesian_values

        # Convert to required formats
        target_orientation = np.array(euler_angles_to_quats([roll, pitch, yaw]), dtype=np.float32)
        target_position = np.array([x, y, z], dtype=np.float32)

        print(f"Moving to position: {target_position}, orientation: {target_orientation}")

        # Compute inverse kinematics
        action, success = self._articulation_kinematics_solver.compute_inverse_kinematics(
            target_position, target_orientation
        )

        if success:
            self._articulation.apply_action(action)
            print(f"Successfully moved to target: {target_position}, {target_orientation}")
        else:
            print("IK did not converge to a solution.")
            print(f"Target Position: {target_position}, Target Orientation: {target_orientation}")

    def reset_cartesian_position(self):
        self._cartesian_values = [0.1, 0.1, 0.99, 0.0, 0.0, 0.0]
        for i, field in enumerate(self._cartesian_input_fields):
            field.model.set_value(self._cartesian_values[i])
        print("Cartesian values reset to defaults.")

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
            print("✅ Set xformOp:orient for RG2 gripper.")
        else:
            print(f"❌ Gripper not found at {rg2_path}")

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
            print("✅ Set physics:localRot0 and localRot1 for robot_gripper_joint.")
        else:
            print(f"❌ Joint not found at {joint_path}")

        print("✅ RG2 successfully attached to UR5e with proper orientation and joint configuration.")

    def open_gripper(self):
        # from omni.isaac.core.articulations import Articulation
        # import numpy as np
        # gripper = Articulation(prim_path="/RG2_Gripper", name="RG2")
        # gripper.initialize()
        # gripper.set_joint_positions(np.array([np.pi / 4, np.pi / 4]), joint_indices=np.array([0, 1]))
        # print("Gripper opened.")

        if not self._gripper_view:
            # Initialize gripper view if not already done
            from omni.isaac.core.articulations import ArticulationView
            self._gripper_view = ArticulationView(prim_paths_expr="/RG2_Gripper", name="RG2_Gripper_View")
            self._gripper_view.initialize()
        
        from omni.isaac.core.utils.types import ArticulationActions
        import numpy as np
        
        target_positions = np.array([np.pi / 4, np.pi / 4])  # Open position
        action = ArticulationActions(joint_positions=target_positions, joint_indices=np.array([0, 1]))
        self._gripper_view.apply_action(action)
        print("✅ RG2 Gripper opened.")

    def close_gripper(self):
        # from omni.isaac.core.articulations import Articulation
        # import numpy as np
        # gripper = Articulation(prim_path="/RG2_Gripper", name="RG2")
        # gripper.initialize()
        # gripper.set_joint_positions(np.array([-np.pi / 4, -np.pi / 4]), joint_indices=np.array([0, 1]))
        # print("Gripper closed.")
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
        print("✅ RG2 Gripper closed.")


    def on_shutdown(self):
        print("[InverseKinematicControl] inv kin ctrl shutdown")
