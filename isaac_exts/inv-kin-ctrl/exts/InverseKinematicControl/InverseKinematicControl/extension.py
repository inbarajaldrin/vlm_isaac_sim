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


    async def load_scene(self):
        world = World()
        await world.initialize_simulation_context_async()
        world.scene.add_default_ground_plane()
        print("Scene loaded successfully.")

    async def load_ur5e(self):
        asset_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
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

    def on_shutdown(self):
        print("[InverseKinematicControl] inv kin ctrl shutdown")
