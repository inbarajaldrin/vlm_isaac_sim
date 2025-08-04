import omni.ext
import omni.ui as ui
import asyncio
import numpy as np
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.transformations import get_world_pose_from_relative
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.storage.native import get_assets_root_path
from typing import Optional


class FrankaOpenDrawerPolicy(PolicyController):
    """The Franka Open Drawer Policy integrated from the Isaac Sim demo"""

    def __init__(
        self,
        prim_path: str,
        cabinet: SingleArticulation,
        root_path: Optional[str] = None,
        name: str = "franka",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """Initialize franka robot and import policy from demo"""
        
        assets_root_path = get_assets_root_path()
        policy_path = "omniverse://localhost/NVIDIA/Assets/Isaac/5.0/Isaac/Samples/Policies/Franka_Policies/Open_Drawer_Policy/"
        
        if usd_path == None:
            usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/5.0/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        self.load_policy(
            policy_path + "policy.pt",
            policy_path + "env.yaml",
            # "/home/aaugus11/IsaacLab/logs/rl_games/franka_open_drawer/2025-08-03_15-29-25/nn/franka_open_drawer.pth",
            # "/home/aaugus11/IsaacLab/logs/rl_games/franka_open_drawer/2025-08-03_15-29-25/params/env.yaml",
        )

        self._action_scale = 1.0
        self._previous_action = np.zeros(9)
        self._policy_counter = 0

        self.cabinet = cabinet

        self.franka_hand_prim = get_prim_at_path(self.robot.prim_path + "/panda_hand")
        self.drawer_handle_top_prim = get_prim_at_path(self.cabinet.prim_path + "/drawer_handle_top")

    def _compute_observation(self):
        """Compute the observation vector for the policy - from demo code"""
        
        # relative transform from the drawer handle to the drawer handle link
        drawer_world_pos, _ = get_world_pose_from_relative(
            self.drawer_handle_top_prim, np.array([0.305, 0, 0.01]), np.array([1, 0, 0, 0])
        )

        # relative transform from the tool center to the hands
        robot_world_pos, _ = get_world_pose_from_relative(
            self.franka_hand_prim, np.array([0, 0, 0.1034]), np.array([1, 0, 0, 0])
        )

        obs = np.zeros(31)
        # Base lin pos
        obs[:9] = self.robot.get_joint_positions() - self.default_pos

        # Base ang vel
        obs[9:18] = self.robot.get_joint_velocities() - self.default_vel

        # Joint states
        obs[18:19] = self.cabinet.get_joint_positions()[self.drawer_link_idx]
        obs[19:20] = self.cabinet.get_joint_velocities()[self.drawer_link_idx]

        # relative distance between drawer and robot
        obs[20:23] = drawer_world_pos - robot_world_pos

        # Previous Action
        obs[23:31] = self._previous_action[0:8]

        return obs

    def forward(self, dt):
        """Compute the desired articulation action - from demo code"""
        
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation()
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()

        # articulation space
        # copy last item for two fingers in order to increase action size from 8 to 9
        # finger positions are absolute positions, not relative to the default position
        self.action[0:8] = self.action[0:8] + self.default_pos[0:8]
        action_input = np.append(self.action, self.action[-1])
        action = ArticulationAction(joint_positions=(action_input * self._action_scale))
        # here action is size 9
        self.robot.apply_action(action)

        self._policy_counter += 1

    def initialize(self, physics_sim_view=None) -> None:
        """Initialize the articulation interface - from demo code"""
        super().initialize(physics_sim_view=physics_sim_view, control_mode="force", set_articulation_props=True)

        self.cabinet.initialize(physics_sim_view=physics_sim_view)
        self.drawer_link_idx = self.cabinet.get_dof_index("drawer_top_joint")

        self.robot.set_solver_position_iteration_count(32)
        self.robot.set_solver_velocity_iteration_count(4)
        self.robot.set_stabilization_threshold(0)
        self.robot.set_sleep_threshold(0)

    def post_reset(self):
        """Reset policy state - from demo code"""
        self._policy_counter = 0
        self._previous_action = np.zeros(9)


class FrankaExtension(omni.ext.IExt):
    """Extension that integrates the complete Franka demo functionality"""
    
    def on_startup(self, ext_id):
        print("[FrankaExtension] startup")

        self._timeline = omni.timeline.get_timeline_interface()
        self._cabinet = None
        self._franka_policy = None
        self._physics_ready = False
        self._is_playing_policy = False
        self._event_timer_callback = None
        
        # World settings from the demo
        self._world_settings = {
            "stage_units_in_meters": 1.0,
            "physics_dt": 1.0 / 400.0,
            "rendering_dt": 1.0 / 60.0
        }

        # Create the window UI
        self._window = ui.Window("Franka Control", width=300, height=250)
        with self._window.frame:
            with ui.VStack(spacing=5):
                self.create_ui()

    def create_ui(self):
        with ui.VStack(spacing=5):
            with ui.CollapsableFrame(title="Setup", collapsed=False, height=0):
                with ui.VStack(spacing=5):
                    ui.Button("Load Scene", width=200, height=40, clicked_fn=lambda: asyncio.ensure_future(self.setup_scene()))
                    ui.Button("Load Cabinet", width=200, height=40, clicked_fn=lambda: asyncio.ensure_future(self.load_cabinet()))
                    ui.Button("Load Franka", width=200, height=40, clicked_fn=lambda: asyncio.ensure_future(self.load_franka()))
            
            with ui.CollapsableFrame(title="Reinforcement Learning", collapsed=False, height=0):
                with ui.VStack(spacing=5):
                    ui.Button("Play", width=200, height=40, clicked_fn=lambda: asyncio.ensure_future(self.play_policy()))
                    ui.Button("Stop", width=200, height=40, clicked_fn=self.stop_policy)

    async def setup_scene(self):
        """Setup scene using the simple approach from UR5e extension"""
        world = World()
        await world.initialize_simulation_context_async()
        world.scene.add_default_ground_plane()
        print("Scene loaded successfully.")

    async def load_cabinet(self):
        """Load cabinet exactly like the demo"""
        cabinet_prim_path = "/World/cabinet"
        cabinet_usd_path = get_assets_root_path() + "/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd"
        cabinet_name = "cabinet"
        cabinet_position = np.array([0.8, 0.0, 0.4])
        cabinet_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        add_reference_to_stage(cabinet_usd_path, cabinet_prim_path)
        
        self._cabinet = SingleArticulation(
            prim_path=cabinet_prim_path, 
            name=cabinet_name, 
            position=cabinet_position, 
            orientation=cabinet_orientation
        )
        
        World.instance().scene.add(self._cabinet)
        await World.instance().reset_async()
        self._timeline.stop()

        print("Cabinet loaded successfully!")

    async def load_franka(self):
        """Load Franka with policy exactly like the demo"""
        if not self._cabinet:
            print("Please load cabinet first!")
            return
            
        self._franka_policy = FrankaOpenDrawerPolicy(
            prim_path="/World/franka", 
            cabinet=self._cabinet,
            name="franka", 
            position=np.array([0, 0, 0])
        )
        
        World.instance().scene.add(self._franka_policy.robot)
        
        # Setup timeline callback exactly like demo
        self._event_timer_callback = self._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self._timeline_timer_callback_fn
        )
        
        await World.instance().reset_async()
        self._timeline.stop()

        print("Franka robot with policy loaded successfully!")

    async def play_policy(self):
        """Start policy execution exactly like the demo setup_post_load"""
        if not self._franka_policy:
            print("Please load Franka first!")
            return
            
        self._physics_ready = False
        self._is_playing_policy = True
        
        # Add physics callback exactly like demo
        World.instance().add_physics_callback("physics_step", callback_fn=self.on_physics_step)
        
        await World.instance().play_async()
        print("Policy started - Franka will open drawer")

    def stop_policy(self):
        """Stop policy execution and cleanup"""
        if self._is_playing_policy:
            self._is_playing_policy = False
            self._physics_ready = False
            
            # Remove physics callback
            if World.instance().physics_callback_exists("physics_step"):
                World.instance().remove_physics_callback("physics_step")
            
            # Stop timeline
            self._timeline.stop()
            print("Policy stopped")

    def on_physics_step(self, step_size):
        """Physics step callback exactly like the demo"""
        if not self._is_playing_policy:
            return
            
        if self._physics_ready:
            try:
                self._franka_policy.forward(step_size)
            except Exception as e:
                print(f"Policy execution error: {e}")
                self.stop_policy()
        else:
            self._physics_ready = True
            try:
                self._franka_policy.initialize()
                self._franka_policy.post_reset()
                self._franka_policy.robot.set_joints_default_state(self._franka_policy.default_pos)
                print("Policy initialized successfully")
            except Exception as e:
                print(f"Policy initialization error: {e}")
                self.stop_policy()

    def _timeline_timer_callback_fn(self, event):
        """Timeline callback exactly like demo"""
        if self._franka_policy:
            self._physics_ready = False

    def world_cleanup(self):
        """Cleanup exactly like demo"""
        world = World.instance()
        self._event_timer_callback = None
        if world and world.physics_callback_exists("physics_step"):
            world.remove_physics_callback("physics_step")

    def on_shutdown(self):
        self.stop_policy()
        self.world_cleanup()
        print("[FrankaExtension] shutdown")