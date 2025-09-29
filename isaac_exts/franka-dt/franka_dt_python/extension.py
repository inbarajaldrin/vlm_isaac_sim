import omni.ext
import omni.ui as ui
import asyncio
import numpy as np
import omni.timeline
import torch
import yaml
import os
import io
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
    """The Franka Open Drawer Policy supporting both .pt (TorchScript) and .pth (checkpoint) formats"""

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
        """Initialize franka robot"""
        
        if usd_path == None:
            usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/5.0/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"#for .pt
            # usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/Franka/franka_instanceable.usd"#for .pth

        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        # Policy loading state
        self.policy_loaded = False
        self.policy_type = None  # 'pt' or 'pth'
        
        # Action and observation settings (will be set based on policy type)
        self._action_scale = 1.0
        self._previous_action = np.zeros(9)
        self._policy_counter = 0
        self.obs_dim = 31  # Default to original demo size, will be updated
        self.action_dim = 9
        
        # Configuration parameters (for .pth files)
        self.action_clip = 1.0
        self.obs_clip = 5.0

        self.cabinet = cabinet
        self.franka_hand_prim = get_prim_at_path(self.robot.prim_path + "/panda_hand")
        self.drawer_handle_top_prim = get_prim_at_path(self.cabinet.prim_path + "/drawer_handle_top")

    def load_policy_pt(self, policy_file_path: str, env_config_path: str) -> bool:
        """
        Load TorchScript .pt policy (original demo format)
        
        Args:
            policy_file_path: Path to .pt file
            env_config_path: Path to env.yaml file
            
        Returns:
            bool: True if successful
        """
        try:
            # Use parent class method for .pt files
            super().load_policy(policy_file_path, env_config_path)
            
            self.policy_type = 'pt'
            self.obs_dim = 31  # Original demo observation size
            self.action_dim = 8  # Original demo action size (before duplication)
            self._action_scale = 1.0
            self.policy_loaded = True
            
            print(f"Successfully loaded .pt policy: {policy_file_path}")
            print(f"Policy type: TorchScript (.pt)")
            print(f"Observation dim: {self.obs_dim}")
            print(f"Action dim: {self.action_dim}")
            
            return True
            
        except Exception as e:
            print(f"Error loading .pt policy: {e}")
            self.policy_loaded = False
            return False

    def load_policy_pth(self, policy_file_path: str, env_config_path: str, agent_config_path: str) -> bool:
        """
        Load checkpoint .pth policy (Isaac Lab format)
        
        Args:
            policy_file_path: Path to .pth file
            env_config_path: Path to env.yaml file  
            agent_config_path: Path to agent.yaml file
            
        Returns:
            bool: True if successful
        """
        try:
            # Load checkpoint
            print(f"Loading checkpoint: {policy_file_path}")
            checkpoint = torch.load(policy_file_path, map_location='cpu', weights_only=False)
            
            # Extract model state dict
            if isinstance(checkpoint, dict) and 'model' in checkpoint:
                model_state_dict = checkpoint['model']
                print(f"Loaded checkpoint with keys: {list(checkpoint.keys())}")
            else:
                model_state_dict = checkpoint
                print("Loaded direct model state dict")
            
            # Load agent configuration
            print(f"Loading agent config: {agent_config_path}")
            with open(agent_config_path, 'r') as f:
                self.agent_config = yaml.safe_load(f)
            
            # Load environment configuration
            print(f"Loading env config: {env_config_path}")
            from isaacsim.robot.policy.examples.controllers.config_loader import parse_env_config, get_physics_properties
            self.policy_env_params = parse_env_config(env_config_path)
            self._decimation, self._dt, self.render_interval = get_physics_properties(self.policy_env_params)
            
            # Extract configuration parameters
            env_params = self.agent_config.get('params', {}).get('env', {})
            self.action_clip = env_params.get('clip_actions', 1.0)
            self.obs_clip = env_params.get('clip_observations', 5.0)
            
            # Create policy network
            self.policy = self._create_policy_from_config(model_state_dict, self.agent_config)
            
            self.policy_type = 'pth'
            self.obs_dim = 23  # Isaac Lab observation size
            self.action_dim = 9   # Isaac Lab action size
            self._action_scale = 7.5  # Isaac Lab action scale
            self.policy_loaded = True
            
            print(f"Successfully loaded .pth policy: {policy_file_path}")
            print(f"Policy type: Isaac Lab checkpoint (.pth)")
            print(f"Observation dim: {self.obs_dim}")
            print(f"Action dim: {self.action_dim}")
            print(f"Action scale: {self._action_scale}")
            print(f"Action clip: {self.action_clip}")
            print(f"Obs clip: {self.obs_clip}")
            
            return True
            
        except Exception as e:
            print(f"Error loading .pth policy: {e}")
            import traceback
            traceback.print_exc()
            self.policy_loaded = False
            return False

    def _create_policy_from_config(self, state_dict, agent_config):
        """Create policy network from agent configuration"""
        
        # Extract network configuration
        network_config = agent_config.get('params', {}).get('model', {}).get('network', {})
        mlp_config = network_config.get('mlp', {})
        
        # Get network architecture parameters
        hidden_units = mlp_config.get('units', [256, 128, 64])
        activation = mlp_config.get('activation', 'elu')
        
        print(f"Creating policy network:")
        print(f"  Hidden units: {hidden_units}")
        print(f"  Activation: {activation}")
        
        class ConfigurablePolicyNetwork(torch.nn.Module):
            def __init__(self, obs_dim, action_dim, hidden_units, activation):
                super().__init__()
                
                # Activation function mapping
                activation_map = {
                    'elu': torch.nn.ELU,
                    'relu': torch.nn.ReLU,
                    'tanh': torch.nn.Tanh,
                    'leaky_relu': torch.nn.LeakyReLU
                }
                
                activation_fn = activation_map.get(activation.lower(), torch.nn.ELU)
                
                # Build network layers
                layers = []
                input_dim = obs_dim
                
                for hidden_dim in hidden_units:
                    layers.append(torch.nn.Linear(input_dim, hidden_dim))
                    layers.append(activation_fn())
                    input_dim = hidden_dim
                
                # Output layer
                layers.append(torch.nn.Linear(input_dim, action_dim))
                
                self.actor = torch.nn.Sequential(*layers)
                
            def forward(self, x):
                return self.actor(x)
        
        # Create network
        policy_net = ConfigurablePolicyNetwork(23, 9, hidden_units, activation)  # Isaac Lab dims
        
        # Load weights
        self._load_weights_from_state_dict(policy_net, state_dict)
        
        policy_net.eval()
        return policy_net

    def _load_weights_from_state_dict(self, policy_net, state_dict):
        """Load weights with multiple fallback strategies"""
        
        print("Available state dict keys:")
        for key in state_dict.keys():
            if 'actor' in key and 'critic' not in key:
                print(f"  {key}: {state_dict[key].shape}")
        
        # Strategy 1: Direct loading
        actor_state_dict = {}
        for key, value in state_dict.items():
            if 'actor' in key and 'critic' not in key:
                clean_key = key.replace('a2c_network.actor.', '').replace('actor.', '')
                actor_state_dict[clean_key] = value
        
        try:
            policy_net.actor.load_state_dict(actor_state_dict, strict=False)
            print("Policy weights loaded successfully via direct loading!")
            return
        except Exception as e:
            print(f"Direct loading failed: {e}")
        
        # Strategy 2: Manual layer-by-layer loading
        print("Attempting manual weight loading...")
        layer_idx = 0
        for name, module in policy_net.actor.named_modules():
            if isinstance(module, torch.nn.Linear):
                # Find corresponding weights
                possible_weight_keys = [
                    f'a2c_network.actor.{layer_idx * 2}.weight',
                    f'actor.{layer_idx * 2}.weight',
                    f'model.actor.{layer_idx * 2}.weight',
                    f'{layer_idx * 2}.weight'
                ]
                
                possible_bias_keys = [
                    f'a2c_network.actor.{layer_idx * 2}.bias',
                    f'actor.{layer_idx * 2}.bias', 
                    f'model.actor.{layer_idx * 2}.bias',
                    f'{layer_idx * 2}.bias'
                ]
                
                # Load weights
                for weight_key in possible_weight_keys:
                    if weight_key in state_dict and module.weight.shape == state_dict[weight_key].shape:
                        module.weight.data.copy_(state_dict[weight_key])
                        print(f"Loaded weight for layer {layer_idx}: {weight_key}")
                        break
                
                # Load bias
                for bias_key in possible_bias_keys:
                    if bias_key in state_dict and module.bias.shape == state_dict[bias_key].shape:
                        module.bias.data.copy_(state_dict[bias_key])
                        print(f"Loaded bias for layer {layer_idx}: {bias_key}")
                        break
                
                layer_idx += 1

    def _compute_observation(self):
        """Compute observation based on policy type"""
        
        if self.policy_type == 'pt':
            return self._compute_observation_pt()
        elif self.policy_type == 'pth':
            return self._compute_observation_pth()
        else:
            raise ValueError("No policy loaded or unknown policy type")

    def _compute_observation_pt(self):
        """Compute 31-dimensional observation for .pt policies (original demo)"""
        
        # Get world positions for target calculation
        drawer_world_pos, _ = get_world_pose_from_relative(
            self.drawer_handle_top_prim, np.array([0.305, 0, 0.01]), np.array([1, 0, 0, 0])
        )
        robot_world_pos, _ = get_world_pose_from_relative(
            self.franka_hand_prim, np.array([0, 0, 0.1034]), np.array([1, 0, 0, 0])
        )

        obs = np.zeros(31)
        # Joint positions relative to default
        obs[:9] = self.robot.get_joint_positions() - self.default_pos
        # Joint velocities relative to default  
        obs[9:18] = self.robot.get_joint_velocities() - self.default_vel
        # Cabinet joint state
        obs[18:19] = self.cabinet.get_joint_positions()[self.drawer_link_idx]
        obs[19:20] = self.cabinet.get_joint_velocities()[self.drawer_link_idx]
        # Target vector
        obs[20:23] = drawer_world_pos - robot_world_pos
        # Previous action
        obs[23:31] = self._previous_action[0:8]

        return obs

    def _compute_observation_pth(self):
        """Compute 23-dimensional observation for .pth policies (Isaac Lab)"""
        
        # Get robot state
        robot_joint_pos = self.robot.get_joint_positions()
        robot_joint_vel = self.robot.get_joint_velocities()
        
        # Get cabinet state
        cabinet_joint_pos = self.cabinet.get_joint_positions()[self.drawer_link_idx]
        cabinet_joint_vel = self.cabinet.get_joint_velocities()[self.drawer_link_idx]
        
        # Joint limits for scaling (Franka Panda)
        robot_dof_lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, 0.0, 0.0])
        robot_dof_upper = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 0.04, 0.04])
        
        # Scale joint positions to [-1, 1]
        dof_pos_scaled = (
            2.0 * (robot_joint_pos - robot_dof_lower) / (robot_dof_upper - robot_dof_lower) - 1.0
        )
        
        # Get target vector
        drawer_world_pos, _ = get_world_pose_from_relative(
            self.drawer_handle_top_prim, np.array([0.3, 0.01, 0.0]), np.array([1, 0, 0, 0])
        )
        robot_world_pos, _ = get_world_pose_from_relative(
            self.franka_hand_prim, np.array([0, 0, 0.1034]), np.array([1, 0, 0, 0])
        )
        to_target = drawer_world_pos - robot_world_pos
        
        # Assemble 23-dimensional observation
        obs = np.zeros(23)
        obs[0:9] = dof_pos_scaled                           # Scaled joint positions
        obs[9:18] = robot_joint_vel * 0.1                   # Scaled joint velocities
        obs[18:21] = to_target                              # Target vector
        obs[21] = cabinet_joint_pos                         # Drawer position
        obs[22] = cabinet_joint_vel                         # Drawer velocity
        
        # Clip observations
        obs = np.clip(obs, -self.obs_clip, self.obs_clip)
        
        return obs

    def forward(self, dt):
        """Execute policy forward pass"""
        
        if not self.policy_loaded:
            print("No policy loaded!")
            return
            
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation()
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()

        if self.policy_type == 'pt':
            self._forward_pt(dt)
        elif self.policy_type == 'pth':
            self._forward_pth(dt)

        self._policy_counter += 1

    def _forward_pt(self, dt):
        """Forward pass for .pt policies (original demo behavior)"""
        
        # Original demo action processing
        action_processed = self.action.copy()
        action_processed[0:8] = action_processed[0:8] + self.default_pos[0:8]
        action_input = np.append(action_processed, action_processed[-1])  # Duplicate last for gripper
        action = ArticulationAction(joint_positions=(action_input * self._action_scale))
        self.robot.apply_action(action)

    def _forward_pth(self, dt):
        """Forward pass for .pth policies (Isaac Lab behavior)"""
        
        # Isaac Lab action processing
        scaled_action = self.action * self._action_scale
        
        # Position control - add to current positions
        current_pos = self.robot.get_joint_positions()
        target_pos = current_pos + scaled_action * dt
        
        # Apply joint limits
        robot_dof_lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, 0.0, 0.0])
        robot_dof_upper = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 0.04, 0.04])
        target_pos = np.clip(target_pos, robot_dof_lower, robot_dof_upper)
        
        action = ArticulationAction(joint_positions=target_pos)
        self.robot.apply_action(action)

    def initialize(self, physics_sim_view=None) -> None:
        """Initialize the articulation interface"""
        
        if self.policy_type == 'pt':
            # Original demo initialization
            super().initialize(physics_sim_view=physics_sim_view, control_mode="force", set_articulation_props=True)
            self.robot.set_solver_position_iteration_count(32)
            self.robot.set_solver_velocity_iteration_count(4)
            self.robot.set_stabilization_threshold(0)
            self.robot.set_sleep_threshold(0)
        elif self.policy_type == 'pth':
            # Isaac Lab initialization
            super().initialize(physics_sim_view=physics_sim_view, control_mode="position", set_articulation_props=True)
            self.robot.set_solver_position_iteration_count(12)
            self.robot.set_solver_velocity_iteration_count(1)
            self.robot.set_enabled_self_collisions(False)

        self.cabinet.initialize(physics_sim_view=physics_sim_view)
        self.drawer_link_idx = self.cabinet.get_dof_index("drawer_top_joint")

    def post_reset(self):
        """Reset policy state"""
        self._policy_counter = 0
        self._previous_action = np.zeros(9)


class FrankaExtension(omni.ext.IExt):
    """Extension supporting both .pt and .pth policy formats"""
    
    def on_startup(self, ext_id):
        print("[FrankaExtension] startup")

        self._timeline = omni.timeline.get_timeline_interface()
        self._cabinet = None
        self._franka_policy = None
        self._physics_ready = False
        self._is_playing_policy = False
        self._event_timer_callback = None
        
        # Policy file paths (you can modify these)
        self._pt_policy_path = "omniverse://localhost/NVIDIA/Assets/Isaac/5.0/Isaac/Samples/Policies/Franka_Policies/Open_Drawer_Policy/policy.pt"
        self._pt_env_path = "omniverse://localhost/NVIDIA/Assets/Isaac/5.0/Isaac/Samples/Policies/Franka_Policies/Open_Drawer_Policy/env.yaml"
        
        self._pth_policy_path = "/home/aaugus11/IsaacLab/logs/rl_games/franka_open_drawer/2025-08-04_08-14-27/nn/franka_open_drawer.pth"
        self._pth_env_path = "/home/aaugus11/IsaacLab/logs/rl_games/franka_open_drawer/2025-08-04_08-14-27/params/env.yaml"
        self._pth_agent_path = "/home/aaugus11/IsaacLab/logs/rl_games/franka_open_drawer/2025-08-04_08-14-27/params/agent.yaml"

        # Create the window UI
        self._window = ui.Window("Franka Control", width=350, height=400)
        with self._window.frame:
            with ui.VStack(spacing=5):
                self.create_ui()

    def create_ui(self):
        with ui.VStack(spacing=5):
            # Scene Setup
            with ui.CollapsableFrame(title="Scene Setup", collapsed=False, height=0):
                with ui.VStack(spacing=5):
                    ui.Button("Load Scene", width=300, height=40, clicked_fn=lambda: asyncio.ensure_future(self.setup_scene()))
                    ui.Button("Load Cabinet", width=300, height=40, clicked_fn=lambda: asyncio.ensure_future(self.load_cabinet()))
                    ui.Button("Load Franka Robot", width=300, height=40, clicked_fn=lambda: asyncio.ensure_future(self.load_franka()))
            
            # Policy Loading
            with ui.CollapsableFrame(title="Policy Loading", collapsed=False, height=0):
                with ui.VStack(spacing=5):
                    ui.Label("TorchScript Policy (.pt files):")
                    ui.Button("Load .pt Policy (Demo)", width=300, height=35, clicked_fn=self.load_pt_policy)
                    
                    ui.Spacer(height=10)
                    ui.Label("Isaac Lab Policy (.pth files):")
                    ui.Button("Load .pth Policy (Isaac Lab)", width=300, height=35, clicked_fn=self.load_pth_policy)
            
            # Policy Control
            with ui.CollapsableFrame(title="Policy Control", collapsed=False, height=0):
                with ui.VStack(spacing=5):
                    ui.Button("Play Policy", width=300, height=40, clicked_fn=lambda: asyncio.ensure_future(self.play_policy()))
                    ui.Button("Stop Policy", width=300, height=40, clicked_fn=self.stop_policy)
            
            # Status
            with ui.CollapsableFrame(title="Status", collapsed=False, height=0):
                with ui.VStack(spacing=5):
                    self._status_label = ui.Label("Status: Ready")

    def update_status(self, message: str):
        """Update status label"""
        if hasattr(self, '_status_label'):
            self._status_label.text = f"Status: {message}"

    async def setup_scene(self):
        """Setup scene"""
        try:
            world = World()
            await world.initialize_simulation_context_async()
            world.scene.add_default_ground_plane()
            self.update_status("Scene loaded successfully")
            print("Scene loaded successfully.")
        except Exception as e:
            self.update_status(f"Scene load failed: {e}")
            print(f"Scene load error: {e}")

    async def load_cabinet(self):
        """Load cabinet"""
        try:
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

            self.update_status("Cabinet loaded successfully")
            print("Cabinet loaded successfully!")
        except Exception as e:
            self.update_status(f"Cabinet load failed: {e}")
            print(f"Cabinet load error: {e}")

    async def load_franka(self):
        """Load Franka robot (without policy)"""
        try:
            if not self._cabinet:
                self.update_status("Please load cabinet first!")
                return
                
            self._franka_policy = FrankaOpenDrawerPolicy(
                prim_path="/World/franka", 
                cabinet=self._cabinet,
                name="franka", 
                position=np.array([0, 0, 0])
            )
            
            World.instance().scene.add(self._franka_policy.robot)
            
            # Setup timeline callback
            self._event_timer_callback = self._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP), self._timeline_timer_callback_fn
            )
            
            await World.instance().reset_async()
            self._timeline.stop()

            self.update_status("Franka robot loaded successfully")
            print("Franka robot loaded successfully!")
        except Exception as e:
            self.update_status(f"Franka load failed: {e}")
            print(f"Franka load error: {e}")

    def load_pt_policy(self):
        """Load .pt TorchScript policy"""
        if not self._franka_policy:
            self.update_status("Please load Franka robot first!")
            return
            
        success = self._franka_policy.load_policy_pt(self._pt_policy_path, self._pt_env_path)
        if success:
            self.update_status("TorchScript (.pt) policy loaded successfully")
        else:
            self.update_status("Failed to load .pt policy")

    def load_pth_policy(self):
        """Load .pth checkpoint policy"""
        if not self._franka_policy:
            self.update_status("Please load Franka robot first!")
            return
            
        # Check if files exist
        if not os.path.exists(self._pth_policy_path):
            self.update_status(f"Policy file not found: {self._pth_policy_path}")
            return
        if not os.path.exists(self._pth_env_path):
            self.update_status(f"Env config not found: {self._pth_env_path}")
            return
        if not os.path.exists(self._pth_agent_path):
            self.update_status(f"Agent config not found: {self._pth_agent_path}")
            return
            
        success = self._franka_policy.load_policy_pth(
            self._pth_policy_path, 
            self._pth_env_path, 
            self._pth_agent_path
        )
        if success:
            self.update_status("Isaac Lab (.pth) policy loaded successfully")
        else:
            self.update_status("Failed to load .pth policy")

    async def play_policy(self):
        """Start policy execution"""
        if not self._franka_policy:
            self.update_status("Please load Franka robot first!")
            return
            
        if not self._franka_policy.policy_loaded:
            self.update_status("Please load a policy first!")
            return
            
        try:
            self._physics_ready = False
            self._is_playing_policy = True
            
            # Add physics callback
            World.instance().add_physics_callback("physics_step", callback_fn=self.on_physics_step)
            
            await World.instance().play_async()
            self.update_status(f"Policy started ({self._franka_policy.policy_type} format)")
            print(f"Policy started - Franka will open drawer using {self._franka_policy.policy_type} policy")
        except Exception as e:
            self.update_status(f"Policy start failed: {e}")
            print(f"Policy start error: {e}")

    def stop_policy(self):
        """Stop policy execution"""
        if self._is_playing_policy:
            self._is_playing_policy = False
            self._physics_ready = False
            
            # Remove physics callback
            if World.instance().physics_callback_exists("physics_step"):
                World.instance().remove_physics_callback("physics_step")
            
            # Stop timeline
            self._timeline.stop()
            self.update_status("Policy stopped")
            print("Policy stopped")

    def on_physics_step(self, step_size):
        """Physics step callback"""
        if not self._is_playing_policy:
            return
            
        if self._physics_ready:
            try:
                self._franka_policy.forward(step_size)
            except Exception as e:
                print(f"Policy execution error: {e}")
                self.update_status(f"Policy error: {e}")
                self.stop_policy()
        else:
            self._physics_ready = True
            try:
                self._franka_policy.initialize()
                self._franka_policy.post_reset()
                
                # Set initial joint positions
                if self._franka_policy.policy_type == 'pt':
                    # Original demo initial position
                    self._franka_policy.robot.set_joints_default_state(self._franka_policy.default_pos)
                elif self._franka_policy.policy_type == 'pth':
                    # Isaac Lab initial position
                    initial_pos = np.array([1.157, -1.066, -0.155, -2.239, -1.841, 1.003, 0.469, 0.035, 0.035])
                    self._franka_policy.robot.set_joints_default_state(initial_pos)
                
                print("Policy initialized successfully")
            except Exception as e:
                print(f"Policy initialization error: {e}")
                self.update_status(f"Policy init error: {e}")
                self.stop_policy()

    def _timeline_timer_callback_fn(self, event):
        """Timeline callback"""
        if self._franka_policy:
            self._physics_ready = False

    def world_cleanup(self):
        """Cleanup"""
        world = World.instance()
        self._event_timer_callback = None
        if world and world.physics_callback_exists("physics_step"):
            world.remove_physics_callback("physics_step")

    def on_shutdown(self):
        self.stop_policy()
        self.world_cleanup()
        self.update_status("Extension shutdown")
        print("[FrankaExtension] shutdown")