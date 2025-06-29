#!/usr/bin/env python3
"""
Complete UR5e ROS2 Setup in Isaac Sim
Loads the UR5e robot and creates ROS2 Action Graphs for subscribing to /joint_states
Run this inside IsaacSim script editor
"""

import asyncio
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView, Articulation
from omni.isaac.nucleus import get_assets_root_path
import omni.graph.core as og
from omni.isaac.core.utils.extensions import enable_extension
import omni.isaac.core.utils.stage as stage_utils

async def load_scene_and_ur5e():
    print("Loading scene...")
    world = World()
    await world.initialize_simulation_context_async()
    world.scene.add_default_ground_plane()
    print(" Ground plane added.")
    
    print("Loading UR5e...")
    asset_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR5e")
    
    ur5e_view = ArticulationView(prim_paths_expr="/World/UR5e", name="ur5e_view")
    world.scene.add(ur5e_view)
    await world.reset_async()
    
    articulation = Articulation("/World/UR5e")
    
    joint_count = ur5e_view.count
    joint_positions = ur5e_view.get_joint_positions().tolist() if joint_count == 6 else [0.0] * 6
    print(f"Initial UR5e Joint Positions: {joint_positions}")
    print(" UR5e robot loaded successfully!")
    
    world.stop()
    print(" Simulation paused.")

def setup_robot_target():
    try:
        robot_prim_path = "/World/UR5e"
        stage = stage_utils.get_current_stage()
        if stage.GetPrimAtPath(robot_prim_path):
            print(f" Robot found at: {robot_prim_path}")
        else:
            print(f" Robot not found at {robot_prim_path}")
    except Exception as e:
        print(f" Error checking robot target: {e}")

def create_ros2_action_graph():
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

    og.Controller.edit(
        graph,
        {
            og.Controller.Keys.SET_VALUES: [
                ("articulation_controller.inputs:targetPrim", ["/World/UR5e"]),
            ]
        }
    )

    print(f" Created main Action Graph at: {graph_path}")
    return graph, nodes

async def main():
    print(" Starting UR5e ROS2 Setup Pipeline")
    await load_scene_and_ur5e()
    setup_robot_target()
    create_ros2_action_graph()
    print("\n Setup complete. Next Steps:")
    print("1. Play simulation to activate graph")
    print("2. Make sure ROS 2 is running with the correct domain ID")
    print("3. Launch your ROS 2 controller nodes")

# Launch the main async logic
asyncio.ensure_future(main())
