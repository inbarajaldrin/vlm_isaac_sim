from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.types import ArticulationActions
import numpy as np

def control_gripper(open=True):
    gripper_view = ArticulationView(prim_paths_expr="/RG2_Gripper", name="RG2_Gripper_View")
    gripper_view.initialize()
    
    if open:
        target_positions = np.array([np.pi / 6, np.pi / 6])
        print("Opening RG2 gripper...")
    else:
        target_positions = np.array([-np.pi / 4, -np.pi / 4])
        print("Closing RG2 gripper...")

    action = ArticulationActions(joint_positions=target_positions, joint_indices=np.array([0, 1]))
    gripper_view.apply_action(action)
