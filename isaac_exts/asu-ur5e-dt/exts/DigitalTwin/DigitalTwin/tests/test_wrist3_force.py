"""
Test script for reading wrist3 forces in Isaac Sim.
Run this in the Script Editor while simulation is playing.
Logs to: /tmp/force_log.txt
"""

import omni.physx
import omni.usd
from pxr import UsdPhysics, Usd
import time

# Global state
_sub = None
_frame_count = 0
_log_file = "/tmp/force_log.txt"
_start_time = None
_articulation_view = None


def log(msg):
    """Write to log file."""
    with open(_log_file, "a") as f:
        f.write(msg + "\n")


def find_prims_info(stage):
    """Log info about UR5e structure on first run."""
    log("\n--- Stage Structure Info ---")

    ur5e = stage.GetPrimAtPath("/World/UR5e")
    if ur5e.IsValid():
        for prim in Usd.PrimRange(ur5e):
            path = str(prim.GetPath())
            apis = []
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                apis.append("ArticulationRoot")
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                apis.append("RigidBody")
            if apis:
                log(f"  {path}: {', '.join(apis)}")

    for gripper_path in ["/World/RG2_Gripper", "/World/UR5e/Gripper", "/World/Gripper"]:
        prim = stage.GetPrimAtPath(gripper_path)
        if prim.IsValid():
            log(f"  Gripper found at: {gripper_path}")
            break

    log("--- End Structure Info ---\n")


def init_articulation_view():
    """Initialize ArticulationView for measured efforts."""
    global _articulation_view
    try:
        from omni.isaac.core.articulations import ArticulationView
        import numpy as np

        # Create ArticulationView - it needs the prim path pattern
        _articulation_view = ArticulationView(
            prim_paths_expr="/World/UR5e",
            name="ur5e_view"
        )
        _articulation_view.initialize()
        log("[ArticulationView] Initialized successfully")
        return True
    except Exception as e:
        log(f"[ArticulationView] Init failed: {e}")
        _articulation_view = None
        return False


def read_forces(dt):
    """Physics step callback - reads forces at physics rate."""
    global _frame_count, _start_time, _articulation_view
    _frame_count += 1

    # Log structure info on first frame
    if _frame_count == 1:
        stage = omni.usd.get_context().get_stage()
        find_prims_info(stage)
        init_articulation_view()

    # Log every 10 frames (~6 times per second at 60Hz)
    if _frame_count % 10 != 0:
        return

    elapsed = time.time() - _start_time
    stage = omni.usd.get_context().get_stage()

    log(f"\n===== t={elapsed:.2f}s frame={_frame_count} =====")

    # Method 1: ArticulationView - get_measured_joint_efforts (preferred)
    if _articulation_view is not None:
        try:
            # Get measured joint efforts (actual torques from physics)
            measured_efforts = _articulation_view.get_measured_joint_efforts()
            if measured_efforts is not None and len(measured_efforts) > 0:
                efforts = measured_efforts[0]  # First (only) articulation
                joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
                efforts_str = []
                for i in range(min(6, len(efforts))):
                    name = joint_names[i] if i < len(joint_names) else f"j{i}"
                    efforts_str.append(f"{name}={efforts[i]:.2f}")
                log(f"[Measured] {', '.join(efforts_str)}")
            else:
                log("[Measured] No data")
        except Exception as e:
            log(f"[Measured] Error: {e}")

    # Method 2: ArticulationView - get_applied_joint_efforts
    if _articulation_view is not None:
        try:
            applied_efforts = _articulation_view.get_applied_joint_efforts()
            if applied_efforts is not None and len(applied_efforts) > 0:
                efforts = applied_efforts[0]
                joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
                efforts_str = []
                for i in range(min(6, len(efforts))):
                    name = joint_names[i] if i < len(joint_names) else f"j{i}"
                    efforts_str.append(f"{name}={efforts[i]:.2f}")
                log(f"[Applied] {', '.join(efforts_str)}")
        except Exception as e:
            log(f"[Applied] Error: {e}")

    # Method 3: Dynamic Control (for comparison)
    try:
        from omni.isaac.dynamic_control import _dynamic_control
        dc = _dynamic_control.acquire_dynamic_control_interface()

        art = dc.get_articulation("/World/UR5e/base_link")
        if art != 0:
            dof_states = dc.get_articulation_dof_states(art, _dynamic_control.STATE_ALL)
            joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
            efforts_str = []
            for i in range(min(6, len(dof_states.effort))):
                name = joint_names[i] if i < len(joint_names) else f"j{i}"
                efforts_str.append(f"{name}={dof_states.effort[i]:.2f}")
            log(f"[DC effort] {', '.join(efforts_str)}")
    except Exception as e:
        log(f"[DC] Error: {e}")

    # Method 4: Contact sensor on wrist_3_link
    try:
        from isaacsim.sensors.physics import _sensor
        cs = _sensor.acquire_contact_sensor_interface()

        sensor_path = "/World/UR5e/wrist_3_link/Contact_Sensor"
        prim = stage.GetPrimAtPath(sensor_path)

        if prim.IsValid():
            raw = cs.get_contact_sensor_raw_data(sensor_path)
            if raw is not None and len(raw) > 0:
                total_force = [0.0, 0.0, 0.0]
                for contact in raw:
                    if contact["dt"] > 0:
                        total_force[0] += contact["impulse"][0] / contact["dt"]
                        total_force[1] += contact["impulse"][1] / contact["dt"]
                        total_force[2] += contact["impulse"][2] / contact["dt"]
                mag = (total_force[0]**2 + total_force[1]**2 + total_force[2]**2)**0.5
                log(f"[Wrist3 Contact] F=({total_force[0]:.2f}, {total_force[1]:.2f}, {total_force[2]:.2f}) mag={mag:.2f}N")
            else:
                log(f"[Wrist3 Contact] No contacts")
        else:
            log(f"[Wrist3 Contact] Sensor not found")
    except Exception as e:
        log(f"[Wrist3 Contact] Error: {e}")

    # Method 5: Gripper contact sensor
    try:
        from isaacsim.sensors.physics import _sensor
        cs = _sensor.acquire_contact_sensor_interface()

        sensor_path = "/World/RG2_Gripper/left_inner_finger/Contact_Sensor"
        prim = stage.GetPrimAtPath(sensor_path)

        if prim.IsValid():
            raw = cs.get_contact_sensor_raw_data(sensor_path)
            if raw is not None and len(raw) > 0:
                total_force = [0.0, 0.0, 0.0]
                for contact in raw:
                    if contact["dt"] > 0:
                        total_force[0] += contact["impulse"][0] / contact["dt"]
                        total_force[1] += contact["impulse"][1] / contact["dt"]
                        total_force[2] += contact["impulse"][2] / contact["dt"]
                mag = (total_force[0]**2 + total_force[1]**2 + total_force[2]**2)**0.5
                log(f"[Gripper Contact] F=({total_force[0]:.2f}, {total_force[1]:.2f}, {total_force[2]:.2f}) mag={mag:.2f}N")
            else:
                log(f"[Gripper Contact] No contacts")
    except Exception as e:
        pass  # Gripper might not exist


def start_force_monitor():
    """Start monitoring forces at physics rate."""
    global _sub, _frame_count, _start_time, _articulation_view
    _frame_count = 0
    _start_time = time.time()
    _articulation_view = None

    with open(_log_file, "w") as f:
        f.write(f"=== Force Monitor Log Started {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")

    _sub = omni.physx.get_physx_interface().subscribe_physics_step_events(read_forces)
    print(f"Force monitor started! Logging to {_log_file}")
    print("Run stop_force_monitor() to stop.")


def stop_force_monitor():
    """Stop the force monitor."""
    global _sub, _articulation_view
    if _sub is not None:
        _sub = None
        log(f"\n=== Monitor stopped at {time.strftime('%Y-%m-%d %H:%M:%S')} ===")
        print(f"Force monitor stopped. Log saved to {_log_file}")
    else:
        print("Force monitor was not running.")
    _articulation_view = None


# Auto-start when script is run
if __name__ == "__main__" or True:
    stop_force_monitor()
    start_force_monitor()
