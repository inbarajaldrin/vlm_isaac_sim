"""
Test script for reading joint FORCES (not efforts) in Isaac Sim.
Tests get_measured_joint_forces() which returns 6-DOF spatial forces per joint.

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
    """Write to log file and print to console."""
    print(msg)
    with open(_log_file, "a") as f:
        f.write(msg + "\n")


def init_articulation_view():
    """Initialize ArticulationView for measured forces."""
    global _articulation_view
    try:
        from omni.isaac.core.articulations import ArticulationView

        _articulation_view = ArticulationView(
            prim_paths_expr="/World/UR5e",
            name="ur5e_force_view"
        )
        _articulation_view.initialize()

        # Log joint names
        joint_names = _articulation_view.joint_names
        log(f"[ArticulationView] Initialized with joints: {joint_names}")
        log(f"[ArticulationView] Number of joints: {len(joint_names)}")
        return True
    except Exception as e:
        log(f"[ArticulationView] Init failed: {e}")
        _articulation_view = None
        return False


def read_forces(dt):
    """Physics step callback - reads forces at physics rate."""
    global _frame_count, _start_time, _articulation_view
    _frame_count += 1

    # Initialize on first frame
    if _frame_count == 1:
        init_articulation_view()

    # Log every 30 frames (~2 times per second at 60Hz)
    if _frame_count % 30 != 0:
        return

    elapsed = time.time() - _start_time
    log(f"\n===== t={elapsed:.2f}s frame={_frame_count} =====")

    if _articulation_view is None:
        log("[ERROR] ArticulationView not initialized")
        return

    # Test 1: get_measured_joint_efforts (original - for comparison)
    try:
        efforts = _articulation_view.get_measured_joint_efforts()
        if efforts is not None and len(efforts) > 0:
            joint_efforts = efforts[0]  # First articulation
            log(f"[EFFORTS] Shape: {efforts.shape}")
            joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
            for i in range(min(6, len(joint_efforts))):
                name = joint_names[i] if i < len(joint_names) else f"j{i}"
                log(f"  {name}: {joint_efforts[i]:.2f} Nm")
    except Exception as e:
        log(f"[EFFORTS] Error: {e}")

    # Test 2: get_measured_joint_forces (NEW - 6-DOF per joint)
    try:
        forces = _articulation_view.get_measured_joint_forces()
        if forces is not None and len(forces) > 0:
            log(f"[FORCES] Shape: {forces.shape}")
            log(f"[FORCES] Format: [Fx, Fy, Fz, Tx, Ty, Tz] per joint")

            joint_forces = forces[0]  # First articulation
            joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

            for i in range(len(joint_forces)):
                name = joint_names[i] if i < len(joint_names) else f"joint_{i}"
                f = joint_forces[i]  # Shape: (6,) = [Fx, Fy, Fz, Tx, Ty, Tz]
                log(f"  {name}: Fx={f[0]:.2f} Fy={f[1]:.2f} Fz={f[2]:.2f} | Tx={f[3]:.2f} Ty={f[4]:.2f} Tz={f[5]:.2f}")

            # Highlight the last joint (most useful for F/T sensing)
            last_idx = len(joint_forces) - 1
            last_name = joint_names[last_idx] if last_idx < len(joint_names) else f"joint_{last_idx}"
            last_f = joint_forces[last_idx]
            log(f"\n  >>> LAST JOINT ({last_name}) - Best for F/T sensing:")
            log(f"      Fz = {last_f[2]:.2f} N (main collision indicator)")
            log(f"      Force magnitude = {(last_f[0]**2 + last_f[1]**2 + last_f[2]**2)**0.5:.2f} N")
        else:
            log("[FORCES] No data returned")
    except Exception as e:
        log(f"[FORCES] Error: {e}")
        import traceback
        log(traceback.format_exc())


def start_force_monitor():
    """Start monitoring forces at physics rate."""
    global _sub, _frame_count, _start_time, _articulation_view
    _frame_count = 0
    _start_time = time.time()
    _articulation_view = None

    with open(_log_file, "w") as f:
        f.write(f"=== Joint Forces Test Started {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        f.write("Testing get_measured_joint_forces() vs get_measured_joint_efforts()\n\n")

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
