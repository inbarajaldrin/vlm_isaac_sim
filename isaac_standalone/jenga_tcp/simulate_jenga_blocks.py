import threading
import socket
import json
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from pxr import UsdPhysics, Gf, UsdGeom, Sdf
import numpy as np
import omni.usd
import queue

# TCP Ports
RECEIVE_PORT = 65432
SEND_PORT = 65433

# Global variables
block_paths = []
world = None
server_running = True
command_queue = queue.Queue()  # Queue for passing commands from TCP server to the main thread
block_counter = 0  # Counter for sequential block naming

def setup_scene():
    global world
    world = World()
    world.scene.add_default_ground_plane()
    world.reset()
    simulation_app.update()
    world.pause()  # Ensure simulation is paused after loading the scene
    print("Scene setup with ground plane and paused.")

def create_and_place_block(data):
    global block_counter  # Use the global block counter to name blocks sequentially
    try:
        # Parse the input data and extract numeric values
        data_parts = {part.split(': ')[0]: float(part.split(': ')[1]) for part in data.split(', ')}
        x = data_parts['X']
        y = data_parts['Y']
        z = data_parts['Z']
        theta = data_parts['Theta']

        # Increment block counter and use it to name the block
        block_counter += 1
        jenga_path = f"/World/JengaBlock_{block_counter}"
        translation = [x, y, z]
        scale = [0.075, 0.025, 0.015]  # Jenga block dimensions
        jenga_prim = create_prim(jenga_path, prim_type="Cube", translation=translation, scale=scale)

        if not jenga_prim:
            raise ValueError(f"Failed to create Jenga block at path: {jenga_path}")

        # Apply physics properties
        UsdPhysics.RigidBodyAPI.Apply(jenga_prim)
        mass_api = UsdPhysics.MassAPI.Apply(jenga_prim)
        mass_api.CreateMassAttr().Set(1.0)
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        UsdPhysics.CollisionAPI.Apply(jenga_prim)

        # Apply rotation
        stage = jenga_prim.GetStage()
        block_prim = stage.GetPrimAtPath(Sdf.Path(jenga_path))

        if not block_prim.IsValid():
            raise ValueError(f"Block prim at {jenga_path} is not valid.")

        # Create rotation as Gf.Quatd (double precision) and Gf.Rotation
        rotatedeg_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), theta).GetQuat()
        new_orientation = Gf.Quatd(rotatedeg_z.GetReal(), *rotatedeg_z.GetImaginary())  # Convert to Gf.Quatd

        # Directly use block_prim's `xformOp:orient` with GfQuatd
        orientation_attr = block_prim.GetAttribute("xformOp:orient")
        if not orientation_attr.IsValid():
            orientation_attr = block_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd)

        orientation_attr.Set(new_orientation)

        return jenga_path
    except Exception as e:
        print(f"Error while creating and placing block: {e}")
        return None

def send_poses(paths):
    try:
        stage = omni.usd.get_context().get_stage()  # Get the stage from the USD context
        poses = {}
        for path in paths:
            if not path:
                print(f"Invalid path: {path}")
                continue
            prim = stage.GetPrimAtPath(Sdf.Path(path))  # Ensure path is an Sdf.Path object
            if not prim.IsValid():
                print(f"Invalid prim at path: {path}")
                continue
            xform = UsdGeom.Xformable(prim)
            pos = xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
            rot = xform.ComputeLocalToWorldTransform(0).ExtractRotation().GetQuat()

            # Collect position and quaternion
            poses[path] = {
                "position": [pos[0], pos[1], pos[2]],
                "rotation": [rot.GetReal()] + list(rot.GetImaginary())
            }
        return poses
    except Exception as e:
        print(f"Error while retrieving poses: {e}")
        return {}

def reset_scene():
    global block_paths, block_counter
    stage = omni.usd.get_context().get_stage()
    for path in block_paths:
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if prim.IsValid():
            prim.GetStage().RemovePrim(Sdf.Path(path))
    block_paths = []
    block_counter = 0  # Reset the block counter when the scene is reset
    print("Scene reset: all Jenga blocks removed.")

def run_simulation_for_seconds(seconds):
    """Run the simulation for the given amount of time in seconds without freezing the thread."""
    steps = int(seconds / world.get_physics_dt())
    for _ in range(steps):
        world.step(render=True)
        simulation_app.update()

def tcp_server():
    global server_running
    host = '127.0.0.1'
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, RECEIVE_PORT))
    s.listen()
    print(f"Server listening on {host}:{RECEIVE_PORT}")

    while server_running:
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                data = data.decode()
                print(f"Data received: {data}")  # Print the incoming data to the terminal

                # Add received data to the queue to be processed by the main thread
                command_queue.put(data)

    s.close()

def process_commands():
    """Process commands from the queue in the main thread."""
    global block_paths

    while not command_queue.empty():
        data = command_queue.get()

        if data == "play":
            world.play()
            print("Simulation started.")
            # Run the simulation for 5 seconds (non-blocking)
            run_simulation_for_seconds(5)
            # Get the poses and send them to another port
            poses = send_poses(block_paths)
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as send_sock:
                send_sock.connect(('127.0.0.1', SEND_PORT))
                send_sock.sendall(json.dumps(poses).encode())
            print(f"Sent poses: {poses}")
            world.pause()
            print("Simulation paused after sending poses.")

        elif data == "reset":
            reset_scene()
            print("Reset command received: Jenga blocks removed.")

        else:
            # Parse for creating Jenga blocks
            block_path = create_and_place_block(data)
            if block_path:
                block_paths.append(block_path)

def main():
    setup_scene()

    # Start TCP server in a separate thread
    tcp_thread = threading.Thread(target=tcp_server, daemon=True)
    tcp_thread.start()

    # Simulation loop, initially paused
    while simulation_app.is_running():
        world.step(render=True)
        simulation_app.update()

        # Process commands from the queue in the main thread
        process_commands()

    # Stop the server on exit
    global server_running
    server_running = False

    # Close the simulation app on exit
    simulation_app.close()

if __name__ == "__main__":
    main()
