# Jenga Communication Scripts

This README provides detailed instructions on how to use the Python scripts in this folder for simulating and controlling a Jenga block setup in Isaac Sim with ROS 2 communication.

---

## Files Overview

### 1. **send_jenga_coords.py**
   - **Description:** This script sends commands (e.g., block coordinates, "play," or "reset") to the simulation.
   - **Usage:**
     - Send coordinates for Jenga block placement:
       ```bash
       ros2 run jenga_communication send_jenga_coords 1 1 0.015
       ```
     - Start the simulation and retrieve poses:
       ```bash
       ros2 run jenga_communication send_jenga_coords play
       ```
     - Reset the scene:
       ```bash
       ros2 run jenga_communication send_jenga_coords reset
       ```

### 2. **simulate_jenga_blocks.py**
   - **Description:** The core simulation script that runs in Isaac Sim. It listens for commands sent by `send_jenga_coords.py` over a TCP connection, places blocks at specified coordinates, plays the simulation, and sends back block poses after a short delay. It also processes the "reset" command to clear the scene.
   - **Usage:**
     Run this script from the Isaac Sim Python environment:
     ```bash
     cd ~/.local/share/ov/pkg/isaac-sim-2023.1.1
     ./python.sh $(pwd)/scripts/simulate_jenga_blocks.py
     ```

### 3. **receive_jenga_coords.py**
   - **Description:** This script listens on a TCP port for block poses sent by the simulation when the "play" command is executed. It decodes the JSON data, prints the positions and rotations of all blocks, and ensures the poses are retrieved in the correct order.
   - **Usage:**
     Run this script in a separate terminal:
     ```bash
     ros2 run jenga_communication receive_jenga_coords
     ```

---

## Example Workflow

1. **Start the Simulation:**
   Open a terminal and run:
   ```bash
   cd ~/.local/share/ov/pkg/isaac-sim-2023.1.1
   ./python.sh $(pwd)/scripts/simulate_jenga_blocks.py
   ```

2. **Send Commands:**
   Open another terminal and run commands such as:
   - Place Jenga blocks:
     ```bash
     ros2 run jenga_communication send_jenga_coords 1 1 0.015
     ros2 run jenga_communication send_jenga_coords 1 1 0.045
     ros2 run jenga_communication send_jenga_coords 0.9 1 0.075
     ```
   - Rotate a block before placement:
     ```bash
     ros2 run jenga_communication send_jenga_coords 1 1 0.105 45
     ```
   - Start the simulation:
     ```bash
     ros2 run jenga_communication send_jenga_coords play
     ```
   - Reset the scene:
     ```bash
     ros2 run jenga_communication send_jenga_coords reset
     ```

3. **Retrieve Block Poses:**
   Open a third terminal and run:
   ```bash
   ros2 run jenga_communication receive_jenga_coords
   ```

---

## Commands Summary

- **Start Simulation:**
  ```bash
  ./python.sh /path/to/simulate_jenga_blocks.py
  ```

- **Send Block Coordinates:**
  ```bash
  ros2 run jenga_communication send_jenga_coords x y z
  ```

- **Rotate Block and Place:**
  ```bash
  ros2 run jenga_communication send_jenga_coords x y z rotation_angle
  ```

- **Play Simulation:**
  ```bash
  ros2 run jenga_communication send_jenga_coords play
  ```

- **Reset Scene:**
  ```bash
  ros2 run jenga_communication send_jenga_coords reset
  ```

- **Retrieve Block Poses:**
  ```bash
  ros2 run jenga_communication receive_jenga_coords
  ```

---

## Notes

1. **ROS 2 Workspace Setup:** Ensure your ROS 2 workspace is properly sourced before running any ROS 2 commands:
   ```bash
   source $(pwd)/install/setup.bash
   ```

2. **Isaac Sim Version:** Ensure you are using Isaac Sim 2023.1.1 for compatibility.

3. **TCP Communication:** Ensure that all scripts are running on the same network for proper TCP communication.

---

## Troubleshooting

- **Connection Issues:** Ensure all scripts are running and that the IP and port configurations match.
- **Command Not Found:** Verify that your `jenga_communication` package is correctly built and installed in your ROS 2 workspace.
- **Isaac Sim Errors:** Ensure Isaac Sim is running in the correct environment and the simulation script is properly executed.

---

## References

- [Isaac Sim Documentation](https://developer.nvidia.com/isaac-sim)
- [ROS 2 Documentation](https://docs.ros.org/)
- [TCP Communication in Python](https://docs.python.org/3/library/socket.html)

