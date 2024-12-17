# ROS2 Joint State Subscriber & Articulation Controller

## Overview

This extension for **NVIDIA Isaac Sim** integrates ROS2 joint state subscription with robot articulation control. It uses OmniGraph to manage ROS2 data flow and robot movement, providing a seamless bridge between Isaac Sim and ROS2.

## Features

- **Create UR5e Robot**: Adds the UR5e robot to the simulation environment.
- **Import Action Graph**: Dynamically creates an OmniGraph for ROS2 communication and articulation control.
- **Listen to ROS Topic**: Subscribes to a specified ROS2 joint state topic.
- **Link Joints to Graph**: Connects ROS2 joint state outputs to the articulation controller.
- **Play/Stop Simulation**: Starts or stops the simulation for real-time visualization.

---

## Prerequisites

- NVIDIA Isaac Sim 2023.1.1 or later
- ROS2 installed and configured
- UR5e robot USD asset available in Omniverse
- Python dependencies for Isaac Sim and ROS2:
  - `omni.graph.core`
  - `omni.isaac.ros2_bridge`
  - `omni.isaac.core_nodes`
  - `omni.kit.uiapp`

---

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/<your-github-username>/vlm_isaac_sim.git
   ```

2. **Add the Extension Path**:
   - Open **Isaac Sim**.
   - Go to **Window > Settings > Extension Search Paths**.
   - Add the following path to the search paths:
     ```
     <your-cloned-repo-location>/vlm_isaac_sim/isaac_exts/ros2-subscriber-controller/exts
     ```

3. **Enable the Extension**:
   - Open **Window > Extensions** in Isaac Sim.
   - Search for **"ROS2 Joint State Subscriber & Articulation Controller"** and enable it.

---

## Usage

1. **Launch the Extension**:
   - Once enabled, the extension window will appear with control buttons.

2. **Steps to Use**:
   - **Create UR5e**: Adds the UR5e robot to the `/World` stage.
   - **Import Action Graph**: Creates the OmniGraph nodes for ROS2 communication.
   - **Enter ROS Topic**: Input the ROS2 joint state topic name (e.g., `/joint_states`).
   - **Listen to Topic**: Subscribes to the specified ROS2 topic.
   - **Link Joints to Graph**: Connects ROS2 joint data to the articulation controller.
   - **Play Simulation**: Starts the simulation to visualize the articulation control.

3. **Stop Simulation**:
   - Use the **Stop Simulation** button to pause the environment.

---

## Troubleshooting

- Ensure the ROS2 bridge is running and ROS2 is correctly configured.
- Verify the ROS2 topic is publishing joint states (`/joint_states` by default).
- Check the console logs for any connection errors.

---

## License

This extension is provided under the NVIDIA Software License Agreement.
