# Inverse Kinematic Control Extension

## Overview

This extension for **NVIDIA Isaac Sim** provides tools to control the UR5e robot using **inverse kinematics (IK)** and direct joint commands. It integrates an **Lula IK Solver** to compute Cartesian targets for the robot and allows interactive joint control through a user-friendly UI.

## Features

- **Load UR5e Robot**: Adds the UR5e robot into the simulation environment.
- **Lula IK Solver**: Provides inverse kinematic solutions for target Cartesian positions and orientations.
- **Joint Control**: Allows direct input and manipulation of joint positions for the robot.
- **Cartesian Control**: Enables users to set target positions (X, Y, Z) and orientations (Roll, Pitch, Yaw).
- **Play/Stop Simulation**: Easily start or stop the simulation to visualize changes.

---

## Prerequisites

- **NVIDIA Isaac Sim** 2023.1.1 or later
- **UR5e robot USD asset** (available in Omniverse).
- Python dependencies for Isaac Sim:
  - `omni.isaac.motion_generation`
  - `omni.isaac.core`
  - `omni.kit.uiapp`
  - `omni.isaac.dynamic_control`

---

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/<your-github-username>/vlm_isaac_sim.git
   ```

2. **Add the Extension Path**:
   - Open **Isaac Sim**.
   - Go to **Window > Settings > Extension Search Paths**.
   - Add the following path:
     ```
     <your-cloned-repo-location>/vlm_isaac_sim/isaac_exts/inv-kin-ctrl/exts
     ```

3. **Enable the Extension**:
   - Open **Window > Extensions**.
   - Search for **"Inverse Kinematic Control"** and enable it.

---

## Usage

1. **Launch the Extension**:
   - Once enabled, the extension UI will appear with multiple collapsible frames.

2. **Steps to Use**:
   - **Simulation Setup**:
     - Click **"Load Scene"** to set up the environment.
     - Use **"Load UR5e"** to load the UR5e robot into the simulation.
     - Start or stop the simulation with the **Play** and **Stop** buttons.

   - **Lula IK Solver**:
     - Click **"Load Lula"** to initialize the Lula inverse kinematics solver.

   - **Joint Control**:
     - Enter joint values for the 6 UR5e joints in the respective input fields.
     - Click **"Set Joint Values"** to apply the joint positions.
     - Use **"Reset Joint Values"** to return all joint positions to zero.

   - **Cartesian Control**:
     - Input target Cartesian coordinates and orientation (X, Y, Z, Roll, Pitch, Yaw).
     - Click **"Set Cartesian Values"** to move the UR5e robot to the specified pose.
     - Use **"Reset Cartesian Values"** to restore default values.

3. **Observe Changes**:
   - Watch the UR5e robot respond in real-time as joint or Cartesian inputs are applied.

---

## Troubleshooting

- Ensure the Lula IK Solver is loaded before applying Cartesian values.
- Verify that the UR5e robot USD asset is correctly located in Omniverse.
- Check the terminal logs for any warnings about joint or articulation mismatches.

---

## License

This extension is provided under the NVIDIA Software License Agreement.
