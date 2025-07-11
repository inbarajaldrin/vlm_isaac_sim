# Setting up URSim + ROS 2 + Isaac Sim Integration

This guide walks you through setting up the **Universal Robots URSim simulator**, installing the **External Control URCap**, and controlling the UR5e robot through **ROS 2**. It also explains how to connect **Isaac Sim** to mirror the robot in simulation.

---

## Prerequisites

1. **Docker Installed** — [Install Docker](https://docs.docker.com/get-docker/)
2. **ROS 2 Installed** (Recommended: Humble or newer)
3. **Universal Robots ROS 2 Driver** — Install from [Universal\_Robots\_ROS2\_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
4. **Isaac Sim Installed** — Download from [NVIDIA Omniverse](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html)

---

## 1. Setup URSim Folders

```bash
mkdir -p ~/.ursim/urcaps
mkdir -p ~/.ursim/programs
```

---

## 2. Download External Control URCap

```bash
URCAP_VERSION=1.0.5
curl -L -o ~/.ursim/urcaps/externalcontrol-${URCAP_VERSION}.jar \
  https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar
```

---

## 3. Launch URSim with Docker

```bash
docker run --rm -it -p 5900:5900 -p 6080:6080 -v ${HOME}/.ursim/urcaps:/urcaps -v ${HOME}/.ursim/programs:/ursim/programs --name ursim universalrobots/ursim_e-series
```

Access the GUI:

```
http://172.17.0.2:6080/vnc.html?host=172.17.0.2&port=6080
```

---

## 4. Launch the ROS 2 Driver

```bash
ros2 launch ur_bringup ur5e.launch.py ur_type:=ur5e robot_ip:=172.17.0.2
```

> Replace IP if different (check Docker output)

---

## 5. Install and Run External Control in URSim

1. In the URSim interface, select **“Program”** from the main screen.

2. On the left panel, go to the **“URCaps”** section.

3. Under **“URCaps”**, click on **“External Control”** to add it to the program tree.

4. Power on the robot if prompted.

5. Click the **“Play”** button.

6. Then click **“Robot Program”** to begin executing the program containing the External Control node.

The robot is now in external control mode and ready to receive joint trajectory commands from ROS 2.

---

## 6. Test with a Simple Joint Command

```bash
ros2 topic pub /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names:
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
points:
- positions: [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  time_from_start:
    sec: 3
    nanosec: 0"
```

---

### 7. Run the Isaac Sim ROS 2 Integration Script

Start Isaac Sim and open the Script Editor from the main window.
Navigate to and execute the script located at:

URsim/ur5e_action_graph.py

This script sets up the UR5e robot and configures the ROS 2 Action Graph for real-time control.
Once the simulation is running, the UR5e in Isaac Sim will accurately mirror the joint motions of the real or simulated robot in URSim.

---

## References

* [URSim Docker Setup Docs](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/ursim_docker.html)
* [UR ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

---
