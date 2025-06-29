# Setting up Polyscope Using Docker

This guide will walk you through the steps to set up Polyscope using Docker and integrate it with the Universal Robots ROS 2 driver.

---

## Prerequisites

1. **Docker Installed:** Ensure Docker is installed on your system. Refer to the [Docker Installation Guide](https://docs.docker.com/get-docker/) for your operating system.
2. **ROS 2 Installed:** Install ROS 2 (recommended version: Humble or newer).
3. **Universal Robots ROS 2 Driver:** Ensure you have installed the `universal_robot_ros2_driver` package. For installation instructions, see the [Universal Robots ROS 2 Driver Documentation](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).

---

## Steps

### 1. Pull the URSim Docker Image

Download the official URSim Docker image for the e-Series:

```bash
docker pull universalrobots/ursim_e-series
```

### 2. Run the URSim Container

Run the URSim container interactively:

```bash
docker run --rm -it universalrobots/ursim_e-series
```

This will start the URSim simulation environment and display the IP address of the simulator, which you will need for the next step. Example output:

```
Universal Robots simulator for e-Series:5.19.0

IP address of the simulator

     172.17.0.2

Access the robots user interface through this URL:

     http://172.17.0.2:6080/vnc.html?host=172.17.0.2&port=6080

Access the robots user interface with a VNC application on this address:

     172.17.0.2:5900

Press Ctrl-C to exit
```

You can view and interact with the URSim Polyscope interface in your web browser by navigating to the provided URL:

```
http://172.17.0.2:6080/vnc.html?host=172.17.0.2&port=6080
```

### 3. Launch the Universal Robots ROS 2 Driver

Launch the ROS 2 driver for the UR5e robot:

```bash
ros2 launch ur_bringup ur5e.launch.py ur_type:=ur5e robot_ip:=172.17.0.2
```

Replace `172.17.0.2` with the IP address displayed when running the URSim container.

---

### 4. Run the Isaac Sim ROS 2 Integration Script

Launch Isaac Sim and open the Script Editor window.

Open and run the following script `URsim/ur5e_action_graph.py`

Once the simulation is running, the robot inside Isaac Sim will replicate the motions of the real robot simulated in URSim.

## Troubleshooting

1. **Container Not Found:** Ensure the Docker container is running by using `docker ps`.
2. **Connection Issues:** Verify that the container's IP address is correctly passed to the `robot_ip` parameter.
3. **ROS 2 Driver Installation:** Ensure that the `ur_bringup` package is correctly installed in your ROS 2 workspace.

---

## References

- [URSim Docker Hub](https://hub.docker.com/r/universalrobots/ursim_e-series)
- [Universal Robots ROS 2 Driver GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
