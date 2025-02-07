# Instructions 

## ---> "primitive_ctrl" Package Setup

#### Installing Pre-requisites (for this package)
Starting at the root directory of the ROS2 WS,
```
echo "yaml file://$(pwd)/src/primitive_ctrl/rosdep.yaml" > ~/.ros/rosdep/sources.list.d/primitive_ctrl.list
rosdep update
rosdep install --from-paths src/primitive_ctrl --ignore-src -r -y
```
This will copy the dependencies from this package's rosdep.yaml file to the local rosdep definitions, followed by updating rosdep definitions. 

#### Building the package
Starting at the root directory of the ROS2 WS,
```
colcon build --packages-select primitive_ctrl
```
This will build the package *primitive_ctrl.* 

#### RUNNING NODES
**Before launching**
Modify the parameters in config/solver_safe_params.yaml. The safest solver.error_scale value is 0.1, and you can increase it gradually to speed up motion. 

**Initialization Launch file**
```
ros2 launch primitive_ctrl tester.launch.py
```
**Start Action Server**
```
ros2 run primitive_ctrl move3d_action_server
```
**Test Action Server**
```
ros2 run primitive_ctrl move3d_action_client_test
```
**Run Action Client**
```
ros2 run primitive_ctrl mover_client_main
```


#### Dependency Details
The dependency packages being installed by this command are:

**ros-humble-ur** Ubuntu (APT) ROS2 metapackage. 
The instructions for the metapackage is given here: https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html
The repo link is here: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble

**ros-humble-ros2-control** and **ros-humble-ros2-controllers** Ubuntu (APT) ROS2 metapackages.
The instructions are given here: https://control.ros.org/humble/doc/getting_started/getting_started.html

**casadi** Python (pip) package. 
The docs for casadi are here: https://web.casadi.org/python-api/

**numpy** and **pytransform3d** Python (pip) packages. 

**cartesian_controllers** GIT ROS2 metapackage.
The link to the repository is: https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/

**cartesian_controllers_universal_robots** GIT ROS2 example package.
The link to the repository is: https://github.com/stefanscherzinger/cartesian_controllers_universal_robots/
This package is used for its URDF and controller_manager.yaml descriptions.

*See package.xml; -> rosdep.yaml file contains references to the pkgs to be installed.*
*Also, see setup.py which contains pip packages, which manages dependencies when package is distributed as a Python package.*



## ---> "custom_actions" Package
This package contains the following custom message definitions:
#### GoalPose2D
```
string controller_to_use
geometry_msgs/Pose2D pose
```
#### GoalPose2D
```
string controller_to_use
geometry_msgs/PoseStamped pose
```
#### MultiplePoses
```
string object_class
geometry_msgs/PoseStamped[] poses
```
This is an array of multiple PoseStamped msgs. 