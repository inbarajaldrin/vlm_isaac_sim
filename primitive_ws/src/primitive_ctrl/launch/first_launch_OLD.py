"""
Launch in Order:

1. Start Robot Simulation
2. Switch to the "cartesian_motion_controller" and "motion_control_handle" 
   (CLI command is ros2 control switch_controllers --activate  cartesian_motion_controller motion_control_handle)
3. Set Parameters of Cartesian Motion Controller using "set_param" node
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage, FindPackageShare

from ament_index_python.packages import get_package_prefix
from pathlib import Path


def generate_launch_description():
    # ROS2 PKG FOLDER PATH
    PACKAGE_NAME = "primitive_ctrl"

    # ---------- START: Get Robot URDF & Start Robot controller
    this_pkg = FindPackageShare(PACKAGE_NAME)
    # Robot URDF Description
    ur_descr_file = PathJoinSubstitution([this_pkg,
                                          "urdf", 
                                          "ur5e_setup.urdf.xacro"])
    ur_descr_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            ur_descr_file,
            " ",
            "robot_ip:=",
            "192.168.1.9",
        ]
    )
    ur_descr_dict = {"robot_description": ur_descr_content}
    # Robot Controllers Params Description
    ur_robot_controllers = PathJoinSubstitution([this_pkg, 
                                                 "config",
                                                 "controller_mgr_start.yaml"])
    # The actual simulation is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.
    control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[ur_descr_dict, ur_robot_controllers],
        # prefix="screen -d -m gdb -command=/home/stefan/.gdb_debug_config --ex run --args",  # noqa E501
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
            ('cartesian_compliance_controller/target_frame', 'target_frame'),
            ('cartesian_force_controller/target_wrench', 'target_wrench'),
            ('cartesian_compliance_controller/target_wrench', 'target_wrench'),
            ('cartesian_force_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ('cartesian_compliance_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ('force_torque_sensor_broadcaster/wrench', 'ft_sensor_wrench'),
        ],
    )


    # ---------- SETUP ACTIVE AND INACTIVE CONTROLLERS
    # Convenience function for easy spawner construction
    # CMDLINE EQUIVALENT: ros2 control switch_controllers --activate joint_state_broadcaster \
    #                                                               cartesian_motion_controller
    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            name=name,
            output="screen",
            arguments=[name] + [a for a in args],
        )
    # Active controllers
    active_list = [
        "joint_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "cartesian_motion_controller"
    ]
    active_spawners = [controller_spawner(controller) for controller in active_list]
    # Inactive controllers
    inactive_list = [
        "cartesian_compliance_controller",
        "cartesian_force_controller",
        "motion_control_handle",
        "joint_trajectory_controller",
        #"invalid_cartesian_compliance_controller",
        #"invalid_cartesian_force_controller",
    ]
    state = "--inactive"
    inactive_spawners = [controller_spawner(controller, state) for controller in inactive_list]


    # ---------- PARAMETER SETTER NODE - cartesian_motion_controller
    # Set the Parameters by reading from a YAML file and running the setter Node
    # YAML file located under primtive_ws/src/primitive_ctrl/config
    # Get the Workspace INSTALL directory
    installed_pkg_dir = Path(get_package_prefix(PACKAGE_NAME))
    # Get the Workspace Home Directory 
    workspace_dir = str(installed_pkg_dir.parent.parent)
    config_file = PathJoinSubstitution([
                        #FindPackageShare("primitive_ctrl"),
                        workspace_dir,
                        "src",
                        PACKAGE_NAME,
                        "config",
                        "solver_safe_params.yaml"
                  ]) 
    parameter_setter = Node(
                            package="primitive_ctrl",
                            namespace="",   #"param_setter",
                            executable="set_param",
                            name="set_param_node",
                            parameters=[config_file]
                       )


    # ---------- ROBOT 3D MOVER NODE
    # Move the Robot from its initial Position to a preset position
    robot_mover = Node(
                        package="primitive_ctrl",
                        namespace="",   #"param_setter",
                        executable="move3d",
                        name="move3d_node",
                        parameters=[] 
                      )


    # ---------- Start Robot State Publisher
    # TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[ur_descr_dict],
    )


    # ---------- VISUALIZATION
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("primitive_ctrl"), "etc", "view_robot.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )


    sequential_nodes = [control_node,
                        robot_state_publisher,
                        rviz,

                        *active_spawners,
                        *inactive_spawners,
                        parameter_setter, 
                        robot_mover]
    chained_runner_nodes = []
    for i in range(len(sequential_nodes) - 1):
        chained_runner_nodes.append(
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=sequential_nodes[i],
                        on_start=[sequential_nodes[i + 1]]
                    )
                )
        )
    
    return LaunchDescription([sequential_nodes[0]] + chained_runner_nodes)