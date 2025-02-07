from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage, FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_prefix
from pathlib import Path

from rich.console import Console

def generate_launch_description():
    console = Console(style="bold magenta underline on white")
    console.print("This is a test line")

    # ROS2 PKG FOLDER PATH
    PACKAGE_NAME = "primitive_ctrl"

    # ---------- START: Get Robot URDF & Start Robot controller
    this_pkg = FindPackageShare(PACKAGE_NAME)
    # Robot URDF Description
    ur_descr_file = PathJoinSubstitution([this_pkg,
                                          "urdf", 
                                          "ur5e_setup.urdf.xacro"])
    


    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")    
    ur_descr_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), 
                                  "urdf", 
                                  description_file]),
            " ",
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur5e",
            " ",
            "tf_prefix:=",
            '""',
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
        package="ur_robot_driver",      #"controller_manager",
        executable="ur_ros2_control_node",      #"ros2_control_node",
        parameters=[ur_descr_dict, ur_robot_controllers],
        # prefix="screen -d -m gdb -command=/home/stefan/.gdb_debug_config --ex run --args",  # noqa E501
        output="both",
        remappings=[
            #("~/robot_description", "/robot_description"),
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
            ('cartesian_compliance_controller/target_frame', 'target_frame'),
            ('cartesian_force_controller/target_wrench', 'target_wrench'),
            ('cartesian_compliance_controller/target_wrench', 'target_wrench'),
            ('cartesian_force_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ('cartesian_compliance_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ('force_torque_sensor_broadcaster/wrench', 'ft_sensor_wrench'),
        ],
        arguments=["--ros-args", "--log-level", "debug"],
    )


    # ---------- SETUP ACTIVE AND INACTIVE CONTROLLERS
    # Convenience function for easy spawner construction
    # CMDLINE EQUIVALENT: ros2 control switch_controllers --activate joint_state_broadcaster \
    #                                                               cartesian_motion_controller
    def controller_spawner(name, *args):
        console.print([a for a in args])
        return Node(
            package="controller_manager",
            executable="spawner",
            name=name,
            output="screen",
            arguments=[name] + [a for a in args],
        )
    # Active controllers
    active_list = [
        #"joint_state_broadcaster",
        #"force_torque_sensor_broadcaster",
        "cartesian_motion_controller"
    ]
    active_spawners = [controller_spawner(controller) for controller in active_list]
    # Inactive controllers
    inactive_list = [
        "cartesian_compliance_controller",
        "cartesian_force_controller",
        "motion_control_handle",
        #"joint_trajectory_controller",
        #"invalid_cartesian_compliance_controller",
        #"invalid_cartesian_force_controller",
    ]
    state = "--inactive"
    inactive_spawners = [controller_spawner(controller, state) for controller in inactive_list]


    return LaunchDescription(declared_arguments + 
                             #[joint_state_publisher_node, robot_state_publisher_node, 
                             #[control_node, *active_spawners, *inactive_spawners]
                             [control_node, active_spawners[0]]
                             )