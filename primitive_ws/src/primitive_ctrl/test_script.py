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



def test1():
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
    print(ur_descr_dict)
   


def test2():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    from launch_ros.parameter_descriptions import ParameterValue
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }
    print(robot_description)



def test3():
    this_pkg = FindPackageShare("cartesian_controllers_universal_robots")

    # Declare arguments
    arg_robot_ip = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.1.9", description="The robot's IP address"
    )
    declared_args = [arg_robot_ip]

    # Robot description
    description_file = PathJoinSubstitution([this_pkg, "urdf", "setup.urdf.xacro"])
    robot_ip = LaunchConfiguration("robot_ip")
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_file,
            " ",
            "robot_ip:=",
            robot_ip,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    print(robot_description)


def test4():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    
    this_pkg = FindPackageShare("cartesian_controllers_universal_robots")

    # Declare arguments
    arg_robot_ip = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.1.9", description="The robot's IP address"
    )
    declared_args = [arg_robot_ip]

    # Robot description
    description_file = PathJoinSubstitution([this_pkg, "urdf", "setup.urdf.xacro"])
    robot_ip = LaunchConfiguration("robot_ip")
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_file,
            " ",
            "robot_ip:=",
            robot_ip,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot control
    robot_controllers = PathJoinSubstitution([this_pkg, "config", "controller_manager.yaml"])
    control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        output="screen",
        #prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
            ('cartesian_compliance_controller/target_frame', 'target_frame'),
            ('cartesian_force_controller/target_wrench', 'target_wrench'),
            ('cartesian_compliance_controller/target_wrench', 'target_wrench'),
            ('cartesian_force_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ('cartesian_compliance_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ('force_torque_sensor_broadcaster/wrench', 'ft_sensor_wrench'),
            ],
        parameters=[robot_description, robot_controllers],
    )



if __name__ == "__main__":
    test3()