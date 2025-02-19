"""
Launch in Order (using delays- not a great way of doing it):

1. Start Robot Simulation using fake hardware from the ur_robot_driver
2. Start the "cartesian_motion_controller" initialize other in inactive mode 
   (CLI command is ros2 control switch_controllers --activate  cartesian_motion_controller)
3. Set Parameters of Cartesian Motion Controller using "set_param" node
"""
from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription, TimerAction,   \
                           ExecuteProcess, OpaqueFunction,   \
                           RegisterEventHandler, DeclareLaunchArgument,     \
                           LogInfo
from launch.event_handlers import OnProcessStart, OnExecutionComplete, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution,     \
                                 LaunchConfiguration

from launch_ros.actions import Node, SetParametersFromFile
from launch_ros.substitutions import FindPackage, FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_prefix
from pathlib import Path


# ROS2 PKG FOLDER NAME
PACKAGE_NAME = "primitive_ctrl"


def event_based_notifier(target_action, ready_message):
    ready_notifier = RegisterEventHandler(
                        event_handler=OnProcessExit(
                                        target_action=target_action,
                                        on_exit=[LogInfo(
                                                msg=colorize(
                                                103,
                                                f"\t\t\t\t\t\t\t{ready_message}!!!!!!\n"
                                                )
                                                )
                                                ]
                                    )
                     )
    return ready_notifier


# This function edits the "Text Color" or "Background Color" of terminal text using ANSI codes
def colorize(color_code, message):
    # TEXT::: color_code = 31<Red>, 32<Green>, 93<Yellow>, 34<Blue>, 36<Cyan>, 95<BrtMagenta>
    # BACKGRD:: color_code = 41<Red>, 42<Green>, 103<Yellow>, 44<Blue>, 46<Cyan>, 105<BrtMagenta>
    # \033[  <Initiate ANSI escape code>, and  \033[0m  <Reset formatting to default>
    return f"\033[{color_code}m{message}\033[0m"


def generate_launch_description():

    logger = logging.get_logger("launch_file_logger")
    logger.info(colorize(46, "\n\n\t\t\t\t\t\t\tStarting the Launch file...\n\n     \
        This node does the following: 1. Start the UR Robot Node & DEFAULT Controllers \n  \
                        \t\t\t\t<<VIRTUAL MODE BY DEFAULT>>\n   \
            \t\t\t\t2. Stop the <scaled_joint_trajectory_controller>\n  \
            \t\t\t\t3. Setup Cartesian Ctrl <cartesian_motion_controllers>\n  \
            \t\t\t\t4. Modify Params of <cartesian_motion_controller>: solver.error_scale \
            \n\n        \
                "))
    
    # Launch argument for starting Robot in "Simulated" or "Real" Mode (default is simulated)
    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware_param', 
                                default_value='true', 
                                description='Launch argument for starting Robot in ' +
                                            'Simulated Mode <"true"> ' +
                                            'or Real Mode <"false>'
                                )
    robot_ip_arg = DeclareLaunchArgument('robot_ip_param', 
                                    default_value='192.168.1.66',
                                )

    # Get ROS2 Package paths
    this_pkg_share = FindPackageShare(PACKAGE_NAME)
    # Get the Workspace INSTALL directory
    installed_pkg_dir = Path(get_package_prefix(PACKAGE_NAME))
    # Get the Workspace Home Directory 
    workspace_home_dir = str(installed_pkg_dir.parent.parent)


    # ---------- START THE RELEVANT NODES FROM ur_robot_driver USING LAUNCH FILE
    # OVERRIDE Robot Controllers Params Description - Modified from ur_robot_control repo 
    # to add the cartesian_controllers.
    cartes_controllers = PathJoinSubstitution([this_pkg_share, 
                                                 "config",
                                                 "controller_mgr_start.yaml"])
    ur_ctrl_launch = PathJoinSubstitution([
                            FindPackageShare("ur_robot_driver"), 
                            "launch",
                            "ur_control.launch.py", 
                     ])
    ur_ctrl_include = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(ur_ctrl_launch),
                            launch_arguments={"ur_type": "ur5e",
                            "use_fake_hardware": LaunchConfiguration('use_fake_hardware_param'),  #"true",
                            "launch_rviz": "true",
                            "robot_ip": LaunchConfiguration('robot_ip_param'),  #"192.168.10.1",
                            "controllers_file": cartes_controllers}.items(),
                      )
    '''ur_ctrl_starter_notifier = event_based_notifier(
                                        ur_ctrl_include, 
                                        "<ur_control.launch.py> Started")'''
    # ----- STOP THE scaled_joint_trajectory_controller which conflicts with cartesian ctrl
    stop_ctrl_proc = ExecuteProcess(cmd=[
                                            "ros2", "control", 
                                            "switch_controllers",
                                            "--deactivate",
                                            "scaled_joint_trajectory_controller"
                                        ],
                                    output="screen"
                                   )
    delayed_stop_ctrl_action = TimerAction(
                                    period=9.0,
                                    actions=[stop_ctrl_proc],
                               )                              
    '''scaled_joint_trajectory_notifier = event_based_notifier(
                                            stop_ctrl_proc, 
                                            "<scaled_joint_trajectory_ctrl> Stopped.")
    '''

    # ---------- SETUP ACTIVE AND INACTIVE CONTROLLERS
    # Convenience function for easy spawner construction
    # CMDLINE EQUIVALENT: ros2 run controller_manager spawner cartesian_motion_controller  \
    #                          -t cartesian_motion_controller/CartesianMotionController    \
    #                          --inactive  -p /home/dimelab/Desktop/vlm_isaac_sim/primitive_ws/src/primitive_ctrl/config/controller_mgr_start.yaml
    def controller_spawner(name, *args):
        ctrl_node = Node(
                        package="controller_manager",
                        executable="spawner",
                        #name="controller_manager",
                        output="screen",
                        parameters=[cartes_controllers],
                        arguments=[name]  +
                                  [a for a in args]  +
                                  ["--controller-manager", "/controller_manager"]  +
                                  ["--ros-args", "--log-level", "info"]                     
                    )
        return ctrl_node
    
    # Active controllers
    active_list = [
        "cartesian_motion_controller"
    ]
    active_spawners = [controller_spawner(controller) for controller in active_list]
    
    # Inactive controllers
    inactive_list = [
        "cartesian_compliance_controller",
        "cartesian_force_controller",
        "motion_control_handle",
        #"invalid_cartesian_compliance_controller",
        #"invalid_cartesian_force_controller",
    ]
    state = "--inactive"
    inactive_spawners = [controller_spawner(controller, state) for controller in inactive_list]

    # Start all Active and Inactive Spawners - Delayed by 10s
    delayed_spawners = TimerAction(
                            period=12.0,  # Wait 10 seconds after ur_ctrl_include starts
                            actions=active_spawners + inactive_spawners,
                       )
    '''spawner_notifier = event_based_notifier(
                            active_spawners[-1], 
                            "<cartesian_controllers> Started.")
    '''

    # ---------- MODIFY THE PARAMETERS OF THE cartesian_motion_controller SOLVER 
    # --- Control the P-D Gains of the controller together using "solver.error_scale" parameter
    # --- Passing in the safe parameters of the controller through the "solver_safe_params.yaml" file
    # Solver details: https://github.com/fzi-forschungszentrum-informatik/cartesian_controllerontrollerss/blob/f8aad77697802d9b9ca08d044b87ce58167e4bf9/resources/doc/Solver_details.md
    modify_error_scale_node = Node(
                                package="primitive_ctrl",
                                executable="set_param",
                                output="screen",
                                arguments=["--ros-args", "--log-level", "info"]  +
                                          ["--params-file",
                                           workspace_home_dir   +
                                                "/src/primitive_ctrl/config/solver_safe_params.yaml"]                     
                             )
    delayed_modify_node = TimerAction(
                            period=15.0,  # Wait 10 seconds after ur_ctrl_include starts
                            actions=[modify_error_scale_node],
                          )
    final_ready_notifier = RegisterEventHandler(
                        event_handler=OnProcessExit(
                                        target_action=modify_error_scale_node,
                                        on_exit=[LogInfo(
                                                         msg=colorize(
                                                             42,
                                                             "\n\n\t\t\t\t\t\t\tREADY TO USE!!!!!! \n\n"
                                                             )
                                                        )
                                                ]
                                      )
                     )


    return LaunchDescription([  
                                robot_ip_arg,   # Input args: True/False
                                use_fake_hardware_arg,  # Input arg: IP Address
                                
                                ur_ctrl_include,        # Start UR control
                                delayed_stop_ctrl_action, # Stop scaled_joint_trajectory_controller after 6 sec
                                delayed_spawners,       # Spawn controllers after 10 sec
                                delayed_modify_node,     # Run node to modify the Solver params
                                
                                final_ready_notifier
                            ])

    '''return LaunchDescription(
                              [ur_ctrl_include] +  \
                              [delayed_stop_ctrl_action]  +   \
                              [*active_spawners, *inactive_spawners]
                             )'''