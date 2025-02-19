import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import Pose2D, PoseStamped, Point, Quaternion

from custom_actions.action import CartesianTrajectoryAct
from custom_actions.msg import GoalPose2D

import sys
import casadi as ca
import numpy as np
from pytransform3d.rotations import quaternion_from_euler

from .gen_push_traj import gen_push_cartesian_trajectory


# This function edits the "Text Color" or "Background Color" of terminal text using ANSI codes
def colorize(color_code, message):
    # TEXT::: color_code = 31<Red>, 32<Green>, 93<Yellow>, 34<Blue>, 36<Cyan>, 95<BrtMagenta>
    # BACKGRD:: color_code = 41<Red>, 42<Green>, 103<Yellow>, 44<Blue>, 46<Cyan>, 105<BrtMagenta>
    # \033[  <Initiate ANSI escape code>, and  \033[0m  <Reset formatting to default>
    return f"\033[{color_code}m{message}\033[0m"


class MoverClientMain(Node):
    def __init__(self):
        super().__init__('multiple_controller_action_client')
        # Create the Action Client to send the trajectory commands to the Action Server
        self.client = ActionClient(self,
                                   CartesianTrajectoryAct, 
                                   'follow_cartesian_traj')
        self.get_logger().info(colorize(42, 
                                   "Running Main Action Client ----- Listening to <TOPIC NAME>"))
        self.sub_current_pose = self.create_subscription(
                                                    Pose2D,
                                                    '/current_pose_2d',
                                                    self.current_pose_callback,
                                                    10  # Queue size
                                                    )
        self.sub_goal_pose = self.create_subscription(
                                                    GoalPose2D,
                                                    '/goal_pose_2d',
                                                    self.goal_callback,
                                                    10  # Queue size
                                                    )

    def current_pose_callback(self, current_msg):
        self.get_logger().info(colorize(35, f"Controller to use: {current_msg.controller_to_use}," +
                                            f"Pose: {current_msg.pose}"))

    def goal_callback(self, goal_msg):
        """Callback function for goal subscriber."""
        self.get_logger().info(colorize(35, f"Received goal: {goal_msg.controller_to_use},"  +
                                            f"Pose: {goal_msg.pose}"))
        # Get the controller to use
        if goal_msg.controller_to_use in ["push", "Push", "PUSH"]:
            print(goal_msg.pose)
            # Generate waypoints based on the received message
            waypoints = gen_push_cartesian_trajectory(
                                                        xb = 0.2,
                                                        yb = 0.2,
                                                        zb = 0.1,
                                                        thetab = 1.57,
                                                        xf = goal_msg.pose.x,
                                                        yf = goal_msg.pose.y,
                                                        thetaf = 0.785,
                                                     )
        if goal_msg.controller_to_use in ["cartesian", "CARTESIAN", "move3d", "MOVE3D"]:
            print(goal_msg.pose)
        
        # Send the generated waypoints to the action server
        self.send_cartesian_trajectory(waypoints)

    def send_cartesian_trajectory(self, waypoints): 
        """Send the Cartesian Trajectory to the selected action server"""
        self.get_logger().info(colorize(93, "Waiting for action server..."))
        self.client.wait_for_server()

        # Get the Goal Msg data type from .action file { geometry_msgs/Pose[] }
        goal_msg = CartesianTrajectoryAct.Goal()
        # Set the waypoints
        goal_msg.waypoints = waypoints
        # Send the goal messages to the Action Server
        self.get_logger().info(colorize(44, f"Sending {len(waypoints)} waypoints " +    \
                                            "to the action server..."))
        self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(colorize(34, 
                                        f"Current Pose: {feedback_msg.feedback.current_pose}")
                              )


def main(args=None):
    rclpy.init(args=args)
    node = MoverClientMain()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()