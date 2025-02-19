import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseArray, Pose2D, PoseStamped, Point, Quaternion

from custom_actions.action import CartesianTrajectoryAct
from custom_actions.msg import GoalPose2D

import sys
from collections import deque
import casadi as ca
import numpy as np
from pytransform3d.rotations import quaternion_from_euler, euler_from_quaternion

from .gen_push_traj import gen_push_cartesian_trajectory


# This function edits the "Text Color" or "Background Color" of terminal text using ANSI codes
def colorize(color_code, message):
    # TEXT::: color_code = 31<Red>, 32<Green>, 93<Yellow>, 34<Blue>, 36<Cyan>, 95<BrtMagenta>
    # BACKGRD:: color_code = 41<Red>, 42<Green>, 103<Yellow>, 44<Blue>, 46<Cyan>, 105<BrtMagenta>
    # \033[  <Initiate ANSI escape code>, and  \033[0m  <Reset formatting to default>
    return f"\033[{color_code}m{message}\033[0m"


# OFFSET VARIABLES FROM ARUCO TO BASE_LINK
X_OFFSET = 0.790
Y_OFFSET = 0.822
Z_HEIGHT_PUSH = 0.135
Z_HEIGHT_SAFE = 0.17
FINAL_X_Y = (0.2, 0.5)
FINAL_THETA = 1.5707
# Set the universal safe position to return to
safe_pos = Pose()
safe_pos.position = Point(x=FINAL_X_Y[0], y=FINAL_X_Y[1], 
                                z=Z_HEIGHT_SAFE)
safe_pos.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


class MoverClientMain(Node):
    def __init__(self):
        super().__init__('push_controller_action_client')
        # Create publisher
        self.publisher = self.create_publisher(
                                PoseStamped, 
                                "/cartesian_motion_controller/target_frame", 
                                10)
        # FIRST -> Move Robot to univeral safe position 
        self.move3d_command(safe_pos)
        # Create the Action Client to send the trajectory commands to the Action Server
        self.client = ActionClient(self,
                                   CartesianTrajectoryAct, 
                                   'follow_cartesian_traj')
        self.get_logger().info(colorize(42, 
                                   "Running Main Action Client ----- Listening to <TOPIC NAME>"))
        # Create subscriber
        self.sub_current_pose = self.create_subscription(
                                                    PoseArray,
                                                    '/jenga_detection_poses',
                                                    self.current_poses_callback,
                                                    10  # Queue size
                                                    )
        timer_period = 0.1
        self.message_count = 0

    def current_poses_callback(self, current_msg):
        self.poses_deque = deque(maxlen=2)
        for pose in current_msg.poses:
            self.get_logger().info(colorize(93, f"Found Pose: {pose}"))
            pose.position.x = pose.position.x - X_OFFSET
            pose.position.y = pose.position.y + Y_OFFSET
            self.poses_deque.append(pose)
            self.get_logger().info(colorize(35, f"Modified Pose: {pose}"))
        
        # Listen Thrice
        self.message_count += 1
        if self.message_count >= 3:
            self.message_count = 0
            # Shutdown the node to stop listening
            #self.sub_current_pose.destroy()
        
        # Generate Waypoints and the Send Trajectory to Action Server
        for pose in self.poses_deque:
            final_waypoints = []
            print(pose)
            # Move 3D to safe height
            self.move3d_command(pose, Z_HEIGHT_SAFE)
            # Move 3D to push height
            #self.move3d_command(pose, Z_HEIGHT_PUSH)
            # Generate PUSH trajectory
            #final_waypoints = self.generate_push_waypoints(pose, Z_HEIGHT_PUSH)
            # Send the generated PUSH waypoints to the action server
            #self.send_cartesian_trajectory(final_waypoints)
            # Final Move 3D to safe height
            #self.move3d_command(safe_pos, Z_HEIGHT_SAFE)
            print()
            print(final_waypoints)


    def generate_push_waypoints(self, current_obj_pose, z_height):
        #self.get_logger().info(colorize(35, f"Target Pose: {goal_msg}"))
        # -------- START the PUSH controller
        # Generate waypoints based on the received message
        waypoints = gen_push_cartesian_trajectory(
                                xb = current_obj_pose.position.x,
                                yb = current_obj_pose.position.y,
                                zb = z_height,
                                thetab = euler_from_quaternion( 
                                            [current_obj_pose.orientation.w,
                                             current_obj_pose.orientation.x,
                                             current_obj_pose.orientation.y,
                                             current_obj_pose.orientation.z],
                                             i=0,j=1,k=2,
                                             extrinsic=True
                                         )[2],                     #1.57,
                                xf = FINAL_X_Y[0], #goal_msg.position.x,
                                yf = FINAL_X_Y[1], #goal_msg.position.y,
                                thetaf = FINAL_THETA)
    
    def move3d_command(self, goal_pose, z_height=Z_HEIGHT_SAFE):
        # Create a target pose to publish
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position = Point(x=goal_pose.position.x,
                                          y=goal_pose.position.y,
                                          z=z_height
                                         )    # of Type 'Point'. Contains .x,.y & .z
        target_pose.pose.orientation = goal_pose.orientation     # of Type 'Quaternion'. Contains .w,.x,.y & .z
        # Publish the target
        self.publisher.publish(target_pose)
        self.get_logger().info(colorize(32, 
                                        "Publishing Target Pose: " +
                                        f" {target_pose.pose.position} " +
                                        f" {target_pose.pose.orientation} ")
                              )


    def send_cartesian_trajectory(self, waypoints): 
        """Send the Cartesian Trajectory to the selected action server"""
        self.get_logger().info(colorize(93, "Waiting for action server..."))
        self.client.wait_for_server()

        # Get the Goal Msg data type from .action file { geometry_msgs/Pose[] }
        goal_msg = CartesianTrajectoryAct.Goal()
        # Set the waypoints
        goal_msg.waypoints = waypoints
        # Send the goal messages to the Action Server
        self.get_logger().info(colorize(44, f"Sending {len(waypoints)} PUSH waypoints " +    \
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




'''thetaf = euler_from_quaternion( 
                                [goal_msg.orientation.w,
                                    goal_msg.orientation.x,
                                    goal_msg.orientation.y,
                                    goal_msg.orientation.z],
                                    i=0,j=1,k=2,
                                    extrinsic=True
                                )[2],'''