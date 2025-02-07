import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import rclpy.callback_groups
from rclpy.node import Node
from rclpy.action import ActionServer
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from custom_actions.action import CartesianTrajectoryAct

import time


# This function edits the "Text Color" or "Background Color" of terminal text using ANSI codes
def colorize(color_code, message):
    # TEXT::: color_code = 31<Red>, 32<Green>, 93<Yellow>, 34<Blue>, 36<Cyan>, 95<BrtMagenta>
    # BACKGRD:: color_code = 41<Red>, 42<Green>, 103<Yellow>, 44<Blue>, 46<Cyan>, 105<BrtMagenta>
    # \033[  <Initiate ANSI escape code>, and  \033[0m  <Reset formatting to default>
    return f"\033[{color_code}m{message}\033[0m"


class Move3DActionServer(Node):
    def __init__(self):
        super().__init__("move3d_action_server")
        self.get_logger().info(colorize(42, "Running Cartesian Trajectory Action Server"))
        # Create 3 Groups for each of the Callbacks- THIS IS FOR MULTI-THREADING
        self.action_callback_group = MutuallyExclusiveCallbackGroup()
        self.sub_callback_group = MutuallyExclusiveCallbackGroup()
        self.pub_callback_group = MutuallyExclusiveCallbackGroup()
        # Create Action Server
        self.action_server = ActionServer(self,
                                          CartesianTrajectoryAct,
                                          "follow_cartesian_traj",
                                          self.send_traj_callback,
                                          callback_group=self.action_callback_group
                                         )
        # Create Publisher for Target Frame
        self.publisher = self.create_publisher(PoseStamped, 
                                               "/cartesian_motion_controller/target_frame", 
                                               10,
                                               callback_group=self.sub_callback_group
                                              )
        # Create Subscriber for providing current pose's feedback
        self.subscriber = self.create_subscription(PoseStamped,
                                                   "/cartesian_motion_controller/current_pose",
                                                   self.current_pose_callback, 
                                                   10,
                                                   callback_group=self.pub_callback_group
                                                  )

    
    def send_traj_callback(self, goal_handle):
        self.get_logger().info(colorize(44, "Received goal with " +    \
                                            f"{len(goal_handle.request.waypoints)} waypoints"
                                       )
                              )
        # Get the waypoints from the client
        for i, pose in enumerate(goal_handle.request.waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(colorize(41, "Goal Canceled!!!!!!!"))
                return CartesianTrajectoryAct.Result(success=False)
            # Publish the target
            self.publisher.publish(pose)
            self.get_logger().info(colorize(95, f"Moving to waypoint {i+1}"))
            '''# Give a 5-second gap between trajectory points
            timeout = time.time() + 5
            while time.time() < timeout:'''
            # Send feedback back to the client
            if self.current_pose:
                feedback = CartesianTrajectoryAct.Feedback()
                feedback.current_pose = self.current_pose
                goal_handle.publish_feedback(feedback)
            else:
                self.get_logger().info(colorize(103, "Waiting for Goal..."))
        # Send the Success goal 
        goal_handle.succeed()
        self.get_logger().info(colorize(42, "Successfully followed Cartesian path!"))
        return CartesianTrajectoryAct.Result(success=True)

    def current_pose_callback(self, msg):
        self.current_pose = msg     #.data


def main(args=None):
    rclpy.init(args=args)
    # Create a node with a Multi-Threaded Executor
    node = Move3DActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # Start Multi-threaded Executor
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()