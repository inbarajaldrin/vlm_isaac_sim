import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from custom_actions.action import CartesianTrajectoryAct


# This function edits the "Text Color" or "Background Color" of terminal text using ANSI codes
def colorize(color_code, message):
    # TEXT::: color_code = 31<Red>, 32<Green>, 93<Yellow>, 34<Blue>, 36<Cyan>, 95<BrtMagenta>
    # BACKGRD:: color_code = 41<Red>, 42<Green>, 103<Yellow>, 44<Blue>, 46<Cyan>, 105<BrtMagenta>
    # \033[  <Initiate ANSI escape code>, and  \033[0m  <Reset formatting to default>
    return f"\033[{color_code}m{message}\033[0m"


class CartesianControlClient(Node):
    def __init__(self):
        super().__init__('action_client_tester')
        self.client = ActionClient(self,
                                   CartesianTrajectoryAct, 
                                   'follow_cartesian_traj')
        self.get_logger().info(colorize(42, "Running TEST Action Client"))

    def send_cartesian_trajectory(self, waypoints):
        self.get_logger().info(colorize(93, "Waiting for action server..."))
        self.client.wait_for_server()
        # Get the Goal Msg data type from .action file < geometry_msgs/Pose[] >
        goal_msg = CartesianTrajectoryAct.Goal()
        # Set the waypoints
        goal_msg.waypoints = waypoints
        # Send the goal messages to the Action Server
        self.get_logger().info(colorize(44, f"Sending {len(waypoints)} waypoints " +    \
                                            "to the action server..."))
        self._send_goal_future = self.client.send_goal_async(goal_msg, 
                                                    feedback_callback=self.feedback_callback)
        

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(colorize(34, 
                                        f"Current Pose: {feedback_msg.feedback.current_pose}")
                              )

def main(args=None):
    rclpy.init(args=args)
    node = CartesianControlClient()
    
    goal_poses = [{"position": Point(x=0.281+(0.005 * k), y=0.081, z=0.57939-(0.005 * k)),
                 #Point(x=1.0345, y=0.2608, z=0.2224),
                 "orientation": Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
                 #Quaternion(x=-0.0297, y=0.6974, z=0.7138, w=0.05556)
                 }
                 
                    for k in range(0,100)
                 ] 

    # Create a list of sample waypoints
    waypoints = []
    for i, goal_pose in enumerate(goal_poses):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position = goal_pose["position"]    # of Type 'Point'. Contains .x,.y & .z
        target_pose.pose.orientation = goal_pose["orientation"]

        waypoints.append(target_pose)

    node.send_cartesian_trajectory(waypoints)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
