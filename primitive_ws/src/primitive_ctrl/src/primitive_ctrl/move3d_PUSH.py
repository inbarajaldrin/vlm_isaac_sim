'''
RUN INSTRUCTIONS: 
ros2 run primitive_ctrl move3d

Publish End-effector Position to topic:
    /target_frame  [Type: geometry_msgs/msg/PoseStamped]
'''

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from geometry_msgs.msg import PoseStamped, Point, Quaternion

import time
import math 
from pytransform3d.rotations import quaternion_from_euler

# This function edits the "Text Color" or "Background Color" of terminal text using ANSI codes
def colorize(color_code, message):
    # TEXT::: color_code = 31<Red>, 32<Green>, 93<Yellow>, 34<Blue>, 36<Cyan>, 95<BrtMagenta>
    # BACKGRD:: color_code = 41<Red>, 42<Green>, 103<Yellow>, 44<Blue>, 46<Cyan>, 105<BrtMagenta>
    # \033[  <Initiate ANSI escape code>, and  \033[0m  <Reset formatting to default>
    return f"\033[{color_code}m{message}\033[0m"


class Move3DPublisher(Node):
    def __init__(self):
        super().__init__("move3d_2")
        self.publisher = self.create_publisher(PoseStamped, "/cartesian_motion_controller/target_frame", 10)
        timer_period = 0.1
        self.i = 0
        self.target = 50
        # OFFSETS
        x_offset = 0.790 + 0.02
        y_offset = 0.822 + 0.02
        z=0.135

        # ---------- Move Pose
        x_detect = 246
        y_detect = -260
        angle = 40
        # ---------- Move Pose

        quat = quaternion_from_euler([math.pi, 0, angle*math.pi/180], 0,1,2, extrinsic=True)
        self.goal_pose = {"position": Point(x=(x_detect/1000)-x_offset, 
                                            y=(y_detect/1000)+y_offset, 
                                            z=z),   #Point(x=0.281, y=0.081, z=0.57939),
                          #Point(x=1.0345, y=0.2608, z=0.2224),
                          "orientation": Quaternion(x=quat[1], y=quat[2], 
                                                    z=quat[3], w=quat[0])
                          #Quaternion(x=-0.707, y=0.707, z=0.0, w=0.0)
                          #Quaternion(x=-0.9238, y=0.38268, z=0.0, w=0.0)
                          #Quaternion(x=-0.0297, y=0.6974, z=0.7138, w=0.05556)
                         } 
        # Create Timed Callback to publish topic
        self.timer = self.create_timer(timer_period, self.send_cartesian_trajectory)
        

    def send_cartesian_trajectory(self):
        if self.i >= self.target:
            #self.kill_node()
            self.get_logger().info(colorize(41, "Done Publishing Target Pose: "))
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()
            return

        # Create a target pose to publish
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position = self.goal_pose["position"]    # of Type 'Point'. Contains .x,.y & .z
        target_pose.pose.orientation = self.goal_pose["orientation"]     # of Type 'Quaternion'. Contains .w,.x,.y & .z
        # Publish the target
        self.publisher.publish(target_pose)
        self.get_logger().info(colorize(32, 
                                        "Publishing Target Pose: " +
                                        f" {target_pose.pose.position} " +
                                        f" {target_pose.pose.orientation} ")
                              )
        self.i += 1
    
    def kill_node(self):
        self.get_logger().info(colorize(41, "Done Publishing Target Pose: "))
        self.timer.cancel()
        return
        


def main(args=None):
    rclpy.init(args=args)
    move3d_publisher = Move3DPublisher()
    try:
        rclpy.spin(move3d_publisher)
    except KeyboardInterrupt:
        move3d_publisher.get_logger().info("Node interrupted. Shutting down.")
    finally:
        if move3d_publisher.timer.is_canceled():
            move3d_publisher.get_logger().info(colorize(41,
                                                  "Node stopped after reaching target position.")
                                              )
        move3d_publisher.destroy_node()
        rclpy.shutdown()
        print("\nNode has shut down. Terminal is ready for further use.")


if __name__ == "__main__":
    main()



'''
def main(args=None):
    rclpy.init(args=args)
    move3d_publisher = Move3DPublisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(move3d_publisher, timeout_sec=0.1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        move3d_publisher.get_logger().info("Node interrupted. Shutting down.")
    finally:
        if rclpy.ok():
            move3d_publisher.destroy_node()
            rclpy.shutdown()
        print("\nNode has shut down. Terminal is ready for further use.")
'''