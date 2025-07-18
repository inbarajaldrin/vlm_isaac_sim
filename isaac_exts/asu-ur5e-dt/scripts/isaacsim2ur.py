#!/usr/bin/env python3
"""
Simple ROS2 subscriber that listens to joint commands and moves the robot.
This script receives joint position commands and executes them using the 
scaled_joint_trajectory_controller.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JointCommandListener(Node):
    def __init__(self):
        super().__init__('joint_command_listener')
        
        # Joint names for UR5e
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        # Create subscriber for joint commands
        self.subscription = self.create_subscription(
            String,
            '/robot_joint_command',  # Same topic as the publisher
            self.joint_command_callback,
            10
        )
        
        # Create action client for trajectory controller
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info("Joint Command Listener started!")
        self.get_logger().info("Listening on topic: /robot_joint_command")
        self.get_logger().info("Waiting for action server...")
        
        # Wait for action server
        if self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info("Action server connected!")
        else:
            self.get_logger().error("Action server not available!")

    def joint_command_callback(self, msg):
        """Callback function when joint command is received"""
        try:
            # Parse the comma-separated joint angles
            joint_angles_str = msg.data
            joint_angles = [float(x.strip()) for x in joint_angles_str.split(',')]
            
            if len(joint_angles) != 6:
                self.get_logger().error(f"Expected 6 joint angles, got {len(joint_angles)}")
                return
            
            self.get_logger().info(f"Received joint command: {joint_angles}")
            
            # Send trajectory to robot
            self.send_trajectory(joint_angles)
            
        except ValueError as e:
            self.get_logger().error(f"Failed to parse joint angles: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")

    def send_trajectory(self, joint_angles, duration=3.0):
        """Send trajectory to the robot"""
        try:
            # Create trajectory goal
            goal = FollowJointTrajectory.Goal()
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = joint_angles
            point.velocities = [0.0] * 6
            point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
            
            trajectory.points = [point]
            goal.trajectory = trajectory
            goal.goal_time_tolerance = Duration(sec=1)
            
            # Send goal
            self.get_logger().info(f"Sending trajectory: {joint_angles}")
            future = self.action_client.send_goal_async(goal)
            future.add_done_callback(self.goal_response_callback)
            
        except Exception as e:
            self.get_logger().error(f"Error sending trajectory: {e}")

    def goal_response_callback(self, future):
        """Callback when goal response is received"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Trajectory goal rejected")
                return
            
            self.get_logger().info("Trajectory goal accepted - robot moving!")
            
            # Get result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
            
        except Exception as e:
            self.get_logger().error(f"Error in goal response: {e}")

    def result_callback(self, future):
        """Callback when trajectory execution is complete"""
        try:
            result = future.result().result
            if result.error_code == 0:
                self.get_logger().info("Robot movement completed successfully!")
            else:
                self.get_logger().error(f"Robot movement failed with error code: {result.error_code}")
                
        except Exception as e:
            self.get_logger().error(f"Error in result callback: {e}")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        # Create and run the listener node
        listener = JointCommandListener()
        
        print("Joint Command Listener is running...")
        print("Send joint commands using:")
        print("ros2 topic pub --once /robot_joint_command std_msgs/String \"{data: '0.0,-1.57,1.57,-1.57,-1.57,0.0'}\"")
        print("Press Ctrl+C to stop")
        
        rclpy.spin(listener)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'listener' in locals():
            listener.destroy_node()
        rclpy.shutdown()
        print("Goodbye!")


if __name__ == '__main__':
    main()
