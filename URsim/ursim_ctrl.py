#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from action_msgs.msg import GoalStatus

class URSimTestNode(Node):
    def __init__(self):
        super().__init__("ursim_test_node")

        self.joints = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        self.client = ActionClient(self, FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        self.get_logger().info("Waiting for action server...")
        self.client.wait_for_server()

        self.send_goal()

    def send_goal(self):
        traj = JointTrajectory()
        traj.joint_names = self.joints

        point = JointTrajectoryPoint()
        point.positions = [0.5, -1.2, 1.8, -1.2, 1.57, 0.0]
        point.time_from_start = Duration(sec=4)
        traj.points.append(point)


        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=1)
        goal.goal_tolerance = [
            JointTolerance(name=j, position=0.01, velocity=0.01) for j in self.joints
        ]

        self.get_logger().info("Sending trajectory goal...")
        self._goal_future = self.client.send_goal_async(goal)
        self._goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by URSim.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Trajectory executed successfully!")
        else:
            self.get_logger().error(f"Trajectory failed. Status: {status}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = URSimTestNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
