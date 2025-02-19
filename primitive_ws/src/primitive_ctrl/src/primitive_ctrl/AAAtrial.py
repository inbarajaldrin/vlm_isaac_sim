import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        # Subscribe to the PoseArray topic
        self.subscription = self.create_subscription(
            PoseArray,
            '/jenga_detection_poses',  # Replace with your actual topic name
            self.pose_array_callback,
            10  # Queue size
        )
        self.subscription  # prevent unused variable warning
        self.poses_list = []

    def pose_array_callback(self, msg):
        # Callback that gets called when a PoseArray message is received
        self.get_logger().info(f'Received PoseArray with {len(msg.poses)} poses.')

        # Extract individual poses and store them in a list
        self.poses_list = msg.poses
        self.get_logger().info(f'Individual poses: {self.poses_list}')

        # Optionally, you can process the poses individually if needed
        for pose in self.poses_list:
            self.get_logger().info(f'Pose: Position({pose.position.x}, {pose.position.y}, {pose.position.z}), '
                                   f'Orientation({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})')

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()

    # Spin to keep the node active and process messages
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
