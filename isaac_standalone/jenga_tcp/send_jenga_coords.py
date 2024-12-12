#!/usr/bin/env python3

import socket
import sys
import rclpy
from rclpy.node import Node

class JengaBlockSender(Node):
    def __init__(self):
        super().__init__('jenga_block_sender')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('localhost', 65432)  # Adjust as necessary

    def connect(self):
        self.get_logger().info('Connecting to {} port {}'.format(*self.server_address))
        self.client_socket.connect(self.server_address)

    def send_data(self, message):
        self.get_logger().info(f'Sending data: {message}')
        self.client_socket.sendall(message.encode('utf-8'))

    def close_connection(self):
        self.client_socket.close()
        self.get_logger().info('Connection closed')

def main(args=None):
    rclpy.init(args=args)
    node = JengaBlockSender()

    try:
        node.connect()

        # Check command line input for play, reset, or coordinates
        if len(sys.argv) == 2:
            command = sys.argv[1].lower()
            if command == "reset":
                node.send_data("reset")
            elif command == "play":
                node.send_data("play")
            else:
                raise ValueError("Invalid input. Provide 'play', 'reset', or coordinates (x y z [theta]).")
        elif len(sys.argv) == 5:  # x, y, z, theta provided
            x, y, z, theta = sys.argv[1:5]
            node.send_data(f"X: {x}, Y: {y}, Z: {z}, Theta: {theta}")
        elif len(sys.argv) == 4:  # x, y, z provided, default theta to 0
            x, y, z = sys.argv[1:4]
            node.send_data(f"X: {x}, Y: {y}, Z: {z}, Theta: 0")
        else:
            raise ValueError("Invalid input. Provide 'play', 'reset', or coordinates (x y z [theta]).")

    except Exception as e:
        node.get_logger().error(f'Error: {str(e)}')
    finally:
        node.close_connection()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
