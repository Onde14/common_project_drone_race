#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep


class LetterPublisher(Node):
    def __init__(self):
        super().__init__('letter_publisher')
        self.publisher_ = self.create_publisher(String, '/char', 10)

    def publish_input(self, input_text: str):
        msg = String()
        msg.data = input_text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{input_text}"')


def main(args=None):
    rclpy.init(args=args)
    node = LetterPublisher()

    try:
        while rclpy.ok():
            user_input = input("Enter a letter or word to publish (or 'exit' to quit): ").strip().upper()
            if user_input.lower() == 'exit':
                break
            elif user_input:
                for i in user_input:
                    node.publish_input(i)
                    sleep(30)
                    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
