# tello_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String, Int32
from time import time
from tello_msgs.srv import TelloAction

class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.x_error_sub = self.create_subscription(Int32, '/x_error', self.x_error_callback, 10)
        self.y_error_sub = self.create_subscription(Int32, '/y_error', self.y_error_callback, 10)
        self.timer = self.create_timer(1, self.navigate)
        self.x_error = 0
        self.y_error = 0
        self.error_update = 0
        self.client = self.create_client(TelloAction, '/tello_action')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.state = "start"

    def x_error_callback(self, msg):
        self.x_error = msg.data
        self.error_update = time()

    def y_error_callback(self, msg):
        self.y_error = msg.data
        self.error_update = time()

    def navigate(self):
        if self.state == "start":
            action = TelloAction.request()
            action.cmd = "{cmd: 'takeoff'}"
            self.future = self.client.call_async(action)
            rclpy.spin_until_future_complete(self, self.future)
            self.state = "flying"
            return
        
        elif self.state == "flying":
            action = TelloAction.request()
            action.cmd = "{cmd: 'land'}"
            self.future = self.client.call_async(action)
            rclpy.spin_until_future_complete(self, self.future)
            self.state = "done"
            return

        elif self.state == "done":
            self.cmd_pub.publish(Twist())
            return

        if time() - self.error_update > 5:
            self.cmd_pub.publish(Twist())
            return



        cmd = Twist()
        #cmd.linear.x = 0.3  # move forward
        #cmd.linear.y = -0.002 * error_x  # strafe left/right
        cmd.linear.z = -0.002 * self.y_error  # up/down
        cmd.angular.z = 0.1 * self.x_error

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TelloController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
