# tello_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String

class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gate_sub = self.create_subscription(PointStamped, '/gate_position', self.gate_callback, 10)
        self.type_sub = self.create_subscription(String, '/gate_type', self.type_callback, 10)

        self.gate_position = None
        self.gate_type = None
        self.image_center = (480, 360)  # assuming 960x720 image

    def type_callback(self, msg):
        self.gate_type = msg.data

    def gate_callback(self, msg):
        self.gate_position = (msg.point.x, msg.point.y)
        self.navigate()

    def navigate(self):
        if self.gate_position is None:
            return

        cx, cy = self.gate_position
        ix, iy = self.image_center

        error_x = cx - ix
        error_y = cy - iy

        cmd = Twist()
        cmd.linear.x = 0.3  # move forward
        cmd.linear.y = -0.002 * error_x  # strafe left/right
        cmd.linear.z = -0.002 * error_y  # up/down
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TelloController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
