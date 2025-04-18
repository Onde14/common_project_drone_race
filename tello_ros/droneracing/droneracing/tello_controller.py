# tello_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String, Int32, Float32
from time import time
from tello_msgs.srv import TelloAction

CLOSENESS_TRESHOLD = 0.9

class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_state_pub = self.create_publisher(String, '/control_state', 10)
        self.gate_state_pub = self.create_publisher(String, '/gate_state', 10)
        self.x_error_sub = self.create_subscription(Int32, '/x_error', self.x_error_callback, 10)
        self.y_error_sub = self.create_subscription(Int32, '/y_error', self.y_error_callback, 10)
        self.closeness_sub = self.create_subscription(Float32, '/closeness', self.closeness_callback, 10)
        self.timer = self.create_timer(0.5, self.navigate)
        self.x_error = 0
        self.y_error = 0
        self.update_time = 0
        self.closeness = 0.0
        self.client = self.create_client(TelloAction, '/tello_action')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.state = "start"
        self.gate_state = "centering"
        self.fly_trough_start = 0
        self.FLY_TROUGH_TIME = 4
        self.gates_passed = 0
        self.GATES_TO_PASS = 4

    def x_error_callback(self, msg):
        self.x_error = msg.data
        self.update_time = time()

    def y_error_callback(self, msg):
        self.y_error = msg.data
        self.update_time = time()
    
    def closeness_callback(self, msg):
        self.closeness = msg.data
        self.update_time = time()

    def navigate(self):
        if time() - self.update_time > 5:
            self.cmd_pub.publish(Twist())
            return
        # Takeoff
        cmd = Twist()
        action = TelloAction.Request()
        if self.state == "start":
            print("Takeoff")
            action.cmd = "takeoff"
            self.state = "flying"
        # Fly trough gates
        elif self.state == "flying":
            if self.gate_state == "centering":
                print("Centering")
                print(self.closeness)
                if self.x_error == -1 or self.y_error == -1: # Center not detected
                    cmd.angular.z = 0.01 #  rotate in place
                else:
                    # Rotate to minimize error
                    cmd.linear.z = -0.002 * self.y_error
                    cmd.angular.z = 0.001 * self.x_error
                    # Move closer when error is small
                    if max(abs(self.x_error), abs(self.y_error)) < 50:
                        cmd.linear.x = 0.1
                    if self.closeness > CLOSENESS_TRESHOLD:
                        if self.gates_passed == self.GATES_TO_PASS: # Centered on stop sign!
                            self.state = "land"
                            self.gate_state = "done"
                        self.gate_state = "go_trough"
            # Fly straight
            elif self.gate_state == "go_trough":
                if not self.fly_trough_start:
                    print("go_trough trigger")
                    self.fly_trough_start = time()
                cmd.linear.x = 0.2
                if time() - self.fly_trough_start > self.FLY_TROUGH_TIME:
                    self.gate_state == "centering"
                    self.gates_passed += 1
                    print(f"passed gate {self.gates_passed}/{self.GATES_TO_PASS}")

        if self.state == "land":
            print("Land")
            action.cmd = "land"
            self.state = "done"
        elif self.state == "done":
            pass
        if action.cmd:
            print(f"Action: {action.cmd}")
            cmd = Twist()
            self.future = self.client.call_async(action)
            rclpy.spin_until_future_complete(self, self.future)
        self.cmd_pub.publish(cmd)
        self.control_state_pub.publish(String(data=self.state))
        self.gate_state_pub.publish(String(data=self.gate_state))
        print(f"cmd:{cmd}")
        print(f"gates_passed:{self.gates_passed}/{self.GATES_TO_PASS}, state: {self.state}, gate_state:{self.gate_state}")
        

def main(args=None):
    rclpy.init(args=args)
    node = TelloController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()