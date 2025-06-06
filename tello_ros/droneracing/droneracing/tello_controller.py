# tello_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String, Int32, Float32
from time import time
from tello_msgs.srv import TelloAction
from time import sleep

CLOSENESS_TRESHOLD = 0.8

class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_state_pub = self.create_publisher(String, '/control_state', 10)
        self.gate_state_pub = self.create_publisher(String, '/gate_state', 10)
        self.gates_passed_pub = self.create_publisher(Int32, '/gates_passed', 10)
        self.x_error_sub = self.create_subscription(Int32, '/x_error', self.x_error_callback, 10)
        self.y_error_sub = self.create_subscription(Int32, '/y_error', self.y_error_callback, 10)
        self.closeness_sub = self.create_subscription(Float32, '/closeness', self.closeness_callback, 10)
        self.timer = self.create_timer(0.1, self.handle_action) 
        self.x_error = 0
        self.y_error = 0
        self.update_time = 0
        self.closeness = 0.0
        self.takeoff_client = self.create_client(TelloAction, '/tello_action')
        self.land_client = self.create_client(TelloAction, '/tello_action')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.state = "start"
        self.gate_state = "centering"
        self.fly_trough_start = 0
        self.FLY_TROUGH_TIME = 4
        self.gates_passed = 0
        self.GATES_TO_PASS = 4
        self.GOUPTIME = 3
        self.goupstart = 0
        self.gategoupstart = 0
        self.TAG_GATE = 3
        self.takeoff_started = False
        self.takeoff_time = 5

    def x_error_callback(self, msg):
        self.x_error = msg.data
        self.update_time = time()

    def y_error_callback(self, msg):
        self.y_error = msg.data
        self.update_time = time()
    
    def closeness_callback(self, msg):
        self.closeness = msg.data
        self.update_time = time()

    def tello_response(self, msg):
        self.tello_response_msg = msg.data
        self.update_time = time()

    def send_action(self, cmd_str):
        request = TelloAction.Request()
        request.cmd = cmd_str
        self.future = self.takeoff_client.call_async(request)
        self.future.add_done_callback(self.handle_action_response)

    def handle_action_response(self, future):
        try:
            result = future.result()
            self.state == "goup"
            self.get_logger().info(f"Service response: {result}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    async def handle_action(self):
        print("Choosing action and state is ", self.state)
        if self.state == "waiting for takeoff":
            pass
        else:
            if self.state == "start":
                self.state == self.takeoff()
                self.sleeper()
            elif self.state != "land":
                self.navigate()
            elif self.state == "land":
                self.land()


    def takeoff(self):
        if self.takeoff_started == False:
            self.state = "waiting for takeoff"
            self.takeoff_started == True
        print("Takeoff")
        action = TelloAction.Request()
        self.takeoff_start = time()
        action.cmd = "takeoff"
        future = self.takeoff_client.call_async(action)
        sleep(4)

    def land(self):
        print("Land")
        action = TelloAction.Request()
        action.cmd = "land"
        future = self.land_client.call_async(action)
        sleep(4)
        self.state = "done"
        self.timer.cancel()

    def sleeper(self):
        sleep(4)
        self.state = "goup"

    def navigate(self):
        print("getting state")
        cmd = Twist()
        action = TelloAction.Request()

        if self.state == "goup":
            cmd.linear.z = 0.5
            if not self.goupstart:
                self.goupstart = time()
            if time() - self.goupstart > self.GOUPTIME:
                self.state = "flying"
        # Fly trough gates
        elif self.state == "flying":
            if self.gate_state == "centering":
                print("Centering")
                if self.x_error == -1 or self.y_error == -1: # Center not detected
                    cmd.angular.z = -0.2 #  rotate in place
                else:
                    # Rotate to minimize error
                    cmd.linear.z = 0.001 * self.y_error
                    cmd.angular.z = 0.0005 * self.x_error
                    # Move closer when error is small
                    print("closeness: ", self.closeness)
                    print("max: ", max(abs(self.x_error), abs(self.y_error)))
                    if max(abs(self.x_error), abs(self.y_error)) < 100:
                        cmd.linear.x = 0.15
                        if self.gates_passed == self.GATES_TO_PASS: # Centered on stop sign!
                            if self.state != "land":
                                if self.closeness > 0.33:
                                    cmd.linear.x = 0.0
                                    cmd.linear.z = 0.0
                                    cmd.angular.z = 0.0
                                    self.state = "land"
                                    self.gate_state = "done"
                        else:
                            if self.closeness > CLOSENESS_TRESHOLD:
                                self.gate_state = "go_trough"
            # Fly straight
            elif self.gate_state == "go_trough" and self.gates_passed != self.GATES_TO_PASS:
                if not self.fly_trough_start:
                    print("go_trough trigger")
                    self.fly_trough_start = time()
                cmd.linear.x = 0.3
                cmd.linear.z = -0.2
                fly_throught_time = self.FLY_TROUGH_TIME
                if self.gates_passed == self.TAG_GATE:
                    fly_throught_time += 1.5
                if time() - self.fly_trough_start > fly_throught_time:
                    self.gate_state = "goup"
                    self.gates_passed += 1
                    self.fly_trough_start = 0
                    print(f"passed gate {self.gates_passed}/{self.GATES_TO_PASS}")
            elif self.gate_state == "goup":
                cmd.linear.z = 0.5
                if not self.gategoupstart:
                    self.gategoupstart = time()
                goup_time = 1.5
                if self.TAG_GATE == self.gates_passed:
                    goup_time = 2.5
                elif self.gates_passed == 2:
                    goup_time = 4
                if time() - self.gategoupstart > goup_time:
                    self.gategoupstart = 0
                    self.gate_state = "centering"
        elif self.state == "done":
            pass
        if cmd:
            self.cmd_pub.publish(cmd)
        self.control_state_pub.publish(String(data=self.state))
        self.gate_state_pub.publish(String(data=self.gate_state))
        self.gates_passed_pub.publish(Int32(data=self.gates_passed))
        print(f"gates_passed:{self.gates_passed}/{self.GATES_TO_PASS}, state: {self.state}, gate_state:{self.gate_state}")
        

def main(args=None):
    rclpy.init(args=args)
    node = TelloController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()






