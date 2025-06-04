# tello_controller.py
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from time import time, sleep
from tello_msgs.srv import TelloAction
from rclpy.executors import MultiThreadedExecutor
from traceback import print_exc
from functools import partial
import tf_transformations

NUM_DRONES = 30

best_effort_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    depth=10
)

reliable_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    depth=10
)

class TelloController(Node):
    def __init__(self, drone_num):
        self.drone_num = drone_num
        super().__init__(f'tello_controller{drone_num}')
        self.cmd_pub = self.create_publisher(Twist, f'/drone{drone_num}/cmd_vel', reliable_qos)
        self.target_sub = self.create_subscription(Point, f'/drone{drone_num}/target', self.target_callback, best_effort_qos)

        odom_topics = [(i, f'/drone{i}/odom') for i in range(1, NUM_DRONES+1)]

        for i, topic in odom_topics:
            self.create_subscription(
                Odometry,
                topic,
                partial(self.odom_callback, i),
                best_effort_qos
            )

        self.timer = self.create_timer(0.1, self.navigate)
        # Target position
        self.target = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        }
        self.home_position = None
        self.odoms = {}

        self.client = self.create_client(TelloAction, f'/drone{drone_num}/tello_action')
        while not self.client.wait_for_service():
            self.get_logger().info('service not available, waiting again...')
        self.state = "start"
        self.delay = (None, None)

    def target_callback(self, msg: Point):
        try:
            self.target = {
                "x": msg.x,
                "y": msg.y,
                "z": msg.z
            }
            if self.target["z"] == -1.0:
                self.target = dict(self.home_position)
        except:
            print_exc()

    def odom_callback(self, i, msg: Odometry):
        try:
            self.odoms.update({str(i): {"pose": msg.pose.pose.position,
                                        "orientation": msg.pose.pose.orientation}})
        except:
            print_exc()
    

    def navigate(self):
        try:
            if self.delay[0] and time() - self.delay[0] < self.delay[1]:
                return
            self.delay = (None, None)
            # Takeoff
            cmd = Twist()
            action = TelloAction.Request()
            if self.state == "start":
                print("Takeoff")
                action.cmd = "takeoff"
                self.state = "flying"
            elif self.state == "flying":
                    #Keep drone pointing straight
                    orientation: Quaternion = self.odoms[str(self.drone_num)]["orientation"]
                    qx = orientation.x
                    qy = orientation.y
                    qz = orientation.z
                    qw = orientation.w
                    roll, pitch, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
                    cmd.angular.z = max(-0.5, min(0.5, -0.1 * yaw)) # yaw control is very broken...
                    
                    # Only correct angle if its way off
                    if abs(yaw) > 1:
                        #self.get_logger().info(f"{self.drone_num} {yaw}")
                        cmd.linear.x = 0.0
                        cmd.linear.y = 0.0
                        cmd.linear.z = 0.0
                    else:
                        pos: Point = self.odoms[str(self.drone_num)]["pose"]
                        if self.home_position is None:
                            self.home_position = {
                                "x": pos.x,
                                "y": pos.y,
                                "z": pos.z
                            }
                        # If to keep drones still at the start
                        errors = {
                            "x": self.target["x"] - pos.x if self.target["x"] else 0.0,
                            "y": self.target["y"] - pos.y if self.target["y"] else 0.0,
                            "z": self.target["z"] - pos.z if self.target["z"] else 0.0,
                        }
                        #print("ERRORS", self.drone_num, errors)
                        cmd.linear.x = max(-0.2, min(0.2, 0.05 * errors["x"]))
                        cmd.linear.y = max(-0.2, min(0.2, 0.05 * errors["y"]))
                        cmd.linear.z = max(-0.2, min(0.2, 0.05 * errors["z"]))
            if self.state == "land":
                print("Land")
                action.cmd = "land"
                self.state = "done"
            elif self.state == "done":
                pass
            if action.cmd:
                print(f"Action: {action.cmd}")
                cmd = Twist()
                future = self.client.call_async(action)
                self.delay = (time(), 5)
            self.cmd_pub.publish(cmd)
        except:
            print_exc()
        

def main(args=None):
    rclpy.init(args=args)
    nodes = [TelloController(i) for i in range(1, NUM_DRONES+1)]
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()