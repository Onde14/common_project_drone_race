# tello_controller.py
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32, Float32
from time import time
from tello_msgs.srv import TelloAction
from functools import partial
from rclpy.executors import MultiThreadedExecutor

NUM_DRONES = 20

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
        print(f"PÖÖÖÖÖÖÖÖÖÖÖÖÖ{drone_num}")
        super().__init__(f'tello_controller{drone_num}')
        self.cmd_pub = self.create_publisher(Twist, f'/drone{drone_num}/cmd_vel', reliable_qos)
        self.target_sub = self.create_subscription(PoseStamped, f'/drone{drone_num}/target', self.target_callback, best_effort_qos)

        odom_topics = [(i, f'/drone{i}/odom') for i in range(1, NUM_DRONES+1)]

        for i, topic in odom_topics:
            self.create_subscription(
                Odometry,
                topic,
                partial(self.odom_callback, i),
                best_effort_qos
            )

        self.timer = self.create_timer(0.2, self.navigate)
        # Target position
        self.target = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        }
        self.odoms = {}

        self.client = self.create_client(TelloAction, f'/drone{drone_num}/tello_action')
        while not self.client.wait_for_service():
            self.get_logger().info('service not available, waiting again...')
        self.state = "start"

    def target_callback(self, msg: PoseStamped):
        self.target = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z
        }

    def odom_callback(self, msg: Odometry, i):
        self.odoms.update({str(i): {"pose": msg.pose.pose.position,
                                    "orientation:": msg.pose.pose.orientation}})
    

    def navigate(self):
        # Takeoff
        cmd = Twist()
        action = TelloAction.Request()
        if self.state == "start":
            print("Takeoff")
            action.cmd = "takeoff"
            self.state = "flying"
        elif self.state == "flying":
            pass
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