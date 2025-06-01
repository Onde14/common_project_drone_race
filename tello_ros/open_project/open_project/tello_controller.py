# tello_controller.py
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from time import time, sleep
from tello_msgs.srv import TelloAction
from traceback import print_exc
from functools import partial
import tf_transformations
import math


NUM_DRONES = 15

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
        super().__init__(f'tello_controller{drone_num}')
        self.drone_num = drone_num
        self.get_logger().info(f"Started node tello_controller{drone_num}")
        self.cmd_pub = self.create_publisher(Twist, f'/drone{drone_num}/cmd_vel', reliable_qos)
        self.target_sub = self.create_subscription(Point, f'/drone{drone_num}/target', self.target_callback, best_effort_qos)
        self.position_status = "No target"
        self.pos_status_pub = self.create_publisher(String, f'/drone{drone_num}/status', reliable_qos)


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
    
    def calc_repelling(self):
        x_repel = 0.0
        y_repel = 0.0
        z_repel = 0.0
        og: Point = self.odoms[str(self.drone_num)]["pose"]
        for i in range(1, NUM_DRONES+1):
            if self.drone_num != i:
                comp: Point = self.odoms[str(i)]["pose"]
                if math.sqrt((og.x-comp.x)**2 + (og.y-comp.y)**2 + (og.z-comp.z)**2) < 0.4: #<--minimum allowed distance
                        x_repel += og.x - comp.x
                        y_repel += og.y - comp.y
                        z_repel += og.z - comp.z
        return x_repel, y_repel, z_repel
    
    def navigate(self):
        try:
            if self.delay[0] and time() - self.delay[0] < self.delay[1]:
                return
            self.delay = (None, None)
            # Takeoff
            cmd = Twist()
            status_msg = String()
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
                    pos: Point = self.odoms[str(self.drone_num)]["pose"]
                    if self.home_position is None:
                        self.home_position = {
                            "x": pos.x,
                            "y": pos.y,
                            "z": pos.z
                        }

                    errors = {
                        "x": self.target["x"] - pos.x if self.target["x"] else 0.0,
                        "y": self.target["y"] - pos.y if self.target["y"] else 0.0,
                        "z": self.target["z"] - pos.z if self.target["z"] else 0.0,
                    }
                    x_repel, y_repel, z_repel = self.calc_repelling()
                    cmd.linear.x = min(0.2, 0.05 * errors["x"]) if x_repel == 0.0 else 0.05 * x_repel
                    cmd.linear.y = min(0.2, 0.05 * errors["y"]) if y_repel == 0.0 else 0.5 * y_repel # drones should flank from y-axis
                    cmd.linear.z = min(0.2, 0.05 * errors["z"]) if z_repel == 0.0 else 0.05 * z_repel
                    #print("ERRORSPOS DRONE ", self.drone_num, ": ", errors["x"], " ", errors["y"], " ", errors["z"])
                    if abs(errors["x"]) < 0.1 and abs(errors["y"]) < 0.1 and abs(errors["z"]) < 0.1:
                        self.position_status = "Ready"
                    else:
                        self.position_status = "Moving"
                    status_msg.data = self.position_status
                    self.pos_status_pub.publish(status_msg)
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

    # Get drone_num from ROS params
    node = rclpy.create_node('param_reader')
    drone_num = node.declare_parameter('drone_num', 1).get_parameter_value().integer_value
    node.destroy_node()

    controller = TelloController(drone_num)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()