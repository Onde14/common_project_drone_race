
# gate_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class GateDetector(Node):
    def __init__(self):
        super().__init__('gate_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image,
         '/image_raw',
          self.image_callback,
           QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        ))
        self.pub = self.create_publisher(PointStamped, '/gate_position', 10)
        self.type_pub = self.create_publisher(String, '/gate_type', 10)
        self.debug_pub = self.create_publisher(Image, '/gate_debug_image', 10)
        self.gate_color_hsv =  [160, 61, 34]
        self.green_lower = np.array([40, 70, 70])
        self.green_upper = np.array([80, 255, 255])
        self.x_error_pub = self.create_publisher(Int32, '/x_error', 10)
        self.y_error_pub = self.create_publisher(Int32, '/y_error', 10)


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        best_contour = None
        max_area = 0
        gate_type = None
        try:
            center = [ int(np.average(indices)) for indices in np.where(mask >= 255) ]
            print(center)
            cv2.circle(cv_image, (center[1],center[0]), 40, (0, 0, 255), -1)
        except (ValueError, TypeError):
            center = None
            pass
        height, width, channels = cv_image.shape
        if center:
            x_error = width // 2 - center[1]
            y_error = height // 2 - center[0]
            print(f"x error: {x_error},y error: {y_error}")
            self.x_error_pub.publish(Int32(data=x_error))
            self.y_error_pub.publish(Int32(data=y_error))
        for contour in contours:
            area = cv2.contourArea(contour)
            cv2.drawContours(cv_image, [contour], -1, (0, 255, 255), 2)
                # Publish debug image (with drawings)
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.debug_pub.publish(debug_msg)
"""
            #cv2.putText(cv_image, gate_type, (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            if area < 500:  # ignore small detections
                continue

            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
            if len(approx) >= 8:
                shape = 'circle'
            elif len(approx) == 4:
                shape = 'rectangle'
            else:
                continue

            if area > max_area:
                best_contour = contour
                max_area = area
                gate_type = shape


        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Publish gate center as PointStamped
                gate_msg = PointStamped()
                gate_msg.header.stamp = self.get_clock().now().to_msg()
                gate_msg.header.frame_id = "camera"
                gate_msg.point.x = float(cx)
                gate_msg.point.y = float(cy)
                gate_msg.point.z = 0.0
                self.pub.publish(gate_msg)

                # Publish type
                type_msg = String()
                type_msg.data = gate_type

                self.type_pub.publish(type_msg)

                # Optional: draw the detected contour on the original image
        if best_contour is not None:
            cv2.drawContours(cv_image, [best_contour], -1, (0, 0, 255), 2)
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(cv_image, gate_type, (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
"""

def main(args=None):
    print("upper main")
    rclpy.init(args=args)
    node = GateDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("main)")
    main()