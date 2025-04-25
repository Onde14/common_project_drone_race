
# gate_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from time import time
from nav_msgs.msg import Odometry

RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)
CLOSENESS_TRESHOLD = 0.8

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
        self.debug_pub = self.create_publisher(Image, '/gate_debug_image', 10)
        self.closeness_pub = self.create_publisher(Float32, '/closeness', 10)
        self.x_error_pub = self.create_publisher(Int32, '/x_error', 10)
        self.y_error_pub = self.create_publisher(Int32, '/y_error', 10)
#        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.gates_passed_sub = self.create_subscription(Int32, '/gates_passed', self.gates_passed_callback, 10)
        self.green_lower = np.array([40, 50, 40])
        self.green_upper = np.array([80, 255, 255])
        self.red_lower1 = np.array([0, 90, 80])
        self.red_upper1 = np.array([3,255,255])
        self.red_lower2 = np.array([160, 90, 80])
        self.red_upper2 = np.array([173, 255,255])
        self.CONTOUR_AREA_THRESHOLD = 2000
        self.gates_passed = 4
        self.TAG_GATE = 3
        self.gate_detected_time = 0

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def gates_passed_callback(self, msg):
        self.gates_passed = msg.data

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        height, width, _ = cv_image.shape
        if self.gates_passed != self.TAG_GATE and self.gates_passed != 4:
            print("Looking for green colors")
            # FIND GREEN GATE CENTER
            mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # Filter small contours out of the mask
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.CONTOUR_AREA_THRESHOLD:
                    cv2.drawContours(mask, [contour], -1, 0, -1) # Fill small contours with black
                    cv2.drawContours(cv_image, [contour], -1, RED, 2)
                else:
                    cv2.drawContours(cv_image, [contour], -1, GREEN, 2)
            # Calcualte the center of mass of the mask

            try:
                center = [ int(np.average(indices)) for indices in np.where(mask >= 255) ]
                x_error = width // 2 - center[1]
                y_error = height // 3 - center[0]
                cv2.circle(cv_image, (center[1], center[0]), 40, RED, -1)
                # Calculate closeness of the gate
                topmost = height  # Initialize with max value
                bottommost = 0    # Initialize with min valueop
                for contour in contours:
                    if cv2.contourArea(contour) >= self.CONTOUR_AREA_THRESHOLD:
                        ys = contour[:, 0, 1]
                        topmost = min(topmost, np.min(ys))
                        bottommost = max(bottommost, np.max(ys))
                top_closeness = 1.0 - (topmost / height) if topmost < height else 0.0
                bottom_closeness = bottommost / height if bottommost > 0 else 0.0
                cv2.line(cv_image, (0, topmost), (width, topmost), BLUE if top_closeness > CLOSENESS_TRESHOLD else RED, 4)
                cv2.line(cv_image, (0, bottommost), (width, bottommost), BLUE if bottom_closeness > CLOSENESS_TRESHOLD else RED, 4)

                closeness = min(top_closeness, bottom_closeness)
                if center[0] != -1 and closeness > CLOSENESS_TRESHOLD:
                    cv2.circle(cv_image, (center[1],center[0]), 40, BLUE, -1)
            except (ValueError, TypeError):
                center = [-1, -1]
                x_error = -1
                y_error = -1
                closeness = 0.0
        elif self.gates_passed == self.TAG_GATE:        
            # If green gate center was not found, try to find fiducial gate center
            # FIND FIDUCUAL GATE CENTER
            corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
            if ids is not None and len(ids) >= 3:
                topmost = height
                center = []
                bottommost = 0
                for marker in corners:
                    c = marker[0]
                    cx = int(np.mean(c[:, 0]))
                    cy = int(np.mean(c[:, 1]))
                    center.append((cx, cy))
                    cv2.circle(cv_image, (cx, cy), 10, BLUE, -1)
                    topmost = min(topmost, cy)
                    bottommost = max(bottommost, cy)

                avg_x = int(np.mean([pt[0] for pt in center]))
                avg_y = int(np.mean([pt[1] for pt in center]))
                x_error = width // 2 - avg_x
                y_error = height // 3 - avg_y
                y_error += 25
                closeness = min(1.0 - topmost / height, bottommost / height) + 0.2
                cv2.circle(cv_image, (avg_x, avg_y), 20, RED, 3)
            else:
                center = [-1, -1]
                x_error = -1
                y_error = -1
                closeness = 0.0
        elif self.gates_passed == 4:
            print("Looking for red colors")
            mask_red1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mask_red2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mask2 = cv2.bitwise_or(mask_red1, mask_red2)
            contours2, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours2:   
                area = cv2.contourArea(contour)
                if area < self.CONTOUR_AREA_THRESHOLD:
                    cv2.drawContours(mask2, [contour], -1, 0, -1) # Fill small contours with black
                    cv2.drawContours(cv_image, [contour], -1, RED, 2)
                else:
                    cv2.drawContours(cv_image, [contour], -1, GREEN, 2)



            try:
                center = [ int(np.average(indices)) for indices in np.where(mask2 >= 255) ]
                x_error = width // 2 - center[1]
                y_error = height // 3 - center[0]
                cv2.circle(cv_image, (center[1], center[0]), 40, RED, -1)
                # Calculate closeness of the gate
                topmost = height  # Initialize with max value
                bottommost = 0    # Initialize with min valueop
                for contour in contours2:
                    if cv2.contourArea(contour) >= self.CONTOUR_AREA_THRESHOLD:
                        ys = contour[:, 0, 1]
                        topmost = min(topmost, np.min(ys))
                        bottommost = max(bottommost, np.max(ys))
                top_closeness = 1.0 - (topmost / height) if topmost < height else 0.0
                bottom_closeness = bottommost / height if bottommost > 0 else 0.0
                cv2.line(cv_image, (0, topmost), (width, topmost), BLUE if top_closeness > CLOSENESS_TRESHOLD else RED, 4)
                cv2.line(cv_image, (0, bottommost), (width, bottommost), BLUE if bottom_closeness > CLOSENESS_TRESHOLD else RED, 4)

                closeness = min(top_closeness, bottom_closeness)
                if center[0] != -1 and closeness > CLOSENESS_TRESHOLD:
                    cv2.circle(cv_image, (center[1],center[0]), 40, BLUE, -1)
            except (ValueError, TypeError):
                center = [-1, -1]
                x_error = -1
                y_error = -1
                closeness = 0.0

            
        
        # If fiducial gate center was not found, try to find stop sign center
        # FIND STOP SIGN CENTER
        #if center[0] == -1:
        #    pass
        # Publish data
        if closeness > 0:
            self.gate_detected_time = time()
        print(f"center: {center} \tx error: {x_error},\ty error: {y_error}\tcloseness: {closeness:.2f}")
        if time() - self.gate_detected_time > 2 or closeness > 0:
            print("publish")
            self.closeness_pub.publish(Float32(data=closeness))
            self.x_error_pub.publish(Int32(data=x_error))
            self.y_error_pub.publish(Int32(data=y_error))
        debug_img = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.debug_pub.publish(debug_img)
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
    rclpy.init(args=args)
    node = GateDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()