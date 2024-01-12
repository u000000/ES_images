#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
from es_interfaces.msg import PosAngle
from es_interfaces.msg import Pos

class nutPosOrient(Node):
    
    def __init__(self):
        super().__init__('nutPosOrient')
        self.pos_tool_pub = self.create_publisher(PosAngle, 'pos_angle_pub', 10)
        self.pose_subscriber = self.create_subscription(Image, '', self.image_callback, 10)
        self.bridge = cv_bridge.CvBridge()
        
        # Parameters
        
        
        self.get_logger().info('Looking for images from AI')
        
    def image_callback(self, msg: Image):
        try:
            """# Convert msg to cv2 format"""
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            image_1 = image.copy()
            c = (int((image.shape[1])/2), int((image.shape[0])/2))

            """# Prepare image"""
            lower_bound = np.array([15, 0, 0])
            upper_bound = np.array([255, 100, 255])
            image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            image_masked = cv2.inRange(image_HSV, lower_bound, upper_bound)

            image_dil = cv2.dilate(image_masked, np.ones((15, 15), np.uint8))
            image_ero = cv2.erode(image_dil, np.ones((10, 10), np.uint8))
            image = cv2.bitwise_not(image_ero)

            # Look for contours
            conts, hierarchy = cv2.findContours(image_1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            centers = []
            for cont in conts:
                M = cv2.moments(cont)
                x0 = int(M['m10']/M['m00'])
                y0 = int(M['m01']/M['m00'])
                centers.append((x0, y0))
                
            """# Calculate position"""
            self.nutpos = Pos()
            for center in centers:
                cv2.circle(image, center, 3, (5, 92, 94), 3)
                cv2.circle(image, c, 3, (0, 82, 88), 3)
                self.nutpos.x = center[0] - c[0]
                self.nutpos.y = center[1] - c[1]
                self.get_logger().info('Nut position: ' + self.nutpos)

            """# Calculate tool movement angle"""
            # Find the furthest point of the contour (from its center of mass)
            max_dist = 0
            max_point = None
            
            for cont in conts:
                for point in cont:
                    dist = np.sqrt((x0 - point[0][0])**2 + (y0 - point[0][1])**2)
                    if dist > max_dist:
                        max_dist = dist
                        max_point = point[0]

            # Calculate angle
            offset_angle_of_camera = 18.5
            tool_angle = 0
            
            if (center[0]<max_point[0] and center[1]>max_point[1]):
                # I quarter
                tool_angle = -np.degrees(np.arctan((max_point[0]-center[0])/(center[1]-max_point[1]))) - offset_angle_of_camera
            elif (center[0]<max_point[0] and center[1]<max_point[1]):
                # IV quarter
                tool_angle = 90 - offset_angle_of_camera - np.degrees(np.arctan((max_point[1]-center[1])/(max_point[0]-center[0])))
            elif (center[0]>max_point[0] and center[1]<max_point[1]):
                # III quarter
                tool_angle = -np.degrees(np.arctan((center[0]-max_point[0])/(max_point[1]-center[1]))) - offset_angle_of_camera
            elif (center[0]>max_point[0] and center[1]>max_point[1]):
                # II quarter
                tool_angle = 90 - offset_angle_of_camera - np.degrees(np.arctan((center[1]-max_point[1])/(center[0]-max_point[0])))
            elif point[0]==center[0]:
                tool_angle = -18.5
            elif point[1]==center[1]:
                tool_angle = 90-18.5
                
            self.send_msg_data()
                     
        except Exception as e:
            self.get_logger().error('Error: %s' % str(e))
    
    
    # publish nut position and tool angle
    def send_msg_data(self):
        # create message
        msg = PosAngle()
        msg.position = self.nutpos
        # publish the msg
        self.pos_tool_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = nutPosOrient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()