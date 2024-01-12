#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np

class testPublishImageNode(Node):
    def __init__(self):
        super().__init__('test_publish')
        self.bridge = cv_bridge.CvBridge()
        self.pos_tool_pub = self.create_publisher(Image, 'test_image', 10)
        
        # Parameters
        
        
        self.cv_image = cv2.imread('/home/adam/ES_images/src/nut_position_orientation/nut_position_orientation/num_710.png')
        self.msg_image = self.bridge.cv2_to_imgmsg((self.cv_image), 'bgr8')
        
        self.send_image_command()
        

    # to publish some data it's good to use timer
    def send_image_command(self):
        # create message
        msg = Image()
        msg = self.msg_image
        self.pos_tool_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = testPublishImageNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()