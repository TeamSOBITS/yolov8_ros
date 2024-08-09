#!/usr/bin/env python3
import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/usb_cam/image_raw', 10)

        # Read image
        filename = "bus.jpg"
        filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
        self.image = cv2.imread(filepath, cv2.IMREAD_COLOR)

        # Initialize CvBridge
        self.bridge = CvBridge()
        self.msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")

        # Timer to periodically publish the image
        self.timer = self.create_timer(1.0 / 10, self.publish_image)  # 10Hz

    def publish_image(self):
        self.publisher.publish(self.msg)
        self.get_logger().info('Published image')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
