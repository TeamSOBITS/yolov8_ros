#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.camera = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(CompressedImage, '/usb_cam/image_raw', 1)
        self.timer = self.create_timer(1.0 / 3, self.publish_camera_image)  # 3Hz

    def publish_camera_image(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().error('Failed to capture frame from camera')
            return

        # Convert the OpenCV image to ROS message and publish it
        msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.publisher.publish(msg)
        self.get_logger().info('Published image')

    def destroy_node(self):
        self.camera.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
