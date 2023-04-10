#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

def publish_camera_image():
    rospy.init_node('camera_publisher')
    camera = cv2.VideoCapture(0)
    bridge = CvBridge()
    pub = rospy.Publisher('/image_data', CompressedImage, queue_size=10)
    rate = rospy.Rate(1) # 30Hz

    while not rospy.is_shutdown():
        ret, frame = camera.read()
        if not ret:
            rospy.logerr('Failed to capture frame from camera')
            continue

        # Convert the OpenCV image to ROS message and publish it
        msg = bridge.cv2_to_compressed_imgmsg(frame)
        pub.publish(msg)

        rate.sleep()

    camera.release()

if __name__ == '__main__':
    try:
        publish_camera_image()
    except rospy.ROSInterruptException:
        pass




