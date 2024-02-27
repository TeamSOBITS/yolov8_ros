#!/usr/bin/env python3
import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def operator():
    rospy.init_node("operator",anonymous=True)
    pub=rospy.Publisher("/usb_cam/image_raw",Image,queue_size=1)

    #read image
    filename="bus.jpg"
    filepath=os.path.join(os.path.dirname(os.path.abspath(__file__)),filename)
    im=cv2.imread(filepath,cv2.IMREAD_COLOR)

    # #read USB-cam
    # cap=cv2.VideoCapture(0)
    # ret, frame = cap.read()
    # cap.release()

    #make bridge
    bridge=CvBridge()
    msg=bridge.cv2_to_imgmsg(im,encoding="bgr8")


    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
        print("published")
        #input()

if __name__ == "__main__":
    try:
        operator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

