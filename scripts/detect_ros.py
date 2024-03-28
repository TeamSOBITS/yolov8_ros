#!/usr/bin/env python3
from ultralytics import YOLO
import rospy
import cv2
import numpy as np
from rostopic import get_topic_type
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from sobits_msgs.msg import BoundingBox, BoundingBoxes, StringArray, ObjectPose, ObjectPoseArray
from sobits_msgs.srv import RunCtrl, RunCtrlResponse
from ultralytics.utils.plotting import Annotator, colors
from copy import deepcopy
# import datetime

class Yolov8Detector:
    def __init__(self):
        #get parameter from .launch(rospy.get_param)
        self.can_predict = rospy.get_param("~initial_predict", True)
        self.view_image = rospy.get_param("~view_image")
        self.save_image = rospy.get_param("~save_image")
        self.conf = rospy.get_param("~conf")
        self.rate = rospy.get_param("~pub_rate")
        # self.output_topic_bb = rospy.get_param("~output_topic_bb") #default: "bjects_rect"
        self.weight_path = rospy.get_param("~weights") #weight name or path
        
        #Define publishers
        self.pub_result_img = rospy.Publisher("/yolov8/detect_result", Image, queue_size=10) #結果画像
        self.pub_detect_list = rospy.Publisher("/yolov8/detect_list", StringArray, queue_size=10) #Label list (class conf)
        self.pub_detect_poses = rospy.Publisher("/yolov8/detect_poses", ObjectPoseArray, queue_size=10) #center xy(NOTnormalized)
        self.pub_prediction = rospy.Publisher("/yolov8/objects_rect", BoundingBoxes, queue_size=10) #BBox (xywh_NOTnormalized)

        #Set Inference size
        self.img_size = [rospy.get_param("~inference_size_w", 1280), rospy.get_param("~inference_size_h", 720)]

        #Start Run_control Service
        self.server = rospy.Service("/yolov8/run_ctr", RunCtrl, self.run_ctrl_server)

        #Initialize Subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~image_topic_name"), blocking=True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(input_image_topic, CompressedImage, self.callback, queue_size=1)
            self.img = CompressedImage()
        else:
            self.image_sub = rospy.Subscriber(input_image_topic, Image, self.callback, queue_size=1)
            self.img = Image()

        self.model = YOLO(self.weight_path)
        self.flag = False

        # publisher
        self.inference_loop()

    # inference function
    def inference_loop(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if ((self.flag) and (self.can_predict)):
                #subscribe images and conversion to bgr
                self.bridge = CvBridge()
                cv_array = np.ndarray
                if not self.compressed_input:
                    cv_array = self.bridge.imgmsg_to_cv2(self.img, "bgr8")
                else:
                    cv_array = self.bridge.compressed_imgmsg_to_cv2(self.img, "bgr8")
                
                #cv2.imwrite
                cv2.imwrite("yolov8_image.jpg", cv_array)

                #Inference with YOLOv8 (ultralytics)
                self.result = self.model.predict("yolov8_image.jpg", show=self.view_image, save=self.save_image, conf=self.conf)
                self.boxes = self.result[0].cpu().numpy().boxes #All Boundig box
                self.names = self.result[0].names #All label list

                #Fill BoundingBox Messages
                detect_list = StringArray()
                detect_poses = ObjectPoseArray()
                bounding_boxes = BoundingBoxes()
                detect_list.header = self.img.header
                detect_poses.header = self.img.header
                bounding_boxes.header = self.img.header
                img_result_img = self.img
                for x in reversed(range(len(self.boxes))):
                    #Fill prediction
                    bounding_box = BoundingBox()
                    box = self.boxes[x]
                    c = int(box.cls[0]) #object number (int)
                    bounding_box.Class = self.names[c] #object name (char)
                    bounding_box.probability = box.conf[0] #Confidence
                    bounding_box.xmin = int(box.xyxy[0][0]) #Xmin NOT normalized 0-1
                    bounding_box.ymin = int(box.xyxy[0][1]) #Ymin NOT normalized 0-1
                    bounding_box.xmax = int(box.xyxy[0][2]) #Xmax NOT normalized 0-1
                    bounding_box.ymax = int(box.xyxy[0][3]) #Ymax NOT normalized 0-1
                    
                    #Fill detect_list
                    label = f"{bounding_box.Class} {bounding_box.probability:.2f}"
                    
                    #generate result image
                    img_result = cv_array
                    (w, h), baseline = cv2.getTextSize(label,
                                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                            fontScale=0.5,
                                            thickness=1)
                    cv2.rectangle(img_result,
                                pt1=(int(box.xyxy[0][0]),int(box.xyxy[0][1])),
                                pt2=(int(box.xyxy[0][2]),int(box.xyxy[0][3])),
                                color=colors(c, True),
                                thickness=2,
                                lineType=cv2.LINE_4)
                    cv2.rectangle(img_result,
                                pt1=(int(box.xyxy[0][0]), int(box.xyxy[0][1]) - h),
                                pt2=(int(box.xyxy[0][0]) + w, int(box.xyxy[0][1])),
                                color=colors(c, True),
                                thickness=-1,
                                lineType=cv2.LINE_4)
                    cv2.putText(img_result,
                                text=label,
                                org=(int(box.xyxy[0][0]),int(box.xyxy[0][1])),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=0.5,
                                color=(255,255,255),
                                thickness=1,
                                lineType=cv2.LINE_AA)
                    #publish result image
                    img_result_img = self.bridge.cv2_to_imgmsg(img_result, "bgr8")

                    #Fill detect_poses
                    obj_pose = ObjectPose()
                    obj_pose.Class = label
                    obj_pose.pose.position.x = bounding_box.xmin + int((bounding_box.xmax - bounding_box.xmin) / 2)
                    obj_pose.pose.position.y = bounding_box.ymin + int((bounding_box.ymax - bounding_box.ymin) / 2)
                    obj_pose.pose.position.z = -1
                    detect_list.data.append(deepcopy(label))
                    detect_poses.object_poses.append(deepcopy(obj_pose))
                    bounding_boxes.bounding_boxes.append(deepcopy(bounding_box))

                self.pub_prediction.publish(bounding_boxes)
                try:
                    self.pub_detect_poses.publish(detect_poses)
                    self.pub_detect_list.publish(detect_list)
                except UnboundLocalError:
                    pass
                self.pub_result_img.publish(img_result_img)
            r.sleep()

    #RunCtrl Server
    def run_ctrl_server(self, msg):
        if msg.request:
            self.can_predict = True
        else:
            self.can_predict = False
        return RunCtrlResponse(True)

    ##### CALLBACK (Publish results when Subscribing a Image) #########
    def callback(self,msg):
        self.img = msg
        self.flag = True

if __name__ == "__main__":
    rospy.init_node("yolov8")
    
    detector = Yolov8Detector()

    rospy.spin()

"""
################ R E F E R E N C E ###########################
YOLOv8 Docs: https://docs.ultralytics.com/

self.model = YOLO("yolov8n.pt")
self.boxes = self.model.predict()

normalizeされたxyminとxymaxをすべて取り出すには:
for x in range(4):
    print(self.boxes[0].xyxyn[0][x])

0個目のboxのxyxyの3個目の要素(ymax?):
print(self.boxes[0].xyxy.numpy()[0][3])

1個目のboxのcls(数字):
print(self.boxes[1].cls.numpy())

検知した物体のラベルをcharで表示:
print(self.names[self.boxes[0].cls[0]])

##############################################################"""