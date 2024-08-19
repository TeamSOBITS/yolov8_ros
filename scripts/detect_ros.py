#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sobits_msgs.msg import BoundingBox, BoundingBoxes, StringArray, ObjectPose, ObjectPoseArray
from sobits_msgs.srv import RunCtrl
from ultralytics.utils.plotting import Annotator, colors ###
from copy import deepcopy

class Yolov8Detector:
    def __init__(self, nd):
        self.nd = nd
        self.nd.declare_parameters(
            namespace='',
            parameters=[
                ('execute_default', True),
                ('view_image', True),
                ('save_image', True),
                ('conf', 0.3),
                ('pub_rate', 5),
                ('weights', "/home/sobits/colcon_ws/src/yolov8_ros/weights/default_weight_file.pt"),
                ('image_topic_name', "rgb/image_raw")
            ]
        )
        #get parameter from .launch(self.nd.get_parameter)
        self.can_predict = self.nd.get_parameter("execute_default").get_parameter_value().bool_value
        self.view_image = self.nd.get_parameter("view_image").get_parameter_value().bool_value
        self.save_image = self.nd.get_parameter("save_image").get_parameter_value().bool_value
        self.conf = self.nd.get_parameter("conf").get_parameter_value().double_value
        self.rate = self.nd.get_parameter("pub_rate").get_parameter_value().integer_value
        self.weight_path = self.nd.get_parameter("weights").get_parameter_value().string_value
        self.img_topic_name = self.nd.get_parameter("image_topic_name").get_parameter_value().string_value
        
        #Define publishers
        self.pub_result_img = self.nd.create_publisher(Image, "/yolov8/detect_result", 10) #結果画像
        self.pub_detect_list = self.nd.create_publisher(StringArray, "/yolov8/detect_list", 10) #Label list (class conf)
        self.pub_detect_poses = self.nd.create_publisher(ObjectPoseArray, "/yolov8/detect_poses", 10) #center xy(NOTnormalized)
        self.pub_prediction = self.nd.create_publisher(BoundingBoxes, "/yolov8/objects_rect", 10) #BBox (xywh_NOTnormalized)

        #Start Run_control Service
        self.server = self.nd.create_service(RunCtrl, "/yolov8/run_ctr", self.run_ctrl_server)

        #Initialize Subscriber
        self.image_sub = self.nd.create_subscription(Image, self.img_topic_name, self.callback, 1)
        self.img = Image()

        self.model = YOLO(self.weight_path)
        self.flag = False

        # publisher
        self.inference_loop()

    # inference function
    def inference_loop(self):
        while rclpy.ok():
            if ((self.flag) and (self.can_predict)):
                #subscribe images and conversion to bgr
                self.bridge = CvBridge()
                cv_array = np.ndarray
                cv_array = self.bridge.imgmsg_to_cv2(self.img, "bgr8")
                
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
                    bounding_box.class_name = self.names[c] #object name (char)
                    bounding_box.probability = float(box.conf[0]) #Confidence
                    bounding_box.xmin = int(box.xyxy[0][0]) #Xmin NOT normalized 0-1
                    bounding_box.ymin = int(box.xyxy[0][1]) #Ymin NOT normalized 0-1
                    bounding_box.xmax = int(box.xyxy[0][2]) #Xmax NOT normalized 0-1
                    bounding_box.ymax = int(box.xyxy[0][3]) #Ymax NOT normalized 0-1
                    
                    #Fill detect_list
                    label = f"{bounding_box.class_name} {bounding_box.probability:.2f}"
                    
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
                    obj_pose.class_name = label
                    obj_pose.pose.position.x = float(bounding_box.xmin + int((bounding_box.xmax - bounding_box.xmin) / 2))
                    obj_pose.pose.position.y = float(bounding_box.ymin + int((bounding_box.ymax - bounding_box.ymin) / 2))
                    obj_pose.pose.position.z = -1.0
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
            rclpy.spin_once(nd, timeout_sec=1/self.rate)

    #RunCtrl Server
    def run_ctrl_server(self, request, response):
        self.can_predict = request.request
        response.success = True
        return response

    ##### CALLBACK (Publish results when Subscribing a Image) #########
    def callback(self,msg):
        self.img = msg
        self.flag = True

if __name__ == "__main__":
    rclpy.init()
    nd = Node("yolov8")
    detector = Yolov8Detector(nd)

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