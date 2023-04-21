#!/usr/bin/env python3
from ultralytics import YOLO
import rospy
import cv2
import numpy as np
from rostopic import get_topic_type
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from sobit_common_msg.msg import BoundingBox, BoundingBoxes, StringArray, ObjectPose, ObjectPoseArray
from sobit_common_msg.srv import RunCtrl, RunCtrlResponse
from ultralytics.yolo.utils.plotting import Annotator, colors
# from ultralytics.yolo.data.augment import LetterBox
# from ultralytics.yolo.utils.checks import check_imgsz, check_requirements
# from ultralytics.yolo.utils import ops
# from ultralytics.yolo.engine.model import DetectionModel
# from copy import deepcopy
import datetime

class Yolov8Detector:
    def __init__(self):
        #get parameter from .launch(rospy.get_param)
        self.can_predict = rospy.get_param("~initial_predict", True)
        self.publish_image = rospy.get_param("~publish_image")
        self.view_image = rospy.get_param("~view_image")
        self.save_image = rospy.get_param("~save_image")
        self.conf = rospy.get_param("~conf")
        self.output_topic_bb = rospy.get_param("~output_topic_bb") #default: "bjects_rect"
        
        #Define publishers
        self.pub_result_img = rospy.Publisher("detect_result", Image, queue_size=10) #結果画像
        self.pub_detect_list = rospy.Publisher("detect_list", StringArray, queue_size=10) #Label list (class conf)
        self.pub_detect_poses = rospy.Publisher("detect_poses", ObjectPoseArray, queue_size=10) #center xy(NOTnormalized)
        self.pub_prediction = rospy.Publisher(self.output_topic_bb, BoundingBoxes, queue_size=10) #BBox (xywh_NOTnormalized)
        self.pub_image = rospy.Publisher("image_pub", Image, queue_size=10) #subscribed Image
        print("publisher defined")

        #Set Inference size
        #height, weight = 
        self.img_size = [rospy.get_param("~inference_size_w", 1280), rospy.get_param("~inference_size_h", 720)]
        #self.img_size = check_imgsz(self.img_size, s = self.stride)

        #Start Run_control Service(???????????????)
        self.server = rospy.Service("runctrl", RunCtrl, self.run_ctrl_server)
        self.sub_run_ctrl = rospy.Subscriber("run_ctrl", Bool, self.run_ctrl_callback)
        print("define sub procedure.")
        #Initialize Subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking=True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"
        # #Define Subscriber

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(input_image_topic, CompressedImage, self.callback, queue_size=1)
        else:
            self.image_sub = rospy.Subscriber(input_image_topic, Image, self.callback, queue_size=1)        
        print("subscriber defined")

        rospy.spin()

    #RunCtrl Server
    def run_ctrl_server(self, msg):
        if msg.request:
            self.can_predict = True
        else:
            self.can_predict = False
        return RunCtrlResponse(True)
    
    def run_ctrl_callback(self, msg):
        self.can_predict = msg.data
        rospy.loginfo("run ctrl -> {}".format(self.can_predict))

    #####CALLBACK (when Subscribing a Image Publish)#########
    def callback(self,msg):
        if not self.can_predict:
            return
    
        rate = rospy.Rate(30)

        #subscribe images and conversion to bgr
        self.bridge = CvBridge()
        cv_array=np.ndarray
        #cv_array = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if not self.compressed_input:
            cv_array = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        else:
            cv_array = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")


        #np.ndarray型で渡すとvisualizeができないので、cv2.imwriteしてから、predict()に保存パスを渡します
        #多分home/<user名>/.ros/yolov8_image.jpgが出力されます
        #要検討　パフォーマンスに影響ある可能性
        cv2.imwrite("yolov8_image.jpg", cv_array)

        #Do the predict with YOLOv8 (ultralytics)
        self.model = YOLO("yolov8n.pt")
        self.result = self.model.predict("yolov8_image.jpg", show=self.view_image, save=self.save_image, conf=self.conf)
        #self.result = self.model.predict(0, show=self.view_image) #USBcamera用
        self.boxes = self.result[0].numpy().boxes #検出した全BBox
        self.names = self.result[0].names #モデルの全ラベル一覧

        #Fill BoundingBox Messages
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

            bounding_boxes = BoundingBoxes()
            bounding_boxes.header = msg.header
            bounding_boxes.bounding_boxes.append(bounding_box)

            #Fill detect_list
            detect_list = StringArray()
            label = f"{bounding_box.Class} {bounding_box.probability:.2f}"
            detect_list.data.append(label)

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
            img_result = self.bridge.cv2_to_imgmsg(img_result, "bgr8")
            self.pub_result_img.publish(img_result)


            #Fill detect_poses
            obj_pose = ObjectPose()
            obj_pose.Class = label
            obj_pose.pose.position.x = bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin) / 2
            obj_pose.pose.position.y = bounding_box.ymin + (bounding_box.ymax - bounding_box.ymin) / 2
            obj_pose.pose.position.z = -1

            detect_poses = ObjectPoseArray()
            detect_poses.object_poses.append(obj_pose)
        detect_poses.header = msg.header

        #Publish PREDICTION
        time = datetime.datetime.now().strftime('%Y/%m/%d %H:%M:%S.%f')[:-3]
        print(time)
        self.pub_prediction.publish(bounding_boxes)
        try:
            self.pub_detect_poses.publish(detect_poses)
            self.pub_detect_list.publish(detect_list)
            if self.publish_image:
                self.pub_image.publish(msg)
        except UnboundLocalError:
            pass

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