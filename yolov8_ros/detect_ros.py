#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from sobits_msgs.msg import BoundingBox, BoundingBoxes, StringArray, ObjectPose, ObjectPoseArray
from sobits_msgs.srv import RunCtrl
from ultralytics.utils.plotting import colors
from copy import deepcopy

class Yolov8Detector(Node):
    def __init__(self):
        super().__init__('yolov8_detector')

        # Get parameters from the parameter server
        self.declare_parameter("initial_predict", True)
        self.declare_parameter("view_image", False)
        self.declare_parameter("save_image", False)
        self.declare_parameter("conf", 0.5)
        self.declare_parameter("pub_rate", 10)
        self.declare_parameter("weights", "yolov8n.pt")
        self.declare_parameter("inference_size_w", 1280)
        self.declare_parameter("inference_size_h", 720)
        self.declare_parameter("image_topic_name", "/camera/image_raw")

        self.can_predict = self.get_parameter("initial_predict").get_parameter_value().bool_value
        self.view_image = self.get_parameter("view_image").get_parameter_value().bool_value
        self.save_image = self.get_parameter("save_image").get_parameter_value().bool_value
        self.conf = self.get_parameter("conf").get_parameter_value().double_value
        self.rate = self.get_parameter("pub_rate").get_parameter_value().integer_value
        self.weight_path = self.get_parameter("weights").get_parameter_value().string_value
        self.img_size = [
            self.get_parameter("inference_size_w").get_parameter_value().integer_value,
            self.get_parameter("inference_size_h").get_parameter_value().integer_value
        ]

        # Define publishers
        self.pub_result_img = self.create_publisher(Image, "/yolov8/detect_result", 10)
        self.pub_detect_list = self.create_publisher(StringArray, "/yolov8/detect_list", 10)
        self.pub_detect_poses = self.create_publisher(ObjectPoseArray, "/yolov8/detect_poses", 10)
        self.pub_prediction = self.create_publisher(BoundingBoxes, "/yolov8/objects_rect", 10)

        # Start Run_control Service
        self.server = self.create_service(RunCtrl, "/yolov8/run_ctrl", self.run_ctrl_server)

        # Initialize Subscriber to Image/CompressedImage topic
        image_topic_name = self.get_parameter("image_topic_name").get_parameter_value().string_value
        self.image_sub = self.create_subscription(
            Image, image_topic_name, self.callback, 1)

        self.model = YOLO(self.weight_path)
        self.flag = False
        self.bridge = CvBridge()

        # Create a timer to call the inference loop
        self.timer = self.create_timer(1.0 / self.rate, self.inference_loop)

    def inference_loop(self):
        
        img_result_msg = None

        if self.flag and self.can_predict:
            cv_array = self.bridge.imgmsg_to_cv2(self.img, "bgr8")

            # Save the image temporarily
            cv2.imwrite("yolov8_image.jpg", cv_array)

            # Inference with YOLOv8
            result = self.model.predict("yolov8_image.jpg", show=self.view_image, save=self.save_image, conf=self.conf)
            boxes = result[0].cpu().numpy().boxes
            names = result[0].names

            # Prepare messages
            detect_list = StringArray()
            detect_poses = ObjectPoseArray()
            bounding_boxes = BoundingBoxes()
            detect_list.header = self.img.header
            detect_poses.header = self.img.header
            bounding_boxes.header = self.img.header

            img_result = cv_array

            for x in reversed(range(len(boxes))):
                bounding_box = BoundingBox()
                box = boxes[x]
                c = int(box.cls[0])
                bounding_box.class_name = names[c]
                bounding_box.probability = float(box.conf[0])
                bounding_box.xmin = int(box.xyxy[0][0])
                bounding_box.ymin = int(box.xyxy[0][1])
                bounding_box.xmax = int(box.xyxy[0][2])
                bounding_box.ymax = int(box.xyxy[0][3])

                label = f"{bounding_box.class_name} {bounding_box.probability:.2f}"
                (w, h), baseline = cv2.getTextSize(label, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, thickness=1)
                cv2.rectangle(img_result, (int(box.xyxy[0][0]), int(box.xyxy[0][1])), (int(box.xyxy[0][2]), int(box.xyxy[0][3])), colors(c, True), 2)
                cv2.rectangle(img_result, (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - h), (int(box.xyxy[0][0]) + w, int(box.xyxy[0][1])), colors(c, True), -1)
                cv2.putText(img_result, label, (int(box.xyxy[0][0]), int(box.xyxy[0][1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255), thickness=1)

                # Convert the result back to a ROS message
                img_result_msg = self.bridge.cv2_to_imgmsg(img_result, "bgr8")

                # Fill detect_poses
                obj_pose = ObjectPose()
                obj_pose.class_name = label
                obj_pose.pose.position.x = float(bounding_box.xmin + ((bounding_box.xmax - bounding_box.xmin) / 2))
                obj_pose.pose.position.y = float(bounding_box.ymin + ((bounding_box.ymax - bounding_box.ymin) / 2))
                obj_pose.pose.position.z = -1.0
                detect_list.data.append(deepcopy(label))
                detect_poses.object_poses.append(deepcopy(obj_pose))
                bounding_boxes.bounding_boxes.append(deepcopy(bounding_box))

            self.pub_prediction.publish(bounding_boxes)
            self.pub_detect_poses.publish(detect_poses)
            self.pub_detect_list.publish(detect_list)
            if img_result_msg is not None:
                self.pub_result_img.publish(img_result_msg)

    def run_ctrl_server(self, request, response):
        if request.request:
            self.can_predict = True
        else:
            self.can_predict = False
        response.response = True
        return response

    def callback(self, msg):
        self.img = msg
        self.flag = True

def main(args=None):
    rclpy.init(args=args)
    detector = Yolov8Detector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
