from ultralytics import YOLO

model = YOLO("yolov8n.pt")

model.train(data="~/catkin_ws/src/yolov8_ros/datasets/train.yaml", epochs=500, save=True,batch=4, imgsz=640)
metrics = model.val()
#sucsess = model.export(fromat="yolo")