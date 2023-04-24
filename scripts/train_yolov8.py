from ultralytics import YOLO

model = YOLO("yolov8n.pt")

model.train(data="datasets/azure_rcjp2022/train.yaml", epochs=500, save=True,batch=4, imgsz=640)
metrics = model.val()
#sucsess = model.export(fromat="yolo")