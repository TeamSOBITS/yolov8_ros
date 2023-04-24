from ultralytics import YOLO

model = YOLO("yolov8n.yaml")

model.train(data="aaa.yaml", epochs=3)
metrics = model.val()
sucsess = model.export(fromat="onnx")