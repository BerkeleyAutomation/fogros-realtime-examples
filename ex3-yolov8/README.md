# Example 3: yolov8

First, install 
```
curl https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt --output ./models/yolov8n.pt
```

Run the ros nodes
```
ros2 run yolo yolo_client
ros2 run yolo yolo_service
``