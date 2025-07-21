import sys
sys.path.append('/home/meturtle/yolov5')  # 👈 Add this line

import cv2
import torch
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords
from utils.plots import Annotator
from utils.torch_utils import select_device

# ================= ROS 2 Node =================
class DetectionPublisher(Node):
    def __init__(self):
        super().__init__('yolov5_detection_publisher')
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)

    def publish_detection(self, label):
        msg = String()
        msg.data = label
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

# ================ Device Setup ================
device = select_device('0' if torch.cuda.is_available() else 'cpu')
torch.backends.cudnn.benchmark = True

# ================ Model Loading ===============
model_dog = attempt_load('yolov5n.pt', map_location=device)
model_turtle = attempt_load('best.pt', map_location=device)
model_dog.eval()
model_turtle.eval()

model_dog.names = model_dog.module.names if hasattr(model_dog, 'module') else model_dog.names
model_turtle.names = model_turtle.module.names if hasattr(model_turtle, 'module') else model_turtle.names

# ================ CSI Camera ==================
gst = ('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! '
       'nvvidconv flip-method=2 ! video/x-raw, format=BGRx ! videoconvert ! appsink')
cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("❌ Could not open CSI camera")
    exit()

print("✅ CSI Camera initialized")

# ============== Detection Result Store ===========
result_lock = threading.Lock()
preds = {'dog': None, 'turtle': None}

def detect(model, img_tensor, key):
    with torch.no_grad():
        pred = model(img_tensor)[0]
        pred = non_max_suppression(pred, 0.5, 0.45)
    with result_lock:
        preds[key] = pred

# ============== Initialize ROS 2 ==============
rclpy.init()
detection_node = DetectionPublisher()

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        resized = cv2.resize(frame, (640, 640))
        img_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        img_np = np.ascontiguousarray(img_rgb).astype(np.float32) / 255.0
        img_tensor = torch.from_numpy(img_np).permute(2, 0, 1).unsqueeze(0).to(device)

        # Reset predictions
        preds['dog'] = None
        preds['turtle'] = None

        # Start detection threads
        t1 = threading.Thread(target=detect, args=(model_dog, img_tensor, 'dog'))
        t2 = threading.Thread(target=detect, args=(model_turtle, img_tensor, 'turtle'))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        # Annotate and publish
        annotator = Annotator(frame, line_width=2, example='turtle/dog')

        for det in preds['turtle']:
            if det is not None and len(det):
                det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], frame.shape).round()
                for *xyxy, conf, cls in det:
                    label = f'{model_turtle.names[int(cls)]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=(0, 255, 0))  # Green
                    detection_node.publish_detection(f"{label}")

        for det in preds['dog']:
            if det is not None and len(det):
                det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], frame.shape).round()
                for *xyxy, conf, cls in det:
                    label = f'{model_dog.names[int(cls)]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=(255, 0, 0))  # Red
                    detection_node.publish_detection(f"{label}")

        # Show annotated frame
        cv2.imshow("YOLOv5 - Turtle & Dog Detection", annotator.result())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n🔁 Interrupted by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    detection_node.destroy_node()
    rclpy.shutdown()
    print("✅ Camera released and ROS 2 shutdown. Goodbye!")
