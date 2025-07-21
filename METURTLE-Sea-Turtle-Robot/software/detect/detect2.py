from ultralytics import YOLO
import cv2
import torch

# Load both YOLO models (ensure GPU is available)
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model1 = YOLO('yolov8n.pt').to(device)
model2 = YOLO('runs/detect/train13/weights/best8_13.pt').to(device)

# Define the class label for turtle hatchlings (update as per your model's label mapping)
TURTLE_HATCHLING_LABEL = "sea turtle"  # Ensure this matches your best.pt class names
DOG_LABEL = "dog"  # Ensure this matches the class name in yolov8n.pt

def main():
    # Open webcam feed
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame from webcam.")
            break

        # Run detections sequentially
        results2 = model2.predict(frame, imgsz=640, conf=0.5, device=device, verbose=False)
        results1 = model1.predict(frame, imgsz=640, conf=0.5, device=device, verbose=False)

        # Annotate frame
        annotated_frame = frame.copy()

        if results2:
            turtle_boxes = [box for box in results2[0].boxes if TURTLE_HATCHLING_LABEL in model2.names[int(box.cls)]]
            for box in turtle_boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                label = f"{TURTLE_HATCHLING_LABEL}: {conf:.2f}"
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if results1:
            dog_boxes = [box for box in results1[0].boxes if DOG_LABEL in model1.names[int(box.cls)]]
            for box in dog_boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                label = f"{DOG_LABEL}: {conf:.2f}"
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(annotated_frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Display the frame
        cv2.imshow('YOLOv8 Sequential Predictions', annotated_frame)

        # Break loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
