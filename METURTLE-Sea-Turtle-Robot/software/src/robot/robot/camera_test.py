import cv2

# === CSI Camera GStreamer Pipeline ===
gst_pipeline = (
    'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! '
    'nvvidconv flip-method=2 ! video/x-raw, format=BGRx ! videoconvert ! appsink'
)

# === Open the CSI Camera ===
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("❌ Failed to open CSI camera")
    exit()

print("✅ CSI Camera initialized. Press 'q' to exit.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        # Display the frame
        cv2.imshow("📷 CSI Camera Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("🔁 Interrupted by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    print("✅ Camera released. Goodbye!")
