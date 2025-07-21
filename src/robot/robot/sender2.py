import socketio
import rclpy
import threading
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String
import requests

# Set your ngrok server URL
SERVER_URL = 'https://8e6c2753cffb.ngrok.app/'

# === ROS 2 Node ===
class CoordinateReceiver(Node):
    def __init__(self):
        super().__init__('coordinate_receiver')
        
        self.subscription = self.create_subscription(
            Float64MultiArray, 'coordinates', self.listener_callback_gps, 10)
        
        self.subscription = self.create_subscription(
            Float64, 'compass', self.listener_callback_compass, 10)
        
        self.subscription2 = self.create_subscription(
            String, 'detected_objects', self.listener_callback2, 10)
        
        self.publisher_markers = self.create_publisher(
            Float64MultiArray, 'marker_location', 10)

        self.latest_coordinates = {
            'lat': 0.0,
            'lng': 0.0,
            'hdg': 0.0
        }


        self.last_sent_time = 0
        self.last_sent_time2 = 0
        self.send_interval = 1.0  # seconds between sends

    def listener_callback_gps(self, msg):
        self.latest_coordinates['lat'] = msg.data[0]
        self.latest_coordinates['lng'] = msg.data[1]
        threading.Thread(target=self.send_to_server).start()

    def listener_callback_compass(self, msg):
        self.latest_coordinates['hdg'] = msg.data
        threading.Thread(target=self.send_to_server).start()


    def listener_callback2(self, msg):
        try:
            label = msg.data.lower()
            timestamp = time.time()

            if any(animal in label for animal in ['dog', 'cat', 'horse']):
                warning_code = 1.0
            elif 'sea turtle' in label:
                warning_code = 2.0
            else:
                self.get_logger().info(f"Ignored detection: {label}")
                return

            payload = Float64MultiArray()
            payload.data = [timestamp, warning_code]

            now = time.time()
            if now - self.last_sent_time2 < self.send_interval:
                return  # Skip this send
            self.last_sent_time2 = now

            r = requests.post(
                f"{SERVER_URL}/warning",
                json={'data': list(payload.data)},
                headers={'Content-Type': 'application/json'},
                timeout=3
            )

            self.get_logger().info(f"Sent warning [{payload.data}] to server, status: {r.status_code}, timestamp: {timestamp}")

        except Exception as e:
            self.get_logger().error(f"Warning POST failed: {e}")


    def send_to_server(self):
        now = time.time()
        if now - self.last_sent_time < self.send_interval:
            return  # Skip this send
        self.last_sent_time = now

        try:
            # Use .get() with default fallback
            data_to_send = {
                'lat': self.latest_coordinates.get('lat', 0.0),
                'lng': self.latest_coordinates.get('lng', 0.0),
                'hdg': self.latest_coordinates.get('hdg', 0.0)
            }
            r = requests.post(f"{SERVER_URL}/communication", json=data_to_send, timeout=3)
            self.get_logger().info(f"Coordinates sent, status: {r.status_code}")
        except Exception as e:
            self.get_logger().error(f"POST error: {e}")




    def publish_markers(self, marker_list):
        msg = Float64MultiArray()
        if len(marker_list) == 1 and marker_list[0] in [0, 1]:
            msg.data = [float(marker_list[0])]
        else:
            flat = []
            for m in marker_list:
                flat += [float(m.get('id', 0)), float(m.get('lat', 0)), float(m.get('lng', 0))]
            msg.data = flat
        self.publisher_markers.publish(msg)
        self.get_logger().info(f"Published marker list to ROS: {msg.data}")

# === Socket.IO Client ===
sio = socketio.Client()

@sio.event
def connect():
    print("Connected to server")
    sio.emit('register_jetson', {'role': 'jetson'})  # optional handshake

@sio.event
def disconnect():
    print("Disconnected from server")

@sio.on('forward_marker')
def handle_marker(data):
    print(f"Received marker: {data}")
    if 'ros_node' in globals():
        ros_node.publish_markers(data.get('markers', []))
    else:
        print("ROS not ready")

# === Entrypoint ===
def main():
    global ros_node

    # Start ROS 2 node in thread
    rclpy.init()
    ros_node = CoordinateReceiver()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Connect to socketio server
    while True:
        try:
            sio.connect(SERVER_URL, transports=['websocket'])
            sio.wait()
        except Exception as e:
            print(f"Reconnect error: {e}")
            time.sleep(5)

if __name__ == '__main__':
    main()
