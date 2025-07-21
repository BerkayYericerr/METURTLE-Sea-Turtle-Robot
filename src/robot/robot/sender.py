import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
import threading
from flask import Flask, request, jsonify

# Define the target server URL (acting as a client)
#site_url = 'http://172.20.10.2:5000/'
site_url = 'https://e73f-185-43-231-91.ngrok-free.app/'
app = Flask(__name__)

# Coordinates class to handle coordinates received from ROS and sent to target server
class CoordinateReceiver(Node):
    def __init__(self):
        super().__init__('coordinate_receiver')
        
        self.subscription = self.create_subscription(
            Float64MultiArray,  # Message type with coordinates as floats
            'coordinates',      # Topic name
            self.listener_callback,
            10                  # Queue size
        )
        self.latest_coordinates = {'lat': None, 'lng': None}

        self.subscription2 = self.create_subscription(
            Int32,  # Message type with coordinates as floats
            'detected',      # Topic name
            self.listener_callback2,
            10                  # Queue size
        )

        self.publisher_15 = self.create_publisher(Float64MultiArray, 'marker_location', 10)


    def listener_callback(self, msg):
        self.latest_coordinates['lat'] = msg.data[0]
        self.latest_coordinates['lng'] = msg.data[1]
        self.latest_coordinates['hdg'] = msg.data[2]
        self.send_coordinates_to_server()

    def listener_callback2(self, msg):
        url = site_url + 'warning'

        message = msg.data
        try:
            response = requests.post(url, json={"data": message})
            if response.status_code == 200:
                self.get_logger().info('Warning sent successfully to the server.')
            else:
                self.get_logger().error(f'Failed to send Warning: {response.status_code}')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Error sending Warning to server: {e}')



    def send_coordinates_to_server(self):
        url = site_url + 'communication'
        try:
            response = requests.post(url, json=self.latest_coordinates)
            if response.status_code == 200:
                self.get_logger().info('Coordinates sent successfully to the server.')
            else:
                self.get_logger().error(f'Failed to send coordinates: {response.status_code}')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Error sending coordinates to server: {e}')

@app.route('/forward_marker', methods=['POST'])
def receive_marker_data():
    marker_data = request.json  # Expecting a JSON object with a 'markers' key
    if marker_data and 'markers' in marker_data:
        markers = marker_data['markers']
        print(f"Received marker data array from server: {markers}")
        
        try:
            # Check if the marker array contains only 0
            if len(markers) == 1:
                if markers[0] == 0:
                    # Handle the single 0 case
                    marker_msg = Float64MultiArray()
                    marker_msg.data = [0.0]  # Publish just 0.0 to indicate stop
                    coordinate_receiver.publisher_15.publish(marker_msg)
                    print("Published stop command (0) to ROS.")
                    return jsonify({"status": "success", "message": "Stop command published"}), 200
                elif markers[0] == 1:
                    # Handle the single 0 case
                    marker_msg = Float64MultiArray()
                    marker_msg.data = [1.0]  # Publish just 1.0 to indicate start
                    coordinate_receiver.publisher_15.publish(marker_msg)
                    print("Published stop command (0) to ROS.")
                    return jsonify({"status": "success", "message": "Stop command published"}), 200
        
            # Create and populate Float64MultiArray message for the entire list
            marker_msg = Float64MultiArray()
            
            # Flatten the marker data into a single list: [id1, lat1, lng1, id2, lat2, lng2, ...]
            for marker in markers:
                marker_msg.data.extend([
                    float(marker.get('id', 0.0)), 
                    float(marker.get('lat', 0.0)), 
                    float(marker.get('lng', 0.0))
                ])

            # Publish the entire list as one message to the ROS topic marker_locations
            coordinate_receiver.publisher_15.publish(marker_msg)
            print(f"Published marker list to ROS: {marker_msg.data}")

            return jsonify({"status": "All markers processed and published"}), 200
        except Exception as e:
            print(f"Error processing markers: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500

    return jsonify({"status": "No valid marker data received"}), 400


# Start the ROS 2 node in a separate thread
def start_ros_node():
    rclpy.init()
    global coordinate_receiver
    coordinate_receiver = CoordinateReceiver()
    try:
        rclpy.spin(coordinate_receiver)
    finally:
        coordinate_receiver.destroy_node()
        rclpy.shutdown()

# Start the Flask server
def start_flask_app():
    app.run(host='0.0.0.0', port=5001)

# Main function to start both Flask and ROS 2
if __name__ == '__main__':
    # Start the ROS 2 node in a separate thread
    ros_thread = threading.Thread(target=start_ros_node)
    ros_thread.start()

    # Start the Flask application on the main thread
    start_flask_app()