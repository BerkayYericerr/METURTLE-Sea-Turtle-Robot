import os
import time
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import requests
import threading

app = Flask(__name__)
CORS(app)  # Enable CORS
socketio = SocketIO(app, cors_allowed_origins="*")

last_ros_update = None
ros_timeout = 10

# Serve the index.html
@app.route('/')
def index():
    return render_template('index.html')

# Event for when a client connects to the server
@socketio.on('connect')
def handle_connection():
    print("WebSocket client connected")
    emit('connection_status', {'status': 'Connected to WebSocket server'})

# Event for when a client disconnects from the server
@socketio.on('disconnect')
def handle_disconnect():
    print('WebSocket client disconnected')
    emit('connection_status', {'status': 'Disconnected from WebSocket server'})

# Endpoint to receive coordinates
@app.route('/communication', methods=['POST'])
def receive_coordinates():
    global last_ros_update
    data = request.get_json()
    lat = data.get('lat')
    lng = data.get('lng')
    hdg = data.get('hdg')

    # Update the timestamp when coordinates are received
    last_ros_update = time.time()
    
    # Notify WebSocket clients that the ROS client is active and sending data
    socketio.emit('ros_client_status', {'status': 'Robot client active'})
    print(f"Received coordinates from ROS: Lat: {lat}, Lng: {lng}, Hdg: {hdg}")

    # Broadcast coordinates to all connected clients
    socketio.emit('robot_coordinates', {'lat': lat, 'lng': lng, 'hdg': hdg})
    return jsonify({'status': 'success'})

latest_marker_data = {}

# Route to receive marked positions from the HTML
@app.route('/receive_marker', methods=['POST'])
def receive_marker():
    data = request.get_json()
    id = data.get('id')
    lat = data.get('lat')
    lng = data.get('lng')

    print(f"Received coordinates from html: Lat: {lat}, Lng: {lng} with ID: {id}")

    forward_url = 'http://localhost:5001/forward_marker'  
    forward_data = {'id': id, 'lat': lat, 'lng': lng}

    try:
        response = requests.post(forward_url, json=forward_data)
        return jsonify({'status': 'success', 'response_from_send': response.json()})
    except requests.exceptions.RequestException as e:
        return jsonify({'status': 'error', 'message': str(e)})
    
    
# Periodic check for ROS connection timeout
def check_ros_connection():
    global last_ros_update
    while True:

        time.sleep(ros_timeout)
        if last_ros_update is None:
            continue

        time_since_last_update = time.time() - last_ros_update
        if time_since_last_update > ros_timeout:
            
            # Emit a "not active" status to clients if ROS node is inactive
            socketio.emit('ros_client_status', {'status': 'ROS not active'})
            print("ROS client inactive due to timeout.")
threading.Thread(target=check_ros_connection, daemon=True).start()

# Use the port provided by Heroku
if __name__ == '__main__':
    port = int(os.environ.get("PORT", 5000))  # Get the port from environment
    socketio.run(app, host="0.0.0.0", port=port, debug=True)

