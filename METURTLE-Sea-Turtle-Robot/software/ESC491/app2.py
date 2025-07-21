import os
import time
import sqlite3
from flask import Flask, render_template, request, jsonify, redirect, url_for
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import bcrypt
import threading
import requests
from geopy.distance import geodesic
import geopandas as gpd
from shapely.geometry import Point

app = Flask(__name__)

land = gpd.read_file("data/ne_110m_admin_0_countries.shp")
#valid_land = land[land["ADMIN"].isin(["Turkey", "Cyprus", "Northern Cyprus"])]
valid_land = land[land["ADMIN"].isin(["Northern Cyprus"])]

robot_lat = None
robot_lng = None


# Create database if it doesn't exist
if not os.path.exists('database.db'):
    conn = sqlite3.connect('database.db')
    cursor = conn.cursor()
    cursor.execute("""
        CREATE TABLE Users (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            surname TEXT NOT NULL,
            email TEXT NOT NULL UNIQUE,
            password TEXT NOT NULL
        )
    """)
    conn.commit()
    conn.close()

CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

last_ros_update = None
ros_timeout = 10

# Marker storage
all_markers = []
marked_positions = []

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/login', methods=['GET'])
def show_login():
    return render_template('login.html')

@app.route('/login', methods=['POST'])
def login():
    email = request.form.get('email')
    password = request.form.get('password')

    try:
        conn = sqlite3.connect('database.db')
        cursor = conn.cursor()
        cursor.execute("SELECT password FROM Users WHERE email = ?", (email,))
        result = cursor.fetchone()
        conn.close()

        if result and bcrypt.checkpw(password.encode('utf-8'), result[0].encode('utf-8')):
            return redirect(url_for('index'))
        else:
            return render_template('login.html', error="Invalid email or password")
    except Exception as e:
        return jsonify({"error": f"Login error: {e}"}), 500

@app.route('/register', methods=['GET'])
def show_register():
    return render_template('register.html')

@app.route('/register', methods=['POST'])
def register():
    name = request.form.get('name')
    surname = request.form.get('surname')
    email = request.form.get('email')
    password = request.form.get('password')

    hashed_password = bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt())

    try:
        conn = sqlite3.connect('database.db')
        cursor = conn.cursor()
        cursor.execute("""
            INSERT INTO Users (name, surname, email, password)
            VALUES (?, ?, ?, ?)
        """, (name, surname, email, hashed_password))
        conn.commit()
        conn.close()
        return redirect(url_for('show_login'))
    except sqlite3.IntegrityError:
        return render_template('register.html', error="Email already registered")
    except Exception as e:
        return jsonify({"error": f"Register error: {e}"}), 500

@socketio.on('connect')
def handle_connection():
    print("WebSocket client connected")
    emit('connection_status', {'status': 'Connected to WebSocket server'})

@socketio.on('disconnect')
def handle_disconnect():
    print('WebSocket client disconnected')
    emit('connection_status', {'status': 'Disconnected from WebSocket server'})

@app.route('/communication', methods=['POST'])
def receive_coordinates():
    global last_ros_update, robot_lat, robot_lng

    data = request.get_json()
    lat = data.get('lat')
    lng = data.get('lng')
    hdg = data.get('hdg')

    robot_lat = lat
    robot_lng = lng

    last_ros_update = time.time()
    socketio.emit('ros_client_status', {'status': 'Robot client active'})
    socketio.emit('robot_coordinates', {'lat': lat, 'lng': lng, 'hdg': hdg})
    return jsonify({'status': 'success'})

@app.route('/receive_marker', methods=['POST'])
def receive_marker():
    data = request.get_json()
    id = data.get('id')
    lat = data.get('lat')
    lng = data.get('lng')

    print(f"Received coordinates from HTML: Lat: {lat}, Lng: {lng}, ID: {id}")
    marked_positions.append({'id': id, 'lat': lat, 'lng': lng})
    update_robot_route()

    return jsonify({'status': 'success'})

def update_robot_route():
    global marked_positions
    try:
        robot_current_pos = (current_lat, current_lng)  # Define these globally if used
    except:
        robot_current_pos = (39.0, 35.0)  # Fallback to center of Türkiye

    sorted_positions = sorted(
        marked_positions,
        key=lambda pos: geodesic(robot_current_pos, (pos['lat'], pos['lng'])).meters
    )

    for pos in sorted_positions:
        socketio.emit('next_target', {'lat': pos['lat'], 'lng': pos['lng']})

@app.route('/validate_marker', methods=['POST'])
def validate_marker():
    data = request.get_json()
    lat = data.get('lat')
    lng = data.get('lng')

    if lat is None or lng is None:
        return jsonify({'status': 'error', 'message': 'Missing coordinates'}), 400

    is_valid, error_message = is_valid_coordinate(lat, lng)
    if is_valid:
        return jsonify({'status': 'valid'}), 200
    else:
        return jsonify({'status': 'invalid', 'message': error_message}), 200

#Validate marker before starting robot
@app.route('/start_robot', methods=['POST'])
def start_robot():
    global all_markers
    data = request.get_json()
    print(f"Received start command. Incoming markers: {data.get('markers')}")
    forward_url = 'http://localhost:5001/forward_marker'

    if not data.get('markers'):
        return jsonify({'status': 'error', 'message': 'No markers available'}), 400

    all_markers = data.get('markers')

    try:
        if len(all_markers) == 1:
            if all_markers[0] == 0:
                response = requests.post(forward_url, json={'markers': [0]})
                if response.status_code != 200:
                    return jsonify({'status': 'error', 'message': 'Failed to send stop command to ROS'}), 500
                return jsonify({'status': 'success', 'message': 'Stop command sent to ROS'}), 200
            elif all_markers[0] == 1:
                response = requests.post(forward_url, json={'markers': [1]})
                if response.status_code != 200:
                    return jsonify({'status': 'error', 'message': 'Failed to send marker unchanged signal to ROS'}), 500
                return jsonify({'status': 'success', 'message': 'Start command sent to ROS'}), 200

        for marker in all_markers:
            lat = marker.get('lat')
            lng = marker.get('lng')
            if not is_valid_coordinate(lat, lng):
                return jsonify({'status': 'error', 'message': f'Invalid marker at [{lat}, {lng}]. Only Türkiye or TRNC land markers are allowed.'}), 400

        response = requests.post(forward_url, json={'markers': all_markers})
        if response.status_code != 200:
            return jsonify({'status': 'error', 'message': 'Failed to send markers to ROS'}), 500

        return jsonify({'status': 'success', 'message': 'Markers sent to ROS'}), 200

    except Exception as e:
        print(f"Error: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

def is_valid_coordinate(lat, lng):
    global robot_lat, robot_lng
    try:
        point = Point(lng, lat)

        # 1. Geometric land check using shapefile
        land_match = None
        for _, country in valid_land.iterrows():
            if country['geometry'].contains(point):
                land_match = country['ADMIN']
                break

        if not land_match:
            print(f"[GeoValidation] Rejected: Point is not on valid land.")
            return False, "Marker must be on valid land within Türkiye or TRNC."

        # 2. Distance check (robot must have sent its position first)
        if robot_lat is None or robot_lng is None:
            print("[GeoValidation] Rejected: Robot location not initialized.")
            return False, "Robot location not initialized yet."

        robot_pos = (robot_lat, robot_lng)
        marker_pos = (lat, lng)
        distance_m = geodesic(robot_pos, marker_pos).meters
        print(f"[GeoValidation] Distance to robot: {distance_m:.2f} meters")

        if distance_m > 500:
            return False, f"Marker is {int(distance_m)} meters away — must be within 500 meters of the robot."

        # 3. Reverse geocoding check using Nominatim
        url = f"https://nominatim.openstreetmap.org/reverse?format=json&lat={lat}&lon={lng}&zoom=10&addressdetails=1"
        headers = {
            "User-Agent": "METU-RTLE-WebApp/1.0"
        }
        response = requests.get(url, headers=headers, timeout=5)
        if response.status_code != 200:
            print(f"[GeoValidation] Nominatim error code: {response.status_code}")
            return False, f"Geocoding failed with status code {response.status_code}"

        data = response.json()
        address = data.get("address", {})
        country = address.get("country", "").lower()
        state = address.get("state", "").lower()
        display_name = data.get("display_name", "").lower()

        print(f"[GeoValidation DEBUG] lat: {lat}, lng: {lng}")
        print(f"Geometric land country: {land_match}")
        print(f"Geocoder country: {country}, State: {state}, Display: {display_name}")

        valid_keywords = [
            "turkey", "türkiye",
            "northern cyprus", "kuzey kıbrıs", "trnc",
            "κύπρος", "kıbrıs", "cyprus"
        ]

        if any(keyword in country for keyword in valid_keywords):
            return True, None
        if any(keyword in display_name for keyword in valid_keywords):
            return True, None
        if any(keyword in state for keyword in valid_keywords):
            return True, None

        print("[GeoValidation] Rejected: Location not in allowed region.")
        return False, "Marker is not within allowed regions (Türkiye/TRNC)."

    except Exception as e:
        print(f"[GeoValidation ERROR] {e}")
        return False, f"Validation failed: {str(e)}"



def check_ros_connection():
    global last_ros_update
    while True:
        time.sleep(ros_timeout)
        if last_ros_update is None:
            continue

        if time.time() - last_ros_update > ros_timeout:
            socketio.emit('ros_client_status', {'status': 'ROS not active'})
            print("ROS client inactive due to timeout.")

threading.Thread(target=check_ros_connection, daemon=True).start()

if __name__ == '__main__':
    port = int(os.environ.get("PORT", 5000))
    socketio.run(app, host="0.0.0.0", port=port, debug=True)
