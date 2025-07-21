from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit

app = Flask(__name__)
CORS(app)  # Enable CORS
socketio = SocketIO(app, cors_allowed_origins="*")  # Allow all origins

# Serve the webpage
@app.route('/')
def index():
    return render_template('index.html')

# Endpoint to receive coordinates
@app.route('/communicaiton', methods=['POST'])
def receive_coordinates():
    data = request.get_json()
    lat = data.get('lat')
    lng = data.get('lng')

    # Broadcast coordinates to all connected clients
    socketio.emit('new_coordinates', {'lat': lat, 'lng': lng})
    return jsonify({'status': 'success'})

# Run the Flask application
if __name__ == '__main__':
    socketio.run(app, debug=True)
