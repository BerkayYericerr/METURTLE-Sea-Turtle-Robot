import requests
from flask import Flask, request, jsonify

# Define the Flask app
app = Flask(__name__)
site_url = 'http://localhost:5000/'
# Function to send coordinates to the Flask server running on port 5000
#Robot2Web
def send_coordinates():
    # The URL of the Flask server (app.py)
    url = site_url + 'communication'

    # Coordinates to send
    coordinates = {
        'lat': 39.92077,  # Latitude
        'lng': 32.85411   # Longitude
    }

    # Send POST request with coordinates
    response = requests.post(url, json=coordinates)

    # Check if the request was successful
    if response.status_code == 200:
        print('Coordinates sent successfully.')
    else:
        print('Failed to send coordinates.')

# Route to receive forwarded coordinates from app.py
#Web2Robot
@app.route('/forward_marker', methods=['POST'])

def forward_marker():
    data = request.get_json()
    lat = data.get('lat')
    lng = data.get('lng')

    # Print the coordinates
    print(f"Coordinates received from app.py: Lat: {lat}, Lng: {lng}")

    # Send back a response to app.py
    return jsonify({'status': 'Coordinates received', 'lat': lat, 'lng': lng})

if __name__ == '__main__':
    # First, send the coordinates to app.py
    send_coordinates()

    # Then, run this Flask server on port 5001 to receive forwarded coordinates
    app.run(port=5001, debug=True)
