import requests

# The URL of the Flask server
url = 'http://localhost:5000/communicaiton'

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
