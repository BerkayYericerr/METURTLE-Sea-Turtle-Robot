import serial
import pynmea2

ser = serial.Serial('/dev/ttyUSB4', 9600, timeout=1)

while True:
    try:
        line = ser.readline().decode('utf-8', errors='ignore')
        if line.startswith('$GPGGA'):
            msg = pynmea2.parse(line)
            print(f"Latitude: {msg.latitude}, Longitude: {msg.longitude}")
    except pynmea2.ParseError:
        continue