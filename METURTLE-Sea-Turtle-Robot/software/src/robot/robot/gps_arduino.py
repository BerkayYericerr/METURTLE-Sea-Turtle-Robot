import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import serial
import pynmea2
import time
import threading

class FastGPSPublisher(Node):
    def __init__(self):
        super().__init__('fast_gps_publisher')

        # Init serial
        try:
            self.ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
            self.get_logger().info("🔌 Connected to GPS on /dev/ttyTHS1")
            #self.configure_gps_update_rate(5)
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial connection failed: {e}")
            raise

        # ROS publisher
        self.publisher = self.create_publisher(Float64MultiArray, 'coordinates', 10)

        # GPS thread
        self.last_lat = None
        self.last_lon = None
        threading.Thread(target=self.gps_loop, daemon=True).start()

    def configure_gps_update_rate(self, hz=5):
        try:
            interval = int(1000 / hz)
            msg = b'\xB5\x62\x06\x08\x06\x00' + interval.to_bytes(2, 'little') + b'\x01\x00\x01\x00'
            ck_a = 0
            ck_b = 0
            for b in msg[2:]:
                ck_a = (ck_a + b) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            msg += bytes([ck_a, ck_b])
            self.ser.write(msg)
            self.get_logger().info(f"✔️ GPS update rate set to {hz} Hz")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to set GPS update rate: {e}")

    def gps_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith('$GPRMC') or line.startswith('$GPGGA'):
                    msg = pynmea2.parse(line)

                    # Ensure we have a valid fix
                    if hasattr(msg, 'status') and msg.status != 'A':
                        continue  # Skip if no valid GPS fix

                    if msg.latitude and msg.longitude:
                        lat = msg.latitude
                        lon = msg.longitude

                        if isinstance(lat, str):  # Sometimes still str
                            lat = float(lat)
                            lon = float(lon)

                        if lat != self.last_lat or lon != self.last_lon:
                            self.last_lat = lat
                            self.last_lon = lon
                            self.publish_coordinates(lat, lon)
            except Exception as e:
                self.get_logger().warn(f"⚠️ GPS read failed: {e}")


    def publish_coordinates(self, lat, lon):
        msg = Float64MultiArray()
        msg.data = [lat, lon]  # heading = 0.0 placeholder
        self.publisher.publish(msg)
        self.get_logger().info(f"📡 Published: Lat {lat:.8f}, Lon {lon:.8f}")

def main(args=None):
    rclpy.init(args=args)
    node = FastGPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
