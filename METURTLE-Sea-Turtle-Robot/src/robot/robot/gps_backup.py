import serial
import pynmea2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import smbus2
import time
import math
import threading

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')

        # === Shared state
        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0
        self.lock = threading.Lock()

        # === Init hardware
        try:
            self.ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
            self.get_logger().info("Serial connection to GPS established via /dev/ttyTHS1")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to GPS: {e}")
            raise

        self.bus = smbus2.SMBus(1)
        self.QMC5883L_ADDR = 0x0D
        self.QMC5883L_CONTROL = 0x09
        self.QMC5883L_RESET = 0x0A
        self.QMC5883L_DATA = 0x00
        self.init_qmc5883l()

        # === ROS Publisher
        self.publisher = self.create_publisher(Float64MultiArray, 'coordinates', 10)
        self.timer = self.create_timer(0.1, self.publish_latest_data)  # 10Hz publish

        # === Start threads
        threading.Thread(target=self.gps_loop, daemon=True).start()
        threading.Thread(target=self.compass_loop, daemon=True).start()

    def init_qmc5883l(self):
        self.bus.write_byte_data(self.QMC5883L_ADDR, self.QMC5883L_RESET, 0x80)
        time.sleep(0.1)
        self.bus.write_byte_data(self.QMC5883L_ADDR, self.QMC5883L_CONTROL, 0b00011101)
        self.get_logger().info("QMC5883L compass initialized.")

    def compass_loop(self):
        while True:
            heading = self.read_compass_heading()
            if heading is not None:
                with self.lock:
                    self.heading = heading
            time.sleep(0.02)  # 50Hz

    def gps_loop(self):
        while True:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore')
                if line.startswith('$GPRMC') or line.startswith('$GPGGA'):
                    msg = pynmea2.parse(line)
                    lat = float(msg.latitude)
                    lon = float(msg.longitude)
                    with self.lock:
                        self.latitude = lat
                        self.longitude = lon
                        self.get_logger().info(f"Published: Lat {self.latitude:.8f}, Lon {self.longitude:.8f}")
            except Exception as e:
                self.get_logger().warn(f"GPS read failed: {e}")
            time.sleep(0.2)  # ~5Hz (adjust to match your GPS speed)

    def read_compass_heading(self):
        try:
            data = self.bus.read_i2c_block_data(self.QMC5883L_ADDR, self.QMC5883L_DATA, 6)
            x = data[1] << 8 | data[0]
            y = data[3] << 8 | data[2]
            z = data[5] << 8 | data[4]
            x = x - 65536 if x > 32767 else x
            y = y - 65536 if y > 32767 else y
            z = z - 65536 if z > 32767 else z

            # Calibration offsets and scale
            offset_x, offset_y, offset_z = 1338, 817.5, -1329
            scale_x, scale_y, scale_z = 1338, 817.5, 1329

            # Normalize X and Y
            x = (x - offset_x) / scale_x
            y = (y - offset_y) / scale_y

            # Heading calculation
            heading_rad = math.atan2(x, y)
            heading_deg = math.degrees(heading_rad)
            if heading_deg < 0:
                heading_deg += 360

            # Apply declination correction (adjust 4.6° if needed)
            heading_deg = (360 - heading_deg + 4.6) % 360
            return heading_deg

        except Exception as e:
            self.get_logger().warn(f"Compass read failed: {e}")
            return None


    def publish_latest_data(self):
        with self.lock:
            lat = self.latitude
            lon = self.longitude
            hdg = self.heading

        msg = Float64MultiArray()
        msg.data = [lat, lon, hdg]
        self.publisher.publish(msg)
        #self.get_logger().info(f"Published: Lat {lat:.6f}, Lon {lon:.6f}, Heading {hdg:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()