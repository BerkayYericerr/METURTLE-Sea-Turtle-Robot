import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import smbus2
import time
import math
import threading

class CompassPublisher(Node):
    def __init__(self):
        super().__init__('compass_publisher')

        # === Compass Setup
        self.bus = smbus2.SMBus(1)
        self.QMC5883L_ADDR = 0x0D
        self.QMC5883L_CONTROL = 0x09
        self.QMC5883L_RESET = 0x0A
        self.QMC5883L_DATA = 0x00
        self.init_qmc5883l()

        # === ROS Publisher
        self.publisher = self.create_publisher(Float64, 'compass', 10)

        # === Start compass reading loop
        threading.Thread(target=self.compass_loop, daemon=True).start()

    def init_qmc5883l(self):
        self.bus.write_byte_data(self.QMC5883L_ADDR, self.QMC5883L_RESET, 0x80)
        time.sleep(0.1)
        self.bus.write_byte_data(self.QMC5883L_ADDR, self.QMC5883L_CONTROL, 0b00011101)
        self.get_logger().info("QMC5883L compass initialized.")

    def compass_loop(self):
        while rclpy.ok():
            heading = self.read_compass_heading()
            if heading is not None:
                msg = Float64()
                msg.data = heading
                self.publisher.publish(msg)
                self.get_logger().info(f"Heading: {heading:.2f}°")
            time.sleep(0.1)  # 50Hz

    def read_compass_heading(self):
            try:
                data = self.bus.read_i2c_block_data(self.QMC5883L_ADDR, self.QMC5883L_DATA, 6)
                x = data[1] << 8 | data[0]
                y = data[3] << 8 | data[2]
                z = data[5] << 8 | data[4]
                x = x - 65536 if x > 32767 else x
                y = y - 65536 if y > 32767 else y
                z = z - 65536 if z > 32767 else z

                # Calibration
                offset_x, offset_y, offset_z = 1186, 1184, -1378
                scale_x, scale_y, scale_z = 1186, 1184, 1378
                x = (x - offset_x) / scale_x
                y = (y - offset_y) / scale_y

                heading_rad = math.atan2(x, y)
                heading_deg = math.degrees(heading_rad)
                if heading_deg < 0:
                    heading_deg += 360
                heading_deg = (360 - heading_deg + 4.6 +30) % 360
                return heading_deg
            except Exception as e:
                self.get_logger().warn(f"Compass read failed: {e}")
                return None

def main(args=None):
    rclpy.init(args=args)
    node = CompassPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
