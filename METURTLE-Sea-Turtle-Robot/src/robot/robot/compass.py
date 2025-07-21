import smbus2
import time
import math

# I2C bus number (usually 1 on Jetson)
bus = smbus2.SMBus(1)

QMC5883L_ADDR = 0x0D
QMC5883L_CONTROL = 0x09
QMC5883L_RESET = 0x0A
QMC5883L_DATA = 0x00

def init_qmc5883l():
    bus.write_byte_data(QMC5883L_ADDR, QMC5883L_RESET, 0x80)
    time.sleep(0.1)
    bus.write_byte_data(QMC5883L_ADDR, QMC5883L_CONTROL, 0b00011101)

def read_raw_data():
    data = bus.read_i2c_block_data(QMC5883L_ADDR, QMC5883L_DATA, 6)
    x = data[1] << 8 | data[0]
    y = data[3] << 8 | data[2]
    z = data[5] << 8 | data[4]
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z
    return x, y, z

def calculate_heading(x, y):
    heading_rad = math.atan2(y, x)
    if heading_rad < 0:
        heading_rad += 2 * math.pi
    heading_deg = math.degrees(heading_rad)
    return heading_deg

if __name__ == "__main__":
    init_qmc5883l()
    try:
        while True:
            x, y, z = read_raw_data()
            heading = calculate_heading(x, y)
            heading = heading -57
            print(f"X: {x}, Y: {y}, Z: {z}, Heading: {heading:.2f}°")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Stopping...")
