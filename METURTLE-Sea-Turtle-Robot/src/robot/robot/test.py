import smbus2
import time

bus = smbus2.SMBus(1)
ADDR = 0x0D
DATA_REG = 0x00
RESET_REG = 0x0A
CTRL_REG = 0x09

# Initialize sensor
bus.write_byte_data(ADDR, RESET_REG, 0x80)
time.sleep(0.1)
bus.write_byte_data(ADDR, CTRL_REG, 0b00011101)

print("Logging raw magnetometer data. Move sensor in figure 8...")
min_x = min_y = min_z = 32767
max_x = max_y = max_z = -32768

try:
    while True:
        data = bus.read_i2c_block_data(ADDR, DATA_REG, 6)
        x = data[1] << 8 | data[0]
        y = data[3] << 8 | data[2]
        z = data[5] << 8 | data[4]

        x = x - 65536 if x > 32767 else x
        y = y - 65536 if y > 32767 else y
        z = z - 65536 if z > 32767 else z

        min_x, max_x = min(min_x, x), max(max_x, x)
        min_y, max_y = min(min_y, y), max(max_y, y)
        min_z, max_z = min(min_z, z), max(max_z, z)

        print(f"X:{x}, Y:{y}, Z:{z}")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nCalibration complete.")
    print(f"X range: min={min_x}, max={max_x}")
    print(f"Y range: min={min_y}, max={max_y}")
    print(f"Z range: min={min_z}, max={max_z}")
