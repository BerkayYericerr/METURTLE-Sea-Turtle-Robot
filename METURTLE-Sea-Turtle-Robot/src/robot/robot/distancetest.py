import Jetson.GPIO as GPIO
import time

# Define pins
TRIG_PIN = 15  # GPIO15 (physical pin 10)
ECHO_PIN = 13  # GPIO13 (physical pin 33)

def setup():
    GPIO.setmode(GPIO.BOARD)  # or GPIO.BCM if using GPIO numbering
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.output(TRIG_PIN, False)
    print("[DEBUG] GPIO Initialized")
    time.sleep(2)

def get_distance():
    # Send trigger pulse
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # 10µs
    GPIO.output(TRIG_PIN, False)

    # Wait for echo to start
    timeout = time.time() + 0.05
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            print("[ERROR] Timeout waiting for ECHO HIGH")
            return None

    # Wait for echo to end
    timeout = time.time() + 0.05
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            print("[ERROR] Timeout waiting for ECHO LOW")
            return None

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound: 34300 cm/s ÷ 2
    distance = round(distance, 2)

    return distance

def loop():
    try:
        while True:
            dist = get_distance()
            if dist is not None:
                print(f"Distance: {dist} cm")
            else:
                print("Distance measurement failed.")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] Measurement stopped by User")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    setup()
    loop()
