import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String, Float64
import math
import copy
import Jetson.GPIO as GPIO
import time

# GPIO pins for ultrasonic sensor
TRIG = 21
ECHO = 27

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, False)

time.sleep(2)

class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot_node')
        
        # Subscriber to GPS coordinates and heading
        self.subscription1 = self.create_subscription(
            Float64MultiArray,
            'coordinates',
            self.coordinates_callback,
            10)
        
        self.subscription2 = self.create_subscription(
            Float64MultiArray,
            'marker_location',
            self.marker_location_callback,
            10)
        
        self.subscription3 = self.create_subscription(
            String,
            'detected_objects',
            self.detected_callback,
            10)

        self.subscription4 = self.create_subscription(
            Int32,
            'distance',
            self.check_obstacle_distance,
            10)

        self.subscription_heading = self.create_subscription(
            Float64,
            'compass',
            self.heading_callback,
            10)

        # Publisher for chassis control
        self.set_velocity_publisher = self.create_publisher(Int32, 'movement', 10)
        self.set_detected_publisher = self.create_publisher(Int32, 'detected', 10)
        
        # Target position and other autopilot variables
        self.target_pos = (100, 100, 100)  # Define target coordinates as an example
        self.current_pos = (0, 0)
        self.current_heading = 0.0
        self.speed = 150.0  # Default speed
        self.target_reached = False
        self.last_detection_time = time.time()
        self.object_was_present = False
        self.forward_counter = 0


        self.stop = 0

        self.patrol_list = []
        self.all_targets = []  # Tüm hedefleri saklayacak bir liste

        self.obstacle_detected = False
        self.avoidance_mode = False

        self.turtle_detected = False
        self.turtle_detected_time = None


        # Set loop rate
        self.timer = self.create_timer(0.05, self.control_loop)  # 10 Hz control loop

    def heading_callback(self, msg):
        self.current_heading = msg.data
        self.get_logger().info(f"Compass heading updated: {self.current_heading:.2f}°")


    def detected_callback(self, msg):
        print("detected!" + msg.data)
        self.last_detection_time = time.time()
        self.object_was_present = True  # Object is currently detected

        detected_thing = msg.data.lower()

        if "cat" in detected_thing or "dog" in detected_thing or "horse" in detected_thing:
            msg2 = Int32()
            msg2.data = 5
            self.set_velocity_publisher.publish(msg2)

            msg2.data = 1
            self.set_detected_publisher.publish(msg2)

        elif "turtle" in detected_thing:
            print("Turtle detected! Initiating 5s right-turn maneuver.")
            self.turtle_detected = True
            self.turtle_detected_time = time.time()

            msg2 = Int32()
            msg2.data = 2  # Turn right
            self.set_velocity_publisher.publish(msg2)

            msg2.data = 2
            self.set_detected_publisher.publish(msg2)


    def check_obstacle_distance(self, msg):
        self.distance = msg.data

        if self.distance <30:
            self.obstacle_detected = True

    def coordinates_callback(self, msg):
        data = msg.data

        if len(data) < 2:
            self.get_logger().warn(f"Insufficient GPS data received: {data}")
            return

        self.current_pos = (data[0], data[1])
        self.get_logger().info(f"Received GPS: {self.current_pos}")




    def marker_location_callback(self, msg):
        data = msg.data  # The flattened list from Float64MultiArray
        if len(data) % 3 == 0:
            
            # Clear previous markers (if necessary)
            self.all_targets = []
            self.patrol_list = []

            # Process markers in chunks of three
            for i in range(0, len(data), 3):
                marker_id = data[i]
                lat = data[i + 1]
                lng = data[i + 2]

                # Store the target position as a tuple
                target_pos = (lat, lng)
                self.all_targets.append((marker_id, target_pos))
                self.patrol_list.append((marker_id, target_pos))

                self.get_logger().info(f"Received marker ID: {marker_id}, Position: {target_pos}")

            # Log the entire updated patrol list (optional)
            self.get_logger().info(f"Updated patrol list: {self.patrol_list}")

        elif len(data) == 1:
            if data[0] == 0:
                self.get_logger().error("Stop!")

                self.stop = 1
                
            elif data[0] == 1:
                self.get_logger().error("Start!")
                self.stop = 0
                self.target_reached = False


    def get_closest_target(self):
        if not self.patrol_list:
            self.get_logger().info("No remaining targets in the list.")
            return None
        
        # En yakın hedefi seç
        closest_target = min(self.patrol_list, key=lambda t: self.distance_to_target(self.current_pos, t[1]))
        self.get_logger().info(f"Closest target selected: {closest_target}")
        return closest_target

    def control_loop(self):

        # Check for disappearance of object
        time_since_last_detection = time.time() - self.last_detection_time
        if self.object_was_present and time_since_last_detection > 1.0:  # 1 second timeout
            msg = Int32()
            msg.data = 6
            self.set_velocity_publisher.publish(msg)
            self.get_logger().info("Object disappeared! Publishing 6.")
            self.object_was_present = False  # Reset flag

        if self.stop == 1:
            msg = Int32()
            msg.data = 8
            self.set_velocity_publisher.publish(msg)
            self.get_logger().info("Robot is stopped. Awaiting further commands.")
            return

        # === Handle turtle detection turn ===
        if self.turtle_detected:
            elapsed = time.time() - self.turtle_detected_time
            if elapsed < 5:
                msg = Int32()
                msg.data = 2  # Turn right
                self.set_velocity_publisher.publish(msg)
                self.get_logger().info("Turning right due to turtle detection...")
                return  # Exit early, skip rest of control logic
            else:
                self.get_logger().info("Finished turtle turning. Resuming normal operation.")
                self.turtle_detected = False  # Reset
        
        if self.obstacle_detected:
            self.get_logger().warn("Obstacle detected! Avoiding")
            self.avoidance_mode = True

        # === AVOIDANCE MODE ===
        if self.avoidance_mode:
            if self.distance < 30:
                # Still seeing obstacle → keep turning
                msg = Int32()
                msg.data = 2  # Turn right
                self.set_velocity_publisher.publish(msg)
                self.get_logger().info("Turning to avoid obstacle...")
                self.forward_counter = 0  # Reset counter if we detect obstacle again
                return

            else:
                # Obstacle cleared → move forward while monitoring
                self.get_logger().info("Clear path ahead. Moving forward during avoidance...")

                msg = Int32()
                msg.data = 3  # Move forward
                self.set_velocity_publisher.publish(msg)

                self.forward_counter += 1
                self.get_logger().info(f"Forward clear steps: {self.forward_counter}")

                # Require N clear cycles (e.g., 5 = 0.5 sec at 10 Hz) before resuming target pursuit
                if self.forward_counter >= 100:
                    self.get_logger().info("Avoidance complete. Resuming target navigation.")
                    self.obstacle_detected = False
                    self.avoidance_mode = False
                    self.forward_counter = 0
                return

        if not self.patrol_list:
            if not self.all_targets:
                self.get_logger().info("No targets. Autopilot on standby.")

                # Stop movement
                msg = Int32()
                msg.data = 8
                self.set_velocity_publisher.publish(msg)

                return
            else:
                self.patrol_list = copy.deepcopy(self.all_targets)

        self.current_target = self.get_closest_target()

        self.get_logger().info(f"Closest target selected: {self.current_target}")

        # If the target is reached or no target is currently selected, choose a new one
        if self.target_reached or not hasattr(self, 'current_target'):
            self.get_logger().info(f"Target reached: {self.target_reached}, Current target: {getattr(self, 'current_target', None)}")
            self.current_target = self.get_closest_target()
            
            # If the target list is empty
            if self.current_target is None:
                self.get_logger().info("All targets reached. Autopilot on standby.")
                return
            
            # When a new target is selected
            self.target_reached = False
            self.get_logger().info(f"New target selected: {self.current_target}")

        # Calculate desired heading to the target
        target_lat, target_lng = self.current_target[1]
        desired_heading = math.atan2(target_lng - self.current_pos[1], target_lat - self.current_pos[0]) * 180 / math.pi
        print(f"Target pos: {target_lat}, {target_lng}")
        print(desired_heading)
        
        # Calculate heading error
        heading_error = (desired_heading - self.current_heading + 360) % 360
        if heading_error > 180:
            heading_error -= 360  # Normalize error to range [-180, 180]
        
        # Adjust direction to minimize heading error
        if abs(heading_error) > 10:  # Allow small tolerance for heading error
            yaw_rate = 2 if heading_error > 0 else 1

            msg = Int32()
            msg.data = yaw_rate
            # Publish velocity message
            self.set_velocity_publisher.publish(msg)
            self.get_logger().info(f"Adjusting yaw rate: {yaw_rate}")
        else:

            # Publish updated GPS and heading
            msg = Int32()
            msg.data = 3

            # Publish velocity message
            self.set_velocity_publisher.publish(msg)

        # Check if target reached
        if self.distance_to_target(self.current_pos, (target_lat, target_lng)) < 0.00006225332:  # Proximity tolerance
            self.get_logger().info(f"Target {self.current_target[0]} reached. Removing from list.")
            self.patrol_list.remove(self.current_target)  # Remove target from the list
            self.target_reached = True

            # Stop movement
            msg = Int32()
            msg.data = 8
            self.set_velocity_publisher.publish(msg)
        

    def distance_to_target(self, current_pos, target_pos):
        distance = math.sqrt((target_pos[0] - current_pos[0]) ** 2 + (target_pos[1] - current_pos[1]) ** 2)
        print(f"distance: {distance}\n")
        return distance


def main(args=None):
    rclpy.init(args=args)
    autopilot_node = AutopilotNode()
    
    try:
        rclpy.spin(autopilot_node)
    except (KeyboardInterrupt):
        pass
    finally:
        GPIO.cleanup()
        autopilot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()