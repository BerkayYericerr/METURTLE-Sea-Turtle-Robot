import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from chassis_control.msg import SetVelocity
import math
import copy

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
            '/ml/detected',
            self.detected_callbask,
            10)

        # Publisher for chassis control
        self.set_velocity_publisher = self.create_publisher(SetVelocity, '/chassis_control/set_velocity', 10)
        self.publisher_testgps = self.create_publisher(Float64MultiArray, 'coordinatessimulated', 10)
        
        # Target position and other autopilot variables
        self.target_pos = (100, 100, 100)  # Define target coordinates as an example
        self.current_pos = (0, 0)
        self.current_heading = 0.0
        self.speed = 150.0  # Default speed
        self.target_reached = False
        self.stop = 0

        self.patrol_list = []
        self.all_targets = []  # Tüm hedefleri saklayacak bir liste

        # Set loop rate
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop

    def detected_callbask(self, msg):
        print("detected!" + msg.data)

    def coordinates_callback(self, msg):

        lat = msg.data[0]
        lng = msg.data[1]

        self.current_pos = (lat, lng)

        self.current_heading = msg.data[2]
        self.get_logger().info(f"Received GPS: {self.current_pos} Heading: {self.current_heading}")

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
        if self.stop == 1:
            self.set_velocity_publisher.publish(SetVelocity(velocity=0.0, direction=0.0, angular=0.0))
            self.get_logger().info("Robot is stopped. Awaiting further commands.")
            return

        if not self.patrol_list:
            if not self.all_targets:
                self.get_logger().info("No targets. Autopilot on standby.")
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
        if abs(heading_error) > 1:  # Allow small tolerance for heading error
            yaw_rate = -0.1 if heading_error > 0 else 0.1

            # Simulate yaw adjustment
            self.current_heading = (self.current_heading + 2 if heading_error > 0 else self.current_heading - 2) % 360

            # Simulate position update during yaw adjustment
            step_size = 0.0000  # Adjust step size as needed
            delta_x = step_size * math.cos(math.radians(self.current_heading))
            delta_y = step_size * math.sin(math.radians(self.current_heading))
            self.current_pos = (self.current_pos[0] + delta_x, self.current_pos[1] + delta_y)

            # Publish updated GPS and heading
            msg = Float64MultiArray()
            msg.data = [self.current_pos[0], self.current_pos[1], self.current_heading] 
            self.publisher_testgps.publish(msg)

            # Publish velocity message
            self.set_velocity_publisher.publish(SetVelocity(velocity=0.0, direction=float(self.current_heading), angular=float(yaw_rate)))
            self.get_logger().info(f"Adjusting yaw rate: {yaw_rate}")
        else:
            # Move forward when heading is aligned
            step_size = 0.0001  # Forward movement step size
            delta_x = step_size * math.cos(math.radians(self.current_heading))
            delta_y = step_size * math.sin(math.radians(self.current_heading))
            self.current_pos = (self.current_pos[0] + delta_x, self.current_pos[1] + delta_y)

            # Publish updated GPS and heading
            msg = Float64MultiArray()
            msg.data = [self.current_pos[0], self.current_pos[1], self.current_heading]
            self.publisher_testgps.publish(msg)

            # Publish velocity message
            self.set_velocity_publisher.publish(SetVelocity(velocity=float(self.speed), direction=float(self.current_heading), angular=0.0))
            self.get_logger().info(f"Moving towards target with speed {self.speed} and heading {self.current_heading}")

        # Check if target reached
        if self.distance_to_target(self.current_pos, (target_lat, target_lng)) < 0.00006225332:  # Proximity tolerance
            self.get_logger().info(f"Target {self.current_target[0]} reached. Removing from list.")
            self.patrol_list.remove(self.current_target)  # Remove target from the list
            self.target_reached = True

            # Stop movement
            self.set_velocity_publisher.publish(SetVelocity(velocity=0.0, direction=0.0, angular=0.0))

            


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
        autopilot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
